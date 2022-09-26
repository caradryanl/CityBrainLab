import json
import os.path as osp
from ossaudiodev import control_labels
from xml.etree.ElementTree import tostring
import cityflow
import cbengine
from torch_geometric.data import Data
import numpy as np
from math import atan2, pi
import torch
import os
from time import time


def _get_direction(delta_y, delta_x):
    tmp = atan2(delta_x, delta_y)
    return tmp if tmp >= 0 else (tmp + 2 * pi)


class Vehicle(object):
    def __init__(self, vehicle_id, vehicle, world):
        self.id = vehicle_id
        self.eng = world.eng
        self.info = vehicle
        self.in_system = None
        self.change = False
        self.original_route = vehicle["route"]
        self.last_distance = float(self.eng.get_vehicle_info(self.id)["distance"][0])
        self.route = self.original_route[1:]  # 记录所选路线
        try:
            self.route_choices = world.route["-".join([str(int(self.original_route[1])), str(int(self.original_route[-1]))])]
        except:
            if self.original_route[1] == self.original_route[-1]:
                self.route_choices = [[self.original_route[1]]]
        # TODO: change with the roadnet
        # 16x3
        # if self.id.split("_")[1] == '0' or self.id.split("_")[1] == '1':
        # porto
        '''
        if int(self.id.split("_")[1]) < 4:
            self.monitor = True
        else:
            self.monitor = False
        '''
        self.monitor = True
        self.reset()

        # add route_choices
        # if self.route not in self.route_choices:
        #     self.route_choices.append(self.route)

    def exit_system(self):
        self.in_system = False

    def step(self, action, interval):
        # change to a route
        current_route = self.eng.get_vehicle_info(self.id)["route"]
        current_route = [int(x) for x in current_route]
        action = [int(x) for x in action]
        
        '''
        if self.id <= 0 and self.eng.get_current_time() <= 400:
            print(self.id, self.info)
        '''

        if set(action) > set(current_route) or len(action) < 1 or self.in_system==False:
            self.info = self.eng.get_vehicle_info(self.id)
            return
        # old_route = self.eng.get_vehicle_route(self.id)

        self.eng.set_vehicle_route(self.id, action)
        self.info = self.eng.get_vehicle_info(self.id)

        # new_route = self.eng.get_vehicle_route(self.id) # *modify*
        # if old_route != new_route:
        #     print('change route succussfully!')
        #     print("old route: {}, new route: {}, action: {}"\
        #         .format(old_route, new_route, action))
        # else:
        #     print('No!')
        

    def reset(self):
        self.in_system = True


class Road(object):
    def __init__(self, road_id, road, world):
        self.id = road_id
        self.eng = world.eng
        self.info = road
        self.price = None
        self.last_vehicle = []
        self.history_vehicle = []
        self.lane = []

        self.length = road["length"]

        lane_vehicles = self.eng.get_lane_vehicles()
        for i in range(self.info["num_lanes"]):
            lane = int(int(self.id) * 100 + i)
            self.lane.append(lane)
            self.last_vehicle.extend(lane_vehicles[lane] if lane in lane_vehicles.keys() else [])
            self.history_vehicle.extend(lane_vehicles[lane] if lane in lane_vehicles.keys() else [])

        self.history_vehicle_count = len(self.history_vehicle)
        self.reset()

    def step(self, action, interval):
        # set price for a road
        self.price = action[0]
        pass

    def reset(self):
        self.price = 0
        pass

    def update_road_vehicle(self):
        lane_vehicles = self.eng.get_lane_vehicles()
        current_vehicles = []
        for i in range(len(self.info["num_lanes"])):
            lane = int(int(self.id) * 100 + i)
            current_vehicles.extend(lane_vehicles[lane])
        self.last_vehicle = current_vehicles




class Lane(object):
    def __init__(self, id, world):
        self.id = id
        self.eng = world.eng
        self.price = None

        lane_vehicles = self.eng.get_lane_vehicles()
        self.last_vehicle = lane_vehicles[id] if id in lane_vehicles.keys() else []
        self.history_vehicle = lane_vehicles[id] if id in lane_vehicles.keys() else []
        self.length = self.get_lane_length(world)

        self.history_vehicle_count = len(self.history_vehicle)
        self.reset()

    def step(self, action, interval):
        # set price for a road
        self.price = (action[0] + 1) * 5
        pass

    def reset(self):
        self.price = 0
        pass

    def update_lane_vehicle(self):
        lane_vehicles = self.eng.get_lane_vehicles()
        current_vehicles = lane_vehicles[self.id]
        self.last_vehicle = current_vehicles

    def get_lane_length(self, world):
        return world.all_roads[int(int(self.id)/100)].info["length"]


class Route():
    def __init__(self, world, origin, destination, id):
        self.id = id
        self.world = world
        self.idx = int(self.id.split("_")[-1])
        self.road_list = world.route["-".join([origin, destination])][self.idx]
        self.lane_list = world.get_lane_route(self.road_list)
        self.price = 0

    def get_speed(self):
        lane_vehicle_speed = self.world.get_lane_vehicle_speed()
        route_speed = []
        for lane in self.lane_list:
            route_speed.append(lane_vehicle_speed[lane])
        route_speed = np.mean(route_speed)
        # print(lane, route_speed)
        return route_speed

    def step(self, action):
        # set price for a road
        self.price = (action[0] + 1) * 5

    def reset(self):
        self.price = 0

    def get_vehicle(self):
        control_vehicle = []
        bg_vehicle = []
        world_vehicle = self.world.eng.get_lane_vehicles()
        route_vehicle = []
        for lane in self.lane_list:
            route_vehicle.extend(world_vehicle[lane])
        for vehicle in route_vehicle:
            v_id = int(vehicle.split("_")[1])
            if v_id < 4:
                control_vehicle.append(vehicle)
            else:
                bg_vehicle.append(vehicle)
        return control_vehicle, bg_vehicle


class Intersection(object):
    def __init__(self, id, intersection, world):
        self.id = id
        self.eng = world.eng

        # incoming and outgoing roads of each intersection, clock-wise order from North
        self.roads = []
        self.outs = []
        self.directions = []
        self.out_roads = None
        self.in_roads = None

        # links and phase information of each intersection
        self.roadlinks = []
        self.lanelinks_of_roadlink = []
        self.startlanes = []
        self.lanelinks = []
        self.phase_available_roadlinks = []
        self.phase_available_lanelinks = []
        self.phase_available_startlanes = []

        self.reset()

    def insert_road(self, road, out, direction):
        self.roads.append(road)
        self.outs.append(out)
        self.directions.append(direction)

    def sort_roads(self, RIGHT):
        order = sorted(
            range(len(self.roads)),
            key=lambda i: (
                self.directions[i],
                self.outs[i] if RIGHT else not self.outs[i],
            ),
        )
        self.roads = [self.roads[i] for i in order]
        self.directions = [self.directions[i] for i in order]
        self.outs = [self.outs[i] for i in order]
        self.out_roads = [self.roads[i] for i, x in enumerate(self.outs) if x]
        self.in_roads = [self.roads[i] for i, x in enumerate(self.outs) if not x]

    def _change_phase(self, phase, interval):
        flag = self.eng.set_ttl_phase(self.id, phase+1)
        # print("intersection: {}, phase: {}, flag: {}".format(self.id, phase+1, flag))
        self._current_phase = phase
        self.current_phase_time = interval

    def step(self, action, interval):
        # if current phase is yellow, then continue to finish the yellow phase
        # recall self._current_phase means true phase id (including yellows)
        # self.current_phase means phase id in self.phases (excluding yellow)
        if action == self.current_phase:
            self.current_phase_time += interval
        else:
            self._change_phase(action, interval)
            self.current_phase = action

    def reset(self):
        # record phase info
        self.current_phase = 0  # phase id in self.phases (excluding yellow)
        # true phase id (including yellow)
        self._current_phase = 0
        self.eng.set_ttl_phase(self.id, self._current_phase+1)
        self.current_phase_time = 0
        self.action_before_yellow = None


class World(object):
    """
    Create a CityFlow engine and maintain informations about CityFlow world
    """

    def __init__(self, engine_cfg, world_config, thread_num, args):
        print("building world...")
        self.eng = cbengine.Engine(engine_cfg, thread_num)
        self.engine_config = engine_cfg
        self.config = world_config
        self.RIGHT = True   # vehicles moves on the right side
        self.interval = 1
        self.args = args
        self.changed_vehicle_num = {}
        self.epsilon = 1
        self.epsilon_decay = 0.99
        self.focus_lane = []

        # initialize the roadnet
        with open(world_config) as f:
            config = json.load(f)
        self._init_roadnet(config)
        
        # initialize the route
        self.route = self._get_route(config)
        
        # initialize the vehicles
        self.vehicles = []
        self.vehicle_ids = []
        self.id2vehicle = {}
        # TODO: change with the roadnet
        # porto
        self.vehicle_route = {str(key): {step: [0, 0, 0] for step in range(6)} for key in range(4)}
        # others:
        # self.vehicle_route = {"0":[0, 0, 0], "1":[0, 0, 0]}

        # initializing info functions
        self.info_functions = {
            "vehicles": (lambda: self.eng.get_vehicles()),  # delete waiting==True
            "lane_count": self.eng.get_lane_vehicle_count,
            "lane_vehicles": self.eng.get_lane_vehicles,
            "time": self.eng.get_current_time,
            "pressure": self.get_pressure,
            "lane_waiting_time_count": self.get_lane_waiting_time_count,
            "lane_delay": self.get_lane_delay,
            "vehicle_trajectory": self.get_vehicle_trajectory,
            "history_vehicles": self.get_history_vehicles,
            "lane_vehicle_speed": self.get_lane_vehicle_speed,
            "lane_throughput": self.get_lane_throughput,
            "lane_distance": self.get_lane_distance,
            "reward_function": self.get_reward_function,
        }
        self.fns = []
        self.info = {}

        # key: vehicle_id, value: the waiting time of this vehicle since last halt.
        self.vehicle_waiting_time = {}
        # key: vehicle_id, value: [[lane_id_1, enter_time, time_spent_on_lane_1], ... , [lane_id_n, enter_time, time_spent_on_lane_n]]
        self.vehicle_trajectory = {}
        self.history_vehicles = set()

        self.lane_vehicles = self.eng.get_lane_vehicles()  # used to distance
        self.last_lane_vehicles = self.eng.get_lane_vehicles()  # used to throughput
        self.vehicle_enter_time = {key: {} for key in self.all_lane_ids}
        self.travel_times = {key: {} for key in self.all_lane_ids}
        print("world built.")

    # initialize the roadnet
    def _init_roadnet(self, config):

        roadnet_file = osp.join(config["dir"], config["roadnetFile"])
        self.intersections = {}
        self.roads = {}
        self.agents = {}
        self.lane_vehicle_state = {}
        with open(roadnet_file,'r') as f:
            lines = f.readlines()
            cnt = 0
            pre_road = 0
            is_obverse = 0
            for line in lines:
                line = line.rstrip('\n').split(' ')
                if('' in line):
                    line.remove('')
                if(len(line) == 1): ## the notation line
                    if(cnt == 0):
                        self.agent_num = int(line[0])   ## start the intersection segment
                        cnt+=1
                    elif(cnt == 1):
                        self.road_num = int(line[0])*2  ## start the road segment
                        cnt +=1
                    elif(cnt == 2):
                        self.signal_num = int(line[0])  ## start the signal segment
                        cnt+=1
                else:
                    if(cnt == 1):   ## in the intersection segment
                        self.intersections[int(line[2])] = {
                            'latitude':float(line[0]),
                            'longitude':float(line[1]),
                            'have_signal':int(line[3]),
                            'end_roads':[],
                            'start_roads':[]
                        }
                    elif(cnt == 2): ## in the road segment
                        if(len(line)!=8):
                            road_id = pre_road[is_obverse]
                            self.roads[road_id]['lanes'] = {}
                            for i in range(self.roads[road_id]['num_lanes']):
                                self.roads[road_id]['lanes'][road_id*100+i] = list(map(int,line[i*3:i*3+3]))
                                self.lane_vehicle_state[road_id*100+i] = set()
                            is_obverse ^= 1
                        else:
                            self.roads[int(line[-2])]={
                                'start_inter':int(line[0]),
                                'end_inter':int(line[1]),
                                'length':float(line[2]),
                                'speed_limit':float(line[3]),
                                'num_lanes':int(line[4]),
                                'inverse_road':int(line[-1])
                            }
                            self.roads[int(line[-1])] = {
                                'start_inter': int(line[1]),
                                'end_inter': int(line[0]),
                                'length': float(line[2]),
                                'speed_limit': float(line[3]),
                                'num_lanes': int(line[5]),
                                'inverse_road':int(line[-2])
                            }
                            self.intersections[int(line[0])]['end_roads'].append(int(line[-1]))
                            self.intersections[int(line[1])]['end_roads'].append(int(line[-2]))
                            self.intersections[int(line[0])]['start_roads'].append(int(line[-2]))
                            self.intersections[int(line[1])]['start_roads'].append(int(line[-1]))
                            pre_road = (int(line[-2]),int(line[-1]))
                    else:
                        # 4 out-roads
                        signal_road_order = list(map(int,line[1:]))
                        now_agent = int(line[0])
                        in_roads = []
                        for road in signal_road_order:
                            if(road != -1):
                                in_roads.append(self.roads[road]['inverse_road'])
                            else:
                                in_roads.append(-1)
                        in_roads += signal_road_order
                        self.agents[now_agent] = in_roads

                        # 4 in-roads
                        # self.agents[int(line[0])] = self.intersections[int(line[0])]['end_roads']
                        # 4 in-roads plus 4 out-roads
                        # self.agents[int(line[0])] += self.intersections[int(line[0])]['start_roads']
        for agent,agent_roads in self.agents.items():
            self.intersections[agent]['lanes'] = []
            for road in agent_roads:
                ## here we treat road -1 have 3 lanes
                if(road == -1):
                    for i in range(3):
                        self.intersections[agent]['lanes'].append(-1)
                else:
                    for lane in self.roads[road]['lanes'].keys():
                        self.intersections[agent]['lanes'].append(lane)

        self.intersections[int(line[2])] = {
                            'latitude':float(line[0]),
                            'longitude':float(line[1]),
                            'have_signal':int(line[3]),
                            'end_roads':[],
                            'start_roads':[]
                        }

        '''
            Create intersections:
                -all_intersections: dict, id: intersection info (include virtual intersections)
                -intersections: list, intersection objects (only non-virtual intersections)
                -intersections_ids: list, ids of intersections
                -id2intersection: dict, id to object
        '''
        print("creating intersections...")
        self.intersection_ids = []
        for key in self.intersections.keys():
            if len(self.intersections[key]['start_roads']) > 1:
                self.intersection_ids.append(key)
        print("non-virtual intersections: {}".format(self.intersection_ids))
        self.all_intersections = self.intersections      
        self.intersections = [Intersection(id, self.all_intersections[id], self) for id in self.intersection_ids]
        self.id2intersection = {i.id: i for i in self.intersections}
        print("intersections created.")

        '''
            Create roads:
                -all_road_ids: list, road ids
                -all_roads: dict, id: road objs
                -all_lane_ids: list, lane ids
                -all_lanes: list, lane objs
                -id2lane: dict, intersection objects
                -id2road: list, ids of intersections
        '''
        print("parsing roads...")
        self.all_road_ids = []
        self.all_roads = {}
        self.all_lane_ids = []
        self.all_lanes = {}
        for road_id, road in self.roads.items():

            self.all_road_ids.append(road_id)
            self.all_roads[road_id] = Road(road_id, road, self)
            for i in range(road["num_lanes"]):
                lane_id = int(road_id) * 100 + i
                self.all_lane_ids.append(lane_id)
                self.all_lanes[lane_id]=Lane(lane_id, self)

            delta_x = self.all_intersections[road["end_inter"]]['longitude']\
                - self.all_intersections[road["start_inter"]]['longitude']
            delta_y = self.all_intersections[road["end_inter"]]['latitude']\
                - self.all_intersections[road["start_inter"]]['latitude']
            direction = _get_direction(delta_y, delta_x)
            iid = road["start_inter"]
            if iid in self.intersection_ids:
                self.id2intersection[iid].insert_road(road, True, direction)
            iid = road["end_inter"]
            if iid in self.intersection_ids:
                self.id2intersection[iid].insert_road(road, False, direction)

        for inter in self.intersections:
            inter.sort_roads(self.RIGHT)
        print("roads parsed.")

        '''
            For congestion pricing, we focus on the non-margin roads
            Create roads:
                -train_id2road: dict, road id: road train id
                -train_id2allroad: dict, road train id: road objs
                -train_id2lane: list, lane ids
                -train_id2alllane: list, lane objs
        '''

        self.train_id2inter = {}
        self.inter2train_id = {}
        self.train_id2road = {}
        self.road2train_id = {}
        # write the train_id2inter
        train_id_inter = 0
        for intersection_id in enumerate(self.intersection_ids):    # here, intersection_ids is non-virtual
            self.train_id2inter[train_id_inter] = intersection_id
            self.inter2train_id[intersection_id] = train_id_inter
            train_id_inter += 1

        # write the train_id2road
        '''
        #   write the margin_road
        virtual_intersections = [i for iid, i in self.all_intersections.items() 
            if iid not in self.intersection_ids]
        self.margin_road = []
        for i in virtual_intersections:
            con_roads = i['start_roads'] + i['end_roads']
            self.margin_road.extend(con_roads)
        '''
        #   write the train_id2road
        train_id_road = 0
        for road_id in self.all_road_ids:
            # if road_id not in self.margin_road:
            self.train_id2road[train_id_road] = road_id
            self.road2train_id[road_id] = train_id_road
            train_id_road += 1

    # load the optional routes for Congestion Pricing
    def _get_route(self, config):
        route_file = osp.join(config["dir"], config["routeFile"])
        with open(route_file) as f:
            route = json.load(f)
        return route

    def get_lane_vehicle_speed(self):
        # value:{lane_id:[vehicle_id]}
        lane_vehicles = self.eng.get_lane_vehicles()
        lane_vehicle_speed = {}
        for lane in self.all_lane_ids:
            vehicle_speed = []
            if len(lane_vehicles[lane]) == 0:
                lane_vehicle_speed[lane] = 16.7 # default
                continue
            for vehicle in lane_vehicles[lane]:
                vehicle_speed.append(float(self.eng.get_vehicle_info(vehicle)["speed"]))
            # print("lane:{},vehicle_speed:{}".format(lane,vehicle_speed))
            lane_vehicle_speed[lane] = np.mean(vehicle_speed)
            # + 0.5 * len(
            #     [v for v in vehicle_speed if v != 0]
            # )
            # print("lane:{},lane_vehicle_speed:{}".format(lane,lane_vehicle_speed[lane]))
        return lane_vehicle_speed

    # help remove the edges (road links) that indicates a turn-over
    def forbidden_edge(self):
        forbiddenr2r = {}  
        forbidden_all = []
        for rid, road in self.all_roads.items():
            forbiddenr2r[rid] = road.info['inverse_road']
        for rid in forbiddenr2r:
            forbidden_all.append([rid, forbiddenr2r[rid]])
        return forbidden_all

    # generate a road link graph
    def generate_edge_index(self):
        '''
        return data, train_data's edges(tensor)
        '''
        edge_index = []
        interinfo = {}

        # all
        interinfo = {}
        for inter_id in self.all_intersections.keys():
            interinfo[inter_id] = {"asstart": [], "asend": []}
        for road_id, road in self.all_roads.items():
            start_inter = road.info["start_inter"]
            end_inter = road.info["end_inter"]
            interinfo[start_inter]["asstart"].append(road_id)
            interinfo[end_inter]["asend"].append(road_id)
        for value in interinfo.values():
            for road2 in value["asend"]:
                for road1 in value["asstart"]:
                    edge_index.append([self.road2train_id[road2], self.road2train_id[road1]])
        
        forbidden_edge = self.forbidden_edge()
        for forbd in forbidden_edge:
            try:
                edge_index.remove(forbd)
            except:
                pass
        
        edge_index = torch.tensor(edge_index, dtype=torch.long).t().contiguous()
        return edge_index

    def get_reward_function(self):
        lane_reward = {}
        current_lane_vehicle_count = self.eng.get_lane_vehicle_count()
        for lane in self.all_lane_ids:
            if current_lane_vehicle_count[lane] <= 12:
                lane_reward[lane] = current_lane_vehicle_count[lane]
            else:
                lane_reward[lane] = 24 - current_lane_vehicle_count[lane]
        return lane_reward

    def get_road_vehicle_count(self):
        """
        use as state
        """
        lane_vehicle_cnt = self.eng.get_lane_vehicle_count()
        road_vehicle_count = {}
        for road_id, road in self.all_roads.items():
            road_vehicle_count[road_id] = 0
            for i in range(road.info["num_lanes"]):
                lane = int(road_id) * 100 + i
                if lane in lane_vehicle_cnt.keys():
                    road_vehicle_count[road_id] += lane_vehicle_cnt[lane]

        return road_vehicle_count

    def get_obs(self):
        road_vehicle = self.get_road_vehicle_count()
        state = [[road_vehicle[road_id]] for road_id in self.train_id2road.values()]
        state = torch.FloatTensor(state)
        return state


    def get_delta_toll(self):
        beta = 8
        # TODO: change with the roadnet
        # 16x3
        # maxSpeed = 16.7
        # 4x4
        # maxSpeed = 11.112
        # porto
        maxSpeed = 16.7
        # free_tt = {key: self.all_lanes[i].length / maxSpeed for i, key in enumerate(self.all_lane_ids)}
        free_tt = {key: self.all_lanes[key].length / maxSpeed for key in self.focus_lane}
        actual_tt = self.get_lane_average_traveltime()
        # delta_toll = {key: np.array([beta * (actual_tt[key] - free_tt[key])/10]) for key in self.all_lane_ids}
        delta_toll = {key: np.array([beta * (actual_tt[key] - free_tt[key]) / 10]) for key in self.focus_lane}
        return delta_toll

    def get_lane_average_traveltime(self):
        # global_vehicles = self.eng.get_lane_vehicles()
        # vehicles = global_vehicles[self.lane]
        # current_time = self.eng.get_current_time()
        actual_tt = {key: {} for key in self.focus_lane}
        # TODO:
        maxSpeed = 16.7
        MIN_TIME = {key: self.all_lanes[key].length / maxSpeed for key in self.focus_lane}
        # MIN_TIME = 300/16.7
        for lane in self.focus_lane:
            if len(self.travel_times[lane]) != 0:
                actual_tt[lane] = np.mean([value for key, value in self.travel_times[lane].items()])
            else:
                actual_tt[lane] = MIN_TIME[lane]
                # actual_tt[lane] = MIN_TIME
        self.travel_times = {key: {} for key in self.focus_lane}
        return actual_tt

    def update_toll(self):
        global_vehicles = self.eng.get_lane_vehicles()  
        current_time = self.get_info("time")
        for lane in self.focus_lane:
            for vehicle in global_vehicles[lane]:
                if not vehicle in self.vehicle_enter_time[lane]:  
                    self.vehicle_enter_time[lane][vehicle] = current_time 
            for vehicle in list(self.vehicle_enter_time[lane]):  
                if not vehicle in global_vehicles[lane]:  
                    self.travel_times[lane][vehicle] = current_time - self.vehicle_enter_time[lane][vehicle]
                    del self.vehicle_enter_time[lane][vehicle]  


    def get_road_distance(self):
        lane_distance = self.get_lane_distance()
        road_distance = {}
        for road_id, road in self.all_roads.items():
            road_distance[road_id] = 0
            for i in range(len(road.info["num_lanes"])):
                lane = int(road_id) * 100 + int(i)
                road_distance[road_id] += lane_distance[lane]

        return road_distance

    def get_lane_distance(self):
        detail = {}
        lane_distance = {}
        current_lane_vehicles = self.eng.get_lane_vehicles()
        old_lane_vehicles = self.lane_vehicles
        for lane in self.all_lane_ids:
            if lane not in old_lane_vehicles.keys():
                old_lane_vehicles[lane] = []
            if lane not in current_lane_vehicles.keys():
                current_lane_vehicles[lane] = []
            detail[lane] = {}
            left_vehicles = set(old_lane_vehicles[lane]) - set(
                current_lane_vehicles[lane]
            )
            remain_vehicles = set(old_lane_vehicles[lane]) & set(
                current_lane_vehicles[lane]
            )
            new_vehicles = set(current_lane_vehicles[lane]) - set(
                old_lane_vehicles[lane]
            )
            lane_distance[lane] = np.array([0])
            road = self._get_road_for_lane(lane)
            length = self.all_roads[road].length

            for vec_id in left_vehicles:
                vehicle = self.vehicles[self.id2vehicle[vec_id]]
                cur_dis = length - vehicle.last_distance
                lane_distance[lane] = lane_distance[lane] + cur_dis
                detail[lane][vec_id] = cur_dis
                # try:
                #     in_road = self.eng.get_vehicle_info(vehicle.id)["road"]
                # except (KeyError,RuntimeError):
                #     # vehicle.last_distance = 0
                #     continue
                # vehicle.last_distance = float(self.eng.get_vehicle_info(vehicle.id)['distance'])
            for vec_id in remain_vehicles:
                vehicle = self.vehicles[self.id2vehicle[vec_id]]
                cur_dis = (
                        float(self.eng.get_vehicle_info(vehicle.id)["distance"][0])
                        - vehicle.last_distance
                )
                vehicle.last_distance = float(
                    self.eng.get_vehicle_info(vehicle.id)["distance"][0]
                )
                lane_distance[lane] = lane_distance[lane] + cur_dis
                detail[lane][vec_id] = cur_dis
            for vec_id in new_vehicles:
                vehicle = self.vehicles[self.id2vehicle[vec_id]]
                cur_dis = float(self.eng.get_vehicle_info(vehicle.id)["distance"][0])
                vehicle.last_distance = float(
                    self.eng.get_vehicle_info(vehicle.id)["distance"][0]
                )
                lane_distance[lane] = lane_distance[lane] + cur_dis
                detail[lane][vec_id] = cur_dis

        self.lane_vehicles = current_lane_vehicles
        return lane_distance, detail

    def get_continuous_vehicle_count(self):
        continuous_vehicle_count = {}
        for road_id, road in self.all_roads.items():
            continuous_vehicle_count[road_id] = road.history_vehicle_count
        return continuous_vehicle_count

    def get_lane_throughput(self):
        current_lane_vehicles = self.eng.get_lane_vehicles()
        old_lane_vehicles = self.last_lane_vehicles
        lane_throughput = {}
        for lane in self.all_lane_ids:
            left_vehicles = set(old_lane_vehicles[lane]) - set(
                current_lane_vehicles[lane]
            )
            lane_throughput[lane] = len(left_vehicles)
            # if(lane_throughput[lane]!=0):
            #     print("lane:{}-throughput:{}".format(lane,lane_throughput[lane]))
            # print(lane,":",left_vehicles)
        self.last_lane_vehicles = current_lane_vehicles  # used to distance
        return lane_throughput

    def get_lane_vehicle_speed(self):
        lane_vehicles = self.eng.get_lane_vehicles()
        lane_vehicle_speed = {}
        for lane in self.all_lane_ids:
            vehicle_speed = []
            if len(lane_vehicles[lane]) == 0:
                lane_vehicle_speed[lane] = 16.7
                continue
            for vehicle in lane_vehicles[lane]:
                vehicle_speed.append(float(self.eng.get_vehicle_info(vehicle)["speed"]))
            # print("lane:{},vehicle_speed:{}".format(lane,vehicle_speed))
            lane_vehicle_speed[lane] = np.mean(vehicle_speed) + 0.5 * len(
                [v for v in vehicle_speed if v != 0]
            )
            # print("lane:{},lane_vehicle_speed:{}".format(lane,lane_vehicle_speed[lane]))
        return lane_vehicle_speed

    def get_road_vehicle_speed(self):
        lane_vehicles = self.eng.get_lane_vehicles()
        road_vehicle_speed = {}
        road_vehicle_cnt = {}
        for road_id, road in self.all_roads.items():
            road_vehicle_cnt[road_id] = 0
            road_vehicle_speed[road_id] = 0
            for i in range(road.info["num_lanes"]):
                lane = int(road_id) * 100 + int(i)
                if lane in lane_vehicles.keys():
                    road_vehicle_cnt[road_id] += len(lane_vehicles[lane])
                    for vehicle in lane_vehicles[lane]:
                        road_vehicle_speed[road_id] += self.eng.get_vehicle_info(vehicle)["speed"][0]
            road_vehicle_speed[road_id] = road_vehicle_speed[road_id] / road_vehicle_cnt[road_id]\
                if  road_vehicle_cnt[road_id] > 0 else 16.7

        return road_vehicle_speed

    def get_pressure(self):
        vehicles = self.eng.get_lane_vehicle_count()
        pressures = {}
        for i in self.intersections:
            pressure = 0
            in_lanes = []
            for road in i.in_roads:
                from_zero = (
                    (road["start_inter"] == i.id)
                    if self.RIGHT
                    else (road["end_inter"] == i.id)
                )
                for n in range(len(road["lanes"]))[:: (1 if from_zero else -1)]:
                    in_lanes.append(road["id"] + "_" + str(n))
            out_lanes = []
            for road in i.out_roads:
                from_zero = (
                    (road["end_inter"] == i.id)
                    if self.RIGHT
                    else (road["start_inter"] == i.id)
                )
                for n in range(len(road["lanes"]))[:: (1 if from_zero else -1)]:
                    out_lanes.append(road["id"] + "_" + str(n))
            for lane in vehicles.keys():
                if lane in in_lanes:
                    pressure += vehicles[lane]
                if lane in out_lanes:
                    pressure -= vehicles[lane]
            pressures[i.id] = pressure
        return pressures

    # return [self.dic_lane_waiting_vehicle_count_current_step[lane] for lane in self.list_entering_lanes] + \
    # [-self.dic_lane_waiting_vehicle_count_current_step[lane] for lane in self.list_exiting_lanes]

    def get_vehicle_lane(self):
        # get the current lane of each vehicle. {vehicle_id: lane_id}
        vehicle_lane = {}
        lane_vehicles = self.eng.get_lane_vehicles()
        for lane in self.all_lane_ids:
            for vehicle in lane_vehicles[lane]:
                vehicle_lane[vehicle] = lane
        return vehicle_lane

    def get_vehicle_waiting_time(self):
        # the waiting time of vehicle since last halt.
        vehicles = self.eng.get_vehicles(include_waiting=False)
        vehicle_speed = self.eng.get_vehicle_speed()
        for vehicle in vehicles:
            if vehicle not in self.vehicle_waiting_time.keys():
                self.vehicle_waiting_time[vehicle] = 0
            if vehicle_speed[vehicle] < 0.1:
                self.vehicle_waiting_time[vehicle] += 1
            else:
                self.vehicle_waiting_time[vehicle] = 0
        return self.vehicle_waiting_time

    def get_lane_waiting_time_count(self):
        # the sum of waiting times of vehicles on the lane since their last halt.
        lane_waiting_time = {}
        lane_vehicles = self.eng.get_lane_vehicles()
        vehicle_waiting_time = self.get_vehicle_waiting_time()
        for lane in self.all_lane_ids:
            lane_waiting_time[lane] = 0
            for vehicle in lane_vehicles[lane]:
                lane_waiting_time[lane] += vehicle_waiting_time[vehicle]
        return lane_waiting_time

    def get_lane_delay(self):
        # set speed limit to 11.11 by default
        speed_limit = 11.11
        lane_vehicles = self.eng.get_lane_vehicles()
        lane_delay = {}
        lanes = self.all_lane_ids
        vehicle_speed = self.eng.get_vehicle_speed()

        for lane in lanes:
            vehicles = lane_vehicles[lane]
            lane_vehicle_count = len(vehicles)
            lane_avg_speed = 0.0
            for vehicle in vehicles:
                speed = vehicle_speed[vehicle]
                lane_avg_speed += speed
            if lane_vehicle_count == 0:
                lane_avg_speed = speed_limit
            else:
                lane_avg_speed /= lane_vehicle_count
            lane_delay[lane] = 1 - lane_avg_speed / speed_limit
        return lane_delay

    def get_vehicle_trajectory(self):
        # lane_id and time spent on the corresponding lane that each vehicle went through
        vehicle_lane = self.get_vehicle_lane()
        vehicles = self.eng.get_vehicles(include_waiting=False)
        for vehicle in vehicles:
            if vehicle not in self.vehicle_trajectory:
                self.vehicle_trajectory[vehicle] = [
                    [vehicle_lane[vehicle], int(self.eng.get_current_time()), 0]
                ]
            else:
                if vehicle not in vehicle_lane.keys():
                    continue
                if vehicle_lane[vehicle] == self.vehicle_trajectory[vehicle][-1][0]:
                    self.vehicle_trajectory[vehicle][-1][2] += 1
                else:
                    self.vehicle_trajectory[vehicle].append(
                        [vehicle_lane[vehicle], int(self.eng.get_current_time()), 0]
                    )
        return self.vehicle_trajectory

    def get_history_vehicles(self):
        self.history_vehicles.update(self.eng.get_vehicles())
        return self.history_vehicles


    def subscribe(self, fns):
        if isinstance(fns, str):
            fns = [fns]
        for fn in fns:
            if fn in self.info_functions:
                if not fn in self.fns:
                    self.fns.append(fn)
            else:
                raise Exception("info function %s not exists" % fn)

    def step(self, actions=None):
        '''
            actions:
                actions["tsc"]: a list, corresponding to the train_id
                actions["cp"]: a list, corresponding to the train_id
                actions["vehicle"]: a dict
        '''


        if actions is not None:            
            # here, we only consider non-virtual intersection, and the indices have corresponded
            for i, action in enumerate(actions["tsc"]):
                self.intersections[i].step(action, self.interval)
            for train_id, action in enumerate(actions["cp"]):
                self.all_roads[self.train_id2road[train_id]].step(action, self.interval)
            for vehicle in self.vehicles:
                # print(actions["vehicle"])
                if vehicle.id in actions["vehicle"].keys():
                    vehicle.step(actions["vehicle"][vehicle.id], self.interval)
            # print(self.eng.get_current_time())
            # show price
            price_record = {}
            for road_id, road in self.all_roads.items():
                price_record[road_id] = road.price
        self.eng.next_step()
        # print("{}, vehicle num: {}".format(self.eng.get_current_time(), self.eng.get_vehicle_count()))
        # print("real phase: {}".format(self.eng.get_ttl_phase(6)))
        '''
        if self.eng.get_current_time() % 20 == 0:
            print(self.eng.get_lane_vehicle_count())
        
        try:
            print("vehicle: {}".format(self.eng.get_lane_vehicles()[13800]))
        except:
            pass
        '''
        self._update_vehicles()
        self._update_infos()

    def reset(self):
        self.eng.reset()
        self.vehicles = []
        self.vehicle_ids = []
        self.id2vehicle = {}
        self.lane_vehicles = self.eng.get_lane_vehicles()
        # TODO: change with the roadnet
        # porto
        #self.vehicle_route = {str(key): {step: [0, 0, 0] for step in range(6)} for key in range(4)}
        # others:
        # self.vehicle_route = {"0": [0, 0, 0], "1": [0, 0, 0]}

        self.vehicle_enter_time = {key: {} for key in self.all_lane_ids}
        self.travel_times = {key: {} for key in self.all_lane_ids}

        for I in self.intersections:
            I.reset()

        for V in self.vehicles:
            V.reset()

        for R in self.all_roads.values():
            R.reset()

        for L in self.all_lanes.values():
            L.reset()

        self._update_vehicles()
        self._update_infos()

    def _update_vehicles(self):

        # print("parsing vehicles")
        new_vehicle_ids = self.eng.get_vehicles()
        old_vehicle_ids = self.vehicle_ids
        new_entered_vehicle_ids = set(new_vehicle_ids) - set(old_vehicle_ids)
        new_left_vehicle_ids = set(old_vehicle_ids) - set(new_vehicle_ids)

        # print("new_left_vehicle_ids:",new_left_vehicle_ids)

        # update vehicle in_system
        for vec_id in new_left_vehicle_ids:
            self.vehicles[self.id2vehicle[vec_id]].exit_system()

        # add new vehicles
        for vec_id in new_entered_vehicle_ids:
            self.id2vehicle[vec_id] = len(self.vehicle_ids)
            self.vehicle_ids.append(vec_id)
            self.vehicles.append(
                Vehicle(vec_id, self.eng.get_vehicle_info(vec_id), self)
            )

        # print("vehicles parsed")

    def _update_infos(self):
        self.info = {}
        for fn in self.fns:
            self.info[fn] = self.info_functions[fn]()

    def get_info(self, info):
        return self.info[info]

    def _get_road_for_lane(self, lane):
        return int(int(lane)/100)


if __name__ == "__main__":
    world = World("dataset/3x3/config.json", thread_num=1)
    # print(len(world.intersections[0].startlanes))
    print(world.intersections[0].phase_available_startlanes)

