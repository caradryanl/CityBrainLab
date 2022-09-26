import torch
import torch.nn.functional as F
import numpy as np
import time
class HumanEcoAgent:
    """
    Agent using Economic model for user
    """

    def __init__(self, vehicle, world):

        self.vehicle = vehicle
        self.world = world

    def get_ob(self):
        lane_vehicle_count = self.world.eng.get_lane_vehicle_count()
        # vehicle_road=self.world.eng.get_vehicle_info(self.vehicle.id).road
        road_vehicle_count = {}
        lane_count = len(self.world.all_lanes)
        for road in self.world.roadnet["roads"]:
            rvc = 0
            for i in range(lane_count):
                lane_id = road["id"] + "_" + str(i)
                rvc = rvc + lane_vehicle_count[lane_id]
            road_vehicle_count[road["id"]] = rvc
        return road_vehicle_count

    def get_action(self, world):
        # todo - get vehicle route time
        # lane_waiting_count = self.world.get_info("lane_waiting_count")
        # road_count = self.get_ob()
        # route_choices = self.vehicle.route_choices
        current_route = self.world.eng.get_vehicle_info(self.vehicle.id)["route"]
        current_route = [int(x) for x in current_route]
        if len(current_route) <= 2:
            return current_route
        else:
            route_choices = world.route["-".join([str(int(current_route[1])), str(int(current_route[-1]))])]
        # print("current: {}, action: {}".format(current_route, route_choices))
        # if self.vehicle.monitor == False:
        #     return current_route[:-1].split(' ')
        '''
        try:
            # id of current road
            current_link = self.world.eng.get_vehicle_info(self.vehicle.id)["road"]
        except KeyError:
            return current_route[:-1].split(" ")
        # check whether it is at the origin link
        if current_link != self.vehicle.original_route[0]:
            return current_route[:-1].split(" ")
        '''
        # vehicle count
        vehicle_cnt = world.get_road_vehicle_count()


        # calculate three route cost
        r = []
        cost = []
        veh = []
        for i in range(len(route_choices)):
            r.append(route_choices[i][0:])
            cost.append(self.cal_cost(route_choices[i][0:]))  # delete the get_lane_route
            veh.append(self.cal_veh(route_choices[i][0:], vehicle_cnt))
            # print("route: {}, cost: {}, veh: {}".format(r[-1], cost[-1], veh[-1]))

        # select one route
        cost = - torch.FloatTensor(cost)
        pro = F.softmax(cost / 50, dim=0)
        seed = time.time()
        seed = int((seed - int(seed))*10e4)
        np.random.seed(seed)
        # choice = np.random.choice(3, 1, p=np.array(pro))[0]
        choice = torch.argmax(cost) 
        real_cost = cost[choice]
        
        r[choice] = [int(x) for x in r[choice]]
        r[choice].insert(0, current_route[0])
        self.vehicle.route = r[choice]
        # print(r[choice], real_cost)
        return r[choice]

    def cal_cost(self, r):

        time_coef = 1
        a = 0.2
        b = 0
        c = 1

        total_fuel = 0
        total_time = 0
        total_cost = 0
        for link_id in r:
            # link_cnt = lc[link_id]
            length = self.world.all_roads[int(link_id)].info["length"]
            total_fuel += length / 1000 * 2.4
            total_cost += self.world.all_roads[int(link_id)].price # id2road

        # if(total_cost!=0):
        #     print("vehicle:{} route is {},total_fuel is {},total_time is {},total_cost is {}".format(self.vehicle.id,r,total_fuel,total_time,total_cost))
        return a * total_fuel + c * total_cost

    def cal_veh(self, r, vehicle_cnt):
        veh = 0
        for link_id in r:
            veh += vehicle_cnt[int(link_id)] # id2road
        return veh


    def get_reward(self):
        return None
