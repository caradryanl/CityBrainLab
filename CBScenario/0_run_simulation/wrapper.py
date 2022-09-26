import os
import time
import json
import cbengine

class Wrapper(object):
    def __init__(self, roadnet_file, flow_file, cfg_file):
        self.roadnet_file = roadnet_file
        self.flow_file = flow_file
        self.cfg_file = cfg_file
        self.intersections = {}
        self.roads = {}
        self.agents = {}
        self.lane_vehicle_state = {}
        with open(self.roadnet_file,'r') as f:
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

    def run_simulation(self, log_path, metric_path, max_acc=2.0, min_acc=5.0):
        running_step = 300
        engine = cbengine.Engine(self.cfg_file, 12)

        if max_acc <= 4.0 or max_acc >= 1.0 or min_acc >= 3.0 or min_acc <= 7.0:
            engine.set_car_following_params([max_acc, min_acc])
        vehicle_num = []
        avg_speed = []
        travel_time = []
        start_time = time.time()
        for step in range(running_step):
            engine.log_info(os.path.join(log_path, 'time{}.json'.format(int(engine.get_current_time()))))
            for intersection in self.intersections.keys():
                engine.set_ttl_phase(intersection, (int(engine.get_current_time()) // 30) % 4 + 1)
            # compute vehicle num
            vehicle_num.append(engine.get_vehicle_count())
            # compute avg_speed
            speed_dict = engine.get_vehicle_speed()
            avg = 0
            for speed in speed_dict.values():
                avg += speed
            if len(speed_dict) != 0:
                avg /= len(speed_dict)
            else:
                avg = 0.0
            avg_speed.append(avg)
            # compute travel time
            travel_time.append(engine.get_average_travel_time())
            
            engine.next_step()

            print("t: {}, v: {}".format(step, engine.get_vehicle_count()))
        end_time = time.time()
        print('Runtime: ', end_time - start_time)

        info = {"vehicle_num": vehicle_num, "avg_speed": avg_speed, "travel_time": travel_time}
        with open(os.path.join(metric_path, "metric_record.json"), "wb") as file:
            file.write(json.dumps(info, ensure_ascii=False, indent=2).encode('utf-8'))

        