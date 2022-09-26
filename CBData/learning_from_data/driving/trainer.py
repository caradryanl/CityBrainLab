import os, sys
from tkinter import font
import numpy as np
import time
import torch
import torch.nn.functional as F
from openbox import Optimizer, sp
import matplotlib.pyplot as plt

# sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cbengine

class Trainer(object):
    '''
        need to generate:
            X: [T/t, R, C_s+C_d]
            y: [T/t, R]
        where 
            T is the total time span of the flow, 
            t is the length of cycle,
            R is the number of roads
            C_s and C_d are number of channels on the view of Supply and Demand.
    '''
    def __init__(self, cfg_file, roadnet_file, flow_file, label=None,
         length_interval = 60, num_interval = 10, tot_interval = 20):
        '''
            
                
        '''
        self.cfg_file = cfg_file
        self.roadnet_file = roadnet_file
        self.flow_file = flow_file
        self.label = label
        self.length_interval = length_interval
        self.num_interval = num_interval
        self.tot_interval = tot_interval
        
        # refer to cbengine/env/CBEngine/envs/CBEngine.py
        # here agent is those intersections with signals
        self.intersections = {}
        self.roads = {}
        self.agents = {}
        self.lane_vehicle_state = {}
        self.log_enable = 1
        self.warning_enable = 1
        self.ui_enable = 1
        self.info_enable = 1
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

    def compute_speed_avg(self, vehicle_speed):
        num_vehicle = len(vehicle_speed.keys())
        speed = 0.0

        if num_vehicle == 0:
            return 0

        for v in vehicle_speed.values():
            speed += v
        return speed / float(num_vehicle)

    def compute_res(self, config):
        res = []
        engine = cbengine.Engine(self.cfg_file, 12)

        # set params
        cf_params = [config["max_acc_"], config["min_acc_"]]
        sl_params = config["speed_limit_"]
        engine.set_car_following_params(cf_params)
        engine.set_road_velocity(4192, sl_params)

        # warm up
        for _ in range(self.tot_interval - self.num_interval):
            for _ in range(self.length_interval):
                for intersection in self.intersections.keys():
                    engine.set_ttl_phase(intersection, (int(engine.get_current_time()) // 30) % 4 + 1)
                engine.next_step()

        # compute 
        for _ in range(self.num_interval):
            res_value = 0.0
            for _ in range(self.length_interval):
                for intersection in self.intersections.keys():
                    engine.set_ttl_phase(intersection, (int(engine.get_current_time()) // 30) % 4 + 1)
                res_value += self.compute_speed_avg(engine.get_vehicle_speed())
                engine.next_step()
                # print("t: {}, v: {}".format(engine.get_current_time(), engine.get_vehicle_count()))
            res.append(res_value / float(self.length_interval))
        # print(res)
        return res
    
    def compute_score(self, config):
        res = self.compute_res(config)
        res, label = torch.Tensor(res), torch.Tensor(self.label[self.num_interval:])
        loss = F.smooth_l1_loss(res, label).item()
        return loss

    def set_label(self, label):
        self.label = label
    
    def train(self, dataSource, round):

        space = sp.Space()
        max_acc_ = sp.Real("max_acc_", 0, 5.0, default_value=2.0)
        min_acc_ = sp.Real("min_acc_", 0, 10.0, default_value=5.0)
        speed_limit_ = sp.Real("speed_limit_", 11.1, 33.3)
        space.add_variables([max_acc_, min_acc_, speed_limit_])

        cliped_losses = []
        mins = []
        for _ in range(round):
            label = dataSource.get_batch_data()
            self.set_label(label)
            opt = Optimizer(
                self.compute_score,
                space,
                max_runs=20,
                surrogate_type='auto',
                time_limit_per_trial=1000,
                task_id='quick_start',
                acq_optimizer_type='auto',
            )
            history = opt.run()
            print(history)
            loss = history.get_all_perfs()
            n_calls = len(loss)
            iterations = range(1, n_calls + 1)
            min = [np.min(loss[:i]) for i in iterations]
            max_min = max(min)
            cliped_loss = np.clip(loss, None, max_min)
            for it in cliped_loss:
                cliped_losses.append(it)
            for it in min:
                mins.append(it)
            # incumbent = history.get_incumbents()
        n_calls = len(cliped_losses)
        iterations = range(0, n_calls)
    
        self.plot_convergence(iterations, mins, cliped_losses, 
                                splits=[0, 20, 40])
        
        plt.savefig("./figs/loss_sp_{}.png".format(1))
        plt.savefig("./figs/loss_sp_{}.pdf".format(1))

    def plot_convergence(self,
        x, y1, y2,
        xlabel="Time step",
        ylabel=r"Loss values",
        ax=None, name=None, alpha=0.2, yscale=None,
        color=None, true_minimum=None,
        splits = None,
        **kwargs):
        plt.figure(figsize=(20, 8))
        if ax is None:
            ax = plt.gca()
        
        
        font = {'family' : 'DejaVu Sans',
        'weight' : 'normal',
        'size'   : 34,
                }

        ax.set_title("Learning Process, Zhuangcun Road", font)
        ax.set_xlabel(xlabel, font)
        ax.set_ylabel(ylabel, font)
        ax.grid()

        if yscale is not None:
            ax.set_yscale(yscale)

        ax.plot(x, y1, c=color, label="Loss Min", lw=2.5, **kwargs)
        ax.scatter(x, y2, s=24, c=color, alpha=alpha)

        plt.tick_params(labelsize=30)

        '''
        if true_minimum is not None:
            ax.axhline(true_minimum, linestyle="--",
                    color="r", lw=3,
                    label="True minimum")
        '''
        if splits is not None:
            for id, split in enumerate(splits):
                if id == 0:
                    ax.axvline(split, linestyle="--",
                            color="r", lw=2,
                            label="New Data Injection")
                else:
                    ax.axvline(split, linestyle="--",
                            color="r", lw=2)


        # if true_minimum is not None or name is not None:
        ax.legend(loc="upper right", fontsize=30)

        plt.subplots_adjust(left=0.06, right=0.98, top=0.92, bottom=0.12)

        return ax

    

        