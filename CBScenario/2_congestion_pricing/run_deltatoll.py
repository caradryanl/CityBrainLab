import argparse
from time import time
import os
import gym

from agent.delta_agent import Price_Agent
from actor_critic.replay_memory import AgentReplayMemory
gym.logger.set_level(40)
import numpy as np
import torch
from agent.human_eco_agent import HumanEcoAgent
from environment import VCTEnv
from agent.fixedtime_agent import Fixedtime_Agent
from world import World
from metric import TravelTimeMetric, ThroughputMetric, FuelMetric, TotalCostMetric
import json

parser = argparse.ArgumentParser(description="Formula Price Args")
parser.add_argument(
    "--steps", type=int, default=180, help="number of steps (default: 3600)"
)
parser.add_argument(
    "--thread", type=int, default=8, help="number of threads (default: 8)"
)
parser.add_argument(
    "--num_routes", type=int, default=3, help="number of route choices (default: 3)"
)
parser.add_argument(
    "--replay_size",
    type=int,
    default=2000,
    metavar="N",
    help="size of replay buffer (default: 2000)",
)
parser.add_argument(
    "--action_interval",
    type=int,
    default=20,
    help="how often agent make decisions (default: 120)",
)
parser.add_argument(
    "--episodes", type=int, default=1, help="training episodes (default: 1)"
)
parser.add_argument(
    "--batch_size", type=int, default=32, metavar="N", help="batch size (default: 32)"
)
parser.add_argument(
    "--config_file", type=str, default="./dataset/manhattan/config_manhattan.json", help="path of config file")

parser.add_argument(
    "--engine_config_file", type=str, default="./dataset/manhattan/manhattan.cfg", help="path of config file")
args = parser.parse_args()

config = json.load(open(args.config_file, 'r'))
net = config['dir'].split('/')[1]
flow = config["flowFile"].split('.')[0]
netandflow = net + flow

world = World(args.engine_config_file, args.config_file, thread_num=args.thread, args=args)

dic_agents = {}

# tsc agents
agents = []
for i in world.intersections:
    action_space = gym.spaces.Discrete(4)
    agents.append(Fixedtime_Agent(action_space, i.id))
dic_agents["tsc"] = agents

# # cp agents
# agents = []
# action_space = gym.spaces.Box(np.array([0]), np.array([10]))
# for i in world.all_lanes:
#     agents.append(Price_Agent(i.id, world, R, beta))
# dic_agents['cp'] = agents

# vehicle agents
agents = []
vehicle_action_space = gym.spaces.Discrete(args.num_routes)
for i in world.vehicles:
    agents.append(HumanEcoAgent(i, world))
dic_agents["vehicle"] = agents

# create metric
metric = [
    TravelTimeMetric(world),
    ThroughputMetric(world),
    FuelMetric(world),
    TotalCostMetric(world),
]
metric_name = [
    "Average Travel Time",
    "Average throughput",
    "Average fuel cost",
    "Average total cost",
]

# Memory
memory = AgentReplayMemory(args.replay_size, agent_num=len(world.all_lanes), state_dim=1)

# Agents num:
# a = 4
# b = 4
# a_num = 6*(a*(b+1) + b*(a+1))

# # create env
# env = VCTEnv(world, dic_agents, metric, args)


def test(args, metric_name, R, beta, round_id):
    # cp agents
    agents = []
    action_space = gym.spaces.Box(np.array([0]), np.array([1]))
    for road_id, road in world.all_roads.items():
        agents.append(Price_Agent(road_id, road, world, R, beta))
    dic_agents['cp'] = agents

    # create env
    env = VCTEnv(world, dic_agents, metric, args)
    # TODO: change with roadnet:
    # 16x3
    # train_movement = [238, 292, 352, 208, 502, 613, 13, 207, 601, 604, 250, 544, 184, 365, 520, 334, 322, 563, 142, 310,
    #                   373, 402, 658, 575, 82, 124, 460, 29, 268, 574, 185, 364, 234, 478, 158, 348, 166, 70, 448, 406,
    #                   586, 280, 223, 376, 562, 587, 532, 24, 28, 112, 66, 418, 627, 625, 254, 27, 674, 490, 394, 616,
    #                   154, 12, 74, 226, 436, 196, 40, 335, 111]
    # 4x4
    # train_movement = [86, 193, 98, 16, 123, 70, 94, 147, 160, 15, 85, 73, 106, 206, 214, 181]
    # train_id = np.array(train_movement)
    initial_price = 5
    interval_reward_record = []
    detail = {}
    for e in range(args.episodes):
        detail[e] = {}
        state_record = []
        action_record = []
        reward_record = []
        travel_time_record = []
        throughput_record = []
        state = env.reset()  # 仅关心road的state
        print("delta_toll", " |episodes is : ", e)
        reward_list = []
        dic_actions = {}
        for i in range(args.steps):
            # road & vehicle take action only if the time is the 'interval'
            print("delta_toll", "|", i, "/", args.steps)
            # print("road_speed: {}".format(world.get_road_vehicle_speed())) # *modify*
            # price the road
            key = "cp"
            dic_actions[key] = []  # only price the trained road
            if i == 0:
                dic_actions[key] = np.array([[initial_price]*1] * len(dic_agents['cp']))
            else:
                for agent in dic_agents['cp']:
                    dic_actions[key].append(agent.get_action(i))
                    # print(agent.road)
                dic_actions[key] = np.array(dic_actions[key])
            dic_actions[key] = dic_actions[key].tolist()
            # print("actions: {}".format(dic_actions[key]))
            # step
            for t in range(args.action_interval):
                # traffic light take action every second
                key = "tsc"
                dic_actions[key] = [agent.get_action(world) for agent in dic_agents[key]]
                # print("tsc: {}".format(dic_actions["tsc"][6]))
                # print("cp: {}".format(dic_actions["cp"]))
                
                _, reward, _, info, vehicle = env.step(dic_actions) # info is the set of new added vehicles
                reward_list.append(reward)
                detail[e][1800 * i + t] = vehicle
                dic_actions["vehicle"] = {}
                for id, agent in enumerate(dic_agents["vehicle"]):
                    if agent is not None and agent.vehicle.id in info and agent.vehicle.monitor:
                        dic_actions["vehicle"][agent.vehicle.id] = agent.get_action(world)
                        # print(agent.vehicle.id, dic_actions["vehicle"][agent.vehicle.id])
                    elif agent is None:
                        pass
                    else:
                        dic_actions["vehicle"][agent.vehicle.id] = []
                
                reward_list.append(reward)
                for ind_m in range(len(env.metric)):
                    env.metric[ind_m].update(done=False)
                for id, agent in enumerate(dic_agents['cp']):
                    agent.update()
            # print(len(next_state[0]))
            pass_distance = np.mean(reward_list, axis=0)
            if i != 0:
                # rewards = pass_distance - pre_pass_distance
                rewards = pass_distance
                interval_reward_record.append(np.sum(rewards))
                reward_list = []

                state_record.append(state)  # empty
                action_record.append([x*5 for x in dic_actions["cp"]])
                reward_record.append(rewards)
                travel_time_record.append(env.metric[0].update(done=False))
                throughput_record.append(env.metric[1].update(done=False))
            # pre_pass_distance = pass_distance
            '''
            if i > 30:
                for road_id, road in world.all_roads.items():
                    print("{}, id: {}, price: {}".format(i, road_id, road.price)) 
            '''
    
        # the following code is done for each episode
        # TODO :change with the date
        dir_name = 'train_log/5-20-1/%s/%s/%s/delta_tolling/%s/' % (round_id, net, flow, e)
        print("total reward:", sum(sum(reward_record)))
        print("ATT:", travel_time_record[-1])
        print("total interval reward:", sum(interval_reward_record))
        # 0-d1; 1-d2
        if not os.path.isdir(dir_name):
            os.makedirs(dir_name)
        state_record = np.concatenate(state_record)
        action_record = np.concatenate(action_record)
        reward_record = np.concatenate(reward_record)
        travel_time_record = np.array(travel_time_record)
        TT_detail = env.metric[0].update(done=True)
        record = {'state': state_record.tolist(), 'action': action_record.tolist(), 'reward': reward_record.tolist(),
                'TT': travel_time_record.tolist(), 'throughput': throughput_record}
        json_str = json.dumps(record, indent=2)
        with open(dir_name + 's-a-r-t.json', 'w') as json_file:
            json_file.write(json_str)
        TT_str = json.dumps(TT_detail, indent=2)
        with open(dir_name + 'TT_detail.json', 'w') as json_file:
            json_file.write(TT_str)
        reroute = json.dumps(world.vehicle_route, indent=2)
        with open(dir_name + 'reroute.json', 'w') as json_file:
            json_file.write(reroute)
        vehicle_pass = json.dumps(detail, indent=2)
        with open(dir_name + 'vehicle_pass.json', 'w') as json_file:
            json_file.write(vehicle_pass)

        reward_json = {}
        reward_json['interval_reward'] = interval_reward_record
        reward_str = json.dumps(reward_json, indent=2)
        with open(dir_name + 'interval_reward.json', 'w') as json_file:
            json_file.write(reward_str)
        dir = 'datasample/5-8/%s/%s/deltatoll/' % (net, flow)
        if not os.path.isdir(dir):
            os.makedirs(dir)
        buffer_size = memory._size
        np.save(dir + 'state.npy', memory.buffer['state'][:buffer_size])
        np.save(dir + 'action.npy', memory.buffer['action'][:buffer_size])
        np.save(dir + 'reward.npy', memory.buffer['reward'][:buffer_size])
        np.save(dir + 'next_state.npy', memory.buffer['next_state'][:buffer_size])


if __name__ == "__main__":
    time0=time()

    # test(args, metric_name, R=10e-3, beta=1, round_id=0)
    # test(args, metric_name, R=10e-1, beta=4, round_id=11)
    # time1 = time.time()
    # print("runtime:\t", time1-time0)
    # test(args, metric_name, R=10e-3, beta=1, round_id=0)
    # test(args, metric_name, R=10e-3, beta=2, round_id=1)
    # test(args, metric_name, R=10e-3, beta=4, round_id=13)
    # test(args, metric_name, R=10e-3, beta=8, round_id=2)
    # test(args, metric_name, R=10e-3, beta=16, round_id=3)
    # test(args, metric_name, R=10e-2, beta=1, round_id=4)
    # test(args, metric_name, R=10e-2, beta=2, round_id=5)
    # test(args, metric_name, R=10e-2, beta=4, round_id=6)
    # test(args, metric_name, R=10e-2, beta=8, round_id=7)
    # test(args, metric_name, R=10e-2, beta=16, round_id=8)
    # test(args, metric_name, R=10e-1, beta=1, round_id=20)
    # test(args, metric_name, R=10e-1, beta=2, round_id=10)
    # test(args, metric_name, R=10e-1, beta=4, round_id=11)
    test(args, metric_name, R=0.6, beta=1, round_id=12)
    # test(args, metric_name, R=10e-4, beta=2, round_id=14)
    # test(args, metric_name, R=10e-4, beta=4, round_id=15)
   # test(args, metric_name, R=10e-4, beta=8, round_id=16)
   # test(args, metric_name, R=10e-1, beta=16, round_id=18)
    time = time()
    print("Time consume: {}".format(time-time0))

