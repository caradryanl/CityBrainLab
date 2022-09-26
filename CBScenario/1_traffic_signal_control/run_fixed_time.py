import argparse
import gym

gym.logger.set_level(40)
import numpy as np
import torch

# from torch.utils.tensorboard import SummaryWriter
from environment import TSCEnv
from agent.fixedtime_agent import Fixedtime_Agent
from world import World
from metric import TravelTimeMetric, ThroughputMetric, FuelMetric, TotalCostMetric
import os
import json
parser = argparse.ArgumentParser(description="Base Actor-Critic Args")
parser.add_argument(
    "--gamma",
    type=float,
    default=0.99,
    metavar="G",
    help="discount factor for reward (default: 0.99)",
)
parser.add_argument(
    "--tau",
    type=float,
    default=0.125,
    metavar="G",
    help="target smoothing coefficient(τ) (default: 0.125)",
)
parser.add_argument(
    "--alpha",
    type=float,
    default=0.2,
    metavar="G",
    help="Temperature parameter α determines the relative importance of the entropy\
                    term against the reward (default: 0.2)",
)
parser.add_argument(
    "--lr",
    type=float,
    default=0.001,
    metavar="G",
    help="learning rate (default: 0.001)",
)
parser.add_argument(
    "--batch_size", type=int, default=100, metavar="N", help="batch size (default: 4)"
)
parser.add_argument(
    "--start_episodes", type=int, default=10, metavar="N", help="random sample before"
)
parser.add_argument(
    "--update_after", type=int, default=24, metavar="N", help="update parameters"
)
parser.add_argument(
    "--updates_per_step",
    type=int,
    default=10,
    metavar="N",
    help="model updates per simulator step (default: 1)",
)
parser.add_argument(
    "--target_update_interval",
    type=int,
    default=1,
    metavar="N",
    help="Value target update per no. of updates per step (default: 20)",
)
parser.add_argument(
    "--replay_size",
    type=int,
    default=200000,
    metavar="N",
    help="size of replay buffer (default: 2000)",
)
parser.add_argument(
    "--steps", type=int, default=20, help="number of steps (default: 3600)"
)
parser.add_argument(
    "--thread", type=int, default=12, help="number of threads (default: 8)"
)
parser.add_argument(
    "--num_routes", type=int, default=3, help="number of route choices (default: 3)"
)
parser.add_argument(
    "--action_interval",
    type=int,
    default=180,
    help="how often agent make decisions (default: 120)",
)
parser.add_argument(
    "--episodes", type=int, default=1, help="training episodes (default: 1)"
)
parser.add_argument(
    "--config_file", type=str, default="./dataset/hangzhou/config_hangzhou.json", help="path of config file")

parser.add_argument(
    "--engine_config_file", type=str, default="./dataset/hangzhou/hangzhou.cfg", help="path of config file")
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
    action_space = gym.spaces.Discrete(8)
    agents.append(Fixedtime_Agent(action_space, i.id, i))
dic_agents["tsc"] = agents

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

# create env
env = TSCEnv(world, dic_agents["tsc"], metric)

def train(args):
    detail = {}
    for e in range(args.episodes):
        detail[e] = {}
        reward_record = []
        travel_time_record = []
        throughput_record = []
        print('random', " |episodes is : ", e)
        reward_list = []
        dic_actions = {} 

        for i in range(args.steps):
            # random, everything is ok
            key = "cp"
            dic_actions[key] = np.zeros((len(world.all_road_ids), 1))
            # print("road_speed: {}".format(world.get_road_vehicle_speed()))
            print("Step {} finished.".format(i))
            for t in range(args.action_interval):
                # traffic light take action every second
                key = "tsc"
                dic_actions[key] = [agent.get_action(world) for agent in dic_agents[key]]
                
                _, reward, _, _ = env.step(dic_actions) # info is the set of new added vehicles
                reward_list.append(reward)
                dic_actions["vehicle"] = {}
                reward_list.append(reward)
                # update metric
                for ind_m in range(len(env.metric)):
                    env.metric[ind_m].update(done=False)
            pass_distance = np.mean(reward_list, axis=0)
            # compute real reward
            if i != 0:
                # rewards = pass_distance - pre_pass_distance
                rewards = pass_distance
                reward_list = []
                reward_record.append(rewards)
                travel_time_record.append(env.metric[0].update(done=False))
                throughput_record.append(env.metric[1].update(done=False))

        dir_name = 'train_log/5-30/%s/%s/fixtime/%s/' % (net, flow, e)
        if not os.path.isdir(dir_name):
            os.makedirs(dir_name)
        reward_record = np.concatenate(reward_record)
        travel_time_record = np.array(travel_time_record)
        record = {'reward': reward_record.tolist(),'TT': travel_time_record.tolist(), 'throughput': throughput_record}
        json_str = json.dumps(record, indent=2)
        with open(dir_name + 's-a-r-t.json', 'w') as json_file:
            json_file.write(json_str)
        

if __name__ == "__main__":
    train(args)
