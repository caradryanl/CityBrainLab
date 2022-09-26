import gym
from environment import TSCEnv
from world import World
from generator import LaneVehicleGenerator
from agent.dqn_agent import DQNAgent
from metric import TravelTimeMetric, ThroughputMetric
import argparse
import os
import numpy as np
import logging
from datetime import datetime
import json

# parse args
parser = argparse.ArgumentParser(description='Run Example')
parser.add_argument('--thread', type=int, default=12, help='number of threads')
parser.add_argument('--steps', type=int, default=10800, help='number of steps')
parser.add_argument('--action_interval', type=int, default=20, help='how often agent make decisions')
parser.add_argument('--episodes', type=int, default=200, help='training episodes')
parser.add_argument('--save_model', action="store_true", default=False)
parser.add_argument('--load_model', action="store_true", default=False)
parser.add_argument("--save_rate", type=int, default=20, help="save model once every time this many episodes are completed")
parser.add_argument('--save_dir', type=str, default="model/hangzhou/dqn", help='directory in which model should be saved')
parser.add_argument('--log_dir', type=str, default="log/hangzhou/dqn", help='directory in which logs should be saved')
parser.add_argument("--config_file", type=str, default="./dataset/hangzhou/config_hangzhou.json", help="path of config file")
parser.add_argument("--engine_config_file", type=str, default="./dataset/hangzhou/hangzhou.cfg", help="path of config file")
parser.add_argument("--city", type=str, default="hangzhou", help="city name")
args = parser.parse_args()

if not os.path.exists(args.log_dir):
    os.makedirs(args.log_dir)
logger = logging.getLogger('main')
logger.setLevel(logging.DEBUG)
fh = logging.FileHandler(os.path.join(args.log_dir, datetime.now().strftime('%Y%m%d-%H%M%S') + ".log"))
fh.setLevel(logging.DEBUG)
sh = logging.StreamHandler()
sh.setLevel(logging.INFO)
logger.addHandler(fh)
logger.addHandler(sh)

# create world
world = World(args.engine_config_file, args.config_file, thread_num=args.thread, args=args)

# create agents
agents = []
for i in world.intersections:
    action_space = gym.spaces.Discrete(8)
    agents.append(DQNAgent(
        action_space,
        LaneVehicleGenerator(world, i, ["lane_count"], in_only=True, average=None),
        LaneVehicleGenerator(world, i, ["lane_waiting_count"], in_only=True, average="all", negative=True),
        i.id
    ))
    if args.load_model:
        agents[-1].load_model(args.save_dir)

# create metric
metric = [
    TravelTimeMetric(world),
    ThroughputMetric(world)
]

# create env
env = TSCEnv(world, agents, metric)

def test():
    obs = env.reset()
    travel_time_record = []
    throughput_record = []
    for agent in agents:
        agent.load_model(args.save_dir)
    for i in range(args.steps):
        if i % args.action_interval == 0:
            actions = {}
            actions["tsc"] = []
            for agent_id, agent in enumerate(agents):
                actions["tsc"].append(agent.get_action(obs[agent_id]))
        obs, rewards, dones, info = env.step(actions)
        for ind_m in range(len(env.metric)):
            env.metric[ind_m].update(done=False)
        if all(dones):
            break
        if i % args.action_interval == 0:
            travel_time_record.append(env.metric[0].update(done=False))
            throughput_record.append(env.metric[1].update(done=False))

    dir_name = 'train_log/5-30/%s/flow_%s/dqn/0/' % (args.city, args.city)
    if not os.path.isdir(dir_name):
        os.makedirs(dir_name)
    record = {'TT': travel_time_record, 'throughput': throughput_record}
    json_str = json.dumps(record, indent=2)
    with open(dir_name + 's-a-r-t.json', 'w') as json_file:
        json_file.write(json_str)


if __name__ == '__main__':
    # simulate
    # import os
    # os.environ["CUDA_VISIBLE_DEVICES"] = '0, 1'
    # train(args, env)
    test()
