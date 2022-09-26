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
parser.add_argument('--steps', type=int, default=1800, help='number of steps')
parser.add_argument('--action_interval', type=int, default=30, help='how often agent make decisions')
parser.add_argument('--episodes', type=int, default=50, help='training episodes')
parser.add_argument('--save_model', action="store_true", default=False)
parser.add_argument('--load_model', action="store_true", default=False)
parser.add_argument("--save_rate", type=int, default=20, help="save model once every time this many episodes are completed")
parser.add_argument('--save_dir', type=str, default="model/hangzhou/dqn", help='directory in which model should be saved')
parser.add_argument('--log_dir', type=str, default="log/hangzhou/dqn", help='directory in which logs should be saved')
parser.add_argument("--config_file", type=str, default="./dataset/hangzhou/config_hangzhou.json", help="path of config file")
parser.add_argument("--engine_config_file", type=str, default="./dataset/hangzhou/hangzhou.cfg", help="path of config file")
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

# train dqn_agent
def train(args, env):
    total_decision_num = 0
    travel_time_record = []
    throughput_record = []
    for e in range(args.episodes):
        last_obs = env.reset()
        episodes_rewards = [0 for i in agents]
        episodes_decision_num = 0
        i = 0
        while i < args.steps:
            if i % args.action_interval == 0:
                actions = {}
                actions["tsc"] = []
                for agent_id, agent in enumerate(agents):
                    if total_decision_num > agent.learning_start:
                        action = agent.get_action(last_obs[agent_id])
                        actions["tsc"].append(action)
                    else:
                        actions["tsc"].append(agent.sample())

                rewards_list = []
                for _ in range(args.action_interval):
                    obs, rewards, dones, _ = env.step(actions)
                    for ind_m in range(len(env.metric)):
                        env.metric[ind_m].update(done=False)
                    i += 1
                    rewards_list.append(rewards)
                rewards = np.mean(rewards_list, axis=0)
                # print(last_obs, actions, rewards, obs)

                for agent_id, agent in enumerate(agents):
                    agent.remember(last_obs[agent_id], actions["tsc"][agent_id], rewards[agent_id], obs[agent_id])
                    episodes_rewards[agent_id] += rewards[agent_id]
                    episodes_decision_num += 1
                    total_decision_num += 1
                
                last_obs = obs

            for agent_id, agent in enumerate(agents):
                if total_decision_num > agent.learning_start and total_decision_num % agent.update_model_freq == agent.update_model_freq - 1:
                    agent.replay()
                if total_decision_num > agent.learning_start and total_decision_num % agent.update_target_model_freq == agent.update_target_model_freq - 1:
                    agent.update_target_network()
            if all(dones):
                break
        if e % args.save_rate == args.save_rate - 1:
            if not os.path.exists(args.save_dir):
                os.makedirs(args.save_dir)
            for agent in agents:
                agent.save_model(args.save_dir)
        travel_time_record.append(env.metric[0].update(done=False))
        throughput_record.append(env.metric[1].update(done=False))
        logger.info("episode:{}/{}, average travel time:{}, throughput: {}".format(e, args.episodes, travel_time_record[-1], throughput_record[-1]))
        # for agent_id, agent in enumerate(agents):
        #    logger.info("agent:{}, mean_episode_reward:{}".format(agent_id, episodes_rewards[agent_id] / episodes_decision_num))
        
        
    
    dir_name = 'train_log/5-30/%s/flow_%s/dqn/0/' % (args.city, args.city)
    if not os.path.isdir(dir_name):
        os.makedirs(dir_name)
    record = {'TT': travel_time_record, 'throughput': throughput_record}
    json_str = json.dumps(record, indent=2)
    with open(dir_name + 's-a-r-t.json', 'w') as json_file:
        json_file.write(json_str)



if __name__ == '__main__':
    train(args, env)
