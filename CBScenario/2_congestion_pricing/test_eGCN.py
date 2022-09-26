import argparse
import gym

gym.logger.set_level(40)
import numpy as np
import torch
from actor_critic.eGCN_agent import eGCN_agent
from actor_critic.replay_memory import AgentReplayMemory
from environment import VCTEnv
from agent.fixedtime_agent import Fixedtime_Agent
from agent.human_eco_agent import HumanEcoAgent
from world import World
from metric import TravelTimeMetric, ThroughputMetric, FuelMetric, TotalCostMetric
from torch.utils.tensorboard import SummaryWriter
import os
import json

def get_focus_id(route_list, world):
    focus_lane = []
    for route in route_list:
        lane_list = world.get_lane_route(route)
        focus_lane.extend(lane_list)
    focus_id = [world.id2lane[lane] for lane in set(focus_lane)]
    return focus_id

parser = argparse.ArgumentParser(description="Base Actor-Critic Args")

parser.add_argument(
    "--batch_size", type=int, default=32, metavar="N", help="batch size (default: 4)"
)
parser.add_argument(
    "--start_episodes", type=int, default=20, metavar="N", help="random sample before"
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
    default=2000,
    metavar="N",
    help="size of replay buffer (default: 2000)",
)
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
    "--action_interval",
    type=int,
    default=20,
    help="how often agent make decisions (default: 120)",
)
parser.add_argument("--date", type=str, default='test',help="date of running")
parser.add_argument(
    "--config_file", type=str, default="./dataset/manhattan/config_manhattan.json", help="path of config file")
parser.add_argument(
    "--engine_config_file", type=str, default="./dataset/manhattan/manhattan.cfg", help="path of config file")
parser.add_argument(
    "--weight_file", type=str, default="./model/test/manhattan/eGCN/", help="path of weight file")
args = parser.parse_args()
args = parser.parse_args()
config = json.load(open(args.config_file, 'r'))
net = config['dir'].split('/')[1]
flow = config["flowFile"].split('.')[0]
netandflow = net + flow
date = args.date
writer = SummaryWriter('tensorboard/%s/eGCN/%s/%s/%s' % (date, net, flow, args.batch_size))

device = torch.device('cuda:0' if torch.cuda.is_available() else "cpu")
world = World(args.engine_config_file, args.config_file, thread_num=args.thread, args=args)

dic_agents = {}

# tsc agents
agents = []
for i in world.intersections:
    action_space = gym.spaces.Discrete(4)
    agents.append(Fixedtime_Agent(action_space, i.id))
dic_agents["tsc"] = agents


memory = AgentReplayMemory(args.replay_size, agent_num=1, state_dim=16, action_dim=len(world.all_roads), attention=True, state_num = len(world.all_roads))
# cp agents
agents = []
action_space = gym.spaces.Box(np.array([-1]), np.array([1]))
dic_agents["cp"] = eGCN_agent(world)
dic_agents["cp"].egcn.load_state_dict(torch.load(args.weight_file+"egcn.pth"))
dic_agents["cp"].policy.load_state_dict(torch.load(args.weight_file+"policy.pth"))
dic_agents["cp"].critic.load_state_dict(torch.load(args.weight_file+"critic.pth"))

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


# create env
env = VCTEnv(world, dic_agents, metric, args)

def test(args, metric_name):
    loss = 0
    detail = {}
    raw_state = []
    raw_next_state = []
    detail[0] = {}
    interval_reward_record = []
    state_record = []
    action_record = []
    reward_record = []
    episode_reward = 0
    travel_time_record = []
    throughput_record = []
    done = False
    env.reset()  # 仅关心road的state
    state = dic_agents["cp"].get_transition(world.get_obs())
    print('eGCN', " |episodes is : ", 0)
    reward_list = []
    dic_actions = {}  # e < 30 set_seed
    # pre_pass_distance = np.zeros((len(world.all_lanes), 1))

    for i in range(args.steps):
        print('eGCN', "|", i, "/", args.steps, " |episodes is : ", 0)
        key = "cp"
        dic_actions[key] = dic_agents["cp"].select_action(state)
        for t in range(args.action_interval):
            # traffic light take action every second
            key = "tsc"
            dic_actions[key] = []
            for id, agent in enumerate(dic_agents[key]):
                dic_actions[key].append(agent.get_action(world))
            
            next_state, reward, done, info, vehicle = env.step(dic_actions)
            dic_actions["vehicle"] = {}
            for id, agent in enumerate(dic_agents["vehicle"]):
                if agent is not None and agent.vehicle.id in info and agent.vehicle.monitor:
                    dic_actions["vehicle"][agent.vehicle.id] = agent.get_action(world)
                elif agent is None:
                    pass
                else:
                    dic_actions["vehicle"][agent.vehicle.id] = []
            reward_list.append(reward)
            for ind_m in range(len(env.metric)):
                env.metric[ind_m].update(done=False)

        next_state = dic_agents["cp"].get_transition(world.get_obs())
        state = next_state
        # pre_pass_distance = pass_distance
        
        dir_name = 'test_log/eGCN/'
        if not os.path.isdir(dir_name):
            os.makedirs(dir_name)
        travel_time_record = np.array(travel_time_record)
        TT_detail = env.metric[0].update(done=True)
        record = {'TT': travel_time_record.tolist(), 'throughput': throughput_record}
        json_str = json.dumps(record, indent=2)
        with open(dir_name + 's-a-r-t.json', 'w') as json_file:
            json_file.write(json_str)
        TT_str = json.dumps(TT_detail, indent=2)
        with open(dir_name + 'TT_detail.json', 'w') as json_file:
            json_file.write(TT_str)
        reroute = json.dumps(world.vehicle_route, indent=2)
        with open(dir_name + 'reroute.json', 'w') as json_file:
            json_file.write(reroute)
        # if e == 1 or e == 199:
        #     vehicle_pass = json.dumps(detail, indent=2)
        #     with open(dir_name + 'vehicle_pass.json', 'w') as json_file:
        #         json_file.write(vehicle_pass)
        
        reward_json = {}
        reward_json['interval_reward'] = interval_reward_record
        reward_str = json.dumps(reward_json, indent=2)
        with open(dir_name + 'interval_reward.json', 'w') as json_file:
            json_file.write(reward_str)
        




if __name__ == "__main__":
    test(args, metric_name)
