from time import time
import gym
import numpy as np
from agent.human_eco_agent import HumanEcoAgent
from agent.fixedtime_agent import Fixedtime_Agent

# from agent.stochatic_agent import StochasticAgent

# from agent.random_agent import RandomAgent


class VCTEnv(gym.Env):
    """
        Environment for Vehicle.

        Parameters
        ----------
        world: World object
        agents: list of agent, corresponding to three different types of agents
        metric: Metric object, used to calculate evaluation metric
    """

    def __init__(self, world, agents, metric, args):
        self.world = world

        self.eng = self.world.eng

        self.args = args

        self.dic_env = {}

        self.metric = metric

        self.dic_env["vehicle"] = VehiclEnv(self.world, agents["vehicle"], metric)

        self.dic_env["cp"] = CPEnv(self.world, agents["cp"], metric)

        self.dic_env["tsc"] = TSCEnv(self.world, agents["tsc"], metric)

        # todo -  assert len(agents) == self.n_agents

        self.dic_agents = agents
        # self.action_space = {key: env.action_space for key, env in self.dic_env.items()}
        self.metric = metric

    def update_metric(self, metric):
        self.metric = metric

    def step(self, actions):
        # todo - assert len(actions) == self.n_agents
        # time_1 = time()
        self.world.step(actions)
        self.world.update_toll()
        # time_2 = time()
        obs, reward, done, detail = self.dic_env["cp"].get_obs_reward_done_info()
        # time_3 = time()
        # equal to update_vehicles(self)
        if "vehicle" in self.dic_agents:
            self.dic_agents["vehicle"], info = self.dic_env["vehicle"].update_agents()
        # time_4 = time()
        # print("world.step: ", time_2 - time_1)
        # print("get_obs_reward_done_info: ", time_3 - time_2)
        # print("update_agents: ", time_4 - time_3)
        return obs, reward, done, info, detail

    def reset(self):
        self.world.reset()
        for ind_m in range(len(self.metric)):
            self.metric[ind_m].reset()
        obs = self.dic_env["cp"].reset()
        self.dic_env["vehicle"].reset()
        self.dic_env["tsc"].reset()
        if "vehicle" in self.dic_agents:
            self.dic_agents["vehicle"], _ = self.dic_env["vehicle"].update_agents()
        return obs

    def update_vehicles(self):
        return self.dic_env["vehicle"].update_agents()


class VehiclEnv(gym.Env):
    """
            Environment for Vehicle.

            Parameters
            ----------
            world: World object
            agents: list of agent, corresponding to each vehicle in world.vehicles
            metric: Metric object, used to calculate evaluation metric
        """

    def __init__(self, world, agents, metric):
        self.world = world

        self.eng = self.world.eng
        self.n_agents = len(self.world.vehicles)
        self.n = self.n_agents

        # self.init_vehicle_agent = init_vehicle_agent

        assert len(agents) == self.n_agents

        self.agents = agents
        self.vehicle_ids = self.world.vehicle_ids.copy()
        self.id2vehicle = self.world.id2vehicle.copy()
        # action_dims = [agent.action_space.n for agent in agents]
        # self.action_space = gym.spaces.MultiDiscrete(action_dims)
        #
        # self.metric = metric

    def update_agents(self):

        new_vehicle_ids = self.world.eng.get_vehicles()
        old_vehicle_ids = self.vehicle_ids
        new_entered_vehicle_ids = set(new_vehicle_ids) - set(old_vehicle_ids)
        new_left_vehicle_ids = set(old_vehicle_ids) - set(new_vehicle_ids)

        # update vehicle in_system
        for vec_id in new_left_vehicle_ids:
            self.agents[self.id2vehicle[vec_id]] = None

        # add new vehicles
        for vec_id in new_entered_vehicle_ids:
            self.id2vehicle[vec_id] = len(self.vehicle_ids)
            self.vehicle_ids.append(vec_id)
            self.agents.append(
                HumanEcoAgent(self.world.vehicles[self.id2vehicle[vec_id]], self.world)
            )

        return self.agents, new_entered_vehicle_ids

    def get_obs_reward_done_info(self):

        obs = [agent.get_ob() for agent in self.agents if agent != None]
        rewards = [agent.get_reward() for agent in self.agents if agent != None]
        dones = [False] * self.n_agents
        # infos = {"metric": self.metric.update()}
        infos = {}

        # update agents

        return obs, rewards, dones, infos

    def reset(self):
        self.agents = []
        self.vehicle_ids = self.world.vehicle_ids.copy()
        self.id2vehicle = self.world.id2vehicle.copy()
        self.n_agents = len(self.world.vehicles)
        assert len(self.agents) == self.n_agents
        # self.init_vehicle_agent = init_vehicle_agent

        obs = [agent.get_ob() for agent in self.agents]
        return obs


class CPEnv(gym.Env):
    """
        Environment for Congestion Pricing.

        Parameters
        ----------
        world: World object
        agents: list of agent, corresponding to each road in world.all_roads
        metric: Metric object, used to calculate evaluation metric
    """

    def __init__(self, world, agents, metric):
        self.world = world

        self.eng = self.world.eng
        # self.n_agents = len(self.world.all_roads)
        # self.n = self.n_agents

        # assert len(agents) == self.n_agents

        self.agents = agents

        self.metric = metric

    def get_obs_reward_done_info(self):
        rewards = []
        obs = []
        # state = self.world.get_state()
        lane_distance, detail = self.world.get_lane_distance()
        # passvehicle_cnt = self.get_passvehicle_cnt()
        # obs = [state[id] for id in self.world.all_lane_ids]
        rewards = [lane_distance[id] for id in self.world.all_lane_ids]
        rewards = np.stack(rewards, axis=0)
        obs = np.zeros((len(rewards), 1))
        dones = np.zeros((len(rewards), 1))
        infos = detail

        # 最后返回obs以图的形式

        return obs, rewards, dones, infos
    
    def get_state(self):
        state = self.world.get_delta_toll()
        obs = [state[id] for id in self.world.all_lane_ids]
        obs = np.stack(obs, axis=0)
        return obs

    def get_passvehicle_cnt(self):
        '''
        passvehicle_cnt = {}
        for lane in self.world.all_lanes:
            flow_at_start = lane.last_vehicle
            
            lane.update_lane_vehicle()
            
            flow_at_end = lane.last_vehicle
            vehicle_pass = len(list(set(flow_at_start).union(set(flow_at_end))))
            passvehicle_cnt[lane.id] = vehicle_pass
        '''
        last_vehicles = self.world.last_lane_vehicles
        current_vehicles = self.world.eng.get_lane_vehicles()
        vehicle_pass = {key: len((set(last_vehicles[key]).union(set(current_vehicles[key])))) for key in last_vehicles}
        self.world.last_lane_vehicles = current_vehicles

        return vehicle_pass

    def reset(self):
        obs = []
        '''
        road_vehicle_count = self.world.get_road_vehicle_count()
        for road in self.world.all_roads:
            id = road.id
            obs.append([road_vehicle_count[id]])
        obs = np.stack(obs, axis=0)
        '''
        obs = [np.zeros(1) for i in range(len(self.world.all_lanes))]
        obs = np.stack(obs, axis=0)
        return obs
   

class TSCEnv(gym.Env):
    """
    Environment for Traffic Signal Control task.

    Parameters
    ----------
    world: World object
    agents: list of agent, corresponding to each intersection in world.intersections
    metric: Metric object, used to calculate evaluation metric
    """

    def __init__(self, world, agents, metric):
        self.world = world

        self.eng = self.world.eng
        self.n_agents = len(self.world.intersection_ids)
        self.n = self.n_agents

        assert len(agents) == self.n_agents

        self.agents = agents

        self.metric = metric

    def get_obs_reward_done_info(self):
        obs = [agent.get_ob() for agent in self.agents]
        rewards = [agent.get_reward() for agent in self.agents]
        dones = [False] * self.n_agents
        # infos = {"metric": self.metric.update()}
        infos = {}

        return obs, rewards, dones, infos

    def reset(self):
        obs = [agent.get_ob() for agent in self.agents]
        for agent in self.agents:
            if isinstance(agent, Fixedtime_Agent):
                agent.last_action_time = 0
                agent.last_action = 1
        return obs
