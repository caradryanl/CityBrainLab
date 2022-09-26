from time import time
import gym
import numpy as np
from agent.fixedtime_agent import Fixedtime_Agent
   

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

    def step(self, actions):
        self.world.step(actions)
        obs, rewards, dones, infos = self.get_obs_reward_done_info()
        return obs, rewards, dones, infos


    def reset(self):
        self.world.reset()
        for ind_m in range(len(self.metric)):
            self.metric[ind_m].reset()
        obs = [agent.get_ob() for agent in self.agents]
        return obs
