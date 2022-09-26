import copy
import numpy as np


class BaseAgent(object):
    def __init__(self, action_space):
        self.action_space = action_space
        self.iid = None

    def init_from_copy(self, iid):
        new_agent = copy.deepcopy(self)
        new_agent.iid = iid
        return new_agent

    def get_ob(self):
        raise NotImplementedError()

    def get_reward(self):
        raise NotImplementedError()

    # def get_action(self):
    #     return np.array([0.01])

    def get_action(self):
        # manual
        return self.action_space.sample() 
