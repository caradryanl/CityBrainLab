import numpy as np
from typing import Dict

import torch


class AgentReplayMemory:
    def __init__(
        self,
        capacity,
        agent_num: int,
        keys: Dict = ["state", "action", "reward", "next_state", "done"],
        state_dim=1,
        action_dim=1,
        attention = False,
        state_num = None
    ):
        # random.seed(seed)
        self.capacity = capacity
        self.keys = keys
        self.agent_num = agent_num
        self.buffer = {key: np.zeros((capacity, agent_num, 1)) for key in keys}
        if attention and state_num == None:
            self.buffer['state'] = torch.zeros((capacity, agent_num, state_dim))
            self.buffer['next_state'] = torch.zeros((capacity, agent_num, state_dim)) 
        elif attention and state_num != None:
            self.buffer['state'] = torch.zeros((capacity, state_num, state_dim))
            self.buffer['next_state'] = torch.zeros((capacity, state_num, state_dim))
        else:
            self.buffer['state'] = np.zeros((capacity, agent_num, state_dim))
            self.buffer['next_state'] = np.zeros((capacity, agent_num, state_dim)) 
        self.buffer['action'] = np.zeros((capacity, agent_num, action_dim))  
        self._pos = 0 
        self._size = 0

    # def push(self, state, action, reward, next_state, done):
    def push(self, **kwargs):
        if self._size < self.capacity:
            self._size += 1
        for k, v in kwargs.items():
            if k not in self.keys:
                raise KeyError(k)
            self.buffer[k][self._pos] = v
        self._pos = (self._pos + 1) % self.capacity

    def sample(self, batch_size, keys=None, idx=False):
        # batch = random.sample(self.buffer, batch_size)
        idxes = np.random.choice(self._size, batch_size, replace=False)
        batch = {}
        if keys is None:
            keys = self.keys
        for k in keys:
            batch[k] = self.buffer[k][idxes]
        if idx:
            return batch, idxes
        else:
            return batch

    def __len__(self):
        # return len(self.buffer)
        return self._size
