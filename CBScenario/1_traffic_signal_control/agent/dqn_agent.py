from agent.rl_agent import RLAgent
import random
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from collections import deque
import torch.optim as optim
import os

class DQNAgent(RLAgent):
    def __init__(self, action_space, ob_generator, reward_generator, iid):
        super().__init__(action_space, ob_generator, reward_generator)

        self.iid = iid
        self.action_space = action_space
        self.phase_id = 8
        self.ob_length = ob_generator.ob_length

        self.memory = deque(maxlen=5000)
        self.learning_start = 2000
        self.update_model_freq = 3
        self.update_target_model_freq = 60

        self.gamma = 0.95  # discount rate
        self.epsilon = 0.1  # exploration rate
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        self.learning_rate = 1e-4
        self.batch_size = 32

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = self._build_model().to(self.device)
        self.target_model = self._build_model().to(self.device)
        self.update_target_network()

        self.optimizer = optim.Adam(self.model.parameters(), lr=self.learning_rate)

    def get_action(self, ob):
        if np.random.rand() <= self.epsilon:
            return self.action_space.sample()
        ob = self._reshape_ob(ob)
        ob = torch.Tensor(ob)
        act_values = self.model(ob.to(self.device)).to("cpu")
        return torch.argmax(act_values[0]).item()

    def sample(self):
        return self.action_space.sample()

    def _build_model(self):
        # Neural Net for Deep-Q learning Model
        layers = []
        layers.append(nn.Linear(in_features=self.ob_length, out_features=1024))
        layers.append(nn.ReLU())
        layers.append(nn.Linear(in_features=1024, out_features=1024))
        layers.append(nn.ReLU())
        layers.append(nn.Linear(in_features=1024, out_features=self.phase_id))
        model = nn.Sequential(*layers)
        return model

    def _reshape_ob(self, ob):
        return np.reshape(ob, (1, -1))

    def update_target_network(self):
        weights = self.model.state_dict()
        self.target_model.load_state_dict(weights)

    def remember(self, ob, action, reward, next_ob):
        self.memory.append((ob, action, reward, next_ob))

    def replay(self):
        minibatch = random.sample(self.memory, self.batch_size)
        obs, actions, rewards, next_obs = [np.stack(x) for x in np.array(minibatch, dtype=object).T]
        obs, actions, rewards, next_obs = torch.Tensor(obs), torch.LongTensor(actions), torch.Tensor(rewards), torch.Tensor(next_obs)
        # print(obs.shape, actions.shape, rewards.shape, next_obs.shape)
        obs, actions, rewards, next_obs = obs.to(self.device), actions.to(self.device), rewards.to(self.device), next_obs.to(self.device)
        target = rewards + torch.max(self.target_model(next_obs), axis=1)[0] * self.gamma
        target_f = self.model(obs)
        # print(target_f.shape)
        target_f = torch.gather(target_f, 1, actions.unsqueeze(-1)).squeeze(-1)
        
        self.optimizer.zero_grad()
        loss = F.mse_loss(target_f, target)
        loss.backward()
        self.optimizer.step()

        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def load_model(self, dir="model/dqn"):
        name = "dqn_agent_{}.pth.tar".format(self.iid)
        model_name = os.path.join(dir, name)
        self.model.load_state_dict(torch.load(model_name))

    def save_model(self, dir="model/dqn"):
        name = "dqn_agent_{}.pth.tar".format(self.iid)
        model_name = os.path.join(dir, name)
        torch.save(self.model.state_dict(), model_name)
