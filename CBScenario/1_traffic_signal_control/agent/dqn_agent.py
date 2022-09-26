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
        self.learning_start = 200000
        self.update_model_freq = 1
        self.update_target_model_freq = 20

        self.gamma = 0.95  # discount rate
        self.epsilon = 0.9  # exploration rate
        self.epsilon_min = 0.2
        self.epsilon_decay = 0.995
        self.learning_rate = 0.005
        self.batch_size = 64

        self.model = self._build_model()
        self.target_model = self._build_model()
        self.update_target_network()

        self.optimizer = optim.RMSprop(self.model.parameters(),
                    lr=0.001,
                    alpha=0.99,
                    eps=1e-08,
                    weight_decay=0,
                    momentum=0,
                    centered=False)
        self.scheduler = optim.lr_scheduler.MultiStepLR(
            self.optimizer, [100000, 500000], gamma=0.1
        )

    def get_action(self, ob):
        if np.random.rand() <= self.epsilon:
            return self.action_space.sample()
        ob = self._reshape_ob(ob)
        ob = torch.Tensor(ob)
        act_values = self.model(ob)
        return torch.argmax(act_values[0]).item()

    def sample(self):
        return self.action_space.sample()

    def _build_model(self):
        # Neural Net for Deep-Q learning Model
        layers = []
        layers.append(nn.Linear(in_features=self.ob_length, out_features=20))
        layers.append(nn.ReLU())
        #model.add(Dense(20, activation='relu'))
        layers.append(nn.Linear(in_features=20, out_features=self.phase_id))
        layers.append(nn.Sigmoid())
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
        obs, actions, rewards, next_obs = [np.stack(x) for x in np.array(minibatch).T]
        obs, actions, rewards, next_obs = torch.Tensor(obs), torch.LongTensor(actions), torch.Tensor(rewards), torch.Tensor(next_obs)
        # print(obs.shape, actions.shape, rewards.shape, next_obs.shape)
        target = rewards + torch.max(self.target_model(next_obs), axis=1)[0] * self.gamma
        target_f = self.model(obs)
        # print(target_f.shape)
        target_f = torch.gather(target_f, 1, actions.unsqueeze(-1)).squeeze(-1)
        
        self.optimizer.zero_grad()
        loss = F.smooth_l1_loss(target, target_f)
        loss.backward()
        self.optimizer.step()
        self.scheduler.step()

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
