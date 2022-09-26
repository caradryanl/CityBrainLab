import logging
import numpy as np
import torch
import torch.nn.functional as F
from torch.optim import Adam
from .eGCN_model.model import eGCN, eActor, eCritic
from .utils import soft_update, hard_update


class eGCN_agent:
    def __init__(self, world):
        self.node_num = len(world.all_roads)
        self.edge_index = world.generate_edge_index()
        self.world = world
        self.lamda = 1
        self.policy_lr = 0.001
        self.critic_lr = 0.0005
        self.tau = 0.125
        self.egcn = eGCN()
        self.critic = eCritic(self.node_num)
        self.critic_optim = Adam(self.critic.parameters(), lr=self.critic_lr)
        self.critic_target = eCritic(self.node_num)
        self.policy = eActor(self.node_num)
        self.policy_optim = Adam(self.policy.parameters(), lr=self.policy_lr)
        self.last_vehicle = world.eng.get_vehicles()
        hard_update(self.critic_target, self.critic)
        
    def select_action(self, state, deterministic=False):
        state = torch.FloatTensor(state)
        action, _ = self.policy.sample(state, deterministic, with_logprob=False)
        return action.detach().numpy()
    
    def get_transition(self, state):
        state = self.egcn(state, self.edge_index) #shape: (intersection_num, 16)
        return state
    
    def get_rewards(self):
        current_vehicles = self.world.eng.get_vehicles()
        left_vehicle_num = len(set(self.last_vehicle) - set(current_vehicles))
        self.last_vehicle = current_vehicles
        return np.array([left_vehicle_num])
    
    def update_parameters(self, memory, batch_size):
        batch = memory.sample(batch_size=batch_size)
        state_batch = batch['state']
        next_state_batch = batch['next_state']
        action_batch = batch['action']
        done_batch = batch['done']
        reward_batch = batch['reward'] 

        state_batch = torch.FloatTensor(state_batch)
        next_state_batch = torch.FloatTensor(next_state_batch)
        action_batch = torch.FloatTensor(action_batch)
        reward_batch = torch.FloatTensor(reward_batch).view(-1, 1)
        mask_batch = 1 - torch.FloatTensor(done_batch).view(-1, 1)

        with torch.no_grad():
            next_state_action, next_state_log_pi = self.policy.sample(next_state_batch)
            next_state_action = torch.FloatTensor(next_state_action)
            '''
            qf_next_target, qf2_next_target = self.critic_target(
                next_state_batch, next_state_action
            )
            min_qf_next_target = (
                torch.min(qf_next_target, qf2_next_target)
                - self.alpha * next_state_log_pi
            )
            '''
            next_q_value = reward_batch + self.lamda

        qf = self.critic(
            state_batch, action_batch
        )  # Two Q-functions to mitigate positive bias in the policy improvement step
        qf_loss = F.mse_loss(
            qf, next_q_value
        )  # JQ = 𝔼(st,at)~D[0.5(Q1(st,at) - r(st,at) - γ(𝔼st+1~p[V(st+1)]))^2]
        

        self.critic_optim.zero_grad()
        qf_loss.backward(retain_graph=True)
        self.critic_optim.step()


        pi, log_pi = self.policy.sample(state_batch)
        pi = torch.FloatTensor(pi)

        qf_pi = self.critic(state_batch, pi)


        policy_loss = (
            log_pi * qf_pi
        ).mean()  # Jπ = 𝔼st∼D,εt∼N[α * logπ(f(εt;st)|st) − Q(st,f(εt;st))]

        self.policy_optim.zero_grad()
        policy_loss.backward(retain_graph=True)
        self.policy_optim.step()

        # if updates % self.target_update_interval == 0:
        soft_update(self.critic_target, self.critic, self.tau)

        return qf_loss.item(), policy_loss.item()
