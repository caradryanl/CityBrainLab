import logging
import numpy as np
import torch
import torch.nn.functional as F
from torch.optim import Adam
from .model import ActorNet, AgentQNet
from .utils import soft_update, hard_update


class BaseAC:
    def __init__(self, action_space, args):
        self.action_space = action_space
        self.gamma = args.gamma
        self.alpha = args.alpha
        self.tau = args.tau
        self.state_dim = 1
        self.action_dim = 1
        self.critic = AgentQNet(self.state_dim, self.action_dim)
        self.critic_optim = Adam(self.critic.parameters(), lr=args.lr)
        self.critic_target = AgentQNet(self.state_dim, self.action_dim)
        self.policy = ActorNet(self.state_dim)
        self.policy_optim = Adam(self.policy.parameters(), lr=args.lr)
        hard_update(self.critic_target, self.critic)
        
    def select_action(self, state, deterministic=False):
        state = torch.FloatTensor(state)
        action, _ = self.policy.sample(state, deterministic, with_logprob=False)
        return action.detach().numpy()
    
    
    def update_parameters(self, memory, batch_size):
        batch = memory.sample(batch_size=batch_size)
        state_batch = batch['state']
        next_state_batch = batch['next_state']
        action_batch = batch['action']
        done_batch = batch['done']
        reward_batch = batch['reward'] 

        state_batch = torch.FloatTensor(state_batch).view(-1, self.state_dim)
        next_state_batch = torch.FloatTensor(next_state_batch).view(-1, self.state_dim)
        action_batch = torch.FloatTensor(action_batch).view(-1, self.action_dim)
        reward_batch = torch.FloatTensor(reward_batch).view(-1, 1)
        mask_batch = 1 - torch.FloatTensor(done_batch).view(-1, 1)

        with torch.no_grad():
            next_state_action, next_state_log_pi = self.policy.sample(next_state_batch)
            next_state_action = torch.FloatTensor(next_state_action)
            qf1_next_target, qf2_next_target = self.critic_target(
                next_state_batch, next_state_action
            )
            min_qf_next_target = (
                torch.min(qf1_next_target, qf2_next_target)
                - self.alpha * next_state_log_pi
            )
            next_q_value = reward_batch + self.gamma * mask_batch * (min_qf_next_target)

        qf1, qf2 = self.critic(
            state_batch, action_batch
        )  # Two Q-functions to mitigate positive bias in the policy improvement step
        qf1_loss = F.mse_loss(
            qf1, next_q_value
        )  # JQ = 𝔼(st,at)~D[0.5(Q1(st,at) - r(st,at) - γ(𝔼st+1~p[V(st+1)]))^2]
        qf2_loss = F.mse_loss(
            qf2, next_q_value
        )  # JQ = 𝔼(st,at)~D[0.5(Q1(st,at) - r(st,at) - γ(𝔼st+1~p[V(st+1)]))^2]
        qf_loss = qf1_loss + qf2_loss

        self.critic_optim.zero_grad()
        qf_loss.backward()
        self.critic_optim.step()


        pi, log_pi = self.policy.sample(state_batch)
        pi = torch.FloatTensor(pi)

        qf1_pi, qf2_pi = self.critic(state_batch, pi)
        min_qf_pi = torch.min(qf1_pi, qf2_pi)

        policy_loss = (
            (self.alpha * log_pi) - min_qf_pi
        ).mean()  # Jπ = 𝔼st∼D,εt∼N[α * logπ(f(εt;st)|st) − Q(st,f(εt;st))]

        self.policy_optim.zero_grad()
        policy_loss.backward()
        self.policy_optim.step()

        # if updates % self.target_update_interval == 0:
        soft_update(self.critic_target, self.critic, self.tau)

        return qf_loss.item(), policy_loss.item()

    def update_from_datasample(self, datasample):
        state_batch = torch.FloatTensor(datasample['state'])
        next_state_batch = torch.FloatTensor(datasample['next_state'])
        action_batch = torch.FloatTensor(datasample['action'])
        reward_batch = torch.FloatTensor(datasample['reward'])

        with torch.no_grad():
            next_state_action, next_state_log_pi = self.policy.sample(next_state_batch)
            next_state_action = torch.FloatTensor(next_state_action)
            qf1_next_target, qf2_next_target = self.critic_target(
                next_state_batch, next_state_action
            )
            min_qf_next_target = (
                torch.min(qf1_next_target, qf2_next_target)
                - self.alpha * next_state_log_pi
            )
            next_q_value = reward_batch + self.gamma * (min_qf_next_target)

        qf1, qf2 = self.critic(
            state_batch, action_batch
        )  # Two Q-functions to mitigate positive bias in the policy improvement step

        # logging.debug(qf1, "    |   ", qf2)
        qf1_loss = F.mse_loss(
            qf1, next_q_value
        )  # JQ = 𝔼(st,at)~D[0.5(Q1(st,at) - r(st,at) - γ(𝔼st+1~p[V(st+1)]))^2]
        qf2_loss = F.mse_loss(
            qf2, next_q_value
        )  # JQ = 𝔼(st,at)~D[0.5(Q1(st,at) - r(st,at) - γ(𝔼st+1~p[V(st+1)]))^2]
        qf_loss = qf1_loss + qf2_loss

        self.critic_optim.zero_grad()
        qf_loss.backward()
        self.critic_optim.step()


        pi, log_pi = self.policy.sample(state_batch)
        pi = torch.FloatTensor(pi)

        qf1_pi, qf2_pi = self.critic(state_batch, pi)
        min_qf_pi = torch.min(qf1_pi, qf2_pi)

        policy_loss = (
            (self.alpha * log_pi) - min_qf_pi
        ).mean()  # Jπ = 𝔼st∼D,εt∼N[α * logπ(f(εt;st)|st) − Q(st,f(εt;st))]

        self.policy_optim.zero_grad()
        policy_loss.backward()
        self.policy_optim.step()

        soft_update(self.critic_target, self.critic, self.tau)

        return qf_loss.item(), policy_loss.item() 
