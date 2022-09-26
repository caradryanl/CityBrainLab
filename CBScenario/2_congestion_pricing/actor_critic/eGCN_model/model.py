from turtle import forward
from pyparsing import alphas
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch_geometric.nn import GCNConv
from torch.distributions import Normal
import math
import numpy as np


LOG_STD_MAX = 2
LOG_STD_MIN = -20
epsilon = 1e-6

class DoubleFC(torch.nn.Module):
    def __init__(self, input_dim, out_features):
        super(DoubleFC, self).__init__()
        self.linear1 = nn.Linear(input_dim, 64)
        self.linear2 = nn.Linear(64, out_features)

    def forward(self, x):
        x = self.linear1(x)
        x = F.relu(x)
        x = self.linear2(x)
        return x

class Attention(torch.nn.Module):
    def __init__(self, in_features, out_features):
        super(Attention, self).__init__()
        self.query_key = nn.Linear(in_features, out_features)
        self.value = nn.Linear(in_features, out_features)
        self._out_features = math.sqrt(out_features)
    
    def forward(self, embedding):
        # embedding shape: (n, 2k)
        Q = self.query_key(embedding)
        K = self.query_key(embedding)
        V = self.value(embedding)
        Z = torch.mm(F.softmax(torch.mm(Q, K.t()) / self._out_features, dim=1), V)
        return Z

class AttentionQ(torch.nn.Module):
    def __init__(self, input_dim, out_features):
        super(AttentionQ, self).__init__()
        # Q1 architecture
        self.FC1 = DoubleFC(input_dim, out_features)
        self.FC2 = DoubleFC(input_dim, out_features)
        self.FC3 = DoubleFC(input_dim, out_features)
        self.FC4 = DoubleFC(3 * out_features, 32)
        self.Qnet = Attention(32, 1)
        # Q2 architecture
        self.FCa = DoubleFC(input_dim, out_features)
        self.FCb = DoubleFC(input_dim, out_features)
        self.FCc = DoubleFC(input_dim, out_features)
        self.FCd = DoubleFC(3 * out_features, 32)
        self._Qnet = Attention(32, 1)

    def forward(self, sa1, sa2, sa3):
        _out1 = self.FC1(sa1)
        _out2 = self.FC2(sa2)
        _out3 = self.FC3(sa3)
        embed = torch.cat([_out1, _out2, _out3], axis=1)
        embed = self.FC4(embed)
        q = self.Qnet(embed)

        _outa = self.FCa(sa1)
        _outb = self.FCb(sa2)
        _outc = self.FCc(sa3)
        _embed = torch.cat([_outa, _outb, _outc], axis=1)
        _embed = self.FCd(_embed)
        _q = self._Qnet(_embed)
        return q, _q

class OD_policy(torch.nn.Module):
    def __init__(self, input_dim):
        super(OD_policy, self).__init__()
        self.input_dim = input_dim
        self.linear1 = nn.Linear(input_dim, 32)
        self.linear2 = nn.Linear(32, 32)
        self.linear3 = nn.Linear(32, 3)
        self.std_linear = nn.Linear(32, 3)

    def forward(self, state_list):
        x = state_list.view(-1, self.input_dim)
        x = F.sigmoid(self.linear1(x))
        x = F.sigmoid(self.linear2(x))
        mu = self.linear3(x)
        std = self.std_linear(x)
        return mu, std

    def sample(self, state, deterministic=False, with_logprob=True):
        # refer to Github
        mu, log_std = self.forward(state)
        log_std = torch.clamp(log_std, LOG_STD_MIN, LOG_STD_MAX)
        std = torch.exp(log_std)
        normal = Normal(mu, std)
        if deterministic:
            # Only used for evaluating policy at test time.
            pi_action = mu
        else:
            pi_action = normal.rsample()
        # for reparameterization trick (mean + std * N(0,1))
        if with_logprob:
            logp_pi = normal.log_prob(pi_action).sum(axis=-1)
            logp_pi -= (2 * (np.log(2) - pi_action - F.softplus(-2 * pi_action))).sum(axis=1)
        else:
            logp_pi = None
        pi_action = torch.tanh(pi_action)
        return pi_action, logp_pi    

class OD_Q(torch.nn.Module):
    def __init__(self, input_dim, out_features):
        super(OD_Q, self).__init__()
        # Q1 architecture
        self.FC1 = DoubleFC(input_dim, out_features)
        self.FC2 = DoubleFC(input_dim, out_features)
        self.FC3 = DoubleFC(input_dim, out_features)
        self.FC4 = DoubleFC(3 * out_features, 32)
        self.Qnet = DoubleFC(32, 1)
        # Q2 architecture
        self.FCa = DoubleFC(input_dim, out_features)
        self.FCb = DoubleFC(input_dim, out_features)
        self.FCc = DoubleFC(input_dim, out_features)
        self.FCd = DoubleFC(3 * out_features, 32)
        self._Qnet = DoubleFC(32, 1)

    def forward(self, sa1, sa2, sa3):
        _out1 = self.FC1(sa1)
        _out2 = self.FC2(sa2)
        _out3 = self.FC3(sa3)
        embed = torch.cat([_out1, _out2, _out3], axis=1)
        embed = self.FC4(embed)
        q = self.Qnet(embed)

        _outa = self.FCa(sa1)
        _outb = self.FCb(sa2)
        _outc = self.FCc(sa3)
        _embed = torch.cat([_outa, _outb, _outc], axis=1)
        _embed = self.FCd(_embed)
        _q = self._Qnet(_embed)
        return q, _q

class SACQ(torch.nn.Module):
    def __init__(self, state_dim=2, action_dim=1):
        super(SACQ, self).__init__()
        # Q1 architecture
        self.linear1 = nn.Linear(state_dim, 128)
        self.linear2 = nn.Linear(action_dim, 128)
        self.linear3 = nn.Linear(256, 1)

        # Q2 architecture
        self.linear4 = nn.Linear(state_dim, 128)
        self.linear5 = nn.Linear(action_dim, 128)
        self.linear6 = nn.Linear(256, 1)

    def forward(self, state, action):
        x_s1 = F.relu(self.linear1(state))
        x_a1 = F.relu(self.linear2(action))
        x = torch.cat([x_s1, x_a1], axis=1)
        x1 = self.linear3(x)

        x_s2 = F.relu(self.linear4(state))
        x_a2 = F.relu(self.linear5(action))
        x = torch.cat([x_s2, x_a2], axis=1)
        x2 = self.linear6(x)
        return x1, x2


class SACP(torch.nn.Module):
    def __init__(self, state_dim, action_dim):
        super(SACP, self).__init__()
        self.linear1 = nn.Linear(state_dim, 256)
        self.linear2 = nn.Linear(256, 256)
        self.linear3 = nn.Linear(256, action_dim)
        self.std_linear = nn.Linear(256, action_dim)

    def forward(self, x):
        x = F.relu(self.linear1(x))
        x = F.relu(self.linear2(x))
        mu = self.linear3(x)
        std = self.std_linear(x)
        # std = torch.clamp(std, min=LOG_SIG_MIN, max=LOG_SIG_MAX)
        return mu, std

    def sample(self, state, deterministic=False, with_logprob=True):
        # refer to Github
        mu, log_std = self.forward(state)
        log_std = torch.clamp(log_std, LOG_STD_MIN, LOG_STD_MAX)
        std = torch.exp(log_std)
        normal = Normal(mu, std)
        if deterministic:
            # Only used for evaluating policy at test time.
            pi_action = mu
        else:
            pi_action = normal.rsample()
        # for reparameterization trick (mean + std * N(0,1))
        if with_logprob:
            logp_pi = normal.log_prob(pi_action).sum(axis=-1)
            logp_pi -= (2 * (np.log(2) - pi_action - F.softplus(-2 * pi_action))).sum(axis=1)
            logp_pi = logp_pi.view(-1, 1)
        else:
            logp_pi = None
        pi_action = torch.tanh(pi_action)
        return pi_action, logp_pi


class AgentQNet(torch.nn.Module):
    def __init__(self, state_dim, action_dim=1):
        super(AgentQNet, self).__init__()
        # # Q1 architecture
        # self.linear1 = nn.Linear(state_dim, 64)
        # self.linear2 = nn.Linear(action_dim, 16)
        # self.linear3 = nn.Linear(80, 32)
        # self.lineara = nn.Linear(32, 32)
        # self.linear4 = nn.Linear(32, 1)

        # # Q2 architecture
        # self.linear5 = nn.Linear(state_dim, 64)
        # self.linear6 = nn.Linear(action_dim, 16)
        # self.linear7 = nn.Linear(80, 32)
        # self.linearb = nn.Linear(32, 32)
        # self.linear8 = nn.Linear(32, 1)

        # Q1 architecture
        self.linear1 = nn.Linear(state_dim, 16)
        self.linear2 = nn.Linear(action_dim, 16)
        self.linear3 = nn.Linear(32, 16)
        self.linear4 = nn.Linear(16, 1)

        # Q2 architecture
        self.linear5 = nn.Linear(state_dim, 16)
        self.linear6 = nn.Linear(action_dim, 16)
        self.linear7 = nn.Linear(32, 16)
        self.linear8 = nn.Linear(16, 1)

    def forward(self, state, action):
        x_s1 = torch.sigmoid(self.linear1(state))
        x_a1 = torch.sigmoid(self.linear2(action))
        x = torch.cat([x_s1, x_a1], axis=1)
        x1 = torch.sigmoid(self.linear3(x))
        # x1 = torch.sigmoid(self.lineara(x1))
        x1 = self.linear4(x1)

        x_s2 = torch.sigmoid(self.linear5(state))
        x_a2 = torch.sigmoid(self.linear6(action))
        x = torch.cat([x_s2, x_a2], axis=1)
        x2 = torch.sigmoid(self.linear7(x))
        # x2 = torch.sigmoid(self.linearb(x2))
        x2 = self.linear8(x2)

        return x1, x2


class GraphQNet(torch.nn.Module):
    def __init__(self, num_agent, state_dim, action_dim=1):
        super(GraphQNet, self).__init__()
        # Q1 architecture
        self.conv1 = GCNConv(state_dim, 16)  # state & action
        self.conv2 = GCNConv(action_dim, 16)
        self.conv3 = GCNConv(32, 2)
        self.linear1 = nn.Linear(2 * num_agent, 32)
        self.linear2 = nn.Linear(32, num_agent)

        # Q1 architecture
        self.conv4 = GCNConv(state_dim, 16)  # state & action
        self.conv5 = GCNConv(action_dim, 16)
        self.conv6 = GCNConv(32, 2)
        self.linear3 = nn.Linear(2 * num_agent, 32)
        self.linear4 = nn.Linear(32, num_agent)

        self.num_agent = num_agent

    def forward(self, graph_state, graph_action, edge_index):
        x_s1 = self.conv1(graph_state, edge_index)
        x_a1 = self.conv2(graph_action, edge_index)
        x1 = torch.cat([x_s1, x_a1], axis=1)
        x1 = F.relu(x1)
        x1 = self.conv3(x1, edge_index)
        x1 = x1.view(-1, 2 * self.num_agent)
        x1 = self.linear1(x1)
        x1 = F.relu(x1)
        x1 = self.linear2(x1)

        x_s2 = self.conv4(graph_state, edge_index)
        x_a2 = self.conv5(graph_action, edge_index)
        x2 = torch.cat([x_s2, x_a2], axis=1)
        x2 = F.relu(x2)
        x2 = self.conv6(x2, edge_index)
        x2 = x2.view(-1, 2 * self.num_agent)
        x2 = self.linear3(x2)
        x2 = F.relu(x2)
        x2 = self.linear4(x2)
        return x1, x2


class QNet(torch.nn.Module):
    def __init__(self, num_agent, state_dim):
        super(QNet, self).__init__()
        self.agentnet = AgentQNet(state_dim)
        self.graphnet = GraphQNet(num_agent, state_dim)
        self.num_agent = num_agent

        """
        input: A list with (n+1) graph ————>  output: A combined (num,1) Q tensor & a (1) Q_total tensor                     
        """

    def forward(self, state, action, edge_index):
        agent_Q1, agent_Q2 = self.agentnet(state, action)
        graph_Q1, graph_Q2 = self.graphnet(state, action, edge_index)
        '''
        Q_1 = agent_Q1.view(-1, self.num_agent) + graph_Q1
        Q_2 = agent_Q2.view(-1, self.num_agent) + graph_Q2
        '''
        return agent_Q1, agent_Q2, graph_Q1, graph_Q2

class MixQNet(torch.nn.Module):
    def __init__(self, num_agent, graph_num_agent, state_dim, graph_weight, local_weight):
        super(MixQNet, self).__init__()
        self.agentnet = [AgentQNet(state_dim) for i in range(num_agent)]
        self.graphnet = GraphQNet(graph_num_agent, state_dim)
        self.num_agent = num_agent
        self.state_dim = state_dim
        self.graph_weight = graph_weight
        self.local_weight = local_weight

        """
        input: A list with (n+1) graph ————>  output: A combined (num,1) Q tensor & a (1) Q_total tensor                     
        """

    def forward(self, state, action, edge_index):
        # action & state:(batch_size, num_agent, d)
        agent_Q1 = []
        agent_Q2 = []
        for i in range(self.num_agent):
            q1, q2 = self.agentnet[i](state[:, i, :], action[:, i, :])
            agent_Q1.append(q1)
            agent_Q2.append(q2) # num_agent × (batch_size, 1)
        agent_Q1 = torch.hstack(agent_Q1).unsqueeze(-1) # (batch_size, num_agent, 1)
        agent_Q2 = torch.hstack(agent_Q2).unsqueeze(-1)
        graph_state = state.view(-1, self.state_dim)
        graph_action = action.view(-1, 1)
        graph_Q1, graph_Q2 = self.graphnet(graph_state, graph_action, edge_index) # (batch_size*num_agent, 1)
        graph_Q1 = graph_Q1.view(-1, self.num_agent, 1)
        graph_Q2 = graph_Q2.view(-1, self.num_agent, 1)
        
        Q_1 = self.local_weight * agent_Q1 + self.graph_weight * graph_Q1
        Q_2 = self.local_weight * agent_Q2 + self.graph_weight * graph_Q2
        # return shape: (batch_size, num_agent, 1)
        return Q_1, Q_2

class ActorNet(torch.nn.Module):
    def __init__(self, state_dim):
        """
        for base training
        """
        super(ActorNet, self).__init__()
        # self.linear1 = nn.Linear(state_dim, 64)
        # self.linear2 = nn.Linear(64, 32)
        # self.linear3 = nn.Linear(32, 1)
        # self.std_linear = nn.Linear(32, 1)

        self.linear1 = nn.Linear(state_dim, 16)
        self.linear2 = nn.Linear(16, 16)
        self.linear3 = nn.Linear(16, 1)
        self.std_linear = nn.Linear(16, 1)

    def forward(self, x):
        x = F.sigmoid(self.linear1(x))
        x = F.sigmoid(self.linear2(x))
        mu = self.linear3(x)
        std = self.std_linear(x)
        # std = torch.clamp(std, min=LOG_SIG_MIN, max=LOG_SIG_MAX)
        return mu, std

    def sample(self, state, deterministic=False, with_logprob=True):
        # refer to Github
        mu, log_std = self.forward(state)
        log_std = torch.clamp(log_std, LOG_STD_MIN, LOG_STD_MAX)
        std = torch.exp(log_std)
        normal = Normal(mu, std)
        if deterministic:
            # Only used for evaluating policy at test time.
            pi_action = mu
        else:
            pi_action = normal.rsample()
        # for reparameterization trick (mean + std * N(0,1))
        if with_logprob:
            logp_pi = normal.log_prob(pi_action)
            logp_pi -= 2 * (np.log(2) - pi_action - F.softplus(-2 * pi_action))
        else:
            logp_pi = None
        pi_action = torch.tanh(pi_action)
        return pi_action, logp_pi


class LocalPNet(torch.nn.Module):
    def __init__(self, state_dim):
        super(LocalPNet, self).__init__()
        self.conv1 = GCNConv(state_dim, 32)  # state
        self.convh = GCNConv(32, 32)
        self.conv2 = GCNConv(32, 1)
        self.std_conv = GCNConv(32, 1)

    def forward(self, x, edge_index):
        x = self.conv1(x, edge_index)
        x = self.convh(x, edge_index)
        mu = F.relu(x)
        mu = self.conv2(mu, edge_index)
        std = self.std_conv(x, edge_index)
        return mu, std

    def get_log_prob(self, x, edge_index, deterministic=False, with_logprob=True):
        mu, log_std = self.forward(x, edge_index) 
        log_std = torch.clamp(log_std, LOG_STD_MIN, LOG_STD_MAX)
        std = torch.exp(log_std)
        normal = Normal(mu, std)
        if deterministic:
            # Only used for evaluating policy at test time.
            pi_action = mu
        else:
            pi_action = normal.rsample()
        if with_logprob:
            logp_pi = normal.log_prob(pi_action)
            logp_pi -= 2 * (np.log(2) - pi_action - F.softplus(-2 * pi_action))
        else:
            logp_pi = None
        pi_action = torch.tanh(pi_action)
        return pi_action, logp_pi

        
"""
class GraphPNet(torch.nn.Module):
    def __init__(self):
        super(GraphPNet, self).__init__()
        self.conv1 = GCNConv(1, 32) #state
        self.conv2 = GCNConv(32, 1)


    def forward(self, data):
        x, edge_index = data.x, data.edge_index
        x = self.conv1(x, edge_index)
        x = F.relu(x)
        mu = self.conv2(x, edge_index)
        logstd = torch.zeros_like(mu)
        std = torch.exp(logstd)
        return mu, std

    
class PolicyNet(torch.nn.Module):
    def __init__(self, num_agent):
        super(PolicyNet, self).__init__()
        self.agentnet = LocalPNet()
        self.graphnet = GraphPNet()
        self.num_agent = num_agent

        #refer to Github
        self.action_space = gym.spaces.Box(np.array([0]), np.array([10]))
        self.action_scale = torch.FloatTensor((self.action_space.high - self.action_space.low) / 2.)
        self.action_bias = torch.FloatTensor((self.action_space.high + self.action_space.low) / 2.)

    def forward(self, x):
        # x is a Data list
        agent_mu = []
        agent_std = []
        for graph in x[:-1]:
            mu, std = self.agentnet(graph)
            agent_mu.append(mu)
            agent_std.append(std)
        agent_mu = torch.tensor(agent_mu)
        agent_mu = agent_mu.view(self.num_agent, -1)
        agent_std = torch.tensor(agent_std)
        agent_std = agent_std.view(self.num_agent, -1)
        graph_mu, graph_std = self.graphnet(x[-1])
        A_i = agent_mu + graph_mu
        # logstd = torch.zeros_like(A_i)
        std = (agent_std + graph_std) / 2
        return A_i, std

    def sample(self, state):
            #refer to Github
            mu, std = self.forward(state)
            normal = Normal(mu, std)   
            x_t = normal.rsample()  # for reparameterization trick (mean + std * N(0,1))
            y_t = torch.tanh(x_t)               
            action = y_t * self.action_scale + self.action_bias         
            return action     
"""

class eGCN(torch.nn.Module):
    def __init__(self):
        super(eGCN, self).__init__()
        self.conv1 = GCNConv(-1, 16)
        self.conv2 = GCNConv(16, 16)

    def forward(self, x, edge_index):
        x = self.conv1(x, edge_index)
        x = F.tanh(x)
        x = self.conv2(x, edge_index)
        x = F.tanh(x)
        return x

class eActor(torch.nn.Module):
    def __init__(self, node_num):
        super(eActor, self).__init__()
        self.node_num = node_num
        self.net1 = nn.Sequential(nn.Linear(node_num*16, 128),
                        nn.Tanh(),
                        nn.Linear(128, 128),
                        nn.Tanh(),
                        nn.Linear(128, node_num))
        self.net2 = nn.Sequential(nn.Linear(node_num*16, 128),
                        nn.Tanh(),
                        nn.Linear(128, 128),
                        nn.Tanh(),
                        nn.Linear(128, node_num))

    def forward(self, x):
        x = x.view(-1, self.node_num*16)
        alpha = self.net1(x)
        beta = self.net2(x)
        return alpha, beta
    
    def sample(self, state, deterministic=False, with_logprob=True):
        # refer to Github
        alpha, beta = self.forward(state)
        log_alpha = torch.clamp(alpha, LOG_STD_MIN, LOG_STD_MAX)
        alpha = torch.exp(log_alpha)
        log_beta = torch.clamp(beta, LOG_STD_MIN, LOG_STD_MAX)
        beta = torch.exp(log_beta)
        distribution = torch.distributions.beta.Beta(alpha, beta)
        if deterministic:
            # Only used for evaluating policy at test time.
            pi_action = alpha / (alpha + beta)
        else:
            pi_action = distribution.rsample()
        # for reparameterization trick (mean + std * N(0,1))
        if with_logprob:
            logp_pi = distribution.log_prob(pi_action)
            logp_pi -= 2 * (np.log(2) - pi_action - F.softplus(-2 * pi_action))
        else:
            logp_pi = None
        pi_action = torch.tanh(pi_action)
        return pi_action, logp_pi

class eCritic(torch.nn.Module):
    def __init__(self, node_num):
        super(eCritic, self).__init__()
        self.node_num = node_num
        self.net = nn.Sequential(nn.Linear(node_num*(16+1), 128),
                        nn.Tanh(),
                        nn.Linear(128, 128),
                        nn.Tanh(),
                        nn.Linear(128, 1))

    def forward(self, state, action):
        state = state.view(-1, self.node_num*16)
        action = action.view(-1, self.node_num)
        x = torch.cat([state, action], axis=1)
        x = self.net(x)
        return x
    
    
