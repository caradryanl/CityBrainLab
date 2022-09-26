from agent.base import BaseAgent
import gym


class Fixedtime_Agent(BaseAgent):
    def __init__(self, action_space, iid, intersection, num=4):
        super().__init__(action_space)
        if num == 2 or num==3 or num==5:
            self.action_space = gym.spaces.Discrete(num)
            print(iid, num)
        if num == 4:
            self.action_space = gym.spaces.Discrete(8)
        self.iid = iid
        self.last_action = 0
        self.last_action_time = 0

        self.phase_time = 20

    def get_ob(self):
        return 0

    def get_reward(self):
        return 0

    def get_action(self, world):
        current_time = world.eng.get_current_time()
        if current_time - self.last_action_time >= self.phase_time:
            # print("action space is:", self.action_space.n)
            self.last_action = (self.last_action+1) % self.action_space.n
            self.last_action_time = current_time

        return self.last_action
