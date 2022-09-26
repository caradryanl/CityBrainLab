import numpy as np


class Price_Agent():
    def __init__(self, lane, world):
        self.lane = lane
        self.world = world
    '''
    def get_ob(self):
        return self.ob_generator.generate(self.ob_generator.average)

    def get_reward(self):
        reward = self.reward_generator.generate(self.reward_generator.average)
        assert len(reward) == 1
        return reward
    '''
    def get_action(self, state):
        # road_vehicle_count = self.world.get_road_vehicle_count()[self.road.id]
        k = 2/7
        b = 17/7
        if(state[0] < 5):
            return np.array([-1.0])
        else:
            # return np.array([min(1.0, state[0] - 2)])
            return np.array([min(1.0, state[0] * k - b)])

