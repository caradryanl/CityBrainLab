from agent.base import BaseAgent

class MaxPressureAgent(BaseAgent):
    """
    Agent using Max-Pressure method to control traffic light
    """
    def __init__(self, id, action_space, intersection):
        super().__init__(action_space)
        self.id = id
        self.intersection = intersection
        
        # the minimum duration of time of one phase
        self.phase_time = 20
        self.last_set_time = 0
        self.last_action = 0
        self.phase_num = 8
        self.phase_lane_map_in = [[1, 7], [2, 8], [4, 10], [5, 11], [2, 1], [5, 4], [8, 7], [11, 10]]
        self.phase_lane_map_out = [[16, 17, 18, 22, 23, 24], [13, 14, 15, 19, 20, 21],
                                   [13, 14, 15, 19, 20, 21], [16, 17, 18, 22, 23, 24],
                                   [16, 17, 18, 19, 20, 21], [19, 20, 21, 22, 23, 24],
                                   [13, 14, 15, 22, 23, 24], [13, 14, 15, 16, 17, 18]]

    def get_ob(self):
        return 0

    def get_action(self, world):
        if not self.intersection.signal:
            return -1
        # get lane pressure
        lvc = world.eng.get_lane_vehicle_count()

        if world.eng.get_current_time() - self.last_set_time < self.phase_time:
            return self.last_action

        # get obs 
        self.last_set_time = world.eng.get_current_time()
        max_pressure = None
        action = -1
        for phase_id in range(self.phase_num):
            pressure = self.get_pressure(phase_id, lvc)
            if max_pressure is None or pressure > max_pressure:
                action = phase_id
                max_pressure = pressure

        return action   # 0-7

    def get_reward(self):
        return 0

    def get_pressure(self, phase_id, lvc):
        pressure = 0
        # print(self.intersection.intersection)
        for lane in self.phase_lane_map_in[phase_id]:
            lane_id = self.intersection.intersection["lanes"][lane-1]

            if lane_id in lvc:
                pressure += lvc[lane_id]
        pressure *= 3 # in lane is 1/3 of out lane
        for lane in self.phase_lane_map_out[phase_id]:
            lane_id = self.intersection.intersection["lanes"][lane-1]
            if lane_id in lvc:
                pressure -= lvc[lane_id]
        
        return pressure
