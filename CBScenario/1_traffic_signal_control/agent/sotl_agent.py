from agent.base import BaseAgent

class SOTLAgent(BaseAgent):
    """
    Agent using Self-organizing Traffic Light(SOTL) Control method to control traffic light
    """
    def __init__(self, id, action_space, intersection, min_green=10, max_red=30):
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

        # some threshold to deal with phase requests
        self.min_green_vehicle = min_green
        self.max_red_vehicle = max_red

    def get_ob(self):
        return 0

    def get_action(self, world):
        if not self.intersection.signal:
            return -1

        lane_waiting_count = world.eng.get_lane_waiting_vehicle_count()

        action = self.last_action
        for lane in [lane for lanes in self.phase_lane_map_in for lane in lanes]:
            lane_id = self.intersection.intersection["lanes"][lane-1]
            if lane_id not in lane_waiting_count.keys():
                lane_waiting_count[lane_id] = 0

        if world.eng.get_current_time() - self.last_set_time >= self.phase_time:
            self.last_set_time = world.eng.get_current_time()
            num_green_vehicles = sum([\
                lane_waiting_count[self.intersection.intersection["lanes"][lane-1]] \
                    for lane in self.phase_lane_map_in[self.last_action]])
            num_red_vehicles = sum([\
                lane_waiting_count[self.intersection.intersection["lanes"][lane-1]] \
                    for lane in [lane for lanes in self.phase_lane_map_in for lane in lanes]])
            num_red_vehicles -= num_green_vehicles

            if num_green_vehicles <= self.min_green_vehicle and num_red_vehicles > self.max_red_vehicle:
                action = (action + 1) % self.phase_num

        return action

    def get_reward(self):
        return 0