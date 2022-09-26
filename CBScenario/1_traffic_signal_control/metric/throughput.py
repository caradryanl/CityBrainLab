from . import BaseMetric
import numpy as np


class ThroughputMetric(BaseMetric):

    def __init__(self, world):
        super().__init__(world)
        self.last_vehicles = []
        self.throughput = []

    def reset(self):
        self.last_vehicles = []
        self.throughput = [] 

    def update(self, done=False):
        lane_vehicles = self.world.eng.get_vehicles()
        new_exited_vehicles = set(self.last_vehicles) - set(lane_vehicles)
        self.throughput += new_exited_vehicles
        self.last_vehicles = lane_vehicles
        if done:
            print("eveluated vehicles:", len(self.throughput))
            return len(self.throughput)
        else:
            return len(self.throughput)

    """
    Calculate average travel time of all vehicles.
    For each vehicle, travel time measures time between it entering and leaving the roadnet.
    

    def __init__(self, world):
        self.world = world
        self.world.subscribe(["lane_vehicles", "time"])
        self.last_lane_vehicles = {key: [] for key in world.eng.get_lane_vehicles()}
        self.lane_throughput = {key: [] for key in world.eng.get_lane_vehicles()}

    def reset(self):
        self.last_lane_vehicles = {key: [] for key in self.world.eng.get_lane_vehicles()}
        self.lane_throughput = {key: [] for key in self.world.eng.get_lane_vehicles()}

    def update(self, done=False):
        lane_vehicles = self.world.eng.get_lane_vehicles()
        current_time = self.world.eng.get_current_time()

        for lane_id, lane in lane_vehicles.items():
            '''
            if lane_id not in self.lane_throughput:
                self.lane_throughput[lane_id] = []
            if lane_id not in self.last_lane_vehicles:
                self.last_lane_vehicles[lane_id] = []
            '''
            new_exited_vehicles = set(self.last_lane_vehicles[lane_id]) - set(lane)
            self.lane_throughput[lane_id] += list(new_exited_vehicles)
            self.last_lane_vehicles[lane_id] = lane

        if done:
            print("eveluated vehicles:", len(self.lane_throughput))
            return self.lane_throughput
        else:
            return np.sum([len(lane) for lane_id, lane in self.lane_throughput.items()])
    
    """
