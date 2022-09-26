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