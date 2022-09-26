from . import BaseMetric
import numpy as np


class TravelTimeMetric(BaseMetric):
    """
    Calculate average travel time of all vehicles.
    For each vehicle, travel time measures time between it entering and leaving the roadnet.
    """

    def __init__(self, world):
        self.world = world
        self.world.subscribe(["vehicles", "time"])
        self.vehicle_enter_time = {}
        #  self.travel_time = []
        self.travel_time = {}

    def reset(self):
        self.vehicle_enter_time = {}
        self.travel_time = {}

    def update(self, done=False):
        vehicles = self.world.get_info("vehicles")
        current_time = self.world.get_info("time")

        for vehicle in vehicles:
            if not vehicle in self.vehicle_enter_time:
                self.vehicle_enter_time[vehicle] = current_time

        for vehicle in list(self.vehicle_enter_time):
            if not vehicle in vehicles:
                # print(vehicle,":",current_time,"-",self.vehicle_enter_time[vehicle])
                self.travel_time[vehicle] = current_time - self.vehicle_enter_time[vehicle]
                del self.vehicle_enter_time[vehicle]

        if done:
            print("eveluated vehicles:", len(self.travel_time))
            return self.travel_time
        else:
            return np.mean([value for key, value in self.travel_time.items()]) if len(self.travel_time) else 0
        # print("self.travel_time:",self.travel_time)
        # print("self.vehicle_enter_time:",self.vehicle_enter_time)

        
