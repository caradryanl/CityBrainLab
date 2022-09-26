import torch
import torch.nn.functional as F
import numpy as np
class Random_Human:
    """
    Agent using Economic model for user
    """

    def __init__(self, vehicle, world):

        self.vehicle = vehicle
        self.world = world

    def get_action(self, world):
        current_route = self.world.eng.get_vehicle_info(self.vehicle.id)["route"][:-1].split(" ")
        if len(current_route) <= 2:
            return current_route
        else:
            route_choices = world.route["-".join([current_route[1], current_route[-1]])]
        # if self.vehicle.monitor == False:
        #     return current_route[:-1].split(' ')
    
        # select one route
        pro = np.random.dirichlet(np.ones(3))
        choice = np.random.choice(3,1,p=pro)[0]
        self.vehicle.route = route_choices[choice]
        world.vehicle_route[self.vehicle.id.split("_")[1]][choice] += 1
        return route_choices[choice]



    def get_reward(self):
        return None
