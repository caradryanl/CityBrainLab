import numpy as np
eps = 10e-5

class Price_Agent():
    def __init__(self, road_id, road, world, R, beta):
        self.toll = 5
        self.road = road_id
        self.world = world
        self.vehicle_enter_time = {}
        self.travel_times = {}
        self.R = R
        self.beta = beta
        self.road_length = road.length
        self.lanes = road.lane

    def get_action(self, i):
        
        if i == 0:
            return np.array([5])

        else:
            vehicles = self._get_vehicles() # *modify*
            R = self.R # 10e-4  # smooth
            beta = self.beta #4  # proportional coefficient
            maxSpeed = 16.7
            free_tt = self.road_length / maxSpeed
            actual_tt = self.get_road_average_traveltime()
            # print("actual tt: {}, free_tt: {}".format(actual_tt, free_tt))
            delta_toll = (1-R) * self.toll + R * (actual_tt - free_tt + len(vehicles) * 15) # *modify*
            # print(f'veh: {len(vehicles)}')
            # if i >= 30:
                # print(f"{i}, road id: {self.road}, toll: {delta_toll}, veh: {len(vehicles)}") # *modify*
            # print("delta toll:", delta_toll)
            # delta_toll = beta * (actual_tt - free_tt)/10
           #  print("delta = ", actual_tt-free_tt, "\tdelta_toll = ", delta_toll, '\n')
            self.toll = delta_toll
            return np.array([delta_toll])

    # tool func: get the vehicle list of THIS road
    def _get_vehicles(self):
        vehicles = []
        lane_vehicles = self.world.eng.get_lane_vehicles()
        for lane in self.lanes:
            if lane not in lane_vehicles.keys():
                continue
            for vehicle in lane_vehicles[lane]:
                vehicles.append(vehicle)
        return vehicles

    def update(self):
        vehicles = self._get_vehicles()
        # print(vehicles)
        # print(vehicles) # list
        current_time = self.world.get_info("time")

        for vehicle in vehicles:
            if not vehicle in self.vehicle_enter_time:  
                self.vehicle_enter_time[vehicle] = current_time  

        for vehicle in list(self.vehicle_enter_time):  
            if not vehicle in vehicles:  
                # print(vehicle,":",current_time,"-",self.vehicle_enter_time[vehicle])
                self.travel_times[vehicle] = current_time - self.vehicle_enter_time[vehicle]
                del self.vehicle_enter_time[vehicle]  
        # print(self.vehicle_enter_time) # dictionary

    def get_road_average_traveltime(self):
        vehicles = self._get_vehicles()
        # print(vehicles)
        # print(vehicles) # list
        current_time = self.world.get_info("time")
        # print("current time: {}".format(current_time))

        for vehicle in vehicles:
            if not vehicle in self.vehicle_enter_time:  
                self.vehicle_enter_time[vehicle] = current_time

        for vehicle in list(self.vehicle_enter_time):  
            if not vehicle in vehicles:  
                # print(vehicle,":",current_time,"-",self.vehicle_enter_time[vehicle])
                self.travel_times[vehicle] = current_time - self.vehicle_enter_time[vehicle]
                del self.vehicle_enter_time[vehicle] 
        # print(self.vehicle_enter_time) # dictionary

        if len(self.travel_times) != 0:
            # print(len(self.travel_times), "in record;")
            actual_tt = sum(value for key, value in self.travel_times.items()) / len(self.travel_times)
        else:
            # print("no vehicles!")
            actual_tt = self.road_length/16.7
        # actual_tt = actual_tt + len(vehicles) * 20 # *modify*
        # print("eveluated tt of lane", id, ":\t", actual_tt)
        # self.vehicle_enter_time = {}
        self.travel_times = {}
        return actual_tt




