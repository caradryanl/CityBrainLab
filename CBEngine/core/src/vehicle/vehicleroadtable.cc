#include "../head/vehicle.h"
#include <iostream>

VehicleRoadTable::VehicleRoadTable() {
  empty_set_.clear();
  is_fixed_ = false;
}

void VehicleRoadTable::AddVehicle(long long road_id, VehiclePlanned *vehicle) {
  if (is_fixed_) {
    throw std::invalid_argument("Vehicle is added after the table is fixed in "
                                "VehicleRoadTable::AddVehicle()");
  }
  cars_on_road_[road_id].push_back(vehicle);
}

// Update cars, cars_on_road_ with vehicles
void VehicleRoadTable::BatchAddVehicle(
    std::vector<std::pair<long long, VehiclePlanned *>> vehicles) {
  std::sort(vehicles.begin(), vehicles.end());
  for (int l = 0; l < vehicles.size(); l++) {
    int r = l;
    long long road_id = vehicles[l].first;
    while (r < vehicles.size() && vehicles[r].first == road_id)
      r++;
    std::vector<VehiclePlanned *> cars(r - l, NULL);
    for (int i = l; i < r; i++)
      cars[i - l] = vehicles[i].second;
    cars_on_road_[road_id] = cars;
    l = r - 1;
  }
}

void VehicleRoadTable::RemoveVehicle(VehiclePlanned *vehicle,
                                     long long road_id) {
  if (is_fixed_) {
    throw std::invalid_argument("Vehicle is removed after the table is fixed "
                                "in VehicleRoadTable::RemoveVehicle()");
  }
  std::vector<VehiclePlanned *> &vehicles = cars_on_road_[road_id];
  std::vector<VehiclePlanned *>::iterator vehicle_it =
      std::find(vehicles.begin(), vehicles.end(), vehicle);
  if (vehicle_it == vehicles.end()) {
    throw std::invalid_argument("Can't find the vehicle on the road in "
                                "VehicleRoadTable::RemoveVehicle()");
  }
  vehicles.erase(vehicle_it);
}

void VehicleRoadTable::MoveVehicle(VehiclePlanned *vehicle, long long old_id,
                                   long long new_id) {
  // (vehicleRoadTable)move vehicle
  if (is_fixed_) {
    throw std::invalid_argument("Vehicle is moved after the table is fixed "
                                "in VehicleRoadTable::MoveVehicle()");
  }
  std::vector<VehiclePlanned *> &old_vehicles = cars_on_road_[old_id];
  std::vector<VehiclePlanned *>::iterator vehicle_it =
      std::find(old_vehicles.begin(), old_vehicles.end(), vehicle);
  if (vehicle_it == old_vehicles.end()) {
    throw std::invalid_argument("Can't find the vehicle on the road in "
                                "VehicleRoadTable::MoveVehicle()");
  }
  old_vehicles.erase(vehicle_it);
  cars_on_road_[new_id].push_back(vehicle);
}

std::vector<VehiclePlanned *> *
VehicleRoadTable::QueryCarsOnRoad(long long road_id) {
  if (cars_on_road_.count(road_id)) {
    return &cars_on_road_[road_id];
  } else {
    return &empty_set_;
  }
}

VehicleRoadTable::~VehicleRoadTable() {
  std::map<long long, std::vector<VehiclePlanned *>>().swap(cars_on_road_);
  std::map<std::pair<long long, int>, std::vector<VehiclePlanned *>>().swap(
      cars_on_lane_);
}

// Move vehicles passing traffic signals to the right road.
void VehicleRoadTable::FixIt() {
  if (is_fixed_) {
    throw std::invalid_argument("Table is already fixed "
                                "in VehicleRoadTable::FixIt()");
  }
  for (auto &pairs : cars_on_road_) {
    for (VehiclePlanned *vehicle : pairs.second) {
      if (vehicle->pos_.road_->unique_id_ != pairs.first) {
        throw std::invalid_argument(
            "Vehicle is on the wrong road in VehicleRoadTable::FixIt()");
      }
      cars_on_lane_[std::make_pair(pairs.first, vehicle->pos_.lane_->lane_num_)]
          .push_back(vehicle);
    }
  }
  for (auto &pairs : cars_on_lane_) {
    auto &cars = pairs.second;
    sort(cars.begin(), cars.end(),
         [&](VehiclePlanned *car1, VehiclePlanned *car2) {
           return car1->pos_.pos_ < car2->pos_.pos_;
         });
  }
  is_fixed_ = true;
}

VehiclePlanned *VehicleRoadTable::GetNextVehicleOnLane(long long road_id,
                                                       int lane_num, double pos,
                                                       bool tie_breaker,
                                                       long long car_id) {
  if (!is_fixed_) {
    throw std::invalid_argument(
        "Didn't implement yet in VehicleRoadTable::GetNextVehicleOnLane()");
  }
  auto it = cars_on_lane_.find(std::make_pair(road_id, lane_num));
  if (it == cars_on_lane_.end()) {
    return NULL;
  }
  auto &cars = it->second;
  for (auto car : cars) {
    if (car->pos_.pos_ < pos - 1e-9 || car->unique_id_ == car_id)
      continue;
    if (car->pos_.pos_ > pos + 1e-9)
      return car;
    if (!tie_breaker || car->unique_id_ > car_id)
      return car;
  }
  return NULL;
}

VehiclePlanned *VehicleRoadTable::GetLastVehicleOnLane(long long road_id,
                                                       int lane_num, double pos,
                                                       bool tie_breaker,
                                                       long long car_id) {
  if (!is_fixed_) {
    throw std::invalid_argument(
        "Didn't implement yet in VehicleRoadTable::GetLastVehicleOnLane()");
  }
  auto it = cars_on_lane_.find(std::make_pair(road_id, lane_num));
  if (it == cars_on_lane_.end()) {
    return NULL;
  }
  auto &cars = it->second;
  for (auto it = cars.rbegin(); it != cars.rend(); it++) {
    auto car = *it;
    if (car->pos_.pos_ > pos - 1e-9 || car->unique_id_ == car_id)
      continue;
    if (car->pos_.pos_ < pos + 1e-9)
      return car;
    if (!tie_breaker || car->unique_id_ > car_id)
      return car;
  }
  return NULL;
}

int VehicleRoadTable::CountNumberVehicleOnLane(long long road_id, int lane_num,
                                               double pos, bool tie_breaker,
                                               long long car_id) {
  if (!is_fixed_) {
    throw std::invalid_argument(
        "Didn't implement yet in VehicleRoadTable::CountNumberVehicleOnLane()");
  }
  auto it = cars_on_lane_.find(std::make_pair(road_id, lane_num));
  if (it == cars_on_lane_.end()) {
    return 0;
  }
  auto &cars = it->second;
  int next_cnt = cars.size();
  for (auto car : cars) {
    if (car->pos_.pos_ < pos - 1e-9 || car->unique_id_ == car_id) {
      next_cnt--;
      continue;
    }
    if (car->pos_.pos_ > pos + 1e-9)
      return next_cnt;
    if (!tie_breaker || car->unique_id_ > car_id)
      return next_cnt;
  }
  return 0;
}
