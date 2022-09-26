#ifndef CITY_BRAIN_TRAFFIC_FLOW_H_
#define CITY_BRAIN_TRAFFIC_FLOW_H_

#include "common.h"
#include "roadnet.h"
#include "vehicle.h"
#include <vector>

class TrafficFlow {
public:
  int start_time_, end_time_, time_interval_;
  std::vector<DynamicRoad *> path_;

public:
  TrafficFlow(std::stringstream &buf, RoadNet *roadnet);
  bool IsTimeValid(int cur_time);
};

class TrafficFlowGroup {
public:
  std::vector<TrafficFlow *> flows_;
  RoadNet *roadnet_;

public:
  TrafficFlowGroup();
  TrafficFlowGroup(std::stringstream &buf, RoadNet *roadnet);
  void FromStream(std::stringstream &buf);
  void AddVehicles(VehicleGroup *vehicle_group);
};

#endif // CITY_BRAIN_TRAFFIC_FLOW_H_