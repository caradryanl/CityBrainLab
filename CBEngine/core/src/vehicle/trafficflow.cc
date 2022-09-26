#include "../head/trafficflow.h"
#include "../head/common.h"
#include "../head/roadnet.h"
#include <sstream>

TrafficFlow::TrafficFlow(std::stringstream &buf, RoadNet *roadnet) {
  buf >> start_time_;
  buf >> end_time_;
  buf >> time_interval_;
  int num_road;
  buf >> num_road;
  //fprintf(stderr, "new flow\n");
  for (int i = 0; i < num_road; i++) {
    long long road_id;
    buf >> road_id;
    path_.push_back(roadnet->QueryRoadByUniqueID(road_id));
  }
  for (int i = 0; i < num_road - 1; i++) {
    if (path_[i]->to_ != path_[i + 1]->from_) {
      throw std::invalid_argument(
          "Path is not consecutive in TrafficFlow::TrafficFlow");
    }
  }
}

bool TrafficFlow::IsTimeValid(int cur_time) {
  //printf("time interval:%d\n", time_interval_);
  if (cur_time < start_time_ || cur_time > end_time_)
    return false;
  return ((cur_time - start_time_) % time_interval_ == 0);
}

TrafficFlowGroup::TrafficFlowGroup() {}

TrafficFlowGroup::TrafficFlowGroup(std::stringstream &buf, RoadNet *roadnet) {
  roadnet_ = roadnet;
  FromStream(buf);
}

void TrafficFlowGroup::FromStream(std::stringstream &buf) {
  int num_flow;
  buf >> num_flow;
  for (int i = 0; i < num_flow; i++) {
    flows_.push_back(new TrafficFlow(buf, roadnet_));
  }
}

void TrafficFlowGroup::AddVehicles(VehicleGroup *vehicle_group) {
  int cur_time = Common::GetTimer();
  for (TrafficFlow *flow : flows_) {
    if (flow->IsTimeValid(cur_time)) {
      VehiclePlanned *vehicle =
          new VehiclePlanned(flow->path_, vehicle_group->id_counter_);
      vehicle_group->AddVehicle(vehicle);
    }
  }
}
