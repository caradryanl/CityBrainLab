#include "../head/roadnet.h"
#include <cmath>
#include <iomanip>
#include <iostream>

std::string VehiclePosition::toString() {
  std::stringstream buf;
  buf << std::setprecision(12);
  buf << road_->GetUniqueID() << " ";
  buf << pos_ << " ";
  return buf.str();
}

VehiclePosition::VehiclePosition() {}

VehiclePosition::VehiclePosition(DynamicRoad *road, double pos, Lane *lane) {
  road_ = road;
  pos_ = pos;
  lane_ = lane;
}

VehiclePosition::VehiclePosition(std::stringstream &buf, RoadNet *roadnet) {
  int road_unique_id;
  double pos;
  buf >> road_unique_id >> pos;
  road_ = roadnet->QueryRoadByUniqueID(road_unique_id);
  pos_ = pos;
}

void VehiclePosition::LaneChange(int lane_delta) {
  int lane_number = lane_->lane_num_;
  lane_number += lane_delta;
  lane_ = road_->GetLane(lane_number);
}

void VehiclePosition::LaneSet(int lane_number) {
  lane_ = road_->GetLane(lane_number);
}

void VehiclePosition::GetLatLon(double &lat, double &lon) {
  if (road_ == NULL)
    throw std::invalid_argument("Empty Road in VehiclePosition::GetLatLon()");
  double lambda = pos_ / road_->length_;
  const TrafficNode *from = road_->from_;
  const TrafficNode *to = road_->to_;
  double dlat = to->latitude_ - from->latitude_;
  double dlon = to->longitude_ - from->longitude_;
  double len = sqrt(dlat * dlat + dlon * dlon);
  dlat /= len;
  dlon /= len;
  double turbulent = 5e-5;
  double from_lat = from->latitude_;
  double from_lon = from->longitude_;
  double to_lat = to->latitude_;
  double to_lon = to->longitude_;

  from_lat = from_lat + dlon * turbulent + dlat * turbulent;
  from_lon = from_lon + (-dlat) * turbulent  + dlon * turbulent;
  to_lat = to_lat + dlon * turbulent + (-dlat) * turbulent;
  to_lon = to_lon + (-dlat) * turbulent + (-dlon) * turbulent;



  lat = from_lat * (1 - lambda) + to_lat * lambda +
        (lane_->lane_num_) * dlon * turbulent;
  lon = from_lon * (1 - lambda) + to_lon * lambda +
        (lane_->lane_num_) * (-dlat) * turbulent;

}
