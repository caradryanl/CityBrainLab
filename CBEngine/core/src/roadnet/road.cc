#include "../head/roadnet.h"
#include <cmath>
#include <iomanip>
#include <sstream>

Lane::Lane(bool turn_left, bool go_straight, bool turn_right, int lane_num, long long lane_id)
    : turn_left_(turn_left), go_straight_(go_straight), turn_right_(turn_right),
      lane_num_(lane_num), lane_id_(lane_id) {}

Road::Road(TrafficNode *from, TrafficNode *to, double length,
           double speed_limit, int num_of_lane, long long unique_id)
    : unique_id_(unique_id), from_(from), to_(to), length_(length),
      speed_limit_(speed_limit), num_of_lane_(num_of_lane) {}

void Road::AddLaneFromStream(std::stringstream &buf, int lane_num) {
  int turn_left, go_straight, turn_right;
  buf >> turn_left >> go_straight >> turn_right;
  int lane_id = (int)lanes_.size();
  lanes_.push_back(new Lane(turn_left, go_straight, turn_right, lane_num,
                            unique_id_ * 100 + lane_id));
}

RoadVelocityRecord::RoadVelocityRecord(double record_time, double ave_velocity,
                                       int cars)
    : record_time_(record_time), ave_velocity_(ave_velocity), cars_(cars) {}

void Road::SetRevRoad(Road *rev_road) { rev_road_ = rev_road; }
Road *Road::GetRevRoad() { return rev_road_; }
long long Road::GetUniqueID() { return unique_id_; }

double DynamicRoad::GetPassTime() {
  return length_ / std::max(1e-5, velocity_);
}

Lane *Road::GetLane(int lane_num) {
  if (lane_num < 0 || lane_num >= num_of_lane_) {
    throw std::invalid_argument("Wrong lane number in Road::GetLane()");
  }
  return lanes_[lane_num];
}

std::string Road::toStringBidirected() {
  std::stringstream buf;
  buf << std::setprecision(12);
  buf << from_->unique_id_ << " ";
  buf << to_->unique_id_ << " ";
  buf << length_ << " ";
  buf << speed_limit_ << " ";
  buf << num_of_lane_ << " ";
  if (rev_road_ != nullptr){
    buf << rev_road_->num_of_lane_ << " ";
    buf << unique_id_ << " ";
    buf << rev_road_->unique_id_ << " ";
  }
  else{
    buf << 0 << " ";
    buf << unique_id_ << " ";
    buf << -1 << " ";
  }
  return buf.str();
}

DynamicRoad::DynamicRoad(TrafficNode *from, TrafficNode *to, double length,
                         double speed_limit, int num_of_lane,
                         long long unique_id)
    : Road(from, to, length, speed_limit, num_of_lane, unique_id) 
{
  num_vehicle_ = 0;
  velocity_ = speed_limit_;
}

std::string Road::Log(int id) {
  std::stringstream ss;
  ss << std::setprecision(12);
  const TrafficNode *from = from_;
  const TrafficNode *to = to_;
  double dlat = to->latitude_ - from->latitude_;
  double dlon = to->longitude_ - from->longitude_;
  double len = sqrt(dlat * dlat + dlon * dlon);
  if (len == 0)
    len = 5e-5;
  dlat /= len;
  dlon /= len;
  double turbulent = 5e-5;
  if(id == -1){
  ss << "[[" << from_->latitude_ + dlon * turbulent + dlat * turbulent<< ","
     << from_->longitude_ + (-dlat) * turbulent  + dlon * turbulent<< "],["
     << to_->latitude_ + dlon * turbulent + (-dlat) * turbulent<< ","
     << to_->longitude_ + (-dlat) * turbulent + (-dlon) * turbulent<< "]]";
  }
  else{
    double id_lat = to_->latitude_ + dlon * turbulent + (-dlat) * turbulent + (-dlon) * turbulent/2;
    double id_lon = to_->longitude_ + (-dlat) * turbulent + (-dlon) * turbulent + dlat * turbulent/2;
    ss << "[[" << from_->latitude_ + dlon * turbulent + dlat * turbulent<< ","
       << from_->longitude_ + (-dlat) * turbulent  + dlon * turbulent<< "],["
       << to_->latitude_ + dlon * turbulent + (-dlat) * turbulent<< ","
       << to_->longitude_ + (-dlat) * turbulent + (-dlon) * turbulent<< "],["
       << id_lat<<","<<id_lon<<","<<id<<"]]";
  }
  return ss.str();
}
