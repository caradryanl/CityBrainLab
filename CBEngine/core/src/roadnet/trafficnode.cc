#include "../head/roadnet.h"
#include <iomanip>
#include <iostream>

// TrafficNode::TrafficNode():longitude_(0), latitude_(0), unique_id_(0) {}

TrafficNode::TrafficNode(double longitude, double latitude, long long unique_id,
                         int id)
    : longitude_(longitude), latitude_(latitude), unique_id_(unique_id) {
  id_ = id;
  type_ = normal;
}

void TrafficNode::AddRoad(DynamicRoad *road) { road_.push_back(road); }

DynamicRoad *TrafficNode::GetBackRoad(DynamicRoad *road) {
  if (road_.size() != 2)
    return NULL;
  bool found = false;
  for (DynamicRoad *roads : road_) {
    if (roads == road)
      found = true;
  }
  if (!found)
    return NULL;
  for (DynamicRoad *roads : road_) {
    if (roads != road)
      return (DynamicRoad *)roads->GetRevRoad();
  }
  throw std::invalid_argument("Unknown behavior in TrafficNode::GetBackRoad");
  return NULL;
}

DynamicRoad *TrafficNode::GetFrontRoad(DynamicRoad *road) {
  if (road_.size() != 2)
    return NULL;
  bool found = false;
  DynamicRoad *road_rev = (DynamicRoad *)road->GetRevRoad();
  for (DynamicRoad *roads : road_) {
    if (roads == road_rev)
      found = true;
  }
  if (!found)
    return NULL;
  for (DynamicRoad *roads : road_) {
    if (roads != road_rev)
      return roads;
  }
  throw std::invalid_argument("Unknown behavior in TrafficNode::GetFrontRoad");
  return NULL;
}

DynamicRoad *TrafficNode::GetRandomFrontRoad(DynamicRoad *road,
                                             std::mt19937 &rnd) {
  if (road_.size() <= 1)
    return NULL;
  DynamicRoad *road_rev = (DynamicRoad *)road->GetRevRoad();
  while (true) {
    auto road = road_[rnd() % road_.size()];
    if (road != road_rev)
      return road;
  }
  throw std::invalid_argument("Unknown behavior in TrafficNode::GetFrontRoad");
  return NULL;
}

int TrafficNode::GetID() const { return id_; }

std::string TrafficNode::toString() {
  std::stringstream buf;
  buf << std::setprecision(12);
  buf << longitude_ << " ";
  buf << latitude_ << " ";
  buf << unique_id_ << " ";
  return buf.str();
}

std::vector<DynamicRoad *> &TrafficNode::GetRoad() { return road_; }

TrafficSignal::TrafficSignal(double longitude, double latitude,
                             long long unique_id, int id,
                             double red_left_period, double red_straight_period,
                             double green_left_period,
                             double green_straight_period)
    : TrafficNode(longitude, latitude, unique_id, id) {
  int range_l = 20, range_r = 25;
  type_ = signal;
  /*if (red_left_period == -1)
    red_left_period_ = Common::RandomInt(range_l, range_r); // Default setting
  else
    red_left_period_ = red_left_period;

  if (red_straight_period == -1)
    red_straight_period_ =
        Common::RandomInt(range_l, range_r); // Default setting
  else
    red_straight_period_ = red_straight_period;

  if (green_left_period == -1)
    green_left_period_ = Common::RandomInt(range_l, range_r); // Default setting
  else
    green_left_period_ = green_left_period;

  if (green_straight_period == -1)
    green_straight_period_ =
        Common::RandomInt(range_l, range_r); // Default setting
  else
    green_straight_period_ = green_straight_period;*/
  /*  phases_.push_back(std::make_pair(left02,
    Common::RandomInt(range_l,range_r)));
    phases_.push_back(std::make_pair(straight02,
    Common::RandomInt(range_l,range_r)));
    phases_.push_back(std::make_pair(left13,
    Common::RandomInt(range_l,range_r)));
    phases_.push_back(std::make_pair(straight13,
    Common::RandomInt(range_l,range_r)));*/
  int kInf = 1000000000;
  phases_.push_back(std::make_pair(left02, kInf));
  phases_.push_back(std::make_pair(straight02, kInf));
  phases_.push_back(std::make_pair(left13, kInf));
  phases_.push_back(std::make_pair(straight13, kInf));
  cur_phases_ = 0;
  remained_time_ = phases_[cur_phases_].second;
  is_yellow_ = false;
  yellow_period_ = 5;
}

void TrafficSignal::AddRoadOrdered(DynamicRoad *road) {
  if (road == NULL) {
    road_ordered_.push_back(road);
    return;
  }

  road_ordered_.push_back(road);
  bool found_road = false;
  for (auto oldroad : GetRoad()) {
    if (oldroad == road) {
      found_road = true;
    }
  }
  if (!found_road) {
    fprintf(stderr, "ERROR node id : %lld, road id : %lld\n", unique_id_,
            road->unique_id_);
    throw std::invalid_argument(
        "Can't find such road in TrafficSignal::AddRoadOrdered()");
  }
}

bool TrafficNode::CanGo(DynamicRoad *from, DynamicRoad *to) { return true; }

int TrafficSignal::GetFromID(DynamicRoad *from) {
  int from_id = -1;
  for (int i = 0; i < road_ordered_.size(); i++) {
    if (road_ordered_[i] == from->GetRevRoad())
      from_id = i;
  }
  if (from_id == -1) {
    throw std::invalid_argument(
        "Unknown from ID in TrafficSignal::GetFromID()");
  }
  return from_id;
}

TrafficSignal::DirectionType TrafficSignal::DirectionRelation(DynamicRoad *from,
                                                              DynamicRoad *to) {

  int from_id = -1, to_id = -1;
  for (int i = 0; i < road_ordered_.size(); i++) {
    if (road_ordered_[i] == from->GetRevRoad())
      from_id = i;
  }
  for (int i = 0; i < road_ordered_.size(); i++) {
    if (road_ordered_[i] == to)
      to_id = i;
  }
  if (from_id == -1 || to_id == -1) {
    throw std::invalid_argument(
        "Can't find such road in TrafficSignal::DirectionRelation()");
  }
  int type = (to_id - from_id + 4) % 4;
  if (type == 2) {
    return straight;
  } else if (type == 1) {
    return left;
  } else if (type == 3) {
    return right;
  } else {
    return uturn;
  }
}

bool TrafficSignal::CanGo(DynamicRoad *from, DynamicRoad *to) {
  if (is_yellow_)
    return false;
  int from_id = -1, to_id = -1;
  for (int i = 0; i < road_ordered_.size(); i++) {
    if (road_ordered_[i] == from->GetRevRoad())
      from_id = i;
  }
  for (int i = 0; i < road_ordered_.size(); i++) {
    if (road_ordered_[i] == to)
      to_id = i;
  }
  if (from_id == -1 || to_id == -1) {
    // printf("cnm %lld %lld %lld\n", from->unique_id_, to->unique_id_,
    //       unique_id_);
    throw std::invalid_argument(
        "Can't find such road in TrafficSignal::CanGo()");
  }
  DirectionType type = DirectionRelation(from, to);
  if (type == right) {
    return true;
  }
  if (GetStatus() == straight02 || GetStatus() == left02) {
    if (from_id % 2 != 0) {
      return false;
    }
    if ((type == left || type == uturn) && GetStatus() == left02) {
      return true;
    }
    if (type == straight && GetStatus() == straight02) {
      return true;
    }
    return false;
  } else if (GetStatus() == straight13 || GetStatus() == left13) {
    if (from_id % 2 == 0) {
      return false;
    }
    if ((type == left || type == uturn) && GetStatus() == left13) {
      return true;
    }
    if (type == straight && GetStatus() == straight13) {
      return true;
    }
    return false;
  } else if (GetStatus() == all0) {
    if (from_id == 0) {
      return true;
    } else {
      return false;
    }
  } else if (GetStatus() == all1) {
    if (from_id == 1) {
      return true;
    } else {
      return false;
    }
  } else if (GetStatus() == all2) {
    if (from_id == 2) {
      return true;
    } else {
      return false;
    }
  } else if (GetStatus() == all3) {
    if (from_id == 3) {
      return true;
    } else {
      return false;
    }
  }
  throw std::invalid_argument("Unknown status in TrafficSignal::CanGo()");
  return false;
}

void TrafficNode::GoTime(double sec, double start_sec) { return; }

void TrafficSignal::GoTime(double sec, double start_sec) {
  /*
    start_sec: the starting time stamp of GoTime op
    sec: the time span to step forward
  */
  //printf("id: %d, remain time: %d, is yellow: %d\n", unique_id_,  remained_time_, is_yellow_);
  while (remained_time_ <= sec) {   
    sec -= remained_time_;
    remained_time_ = 0;
    // if not yellow, set as yellow
    if (!is_yellow_) {
      is_yellow_ = true;
      remained_time_ = yellow_period_;
      //printf("not yellow cur_phases_: %d\n", cur_phases_);
    } else {
      is_yellow_ = false;
      cur_phases_ = (cur_phases_ + 1) % (int)(phases_.size());
      //printf("is yellow, cur_phases_: %d\n", cur_phases_);
      remained_time_ = phases_[cur_phases_].second;
    }
  }
  remained_time_ -= sec;
  //printf("after id: %d, remain time: %d\n", unique_id_,  remained_time_);
}



void TrafficSignal::GetSignalInfo(DynamicRoad *from, DynamicRoad *to,
                                  bool &can_go, double &remained_time) {
  if (is_yellow_) {
    can_go = false;
    remained_time = remained_time_;
    return;
  }
  int from_id = -1, to_id = -1;
  for (int i = 0; i < road_ordered_.size(); i++) {
    if (road_ordered_[i] == from->GetRevRoad())
      from_id = i;
  }
  for (int i = 0; i < road_ordered_.size(); i++) {
    if (road_ordered_[i] == to)
      to_id = i;
  }
  if (from_id == -1 || to_id == -1) {
    fprintf(stderr, "ERROR signal id : %lld from id : %lld to id : %lld\n",
            unique_id_, from->unique_id_, to->unique_id_);
    throw std::invalid_argument(
        "Can't find such road in TrafficSignal::GetSignalInfo()");
  }
  DirectionType type = DirectionRelation(from, to);
  remained_time = -3;
  can_go = false;
  if (type == right) {
    remained_time = -2;
    can_go = true;
  } else if (GetStatus() == straight02 || GetStatus() == left02) {
    if (from_id % 2 != 0) {
      /*if (GetStatus() == left02 && type == straight) {
        remained_time = remained_time_ + yellow_period_;
      } else {
        remained_time = -1;
      }*/
      remained_time = -1;
      can_go = false;
      return;
    }
    if ((type == left || type == uturn) && GetStatus() == left02) {
      remained_time = remained_time_;
      can_go = true;
      return;
    }
    if (type == straight && GetStatus() == straight02) {
      remained_time = remained_time_;
      can_go = true;
      return;
    }
    /*if ((type == left || type == uturn) && GetStatus() == straight02) {
      remained_time = remained_time_ + yellow_period_;
    } else {
      remained_time = -1;
    }*/
    remained_time = -1;
    can_go = false;
    return;
  } else if (GetStatus() == straight13 || GetStatus() == left13) {
    if (from_id % 2 == 0) {
      /*if (GetStatus() == left13 && type == straight) {
        remained_time = remained_time_ + yellow_period_;
      } else {
        remained_time = -1;
      }*/
      remained_time = -1;
      can_go = false;
      return;
    }
    if ((type == left || type == uturn) && GetStatus() == left13) {
      remained_time = remained_time_;
      can_go = true;
      return;
    }
    if (type == straight && GetStatus() == straight13) {
      remained_time = remained_time_;
      can_go = true;
      return;
    }
    /*if ((type == left || type == uturn) && GetStatus() == straight13) {
      remained_time = remained_time_ + yellow_period_;
    } else {
      remained_time = -1;
    }*/
    remained_time = -1;
    can_go = false;
    return;
  } else if (GetStatus() == all0) {
    if (from_id == 0) {
      remained_time = remained_time_;
      can_go = true;
      return;
    } else {
      remained_time = -1;
      can_go = false;
      return;
    }
  } else if (GetStatus() == all1) {
    if (from_id == 1) {
      remained_time = remained_time_;
      can_go = true;
      return;
    } else {
      remained_time = -1;
      can_go = false;
      return;
    }
  } else if (GetStatus() == all2) {
    if (from_id == 2) {
      remained_time = remained_time_;
      can_go = true;
      return;
    } else {
      remained_time = -1;
      can_go = false;
      return;
    }
  } else if (GetStatus() == all3) {
    if (from_id == 3) {
      remained_time = remained_time_;
      can_go = true;
      return;
    } else {
      remained_time = -1;
      can_go = false;
      return;
    }
  }
  if (remained_time == -3) {
    throw std::invalid_argument(
        "Unknown remained_time in TrafficSignal::GetSignalInfo()");
  }
}

double TrafficSignal::GetRemainedTime() { return remained_time_; }

TrafficSignal::Status TrafficSignal::GetStatus() {
  return phases_[cur_phases_].first;
}

TrafficNode::~TrafficNode() {}
