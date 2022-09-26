#include "../head/vehicle.h"
#include <ctime>
#include <iomanip>
#include <iostream>
#include <thread>

//const int NUMBER_OF_THREADS = 200;

VehicleGroup::~VehicleGroup() {
  for (auto it:vehicles_) delete it;
  vehicles_.clear();
  vehicle_table_.clear();
  //printf("This is optimized version.");
}

VehicleGroup::VehicleGroup(RoadNet *roadnet) {
  roadnet_ = roadnet;
  id_counter_ = 0;
}
VehicleGroup::VehicleGroup(RoadNet *roadnet,int thread_num) {
  roadnet_ = roadnet;
  NUMBER_OF_THREADS = thread_num;
  id_counter_ = 0;
  /*
  work_threads_=std::vector<WorkThread>(NUMBER_OF_THREADS);
  for (int i =0; i<NUMBER_OF_THREADS; i++){
    work_threads_.push_back(WorkThread());
  }
  */
}

VehicleGroup::VehicleGroup(RoadNet *roadnet, std::string raw_data) {
  roadnet_ = roadnet;
  std::stringstream buf;
  buf << raw_data;
  FromStream(buf);
  id_counter_ = 0;
}

VehicleGroup::VehicleGroup(RoadNet *roadnet, std::stringstream &buf) {
  roadnet_ = roadnet;
  FromStream(buf);
  id_counter_ = 0;
}

void VehicleGroup::AddVehicle(VehiclePlanned *vehicle) {
  vehicles_.push_back(vehicle);
  vehicle_table_[vehicle->unique_id_] = vehicle;
  id_counter_++;

  vehicle_threads_[thread_cnt_].push_back(vehicle);
  if (++thread_cnt_ >= NUMBER_OF_THREADS){
    thread_cnt_ = 0;
  }
}

std::string VehicleGroup::toString() {
  std::stringstream buf;
  buf << vehicles_.size() << " ";
  for (VehiclePlanned *vehicle : vehicles_)
    buf << vehicle->toString();
  return buf.str();
}

void VehicleGroup::FromStream(std::stringstream &buf) {
  int num_vehicle;
  buf >> num_vehicle;
  vehicles_.clear();
  for (int i = 0; i < num_vehicle; i++) {
    VehiclePlanned *vehicle = new VehiclePlanned(buf, roadnet_, true);
    vehicle->velocity_ = 1.0;
    vehicles_.push_back(vehicle);
  }
}

void VehicleGroup::SetVehicleRoadTable(VehicleRoadTable *vehicle_road_table, 
  RoadNet *roadnet, int time) {
  vehicle_road_table_ = vehicle_road_table;
  std::vector<std::pair<long long, VehiclePlanned *>> tables;
  for (VehiclePlanned *vehicle : vehicles_) {
    if (vehicle->status_ == VehiclePlanned::running)
      tables.push_back(
          std::make_pair(vehicle->pos_.road_->GetUniqueID(), vehicle));
    /*vehicle_road_table_->AddVehicle(vehicle->pos_.road_->GetUniqueID(),
                                    vehicle);*/
  }
  vehicle_road_table_->BatchAddVehicle(tables);
}

void ThreadsGetNextSecAction(std::vector<VehiclePlanned *> &unplaned_vehicles,
                             VehicleRoadTable *vehicle_road_table_) {
  for (VehiclePlanned *vehicle : unplaned_vehicles) {
    vehicle->GetNextSecAction(vehicle_road_table_);
  }
}

void ThreadsReplanning(std::vector<VehiclePlanned *> &unplaned_vehicles,
                       RoadNet *roadnet) {
  for (VehiclePlanned *vehicle : unplaned_vehicles) {
    vehicle->Replanning(roadnet);
  }
}

void ThreadsGetNextSecStatus(std::vector<VehiclePlanned *> &unplaned_vehicles,
                             VehicleRoadTable *vehicle_road_table_) {
  for (VehiclePlanned *vehicle : unplaned_vehicles) {
    vehicle->GetNextSecStatus(vehicle_road_table_);
  }
}

void ThreadsWork(WorkThread &thread, std::vector<VehiclePlanned *> &unplaned_vehicles, 
                              VehicleRoadTable *vehicle_road_table_){
  for (VehiclePlanned *vehicle : unplaned_vehicles) {
    thread.ComputeAct(vehicle, vehicle_road_table_);
  }
}


#define VEHICLEGROUP_MULTI_THREAD

void VehicleGroup::InitThreads(){
  thread_cnt_ = 0;
  int kThread = NUMBER_OF_THREADS;
  for (int i = 0; i < kThread; i++) {
    vehicle_threads_[i].clear();
  }
}

void VehicleGroup::GoTime(double sec, double start_sec) {

  double t1 = 1.0 * clock() / CLOCKS_PER_SEC;

  replanning_vehicles_.clear();
  for (VehiclePlanned *vehicle : vehicles_) {
    vehicle->PassTrafficSignal(vehicle_road_table_, roadnet_, this);
  }

  double t11 = 1.0 * clock() / CLOCKS_PER_SEC;

  int kThread = NUMBER_OF_THREADS;
  int cnt = 0;
  
  vehicle_road_table_->FixIt();

  #ifdef VEHICLEGROUP_MULTI_THREAD
  kThread = NUMBER_OF_THREADS;
  for (int i = 0; i < kThread; i++)
    action_threads_[i] = std::thread(
        ThreadsWork, std::ref(work_threads_[i]), ref(vehicle_threads_[i]), vehicle_road_table_);
  for (int i = 0; i < kThread; i++)
    action_threads_[i].join();

  double t2 = 1.0 * clock() / CLOCKS_PER_SEC;
#else
  // for (VehiclePlanned *vehicle : vehicles_) {
  //  vehicle->GetNextSecAction(vehicle_road_table_);
  //}
#endif
  for (VehiclePlanned *vehicle : vehicles_) {
    vehicle->GoTime(sec, start_sec, vehicle_road_table_, roadnet_);
  }
  double t3 = 1.0 * clock() / CLOCKS_PER_SEC;

//printf("%.3f %.3f %.3f\n", t11 - t1, t2 - t11, t3 - t2);
}

std::string VehicleGroup::Log(int warning_stop_time) {
  std::stringstream ss;
  ss << std::setprecision(12);
  ss << "[";
  bool is_first = true;
  for (VehiclePlanned *vehicle : vehicles_) {
    if (vehicle->status_ == VehiclePlanned::finished)
      continue;
    double lat = 0.0, lon = 0.0;
    vehicle->pos_.GetLatLon(lat, lon);
    if (lat < -180 || lat > 180 || lon < -90 || lon > 90){
      printf("lat: %.4f, lon: %.4f", lat, lon);
      continue;
    }

    if (!is_first) {
      ss << ",";
    } else {
      is_first = false;
    }

    ss << "[";
    ss << lat << "," << lon << ",";
    if (vehicle->stop_time_ >= warning_stop_time) {
      ss << "3";
    } else if (vehicle->next_signal_direction_ ==
               VehicleStatus::straight) {
      ss << "0";
    } else if (vehicle->next_signal_direction_ ==
                   VehicleStatus::left ||
               vehicle->next_signal_direction_ ==
                   VehicleStatus::uturn) {
      ss << "1";
    } else if (vehicle->next_signal_direction_ ==
               VehicleStatus::right) {
      ss << "2";
    } else {
      ss << "-1";
    }
    ss << "]";
  }
  ss << "]";
  return ss.str();
}
/*
std::string VehicleGroup::LogDebugSignal(TrafficSignal *signal) {
  std::stringstream ss;
  ss << std::setprecision(12);
  ss << "[";
  bool is_first = true;
  int cc = 0;
  for (VehiclePlanned *vehicle : vehicles_) {
    if (vehicle->status_ == VehiclePlanned::finished)
      continue;
    if (vehicle->vehicle_status_.next_signal_ != signal &&
        vehicle->debug_time_ == 0)
      continue;
    cc += 1;
    if (vehicle->vehicle_status_.next_signal_ == signal) {
      vehicle->debug_time_ = 5;
    } else {
      vehicle->debug_time_ -= 1;
    }
    if (!is_first) {
      ss << ",";
    } else {
      is_first = false;
    }
    ss << "[";
    double lat = 0.0, lon = 0.0;
    vehicle->pos_.GetLatLon(lat, lon);
    ss << lat << "," << lon << ",";
    if (vehicle->next_signal_direction_ == signal) {
      ss << "4";
    } else if (vehicle->next_signal_direction_ ==
               VehicleStatus::straight) {
      ss << "0";
    } else if (vehicle->next_signal_direction_ ==
               VehicleStatus::left) {
      ss << "1";
    } else if (vehicle->next_signal_direction_ ==
               VehicleStatus::right) {
      ss << "2";
    } else if (vehicle->next_signal_direction_ ==
               VehicleStatus::uturn) {
      ss << "3";
    }
    ss << "," << vehicle->unique_id_ << "]";
  }
  ss << "]";
  // std::cerr << "Alive " << cc << "\n";
  return ss.str();
}*/

void ThreadsSetPathInit(std::vector<VehiclePlanned *> &unplaned_vehicles,
                        RoadNet *roadnet) {
  for (auto vehicle : unplaned_vehicles) {
    vehicle->SetPathInit(roadnet);
  }
}

VehicleGroup::VehicleGroup(RoadNet *roadnet, int num_of_car, bool is_fake) {
  roadnet_ = roadnet;
  id_counter_ = 0;
  for (int i = 0; i < num_of_car; i++) {
    auto pairs = roadnet->RandNodePairs();
    TrafficNode *start = pairs.first;
    TrafficNode *end = pairs.second;
    VehiclePlanned *vehicle = new VehiclePlanned(start, end, i, roadnet, false);
    AddVehicle(vehicle);
  }

  const int kThread = NUMBER_OF_THREADS;

  std::cerr << kThread << std::endl;

  for (int i = 0; i < kThread; i++) {
    vehicle_threads_[i].clear();
  }
  int cnt = 0;
  for (VehiclePlanned *vehicle : vehicles_) {
    vehicle_threads_[cnt].push_back(vehicle);
    if (++cnt >= kThread)
      cnt = 0;
  }
  for (int i = 0; i < kThread; i++)
    action_threads_[i] =
        std::thread(ThreadsSetPathInit, ref(vehicle_threads_[i]), roadnet);

  for (int i = 0; i < kThread; i++)
    action_threads_[i].join();

  // for (auto vehicle : vehicles_)
  //  vehicle->SetPathInit(roadnet);
}

void VehicleGroup::Refresh(RoadNet *roadnet) {
  //std::vector<VehiclePlanned *> new_vehicles_;
  for (VehiclePlanned *vehicle : vehicles_) {
    if (vehicle->status_ != VehiclePlanned::finished) {
      /*auto pairs = roadnet->RandNodePairs();
      TrafficNode *start = pairs.first;
      TrafficNode *end = pairs.second;
      vehicle->Reset(start, end, roadnet);
      if (vehicle->status_ == VehiclePlanned::running)
        vehicle->pos_.pos_ = vehicle->pos_.road_->length_ *
                             (double)Common::RandomInt(0, (1 << 30) - 1) /
                             (double)(1 << 30);*/
      //new_vehicles_.push_back(vehicle);
      vehicle->last_step_ = Common::GetTimer();
    }

  }
  //new_vehicles_ = vehicles_;
}
/*
void VehicleGroup::Replanning(int replanning_num) {
  if (vehicles_.size() < replanning_num) {
    replanning_num = vehicles_.size();
  }
  for (int i = 0; i < replanning_num; i++) {
    VehiclePlanned *vehicle =
        vehicles_[Common::RandomInt(0, vehicles_.size() - 1)];
    vehicle->Replanning(roadnet_);
  }
}
*/
void VehicleGroup::Replanning(int replanning_num) {
  if (vehicles_.size() < replanning_num) {
    replanning_num = vehicles_.size();
  }
  for (int i = 0; i < replanning_num; i++) {
    VehiclePlanned *vehicle =
        vehicles_[Common::RandomInt(0, vehicles_.size() - 1)];
    vehicle->Replanning(roadnet_);
  }
}

void VehicleGroup::LogInfo() {
  int vehicle_cnt = 0;
  double vehicle_velocity = 0.0;
  for (VehiclePlanned *vehicle : vehicles_) {
    if (vehicle->status_ == VehiclePlanned::finished) {
      continue;
      vehicle->stop_time_ = 0;
    }
    vehicle_cnt++;
    vehicle_velocity += vehicle->GetVelocity();
    if (vehicle->GetVelocity() <= 1e-5) {
      vehicle->stop_time_ += 1;
    } else {
      vehicle->stop_time_ = 0;
    }
    if (vehicle->stop_time_ == 200 + 1) {
      std::cerr << "WARNING ID: " << vehicle->unique_id_
                << " Stop time : " << vehicle->stop_time_ << std::endl;
    }
  }
  double cur_ave_vel = vehicle_velocity / vehicle_cnt;
  if (vehicle_cnt != 0) {
    std::cerr << "Avg velocity: " << cur_ave_vel << "\n";
    his_ave_vel_.push_back(cur_ave_vel);
    double tot_vel = 0.0;
    for (auto vel : his_ave_vel_)
      tot_vel += vel;
    std::cerr << "His Ave velocity: " << tot_vel / his_ave_vel_.size() << "\n";
  }
  std::cerr << "Car Arrived #: " << Common::GetCarArriveCounter() << "\n";
}

