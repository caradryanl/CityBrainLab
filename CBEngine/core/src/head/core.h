#pragma once

#include "common.h"
#include "config.h"
#include "roadnet.h"
#include "trafficflow.h"
#include "vehicle.h"
#include "routing.h"
#include <mutex>
#include <thread>
#include <vector>

class Core {
public:
  static Core *create_engine(const std::string &file_addr, int thread_num);
  Core(const Config &config,int thread_num);
  void Reset();
  int thread_num;
  ~Core();
  void Run();
  void Log(int time);
  void LogBasicInfo();
  void LogWarning(VehicleRoadTable *table, int time);
  // Need to move to other place
  void SetVelocity(RoadNet *roadnet, VehicleRoadTable *table, int time);

  // interface
  int GetVehicleCount();  // vehicle_group_->vehicles_
  std::vector<int> GetVehicles(); // vehicle_group_->vehicles_
  std::vector<int> GetVehicles_full();  // vehicle_group_->vehicles_
  std::map<long long, int> GetLaneVehicleCount(); // vehicle_group_->vehicles_
  std::map<long long, std::vector<int>> GetLaneVehicles(); // vehicle_group_->vehicles_
  std::map<long long, int> GetLaneWaitingVehicleCount();
  std::map<std::string, std::vector<double>> GetVehicleInfo_full(long long int vehicle_id);
  std::vector<double> GetDrivingParams();
  void SetDrivingParams(std::vector<double> params);

  std::map<std::string, std::vector<double>> GetVehicleState(long long int vehicle_id);
  std::map<int, double> GetVehicleSpeed();
  double GetCurrentTime();
  double GetAverageTravelTime();
  std::map<std::string, std::vector<double>>
  GetVehicleInfo(long long vehicle_id);
  double GetRoadSpeedLimit(long long unique_id);
  void NextStep();
  std::vector<long long> GetVehicleRoute(long long int vehicle_id);
  void SetVehicleRoute(long long int vehicle_id, std::vector<long long> route);
  void SetRoadVelocity(long long road_id, double velocity);
  void SaveState(std::string file_name);
  void LoadState(std::string file_name);
  void LoadVehicle(int vehicle_id, std::map<std::string, std::vector<double>> info);
  int SetTLPhase(long long unique_id, int phase_id);
  int GetTLPhase(long long unique_id);
  void Start();
  void LogInfo(std::string file_name);
  void LogVehicleInfo(std::string file_name);
  void DebugSignal(long long unque_id);
    TrafficFlowGroup *traffic_flows_;

  // For Routing Module
  RoutingData* PrepareRouting();
  void ConductRouting(RoutingData *data, std::vector<VehicleInfo> &vehicle_);
  void UpdateRouting(std::vector<VehicleInfo> &vehicle_);

  
protected:
  Config config_; // cfg file provided when initialization
  VehicleGroup *vehicle_group_; // vehicle info
  RoadNet *roadnet_;  // static and dynamic (avg speed)
  std::vector<double> his_ave_vel_; 
  double current_time_; // current time
  int thread_num_;
  std::mutex roadnet_mutex, vehicle_group_mutex;

  //Modules
  RoutingModule *routing_module_;
};

