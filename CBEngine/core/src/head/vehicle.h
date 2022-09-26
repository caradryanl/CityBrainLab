#ifndef CITY_BRAIN_VEHICLE_H_
#define CITY_BRAIN_VEHICLE_H_

#include "common.h"
#include "roadnet.h"
#include <random>
#include <thread>
#include <mutex>

class VehicleGroup;
class VehiclePlanned;
class VehicleRoadTable;

class Vehicle {
public:
  VehiclePosition pos_;
  double velocity_;
  double max_acc_, min_acc_;
  long long unique_id_;
  enum VehicleType { normal, bus };
  VehicleType vehicle_type_;
  std::mt19937 rnd_;

public:
  friend class VehicleGroup;
};

class VehicleStatus {
public:
  double dis_to_signal_, signal_remained_time_, dis_to_next_car_;
  TrafficSignal *next_signal_;
  VehiclePlanned *this_car_;
  VehiclePlanned *next_car_;
  VehiclePlanned *left_next_car_, *left_last_car_, *right_next_car_,
      *right_last_car_;
  double dis_to_left_next_car_, dis_to_left_last_car_, dis_to_right_next_car_,
      dis_to_right_last_car_;
  bool signal_can_go_;
  std::vector<int> num_of_car_to_light_;
  // finished means the vehicles are finished now
  enum Direction { left, right, straight, uturn, destination, finished };
  Direction next_signal_direction_;
  void SetSignalInfo(VehiclePlanned *car);

  VehicleStatus():next_signal_(nullptr), this_car_(nullptr), next_car_(nullptr),
    left_next_car_(nullptr), left_last_car_(nullptr), right_next_car_(nullptr),
    right_last_car_(nullptr){};
};

class VehicleTurnStatus : public VehicleStatus {
public:
  std::vector<VehiclePlanned *> next_car_lane_;
  std::vector<double> dis_next_car_lane_;
};

class VehicleAction {
public:
  enum Action { accelerate, decelerate, lane_change, lane_turn};
  Action action_type_;
  double velocity_delta_;
  int lane_delta_;
  int lane_turn_;
};

class DrivingData{
public:
  const VehicleStatus *vehicle_status_;
public:
  DrivingData(const VehicleStatus *vehicle_status=nullptr):vehicle_status_(vehicle_status){};
  void Reset(const VehicleStatus *vehicle_status){
      vehicle_status_ = vehicle_status;
  };
};

class VehiclePlanned : public Vehicle {
public:
  TrafficNode *from_, *to_;
  int safety_distance_min_ = 5;
  int safety_distance_max_ = 10;
  int safety_distance_;
  std::vector<DynamicRoad *> path_;
  double t_ff_;
  //VehicleStatus vehicle_status_;
  std::vector<VehicleAction> actions_;
  int debug_time_, start_time_;
  enum Status { unplanned, running, finished };
  enum ActionType {
    unknown,
    act_finished,
    signal_turn,
    signal_straight,
    signal_stop,
    go_straight
  };
  int last_step_;
  Status status_;
  ActionType action_type_;
  int stop_time_;
  bool is_fixed_;
  VehicleStatus::Direction next_signal_direction_;

public:
  VehiclePlanned(TrafficNode *from, TrafficNode *to, long long unique_id,
                 RoadNet *roadnet = NULL, bool find_path = false);
  VehiclePlanned(TrafficNode *from, TrafficNode *to, long long unique_id, int placeholder);
  VehiclePlanned(std::stringstream &buf, RoadNet *roadnet,
                 bool find_path = false);
  VehiclePlanned(std::vector<DynamicRoad *> path, long long unique_id,
                 bool is_fixed = true);
  ~VehiclePlanned();
  void SetPath(VehiclePosition pos, TrafficNode *to, RoadNet *roadnet);
  void SetPathInit(RoadNet *roadnet);
  void SetActionStatus(int action, int status);
  void Reset(TrafficNode *from, TrafficNode *to, RoadNet *roadnet);
  void Replanning(RoadNet *roadnet);
  void FixUTurn(RoadNet *roadnet);
  double GetVelocity() const;
  VehiclePosition GetVehiclePosition() const;
  void GetNextCarOnLane(VehicleRoadTable *table, int lane_num,
                        VehiclePlanned *&next_car, double &dis_to_next_car,
                        bool tie_breaker = false) const;
  void GetLastCarOnLane(VehicleRoadTable *table, int lane_num,
                        VehiclePlanned *&last_car, double &dis_to_last_car) const;
  void GetNextLastCarOnLane(VehicleRoadTable *table, int lane_num,
                        VehiclePlanned *&next_car, VehiclePlanned *&last_car,
                        double &dis_to_next_car, double &dis_to_last_car, 
                        bool tie_breaker = false) const;
  int GetCarNumOnLaneToSignal(VehicleRoadTable *table, int lane_num) const;
  void GetVehicleStatus(VehicleRoadTable *table, VehicleStatus &status);
  VehicleTurnStatus GetVehicleTurnStatus(VehicleRoadTable *table);
  std::vector<VehicleAction> GetAction(VehicleStatus vehicle_status);
  std::vector<VehicleAction> GetTurnAction(VehicleTurnStatus vehicle_status);
  bool IsSameDirection(VehicleStatus::Direction next_signal_direction, Lane *lane) const;
  bool CanStop(double front_velocity, double back_velocity, double distance,
               double eps, double safe_distance) const;
  double GetMaxVelocity(double initial_velocity, double next_car_velocity,
                        double dis_to_next_car) const;
  double GetMinVelocity(double initial_velocity, double last_car_velocity,
                        double dis_to_last_car, double safe_distance) const;
  double GetMaxVelocityNew(double initial_velocity, double next_car_velocity,
                           double dis_to_next_car) const;
  double GetMinVelocityNew(double initial_velocity, double last_car_velocity,
                           double dis_to_last_car, double safe_distance) const;
  double GetMaxVelocityOld(double initial_velocity, double next_car_velocity,
                           double dis_to_next_car) const;
  double GetMinVelocityOld(double initial_velocity, double last_car_velocity,
                           double dis_to_last_car, double safe_distance) const;
  void DoActions(std::vector<VehicleAction> vehicle_actions);
  void PassTrafficSignal(VehicleRoadTable *table, RoadNet *roadnet,
                         VehicleGroup *vehicle_group);
  void UpdateNextSignalDirection();
  void GetNextSecAction(VehicleRoadTable *table);
  void GetNextSecStatus(VehicleRoadTable *table);
  void GoTime(double sec, double start_sec, VehicleRoadTable *table,
              RoadNet *roadnet);
  double GetCutinDistance(VehicleStatus::Direction next_signal_direction) const;
  void CarArrived();
  std::string toString();
  std::string PrintPath();
  std::string LogPos();
  double CalcStopDis(double, double) const;
  friend class VehicleGroup;
  friend VehicleStatus;

  //DrivingModule
  DrivingData *PrepareDriving(VehicleStatus *vehicle_status);
  std::vector<VehicleAction> ConductDriving(DrivingData *data);
  void UpdateDriving(std::vector<VehicleAction>);

};

class VehicleRoadTable {
public:
  std::map<long long, std::vector<VehiclePlanned *>> cars_on_road_;
  std::vector<VehiclePlanned *> empty_set_;
  std::map<std::pair<long long, int>, std::vector<VehiclePlanned *>>
      cars_on_lane_;
  bool is_fixed_;

public:
  VehicleRoadTable();
  std::vector<VehiclePlanned *> *QueryCarsOnRoad(long long road_id);
  void AddVehicle(long long road_id, VehiclePlanned *vehicle);
  void
  BatchAddVehicle(std::vector<std::pair<long long, VehiclePlanned *>> vehicles);
  void RemoveVehicle(VehiclePlanned *vehicle, long long road_id);
  void MoveVehicle(VehiclePlanned *vehicle, long long old_id, long long new_id);
  VehiclePlanned *GetNextVehicleOnLane(long long road_id, int lane_num,
                                       double pos, bool tie_breaker = false,
                                       long long car_id = -1);
  VehiclePlanned *GetLastVehicleOnLane(long long road_id, int lane_num,
                                       double pos, bool tie_breaker = false,
                                       long long car_id = -1);
  int CountNumberVehicleOnLane(long long road_id, int lane_num, double pos,
                               bool tie_breaker = false, long long car_id = -1);
  void FixIt();
  ~VehicleRoadTable();
};

void ThreadsGetNextSecAction(std::vector<VehiclePlanned *> &unplaned_vehicles,
                             VehicleRoadTable *vehicle_road_table_);

void ThreadsGetNextSecStatus(std::vector<VehiclePlanned *> &unplaned_vehicles,
                             VehicleRoadTable *vehicle_road_table_);

void ThreadsSetPathInit(std::vector<VehiclePlanned *> &unplaned_vehicles,
                        RoadNet *roadnet);

void ThreadsReplanning(std::vector<VehiclePlanned *> &unplaned_vehicles,
                       RoadNet *roadnet);

class DrivingModule{
public:
  struct DrivingParams{
    std::vector<double> params_;
    DrivingParams(){
      params_ = std::vector<double>();
      params_.push_back(2.0); // max_acc_
      params_.push_back(5.0); // min_acc_
    };
  };
    static DrivingParams config_;
private:
    const DrivingData *data_;
private:
    bool IsSameDirection(VehicleStatus::Direction, Lane*) const;
    double GetCutinDistance(VehicleStatus::Direction) const;
    
public:
    DrivingModule(DrivingData *data=nullptr):data_(data){};
    ~DrivingModule(){};
    void SetData(DrivingData *data=nullptr){data_=data;};
    void ConductDriving(std::vector<VehicleAction>& actions);
    std::vector<double> GetModuleParams() const;
    void SetModuleParams(std::vector<double> params);
};

class WorkThread{
private:  
  VehicleStatus status_;
  DrivingData data_;
  DrivingModule module_;
public:
  WorkThread(){};
  ~WorkThread(){};
  void ComputeAct(VehiclePlanned *vehicle, VehicleRoadTable *table);
};

void ThreadsWork(WorkThread &thread, 
  std::vector<VehiclePlanned *> &unplaned_vehicles, 
  VehicleRoadTable *vehicle_road_table_);

class VehicleGroup {
public:
  std::vector<VehiclePlanned *> vehicles_;
  VehicleRoadTable *vehicle_road_table_;
  std::vector<VehiclePlanned *> vehicle_threads_[21];
  std::thread action_threads_[21];
  std::vector<double> his_ave_vel_;
  std::vector<VehiclePlanned *> replanning_vehicles_;
  std::map<long long, VehiclePlanned*> vehicle_table_;
  int id_counter_;
  int thread_cnt_;
  std::vector<WorkThread> work_threads_=std::vector<WorkThread>(21,WorkThread());

public:
  int NUMBER_OF_THREADS;
  RoadNet *roadnet_;
  VehicleGroup(RoadNet *roadnet);
  VehicleGroup(RoadNet *roadnet, int thread_num);
  VehicleGroup(RoadNet *roadnet, std::string raw_data);
  VehicleGroup(RoadNet *roadnet, std::stringstream &buf);
  ~VehicleGroup();
  VehicleGroup(RoadNet *roadnet, int num_of_car, bool is_fake); // used
  void FromStream(std::stringstream &buf);
  void SetVehicleRoadTable(VehicleRoadTable *vehicle_road_table, RoadNet *roadnet, int time);
  void AddVehicle(VehiclePlanned *vehicle);
  void Replanning(int replanning_num);
  void Refresh(RoadNet *roadnet);
  void GoTime(double sec, double start_sec = 0.0);
  void InitThreads();
  std::string toString();
  std::string Log(int warning_stop_time = 1000000000);
  std::string LogDebugSignal(TrafficSignal *signal);
  void LogInfo();
};


class BusPlannned : public VehiclePlanned {
  std::vector<TrafficNode *> planned_route_;
};

#endif // CITY_BRAIN_VEHICLE_H_
