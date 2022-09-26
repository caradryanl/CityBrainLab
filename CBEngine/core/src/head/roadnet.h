#ifndef CITY_BRAIN_ROADNET_H_
#define CITY_BRAIN_ROADNET_H_
#include "common.h"
#include "compacted_graph.h"
#include <algorithm>
#include <deque>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

class TrafficNode;

class Lane {
public:
  bool turn_left_, go_straight_, turn_right_;
  int lane_num_;
  long long lane_id_;

public:
  Lane(bool turn_left, bool go_straight, bool turn_right, int lane_num,
       long long lane_id);
};

class BusLane : public Lane {
public:
  bool can_go_others_;
};

class Road {
public:
  long long unique_id_;
  Road *rev_road_;
  std::vector<Lane *> lanes_;

public:
  TrafficNode *from_, *to_;
  double length_, speed_limit_;
  int num_of_lane_;

  Road(TrafficNode *from, TrafficNode *to, double length, double speed_limit_,
       int num_of_lane, long long unique_id);
  void AddLaneFromStream(std::stringstream &buf, int lane_num);
  void SetRevRoad(Road *rev_road);
  Road *GetRevRoad();
  long long GetUniqueID();
  Lane *GetLane(int lane_num);
  std::string toStringBidirected();
  std::string Log(int id = -1);
  friend class RoadNet;
};

class RoadVelocityRecord {
public:
  double record_time_;
  double ave_velocity_;
  int cars_;
  RoadVelocityRecord(double record_time, double ave_velocity, int cars);
};

class DynamicRoad : public Road {
public:
  double velocity_;
  std::deque<RoadVelocityRecord> velocity_record_;
  DynamicRoad();
  DynamicRoad(TrafficNode *from, TrafficNode *to, double length,
              double speed_limit_, int num_of_lane, long long unique_id);
  double GetPassTime();
public:
  int num_vehicle_;
};

class TrafficNode {
private:

  int id_;

public:
  std::vector<DynamicRoad *> road_;
  const double longitude_, latitude_;
  const long long unique_id_;
  enum Type { normal, signal };
  Type type_;
  TrafficNode(double longitude, double latitude, long long unique_id, int id);
  virtual ~TrafficNode();
  void AddRoad(DynamicRoad *road);
  DynamicRoad *GetBackRoad(DynamicRoad *road);
  DynamicRoad *GetFrontRoad(DynamicRoad *road);
  DynamicRoad *GetRandomFrontRoad(DynamicRoad *road, std::mt19937 &rnd);
  int GetID() const;
  virtual bool CanGo(DynamicRoad *from_, DynamicRoad *to_);
  virtual void GoTime(double sec, double start_sec = 0.0);
  std::vector<DynamicRoad *> &GetRoad();
  std::string toString();
};

class TrafficSignal : public TrafficNode {
public:
  enum Status {
    left02,
    straight02,
    left13,
    straight13,
    all0,
    all1,
    all2,
    all3
  };
  static Status constexpr default_status_[8] = {left02, straight02, left13, straight13,
                                      all0,   all1,       all2,   all3};
  // int turn_cnt_[4][2];

public:
  std::vector<DynamicRoad *> road_ordered_;
  std::vector<std::pair<Status, double>> phases_;
  // Status status_;
  int remained_time_, cur_phases_;
  /*int red_left_period_, red_straight_period_, green_left_period_,
      green_straight_period_;*/
  int yellow_period_;
  bool is_yellow_;

public:
  TrafficSignal(double longitude, double latitude, long long unique_id, int id,
                double red_left_period, double red_straight_period,
                double green_left_period, double green_straight_period);
  Status GetStatus();
  double GetRemainedTime();
  bool CanGo(DynamicRoad *from, DynamicRoad *to);
  void GetSignalInfo(DynamicRoad *from, DynamicRoad *to, bool &can_go,
                     double &remained_time);
  enum DirectionType { left, right, straight, uturn };
  DirectionType DirectionRelation(DynamicRoad *from, DynamicRoad *to);
  void GoTime(double sec, double start_sec = 0.0);
  void AddRoadOrdered(DynamicRoad *road);
  int GetFromID(DynamicRoad *from);
};

class RoadNet {
public:
  std::vector<TrafficNode *> nodes_;
  std::vector<TrafficNode *> border_nodes_;
  std::vector<DynamicRoad *> roads_;
  std::map<long long, TrafficNode *> unique_id_to_node_;
  std::map<long long, DynamicRoad *> unique_id_to_road_;
  std::map<std::pair<int, int>, DynamicRoad *> vertex_id_to_road_;
  CompactedGraph compacted_graph_;
  void AddRoadAndLaneFromStream(TrafficNode *des1, TrafficNode *des2,
                                double length, double speed_limit_,
                                int num_of_lane1, int num_of_lane2,
                                long long unique_id1, long long unique_id2,
                                std::stringstream &buf);
  void AddNodeFromStream(std::stringstream &buf, int id);
  void AddRoadFromStream(std::stringstream &buf);
  void fromString(std::string raw_data);
  void fromStream(std::stringstream &buf);

public:
  RoadNet(std::string rawData);
  RoadNet(std::stringstream &buf);
  ~RoadNet();
  std::vector<DynamicRoad *> FindShortestPathSPFA(TrafficNode *from,
                                                  TrafficNode *to);
  std::vector<DynamicRoad *>
  FindShortestPath(TrafficNode *from, TrafficNode *to, bool is_spfa = true);
  std::vector<DynamicRoad *> FindShortestPathCompacted(TrafficNode *from,
                                                       TrafficNode *to,
                                                       bool is_spfa = true);
  void UpdateCompactedGraph();
  void UpdateVelocity(int time);
  std::string toString();
  TrafficNode *QueryNodeByUniqueID(long long unique_id);
  TrafficNode *RandNode();
  std::pair<TrafficNode *, TrafficNode *> RandNodePairs();
  DynamicRoad *QueryRoadByUniqueID(long long unique_id);
  DynamicRoad *QueryRoadByVertexID(int from_id, int to_id);
  void GoTime(double sec, double start_sec = 0.0);
  std::string LogTrafficSignalPos();
  std::string LogTrafficSignalStatus();
  std::string LogRoadVelocity();
  std::string LogRoadPos();
  std::string LogDebugSignal(TrafficSignal *signal);

};

class VehiclePosition {
public:
  DynamicRoad *road_;
  Lane *lane_;
  double pos_;
  VehiclePosition();
  // VehiclePosition(DynamicRoad *road, double pos);
  VehiclePosition(DynamicRoad *road, double pos, Lane *lane);
  VehiclePosition(std::stringstream &buf, RoadNet *roadnet);
  void LaneChange(int lane_delta);
  void LaneSet(int lane_delta);
  void GetLatLon(double &lat, double &lon);
  std::string toString();
};

#endif // CITY_BRAIN_ROADNET_H_