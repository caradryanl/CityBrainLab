#include "../head/roadnet.h"
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <queue>
#include <set>

RoadNet::RoadNet(std::string rawData) { fromString(rawData); }

RoadNet::RoadNet(std::stringstream &buf) { fromStream(buf); }

void RoadNet::AddNodeFromStream(std::stringstream &buf, int id) {
  double longitude=0, latitude=0;
  long long unique_id=0;
  bool is_signal=0;
  buf >> longitude;
  buf >> latitude;
  buf >> unique_id;
  buf >> is_signal;

  if (!is_signal) {
    TrafficNode *new_node = new TrafficNode(longitude, latitude, unique_id, id);
    if (unique_id_to_node_.count(unique_id)) {
      printf("Node id: %lld", unique_id);
      throw std::invalid_argument("Node id is not unique");
    }
    unique_id_to_node_[unique_id] = new_node;
    nodes_.push_back(new_node);
  } else {
    TrafficNode *new_node =
        new TrafficSignal(longitude, latitude, unique_id, id, -1, -1, -1, -1);
    if (unique_id_to_node_.count(unique_id)) {
      printf("Node id: %lld", unique_id);
      throw std::invalid_argument("Node (with signal) id is not unique");
    }
    unique_id_to_node_[unique_id] = new_node;
    nodes_.push_back(new_node);
  }
}

void RoadNet::AddRoadFromStream(std::stringstream &buf) {
  double length;
  double speed_limit;
  int num_of_lane1, num_of_lane2;
  long long unique_id1, unique_id2;
  long long id1, id2;
  buf >> id1 >> id2 >> length >> speed_limit >> num_of_lane1 >> num_of_lane2 >>
      unique_id1 >> unique_id2;
  if (!unique_id_to_node_.count(id1)) {
    printf("Node id1: %lld", id1);
    throw std::invalid_argument("Can't find the node id1");
  }
  if (!unique_id_to_node_.count(id2)) {
    printf("Node id2: %lld", id2);
    throw std::invalid_argument("Can't find the node id2");
  }
  if (unique_id_to_road_.count(unique_id1)) {
    printf("Road id1: %lld", unique_id1);
    throw std::invalid_argument("Road id1 is not unique");
  }
  if (unique_id_to_road_.count(unique_id2)) {
    printf("Road id2: %lld", unique_id2);
    throw std::invalid_argument("Road id2 is not unique");
  }
  AddRoadAndLaneFromStream(unique_id_to_node_[id1], unique_id_to_node_[id2],
                           length, speed_limit, num_of_lane1, num_of_lane2,
                           unique_id1, unique_id2, buf);
}

void RoadNet::fromStream(std::stringstream &buf) {
  int num_nodes, num_roads;
  buf >> num_nodes;
  nodes_.clear();
  roads_.clear();

  for (int i = 0; i < num_nodes; i++)
    AddNodeFromStream(buf, i);
  buf >> num_roads;
  for (int i = 0; i < num_roads; i++)
    AddRoadFromStream(buf);
  int num_signals;
  buf >> num_signals;
  for (int i = 0; i < num_signals; i++) {
    long long node_id;
    buf >> node_id;
    TrafficNode *node = QueryNodeByUniqueID(node_id);
    for (int j = 0; j < 4; j++) {
      long long road_id;
      buf >> road_id;
      if (road_id != -1) {
        dynamic_cast<TrafficSignal *>(node)->AddRoadOrdered(
            QueryRoadByUniqueID(road_id));
      } else {
        dynamic_cast<TrafficSignal *>(node)->AddRoadOrdered(NULL);
      }
    }
  }
  UpdateCompactedGraph();
  for (TrafficNode *node : nodes_) {
    if (node->GetRoad().size() == 1) {
      border_nodes_.push_back(node);
    }
    /*if (node->GetRoad().size() > 2 && node->type_ != TrafficNode::signal) {
      throw std::invalid_argument("cnm");
    }*/
  }
}

void RoadNet::fromString(std::string raw_data) {
  std::stringstream buf;
  buf << raw_data;
  fromStream(buf);
}
// bug: 
std::string RoadNet::toString() {
  std::stringstream buf;
  buf << nodes_.size() << " ";
  for (TrafficNode *node : nodes_) {
    buf << node->toString();
  }
  if (roads_.size() % 2 != 0) {
    throw std::invalid_argument("Road is not paired");
  }
  buf << roads_.size() / 2 << " ";
  for (int i = 0; i < roads_.size(); i += 2) {
    // roads[i] and roads[i+1] are reverse edges, only num_of_lane are different
    Road *road1 = roads_[i];
    Road *road2 = roads_[i + 1];
    if (road1->rev_road_ != road2 || road2->rev_road_ != road1) {
      // throw std::invalid_argument("Road is not paired");
      i -= 1; // one-way-street
    }
    buf << road1->toStringBidirected();
  }
  return buf.str();
}

void RoadNet::AddRoadAndLaneFromStream(TrafficNode *des1, TrafficNode *des2,
                                       double length, double speed_limit,
                                       int num_of_lane1, int num_of_lane2,
                                       long long unique_id1,
                                       long long unique_id2,
                                       std::stringstream &buf) {  
  // unique_id <= 0 means no such way, the road is a one-way street
  DynamicRoad *road1 = new DynamicRoad(des1, des2, length, speed_limit,
                                      num_of_lane1, unique_id1);
  for (int i = 0; i < num_of_lane1; i++)
    road1->AddLaneFromStream(buf, i);

  DynamicRoad *road2 = new DynamicRoad(des2, des1, length, speed_limit,
                                      num_of_lane2, unique_id2);
  for (int i = 0; i < num_of_lane2; i++)
    road2->AddLaneFromStream(buf, i);
  
  if (unique_id1 >= 0 && unique_id2 >= 0){
    road1->SetRevRoad(road2);
    road2->SetRevRoad(road1);
    des1->AddRoad(road1);
    des2->AddRoad(road2);
    roads_.push_back(road1);
    roads_.push_back(road2);
    unique_id_to_road_[unique_id1] = road1;
    unique_id_to_road_[unique_id2] = road2;
    vertex_id_to_road_[{des1->GetID(), des2->GetID()}] = road1;
    vertex_id_to_road_[{des2->GetID(), des1->GetID()}] = road2;
  }
  else if (unique_id2 >= 0){
    delete road1;
    road2->SetRevRoad(nullptr);
    des2->AddRoad(road2);
    roads_.push_back(road2);
    unique_id_to_road_[unique_id2] = road2;
    vertex_id_to_road_[{des2->GetID(), des1->GetID()}] = road2;
  }
  else if (unique_id1 >= 0){
    delete road2;
    road1->SetRevRoad(nullptr);
    des1->AddRoad(road1);
    roads_.push_back(road1);
    unique_id_to_road_[unique_id1] = road1;
    vertex_id_to_road_[{des1->GetID(), des2->GetID()}] = road1;
  }
}


std::vector<DynamicRoad *> RoadNet::FindShortestPathSPFA(TrafficNode *from,
                                                         TrafficNode *to) {
  int n = nodes_.size();
  std::vector<double> dis(n, 1e30);
  std::vector<bool> vis(n, false);
  std::vector<DynamicRoad *> pre(n, NULL);
  int start = from->GetID();
  dis[start] = 0;
  std::queue<int> q;
  q.push(start);
  vis[start] = true;
  // fprintf(stderr,"ss %d\n",start);
  while (!q.empty()) {
    int u = q.front();
    q.pop();
    for (DynamicRoad *road : nodes_[u]->GetRoad()) {
      int v = road->to_->GetID();
      if (dis[v] > dis[u] + road->GetPassTime()) {
        pre[v] = road;
        dis[v] = dis[u] + road->GetPassTime();
        if (!vis[v]) {
          vis[v] = true;
          q.push(v);
        }
      }
    }
    vis[u] = false;
  }
  if (dis[to->GetID()] >= 1e20) {
    throw std::invalid_argument("Graph are disconnected");
  }
  std::vector<DynamicRoad *> path;
  while (to != from) {
    path.push_back(pre[to->GetID()]);
    to = (TrafficNode *)pre[to->GetID()]->from_;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<DynamicRoad *>
RoadNet::FindShortestPath(TrafficNode *from, TrafficNode *to, bool is_spfa) {
  puts("slow");
  if (is_spfa)
    return FindShortestPathSPFA(from, to);
  if (from == NULL)
    throw std::invalid_argument("From is empty in RoadNet::FindShortestPath()");
  if (to == NULL)
    throw std::invalid_argument("To is empty in RoadNet::FindShortestPath()");
  int n = nodes_.size();
  std::vector<double> dis(n, 1e30);
  std::vector<bool> vis(n, false);
  std::set<std::pair<double, int>> pq;
  std::vector<DynamicRoad *> pre(n, NULL);
  int start = from->GetID();
  dis[start] = 0;
  for (int i = 0; i < n; i++)
    pq.insert({dis[i], i});
  for (int i = 0; i < n; i++) {
    int u = pq.begin()->second;
    pq.erase(pq.begin());
    vis[u] = true;
    for (DynamicRoad *road : nodes_[u]->GetRoad()) {
      int v = road->to_->GetID();
      if (dis[v] > dis[u] + road->GetPassTime()) {
        pq.erase({dis[v], v});
        pre[v] = road;
        dis[v] = dis[u] + road->GetPassTime();
        pq.insert({dis[v], v});
      }
    }
  }
  if (dis[to->GetID()] >= 1e20) {
    throw std::invalid_argument("Graph are disconnected");
  }
  std::vector<DynamicRoad *> path;
  while (to != from) {
    path.push_back(pre[to->GetID()]);
    to = (TrafficNode *)pre[to->GetID()]->from_;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<DynamicRoad *> RoadNet::FindShortestPathCompacted(TrafficNode *from,
                                                              TrafficNode *to,
                                                              bool is_spfa) {
  int from_id = from->GetID();
  int to_id = to->GetID();
  std::vector<int> path =
      compacted_graph_.FindShortestPath(from_id, to_id, is_spfa);
  std::vector<DynamicRoad *> path_road;
  for (int i = 0; i + 1 < path.size(); i++)
    path_road.push_back(QueryRoadByVertexID(path[i], path[i + 1]));
  return path_road;
}

TrafficNode *RoadNet::QueryNodeByUniqueID(long long unique_id) {
  if (unique_id_to_node_.count(unique_id)) {
    return unique_id_to_node_[unique_id];
  } else {
    throw std::invalid_argument(
        "No such node in RoadNet::QueryNodeByUniqueID()");
    return NULL;
  }
}

DynamicRoad *RoadNet::QueryRoadByUniqueID(long long unique_id) {
  if (unique_id_to_road_.count(unique_id)) {
    return unique_id_to_road_[unique_id];
  } else {
    printf("road id: %lld", unique_id);
    throw std::invalid_argument(
        "No such road in RoadNet::QueryRoadByUniqueID()");
    return NULL;
  }
}

DynamicRoad *RoadNet::QueryRoadByVertexID(int from_id, int to_id) {
  if (vertex_id_to_road_.count({from_id, to_id})) {
    return vertex_id_to_road_[{from_id, to_id}];
  } else {
    throw std::invalid_argument(
        "No such road in RoadNet::QueryRoadByVertexID()");
    return NULL;
  }
}

TrafficNode *RoadNet::RandNode() {
  return nodes_[Common::RandomInt(0, nodes_.size() - 1)];
}
std::pair<TrafficNode *, TrafficNode *> RoadNet::RandNodePairs() {
  const int kProbBorder = 20;
  if (Common::RandomInt(0, 99) < kProbBorder && !border_nodes_.empty()) {
    return std::make_pair(
        border_nodes_[Common::RandomInt(0, (int)border_nodes_.size() - 1)],
        border_nodes_[Common::RandomInt(0, (int)border_nodes_.size() - 1)]);
  } else {
    return std::make_pair(RandNode(), RandNode());
  }
}

void RoadNet::GoTime(double sec, double start_sec) {
  /*
    start_sec: the starting time stamp of GoTime op
    sec: the time span to step forward
  */
  for (TrafficNode *node : nodes_) {
    node->GoTime(sec, start_sec);
    if (node->type_ == TrafficNode::signal) {
      TrafficSignal *signal = (TrafficSignal *)node;
      /*for (int i = 0; i < 4; i++)
        for (int j = 0; j < 2; j++) {
          signal->turn_cnt_[i][j] = 0;
        }*/
    }
  }
}

RoadNet::~RoadNet() {}

std::string RoadNet::LogTrafficSignalPos() {
  std::stringstream ss;
  ss << std::setprecision(12);
  for (TrafficNode *node : nodes_) {
    if (node->type_ == TrafficNode::signal) {
      ss << "[" << node->latitude_ << "," << node->longitude_ << "],";
    }
  }
  std::string ss_log = ss.str();
  if (!ss_log.empty())
    ss_log.pop_back();
  return "[" + ss_log + "]";
}

std::string RoadNet::LogTrafficSignalStatus() {
  std::stringstream ss;
  ss << std::setprecision(12);
  for (TrafficNode *node : nodes_) {
    if (node->type_ == TrafficNode::signal) {
      TrafficSignal *signal = dynamic_cast<TrafficSignal *>(node);
      if (signal->is_yellow_) {
        ss << "0";
      }
      else if(signal->GetStatus()==TrafficSignal::left02){
        ss << "1";
      }
      else if(signal->GetStatus()==TrafficSignal::straight02){
        ss << "2";
      }
      else if(signal->GetStatus()==TrafficSignal::left13){
        ss << "3";
      }
      else if(signal->GetStatus()==TrafficSignal::straight13){
        ss << "4";
      }
      else if(signal->GetStatus()==TrafficSignal::all0){
        ss << "5";
      }
      else if(signal->GetStatus()==TrafficSignal::all1){
        ss << "6";
      }
      else if(signal->GetStatus()==TrafficSignal::all2){
        ss << "7";
      }
      else if(signal->GetStatus()==TrafficSignal::all3){
        ss << "8";
      }
//      if(signal->is_yellow_){
//        ss << "2";
//      }
//      else if (signal->GetStatus() == TrafficSignal::left13 ||
//                 signal->GetStatus() == TrafficSignal::straight13) {
//        ss << "1";
//      } else {
//        ss << "0";
//      }
      ss << ",";
    }
  }
  std::string ss_log = ss.str();
  if (!ss_log.empty())
    ss_log.pop_back();
  return "[" + ss_log + "]";
}

std::string RoadNet::LogDebugSignal(TrafficSignal *signal) {
  std::stringstream ss;
  ss << std::setprecision(12);
  ss << "[[" << signal->latitude_ << "," << signal->longitude_ << ","
     << ((signal->GetStatus() == TrafficSignal::straight13 ||
          signal->GetStatus() == TrafficSignal::straight02)
             ? "1"
             : "0")
     << "]],";
  int type = -1;
  if (signal->GetStatus() == TrafficSignal::straight02 ||
      signal->GetStatus() == TrafficSignal::left02) {
    type = 0;
  } else {
    type = 1;
  }
  ss << "[";
  bool is_first = true;
  for (int i = 0; i < 4; i++) {
    if (i % 2 != type || signal->road_ordered_[i] == NULL)
      continue;
    if (!is_first) {
      ss << ",";
    } else {
      is_first = false;
    }
    ss << signal->road_ordered_[i]->Log();
  }
  ss << "]";
  return ss.str();
}

std::string RoadNet::LogRoadPos() {
  std::stringstream ss;
  ss << std::setprecision(12);
  for(TrafficNode *node :nodes_){
    if(node->type_ == TrafficNode::signal){
      int id = -1;
      for(DynamicRoad *road : dynamic_cast<TrafficSignal *>(node)->road_ordered_){
        id+=1;
        if(road == NULL){
          continue;
        }
        ss << (road->GetRevRoad())->Log(id) << ",";
      }
    }
    else{
      for(DynamicRoad *road : (node)->road_){
        if(road == NULL){
          continue;
        }
        ss << (road->GetRevRoad())->Log() << ",";
      }
    }
  }
//  for (DynamicRoad *road : roads_) {
//    ss << road->Log() << ",";
//  }
  std::string ss_log = ss.str();
  if (!ss_log.empty())
    ss_log.pop_back();
  return "[" + ss_log + "]";
}

std::string RoadNet::LogRoadVelocity() {
  std::stringstream ss;
  ss << std::setprecision(12);
  for(TrafficNode *node :nodes_){
    if(node->type_ == TrafficNode::signal){
      int id = -1;
      for(DynamicRoad *road : dynamic_cast<TrafficSignal *>(node)->road_ordered_){
        id+=1;
        if(road == NULL){
          continue;
        }
        ss << ((DynamicRoad *)(road->GetRevRoad())) ->velocity_ << ",";
      }
    }
    else{
      for(DynamicRoad *road : (node)->road_){
        if(road == NULL){
          continue;
        }
        ss << ((DynamicRoad *)(road->GetRevRoad())) ->velocity_ << ",";
      }
    }
  }
//  for (DynamicRoad *road : roads_) {
//    ss << road->velocity_ << ",";
//  }
  std::string ss_log = ss.str();
  if (!ss_log.empty())
    ss_log.pop_back();
  return "[" + ss_log + "]";
}

void RoadNet::UpdateVelocity(int time) {
  const int kKeepTime = 20;
  for (DynamicRoad *road : roads_) {
    while (!road->velocity_record_.empty() &&
           road->velocity_record_.front().record_time_ < time - kKeepTime) {
      road->velocity_record_.pop_front();
    }
    if (road->velocity_record_.empty()) {
      road->velocity_ = road->speed_limit_;
      road->num_vehicle_ = 0;
    } else {
      double tot_velocity = 0.0;
      int tot_vehicle = 0;
      for (RoadVelocityRecord record : road->velocity_record_) {
        tot_velocity += record.ave_velocity_;
        tot_vehicle += 1;
      }
      road->velocity_ = tot_velocity / road->velocity_record_.size();
      road->num_vehicle_ = tot_vehicle;
    }
  }
}

void RoadNet::UpdateCompactedGraph() {
  std::vector<std::pair<std::pair<int, int>, double>> edges;
  for (DynamicRoad *road : roads_) {
    edges.push_back(
        {{road->from_->GetID(), road->to_->GetID()}, road->GetPassTime()});
  }
  compacted_graph_.InitFromEdgeList(edges, (int)nodes_.size());
}
