#ifndef MATCHER_H
#define MATCHER_H

#include <fstream>
#include <iostream>
#include <vector>
#include <map>
#include "utils.h"
#include "graph.h"
#include "kdtree.h"
#include "filter.h"

class Matcher
{
private:
    std::map<int, uint64_t> inter_id_to_unique_id_;
    std::vector<std::vector<point_t>> origin_track_;
    std::map<road_t, std::map<int, RoadVehicleInfo>> frame_to_road_vehicle_info_;
    std::map<road_t, int> road_to_dir_id_;
    std::vector<std::pair<std::pair<point_t, point_t>, int>> road_coord_to_dir_id_;
    CompactedGraph* graph_;
    KDTree* kdtree_;
    std::string output_path_;
    void PrintPath(const std::vector<int> &path) const;
    void PrintRoadLength(int prev_index, int index) const;
    void ProcessFilePath(std::string &input_path, std::string &output_path);
    inline void UpdateRoadVehicleInfo(road_t prev_road, road_t road, const Time &time, double avg_speed);
    void OutputCBEngineVehicleFlow(std::vector<std::pair<std::vector<int>, Time>> &vehicle_whole_path);
public:
    Matcher(std::string road_path);
    void Match(std::string vehicle_path, std::string output_path);
};

#endif // MATCHER_H
