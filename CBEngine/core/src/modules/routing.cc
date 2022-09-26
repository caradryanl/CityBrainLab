#include "../head/routing.h"

void RoutingModule::ConductRouting(std::vector<VehicleInfo> &vehicle)
{
//    for (std::vector<VehicleInfo>::iterator vehicle_it=vehicle.begin();vehicle_it!=vehicle.end();++vehicle_it)
//    {
//        bool is_spfa=true;
//        int from_id = vehicle_it->from_;
//        int to_id = vehicle_it->to_;
//        std::vector<int> path =
//                data_->compacted_graph_->FindShortestPath(from_id, to_id, is_spfa);
//        vehicle_it->path_.clear();
//        for (int i = 0; i + 1 < path.size(); i++){
//            std::map<std::pair<int,int>,long long>::const_iterator path_it=data_->vertex_road_table_.find(std::pair<int,int>(path[i],path[i+1]));
//            if (path_it==data_->vertex_road_table_.end())
//            {
//                throw std::invalid_argument(
//                            "No such road in RoadNet::QueryRoadByVertexID()");
//            }
//            long long path_id=path_it->second;
//            vehicle_it->path_.push_back(path_id);
//        }
//    }
}
