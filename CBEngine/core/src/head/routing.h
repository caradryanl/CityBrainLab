#ifndef CITY_BRAIN_ROUTING_H_
#define CITY_BRAIN_ROUTING_H_

#include "roadnet.h"
#include "vehicle.h"
#include <iostream>
#include <map>
#include <vector>
#include <mutex>
#include <thread>

struct RoadInfo{
    const double length_;
    const int num_vehicle_;
    const double velocity_;
    RoadInfo(double length, int num_vehicle, double velocity):
        length_(length), num_vehicle_(num_vehicle), velocity_(velocity){}
};

struct VehicleInfo{
    long long unique_id_;
    std::vector<long long> path_;
    int from_;
    int to_;
    VehicleInfo(long long unique_id, std::vector<DynamicRoad *> path)
        :unique_id_(unique_id)
    {
        from_=path.front()->from_->GetID();
        for (auto i : path){
            path_.push_back(i->unique_id_);
            to_=i->to_->GetID();
        }
    }
};

class RoutingData{
public:
    const RoadNet* roadnet_;
    const VehicleGroup* vehicle_group_;
    RoutingData(const RoadNet* roadnet, const VehicleGroup* vehicle_group):
        roadnet_(roadnet), vehicle_group_(vehicle_group){}
};

class RoutingModule{
protected:
    const RoutingData *data_;
public:
    RoutingModule(RoutingData *data):data_(data){};
    ~RoutingModule(){delete data_;};
    void ConductRouting(std::vector<VehicleInfo> &);
};

#endif
