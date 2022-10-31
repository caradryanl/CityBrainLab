#include "../head/core.h"
#include "../head/common.h"
#include "../head/vehicle.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>

RoutingData* Core::PrepareRouting(){
    RoutingData* data=new RoutingData(roadnet_,vehicle_group_);
    return data;
}

void Core::ConductRouting(RoutingData *data, std::vector<VehicleInfo> &vehicle){
    RoutingModule tmp(data);
    tmp.ConductRouting(vehicle);
}

void Core::UpdateRouting(std::vector<VehicleInfo> &vehicle){
    for (std::vector<VehicleInfo>::iterator vehicle_it=vehicle.begin();vehicle_it!=vehicle.end();++vehicle_it)
    {
        VehiclePlanned* tmp=vehicle_group_->vehicle_table_.at(vehicle_it->unique_id_);
        tmp->path_.clear();
        for (std::vector<long long>::iterator path_it=vehicle_it->path_.begin();path_it!=vehicle_it->path_.end();++path_it)
        {
            tmp->path_.push_back(roadnet_->unique_id_to_road_.at(*path_it));
        }
    }
}

void GetInfoFromFile(const std::string &file_addr, std::stringstream &ss) {
    std::ifstream ifs(file_addr.c_str());
    if (ifs) {
        ss << ifs.rdbuf();
        ifs.close();
    } else {
        std::cout << "Error: File " << file_addr
                  << " not found in Core::GetInfoFromFile()\n";
    }
}

Core::Core(const Config &config,int thread_num) : config_(config), thread_num_(thread_num) {
    //std::cout<<"This is OpenEngine"<<std::endl;
    std::stringstream road_net_ss;
    GetInfoFromFile(config_.GetRoadAddr(), road_net_ss);
    //  Common::GetReplanningTheFixed() = (config_.GetReplanningTheFixed() ==
    //  "true");
    Common::GetTimer() = 0;
    Common::GetCarArriveTime() = 0.0;
    Common::GetCarArriveCounter() = 0;
    Common::seed(113);
    roadnet_ = new RoadNet(road_net_ss);
    if (config_.GetVehicleFileAddr() == "fake") {
        vehicle_group_ =
                new VehicleGroup(roadnet_, config_.GetVehicleFakeNum(), true);
        traffic_flows_ = new TrafficFlowGroup();
    } else {
        std::stringstream vehicle_ss;
        GetInfoFromFile(config_.GetVehicleFileAddr(), vehicle_ss);
        vehicle_group_ = new VehicleGroup(roadnet_,thread_num);
        vehicle_group_->InitThreads();
        traffic_flows_ = new TrafficFlowGroup(vehicle_ss, roadnet_);
        // vehicle_group_ = new VehicleGroup(roadnet_, vehicle_ss);
    }
    current_time_ = 0;
    //printf("Core created! \n\n\n\n\n\n\n\n\n\n Start!");
}

Core *Core::create_engine(const std::string &file_addr, int thread_num) {
    Config cfg = Config(file_addr);
    return new Core(cfg,thread_num);
}

void Core::Reset(){
    delete roadnet_;
    delete vehicle_group_;
    delete traffic_flows_;

    std::stringstream road_net_ss;
    GetInfoFromFile(config_.GetRoadAddr(), road_net_ss);
    //  Common::GetReplanningTheFixed() = (config_.GetReplanningTheFixed() ==
    //  "true");
    Common::GetTimer() = 0;
    Common::GetCarArriveTime() = 0.0;
    Common::GetCarArriveCounter() = 0;
    Common::seed(113);
    roadnet_ = new RoadNet(road_net_ss);
    if (config_.GetVehicleFileAddr() == "fake") {
        vehicle_group_ =
                new VehicleGroup(roadnet_, config_.GetVehicleFakeNum(), true);
        traffic_flows_ = new TrafficFlowGroup();
    } else {
        std::stringstream vehicle_ss;
        GetInfoFromFile(config_.GetVehicleFileAddr(), vehicle_ss);
        vehicle_group_ = new VehicleGroup(roadnet_, thread_num_);
        vehicle_group_->InitThreads();
        traffic_flows_ = new TrafficFlowGroup(vehicle_ss, roadnet_);
        // vehicle_group_ = new VehicleGroup(roadnet_, vehicle_ss);
    }
    current_time_ = 0;
}

Core::~Core() {  
                delete roadnet_;
                delete vehicle_group_;
              }

void Core::LogBasicInfo() {
    if (config_.GetReportlogMode() != "normal")
        return;
    std::ofstream ofs1(config_.GetReportlogAddr() + "roadinfo.json");
    ofs1 << roadnet_->LogRoadPos() << "\n";
    ofs1.close();
    std::ofstream ofs2(config_.GetReportlogAddr() + "lightinfo.json");
    ofs2 << roadnet_->LogTrafficSignalPos() << "\n";
    ofs2.close();
}

void Core::Log(int time) {
    std::cerr << "Time : " << time << std::endl;
    if (config_.GetReportlogMode() == "nolog")
        return;
    if (config_.GetReportlogMode() == "cerr") {
        std::cerr << "[" << vehicle_group_->Log() << ","
                  << roadnet_->LogTrafficSignalStatus() << "]\n";
    } else if (config_.GetReportlogMode() == "cerrcar") {
        std::cerr << vehicle_group_->Log() << "\n";
    } else if (config_.GetReportlogMode() == "normal") {
        int rate = config_.GetReportlogRate();
        if (time % rate == 0) {
            std::ofstream ofs(config_.GetReportlogAddr() + "time" +
                              std::to_string(time / rate) + ".json");
            ofs << "[" << vehicle_group_->Log(config_.GetWarningStopTimeLog()) << ","
                << roadnet_->LogTrafficSignalStatus() << ","
                << roadnet_->LogRoadVelocity() << "]\n";
            ofs.close();
        }
    } else if (config_.GetReportlogMode() == "debugsignal") {
        /*
        int rate = config_.GetReportlogRate();
        if (time % rate == 0) {
            TrafficSignal *signal = (TrafficSignal *)roadnet_->QueryNodeByUniqueID(
                        config_.GetReportLogDebugTrafficID());
            std::ofstream ofs(config_.GetReportlogAddr() + "time" +
                              std::to_string(time / rate) + ".json");
            ofs << "[" << vehicle_group_->LogDebugSignal(signal) << ","
                << roadnet_->LogDebugSignal(signal) << "]\n";
            ofs.close();
        // delete
        }*/
    }
}

void Core::LogWarning(VehicleRoadTable *table, int time) {
    for (auto road_car_pair : table->cars_on_road_) {
        long long roadid = road_car_pair.first;
        std::vector<VehiclePlanned *> cars = road_car_pair.second;
        for (VehiclePlanned *car1 : cars)
            for (VehiclePlanned *car2 : cars) {
                if (car2->unique_id_ > car1->unique_id_ &&
                        car1->pos_.lane_ == car2->pos_.lane_ &&
                        abs(car1->pos_.pos_ - car2->pos_.pos_) <= 0.1) {
                    std::cerr << "WARNING : Car #" << car1->unique_id_ << " and #"
                              << car2->unique_id_ << " are too close on road #" << roadid
                              << " at time " << time << "\n";
                    std::cerr << "Car #" << car1->unique_id_
                              << " Lane :" << car1->pos_.lane_->lane_num_
                              << " Pos : " << car1->pos_.pos_ << "\n";
                    std::cerr << "Car #" << car2->unique_id_
                              << " Lane :" << car2->pos_.lane_->lane_num_
                              << " Pos : " << car2->pos_.pos_ << "\n";
                    if (abs(car1->pos_.pos_ - car2->pos_.pos_) <= 1e-2) {
                        std::cerr << "##################### Very Close\n";
                    }
                }
            }
    }
}

void Core::SetVelocity(RoadNet *roadnet, VehicleRoadTable *table, int time) {
    for (std::pair<long long, std::vector<VehiclePlanned *>> id_car_pair :
         table->cars_on_road_) {
        DynamicRoad *road = roadnet->QueryRoadByUniqueID(id_car_pair.first);
        double tot_velocity = 0.0;
        int tot_car = id_car_pair.second.size();
        for (VehiclePlanned *vehicle : id_car_pair.second) {
            tot_velocity += vehicle->GetVelocity();
        }
        road->velocity_record_.push_back(
                    RoadVelocityRecord(time, tot_velocity / tot_car, tot_car));
    }
}

void Core::Run() {
    int start_time = config_.GetStartTimeEpoch();
    int end_time = config_.GetMaxTimeEpoch();
    int replanning_per_sec = config_.GetReplanningPerSec();
    int warning_too_close = config_.GetWarningTooClose();
    LogBasicInfo();
    for (int time = start_time; time < end_time; ++time) {
        double t1 = 1.0 * clock() / CLOCKS_PER_SEC;
        Common::GetTimer() = time;
        VehicleRoadTable *table = new VehicleRoadTable();
        vehicle_group_->Replanning(replanning_per_sec);
        vehicle_group_->SetVehicleRoadTable(table, roadnet_, current_time_);
        if (warning_too_close)
            LogWarning(table, time);
        //SetVelocity(roadnet_, table, time);
        roadnet_->UpdateVelocity(time);
        double t2 = 1.0 * clock() / CLOCKS_PER_SEC;
        roadnet_->UpdateCompactedGraph();
        double t3 = 1.0 * clock() / CLOCKS_PER_SEC;
        vehicle_group_->GoTime(1, time);
        double t4 = 1.0 * clock() / CLOCKS_PER_SEC;
        roadnet_->GoTime(1, time);
        double t5 = 1.0 * clock() / CLOCKS_PER_SEC;
        printf("time = %.3f %.3f %.3f %.3f\n", t2 - t1, t3 - t2, t4 - t3, t5 - t4);
        Log(time);
        vehicle_group_->LogInfo();
        vehicle_group_->Refresh(roadnet_);
        traffic_flows_->AddVehicles(vehicle_group_);
        delete table;
    }
    return;
}

int Core::GetVehicleCount() {
    int cnt = 0;
    for (VehiclePlanned *vehicle : vehicle_group_->vehicles_) {
        if (vehicle->status_ == VehiclePlanned::running)
            cnt++;
    }
    return cnt;
}

std::vector<int> Core::GetVehicles() {
    std::vector<int> vehicle_ids;
    for (VehiclePlanned *vehicle : vehicle_group_->vehicles_) {
        if (vehicle->status_ == VehiclePlanned::running) {
            vehicle_ids.push_back(vehicle->unique_id_);
        }
    }
    return vehicle_ids;
}
std::vector<int> Core::GetVehicles_full() {
    std::vector<int> vehicle_ids;
    for (VehiclePlanned *vehicle : vehicle_group_->vehicles_) {
        if (vehicle->status_ == VehiclePlanned::running || vehicle->status_ == VehiclePlanned::finished ) {
            vehicle_ids.push_back(vehicle->unique_id_);
        }
    }
    return vehicle_ids;
}
std::map<int, double> Core::GetVehicleSpeed() {
    std::map<int, double> vehicle_speed;
    for (VehiclePlanned *vehicle : vehicle_group_->vehicles_) {
        if (vehicle->status_ == VehiclePlanned::running) {
            vehicle_speed[vehicle->unique_id_] = vehicle->velocity_;
        }
    }
    return vehicle_speed;
}

double Core::GetCurrentTime() { return current_time_; }

void Core::NextStep() {
    Common::GetTimer() = current_time_;
    clock_t start,end;
    clock_t t_0, t_1, t_2, t_3, t_4, t_5; 
    start = clock(); 
    VehicleRoadTable *table = new VehicleRoadTable();
    end = clock(); 
    t_0 = double(end-start);
   

    // Routing
    start = clock(); 
    RoutingData* data = PrepareRouting();
    std::vector<VehicleInfo> vehicle_;
    ConductRouting(data, vehicle_);
    UpdateRouting(vehicle_);
    end = clock(); 
    t_1 = double(end-start);
    //printf("routing\n");

    //clock_t start_, end_;
    //start = clock(); 
    //start_ = clock();
    vehicle_group_->SetVehicleRoadTable(table, roadnet_, current_time_);
    //end_ = clock();
    //std::cout<<"        set vrtable:"<<double(end_-start_);
    //start_ = clock();
    //roadnet_->UpdateVelocity(current_time_);
    //roadnet_->UpdateCompactedGraph();
    //end_ = clock();
    //std::cout<<"        update vel:"<<double(end_-start_);
    //end = clock(); 
    //t_2 = double(end-start);
    //printf("set table\n");
    start = clock(); 
    vehicle_group_->GoTime(1, current_time_);
    end = clock(); 
    t_3 = double(end-start);
    //printf("vehicle go time\n");

    start = clock(); 
    roadnet_->GoTime(1, current_time_);
    end = clock(); 
    t_4 = double(end-start);
    //printf("roadnet go time\n");
    
    // vehicle_group_->LogInfo();
    start = clock(); 
    
    vehicle_group_->Refresh(roadnet_);
    traffic_flows_->AddVehicles(vehicle_group_);
    end = clock(); 
    t_5 = double(end-start);
    delete table;
    current_time_ += 1;
    
    /*
    double t = t_0 + t_1 + t_2 + t_3 + t_4 + t_5;
    std::cout<<"    Create table: "<<t_0<<" units, "<<double(t_0)/t*100<<"%"<<std::endl; 
    std::cout<<"    Routing: "<<t_1<<" units, "<<double(t_1)/t*100<<"%"<<std::endl;
    std::cout<<"    Update roadnet: "<<t_2<<" units, "<<double(t_2)/t*100<<"%"<<std::endl;
    std::cout<<"    Vehicle Go: "<<t_3<<" units, "<<double(t_3)/t*100<<"%"<<std::endl;
    std::cout<<"    Roadnet Go: "<<t_4<<" units, "<<double(t_4)/t*100<<"%"<<std::endl;
    std::cout<<"    Add vehicles: "<<t_5<<" units, "<<double(t_5)/t*100<<"%"<<std::endl;
    std::cout<<"    Total: "<<t<<std::endl;
    */
    return;
}

std::map<long long, int> Core::GetLaneVehicleCount() {
    std::map<long long, int> cnt;
    for (VehiclePlanned *vehicle : vehicle_group_->vehicles_) {
        if (vehicle->status_ == VehiclePlanned::running) {
            cnt[vehicle->pos_.lane_->lane_id_] += 1;
        }
    }
    return cnt;
}

std::map<long long, int> Core::GetLaneWaitingVehicleCount() {
    std::map<long long, int> cnt;
    for (VehiclePlanned *vehicle : vehicle_group_->vehicles_) {
        if (vehicle->status_ == VehiclePlanned::running && vehicle->velocity_ < 1.0) {
            cnt[vehicle->pos_.lane_->lane_id_] += 1;
        }
    }
    return cnt;
}

std::map<long long, std::vector<int>> Core::GetLaneVehicles() {
    std::map<long long, std::vector<int>> cnt;
    for (VehiclePlanned *vehicle : vehicle_group_->vehicles_) {
        if (vehicle->status_ == VehiclePlanned::running) {
            cnt[vehicle->pos_.lane_->lane_id_].push_back(vehicle->unique_id_);
        }
    }
    return cnt;
}

double Core::GetAverageTravelTime() {
    int tot = Common::GetCarArriveCounter();
    double tot_time = Common::GetCarArriveTime();
    for (VehiclePlanned *vehicle : vehicle_group_->vehicles_) {
        if (vehicle->status_ == VehiclePlanned::running) {
            tot += 1;
            tot_time += Common::GetTimer() - vehicle->start_time_;
        }
    }
    if (tot == 0)
        return 0.0;
    else
        return tot_time / tot;
}

std::map<std::string, std::vector<double>>
Core::GetVehicleInfo(long long vehicle_id) {
    VehiclePlanned *v = NULL;

    //  for (VehiclePlanned *vehicle : vehicle_group_->vehicles_) {
    //    if (vehicle->unique_id_ == vehicle_id) {
    //      v = vehicle;
    //      break;
    //    }
    //  }
    v = vehicle_group_->vehicle_table_[vehicle_id];
    std::map<std::string, std::vector<double>> info;
    if (v == NULL || v->status_ != VehiclePlanned::running)
        return info;
    std::vector<double> speed(1, v->velocity_);
    info["speed"] = speed;

    std::vector<double> distance(1, v->pos_.pos_);
    info["distance"] = distance;

    //  info["drivable"] = -1;
    std::vector<double> drivable(1);
    if (v->pos_.lane_ != NULL)
        drivable[0] = (v->pos_.lane_->lane_id_);
    else
        drivable[0] = (-1);
    info["drivable"] = drivable;

    std::vector<double> start_time(1);
    start_time[0] = v->start_time_;
    info["start_time"] = start_time;

    //  info["road"] = -1;
    //  if (v->pos_.road_ != NULL)
    //    info["road"] = v->pos_.road_->unique_id_;

    std::vector<double> road(1);
    if (v->pos_.road_ != NULL)
        road[0] = (v->pos_.road_->unique_id_);
    else
        road[0] = (-1);
    info["road"] = road;

    std::vector<double> route;
    for (auto r: v->path_){
        if (r == NULL)
            break;
        route.push_back(r->unique_id_);
    }
    info["route"] = route;


    // v->GetVehicleStatus(roadnet_->table);
    //  info["intersection"] = -1;
    //  if (v->vehicle_status_.next_signal_ != NULL) {
    //    info["intersection"] = v->vehicle_status_.next_signal_->unique_id_;
    //  }
    //  std::vector<double> intersection(1);
    //  if(v->vehicle_status_.next_signal_ != NULL){
    //    intersection.append(v->vehicle_status_.next_signal_->unique_id_);
    //  }
    //  else{
    //    intersection.append(-1);
    //  }

    // hide for participants
    //  std::vector<double> path;
    //  for (auto p : v->path_) {
    //    path.push_back(p->unique_id_);
    //  }
    //  info["route"] = path;

    //  std::vector<double> t_ff_vec(1, v->t_ff_);
    //  info["t_ff"] = t_ff_vec;
    return info;
}

double Core::GetRoadSpeedLimit(long long unique_id){
    DynamicRoad *road;
    try {
        road = roadnet_->QueryRoadByUniqueID(unique_id);
    } catch (std::invalid_argument e) {
        return -1;
    }
    return road -> speed_limit_;
}

void Core::SetRoadVelocity(long long unique_id, double velocity){
    DynamicRoad *road;
    try {
        road = roadnet_->QueryRoadByUniqueID(unique_id);
    } catch (std::invalid_argument e) {
        std::cout << "Cannot set the velocity of Road " << unique_id;
        return;
    }
    if (velocity < 0) {
        return;
    }
    road -> speed_limit_ = velocity;
}

int Core::SetTLPhase(long long unique_id, int phase_id) {
    TrafficNode *node;
    try {
        node = roadnet_->QueryNodeByUniqueID(unique_id);
    } catch (std::invalid_argument e) {
        return 0;
    }
    if (node->type_ != TrafficNode::signal) {
        return 0;
    }
    TrafficSignal *signal = (TrafficSignal *)node;
    //printf("This way");
    if (phase_id <= 0 || phase_id > 8)
        return 0;
    phase_id -= 1;
    static TrafficSignal::Status default_status_[8] = {
        TrafficSignal::left02, TrafficSignal::straight02,
        TrafficSignal::left13, TrafficSignal::straight13,
        TrafficSignal::all0,   TrafficSignal::all1,
        TrafficSignal::all2,   TrafficSignal::all3};
    if (default_status_[phase_id] == signal->phases_[signal->cur_phases_].first) {
        return 1;
    }
    int warning = 1;
    if (signal->road_ordered_[0] == NULL) {
        if (phase_id == 0 || phase_id == 1 || phase_id == 4)
            warning = -1;
    }
    if (signal->road_ordered_[2] == NULL) {
        if (phase_id == 0 || phase_id == 1 || phase_id == 6)
            warning = -1;
    }
    if (signal->road_ordered_[1] == NULL) {
        if (phase_id == 2 || phase_id == 3 || phase_id == 5)
            warning = -1;
    }
    if (signal->road_ordered_[3] == NULL) {
        if (phase_id == 2 || phase_id == 3 || phase_id == 7)
            warning = -1;
    }
    if (!signal->is_yellow_)
        signal->remained_time_ = 1;
    //printf("id: %d, cur_phases_ when set: %d, phase_set: %d, is yellow: %d, time: %d\n", signal->unique_id_, signal->cur_phases_, phase_id, signal->is_yellow_, signal->remained_time_);
    signal->phases_[(signal->cur_phases_ + 1) % signal->phases_.size()].first =
            default_status_[phase_id];
    
    /*  signal->cur_phases_ = phase_id;
    signal->remained_time_ = signal->phases_[phase_id].second;*/
    return warning;
}

int Core::GetTLPhase(long long unique_id) {
    TrafficNode *node;
    try {
        node = roadnet_->QueryNodeByUniqueID(unique_id);
    } catch (std::invalid_argument e) {
        return 0;
    }
    if (node->type_ != TrafficNode::signal) {
        return 0;
    }
    TrafficSignal *signal = (TrafficSignal *)node;
    return signal->phases_[signal->cur_phases_].first;
}

void Core::LogInfo(std::string file_name) {
    Core::LogBasicInfo();
    std::ofstream ofs(file_name);
    ofs << "[" << vehicle_group_->Log(config_.GetWarningStopTimeLog()) << ","
        << roadnet_->LogTrafficSignalStatus() << ","
        << roadnet_->LogRoadVelocity() << "]\n";
    ofs.close();
}


std::map<std::string, std::vector<double>>
Core::GetVehicleInfo_full(long long vehicle_id) {
    VehiclePlanned *v = NULL;
    v = vehicle_group_->vehicle_table_[vehicle_id];
    std::map<std::string, std::vector<double>> info;
    if (v == NULL)
        return info;
    std::vector<double> speed(1, v->velocity_);
    info["speed"] = speed;

    std::vector<double> distance(1, v->pos_.pos_);
    info["distance"] = distance;

    //  info["drivable"] = -1;
    std::vector<double> drivable(1);
    if (v->pos_.lane_ != NULL)
        drivable[0] = (v->pos_.lane_->lane_id_);
    else
        drivable[0] = (-1);
    info["drivable"] = drivable;

    std::vector<double> start_time(1);
    start_time[0] = v->start_time_;
    info["start_time"] = start_time;

    std::vector<double> road(1);
    if (v->pos_.road_ != NULL)
        road[0] = (v->pos_.road_->unique_id_);
    else
        road[0] = (-1);
    info["road"] = road;


    // hide for participants
    std::vector<double> path;
    for (auto p : v->path_) {
        path.push_back(p->unique_id_);
    }
    info["route"] = path;

    std::vector<double> t_ff_vec(1, v->t_ff_);
    info["t_ff"] = t_ff_vec;

    std::vector<double> step(1,v->last_step_);
    info["step"] = step;
    return info;
}
void Core::LogVehicleInfo(std::string file_name) {
    std::ofstream ofs(file_name);
    ofs<<Core::GetVehicleCount()<<"\n";
    std::vector<int> vehicles = Core::GetVehicles_full();
    for(int vehicle : vehicles){
        std::map<std::string, std::vector<double>> vehicle_dict = Core::GetVehicleInfo_full(vehicle);
        ofs<<"for vehicle "<<vehicle<<"\n";

        ofs<<"distance : "<<vehicle_dict["distance"][0]<<"\n";
        ofs<<"drivable : "<<vehicle_dict["drivable"][0]<<"\n";
        ofs<<"road : "<<vehicle_dict["road"][0]<<"\n";
        ofs<<"route :";
        for(double road_in_path : vehicle_dict["route"]){
            ofs<<" " <<road_in_path;
        }
        ofs<<"\n";
        ofs<<"speed : "<<vehicle_dict["speed"][0]<<"\n";
        ofs<<"start_time : "<<vehicle_dict["start_time"][0]<<"\n";
        ofs<<"t_ff : "<<vehicle_dict["t_ff"][0]<<"\n";
        ofs<<"step : "<<vehicle_dict["step"][0]<<"\n";
        ofs<<"-----------------"<<"\n";
    }
    ofs.close();
}

void Core::DebugSignal(long long unique_id) {
    TrafficNode *node = roadnet_->QueryNodeByUniqueID(unique_id);
    fprintf(stderr, "Debuging Traffic Node: %lld\n", unique_id);
    if (node->type_ != TrafficNode::signal) {
        fprintf(stderr, "WARNING : Traffic Node is not a signal.");
    }
    TrafficSignal *signal = (TrafficSignal *)node;
    fprintf(stderr, "Remained time: %d\n", (int)signal->remained_time_);
    fprintf(stderr, "Is yellow: %d\n", (int)signal->is_yellow_);
    fprintf(stderr, "Now phase: %d\n",signal->phases_[signal->cur_phases_].first);
}

std::map<std::string, std::vector<double>>
Core::GetVehicleState(long long vehicle_id){
    VehiclePlanned *v = NULL;
    v = vehicle_group_->vehicle_table_[vehicle_id];
    std::map<std::string, std::vector<double>> info;
    if (v == NULL)
        return info;

    //VehiclePosition
    std::vector<double> pos(3);
    pos[0] = v->pos_.pos_;
    pos[1] = v->pos_.road_->unique_id_;
    pos[2] = v->pos_.lane_->lane_id_;
    info["pos"] = pos;

    //base info
    std::vector<double> base_info(5);
    base_info[0] = v -> velocity_;
    base_info[1] = v -> max_acc_;
    base_info[2] = v -> min_acc_;
    base_info[3] = v -> unique_id_;
    base_info[4] = int(v -> vehicle_type_);
    info["base_info"] = base_info;

    //planned info
    //from & to
    std::vector<double> from_to(2);
    from_to[0] = v -> from_ -> unique_id_;
    from_to[1] = v -> to_ -> unique_id_;
    info["from_to"] = from_to;
    //safety_dist
    std::vector<double> safety_dist(3);
    safety_dist[0] = v->safety_distance_min_;
    safety_dist[1] = v->safety_distance_max_;
    safety_dist[2] = v->safety_distance_;
    info["safety_dist"] = safety_dist;
    //path
    std::vector<double> path;
    for (auto p : v->path_) {
        path.push_back(p->unique_id_);
    }
    info["path"] = path;

    //other
    std::vector<double> t_ff_(1, v->t_ff_);
    info["t_ff_"] = t_ff_;
    std::vector<double> debug_time_(1, v->debug_time_);
    info["debug_time_"] = debug_time_;
    std::vector<double> start_time_(1, v->t_ff_);
    info["start_time_"] = start_time_;
    std::vector<double> last_step_(1, v->last_step_);
    info["last_step_"] = last_step_;
    std::vector<double> status_(1, v->status_);
    info["status_"] = status_;
    std::vector<double> action_type_(1, v->action_type_);
    info["action_type_"] = action_type_;
    std::vector<double> stop_time_(1, v->stop_time_);
    info["stop_time_"] = stop_time_;
    std::vector<double> is_fixed_(1, v->is_fixed_);
    info["is_fixed_"] = is_fixed_;

    return info;
}

void Core::SaveState(std::string file_name) {
    std::ofstream ofs(file_name);
    ofs << std::fixed;
    ofs << current_time_ << '\n';

    // Save the state of vehicle group
    std::vector<int> vehicles = Core::GetVehicles_full();
    ofs << vehicles.size() << '\n';
    for(int vehicle : vehicles){
        std::map<std::string, std::vector<double>> vehicle_dict = Core::GetVehicleState(vehicle);
        ofs<<vehicle<<"\n";
        // pos
        ofs<<vehicle_dict["pos"][0]<<" ";
        ofs<<vehicle_dict["pos"][1]<<" ";
        ofs<<vehicle_dict["pos"][2]<<"\n";
        // base_info
        ofs<<vehicle_dict["base_info"][0]<<" ";
        ofs<<vehicle_dict["base_info"][1]<<" ";
        ofs<<vehicle_dict["base_info"][2]<<" ";
        ofs<<vehicle_dict["base_info"][3]<<" ";
        ofs<<vehicle_dict["base_info"][4]<<"\n";
        // fromto
        ofs<<vehicle_dict["from_to"][0]<<" ";
        ofs<<vehicle_dict["from_to"][1]<<"\n";

        // safety_dist
        ofs<<vehicle_dict["safety_dist"][0]<<" ";
        ofs<<vehicle_dict["safety_dist"][1]<<" ";
        ofs<<vehicle_dict["safety_dist"][2]<<"\n";

        // path
        ofs << vehicle_dict["path"].size() << '\n';
        for(double road_in_path : vehicle_dict["path"]){
            ofs<<road_in_path<<" " ;}
        ofs << '\n';
        // other info
        ofs<<vehicle_dict["t_ff_"][0]<<" ";
        ofs<<vehicle_dict["debug_time_"][0]<<" ";
        ofs<<vehicle_dict["start_time_"][0]<<" ";
        ofs<<vehicle_dict["last_step_"][0]<<" ";
        ofs<<vehicle_dict["status_"][0]<<" ";
        ofs<<vehicle_dict["action_type_"][0]<<" ";
        ofs<<vehicle_dict["stop_time_"][0]<<" ";
        ofs<<vehicle_dict["is_fixed_"][0]<<"\n";
        ofs<<"\n";
    }
    ofs.close();
}

void Core::LoadVehicle(int vehicle_id, std::map<std::string, std::vector<double>> info){
    //printf("id: %d", vehicle_id);
    try{
        TrafficNode *from, *to;
        {
            std::unique_lock<std::mutex> lock(roadnet_mutex);
            from = roadnet_->QueryNodeByUniqueID((long long)info["from_to"][0]);
            to = roadnet_->QueryNodeByUniqueID((long long)info["from_to"][1]);
        }
        VehiclePlanned *v = new VehiclePlanned(from, to, info["base_info"][3], 0);
        v->velocity_ = info["base_info"][0];
        v->max_acc_ = info["base_info"][1];
        v->min_acc_ = info["base_info"][2];
        v->stop_time_ = info["stop_time_"][0];
        v->debug_time_ = info["debug_time_"][0];
        v->start_time_ = info["start_time_"][0];
        v->is_fixed_ = bool(info["is_fixed_"][0]);
        v->safety_distance_min_ = info["safety_dist"][0];
        v->safety_distance_max_ = info["safety_dist"][1];
        v->safety_distance_ = info["safety_dist"][2];
        v ->path_.clear();

        for (int j=0; j<info["path"].size(); j++){
            DynamicRoad *road;
            {
                std::unique_lock<std::mutex> lock(roadnet_mutex);
                road = roadnet_->QueryRoadByUniqueID(info["path"][j]);
            }
            v->path_.push_back(road);
        }
        v -> pos_ = VehiclePosition(v->path_[0], info["pos"][1], v->path_[0]->GetLane((long long)(info["pos"][2]) % 100));
        v -> SetActionStatus(info["action_type_"][0], info["status_"][0]);
        {
            std::unique_lock<std::mutex> lock(vehicle_group_mutex);
            vehicle_group_ -> AddVehicle(v);
        }

    } catch (std::invalid_argument e) {
        std::cout << "Cannot construct vehicle "<< (long long)vehicle_id;
    }
}

void Core::LoadState(std::string file_name) {
    std::ifstream ifs(file_name);
    ifs >> current_time_;

    int vehicle_num;
    int num, vehicle_id;
    double buf;
    int status;
    //int tot = 0;
    ifs >> vehicle_num;

    //ThreadPool threads_(500);
    const int kThread = 200;
    std::thread threadpool[kThread+1];
    int num_threads_used = 0;

    for (int i=0; i<vehicle_num; i++){
        std::map<std::string, std::vector<double>> info;
        ifs >> vehicle_id;
        // if (status == 2) continue; // If it is finished, not do it.
        //std::cout << "Load Vehicle: "<< i << '\n';
        //VehiclePosition
        std::vector<double> pos(3);
        ifs >> pos[0] >> pos[1] >> pos[2];
        info["pos"] = pos;

        //base info
        std::vector<double> base_info(5);
        ifs >> base_info[0] >> base_info[1] >> base_info[2] >> base_info[3] >> base_info[4];
        info["base_info"] = base_info;

        //planned info
        //from & to
        std::vector<double> from_to(2);
        ifs >> from_to[0] >> from_to[1];
        info["from_to"] = from_to;
        //safety_dist
        std::vector<double> safety_dist(3);
        ifs >> safety_dist[0] >> safety_dist[1] >> safety_dist[2];
        info["safety_dist"] = safety_dist;
        //path
        std::vector<double> path;
        ifs >> num;
        for (int j=0; j<num; j++) {
            ifs >> buf;
            path.push_back(buf);
        }
        info["path"] = path;

        //other
        std::vector<double> t_ff_(1);
        ifs >> t_ff_[0];
        info["t_ff_"] = t_ff_;
        std::vector<double> debug_time_(1);
        ifs >> debug_time_[0];
        info["debug_time_"] = debug_time_;
        std::vector<double> start_time_(1);
        ifs >> start_time_[0];
        info["start_time_"] = start_time_;
        std::vector<double> last_step_(1);
        ifs >> last_step_[0];
        info["last_step_"] = last_step_;
        std::vector<double> status_(1);
        ifs >> status_[0];
        info["status_"] = status_;
        std::vector<double> action_type_(1);
        ifs >> action_type_[0];
        info["action_type_"] = action_type_;
        std::vector<double> stop_time_(1);
        ifs >> stop_time_[0];
        info["stop_time_"] = stop_time_;
        std::vector<double> is_fixed_(1);
        ifs >> is_fixed_[0];
        info["is_fixed_"] = is_fixed_;

        if (info["status_"][0]==2.0)
            continue;

        //Construct VehiclePlanned
        //threads_.enqueue(&Core::LoadVehicle, this, vehicle_id, info);

        threadpool[num_threads_used++] = std::thread{&Core::LoadVehicle, this, vehicle_id, info};
        if (num_threads_used == kThread){
            for (int i = 0; i < kThread; i++)
                threadpool[i].join();
            num_threads_used = 0;
        }

    }
    for (int i = 0; i < num_threads_used; i++)
        threadpool[i].join();
    
    Common::GetTimer() = current_time_;
    VehicleRoadTable *table = new VehicleRoadTable();
    vehicle_group_->SetVehicleRoadTable(table, roadnet_, current_time_);
    //SetVelocity(roadnet_, table, current_time_);
    roadnet_->UpdateVelocity(current_time_);
    roadnet_->UpdateCompactedGraph();

}
std::vector<double> Core::GetDrivingParams(){
    DrivingModule tmp = DrivingModule();
    return tmp.GetModuleParams();
}

void Core::SetDrivingParams(std::vector<double> params){
    DrivingModule tmp = DrivingModule();
    tmp.SetModuleParams(params);
}

// Serve for Congestion Pricing
std::vector<long long> Core::GetVehicleRoute(long long int vehicle_id){
    // Get the vehicle
    std::vector<long long> route;
    VehiclePlanned *v = vehicle_group_->vehicle_table_[vehicle_id];
    for (auto road: v->path_)
        route.push_back(road->unique_id_);
    return route;
}

void Core::SetVehicleRoute(long long int vehicle_id, std::vector<long long> route){
    // Get the vehicle
    VehiclePlanned *v = vehicle_group_->vehicle_table_[vehicle_id];
    // access the route: v->path_[i]->unique_id_

    // Check if route starts from the present location of the vehicle
    if (route[0] != v->path_[0]->unique_id_){
        throw std::invalid_argument("Route not starting from the current loc."); 
    }

    // Check if route is connected
    DynamicRoad *pre=nullptr, *pro=nullptr;
    for (int i=0; i<route.size()-1; i++){
        pre = roadnet_->QueryRoadByUniqueID(route[i]);
        pro = roadnet_->QueryRoadByUniqueID(route[i+1]);
        if (pre->to_->unique_id_ != pro->from_->unique_id_)
            throw std::invalid_argument("Route not connected."); 
    }

    // Set route for the vehicle
    v->path_.clear();
    for (int i=0; i<route.size(); i++){
        v->path_.push_back(roadnet_->QueryRoadByUniqueID(route[i]));
    }
}

