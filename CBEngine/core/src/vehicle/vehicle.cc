#include "../head/vehicle.h"
#include "../head/common.h"
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <set>
#include <mutex>

void VehiclePlanned::SetPath(const VehiclePosition pos, TrafficNode *to,
                             RoadNet *roadnet) {
    status_ = running;
    path_ = roadnet->FindShortestPathCompacted((TrafficNode *)pos.road_->to_, to);
    path_.insert(path_.begin(), (DynamicRoad *)pos.road_);
    velocity_ = 0;
}

void VehiclePlanned::SetPathInit(RoadNet *roadnet) {
    path_ = roadnet->FindShortestPathCompacted(from_, to_);
    if (path_.empty()) {
        status_ = finished;
    } else {
        status_ = running;
        pos_ = VehiclePosition(path_[0], 0.0, path_[0]->GetLane(0));
        pos_.pos_ = pos_.road_->length_ *
                (double)Common::RandomInt(0, (1 << 30) - 1) / (double)(1 << 30);
    }
}

void VehiclePlanned::SetActionStatus(int action, int status){
    action_type_ = ActionType(action);
    status_ = Status(status);
}

void VehiclePlanned::Replanning(RoadNet *roadnet) {
    if (status_ != running)
        return;
    if (is_fixed_ && Common::GetReplanningTheFixed())
        return;
    TrafficNode *from = path_.at(0)->from_;
    auto pre_path = roadnet->FindShortestPathCompacted(path_.at(0)->to_, to_);
    pre_path.insert(pre_path.begin(), path_.at(0));
    bool have_uturn = false;
    for (int i = 0; i + 1 < pre_path.size(); i++) {
        auto road1 = pre_path.at(i);
        auto road2 = pre_path.at(i + 1);
        if (road1->to_->type_ == TrafficNode::signal)
            break;
        if (road1->GetRevRoad() == road2) {
            have_uturn = true;
        }
    }
    if (!have_uturn) {
        path_ = pre_path;
        if (pre_path.empty())
            CarArrived();
        return;
    }
    int walk_cnt = 0;
    std::vector<DynamicRoad *> path;
    TrafficNode *node = NULL;
    bool dead_end = false;
    while (true) {
        path.clear();
        node = NULL;
        dead_end = false;
        DynamicRoad *road = path_[0];
        while (true) {
            path.push_back(road);
            node = road->to_;
            if (node->type_ == TrafficNode::signal || node == to_)
                break;
            road = node->GetRandomFrontRoad(road, rnd_);
            if (road == NULL) {
                dead_end = true;
                break;
            }
        }
        if (node == to_) {
            path_ = path;
            return;
        }
        ++walk_cnt;
        if (node != from || walk_cnt >= 10)
            break;
    }
    auto shortest_path = roadnet->FindShortestPathCompacted(node, to_);
    for (auto road : shortest_path)
        path.push_back(road);
    path_ = path;
    for (int i = 0; i + 1 < path_.size(); i++) {
        auto road1 = path_[i];
        auto road2 = path_[i + 1];
        if (road1->to_->type_ == TrafficNode::signal || dead_end)
            break;
        if (road1->GetRevRoad() == road2) {
            throw std::invalid_argument(
                        "Didn't fix UTurrn in VehiclePlanned::Replanning()");
        }
    }
}

void VehiclePlanned::FixUTurn(RoadNet *roadnet) {
    Replanning(roadnet);
    return;
}

VehiclePlanned::VehiclePlanned(TrafficNode *from, TrafficNode *to,
                               long long unique_id, RoadNet *roadnet,
                               bool find_path) {
    from_ = from;
    to_ = to;
    unique_id_ = unique_id;
    velocity_ = 0;
    //max_acc_ = 2.0;
    max_acc_ = 5.0;
    min_acc_ = 5.0;
    stop_time_ = 0;
    debug_time_ = 0;
    start_time_ = Common::GetTimer();
    is_fixed_ = false;
    rnd_ = std::mt19937(Common::RandomInt(0, 100000000));
    if (find_path == false) {
        status_ = unplanned;
    } else {
        path_ = roadnet->FindShortestPathCompacted(from, to);
        if (path_.empty()) {
            status_ = finished;
        } else {
            status_ = running;
            pos_ = VehiclePosition(path_[0], 0.0, path_[0]->GetLane(0));
        }
    }
}

VehiclePlanned::VehiclePlanned(TrafficNode *from, TrafficNode *to,
                               long long unique_id, int placeholder) {
    from_ = from;
    to_ = to;
    unique_id_ = unique_id;
    velocity_ = 0;
    //max_acc_ = 2.0;
    max_acc_ = 5.0;
    min_acc_ = 5.0;
    stop_time_ = 0;
    debug_time_ = 0;
    start_time_ = Common::GetTimer();
    is_fixed_ = false;
    rnd_ = std::mt19937(Common::RandomInt(0, 100000000));
}

VehiclePlanned::VehiclePlanned(std::vector<DynamicRoad *> path,
                               long long unique_id, bool is_fixed) {
    if (path.empty()) {
        throw std::invalid_argument(
                    "Path is empty in VehiclePlanned::VehiclePlanned()");
    }
    from_ = path[0]->from_;
    to_ = path.back()->to_;
    unique_id_ = unique_id;
    velocity_ = 0;

    //max_acc_ = 2.0;
    max_acc_ = 5.0;
    min_acc_ = 5.0;
    stop_time_ = 0;
    debug_time_ = 0;
    start_time_ = Common::GetTimer();
    is_fixed_ = is_fixed;
    path_ = path;
    safety_distance_ = Common::RandomInt(safety_distance_min_,safety_distance_max_);
    // added in 3.31
    double t_ff = 0.0;
    for (auto p : path) {
        t_ff += p->length_ / p->speed_limit_;
    }
    t_ff_ = t_ff;
    // added in 6.11
    last_step_ = Common::GetTimer();
    rnd_ = std::mt19937(Common::RandomInt(0, 100000000));
    status_ = running;
    pos_ = VehiclePosition(path_[0], 0.0, path_[0]->GetLane(0));
}

VehiclePlanned::VehiclePlanned(std::stringstream &buf, RoadNet *roadnet,
                               bool find_path) {
    int from_id, to_id;
    std::string status;
    long long unique_id;
    buf >> from_id >> to_id;
    buf >> status;
    buf >> unique_id;
    TrafficNode *from = roadnet->QueryNodeByUniqueID(from_id);
    TrafficNode *to = roadnet->QueryNodeByUniqueID(to_id);
    new (this) VehiclePlanned(from, to, unique_id, roadnet, false);
    if (status == "UNPLANNED") {
        if (find_path == false) {
            status = unplanned;
        } else {
            status = running;
            path_ = roadnet->FindShortestPathCompacted(from_, to_);
            if (path_.empty()) {
                status_ = finished;
            } else {
                status_ = running;
                pos_ = VehiclePosition(path_[0], 0.0, path_[0]->GetLane(0));
                velocity_ = 0;
            }
        }
    } else if (status == "FINISHED") {
        status = finished;
    } else if (status == "RUNNING") {
        pos_ = VehiclePosition(buf, roadnet);
        SetPath(pos_, to_, roadnet);
    } else {
        throw std::invalid_argument("unknown status");
    }
}

std::string VehiclePlanned::toString() {
    std::stringstream buf;
    buf << from_->unique_id_ << " ";
    buf << to_->unique_id_ << " ";
    if (status_ == unplanned) {
        buf << "UNPLANNED ";
    } else if (status_ == finished) {
        buf << "FINISHED ";
    } else {
        buf << "RUNNING ";
        buf << pos_.toString();
    }
    buf << unique_id_;
    return buf.str();
}

void VehiclePlanned::Reset(TrafficNode *from, TrafficNode *to,
                           RoadNet *roadnet) {
    from_ = from;
    to_ = to;
    velocity_ = 0;
    //max_acc_ = 2.0;
    max_acc_ = 5.0;
    min_acc_ = 5.0;
    stop_time_ = 0;
    debug_time_ = 0;
    start_time_ = Common::GetTimer();
    is_fixed_ = false;
    rnd_ = std::mt19937(Common::RandomInt(0, 100000000));
    path_ = roadnet->FindShortestPathCompacted(from, to);
    if (path_.empty()) {
        status_ = finished;
    } else {
        status_ = running;
        pos_ = VehiclePosition(path_[0], 0.0, path_[0]->GetLane(0));
    }
}

double VehiclePlanned::GetVelocity() const{
    if (status_ == finished || status_ == unplanned) {
        return 0.0;
    } else if (pos_.road_ != NULL) {
        return velocity_;
    } else {
        return 0.0;
    }
}
VehiclePosition VehiclePlanned::GetVehiclePosition() const{ return pos_; }

void VehiclePlanned::GetNextCarOnLane(VehicleRoadTable *table, int lane_num,
                                      VehiclePlanned *&next_car,
                                      double &dis_to_next_car,
                                      bool tie_breaker) const{
    const double kDisThread = 50;
    next_car = NULL;
    if (lane_num < 0 || lane_num >= path_.at(0)->num_of_lane_) {
        dis_to_next_car = 0;
        return;
    }    
    dis_to_next_car = -pos_.pos_;
    for (int i = 0; i < 1/*path_.size()*/; i++) {
        DynamicRoad *road = path_[i];
        if (next_car == NULL && lane_num >= 0 && lane_num < road->num_of_lane_) {
            double tpos = 0.0;
            if (i == 0)
                tpos = pos_.pos_;
            VehiclePlanned *car = table->GetNextVehicleOnLane(
                        road->unique_id_, lane_num, tpos, tie_breaker, unique_id_);
            if (car != NULL)
                next_car = car;
            if (next_car != NULL) {
                dis_to_next_car += next_car->pos_.pos_;
                break;
            }
        }
        if (next_car == NULL)
            dis_to_next_car += road->length_;
        if (dis_to_next_car > kDisThread) {
            break;
        }
    }
    if (next_car == NULL)
        dis_to_next_car = 0;
}

int VehiclePlanned::GetCarNumOnLaneToSignal(VehicleRoadTable *table,
                                            int lane_num) const{
    int cnt = 0;
    for (int i = 0; i < path_.size(); i++) {
        DynamicRoad *road = path_[i];
        double tpos = -1.0;
        if (i == 0)
            tpos = pos_.pos_;
        cnt += table->CountNumberVehicleOnLane(road->unique_id_, i, tpos);
        if (road->to_->type_ == TrafficNode::signal)
            break;
    }
    return cnt;
}

void VehiclePlanned::GetLastCarOnLane(VehicleRoadTable *table, int lane_num,
                                      VehiclePlanned *&last_car,
                                      double &dis_to_last_car) const{
    const double kDisThread = 50;
    if (lane_num < 0 || lane_num >= path_.at(0)->num_of_lane_) {
        last_car = NULL;
        dis_to_last_car = 0;
        return;
    }
    DynamicRoad *road = path_[0];
    last_car = NULL;
    dis_to_last_car = pos_.pos_ - road->length_;
    while (true) {
        if (last_car == NULL) {
            double tpos = road->length_ + 1.0;
            if (road == path_.at(0))
                tpos = pos_.pos_;
            VehiclePlanned *car = table->GetLastVehicleOnLane(
                        road->unique_id_, lane_num, tpos, false, unique_id_);
            if (car != NULL)
                last_car = car;
            dis_to_last_car += road->length_;
            if (last_car != NULL) {
                dis_to_last_car -= last_car->GetVehiclePosition().pos_;
                break;
            }
        }
        // Trace back
        TrafficNode *node = road->from_;
        if (node->type_ == TrafficNode::signal)
            break;
        road = node->GetBackRoad(road);
        if (road == NULL)
            break;
        if (dis_to_last_car > kDisThread) {
            break;
        }
    }
    if (last_car == NULL) {
        dis_to_last_car = 0;
    }
}
/*
void VehiclePlanned::GetNextLastCarOnLane(VehicleRoadTable *table, int lane_num,
                                      VehiclePlanned *&next_car, VehiclePlanned *&last_car,
                                      double &dis_to_next_car, double &dis_to_last_car, bool tie_breaker) const{

    next_car = NULL;
    last_car = NULL;
    if (lane_num < 0 || lane_num >= path_.at(0)->num_of_lane_) {
        dis_to_next_car = 0;
        dis_to_last_car = 0;
        return;
    }
    DynamicRoad *road = path_[0];
    dis_to_next_car = -pos_.pos_;
    dis_to_last_car = pos_.pos_ - road->length_;
    if (last_car == NULL) {
        double tpos = road->length_ + 1.0;
        if (road == path_.at(0))
            tpos = pos_.pos_;
        VehiclePlanned *car = table->GetLastVehicleOnLane(
                    road->unique_id_, lane_num, tpos, false, unique_id_);
        dis_to_last_car += road->length_;
        if (car != NULL){
            last_car = car;
            dis_to_last_car -= last_car->pos_.pos_;
        }
    }
    if (next_car == NULL && lane_num >= 0 && lane_num < road->num_of_lane_) {
        double tpos = pos_.pos_;
        VehiclePlanned *car = table->GetNextVehicleOnLane(
                    road->unique_id_, lane_num, tpos, tie_breaker, unique_id_);
        if (car != NULL){
            next_car = car;
            dis_to_next_car += next_car->pos_.pos_;
        }
    }
    if (next_car == NULL)
        dis_to_next_car += road->length_;
    if (last_car == NULL) {
        dis_to_last_car = 0;
    }                                     
}*/

void VehicleStatus::SetSignalInfo(VehiclePlanned *car) {
    dis_to_signal_ = -car->pos_.pos_;
    next_signal_ = NULL;
    bool find_signal = false;
    for (int i = 0; i < car->path_.size(); i++) {
        DynamicRoad *road = car->path_[i];
        dis_to_signal_ += road->length_;
        if (i + 1 == car->path_.size())
            break;
        if (road->to_->type_ == TrafficNode::signal) {
            ((TrafficSignal *)road->to_)
                    ->GetSignalInfo(car->path_[i], car->path_[i + 1], signal_can_go_,
                    signal_remained_time_);
            // if (car->unique_id_ <= 10)
            //  printf("cnmmmmm %lld %f\n", car->unique_id_, signal_remained_time_);
            TrafficSignal::DirectionType type =
                    ((TrafficSignal *)road->to_)
                    ->DirectionRelation(car->path_[i], car->path_[i + 1]);
            if (type == TrafficSignal::straight) {
                next_signal_direction_ = VehicleStatus::straight;
            } else if (type == TrafficSignal::left) {
                next_signal_direction_ = VehicleStatus::left;
            } else if (type == TrafficSignal::right) {
                next_signal_direction_ = VehicleStatus::right;
            } else if (type == TrafficSignal::uturn) {
                // TODO : should be U-turn
                next_signal_direction_ = VehicleStatus::uturn;
            } else {
                throw std::invalid_argument(
                            "Unknown direction in VehicleStatus::SetSignalInfo()");
            }

            find_signal = true;
            next_signal_ = (TrafficSignal *)road->to_;
            if (car->velocity_ <= 0.1 && dis_to_signal_ <= 250 &&
                    next_signal_ != NULL) {
                int from_id = next_signal_->GetFromID(car->path_[i]);
                if (next_signal_direction_ == VehicleStatus::straight) {
                    // next_signal_->turn_cnt_[from_id][0]++;
                } else if (next_signal_direction_ == VehicleStatus::left ||
                           next_signal_direction_ == VehicleStatus::uturn) {
                    // next_signal_->turn_cnt_[from_id][1]++;
                }
            }

            break;
        }
    }
    if (!find_signal) {
        next_signal_direction_ = VehicleStatus::destination;
        signal_remained_time_ = 1e9;
    }
}

// signal_remained_time 1e9 means destination
//                      -2 means go right, always possible to go
//                      -1 means unknown

void VehiclePlanned::GetVehicleStatus(VehicleRoadTable *table, VehicleStatus &vstatus){
    //VehicleStatus vstatus;
    //double t_0 = clock();
    vstatus.dis_to_signal_ = 0;
    vstatus.signal_can_go_ = false;
    vstatus.signal_remained_time_ = 1e9;
    if (status_ == finished) {
        vstatus.next_signal_direction_ = VehicleStatus::finished;
        //status = vstatus;
    }
    if (status_ == unplanned) {
        throw std::invalid_argument("The car is not planned.");
    }
    //double t_1 = clock();
    GetNextCarOnLane(table, pos_.lane_->lane_num_, vstatus.next_car_,
                     vstatus.dis_to_next_car_, true);
    /*
    GetNextLastCarOnLane(table, pos_.lane_->lane_num_ - 1, vstatus.left_next_car_,
                     vstatus.left_last_car_, vstatus.dis_to_left_next_car_,
                     vstatus.dis_to_left_last_car_);
    GetNextLastCarOnLane(table, pos_.lane_->lane_num_ - 1, vstatus.right_next_car_,
                     vstatus.right_last_car_, vstatus.dis_to_right_next_car_,
                     vstatus.dis_to_right_last_car_);*/
    GetNextCarOnLane(table, pos_.lane_->lane_num_ - 1, vstatus.left_next_car_,
                     vstatus.dis_to_left_next_car_);
    GetNextCarOnLane(table, pos_.lane_->lane_num_ + 1, vstatus.right_next_car_,
                     vstatus.dis_to_right_next_car_);
    GetLastCarOnLane(table, pos_.lane_->lane_num_ - 1, vstatus.left_last_car_,
                     vstatus.dis_to_left_last_car_);
    GetLastCarOnLane(table, pos_.lane_->lane_num_ + 1, vstatus.right_last_car_,
                     vstatus.dis_to_right_last_car_);
    vstatus.this_car_=this;
    //double t_2 = clock();
    vstatus.SetSignalInfo(this);
    //double t_3 = clock();
    for (int i = 0; i < pos_.road_->num_of_lane_; i++) {
        vstatus.num_of_car_to_light_.push_back(GetCarNumOnLaneToSignal(table, i));
        //vstatus.num_of_car_to_light_.push_back(5);
    }
    //double t_4 = clock();
    //printf("%.3f, %.3f, %.3f, %.3f, %.3f\n", t_1-t_0, t_2-t_1, t_3-t_2, t_4-t_3, t_4-t_0);
    //status = vstatus;
}

VehicleTurnStatus
VehiclePlanned::GetVehicleTurnStatus(VehicleRoadTable *table) {
    VehicleTurnStatus vstatus;
    vstatus.dis_to_signal_ = 0;
    vstatus.signal_can_go_ = false;
    vstatus.signal_remained_time_ = 1e9;
    if (status_ == finished) {
        vstatus.next_signal_direction_ = VehicleStatus::finished;
        return vstatus;
    }
    if (status_ == unplanned) {
        throw std::invalid_argument(
                    "The car is not planned in VehiclePlanned::GetVehicleTurnStatus()");
    }
    for (int i = 0; i < pos_.road_->num_of_lane_; i++) // for each lane in the road
    {
        VehiclePlanned *vehicle = NULL;
        double dis = 0;
        GetNextCarOnLane(table, i, vehicle, dis);   // get the next dis and vehicle prepared
        vstatus.next_car_lane_.push_back(vehicle);
        vstatus.dis_next_car_lane_.push_back(dis);
    }
    vstatus.SetSignalInfo(this);    // update other features of vstatus, based on vehicle
    return vstatus;
}

bool VehiclePlanned::IsSameDirection(VehicleStatus::Direction next_signal_direction, Lane *lane) const{
    if ((next_signal_direction == VehicleStatus::left ||
         next_signal_direction == VehicleStatus::uturn) &&
            lane->turn_left_)
        return true;
    if (next_signal_direction == VehicleStatus::right &&
            lane->turn_right_)
        return true;
    if (next_signal_direction == VehicleStatus::straight &&
            lane->go_straight_)
        return true;
    return false;
}

double VehiclePlanned::CalcStopDis(double velocity, double acc) const{
    double down_time = ceil(velocity / acc);
    if (velocity - down_time * acc < 0)
        down_time -= 1;
    // down_time == 0
    return (velocity + velocity - down_time * acc) * (down_time + 1) / 2;
}

bool VehiclePlanned::CanStop(double front_velocity, double back_velocity,
                             double distance, double eps = 0,
                             double safe_distance = 20.) const{
    // front_velocity: v of the car before self
    // back_velocity: v of self
    // distance: distance from self to the car before self
    safe_distance = 5;

    double tmp = back_velocity - front_velocity;
    if (tmp < 0)
        return true;

    // Because they have the same acceleration, the velocity of the back car
    // (self) is always larger than that of the front car. Thus, we simply compare
    // the stop distance of them.
    double back_dis = CalcStopDis(back_velocity, min_acc_);
    double front_dis = CalcStopDis(front_velocity, min_acc_);

    return back_dis + safe_distance < front_dis + distance + eps;
}

double VehiclePlanned::GetMaxVelocityNew(double initial_velocity,
                                         double next_car_velocity,
                                         double dis_to_next_car) const{

    double max_velocity = pos_.road_->speed_limit_;

    double safe_distance = std::max(5.0, 2 * initial_velocity);
    if (dis_to_next_car < safe_distance)
        return -10000;
    safe_distance = 5;

    double min_velocity = // next_car_velocity;
            std::max(0.0, next_car_velocity - min_acc_);

    double distance_remaining = dis_to_next_car - safe_distance;

    next_car_velocity = min_velocity;

    double ceil_velocity = ceil(next_car_velocity / min_acc_) * min_acc_;
    double time_to_stop = floor(ceil_velocity / min_acc_ + 0.5);
    if (time_to_stop < 0.5) {
        time_to_stop = 1;
        ceil_velocity = min_acc_;
    }
    if (!(ceil_velocity - next_car_velocity * time_to_stop >=
          distance_remaining)) {

        distance_remaining -= (ceil_velocity - next_car_velocity) * time_to_stop;
        // cv~cv+acc: * (tts + 1), cv+acc~cv+2acc: * (tts + 2),  ...
        // max x such that acc * \sum_{i=1..x}(tts + i) <= dr
        // tts * x + x * (x + 1) / 2 <= dr / acc
        // 0.5x^2+(tts+0.5)*x - dr/acc <= 0
        double B = time_to_stop + 0.5;
        double x = -B + sqrt(B * B + 2 * distance_remaining / min_acc_);
        x = floor(x);
        distance_remaining -= min_acc_ * (time_to_stop * x + x * (x + 1) / 2);
        ceil_velocity += x * min_acc_;
        time_to_stop = floor(ceil_velocity / min_acc_ + 0.5) + 1;

    } else
        ceil_velocity = next_car_velocity;
    double ans = ceil_velocity + distance_remaining / time_to_stop;

    double Left = std::max(0.0, initial_velocity - min_acc_);
    double Right = std::min(initial_velocity + max_acc_, max_velocity);
    if (ans < Left)
        return -10000;
    return std::min(std::max(ans, Left), Right) - initial_velocity;
}

double VehiclePlanned::GetMinVelocityNew(double initial_velocity,
                                         double last_car_velocity,
                                         double dis_to_last_car,
                                         double safe_distance = 20) const{
    double max_velocity = pos_.road_->speed_limit_;
    double cur_max_velocity =
            std::min(max_velocity, last_car_velocity + max_acc_);
    //    std::max(0.0, last_car_velocity - min_acc_);
    // temp version std::min(last_car_velocity + max_acc_, max_velocity);
    double Left = std::max(0.0, initial_velocity - min_acc_);
    double Right = std::min(initial_velocity + max_acc_, max_velocity);

    // aggressive strategy  safe_distance = std::max(5.0, 2 * last_car_velocity);

    safe_distance = std::max(5.0, 2.0 * last_car_velocity);
    if (dis_to_last_car < safe_distance)
        return 10000;

    // double Mid = (Left + Right) / 2.;
    if (!CanStop(Right, cur_max_velocity, dis_to_last_car, 1e-5, safe_distance))
        return 10000;

    safe_distance = 5;

    double back_up = initial_velocity;

    initial_velocity = std::min(last_car_velocity + max_acc_, max_velocity);
    ;

    double dis_remaining = dis_to_last_car - safe_distance;

    double floor_velo = floor(initial_velocity / min_acc_) * min_acc_;

    double time_to_stop = floor(floor_velo / min_acc_ + 0.5) + 1;

    double res;
    if ((initial_velocity - floor_velo) * time_to_stop > dis_remaining) {
        res = initial_velocity - (dis_remaining) / time_to_stop;
    } else {

        dis_remaining -= (initial_velocity - floor_velo) * time_to_stop;
        initial_velocity = floor_velo;
        // max x such that (tts - 1) * minacc + ... + (tts - x) * minacc <=
        // dis_remaining
        if ((time_to_stop - 1) * time_to_stop / 2 * min_acc_ <= dis_remaining) {
            res = 0;
        } else {
            // minacc * (2 * tts - 1 - x) * x / 2 <= dis_remain
            //-min_acc / 2 * x^2 + minacc * (2 * tts - 1) / 2 * x - dis_remain <= 0
            double A = -min_acc_ / 2, B = min_acc_ / 2 * (2 * time_to_stop - 1),
                    C = -dis_remaining;
            double ub = (-B + sqrt(B * B - 4 * A * C)) / 2 / A;
            ub = floor(ub);
            dis_remaining -= min_acc_ * (2 * time_to_stop - 1 - ub) * ub / 2;
            initial_velocity -= ub * min_acc_;
            time_to_stop -= 1 + ub;
            res = initial_velocity - (dis_remaining) / time_to_stop;
        }
    }
    return std::min(Right, std::max(Left, res)) - back_up;
}

double VehiclePlanned::GetMaxVelocityOld(double initial_velocity,
                                         double next_car_velocity,
                                         double dis_to_next_car) const{
    double max_velocity = pos_.road_->speed_limit_;

    double safe_distance = std::max(5.0, 2 * initial_velocity);

    if (dis_to_next_car < safe_distance)
        return -10000;

    double min_velocity = // next_car_velocity;
            std::max(0.0, next_car_velocity - min_acc_);

    double Left = std::max(0.0, initial_velocity - min_acc_);
    double Right = std::min(initial_velocity + max_acc_, max_velocity);
    double Mid = (Left + Right) / 2.;

    if (!CanStop(min_velocity, Left, dis_to_next_car, 1e-5)) {
        // std::cerr << "Sile" << std::endl;
        return -10000;
    }

    for (int i = 1; i <= 100; i++, Mid = (Left + Right) / 2.)
        if (CanStop(min_velocity, Mid, dis_to_next_car))
            Left = Mid;
        else
            Right = Mid;

    return Left - initial_velocity;
}

double VehiclePlanned::GetMinVelocityOld(double initial_velocity,
                                         double last_car_velocity,
                                         double dis_to_last_car,
                                         double safe_distance = 20) const{
    double max_velocity = pos_.road_->speed_limit_;
    // double cur_max_velocity = last_car_velocity;
    double cur_max_velocity =
            std::min(max_velocity, last_car_velocity + max_acc_);

    //    std::max(0.0, last_car_velocity - min_acc_);
    // temp version std::min(last_car_velocity + max_acc_, max_velocity);
    double Left = std::max(0.0, initial_velocity - min_acc_);
    double Right = std::min(initial_velocity + max_acc_, max_velocity);

    // aggressive strategy  safe_distance = std::max(5.0, 2 * last_car_velocity);

    safe_distance = std::max(5.0, 2.0 * last_car_velocity);

    if (dis_to_last_car < safe_distance)
        return 10000;

    double Mid = (Left + Right) / 2.;
    if (!CanStop(Right, cur_max_velocity, dis_to_last_car, 1e-5, safe_distance))
        return 10000;

    for (int i = 1; i <= 100; i++, Mid = (Left + Right) / 2.) {
        if (CanStop(Mid, cur_max_velocity, dis_to_last_car, 0, safe_distance))
            Right = Mid;
        else
            Left = Mid;
    }
    return Right - initial_velocity;
}

double VehiclePlanned::GetMaxVelocity(double initial_velocity,
                                      double next_car_velocity,
                                      double dis_to_next_car) const{
    //    double rold = GetMaxVelocityOld(initial_velocity, next_car_velocity,
    //    dis_to_next_car);
    double rnew =
            GetMaxVelocityNew(initial_velocity, next_car_velocity, dis_to_next_car);
    //    if(abs(rnew - rold) > 1e-6) {
    //        printf("GETMAX old = %lf new = %lf\n", rold, rnew);
    //    }
    return rnew;
}

double VehiclePlanned::GetMinVelocity(double initial_velocity,
                                      double last_car_velocity,
                                      double dis_to_last_car,
                                      double safe_distance = 20) const{
    //    double rold = GetMinVelocityOld(initial_velocity, last_car_velocity,
    //    dis_to_last_car, safe_distance);
    double rnew = GetMinVelocityNew(initial_velocity, last_car_velocity,
                                    dis_to_last_car, safe_distance);
    //    if(abs(rnew - rold) > 1e-6) {
    //        printf("GETMIN old = %lf new = %lf\n", rold, rnew);
    //    }
    return rnew;
}

double VehiclePlanned::GetCutinDistance(VehicleStatus::Direction next_signal_direction) const{
    double dist_threshold = 200;
    if (next_signal_direction == VehicleStatus::right)
        dist_threshold = 200;
    if (next_signal_direction == VehicleStatus::straight)
        dist_threshold = 300;
    if (next_signal_direction == VehicleStatus::left ||
            next_signal_direction == VehicleStatus::uturn)
        dist_threshold = 400;
    return dist_threshold;
}

std::vector<VehicleAction>
VehiclePlanned::GetTurnAction(VehicleTurnStatus vehicle_status) {
    VehicleAction vehicle_action;
    vehicle_action.action_type_ = VehicleAction::lane_turn;
    std::vector<double> dis_next_car_lane_ = vehicle_status.dis_next_car_lane_;
    std::vector<VehiclePlanned *> next_car_lane = vehicle_status.next_car_lane_;
    int len = dis_next_car_lane_.size();
    double tmp = 0;
    int pos = 0;

    for (int i = 0; i < len; i++)
        if (next_car_lane[i] == NULL) {
            pos = i;
            tmp = 1;
            break;
        } else if (dis_next_car_lane_[i] > tmp) {
            tmp = dis_next_car_lane_[i];
            pos = i;
        }
    if (tmp < 1e-7) {
        vehicle_action.lane_turn_ = 0;
        // std::cerr << "dusile\n";
    }
    if(vehicle_status.next_signal_direction_ == VehicleStatus::left||vehicle_status.next_signal_direction_ == VehicleStatus::uturn){
        pos = 0;
    }
    else if(vehicle_status.next_signal_direction_ == VehicleStatus::straight){
        pos = 1;
    }
    else if(vehicle_status.next_signal_direction_ == VehicleStatus::right){
        pos = 2;
    }

    vehicle_action.lane_turn_ = pos;
    std::vector<VehicleAction> res;
    res.clear();
    res.push_back(vehicle_action);
    return res;
}

void VehiclePlanned::DoActions(std::vector<VehicleAction> vehicle_actions) {
    for (VehicleAction action : vehicle_actions) {
        if (action.action_type_ == VehicleAction::lane_change) {
            pos_.LaneChange(action.lane_delta_);
        } else if (action.action_type_ == VehicleAction::accelerate) {
            if (action.velocity_delta_ > max_acc_ + 0.1) {
                throw std::invalid_argument(
                            "Accleration is too large in VehiclePlanned::DoActions()");
            }
            velocity_ += action.velocity_delta_;
        } else if (action.action_type_ == VehicleAction::decelerate) {
            if (action.velocity_delta_ > min_acc_ + 0.1) {
                action.velocity_delta_ = std::max(action.velocity_delta_, -min_acc_);
                // printf("%.10f %.10f %lld\n", velocity_, action.velocity_delta_,
                //  unique_id_);
                // throw std::invalid_argument(
                //    "Decleration is too large in VehiclePlanned::DoActions()");
            }
            velocity_ -= action.velocity_delta_;
            if (velocity_ < -0.1) {
                // printf("%.10f %.10f\n", velocity_, action.velocity_delta_);
                throw std::invalid_argument(
                            "The speed is negative in VehiclePlanned::DoActions()");
            }
            velocity_ = std::max(0.0, velocity_);
        } else if (action.action_type_ == VehicleAction::lane_turn) {
            pos_.LaneSet(action.lane_turn_);
        }
    }
}

void VehiclePlanned::UpdateNextSignalDirection(){
    for (int i = 0; i < path_.size(); i++) {
        DynamicRoad *road = path_[i];
        if (i + 1 == path_.size()){
            next_signal_direction_ = VehicleStatus::destination;
            break;
        }
        if (road->to_->type_ == TrafficNode::signal) {
            TrafficSignal::DirectionType type =
                    ((TrafficSignal *)road->to_)
                    ->DirectionRelation(path_[i], path_[i + 1]);
            if (type == TrafficSignal::straight) {
                next_signal_direction_ = VehicleStatus::straight;
            } else if (type == TrafficSignal::left) {
                next_signal_direction_ = VehicleStatus::left;
            } else if (type == TrafficSignal::right) {
                next_signal_direction_ = VehicleStatus::right;
            }
            break;
        }
    }
}

void VehiclePlanned::PassTrafficSignal(VehicleRoadTable *table,
                                       RoadNet *roadnet,
                                       VehicleGroup *vehicle_group) {
    /*
        deal with each case of getting near to the intersection
    */
    action_type_ = unknown;
    if (status_ == finished) {
        action_type_ = act_finished;
        return;
    }
    if (status_ == unplanned) {
        throw std::invalid_argument("The car is not planned.");
    }
    while (!path_.empty() && path_[0] != pos_.road_) {
        path_.erase(path_.begin());
    }
    if (path_.empty()) {
        action_type_ = act_finished;
        CarArrived();
        return;
    }
    // need to consider signal light
    if (fabs(pos_.pos_ - pos_.road_->length_) <= 1e-3 &&    // near the inter and have signal
            pos_.road_->to_->type_ == TrafficNode::signal) {
        if (path_.size() <= 1) { // arrive
            table->RemoveVehicle(this, pos_.road_->unique_id_);
            action_type_ = act_finished;
            CarArrived();
            return;
        }
        std::function<bool(VehicleRoadTable *, DynamicRoad *)> CanTurnIn =
                [&](VehicleRoadTable *table, DynamicRoad *road) {
            auto cars = table->QueryCarsOnRoad(road->unique_id_);
            std::set<int> blocked_lane;
            for (VehiclePlanned *vehicle : *cars) {
                if (vehicle->pos_.pos_ < 1e-5) {
                    blocked_lane.insert(vehicle->pos_.lane_->lane_num_);
                }
            }
            if (blocked_lane.size() == road->num_of_lane_) {
                // fprintf(stderr, "sai bu jin\n");
                return false;
            }
            return true;
        }; // define a can turn in function: whether the target road is blocked.
        bool cango = ((TrafficNode *)(pos_.road_->to_))->CanGo(path_[0], path_[1]); // true
        if (cango && CanTurnIn(table, path_[1])) {  // can go
            bool is_turn = ((TrafficSignal *)(pos_.road_->to_))
                    ->DirectionRelation(path_[0], path_[1]) !=
                    TrafficSignal::straight; // whether the link is a turn
            path_.erase(path_.begin()); // change the path_

            if (is_turn) {  // just different in action_type
                table->MoveVehicle(this, pos_.road_->unique_id_, path_[0]->unique_id_);
                pos_.road_ = path_[0];
                pos_.pos_ = 0.0;
                pos_.lane_ = pos_.road_->GetLane(
                            std::min(pos_.lane_->lane_num_, pos_.road_->num_of_lane_ - 1));
                // update lane_id (may decrease the congesting performance)
                UpdateNextSignalDirection();
                //for (Lane *candidate_lane: pos_.road_->lanes_) 
                //    if (IsSameDirection(next_signal_direction_, candidate_lane))
                //       pos_.lane_ = candidate_lane;
                action_type_ = signal_turn;
            } else {
                table->MoveVehicle(this, pos_.road_->unique_id_, path_[0]->unique_id_);
                pos_.road_ = path_[0];
                pos_.pos_ = 0.0;
                pos_.lane_ = pos_.road_->GetLane(
                            std::min(pos_.lane_->lane_num_, pos_.road_->num_of_lane_ - 1));
                // update lane_id (may decrease the congesting performance)
                UpdateNextSignalDirection();
                //for (Lane *candidate_lane: pos_.road_->lanes_) 
                //    if (IsSameDirection(next_signal_direction_, candidate_lane))
                //        pos_.lane_ = candidate_lane;
                action_type_ = signal_straight;
            }
        } else { // have to stop and wait
            action_type_ = signal_stop;
            // todo
            velocity_ = 0;
            // WARNING : force the velocity to zero, need to check further
        }
    } else {    // no traffic signal, go straight on (2-way inter)
        action_type_ = go_straight;
    }
    // wait too long, replanning
    if (action_type_ == signal_turn || action_type_ == signal_straight ||
            (stop_time_ > 100 && stop_time_ % 100 == 0)) {
        // Replanning(roadnet);
        vehicle_group->replanning_vehicles_.push_back(this);
    }
}

void VehiclePlanned::GoTime(double sec, double start_sec,
                            VehicleRoadTable *table, RoadNet *roadnet) {
    if (action_type_ != signal_turn && action_type_ != signal_straight &&
            action_type_ != go_straight) {
        return;
    }
    DoActions(actions_);
    double all_time = sec;
    while (all_time > 0) {
        double remained_time = (pos_.road_->length_ - pos_.pos_);
        if (velocity_ > 0)
            remained_time = remained_time / velocity_;
        else
            remained_time = 1e10;
        if (all_time < remained_time) {
            pos_.pos_ += all_time * velocity_;
            break;
        } else {
            all_time -= remained_time;
            if (path_.size() <= 1) {
                CarArrived();
                break;
            }
            if (((TrafficNode *)(pos_.road_->to_))->CanGo(path_[0], path_[1]) &&
                    pos_.road_->to_->type_ != TrafficNode::signal) {
                if (path_[0]->GetRevRoad() == path_[1]) {
                    FixUTurn(roadnet);
                    if (path_[0]->GetRevRoad() == path_[1]) {
                        // fprintf(stderr, "cao ni ma ma\n");
                    }
                }
                //////////////////
                int pos = std::min(pos_.lane_->lane_num_, (int)pos_.road_->num_of_lane_ - 1);
                VehicleTurnStatus vehicle_status = GetVehicleTurnStatus(table);
                if(vehicle_status.next_signal_direction_ == VehicleStatus::left||vehicle_status.next_signal_direction_ == VehicleStatus::uturn){
                    pos = 0;
                }
                else if(vehicle_status.next_signal_direction_ == VehicleStatus::straight){
                    pos = 1;
                }
                else if(vehicle_status.next_signal_direction_ == VehicleStatus::right){
                    pos = 2;
                }
                path_.erase(path_.begin());
                pos_.road_ = path_[0];
                pos_.pos_ = 0.0;
                pos_.lane_ = pos_.road_->GetLane(pos);
                //////////////////////////////
            } else {
                pos_.pos_ = path_[0]->length_;
                break;
            }
        }
    }
    if (status_ == running && pos_.road_ != path_.at(0)) {
        throw std::invalid_argument(
                    "The car is not on the path in VehiclePlanned::GoTime()");
    }
}

std::string VehiclePlanned::PrintPath() {
    std::stringstream buf;
    if (status_ == unplanned) {
        throw std::invalid_argument("The car is not planned.");
    }
    if (status_ == finished) {
        return std::string("UNFINISHED");
    }
    for (DynamicRoad *road : path_) {
        buf << road->GetUniqueID() << " ";
    }
    return buf.str();
}

VehiclePlanned::~VehiclePlanned() {}

std::string VehiclePlanned::LogPos() {
    std::stringstream ss;
    ss << std::setprecision(12);
    ss << "[";
    double lat = 0.0, lon = 0.0;
    pos_.GetLatLon(lat, lon);
    ss << lat << "," << lon << "," << pos_.road_->GetUniqueID() % 2 << "]";
    return ss.str();
}

void VehiclePlanned::CarArrived() {
    status_ = finished;
    Common::GetCarArriveCounter()++;
    Common::GetCarArriveTime() += Common::GetTimer() - start_time_;
}

DrivingData *VehiclePlanned::PrepareDriving(VehicleStatus *vehicle_status){
    DrivingData* data=new DrivingData(vehicle_status);
    return data;
}

std::vector<VehicleAction> VehiclePlanned::ConductDriving(DrivingData *data){
    DrivingModule tmp(data);
    std::vector<VehicleAction> actions;
    tmp.ConductDriving(actions);
    return actions;
}

void VehiclePlanned::UpdateDriving(std::vector<VehicleAction> vehicle_actions){
    actions_=vehicle_actions;
}
