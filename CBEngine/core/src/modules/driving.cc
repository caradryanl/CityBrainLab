#include "../head/vehicle.h"

void DrivingModule::ConductDriving(std::vector<VehicleAction> &actions)
{
    double t_0 = clock();
    const VehicleStatus *status = data_->vehicle_status_;
    if (status->this_car_->action_type_ == VehiclePlanned::act_finished)
        return;
    // if next_car == NULL , there is no car between itself and signal,
    // dis_to_next_car = 0.0 Get the velocity of next_car by
    // next_car->GetVelocity();
    VehiclePlanned *this_car = status->this_car_;
    Lane *lane = status->this_car_->pos_.lane_;
    Road *road = status->this_car_->pos_.road_;
    double max_acc=config_.params_[0];
    double min_acc=config_.params_[1];
    double velocity=status->this_car_->velocity_;
    int road_lane_num = road->num_of_lane_;
    double max_velocity = status->this_car_->pos_.road_->speed_limit_;
    VehicleAction vehicle_action;
    //double t_1 = clock();
    double dis_to_signal = status->dis_to_signal_;
    double signal_remained_time = status->signal_remained_time_;
    double dis_to_next_car = status->dis_to_next_car_;
    
    const VehiclePlanned *next_car = status->next_car_;
    const VehiclePlanned *left_next_car = status->left_next_car_;
    const VehiclePlanned *left_last_car = status->left_last_car_;
    const VehiclePlanned *right_next_car = status->right_next_car_;
    const VehiclePlanned *right_last_car = status->right_last_car_;
    //double t_2 = clock();
    double dis_to_left_next_car = status->dis_to_left_next_car_;
    double dis_to_left_last_car = status->dis_to_left_last_car_;
    double dis_to_right_next_car = status->dis_to_right_next_car_;
    double dis_to_right_last_car = status->dis_to_right_last_car_;
    bool signal_can_go = status->signal_can_go_;
    VehicleStatus::Direction next_signal_direction=(VehicleStatus::Direction)status->next_signal_direction_;
    //double t_3 = clock();

    double change_min_acc = -min_acc, change_max_acc = max_acc;
    int change_lane_delta = 0;
    bool first_car = true;
    if (next_car != NULL)
        first_car = false;

    //double t_4 = clock();
    
    bool correct_lane = false;
    if (IsSameDirection(next_signal_direction, lane))
        correct_lane = true;
    if (next_signal_direction == VehicleStatus::destination)
        correct_lane = true;

    int target_direction = 0;
    int num_of_front_car = 1 << 30, pos = 0;

    // find the correct and least vehicles lane, name it 'pos'
    // if a lane contains multiple accessible direction, then it may cause lane change
    int len = road->lanes_.size();
    for (int i = 0; i < len; i++) {
        Lane *target_lane = road->lanes_[i];
        if (IsSameDirection(next_signal_direction, target_lane))
            if (status->num_of_car_to_light_[i] < num_of_front_car) {
                num_of_front_car = status->num_of_car_to_light_[i];
                pos = i;
            }
    }

    bool dangerous = false, jam = false;
    // current lane != correct and least vehicles lane ,and car front distance is high
    if (pos != lane->lane_num_ && 10 * num_of_front_car >= dis_to_signal) {
        dangerous = true;
        if (pos < lane->lane_num_)
            target_direction = -1;
        else
            target_direction = 1;
    }

    // here different signal turn has different cut in distance.
    int dist_threshold = GetCutinDistance(next_signal_direction);
    // if not correct lane and not destination and need to cut in
    if (next_signal_direction != VehicleStatus::destination &&
            !correct_lane && dis_to_signal <= dist_threshold) {
        dangerous = true;
    }
    //
    if (!target_direction && dis_to_signal < 1000) {
        for (auto target_lane : road->lanes_)
            if (IsSameDirection(next_signal_direction, target_lane)) {
                if (target_lane->lane_num_ < lane->lane_num_)
                    target_direction = -1;
                else
                    target_direction = 1;
                break;
            }
    }

    if (dangerous) {
        if (10 * status->num_of_car_to_light_.at(lane->lane_num_ +
                                                        target_direction) >=
                dis_to_signal)
            jam = true;
    }

    if (next_signal_direction != VehicleStatus::destination &&
            !correct_lane && target_direction) {
        const VehiclePlanned *next_lane_car = left_next_car,
                *last_lane_car = left_last_car;
        double dis_to_next_lane_car = dis_to_left_next_car,
                dis_to_last_lane_car = dis_to_left_last_car;
        if (target_direction == 1) {
            next_lane_car = right_next_car;
            last_lane_car = right_last_car;
            dis_to_next_lane_car = dis_to_right_next_car;
            dis_to_last_lane_car = dis_to_right_last_car;
        }

        if (next_lane_car == NULL)
            change_max_acc = std::min(max_velocity - velocity, max_acc);
        else
            change_max_acc = this_car->GetMaxVelocity(velocity, next_lane_car->GetVelocity(),
                                            dis_to_next_lane_car);

        if (last_lane_car == NULL)
            change_min_acc = -std::min(velocity, min_acc);
        else
            change_min_acc = this_car->GetMinVelocity(velocity, last_lane_car->GetVelocity(),
                                            dis_to_last_lane_car, 0);

        if (change_min_acc > change_max_acc) {
            if (dangerous) {
                if (!jam && velocity < 1e-5 &&
                        (next_signal_direction == VehicleStatus::left ||
                         next_signal_direction == VehicleStatus::uturn))
                    change_min_acc = 0, change_max_acc = 2.0;
                else
                    change_min_acc = change_max_acc = -min_acc;
            } else {
                change_min_acc = -std::min(velocity, min_acc);
                change_max_acc = std::min(max_velocity - velocity, max_acc);
            }
        } else
            change_lane_delta = target_direction;
    }

    // try one try, ac is ok
    /*
    int cut_in = 0;
    for (auto target_lane : road->lanes_)
        if (IsSameDirection(next_signal_direction, target_lane)) {
            if (target_lane->lane_num_ == lane->lane_num_ - 1)
                cut_in |= 1;
            else if (target_lane->lane_num_ == lane->lane_num_ + 1)
                cut_in |= 2;
        }*/

    if ((Common::GetTimer() & 1) && change_lane_delta == 1) {
        change_max_acc = max_acc;
        change_min_acc = -std::min(velocity, min_acc);
        change_lane_delta = 0;
    }

    if (!(Common::GetTimer() & 1) && change_lane_delta == -1) {
        change_max_acc = max_acc;
        change_min_acc = -std::min(velocity, min_acc);
        change_lane_delta = 0;
    }

    if (change_lane_delta == -1) {
        next_car = left_next_car;
        dis_to_next_car = dis_to_left_next_car;
    }

    if (change_lane_delta == 1) {
        next_car = right_next_car;
        dis_to_next_car = dis_to_right_next_car;
    }
    //double t_2 = clock();
    

    // the distance is enough
    bool rush = false;
    if ((first_car && (dis_to_signal >= 200 || signal_remained_time >= 1e7)) ||
            dis_to_next_car >= 200) {
        rush = true;
        change_max_acc =
                std::min(change_max_acc, std::min(max_velocity - velocity, max_acc));
    }
    
    double initial_velocity = velocity;
    
    // evaluate traffic light
    if (signal_remained_time < 0)
        signal_remained_time = 1000;
    if (!(dis_to_signal >= 200 || signal_remained_time >= 1e7)) {
        if (signal_can_go == false) {
            
            double Left = std::max(0.0, velocity - min_acc);
            double Right = std::min(velocity + max_acc, max_velocity);
            double Mid = (Left + Right) / 2.;
            for (int i = 1; i <= 100; i++, Mid = (Left + Right) / 2.) {
                double tmp = Mid;
                double down_time = int(tmp / min_acc + 1e-9);
                double down_dist =
                        (2 * tmp - min_acc * down_time) * (down_time + 1) / 2;
                if ((down_time <= signal_remained_time && down_dist <= dis_to_signal) ||
                        tmp * signal_remained_time <= dis_to_signal)
                    Left = Mid;
                else
                    Right = Mid;
            }
            change_max_acc = std::min(change_max_acc, Left - velocity);
            
        } else {
            // accelerate
            int acc_sec = (max_velocity - velocity) / max_acc;
            double max_dist = 0;
            if (acc_sec >= signal_remained_time)
                max_dist = (2 * velocity + max_acc * (signal_remained_time + 1)) *
                        signal_remained_time / 2;
            else
                max_dist = (2 * velocity + max_acc * (acc_sec + 1)) * acc_sec / 2 +
                        (signal_remained_time - acc_sec) * max_velocity;
            if (max_dist >= dis_to_signal) {
                double cur_velocity = std::min(velocity + max_acc, max_velocity);
                change_max_acc = std::min(change_max_acc, cur_velocity - velocity);
            } else {
                double Left = std::max(0.0, velocity - min_acc);
                double Right = std::min(velocity + max_acc, max_velocity);
                double Mid = (Left + Right) / 2.;
                for (int i = 1; i <= 100; i++, Mid = (Left + Right) / 2.) {
                    double tmp = Mid;
                    double down_time = tmp / min_acc;
                    double down_dist =
                            (2 * tmp - min_acc * down_time) * (down_time + 1) / 2;
                    if (down_dist <= dis_to_signal)
                        Left = Mid;
                    else
                        Right = Mid;
                }
                change_max_acc = std::min(change_max_acc, Left - velocity);
            }
        }
    }

    if (!first_car) {
        //double threshold_safe_distance = status->this_car_->safety_distance_;
        //double safe_distance =
        //        std::max(threshold_safe_distance, 2 * initial_velocity);
        double safe_distance = 5;
        if (dis_to_next_car <= safe_distance) {
            change_max_acc = -std::min(velocity, min_acc);
        } else {
            change_max_acc = std::min(
                        change_max_acc,
                        this_car->GetMaxVelocity(velocity, next_car->GetVelocity(), dis_to_next_car));
        }
    }
    
    if (change_lane_delta && change_min_acc > change_max_acc) {
        change_lane_delta = 0;
        change_max_acc = -std::min(velocity, min_acc);
    }

    if (change_max_acc < -9999) {
        change_max_acc = -std::min(velocity, min_acc);
    }

    // wait for cutting in
    if (!change_lane_delta && velocity < min_acc) {
        bool stop = false;
        if (left_next_car != NULL) {
            double dist = dis_to_signal - dis_to_left_next_car;
            if (dis_to_left_next_car >= 5 && dis_to_left_next_car <= 10 &&
                    dist >= 0 && !first_car &&
                    (dist <= this_car->GetCutinDistance(left_next_car->next_signal_direction_) ||
                     left_next_car->velocity_ < 1e-5) &&
                    !this_car->IsSameDirection(left_next_car->next_signal_direction_,
                                     road->lanes_[lane->lane_num_ - 1]) &&
                    (left_next_car->next_signal_direction_ ==
                     VehicleStatus::right ||
                     left_next_car->next_signal_direction_ ==
                     VehicleStatus::straight))
                stop = true;
        }

        if (right_next_car != NULL) {
            double dist = dis_to_signal - dis_to_right_next_car;
            if (dis_to_right_next_car >= 5 && dis_to_right_next_car <= 10 &&
                    dist >= 0 && !first_car &&
                    (dist <= this_car->GetCutinDistance(right_next_car->next_signal_direction_) ||
                     right_next_car->velocity_ < 1e-5) &&
                    !this_car->IsSameDirection(right_next_car->next_signal_direction_,
                                     road->lanes_[lane->lane_num_ + 1]) &&
                    (right_next_car->next_signal_direction_ ==
                     VehicleStatus::left ||
                     right_next_car->next_signal_direction_ ==
                     VehicleStatus::uturn))
                stop = true;
        }
        if (stop)
            change_max_acc = -std::min(velocity, min_acc);
    }

    if (change_lane_delta) {
        vehicle_action.action_type_ = VehicleAction::lane_change;
        vehicle_action.velocity_delta_ = 0;
        vehicle_action.lane_delta_ = change_lane_delta;
        actions.push_back(vehicle_action);
    }
    //double t_3 = clock();

    vehicle_action.action_type_ = VehicleAction::accelerate;
    vehicle_action.velocity_delta_ =
            std::min(change_max_acc, max_velocity - velocity);
    vehicle_action.lane_delta_ = 0;

    if (change_max_acc < 0) {
        vehicle_action.action_type_ = VehicleAction::decelerate;
        vehicle_action.velocity_delta_ = std::min(-change_max_acc, velocity);
    }
    
    if(Common::RandomInt(0,99)<=2){

        if(vehicle_action.action_type_ == VehicleAction::accelerate){

            vehicle_action.action_type_ = VehicleAction::decelerate;
            vehicle_action.velocity_delta_ = std::min(min_acc, velocity);
        }
    }
    actions.push_back(vehicle_action);

    //double t_4 = clock();
    //printf("%.3f, %.3f, %.3f, %.3f, %.3f\n", t_1-t_0, t_2-t_1, t_3-t_2, t_4-t_3, t_4-t_0);
}

bool DrivingModule::IsSameDirection(VehicleStatus::Direction next_signal_direction, Lane* lane) const{
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

double DrivingModule::GetCutinDistance(VehicleStatus::Direction next_signal_direction) const{
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

void WorkThread::ComputeAct(VehiclePlanned *vehicle, VehicleRoadTable *table){
    
    // Prepare status
    vehicle->actions_.clear();
    double t1 = 1.0 * clock();
    if (vehicle->action_type_ == vehicle->unknown) {
        throw std::invalid_argument(
                    "Action type is unknown in VehiclePlanned::GetNextSecAction()");
    } else if (vehicle->action_type_ == vehicle->act_finished) {
        return;
    } else if (vehicle->action_type_ == vehicle->signal_turn) {   // set in PassTrafficSignal()
        VehicleTurnStatus vehicle_status = vehicle->GetVehicleTurnStatus(table); // nearby vehicle info
        vehicle->actions_ = vehicle->GetTurnAction(vehicle_status);   // type: lane_change
        vehicle->DoActions(vehicle->actions_);
    } else if (vehicle->action_type_ == vehicle->signal_straight) {
        VehicleTurnStatus vehicle_status = vehicle->GetVehicleTurnStatus(table);
        vehicle->actions_ = vehicle->GetTurnAction(vehicle_status);
        vehicle->DoActions(vehicle->actions_);
    }
    double t2 = 1.0 * clock();
    vehicle->GetVehicleStatus(table, status_);
    double t3 = 1.0 * clock();
    //printf("Sec: %.3f %.3f\n", t2 - t1, t3 - t2);

    //Prepare Carfollowing data
    data_.Reset(&status_);
    module_.SetData(&data_);
    double t4 = 1.0 * clock();

    //Compute Act
    vehicle->actions_.clear();
    module_.ConductDriving(vehicle->actions_);
    double t5 = 1.0 * clock();
    //printf("Act: %.3f, %.3f, %.3f, %.3f\n", t2-t1, t3-t2, t4-t3, t5-t4);

}

// Static member
DrivingModule::DrivingParams DrivingModule::config_ = DrivingParams();

// Optimization function
std::vector<double> DrivingModule::GetModuleParams() const{
    return config_.params_;
}
void  DrivingModule::SetModuleParams(std::vector<double> params){
    config_.params_.clear();
    for (auto p : params)
		config_.params_.push_back(p);
}

