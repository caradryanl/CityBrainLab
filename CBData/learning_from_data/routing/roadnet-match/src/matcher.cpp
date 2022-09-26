#include "../head/matcher.h"

Matcher::Matcher(std::string road_path)
{
    frame_to_road_vehicle_info_.clear();
    std::ifstream road_in(road_path);
    if (!road_in)
    {
        throw std::invalid_argument("File not found");
        return;
    }
    std::cout<<"Perparing data..."<<std::endl;
    int node_num;
    road_in >> node_num;
    double latitude, longitude;
    int64_t unique_id;
    bool is_signal;
    point_t point;
    std::vector<std::pair<std::vector<double>, size_t>> point_index;
    inter_id_to_unique_id_.clear();
    origin_track_.clear();
    point_index.clear();

    std::map<int, point_t> inter_id_to_coord;

    std::map<int64_t, int> unique_id_to_id;

    for (int i = 0; i < node_num; ++i)
    {
        road_in >> latitude;
        road_in >> longitude;
        road_in >> unique_id;
        road_in >> is_signal;
        if (unique_id_to_id.count(unique_id) > 0)
        {
            throw std::invalid_argument("Node id is not unique");
        }

        point.clear();
        point.push_back(latitude);
        point.push_back(longitude);
        unique_id_to_id.insert(std::pair<int64_t, int>(unique_id, i));
        inter_id_to_unique_id_.insert(std::pair<int, uint64_t>(i, unique_id));
        inter_id_to_coord.insert(std::pair<int, point_t>(i, point));
        point_index.push_back(std::pair<std::vector<double>, size_t>(point, i));
    }
    kdtree_ = new KDTree(point_index);

    int road_num;
    road_in >> road_num;
    int64_t unique_id1, unique_id2;
    double length;
    double speed_limit;
    int num_of_lane1, num_of_lane2;
    int dir1_id, dir2_id;
    std::vector<std::pair<std::pair<int, int>, double>> edges;
    edges.clear();
    int ignore_data;
    for (int i = 0; i < road_num; ++i)
    {
        road_in >> unique_id1 >> unique_id2 >> length >> speed_limit >> num_of_lane1 >> num_of_lane2 >> dir1_id >> dir2_id;
        for (int j = 0; j < 18; ++j)
        {
            road_in >> ignore_data;
        }
        if (unique_id_to_id.count(unique_id1) == 0)
        {
            throw std::invalid_argument("Can't find the node id1");
        }
        if (unique_id_to_id.count(unique_id2) == 0)
        {
            throw std::invalid_argument("Can't find the node id2");
        }
        int id1;
        int id2;
        try
        {
            id1 = unique_id_to_id.at(unique_id1);
            id2 = unique_id_to_id.at(unique_id2);
        } catch(const std::out_of_range &e) {
            std::cerr << "Exception at " << e.what() << std::endl;
        }
        edges.push_back(std::pair<std::pair<int, int>, double>(std::pair<int, int>(id1, id2), length));
        road_to_dir_id_.insert(std::pair<std::pair<int, int>, int>(std::pair<int, int>(id1, id2), dir1_id));
        road_to_dir_id_.insert(std::pair<std::pair<int, int>, int>(std::pair<int, int>(id2, id1), dir2_id));
        point_t point1 = inter_id_to_coord.at(id1);
        point_t point2 = inter_id_to_coord.at(id2);
        road_coord_to_dir_id_.push_back(std::pair<std::pair<point_t, point_t>, int>(std::pair<point_t, point_t>(point1, point2), dir1_id));
        road_coord_to_dir_id_.push_back(std::pair<std::pair<point_t, point_t>, int>(std::pair<point_t, point_t>(point2, point1), dir2_id));
    }

    graph_ = new CompactedGraph(edges, node_num);
    road_in.close();
    std::cout<<"Data perpared"<<std::endl;
}

void Matcher::Match(std::string vehicle_path, std::string output_path)
{
    ProcessFilePath(vehicle_path, output_path);

    std::ofstream road_temp_out(output_path + "road/road-temp.txt");
    for (std::map<std::pair<int, int>, int>::iterator road_it = road_to_dir_id_.begin();
         road_it != road_to_dir_id_.end();
         ++road_it)
    {
        road_temp_out << road_it->first.first << '\t' << road_it->first.second << '\t' << road_it->second << std::endl;
    }
    road_temp_out.close();

    std::ofstream road_coord_temp_out(output_path + "road/road-coord-temp.txt");
    for (std::vector<std::pair<std::pair<point_t, point_t>, int>>::iterator road_it = road_coord_to_dir_id_.begin();
         road_it != road_coord_to_dir_id_.end();
         ++road_it)
    {
        road_coord_temp_out << road_it->first.first[0] << '\t' << road_it->first.first[1] << '\t';
        road_coord_temp_out << road_it->first.second[0] << '\t' << road_it->first.second[1] << '\t' << road_it->second << std::endl;
    }
    road_coord_temp_out.close();

    std::cout<<"Matching..."<<std::endl;
    FileInfo file(vehicle_path);
    int iter = 0;
    if (file.IsOpen())
    {
        std::cout<<"File matched: "<<std::flush;
        std::cout<<iter<<std::flush;
        std::string name;
        // match a vehicle file
        while ((name = file.Next()).length() != 0)
        {
            std::ifstream vehicle_in(vehicle_path + name);
            std::ofstream vehicle_info_out(output_path + "vehicle/" + "matched-" + name);
            if (!vehicle_in)
            {
                throw ("Failed to open file");
            }
            vehicle_info_out << "curr time\t" << "start\t" << "end\t" << "position\n";

            std::string str;
            std::vector<std::string> items;
            int index = 0;

            point_t origin_point;
            Time time;

            if (getline(vehicle_in, str) && str != "")
            {
                items = Split(str, ':');
                origin_point = CoordTrans(std::stod(items[0]), std::stod(items[1]));
                time = ParseTime(items[2]);
                time.minute_ = (time.minute_ / SAMPLE_INTERVAL) * SAMPLE_INTERVAL;
                index = kdtree_->nearest_index(origin_point);
            }

            int prev_index = index;
            Time prev_time = time;
            std::vector<std::pair<std::vector<int>, Time>> vehicle_whole_path;

            // match a line
            while (getline(vehicle_in, str) && str != "")
            {
                prev_index = index;
                items = Split(str, ':');
                origin_point = CoordTrans(std::stod(items[0]), std::stod(items[1]));
                index = kdtree_->nearest_index(origin_point);
                time = ParseTime(items[2]);
                time.minute_ = (time.minute_ / SAMPLE_INTERVAL) * SAMPLE_INTERVAL;
                if (prev_index != index)
                {
                    std::vector<int> path = graph_->FindShortestPath(prev_index, index);
                    if (path.size() != 0)
                    {
                        // catenate path
                        if (!vehicle_whole_path.empty() && vehicle_whole_path.back().first.back() == path.front())
                        {
                            vehicle_whole_path.back().first.pop_back();
                            vehicle_whole_path.back().first.insert(vehicle_whole_path.back().first.end(), path.begin(), path.end());
                        }
                        else
                        {
                            vehicle_whole_path.push_back(std::pair<std::vector<int>, Time>{path, prev_time});
                        }
                        // output shortest path
                        double length = graph_->QueryPathLength(path);
                        uint64_t prev_matched_unique_id = inter_id_to_unique_id_.at(*(path.begin()));
                        int size = 1;
                        int interval = (SAMPLE_INTERVAL > time - prev_time)? SAMPLE_INTERVAL : (time - prev_time);
                        double vehicle_avg_speed_by_path = length / interval;
                        double vehicle_avg_speed_by_record = std::stod(items[3]);
                        double path_length = 0;
                        double prev_path_length = 0;
                        uint64_t matched_unique_id;

                        for (std::vector<int>::iterator path_it = path.begin(); path_it + 1 < path.end(); ++path_it)
                        {
                            prev_path_length = path_length;
                            matched_unique_id = inter_id_to_unique_id_.at(*(path_it + 1));
                            path_length = graph_->QueryPathLength(path, ++size);
                            road_t prev_road = {0, 0};
                            road_t road = {0, 0};
                            for (double pos = 0.0; pos < path_length - prev_path_length; pos += (double)SAMPLE_INTERVAL * vehicle_avg_speed_by_path)
                            {
                                road = {*(path_it), *(path_it + 1)};
                                UpdateRoadVehicleInfo(prev_road, road, prev_time, vehicle_avg_speed_by_record);
                                vehicle_info_out << prev_time << '\t';
                                vehicle_info_out << prev_matched_unique_id << '\t';
                                vehicle_info_out << matched_unique_id << '\t';
                                vehicle_info_out << pos << '\n';
                                prev_time = prev_time + SAMPLE_INTERVAL;
                                prev_road = road;
                            }
                            prev_matched_unique_id = matched_unique_id;
                        }
                    }
                    else
                    {
                        // shortest path not found
                        uint64_t prev_matched_unique_id = inter_id_to_unique_id_.at(prev_index);
                        uint64_t matched_unique_id = inter_id_to_unique_id_.at(index);
                        int interval = (SAMPLE_INTERVAL > time - prev_time)? SAMPLE_INTERVAL : (time - prev_time);
                        for (int i = 0; i < interval; i += SAMPLE_INTERVAL)
                        {
                            vehicle_info_out << prev_time << '\t';
                            vehicle_info_out << prev_matched_unique_id << '\t';
                            vehicle_info_out << matched_unique_id << '\t';
                            vehicle_info_out << -1 << '\n';
                            prev_time = prev_time + SAMPLE_INTERVAL;
                        }
                    }
                    prev_time = time;
                }
            }
            vehicle_in.close();
            vehicle_info_out.close();

            // output vehicle flow in cbengine style
            OutputCBEngineVehicleFlow(vehicle_whole_path);
            PrintStep(iter++);
        }
        file.Close();

        std::cout << "\nmatching finished, writing road info" << std::endl;
        std::ofstream vehicle_num_out(output_path + "road/" + "roadinfo-vehicle-num.txt");
        std::ofstream avg_speed_out(output_path + "road/" + "roadinfo-avg-speed.txt");
        std::ofstream vehicle_in_out(output_path + "road/" + "roadinfo-vehicle-in.txt");

        int64_t period = Filter::max_time_ - Filter::min_time_;

        vehicle_num_out << "start\t" << "end";
        avg_speed_out << "start\t" << "end";
        vehicle_in_out << "start\t" << "end";
        for (int64_t i = 0; i < period; i += SAMPLE_INTERVAL * OUTPUT_SAMPLE_RATIO)
        {
            vehicle_num_out << '\t' << Filter::min_time_ + i;
            avg_speed_out << '\t' << Filter::min_time_ + i;
            vehicle_in_out << '\t' << Filter::min_time_ + i;
        }
        vehicle_num_out << std::endl;
        avg_speed_out << std::endl;
        vehicle_in_out << std::endl;

        for (std::map<road_t, std::map<int, RoadVehicleInfo>>::const_iterator road_it = frame_to_road_vehicle_info_.begin();
             road_it != frame_to_road_vehicle_info_.end();
             ++road_it)
        {
            uint64_t from = inter_id_to_unique_id_.at(road_it->first.first);
            uint64_t to = inter_id_to_unique_id_.at(road_it->first.second);
            vehicle_num_out << from << '\t' << to << '\t';
            avg_speed_out << from << '\t' << to << '\t';
            vehicle_in_out << from << '\t' << to << '\t';
            for (int64_t i = 0; i < period; i += SAMPLE_INTERVAL * OUTPUT_SAMPLE_RATIO)
            {
                std::map<int, RoadVehicleInfo>::const_iterator time_it = road_it->second.find(i / SAMPLE_INTERVAL);
                if (time_it != road_it->second.end())
                {
                    vehicle_num_out << time_it->second.vehicle_num_ << '\t';
                    avg_speed_out << time_it->second.avg_speed_ << '\t';
                    vehicle_in_out << time_it->second.vehicle_in_ << '\t';
                }
                else
                {
                    vehicle_num_out << -1 << '\t';
                    avg_speed_out << -1.0 << '\t';
                    vehicle_in_out << -1 << '\t';
                }
            }
            vehicle_num_out << std::endl;
            avg_speed_out << std::endl;
            vehicle_in_out << std::endl;
        }
        vehicle_num_out.close();
        avg_speed_out.close();
        vehicle_in_out.close();
    }
    else
    {
        throw("File path not found");
    }

    std::cout<<"Completed"<<std::endl;
}

void Matcher::PrintPath(const std::vector<int> &path) const
{
    // debug use
    for (std::vector<int>::const_iterator it = path.begin(); it != path.end(); ++it)
    {
        std::cout<< *it << " ";
    }
}

void Matcher::PrintRoadLength(int prev_index, int index) const
{
    // debug use
    try
    {
        double l = graph_->QueryRoadLength(prev_index, index);
        std::cout << "length: " << l << std::endl;
    }
    catch (std::invalid_argument &ia)
    {
        std::cout << ia.what() << std::endl;
    }
}

void Matcher::ProcessFilePath(std::string &input_path, std::string &output_path)
{
    if (input_path[input_path.length() - 1] != '/')
    {
        input_path.append("/");
    }
    if (output_path[output_path.length() - 1] != '/')
    {
        output_path.append("/");
    }
    output_path_ = output_path;
    if (access((output_path + "vehicle/").c_str(), 0) == -1)
    {
        if (MakeDirection((output_path + "vehicle/").c_str()) == -1)
        {
            std::cout << "fail to make output direction" << std::endl;
            return;
        }
    }
    if (access((output_path + "road/").c_str(), 0) == -1)
    {
        if (MakeDirection((output_path + "road/").c_str()) == -1)
        {
            std::cout << "fail to make output direction" << std::endl;
            return;
        }
    }
    if (access((output_path + "cbengine/").c_str(), 0) == -1)
    {
        if (MakeDirection((output_path + "cbengine/").c_str()) == -1)
        {
            std::cout << "fail to make output direction" << std::endl;
            return;
        }
    }
    std::ofstream cbengine_vehicle_out(output_path_ + "cbengine/cbengine-flow.txt");
    cbengine_vehicle_out << std::endl;
    cbengine_vehicle_out.close();
}

inline void Matcher::UpdateRoadVehicleInfo(road_t prev_road, road_t road, const Time &time, double avg_speed)
{
    std::map<road_t, std::map<int, RoadVehicleInfo>>::iterator road_it;
    std::map<int, RoadVehicleInfo>::iterator time_it;
    RoadVehicleInfo info;
    bool vehicle_in_flag = true;
    if (prev_road.first == road.first && prev_road.second == road.second)
    {
        vehicle_in_flag = false;
    }
    if (road.first > road.second)
    {
        int tmp = road.first;
        road.first = road.second;
        road.second = tmp;
    }
    road_it = frame_to_road_vehicle_info_.find(road);
    if (road_it != frame_to_road_vehicle_info_.end())
    {
        time_it = road_it->second.find(Filter::TimeToIndex(time));
        if (time_it != road_it->second.end())
        {
            info = time_it->second;
            time_it->second.avg_speed_ = (info.avg_speed_ * info.vehicle_num_ + avg_speed) / (info.vehicle_num_ + 1);
            time_it->second.vehicle_num_ = info.vehicle_num_ + 1;
            if (vehicle_in_flag)
            {
                time_it->second.vehicle_in_ = info.vehicle_in_ + 1;
            }
        }
        else
        {
            info.vehicle_num_ = 1;
            info.avg_speed_ = avg_speed;
            if (vehicle_in_flag)
            {
                info.vehicle_in_ = 1;
            }
            road_it->second.insert(std::pair<int, RoadVehicleInfo>(Filter::TimeToIndex(time), info));
        }
    }
    else
    {
        std::map<int, RoadVehicleInfo> time_to_road;
        info.vehicle_num_ = 1;
        info.avg_speed_ = avg_speed;
        info.vehicle_in_ = 1;
        time_to_road.insert(std::pair<int, RoadVehicleInfo>(Filter::TimeToIndex(time), info));
        frame_to_road_vehicle_info_.insert(std::pair<road_t, std::map<int, RoadVehicleInfo>>(road, time_to_road));
    }
}

void Matcher::OutputCBEngineVehicleFlow(std::vector<std::pair<std::vector<int>, Time>> &vehicle_whole_path)
{
    if (vehicle_whole_path.size() < 2)
    {
        return;
    }

    std::ofstream cbengine_vehicle_out(output_path_ + "cbengine/cbengine-flow.txt", std::ios::app);
    for (std::vector<std::pair<std::vector<int>, Time>>::const_iterator path_it = vehicle_whole_path.begin();
         path_it != vehicle_whole_path.end();
         ++path_it)
    {
        int prev_inter = path_it->first.front();
        Time start_time = path_it->second;
        cbengine_vehicle_out << start_time - Filter::min_time_ << " " << start_time - Filter::min_time_ + 1 << " " << 3 << std::endl;
        cbengine_vehicle_out << path_it->first.size() - 1 << std::endl;
        for (std::vector<int>::const_iterator inter_it = path_it->first.begin() + 1; inter_it != path_it->first.end(); ++inter_it)
        {
            int inter = *inter_it;
            try
            {
                int dir_id = road_to_dir_id_.at(std::pair<int, int>(prev_inter, inter));
                cbengine_vehicle_out << dir_id << " ";
            }
            catch(const std::out_of_range &e)
            {
                std::cerr << "Exception at " << e.what() << std::endl;
            }
            prev_inter = inter;
        }
        cbengine_vehicle_out << std::endl;
    }
    cbengine_vehicle_out.close();
}
