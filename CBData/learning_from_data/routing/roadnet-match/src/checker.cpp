#include "checker.h"

Checker::Checker(int threshold) : threshold_(threshold) {}

void Checker::Check(std::string input_path, std::string output_path)
{
    std::cout << "Checking roads with vehicle flow > " << threshold_ * OUTPUT_SAMPLE_RATIO << std::endl;
    std::ifstream vehicle_num_in(input_path + "roadinfo-vehicle-num.txt");
    std::ifstream avg_speed_in(input_path + "roadinfo-avg-speed.txt");
    std::ifstream vehicle_in_in(input_path + "roadinfo-vehicle-in.txt");

    std::ofstream vehicle_num_out(output_path + "checked-roadinfo-vehicle-num.txt");
    std::ofstream avg_speed_out(output_path + "checked-roadinfo-avg-speed.txt");
    std::ofstream vehicle_in_out(output_path + "checked-roadinfo-vehicle-in.txt");
    std::ofstream vehicle_out_out(output_path + "checked-roadinfo-vehicle-out.txt");

    if (!vehicle_num_in || !avg_speed_in || !vehicle_in_in)
    {
        std::cout << "Check file not found" << std::endl;
    }
    std::string vehicle_num_str;
    std::string avg_speed_str;
    std::string vehicle_in_str;
    getline(vehicle_num_in, vehicle_num_str);
    vehicle_num_out << vehicle_num_str << std::endl;
    getline(avg_speed_in, avg_speed_str);
    avg_speed_out << avg_speed_str << std::endl;
    getline(vehicle_in_in, vehicle_in_str);
    vehicle_in_out << vehicle_in_str << std::endl;
    vehicle_out_out << vehicle_in_str << std::endl;
    while (getline(vehicle_num_in, vehicle_num_str) && vehicle_num_str != ""
           && getline(avg_speed_in, avg_speed_str) && avg_speed_str != ""
           && getline(vehicle_in_in, vehicle_in_str) && vehicle_in_str != "")
    {
        std::vector<std::string> vehicle_num_items = Split(vehicle_num_str, '\t');
        int sum = 0;
        for (std::vector<std::string>::iterator item_it = vehicle_num_items.begin() + 4; item_it < vehicle_num_items.end(); ++item_it)
        {
            int num = std::atoi(item_it->c_str());
            if (num > 0)
            {
                sum += num;
            }
        }
        if (sum > threshold_)
        {
            std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> path;
            for (int i = 0; i < 5; ++i)
            {
                std::vector<std::string> vehicle_in_items = Split(vehicle_in_str, '\t');
                for (int j = 0; j < 4; ++j)
                {
                    vehicle_out_out << std::stod(vehicle_in_items[j].c_str()) << '\t';
                }
                std::vector<std::string>::iterator vehicle_num_it = vehicle_num_items.begin() + 5;
                for (std::vector<std::string>::iterator vehicle_in_it = vehicle_in_items.begin() + 4;
                     vehicle_in_it < vehicle_in_items.end() && vehicle_num_it < vehicle_num_items.end();
                     ++vehicle_in_it, ++vehicle_num_it)
                {
                    int vehicle_in_num = std::atoi(vehicle_in_it->c_str());
                    int vehicle_num = std::atoi(vehicle_num_it->c_str());
                    int prev_vehicle_num = std::atoi((vehicle_num_it - 1)->c_str());
                    if (vehicle_in_num > -1 && vehicle_num > -1)
                    {
                        int vehicle_out = ((vehicle_in_num - (vehicle_num - prev_vehicle_num)) > -1)?
                                    vehicle_in_num - (vehicle_num - prev_vehicle_num) : 0;
                        vehicle_out_out << vehicle_out << '\t';
                    }
                    else
                    {
                        vehicle_out_out << -1 << '\t';
                    }
                }
                vehicle_num_out << vehicle_num_str << std::endl;
                avg_speed_out << avg_speed_str << std::endl;
                vehicle_in_out << vehicle_in_str << std::endl;
                vehicle_out_out << std::endl;

                vehicle_num_in.seekg(0, std::ios::beg);
                avg_speed_in.seekg(0, std::ios::beg);
                vehicle_in_in.seekg(0, std::ios::beg);
                double prev_to_lat = std::stod(vehicle_in_items[2].c_str());
                double prev_to_lng = std::stod(vehicle_in_items[3].c_str());

                bool next_road_exist = false;
                getline(vehicle_num_in, vehicle_num_str);
                getline(avg_speed_in, avg_speed_str);
                getline(vehicle_in_in, vehicle_in_str);
                while (getline(vehicle_num_in, vehicle_num_str) && vehicle_num_str != ""
                       && getline(avg_speed_in, avg_speed_str) && avg_speed_str != ""
                       && getline(vehicle_in_in, vehicle_in_str) && vehicle_in_str != "")
                {
                    std::vector<std::string> items = Split(vehicle_num_str, '\t');
                    if (std::stod(items[0]) == prev_to_lat && std::stod(items[1]) == prev_to_lng)
                    {
                        next_road_exist = true;
                        break;
                    }
                }
                if (!next_road_exist)
                {
                    break;
                }
            }
            break;
        }
    }
    vehicle_num_in.close();
    avg_speed_in.close();
    vehicle_in_in.close();

    vehicle_num_out.close();
    avg_speed_out.close();
    vehicle_in_out.close();
    vehicle_out_out.close();
    std::cout << "Checking completed" << std::endl;
}
