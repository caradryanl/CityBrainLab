#include "../head/filter.h"
#include <functional>
#include <iostream>
#include <queue>
#include <set>

Time Filter::min_time_ = {2050, 1, 1, 0, 0, 0};
Time Filter::max_time_ = {2000, 1, 1, 0, 0, 0};

Filter::Filter()
{

}

void Filter::Filtrate(std::string input_path, std::string output_path)
{
    if (input_path[input_path.length() - 1] != '/')
    {
        input_path.append("/");
    }
    if (output_path[output_path.length() - 1] != '/')
    {
        output_path.append("/");
    }
    if (access((output_path).c_str(), 0) == -1)
    {
        if (MakeDirection((output_path).c_str()) == -1)
        {
            std::cout << "fail to make output direction" << std::endl;
            return;
        }
    }

    FileInfo file(input_path);
    int iter = 0;
    if (file.IsOpen())
    {
        std::cout<<"File checked: "<<std::flush;
        std::cout<<iter<<std::flush;
        std::string name;
        while ((name = file.Next()).length() > 0)
        {
            std::ifstream vehicle_in(input_path + name);
            std::ofstream vehicle_out(output_path + name);
            if (!vehicle_in)
            {
                throw ("Failed to open file");
            }
            std::string str;

            getline(vehicle_in, str);
            std::vector<std::string> items = Split(str, ':');
            point_t point = CoordTrans(std::stod(items[0]), std::stod(items[1]));
            Time time = ParseTime(items[2]);
            while (point.empty() && getline(vehicle_in, str))
            {
                items = Split(str, ':');
                point = CoordTrans(std::stod(items[0]), std::stod(items[1]));
                time = ParseTime(items[2]);
            }
            std::string prev_str = str;
            Time prev_time = time;
            std::stack<std::string> buffer;
            while (getline(vehicle_in, str))
            {
                items = Split(str, ':');
                point = CoordTrans(std::stod(items[0]), std::stod(items[1]));
                time = ParseTime(items[2]);
                if (point.empty() || prev_time < MIN_TIME || MAX_TIME < prev_time)
                {
                    continue;
                }
                max_time_ = (prev_time < max_time_)? max_time_ : prev_time;
                min_time_ = (min_time_ < prev_time)? min_time_ : prev_time;
                if (time < prev_time)
                {
                    buffer.push(prev_str);
                }
                else
                {
                    vehicle_out << prev_str << std::endl;
                    while (!buffer.empty())
                    {
                        vehicle_out << buffer.top() << std::endl;
                        buffer.pop();
                    }
                }
                prev_time = time;
                prev_str = str;
            }
            vehicle_out << prev_str << std::endl;
            vehicle_in.close();
            vehicle_out.close();
            PrintStep(iter++);
        }
        file.Close();
        std::cout << std::endl;
        max_time_.minute_ = (max_time_.minute_ / SAMPLE_INTERVAL) * SAMPLE_INTERVAL;
        min_time_.minute_ = (min_time_.minute_ / SAMPLE_INTERVAL) * SAMPLE_INTERVAL;
    }
}
