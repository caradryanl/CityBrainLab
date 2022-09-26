#include "../head/utils.h"

std::vector<std::string> Split(const std::string &str, char delim)
{
    std::istringstream iss(str);
    std::string item;
    std::vector<std::string> items;
    items.clear();
    while (getline(iss, item, delim))
    {
        items.push_back(item);
    }
    return items;
}

void PrintStep(int iter)
{
    std::string iter_str = std::to_string(iter);
    int iter_len = iter_str.length();
    for (int  i = 0; i < iter_len; ++i)
    {
        std::cout<<"\b \b"<<std::flush;
    }
    ++iter;
    std::cout<<iter<<std::flush;
}

point_t CoordTrans(double longitude, double latitude)
{
    longitude = longitude / 600000;
    latitude = latitude / 600000;
    point_t point;
    point.clear();
    if (!(longitude > 73.66 && longitude < 135.05 && latitude > 3.86 && latitude < 53.55))
    {
        return point_t();
    }
    double dlat = TransformLat(longitude - 105.0, latitude - 35.0);
    double dlng = TransformLng(longitude - 105.0, latitude - 35.0);
    double radlat = latitude / 180.0 * M_PI;
    double magic = sin(radlat);
    magic = 1 - EE * magic * magic;
    double sqrtmagic = sqrt(magic);
    dlat = (dlat * 180.0) / ((A * (1 - EE)) / (magic * sqrtmagic) * M_PI);
    dlng = (dlng * 180.0) / (A / sqrtmagic * cos(radlat) * M_PI);
    double mglat = latitude + dlat;
    double mglng = longitude + dlng;
    point.push_back(latitude * 2 - mglat);
    point.push_back(longitude * 2 - mglng);
    return point;
}

double TransformLat(double longitude, double latitude)
{
    double ret = -100.0 + 2.0 * longitude + 3.0 * latitude + 0.2 * latitude * latitude + 0.1 * longitude * latitude + 0.2 * sqrt(fabs(longitude));
    ret += (20.0 * sin(6.0 * longitude * M_PI) + 20.0 * sin(2.0 * longitude * M_PI)) * 2.0 / 3.0;
    ret += (20.0 * sin(latitude * M_PI) + 40.0 * sin(latitude / 3.0 * M_PI)) * 2.0 / 3.0;
    ret += (160.0 * sin(latitude / 12.0 * M_PI) + 320 * sin(latitude * M_PI / 30.0)) * 2.0 / 3.0;
    return ret;
}

double TransformLng(double longitude, double latitude)
{
    double ret = 300.0 + longitude + 2.0 * latitude + 0.1 * longitude * longitude + 0.1 * longitude * latitude + 0.1 * sqrt(fabs(longitude));
    ret += (20.0 * sin(6.0 * longitude * M_PI) + 20.0 * sin(2.0 * longitude * M_PI)) * 2.0 / 3.0;
    ret += (20.0 * sin(longitude * M_PI) + 40.0 * sin(longitude / 3.0 * M_PI)) * 2.0 / 3.0;
    ret += (150.0 * sin(longitude / 12.0 * M_PI) + 300.0 * sin(longitude / 30.0 * M_PI)) * 2.0 / 3.0;
    return ret;
}

Time ParseTime(const std::string &str)
{
    std::vector<std::string> items = Split(str, '/');
    int year_month_day = std::atoi(items[0].c_str());
    int hour_minute_second = std::atoi(items[1].c_str());
    Time time = {
        year_month_day / 10000,
        (year_month_day / 100) % 100,
        year_month_day % 100,
        hour_minute_second / 10000,
        (hour_minute_second / 100) % 100,
        hour_minute_second % 100
    };
    return time;
}

int MakeDirection(const char *path)
{
#ifdef __linux__
    return mkdir(path, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
#endif
#ifdef _WIN64
    return mkdir(path);
#endif
}

FileInfo::FileInfo(std::string path)
{
#ifdef __linux__
    dp_ = opendir(path.c_str());
#endif
#ifdef _WIN64
    intptr_t handle_ = _findfirst((path + "*.txt").c_str(), &file_);
    index_ = 0;
#endif
}

FileInfo::~FileInfo() = default;

bool FileInfo::IsOpen() const
{
#ifdef __linux__
    if (!dp_)
    {
        return false;
    }
#endif
#ifdef _WIN64
    if (handle_ == 0)
    {
        return false;
    }
#endif
    return true;
}

std::string FileInfo::Next()
{
#ifdef __linux__
    int n = 0;
    while ((file_ = readdir(dp_)))
    {
        std::string name(file_->d_name);
        if ((n = name.find_last_of('.')) != std::string::npos && name.substr(n) == ".txt")
        {
            return name;
        }
    }
    return "";
#endif
#ifdef _WIN64
    if (index_ == 0)
    {
        index_ += 1;
        return std::string(file_.name);
    }
    else
    {
        index_ += 1;
        if (_findnext(handle_, &file_) > -1)
        {
            return std::string(file_.name);
        }
        else
        {
            return "";
        }
    }
#endif
}

void FileInfo::Close()
{
#ifdef __linux__
    closedir(dp_);
#endif
#ifdef _WIN64
    _findclose(handle_);
#endif
}