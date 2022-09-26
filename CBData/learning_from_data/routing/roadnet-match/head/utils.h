#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <cmath>

#ifdef __linux__ 
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#endif
#ifdef _WIN64
#include <direct.h>
#include <io.h>
#endif

using point_t = std::vector< double >;      // Latitude Longitude
using indexArr = std::vector< size_t >;
using pointIndex = typename std::pair< std::vector< double >, size_t >;
using pointVec = std::vector< point_t >;
using pointIndexArr = typename std::vector< pointIndex >;
using road_t = std::pair<int, int>;

const double EE = 0.00669342162296594323;   // Square of eccentricity
const double A = 6378245.0;                 // Long axis of the earth
const int SAMPLE_INTERVAL = 1;              // Sample time interval
const int OUTPUT_SAMPLE_RATIO = 1;          // Output time interval to sample time interval

struct Time
{
    int year_;
    int month_;
    int day_;
    int hour_;
    int minute_;
    int second_;
    int64_t operator - (const Time &t) const    // the smallest unit: minite
    {
        return (int64_t)(year_ - t.year_) * 365 * 1440 +
                (month_ - t.month_) * 30 * 1440 +
                (day_ - t.day_) * 1440 +
                (hour_ - t.hour_) * 60 +
                (minute_ - t.minute_);
    }
    Time operator + (int i) const           // the smallest unit: minite
    {
        Time t = *this;
        t.minute_ += i;
        if (t.minute_ > 59)
        {
            int carry = t.minute_ / 60;
            t.minute_ -= carry * 60;
            t.hour_ += carry;
            if (t.hour_ > 23)
            {
                carry = t.hour_ / 24;
                t.hour_ -= carry * 24;
                t.day_ += carry;
                if (t.day_ > 29)
                {
                    carry = t.day_ / 30;
                    t.day_ -= carry * 30;
                    t.month_ += carry;
                    if (t.month_ > 12)
                    {
                        carry = (t.month_ - 1) / 12;
                        t.month_ -= carry * 12;
                        t.year_ += carry;
                        return t;
                    }
                    return t;
                }
                return t;
            }
            return t;
        }
        return t;
    }
    bool operator < (const Time &t) const
    {
        uint64_t this_second = (uint64_t)year_ * 365 * 24 * 3600 +
                month_ * 30 * 24 * 3600 +
                day_ * 24 * 3600 +
                hour_ * 3600 +
                minute_ * 60 +
                second_;
        uint64_t t_second = (uint64_t)t.year_ * 365 * 24 * 3600 +
                t.month_ * 30 * 24 * 3600 +
                t.day_ * 24 * 3600 +
                t.hour_ * 3600 +
                t.minute_ * 60 +
                t.second_;
        return this_second < t_second;
    }
    friend std::ostream & operator << (std::ostream &out, const Time &t)
    {
        out << t.hour_ << ":" << t.minute_;
        return out;
    }
};

const Time MAX_TIME = {2019, 1, 10, 23, 59, 59};
const Time MIN_TIME = {2019, 1, 10, 0, 0, 0};

struct RoadVehicleInfo
{
    double avg_speed_;
    int vehicle_num_;
    int vehicle_in_;
    RoadVehicleInfo()
    {
        avg_speed_ = 0.0;
        vehicle_num_ = 0;
        vehicle_in_ = 0;
    }
};

struct RoadInfo
{
    double length_;
    int dir_id_;
    RoadInfo(double length, int dir_id):
        length_(length), dir_id_(dir_id) {}
};

class FileInfo
{
private:
#ifdef __linux__
    DIR* dp_;
    dirent* file_;
#endif
#ifdef _WIN64
    int index_;
    intptr_t handle_;
    _finddata_t file_;
#endif
public:
    FileInfo(std::string path);
    ~FileInfo();
    bool IsOpen() const;
    std::string Next();
    void Close();
};

std::vector<std::string> Split(const std::string &str, char delim);
void PrintStep(int iter);
point_t CoordTrans(double latitude, double longitude);
double TransformLat(double latitude, double longitude);
double TransformLng(double latitude, double longitude);
Time ParseTime(const std::string &str);
int MakeDirection(const char *path);

#endif // UTILS_H
