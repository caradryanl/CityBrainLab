#ifndef FILTER_H
#define FILTER_H

#include <vector>
#include <unistd.h>
#include <fstream>
#include <stack>

#include "utils.h"

class Filter
{
public:
    static Time min_time_;
    static Time max_time_;
    Filter();
    void Filtrate(std::string input_path, std::string output_path);
    inline static int TimeToIndex(const Time &time);
};

inline int Filter::TimeToIndex(const Time &time)
{
    return (time - min_time_ > 0)? ((time - min_time_) / SAMPLE_INTERVAL) : -1;
}

#endif // FILTER_H
