#pragma once
#include <cstdlib>
#include <cstring>
#include <random>

namespace Common {
int &GetTimer();
int &GetCarArriveCounter();
double &GetCarArriveTime();
bool &GetReplanningTheFixed();
int RandomInt(int s, int t);
void seed(unsigned  int seed);
} // namespace Common
