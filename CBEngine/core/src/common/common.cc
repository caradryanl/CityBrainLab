//
// Created by Apia Du on 2020/12/21.
//

#include "../head/common.h"

int &Common::GetTimer() {
  static int timer = 0;
  return timer;
}

int &Common::GetCarArriveCounter() {
  static int cnt = 0;
  return cnt;
}

double &Common::GetCarArriveTime() {
  static double tot_time = 0.0;
  return tot_time;
}

int Common::RandomInt(int s, int t) {
  // std::mt19937 rng(0);
  if (s >= t)
    return s;
  return rand() % (t - s + 1) + s;
}

void Common::seed(unsigned  int seed){
  srand(seed);
}

bool &Common::GetReplanningTheFixed() {
  static bool replan = false;
  return replan;
}
