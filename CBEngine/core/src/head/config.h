#pragma once

#include <algorithm>
#include <cstring>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

class Config {
public:
  Config(const std::string &file_addr);
  ~Config();

  std::string GetRoadAddr();
  std::string GetVehicleFileAddr();
  int GetStartTimeEpoch();
  int GetMaxTimeEpoch();
  int GetVehicleFakeNum();
  int GetReplanningPerSec();
  int GetWarningTooClose();
  int GetWarningStopTime();
  int GetWarningStopTimeLog();

  std::string GetReportlogAddr();
  std::string GetReportlogMode();
  std::string GetReplanningTheFixed();
  int GetReportlogRate();
  long long GetReportLogDebugTrafficID();

private:
  std::unordered_map<std::string, int> digit_params;
  std::unordered_map<std::string, std::string> string_params;
};
