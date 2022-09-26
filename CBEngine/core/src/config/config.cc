#include "../head/config.h"

#include <cstdio>
#include <iostream>
#include <sstream>

using namespace std;

Config::Config(const std::string &file_addr) {
  digit_params.clear();
  FILE *fp = fopen(file_addr.c_str(), "r");
  string s;
  char lineBuf[200];
  char addrBuf[200];
  while (!feof(fp)) {
    char *getS = fgets(lineBuf, 200, fp);
    if (getS == NULL)
      continue;
    s = string(getS);
    if (s.empty() || s == "\n")
      continue;
    if (string::npos != s.find("#"))
      continue;
    if (string::npos != s.find(":")) {
      sscanf(s.c_str(), "%s : %s", lineBuf, addrBuf);
      string_params.emplace(string(lineBuf), string(addrBuf));
      continue;
    }
    size_t pos_before = s.find("=");
    size_t pos = pos_before + 1;
    for (; pos_before >= 0 && s.at(pos_before) == ' '; --pos_before)
      ;
    if (pos_before <= 0)
      exit(1);
    for (; pos < s.size() && s.at(pos) == ' '; ++pos)
      ;
    string s_before = s;
    s = s.substr(pos);
    if (s.empty() || !((s.at(0) >= '0' && s.at(0) <= '9') || s.at(0) == '-')) {
      std::cout << "error parameter: " << getS << s << std::endl;
      exit(1);
    }
    int p;
    // sscanf(s.c_str(), "%s = %d", lineBuf, &p);
    std::istringstream iss(s);
    iss >> p;
    digit_params[s_before.substr(0, pos_before - 1)] = p;
//    std::cout << s_before.substr(0, pos_before - 1) << " = " << p << std::endl;
  }
}

Config::~Config() {}

int Config::GetMaxTimeEpoch() { return digit_params.at("max_time_epoch"); }

int Config::GetStartTimeEpoch() { return digit_params.at("start_time_epoch"); }

int Config::GetVehicleFakeNum() { return digit_params.at("vehicle_fake_num"); }

int Config::GetReplanningPerSec() {
  return digit_params.at("replanning_per_sec");
}

int Config::GetWarningTooClose() {
  return digit_params.at("warning_too_close");
}

int Config::GetWarningStopTime() {
  return digit_params.at("warning_stop_time");
}

int Config::GetWarningStopTimeLog() {
  return digit_params.at("warning_stop_time_log");
}

std::string Config::GetRoadAddr() { return string_params.at("road_file_addr"); }

std::string Config::GetVehicleFileAddr() {
  return string_params.at("vehicle_file_addr");
}

std::string Config::GetReportlogAddr() {
  return string_params.at("report_log_addr");
}

std::string Config::GetReportlogMode() {
  return string_params.at("report_log_mode");
}

std::string Config::GetReplanningTheFixed() {
  return string_params.at("replanning_the_fixed");
}

int Config::GetReportlogRate() { return digit_params.at("report_log_rate"); }

long long Config::GetReportLogDebugTrafficID() {
  return stoll(string_params.at("report_log_debug_traffic_id"));
}
