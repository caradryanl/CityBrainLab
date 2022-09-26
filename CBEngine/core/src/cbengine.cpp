#include <pybind11/pybind11.h>
#include "../externals/pybind11/include/pybind11/stl.h"
#include "head/core.h"

namespace py = pybind11;
using namespace py::literals;

PYBIND11_MODULE(cbengine, m){
    py::class_<Core>(m, "Engine")
    .def(py::init(&Core::create_engine))
    .def("reset", &Core::Reset)
    .def("get_vehicle_count", &Core::GetVehicleCount)
    .def("get_vehicles", &Core::GetVehicles)
    .def("get_lane_vehicle_count", &Core::GetLaneVehicleCount)
    .def("get_lane_vehicles", &Core::GetLaneVehicles)
    .def("get_lane_waiting_vehicle_count", &Core::GetLaneWaitingVehicleCount)
    .def("get_vehicle_speed", &Core::GetVehicleSpeed)
    .def("get_current_time", &Core::GetCurrentTime)
    .def("get_average_travel_time", &Core::GetAverageTravelTime)
    .def("get_vehicle_info", &Core::GetVehicleInfo)
    .def("get_road_speed_limit", &Core::GetRoadSpeedLimit)
    .def("set_road_velocity", &Core::SetRoadVelocity)
    .def("save_state", &Core::SaveState)
    .def("load_state", &Core::LoadState)
    .def("get_car_following_params", &Core::GetDrivingParams)
    .def("set_car_following_params", &Core::SetDrivingParams)
    .def("get_vehicle_route", &Core::GetVehicleRoute)
    .def("set_vehicle_route", &Core::SetVehicleRoute)
//    .def("reset", &Core)
    .def("set_ttl_phase", &Core::SetTLPhase)
    .def("get_ttl_phase", &Core::GetTLPhase)
//    .def("load_signal_plan", &Core)
    .def("next_step", &Core::NextStep)
    .def("log_info", &Core::LogInfo)
    .def("DebugSignal", &Core::DebugSignal)
    .def("log_vehicle_info", &Core::LogVehicleInfo)
    
    ;
}