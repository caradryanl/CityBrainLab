import pickle
from random import randint
import json
import os

method = "random"

single_ods = []
with open("./Result/Generated_OD/Single_OD.pkl", "rb") as f:
    single_ods = pickle.load(f)

max_time = 3600
    
vehicle_flows = []
params = []

if os.path.exists(os.path.join(os.getcwd(), "flow", "param.json")):
    with open(os.path.join(os.getcwd(), "flow", "param.json"), "r") as f:
        params = json.load(f)
    if len(single_ods) == len(params):
        method = "param"

print("Generate vehicle flow in {} method".format(method))
if method == "random":
    for single_od in single_ods:
        vehicle_flow = {}
        vehicle_flow["route"] = [str(item) for item in single_od[1:-1]]
        vehicle_flow["interval"] = 1
        rand = randint(0, max_time - 1)
        vehicle_flow["startTime"] = rand
        vehicle_flow["endTime"] = rand
        param = {"interval" : 1.0, "startTime" : float(rand), "endTime" : float(rand)}
        vehicle_flows.append(vehicle_flow)
        params.append(param)
else:
    for param, single_od in zip(params, single_ods):
        vehicle_flow = {}
        vehicle_flow["route"] = [str(item) for item in single_od[1:-1]]
        vehicle_flow["interval"] = param["interval"]
        vehicle_flow["startTime"] = param["startTime"]
        vehicle_flow["endTime"] = param["endTime"]
        vehicle_flows.append(vehicle_flow)

vehicle_flows_sorted = sorted(vehicle_flows, key=lambda x : x["startTime"])

with open(os.path.join(os.getcwd(), "flow", "vehicle_flow.txt"), "w") as f:
    f.write(str(len(single_ods)) + os.linesep)
    line_index = 0
    for vehicle_flow in vehicle_flows_sorted:
            line = str(vehicle_flow["startTime"]) + " " + str(vehicle_flow["endTime"]) + " " + str(vehicle_flow["interval"])
            f.write(line + os.linesep)
            f.write(str(len(vehicle_flow["route"])) + os.linesep)
            line = ""
            for item in vehicle_flow["route"]:
                line += item + " "
            f.write(line[:-1] + os.linesep)

if method == "random":
    params_sorted = sorted(params, key=lambda x : x["startTime"])        
    with open(os.path.join(os.getcwd(), "flow", "param.json"), "w", encoding="utf8") as f:
        json.dump(params_sorted, f, ensure_ascii=False)