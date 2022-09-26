import os
import sys
import time

os.system('export SUMO_HOME="/root/sumo"')
if '/root/sumo/bin' not in sys.path:
    sys.path.append('/root/sumo/bin')
if '/root/sumo/tools' not in sys.path:
    sys.path.append('/root/sumo/tools')

import traci

SUMO_Binary = '/root/sumo/bin/sumo'
SUMO_Cmd = [SUMO_Binary, '-c', './cfg/config.sumocfg']

traci.start(SUMO_Cmd)
ts_id_list = traci.trafficlight.getIDList()
max_vehicle_num = 0
start_time = time.time()

for step in range(3600):
    for ts_id in ts_id_list:
        if int(traci.trafficlight.getNextSwitch(ts_id)) >= 1:
            traci.trafficlight.setPhase(ts_id, traci.trafficlight.getPhase(ts_id))
    total_sum = len(traci.vehicle.getIDList())
    # for veh_id in traci.vehicle.getIDList():
    #     traci.vehicle.getLaneID(veh_id)
    vehicle_num = traci.simulation.getMinExpectedNumber()
    if vehicle_num > max_vehicle_num:
        max_vehicle_num = vehicle_num
    print('t: {}, v: {}, total_sum: {}'.format(step, vehicle_num, total_sum))
    traci.simulationStep()
end_time = time.time()
traci.close()

print('Runtime: {}, max vehicle num: {}'.format(end_time - start_time, max_vehicle_num))