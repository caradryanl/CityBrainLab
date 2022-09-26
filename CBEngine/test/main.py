import os, sys
import time
import cbengine
from utils import Dataloader


roadnet_file = './data/roadnet.txt'
flow_file = './data/flow.txt'
cfg_file = './cfgs/config.cfg'
dataloader = Dataloader(roadnet_file, flow_file, cfg_file)

def main():
    running_step = 300
    phase_time = 30
    engine = cbengine.Engine(cfg_file, 12)

    print('Simulation starts ...')
    start_time = time.time()
    for step in range(running_step):
        for intersection in dataloader.intersections.keys():
            engine.set_ttl_phase(intersection, (int(engine.get_current_time()) // phase_time) % 4 + 1)
        engine.next_step()
        print(" time step: {}, number of vehicles: {}".format(step, engine.get_vehicle_count()))
    end_time = time.time()
    print('Simulation finishes. Runtime: ', end_time - start_time)
    
    

if __name__ == '__main__':
    main()