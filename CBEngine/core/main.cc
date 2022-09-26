#include <iostream>

#include "src/head/config.h"
#include "core.h"

int main(int argc, char *argv[]) {
  long long int agent_list[22] = {14670355735, 22318148293, 14790105773, 14454920703, 12365406899, 13987210067, 42167350479, 42167350456, 42265990106, 42167350476, 44052836274, 42167350445, 42365406897, 42365409354, 42294463892, 42167350405, 42294477575, 42167350420, 22372852612, 42381408549, 42167350403, 22318148289};
  int agent_len = 22;
  try {
    Config cfg = Config(argv[1]);
    Core *core = new Core(cfg,4);
    int flg = 0;
    int id = 0;
    for (TrafficFlow *flow :  core->traffic_flows_->flows_) {
      id++;
      if(flg) {
        for (DynamicRoad *road : flow->path_) {
          std::cout << road->unique_id_ << std::endl;
        }
        std::cout << "------------------------" << std::endl;
        flg = 0;
      }
      if(flow->path_.empty())
      {std::cout<<"123"<<std::endl;flg = 1;std::cout<<id<<std::endl;}
    }
    std::cout << "Hello, World!" << std::endl;
//    core->Run();
    int max_step = cfg.GetMaxTimeEpoch();
    std::vector<int> v_list;
    for (int i = 0; i < max_step; i++) {
      core->NextStep();
      if(i % 10 == 0) {
        for (int kk = 0; kk < agent_len; kk++) {
          core->SetTLPhase(agent_list[kk], (i / 10) % 4 + 5);
        }
        v_list = core->GetVehicles();
        for (int v:v_list){
          core->GetVehicleInfo(v);
        }
        core->LogInfo(cfg.GetReportlogAddr() + "time" +
                      std::to_string(i /
                                     cfg.GetReportlogRate()) + ".json");
      }

      if(i % 100 == 0){
        core->LogVehicleInfo("./info_step 100.log");
      }
        printf("step%d/%d+++++++++++++++++++++++++++++++++++++++\n",i,max_step);
  //    for(auto i:core->GetVehicles()) {
  //      std::cout << "for vehicle " << i << std::endl;
  //      for (auto mp: core->GetVehicleInfo(i)) {
  //    //              std::cout<<mp.first<<":"<<mp.second<<std::endl;
  //        std::cout << mp.first << ":";
  //        for (auto val:mp.second) {
  //          std::cout << val << " ";
  //        }
  //        std::cout << std::endl;
  //      }
  //      std::cout << "---------------------" << std::endl;
  //    }
    }
    std::cout << "Happy Ending!" << std::endl;

    delete core;
  } catch (const std::invalid_argument &ia) {
    std::cerr << "Invalid argument: " << ia.what() << '\n';
  }
  return 0;
}
