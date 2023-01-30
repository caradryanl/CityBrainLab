from itertools import starmap
import networkx as nx
import random
import pandas as pd
import pickle
import json
import time
import multiprocessing as mp
import xml.dom.minidom as minidom


class Traffic_generator:
    def __init__(self, city, main_folder, output_folder, numveh, delete_opposite=True, traffic_duration=1200, beta=0.2):

        self.numveh = numveh
        self.beta = beta
        self.city = city
        self.main_folder = main_folder
        self.data_folder = output_folder
        #  self.roadnet_path = self.data_folder + self.city + ".txt"
        self.roadnet_path = main_folder + "roadnet_dict.pkl"

        self.delete_opposite = delete_opposite
        self.traffic_duration = traffic_duration
        self.edge_opposite = [] 
        self.edge_data = {
                        'case_id': [],
                        'cfg_name': [],
                        'edge_id': [],
                        'numveh': [],
                        'length': [],
                        'inter_ids': [],
                        } 

        self.percent_veh = [.6, .4]

    def initialize(self, cpus):
        print('Start to generate 0_flow.txt.')
        self.G = None
        self.cpus = cpus

        return self.single_run(-1)

    def single_run(self, run):
        '''
        :initialize the traffic generator or generate traffic 1 time
        ''' 
        t0 = time.time()      

        # this way for normal generation
        print('start initialization and the first run!')
        self.fl = Flow(self.traffic_duration, self.roadnet_path)
        
        
        self.fl.divide_roadNet(numRows=6, numColumns=8, Oprob_mode=3)

        # generate traffic and write edge_list and roadgraph to pickles
        self.fl.generate_traffic(numVeh=self.numveh, percentVeh=self.percent_veh, cpus=self.cpus)
        
        # self.fl.check_flow()
        self.write_data(run)
        self.fl.sort_edges(self.delete_opposite)
        
        
        t1 = time.time()
        print("single_run time is {}".format(t1-t0)) 
            
    def write_data(self, run):    
        '''write data to files'''
        # step-1 write generated traffic flow data
        cbengine_data_path = self.data_folder + 'CBEngine_flow.txt'
        self.fl.output_cbengine(output_path=cbengine_data_path)
        cityflow_data_path = self.data_folder + 'Cityflow_flow.json'
        self.fl.output_cityflow(output_path=cityflow_data_path)
        sumo_data_path = self.data_folder + 'SUMO_flow.xml'
        self.fl.output_sumo(output_path=sumo_data_path)

        # step-2 write config file
        # road_file_add = "road_file_addr : " + self.roadnet_path
        # default_content0 = ['start_time_epoch = 0', 'max_time_epoch = 1200', road_file_add ]
        # default_content1 = ['report_log_mode : normal', 'report_log_rate = 10', 'warning_stop_time_log = 100']
        # flow_cfg = "vehicle_file_addr : " + self.data_folder + str(run+1) + '_flow.txt'
        # log_folder = self.main_folder + self.city + "/log/"
        # log_cfg = "report_log_addr : " + log_folder + str(run+1) +'/'
        
        # cfg_folder = self.main_folder
        # cfg_path = cfg_folder + str(run+1) + '_flow.cfg'
        # f = open(cfg_path, 'w')
        # f.write("#configuration for simulator \n")

        # for content in default_content0:
        #     f.write(content + "\n")
        # f.write(flow_cfg + "\n")
        # f.write(log_cfg + "\n")
        # f.write("\n")

        # for cont in default_content1:
        #     f.write(cont + "\n")
        # f.close()
        
            

class Flow:
    def __init__(self, traffic_duration=1200, roadnet_path=None, beta=.2):
        '''
        :param roadmap_path: road map file
        :param graph: whether to build road graph again
        :param roadgraph_path: path to load road graph
        '''
        # initial run and get roadgraph from roadnet.txt file
        self.roadnet_path = roadnet_path
        self.roadgraph = None # a networkX Graph instance
            # read roadnet.txt file
        # self.read_roadnet(roadnet_path=roadnet_path) 
        self.read_pkl(roadnet_path=roadnet_path)
        
        self.generate_roadGraph()
        self.edge_id_list = [self.roadgraph[e[0]][e[1]]['id'] for e in self.roadgraph.edges()]
        
        self.edge_flow = dict()
        self.Veh_num_cur = 0 # total number of generated vehicle trips until now
        self.zone_info = None # zone information data
        self.Oprob = None # probabilities of a vehicle departs from Origin traffic zones
        self.ODprob = None # probabilities of a vehicle depart from an Origin zone and arrived into a Destination zone
        self.traffic_duration = traffic_duration # By default, 1200-second traffic sample data is generated
        self.beta = beta # parameter for get road graph edge
        

    def clear(self):
        self.flow = dict()
        self.Veh_num_cur = 0
        self.zone_info = None

    def divide_roadNet(self, numRows=6, numColumns=8, Oprob_mode=3, prob_corner=0.2, prob_mid=0.15):
    
        '''
        Divide road network into numRows * numColumns rectangle traffic zones
        return: IDs (rowIndex, columnIndex) of created traffic sub-zones
        '''
        if type(numRows) != int or type(numColumns) != int:
            exit("Please enter integer numRows and numColumns.")

        self.numRows = numRows
        self.numColumns = numColumns
        
        self.unit_lon = (self.right_lon - self.left_lon) / self.numColumns
        print(self.right_lon, self.left_lon, self.numColumns, self.unit_lon)
        self.unit_lat = (self.top_lat - self.bottom_lat) / self.numRows
        self.zone_info = {}
        print("Row:{}, Col:{}".format(self.numRows, self.numColumns))
        for row in range(self.numRows):
            for col in range(self.numColumns):
                self.zone_info[(row, col)] = {
                    'inters_id': [], # a list of IDs of intersections
                    'left_lon': self.left_lon + col*self.unit_lon, # leftmost longitude of the zone
                    'right_lon': self.left_lon + (col+1)*self.unit_lon, # rightmost logitude of the zone
                    'top_lat': self.top_lat - row*self.unit_lat, # top latitude of the zone
                    'bottom_lat':self.top_lat - (row+1)*self.unit_lat, # bottom latitude of the zone
                    'num_inters': 0, # number of intersections within the zone
                    'num_signals': 0, # number of signalized intersections within the zone
                    'roadlen': 0.0, # road length within the zone
                    'roadseg':0 # number of road segments within the zone
                }
        for key in self.inter_info.keys():
            zone_id = self.get_inter_zoneID(key)
            self.zone_info[zone_id]['inters_id'].append(key)
            self.zone_info[zone_id]['num_inters'] += 1
            self.zone_info[zone_id]['num_signals'] += self.inter_info[key]['sign']
            self.zone_info[zone_id]['roadlen'] += self.inter_info[key]['roadlen']
            self.zone_info[zone_id]['roadseg'] += self.inter_info[key]['roadseg']
        print('Divide road network into {}*{} traffic zones successfully!'.format(self.numRows, self.numColumns))
        
        # Estimate the OD matrix (in form of probabilities) based on road network information
        self.get_Oprob(Oprob_mode) # Get probabilities of a vehicle departs from zones (Origin zone) of the road network
        self.get_ODprob(prob_corner, prob_mid) #Get probabilities of a vehicle depart from an Origin zone and arrived into a Destination zone
        
        return self.zone_info.keys()
    
    
    def generate_traffic(self, numVeh=5000, percentVeh=[.1, .1, .05, .05, .1, .1, .05, .05, .15, .15, .05, .05], cpus=16):
        '''
        Generate initial traffic flow data given the road network data 
        :param numVeh: total number of vehicles that will enter the network in 1-hour
        :param percentVeh: percentages of vehicles that will enter the network in each period (e.g., in 4-minute)  
        :param weight: the larger the weight, the more diverse of route choice given Origin-Destination of a trip 
        '''
        print("Start to generate initial traffic flow!")
            # by default, use length + beta * num_expected_passed_vehicle as edge weight to find shortest_path
            # if mode != None, other options for edge weight

        if self.zone_info is None:
            exit('You need to divide road network into traffic sub-zones!')
        
        if abs(sum(percentVeh) - 1) > 0.001:
            exit('sum of percentages of vehicle entering in network over time should be 1!')

        num_intervals = len(percentVeh) # total number of periods with different traffic demands
        interval_length  = self.traffic_duration / len(percentVeh) # compute the duration of one period (e.g., 600 seconds)
        numVeh_perInterval = [int(percent * numVeh) for percent in percentVeh] # number of vehicles entering the network over periods

        # step 1 create params
        res_list = list()
  
        for interval in range(num_intervals):
            params = list()
            for o_zone in self.zone_info.keys():
                if len(self.zone_info[o_zone]['inters_id']) == 0:
                    continue
                num_per_inter = numVeh_perInterval[interval] * self.Oprob[o_zone] / self.zone_info[o_zone]['num_inters'] # number of vehicles per intersection
                for o_interid in self.zone_info[o_zone]['inters_id']:
                    d_zone = self.random_weight_choose(self.ODprob[o_zone]) # choose destination zone given ODprobs
                    while len(self.zone_info[d_zone]['inters_id']) == 0:
                        d_zone = self.random_weight_choose(self.ODprob[o_zone])
                    d_interid = random.choice(self.zone_info[d_zone]['inters_id'])
                    
                    # start_time = random.randint(interval * interval_length, interval * interval_length + 1600) # start time is in range of interval begining and +10 seconds
                    start_time = random.randint(interval * interval_length, interval * interval_length + 10) # start time is in range of interval begining and +10 seconds
                    end_time = (interval + 1) * interval_length

                    params.append((o_interid, d_interid, num_per_inter, start_time, end_time)) 
            print("total od: {}".format(len(params)))
                    
        # step 2 run parallel trip generation
            # step -2.1 initialize pool and start to run
            # result = starmap(self._generate_trip, params)
            # print("demo result is {}".format(list(result)[0]))
            # print("length of params is {} for interval - {}".format(len(params), interval))
            
            pool = mp.Pool(cpus)
            smap = pool.starmap_async(self._generate_trip, params)
            tmp_result = smap.get()
            print(type(tmp_result))
            pool.close()
            pool.join()

        # step 3 post-processing to merge tmp_result
            # update flows and edges data, update roadgraph with newly added numveh
            res = self._process_tmp_results(tmp_result)
            res_list.append(res)

        final_results = self._merge_final_results(res_list)
        self.flow = final_results[0]
        # final step: write edge flow data to json files   
        self._write_edge_flow(final_results)
    
    
    def regenerate_traffic(self, del_edges, edge_flow, init_flows, road_graph, cpus=16):
        '''
        :load initial traffic flow and re-generate new traffic flow 
        :edge_flow: {edge_id: flow_id_list}
        '''

        if not type(del_edges) == list:
            del_edges = [del_edges]
        
        print("delete_edges are {}".format(del_edges))
        
        self.roadgraph = road_graph
        self.flow = init_flows

        regen_flows = []

        for e in del_edges:
            flow_ids = edge_flow[str(e)]
            regen_flows += flow_ids
        
        # print("regenerate flows is {}".format(regen_flows))
        
        pool = mp.Pool(cpus)
        smap = pool.map_async(self._regenerate_trip, regen_flows)
        tmp_result = smap.get()
        pool.close()
        pool.join()

        #post-processing
        non_null_count = 0
        for res in tmp_result:
            if res:
                self.flow.update(res)
            else:
                non_null_count += 1

        print("after delete, total number of vehicle is {}".format(non_null_count))

        # # post-process tmp_result & get self.flow updated
        # for res in tmp_result:
        #     if res == None:
        #         continue
        #     else:
        #         self.flow.update(res)
        # for flow in regen_flows:
        #     self._regenerate_trip(flow)

    def _regenerate_trip(self, flow_id):
        if type(flow_id) != str:
            flow_id = str(flow_id)

        flow_input = self.flow[flow_id][:5]
   

        # update with new traffic flow
        try:
            (tmp_flow, _) = self._generate_trip(*flow_input, re_generate=True)
            # delete the existing flow   
            del self.flow[flow_id]
        except:
            tmp_flow = None
            # print("cannot find another alternative path")

        # self.flow.update(tmp_flow)
        return tmp_flow
    
    def check_flow(self):
        print("check generated flow data")
        for flow_data in self.flow.values():
            i = flow_data[3:]
            for j in range(len(i)):
                if j in range(4):
                    continue
                else:
                    in_g = (i[j] in self.edge_id_list)
                    if not in_g:
                        print("{} is not in roadgraph".format(i[j]))
                        
    # input OD, output a list record
    def _generate_trip(self, O_interid, D_interid, numVeh, start_time, end_time, re_generate=False):
        '''
        :1) generate flow of trips data
        :2) get edge flow pair information
        '''
        start_time = int(start_time)
        end_time = int(end_time)

        flow_id = str(O_interid) + '_' + str(D_interid) + '_' + str(start_time)
        flow_data = dict()
        edge_flow = dict()

        # step 1: run shortest_path, default - Dijkstra 

        try:
            path_nodes = nx.shortest_path(self.roadgraph, source=O_interid, target=D_interid, weight='weight') # return a list of intersection IDs
        except:
            # print("no path nodes")
            return None

        # step 2: post-processing

        if len(path_nodes) <= 3: # we omit the vehicle trips with number of edges <= 2
            # print("path nodes less than 3")
            return None

        interval = max(1,int((end_time - start_time) / numVeh))
        tmp_data = []

        # append flow information for re-generate traffic
        tmp_data.append(O_interid)
        tmp_data.append(D_interid)
        tmp_data.append(numVeh)

        # append shared information
        tmp_data.append(start_time)
        tmp_data.append(end_time)

        # todo1

        # append information to write flow data
        tmp_data.append(interval)
        tmp_data.append(len(path_nodes) - 1) # append number of edges (road segments)

        if re_generate:
            for idx in range(len(path_nodes) - 1):
                source = path_nodes[idx]
                target = path_nodes[idx + 1]

                edge_id = self.roadgraph[source][target]['id'] 
                tmp_data.append(edge_id)
            
            flow_data[flow_id] = tmp_data
            return (flow_data, edge_flow)

        for idx in range(len(path_nodes) - 1):
            source = path_nodes[idx]
            target = path_nodes[idx + 1]

            edge_id = self.roadgraph[source][target]['id'] 

            tmp_data.append(edge_id)
            if edge_id in edge_flow.keys():
                edge_flow[edge_id]['flows'].append(flow_id)
            else:
                tmp_info = {'numveh': numVeh, 'nodes': (source, target), 'flows': [flow_id]}
                edge_flow[edge_id] = tmp_info
        
        flow_data[flow_id] = tmp_data
        return (flow_data, edge_flow)

    def _process_tmp_results(self, results):
        '''
        merge flows and edge_flow data
        :input: a list of temp results from multiprocessing
        '''
        flows = dict()
        edges = dict()
        for res in results:
            if res == None:
                continue

            else:
                (flow, edge_flow) = res
                # step 1 merge flows
                flows.update(flow)

                # step 2: update flow_ids and numveh of edge_flow dict
                for e_id in edge_flow.keys():
                    if e_id in edges.keys():
                        edges[e_id] += edge_flow[e_id]['flows']
                    else:
                        edges[e_id] = edge_flow[e_id]['flows']

            # step 3: update numveh in self.roadgraph
                
                    num_veh = edge_flow[e_id]['numveh']
                    nodes = edge_flow[e_id]['nodes']
                    self.roadgraph[nodes[0]][nodes[1]]['numveh'] += num_veh
                    self.roadgraph[nodes[0]][nodes[1]]['weight'] += self.beta * num_veh
        
        return (flows, edges)

    def _merge_final_results(self, results):
        flows = dict()
        edges = dict()

        for res in results:
            (flow, edge_flow) = res
            flows.update(flow)

            # update flow id list of edges
            for e in edge_flow.keys():
                if e in edges.keys():
                    edges[e] += edge_flow[e]
                else:
                    edges[e] = edge_flow[e]
        
        return (flows, edges)

    def _write_edge_flow(self, result):
        flow_path = self.roadnet_path.replace('.pkl', '-flows.json')
        ef_path = self.roadnet_path.replace('.pkl', '-ef.json')

        (flows, edges) = result

        with open(flow_path, 'w') as f1:
            json.dump(flows, f1)

        with open(ef_path, 'w') as f2:
            json.dump(edges, f2)

    def get_route(self, O_interid, D_interid, numVeh, start_time, end_time):
        '''
        DEPRECATED: get route from o_inter to d_inter
        :param O_interid:
        :param D_interid:
        :param numVeh:
        :param start_time:
        :param end_time:
        :param weight: 
        :return:
        '''
        start_time = int(start_time)
        end_time = int(end_time)
        try:
            path_nodes = nx.shortest_path(self.roadgraph, source=O_interid, target=D_interid, weight='length') # return a list of intersection IDs
        except:
            return None

        if len(path_nodes) <= 3: # we omit the vehicle trips with number of edges <= 3 
            return None

        interval = max(1,int((end_time - start_time) / numVeh))
        tmp_data = []
        tmp_data.append(numVeh)
        tmp_data.append(start_time)
        tmp_data.append(end_time)
        tmp_data.append(interval)
        tmp_data.append(len(path_nodes) - 1) # append number of edges (road segments)

        for idx in range(len(path_nodes) - 1):
            self.roadgraph[path_nodes[idx]][path_nodes[idx + 1]]['length'] += numVeh * self.beta # update the 'length' of roadgraph considering the number of vehicles passed the edge
            
            edge_id = self.roadgraph[path_nodes[idx]][path_nodes[idx + 1]]['id'] # generate edge (road segment) ID given two adjacent node (intersection) IDs 
            self.roadgraph[path_nodes[idx]][path_nodes[idx + 1]]['numveh'] += numVeh

            tmp_data.append(edge_id)
        self.Veh_num_cur += (end_time-start_time) / interval

        return tmp_data, (end_time-start_time) / interval

    def sort_edges(self, delete_opposite, threshold=[0, 200]):

        # step-1 filter out edges with number of vehicle less than 100veh/20min = 5veh/min
        if delete_opposite:

            edges = [e for e in nx.Graph(self.roadgraph).edges(data=True) if e[-1].get('numveh', -1) > threshold[0] and e[-1].get('length', -1) > threshold[1]]

        else:
            edges = [e for e in self.roadgraph.edges(data=True) if e[-1].get('numveh', -1) > threshold[0] and e[-1].get('length', -1) > threshold[1]]

        edges_sorted = sorted(edges, key=lambda t: t[-1].get('numveh', -1), reverse=True)
        
        # step -2 save sorted edges and roadgraph to (g)pickles
        # gpickle_path = self.roadnet_path.replace('.txt', '.gpickle')
        # nx.write_gpickle(self.roadgraph, gpickle_path)

        graphml_path = self.roadnet_path.replace('.pkl', '_updated.graphml')
        nx.write_graphml(self.roadgraph, graphml_path)

        pickle_path = self.roadnet_path.replace('.pkl', '.pickle')
        with open(pickle_path, 'wb') as f:
            pickle.dump(edges_sorted, f)

    def add_tripOD(self, o_zone, d_zone, start_time, end_time, num_Veh, beta=0.2):
        '''
            Add extra trips from o_zone to d_zone in addition to background traffic
            This will help you simulate traffic during special events, for example, football games
        '''
        if end_time > self.traffic_duration or start_time < 0:
            exit("end_time should be in time range of [0, traffic_duration]!")

        if o_zone[0] >= self.numRows or d_zone[0] > self.numRows:
            exit("The zone row index should be smaller than numRows!")

        if o_zone[1] >= self.numColumns or d_zone[1] > self.numColumns:
            exit("The zone column index should be smaller than numColumns!")

        num = 0
        num_per_inter = num_Veh / self.zone_info[o_zone]['num_inters']
        for o_interid in self.zone_info[o_zone]['inters_id']:
            d_interid = random.choice(self.zone_info[d_zone]['inters_id'])
            try:
                tmp_flow, tmp_num = self.get_route(o_interid, d_interid, num_per_inter, start_time, end_time, beta)
            except TypeError:
                continue
            self.flow.append(tmp_flow)
            num += tmp_num
        print('Adding {} vehicles. Current {} Vehicles'.format(int(num), int(self.Veh_num_cur)))

    def output(self, output_path):
        '''
        Write the traffic flow data into a .txt file
        :param output_path:
        :return: self.zone_info.keys()
        '''
        file = open(output_path, 'w')
        file.write("{}\n".format(len(self.flow)))
        for flow_data in self.flow.values():
            i = flow_data[3:]
            for j in range(len(i)):
                if (j == 2) or (j == 3) or (j == len(i) - 1):
                    file.write("{}\n".format(i[j]))
                else:
                    file.write("{} ".format(i[j]))
        file.close()
        
    def output_cbengine(self, output_path):
        '''
        Write the traffic flow data into a .txt file
        :param output_path:
        :return: self.zone_info.keys()
        '''
        file = open(output_path, 'w')
        file.write("{}\n".format(len(self.flow)))
        for flow_data in self.flow.values():
            i = flow_data[3:]
            for j in range(len(i)):
                if (j == 2) or (j == 3) or (j == len(i) - 1):
                    file.write("{}\n".format(i[j]))
                else:
                    file.write("{} ".format(i[j]))
        file.close()
        
    def output_cityflow(self, output_path):
        '''
        Write the traffic flow data into a .txt file
        :param output_path:
        :return: self.zone_info.keys()
        '''
        flow_list = []
        for flow_data in self.flow.values():
            vehicle = {
                "vehicle": {
                "length": 5.0,
                "width": 2.0,
                "maxPosAcc": 2.0,
                "maxNegAcc": 4.5,
                "usualPosAcc": 2.0,
                "usualNegAcc": 4.5,
                "minGap": 2.5,
                "maxSpeed": 16.67,
                "headwayTime": 1.5
                },
                "route": [str(i) for i in flow_data[7:]],
                "interval": flow_data[5],
                "startTime": flow_data[3],
                "endTime": flow_data[4]
            }
            flow_list.append(vehicle)
        with open(output_path, 'w') as file:
            json.dump(flow_list, file)
        
    def output_sumo(self, output_path):
        '''
        Write the traffic flow data into a .txt file
        :param output_path:
        :return: self.zone_info.keys()
        '''
        dom = minidom.getDOMImplementation().createDocument(None,'routes',None)
        root = dom.documentElement
        vType = dom.createElement('vType')
        vType.setAttribute('id', 'Car')
        vType.setAttribute('accel', "2.6")
        vType.setAttribute('decel', "4.5")
        vType.setAttribute('sigma', "0.5")
        vType.setAttribute('length', "5")
        vType.setAttribute('maxSpeed', "16.67")
        root.appendChild(vType)

        flow_list = []
        for id, flow_data in self.flow.items():
            pass
        sorted_flow = sorted(self.flow.items(), key=lambda flow: flow[1][3])
        for (id, flow_data) in sorted_flow:
            str_route = ""
            for road in flow_data[7:]:
                if road != flow_data[7]:
                    str_route = str_route + " " + str(road)
                else:
                    str_route = str(road)
            flow = dom.createElement('flow')
            flow.setAttribute('id', str(id))
            flow.setAttribute('type', "Car")
            flow.setAttribute('begin', str(flow_data[3]))
            flow.setAttribute('end', str(flow_data[4]))
            flow.setAttribute('period', str(flow_data[5]))
            route = dom.createElement('route')
            route.setAttribute('edges', str_route)
            flow.appendChild(route)
            root.appendChild(flow)
        with open(output_path, 'w', encoding='utf-8') as f:
            dom.writexml(f, addindent='\t', newl='\n',encoding='utf-8')
        '''
        <flow id="type1" color="1,1,0"  begin="0" end= "7200" period="900" type="BUS">
            <route edges="beg middle end rend"/>
        </flow>
        
        <flow id="01" type="Car" begin="0" end="72000" number="2400" 
            from="edge1-0" to="edge0-2" departPos="1" departLane="best" departSpeed="5.0"/>
        '''
        
    def get_Oprob(self, mode=None):
        '''
        Get probabilities of a vehicle departs from zones (Origin zone) of the road network,
        Default method: use road length within zones as default reference for probabilities estimation

        :param mode: 
            1->use road length as reference for origin zone probabilities estimation 
            2->use number of road segments as reference for origin zone probabilities estimation  
            3->use number of intersections as reference for origin zone probabilities estimation 
            4->use number of signalized intersection as reference for origin zone probabilities estimation
        :return:
        '''
        if mode == 1:
            self.Oprob = {(row, col): self.zone_info[(row,col)]['roadlen']
                          for row in range(self.numRows) for col in range(self.numColumns)}
        elif mode == 2:
            self.Oprob = {(row, col): self.zone_info[(row, col)]['roadseg']
                          for row in range(self.numRows) for col in range(self.numColumns)}
        elif mode == 3:
            self.Oprob = {(row, col): self.zone_info[(row, col)]['num_inters']
                          for row in range(self.numRows) for col in range(self.numColumns)}
        elif mode == 4:
            self.Oprob = {(row, col): self.zone_info[(row, col)]['num_signals']
                          for row in range(self.numRows) for col in range(self.numColumns)}
        else:
            exit("Mode Error")
        total = sum(self.Oprob.values())
        for key in self.Oprob.keys():
            self.Oprob[key] /= total
        return self.Oprob

    def get_ODprob(self, prob_corner=0.3, prob_mid=0.8):
        '''
        Get probabilities of a vehicle depart from an Origin zone and arrived into a Destination zone
        Default method: use zone's row and column indices difference to estimate the probabilities

        :param prob_corner: probability for a vehicle's Origin and Destination within the same zone if it is a corner traffic zone, e.g. zone-(0,0)
        :param prob_mid: probability for a vehicle's Origin and Destination within the same zone if it is a middle zone, e.g. zone-(3,4)
        :return:
        '''
        self.ODprob = {}  # (row, col):{(row2, col2): probability}
        for o_row in range(self.numRows):
            for o_col in range(self.numColumns):
                self.ODprob[(o_row, o_col)] = {}
                for d_row in range(self.numRows):
                    for d_col in range(self.numColumns):
                        dis = abs(o_row - d_row) + abs(o_col - d_col)
                        if dis == 0:
                            if (o_row == 0 or o_row == 5 or o_col == 0 or o_col == 7):
                                self.ODprob[(o_row, o_col)][(d_row, d_col)] = prob_corner
                            else:
                                self.ODprob[(o_row, o_col)][(d_row, d_col)] = prob_mid
                        else:
                            self.ODprob[(o_row, o_col)][(d_row, d_col)] = 1 / dis
        return

    def get_zoneInfo(self, zoneid=None):
        '''Return information of a traffic zone given zoneID'''
        if zoneid is None:
            return self.zone_info
        if zoneid[0] not in range(self.numRows) or zoneid[1] not in range(self.numColumns):
            print('Error: zone id')
            return
        print(self.zone_info[zoneid])
        return self.zone_info[zoneid]

    def get_inter_zoneID(self, inter_id):
        '''Given an intersection ID, return the traffic zone ID - (rowIndex, columnIndex)'''
        lat = self.inter_info[inter_id]['lat']
        lon = self.inter_info[inter_id]['lon']
        row = int((lat - self.bottom_lat) / self.unit_lat)
        col = int((lon - self.left_lon) / self.unit_lon)
        return (row, col)

    def generate_roadGraph(self):
        '''Translate roadnetwork data into networkX format, return a networkX graph'''
        DG = nx.DiGraph()
        for key in self.inter_info.keys():
            DG.add_node(key, **self.inter_info[key])  # node_id
        for key in self.road_info.keys():
            pair = (self.road_info[key]['ininter_id'], self.road_info[key]['outinter_id'])
            road_length = self.road_info[key]['roadlen']
            DG.add_edge(*pair, **{"id": key, "length": road_length, "speed": self.road_info[key]['speed'], 'numveh': 0, 'weight': road_length})

        # gpickle_file = self.roadnet_path.replace('.txt', '.gpickle')
        # nx.write_gpickle(DG, gpickle_file)
        print('Building road networkX graph successfully!')
        self.roadgraph = DG

    def set_inf_edge(self, node_pair):

        self.roadgraph.edges[node_pair]['weight'] = float('inf')
        e_id = self.roadgraph.edges[node_pair]['id']
        print('set edge-{} to {}!'.format(e_id, self.roadgraph.edges[node_pair]['weight']))
        return e_id

    def read_pkl(self, roadnet_path):
        f = open(roadnet_path,'rb')
        roadnet_dict = pickle.load(f)
        
        min_lat, max_lat = 1000, -1000
        min_lon, max_lon = 1000, -1000
        
        self.inter_num = len(roadnet_dict['inter'])
        self.inter_info = {}
        print("Total number of intersections:{}".format(self.inter_num))
        for k, v in roadnet_dict['inter'].items():
            self.inter_info[k] = {'lat': v['lat'], 'lon': v['lon'], 
                                  'sign': v['sign'], 'roadlen': 0.0, 
                                  'roadseg':len(v['start_roads'])+len(v['end_roads'])}
            if v['lat'] < min_lat:
                min_lat = v['lat']
            if v['lat'] > max_lat:
                max_lat = v['lat']
            if v['lon'] < min_lon:
                min_lon = v['lon']
            if v['lon'] > max_lon:
                max_lon = v['lon']
            
        self.road_num = len(roadnet_dict['inter'])
        self.road_info = {}
        print("Total number of road segments:{}".format(self.road_num))
        for k, v in roadnet_dict['road'].items():
            self.road_info[k] = {'ininter_id': v['start_inter'], 'outinter_id': v['end_inter'],
                                        'roadlen': v['roadlen'], 'speed': 16.67}
            self.inter_info[v['start_inter']]['roadlen'] += v['roadlen']
            self.inter_info[v['end_inter']]['roadlen'] += v['roadlen']
            
        # update bbox    
        self.left_lon = min_lon - 1e-6
        self.right_lon = max_lon + 1e-6
        self.bottom_lat = min_lat - 1e-6
        self.top_lat = max_lat + 1e-6
        print(self.left_lon, self.right_lon, self.bottom_lat, self.top_lat)
        

    def read_roadnet(self, roadnet_path):
        '''Read road network data'''
        roadnet = open(roadnet_path, 'r')

        # read inters
        self.inter_num = int(roadnet.readline())
        self.inter_info = {}
        print("Total number of intersections:{}".format(self.inter_num))
        for _ in range(self.inter_num):
            line = roadnet.readline()
            lat, lon, inter_id, sign = self.read_inter_line(line)
            self.inter_info[inter_id] = {'lat': lat, 'lon': lon, 'sign': sign, 'roadlen': 0.0, 'roadseg':0}

        # read roads
        self.road_info = {}
        self.road_num = int(roadnet.readline())
        print("Total number of road segments:{}".format(self.road_num))
        for _ in range(self.road_num):
            line = roadnet.readline()
            inter_id1, inter_id2, roadlen, speed, road_id1, road_id2 = self.read_road_line(line)
            #print(road_id1, road_id2)
            self.road_info[road_id1] = {'ininter_id': inter_id1, 'outinter_id': inter_id2,
                                        'roadlen': roadlen, 'speed': speed}
            self.road_info[road_id2] = {'ininter_id': inter_id2, 'outinter_id': inter_id1,
                                        'roadlen': roadlen, 'speed': speed}
            self.inter_info[inter_id1]['roadlen'] += roadlen
            self.inter_info[inter_id2]['roadlen'] += roadlen
            self.inter_info[inter_id1]['roadseg'] += 1
            self.inter_info[inter_id2]['roadseg'] += 1
            roadnet.readline()
            roadnet.readline()
        roadnet.close()

    def read_inter_line(self, line):
        '''Read intersection data line-by-line'''
        line = line.split()
        lat = float(line[0])
        lon = float(line[1])
        inter_id = int(line[2])
        signal = bool(line[3])
        return lat, lon, inter_id, signal

    def read_road_line(self, line):
        '''Read road segment data line-by-line'''
        line = line.split()
        inter_id1 = int(line[0])
        inter_id2 = int(line[1])
        roadlen = float(line[2])
        speed = float(line[3])

        road_id1 = int(line[6])
        road_id2 = int(line[7])
        return inter_id1, inter_id2, roadlen, speed, road_id1, road_id2

    def random_weight_choose(self, weight_data):
        '''
        Helper function for choosing the Destination zone of a vehicle trip, given weight_data = self.ODprob[o_zone]
        return the destination traffic zone, i.e., d_zone - (rowIndex, columnIndex)
        '''
        total = sum(weight_data.values())   
        ran = random.uniform(0, total)  
        curr_sum = 0
        d_zone = None

        for key in weight_data.keys():
            curr_sum += weight_data[key]  
            if ran <= curr_sum: 
                d_zone = key
                break
        return d_zone
