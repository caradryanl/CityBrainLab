'''
Transform .csv files to traffic simulator CBEngine/CityFlow/SUMO data format

Dependent libraries:
    pandas,
    math,
    itertools,
    numpy,
    time,
    pickle,
    re,
    json

Input: node_path, edge_path, directed, output_path
    node_path: path of node.csv
        node.csv: with columns ['node_id', 'latitude', 'longitude']
    
    edge_path: path of edge.csv
        edge.csv: with columns ['edge_id', 'from_node_id', 'to_node_id']
        
    directed:
        True or False, represents whether edges are directed(True) or not(False)
        
    output_path: path of output file

Output: a roadnet file(.txt/.json/.xml) with CBEngine/CityFlow/SUMO data format
    CityFlow and SUMO data transform is coming......
'''

import pandas as pd
import numpy as np
import math
import time
import pickle
import json
from itertools import permutations
import re


class Data_transform:
    
    class _angle_edgeT:
    
        def __init__(self, id, angle):
            self.edge_id = id
            self.angle = angle
    
    
    def __init__(self, node_path=None, edge_path=None, directed=False, isCityflow_json=False, json_file=None):
        if isCityflow_json == False:
            self._read_csv_files(node_path, edge_path)
        else:
            self._get_src_from_json(json_file)
        self.isDirected = directed
        if self.isDirected == True:
            self._directed_get_roadnet_dict()
        else:
            self._undirected_to_directed_get_roadnet_dict()
    
    
    def _read_csv_files(self, node_path, edge_path):
        self.src_node_df = pd.read_csv(node_path)
        self.src_edge_df = pd.read_csv(edge_path)
    
    
    def _get_src_from_json(self, json_file):

        def calc_lat_lon(x, y):
            # x, y is croods (meter)
            ori_point = {'lat':30, 'lon':120} # 30 N latitude, 120 E longitude as origin point
            earth_radius = 6371393 # meter
            PI = math.pi
            meter_per_lon = 2 * PI * earth_radius / 360
            lat_rad = ori_point['lat'] * PI / 180
            meter_per_lat = meter_per_lon * math.cos(lat_rad)
            lat = ori_point['lat'] + y / meter_per_lon
            lon = ori_point['lon'] + x / meter_per_lat
            return lat, lon

        with open(json_file, 'r') as f:
            json_dict = json.load(f)
            
            # node
            node_rows = []
            node_cols = ['node_id', 'latitude', 'longitude']
            for inter in json_dict['intersections']:
                node_row = []

                # node_id
                node_row.append(inter['id'])

                # latitude, longitude
                x, y = inter['point']['x'], inter['point']['y']
                lat, lon = calc_lat_lon(x, y)
                node_row.append(lat)
                node_row.append(lon)

                node_rows.append(node_row)

            self.src_node_df = pd.DataFrame(node_rows, columns=node_cols)

            # edge
            edge_rows = []
            edge_cols = ['edge_id', 'from_node_id', 'to_node_id']
            for road in json_dict['roads']:
                edge_row = []

                # edge_id
                edge_row.append(road['id'])

                # from_node_id
                edge_row.append(road['startIntersection'])

                # from_node_id
                edge_row.append(road['endIntersection'])

                edge_rows.append(edge_row)

            self.src_edge_df = pd.DataFrame(edge_rows, columns=edge_cols)
    
    
    # def _json_rename(self):
    #     self.json_rename_dict = {'inter':{}, 'road':{}}
        
    #     for i in range(len(self.src_node_df)):
    #         node_id = self.src_node_df.iloc[i]['node_id']
    #         self.json_rename_dict['inter'][node_id] = i
    #         self.src_node_df.loc[i, 'node_id'] = i
        
    #     for i in range(len(self.src_edge_df)):
    #         edge_id = self.src_edge_df.iloc[i]['edge_id']
    #         self.json_rename_dict['road'][edge_id] = i
    #         self.src_edge_df.loc[i, 'edge_id'] = i
    #         self.src_edge_df.loc[i, 'from_node_id'] = self.json_rename_dict['inter'][self.src_edge_df.loc[i]['from_node_id']]
    #         self.src_edge_df.loc[i, 'to_node_id'] = self.json_rename_dict['inter'][self.src_edge_df.loc[i]['to_node_id']]


    def routes_file_rename(self, input_routes_file, output_routes_file):
        with open(input_routes_file, 'r') as in_f, open(output_routes_file, 'w') as out_f:
            for line in in_f.readlines():
                line_split = line.split('"')
                if len(line_split) == 1:
                    out_f.write(line)
                elif len(line_split) == 3:
                    road_name = line_split[1].split('-')
                    if len(road_name) == 1:
                        new_line = '{pre}"{new_name}"{post}'.format(pre=line_split[0],
                                                                    new_name=self.json_rename_dict['road'][road_name[0]],
                                                                    post=line_split[2])
                        out_f.write(new_line)
                    elif len(road_name) == 2:
                        new_line = '{pre}"{new_name_1}-{new_name_2}"{post}'.format(pre=line_split[0],
                                                                                   new_name_1=self.json_rename_dict['road'][road_name[0]],
                                                                                   new_name_2=self.json_rename_dict['road'][road_name[1]],
                                                                                   post = line_split[2])
                        out_f.write(new_line)
                    else:
                        raise ValueError("routes file format is wrong")
                else:
                    raise ValueError("routes file format is wrong")

            
    def _undirected_to_directed_get_roadnet_dict(self):
        # get roadnet_dict from node DataFrame and edge DataFrame

        
        self.roadnet_dict = {'inter':{}, 'road':{}}
        
        for i in range(len(self.src_node_df)):
            node = self.src_node_df.loc[i]
            node_id = int(node['node_id'])
            lat = node['latitude']
            lon = node['longitude']
            self.roadnet_dict['inter'][node_id] = {'lat':lat, 'lon':lon, 'sign':1,
                                                   'start_roads':[], 'end_roads':[],
                                                   'dir_of_end_roads':{}}
        
        self.angle_of_edge_dict = {}
        
        for i in range(len(self.src_edge_df)):
            edge = self.src_edge_df.loc[i]
            edge_id = i + 1 # rename edge_id
            fromNode = int(edge['from_node_id'])
            toNode = int(edge['to_node_id'])
            
            from_node_lat, from_node_lon = self.roadnet_dict['inter'][fromNode]['lat'], self.roadnet_dict['inter'][fromNode]['lon']
            to_node_lat, to_node_lon = self.roadnet_dict['inter'][toNode]['lat'], self.roadnet_dict['inter'][toNode]['lon']
            
            # get edge angle
            edge_angle = self._get_angle(from_node_lat, from_node_lon, to_node_lat, to_node_lon)
            e_1 = self._angle_edgeT(2*edge_id, edge_angle[1])
            e_2 = self._angle_edgeT(2*edge_id + 1, edge_angle[0])
            if fromNode not in self.angle_of_edge_dict:
                self.angle_of_edge_dict[fromNode] = [ e_2 ]
            else:
                self.angle_of_edge_dict[fromNode].append(e_2)
                
            if toNode not in self.angle_of_edge_dict:
                self.angle_of_edge_dict[toNode] = [ e_1 ]
            else:
                self.angle_of_edge_dict[toNode].append(e_1)
        
        # determine directions of edges of a node
        for inter in self.angle_of_edge_dict:
            angle_edge_list = self.angle_of_edge_dict[inter]
            
            [north_road, east_road, south_road, west_road] = self._determine_dir(angle_edge_list)
                
            self.roadnet_dict['inter'][inter]['dir_of_end_roads'] = {'north':int(north_road), 'east':int(east_road), 'south':int(south_road), 'west':int(west_road)}
        
        # get road
        for i in range(len(self.src_edge_df)):
            edge = self.src_edge_df.loc[i]
            edge_id = i + 1
            fromNode = int(edge['from_node_id'])
            toNode = int(edge['to_node_id'])
            
            if (2*edge_id) not in self.roadnet_dict['inter'][toNode]['dir_of_end_roads'].values() or (2*edge_id + 1)not in self.roadnet_dict['inter'][fromNode]['dir_of_end_roads'].values():
                for direction in self.roadnet_dict['inter'][toNode]['dir_of_end_roads']:
                    if self.roadnet_dict['inter'][toNode]['dir_of_end_roads'][direction] == (2*edge_id):
                        self.roadnet_dict['inter'][toNode]['dir_of_end_roads'][direction] = -1
                        
                for direction in self.roadnet_dict['inter'][fromNode]['dir_of_end_roads']:
                    if self.roadnet_dict['inter'][fromNode]['dir_of_end_roads'][direction] == (2*edge_id + 1):
                        self.roadnet_dict['inter'][fromNode]['dir_of_end_roads'][direction] = -1
                        
                continue
            
            self.roadnet_dict['inter'][fromNode]['start_roads'].append(2*edge_id)
            self.roadnet_dict['inter'][toNode]['end_roads'].append(2*edge_id)
            self.roadnet_dict['inter'][toNode]['start_roads'].append(2*edge_id + 1)
            self.roadnet_dict['inter'][fromNode]['end_roads'].append(2*edge_id + 1)
            
            from_node_lat, from_node_lon = self.roadnet_dict['inter'][fromNode]['lat'], self.roadnet_dict['inter'][fromNode]['lon']
            to_node_lat, to_node_lon = self.roadnet_dict['inter'][toNode]['lat'], self.roadnet_dict['inter'][toNode]['lon']
            length = self._calc_road_length(from_node_lat, from_node_lon, to_node_lat, to_node_lon)
            
            self.roadnet_dict['road'][2*edge_id] = {'start_inter':fromNode, 'end_inter':toNode, 'roadlen':length, 'lane_num':3, 'max_speed':16.67}
            self.roadnet_dict['road'][2*edge_id + 1] = {'start_inter':toNode, 'end_inter':fromNode, 'roadlen':length, 'lane_num':3, 'max_speed':16.67}

        # drop nodes which do not connected to any edges
        for inter in self.roadnet_dict['inter']:
            if len(self.roadnet_dict['inter'][inter]['start_roads']) == 0 and len(self.roadnet_dict['inter'][inter]['end_roads']) == 0:
                del self.roadnet_dict['inter'][inter]
    
    
    def _directed_get_roadnet_dict(self):
        # rename nodes and edges so that they are integers
        # get roadnet_dict from node DataFrame and edge DataFrame
        
        self.roadnet_dict = {'inter':{}, 'road':{}}
        self.json_rename_dict = {'inter':{}, 'road':{}}

        self.rename_node_df = self.src_node_df

        for i in range(len(self.rename_node_df)):
            node = self.rename_node_df.loc[i]
            # rename nodes
            self.json_rename_dict['inter'][node['node_id']] = i + 1
            self.rename_node_df.loc[i, 'node_id'] = i + 1

            lat = node['latitude']
            lon = node['longitude']
            self.roadnet_dict['inter'][i+1] = {'lat':lat, 'lon':lon, 'sign':1,
                                                   'start_roads':[], 'end_roads':[],
                                                   'dir_of_end_roads':{}, 'dir_of_start_roads':{}}
        
        # rename road such that opposite roads have adjacent IDs
        self.rename_edge_df = self.src_edge_df
        adj_dict = {}
        
        for i in range(len(self.rename_edge_df)):
            edge = self.rename_edge_df.loc[i]
            fromNode = edge['from_node_id']
            toNode = edge['to_node_id']
            
            if toNode in adj_dict and fromNode in adj_dict[toNode] and len(adj_dict[toNode][fromNode]) > 0:
                # rename edges
                self.json_rename_dict['road'][edge['edge_id']] = adj_dict[toNode][fromNode][0] + 1
                self.rename_edge_df.loc[i, 'edge_id'] = adj_dict[toNode][fromNode][0] + 1
                self.rename_edge_df.loc[i, 'from_node_id'] = self.json_rename_dict['inter'][fromNode]
                self.rename_edge_df.loc[i, 'to_node_id'] = self.json_rename_dict['inter'][toNode]
        
                del adj_dict[toNode][fromNode][0]
                continue
            
            if fromNode not in adj_dict:
                adj_dict[fromNode] = {}
                adj_dict[fromNode][toNode] = [2 * (i+1)]
            elif toNode not in adj_dict[fromNode]:
                adj_dict[fromNode][toNode] = [2 * (i+1)]
            else:
                adj_dict[fromNode][toNode].append(2 * (i+1))
            # rename edges
            self.json_rename_dict['road'][edge['edge_id']] = 2 * (i+1)
            self.rename_edge_df.loc[i, 'edge_id'] = 2 * (i+1)
            self.rename_edge_df.loc[i, 'from_node_id'] = self.json_rename_dict['inter'][fromNode]
            self.rename_edge_df.loc[i, 'to_node_id'] = self.json_rename_dict['inter'][toNode]
        
        # gets angles of all end roads corresponding to its intersection
        self.angle_of_end_roads_dict = {}
        self.angle_of_start_roads_dict = {}
        
        for i in range(len(self.rename_edge_df)):
            edge = self.rename_edge_df.loc[i]
            edge_id = int(edge['edge_id'])
            fromNode = int(edge['from_node_id'])
            toNode = int(edge['to_node_id'])
            
            from_node_lat, from_node_lon = self.roadnet_dict['inter'][fromNode]['lat'], self.roadnet_dict['inter'][fromNode]['lon']
            to_node_lat, to_node_lon = self.roadnet_dict['inter'][toNode]['lat'], self.roadnet_dict['inter'][toNode]['lon']
            
            # get edge angle
            edge_angle = self._get_angle(from_node_lat, from_node_lon, to_node_lat, to_node_lon)
            e_1 = self._angle_edgeT(edge_id, edge_angle[1])
            if toNode not in self.angle_of_end_roads_dict:
                self.angle_of_end_roads_dict[toNode] = [ e_1 ]
            else:
                self.angle_of_end_roads_dict[toNode].append(e_1)

            e_2 = self._angle_edgeT(edge_id, edge_angle[0])
            if fromNode not in self.angle_of_start_roads_dict:
                self.angle_of_start_roads_dict[fromNode] = [ e_2 ]
            else:
                self.angle_of_start_roads_dict[fromNode].append(e_2)
        
        # determine directions of edges of a node
        for inter in self.angle_of_end_roads_dict:
            angle_end_road_list = self.angle_of_end_roads_dict[inter]
            [north_road, east_road, south_road, west_road] = self._determine_dir(angle_end_road_list)
            self.roadnet_dict['inter'][inter]['dir_of_end_roads'] = {'north':int(north_road), 'east':int(east_road), 'south':int(south_road), 'west':int(west_road)}

        for inter in self.angle_of_start_roads_dict:
            angle_start_road_list = self.angle_of_start_roads_dict[inter]
            [north_road, east_road, south_road, west_road] = self._determine_dir(angle_start_road_list)
            self.roadnet_dict['inter'][inter]['dir_of_start_roads'] = {'north':int(north_road), 'east':int(east_road), 'south':int(south_road), 'west':int(west_road)}
            
        # get road
        for i in range(len(self.rename_edge_df)):
            edge = self.rename_edge_df.loc[i]
            edge_id = int(edge['edge_id'])
            fromNode = int(edge['from_node_id'])
            toNode = int(edge['to_node_id'])
            
            # if edge_id not in self.roadnet_dict['inter'][toNode]['dir_of_end_roads'].values() or edge_id not in self.roadnet_dict['inter'][fromNode]['dir_of_start_roads'].values():
            if edge_id not in self.roadnet_dict['inter'][fromNode]['dir_of_start_roads'].values():
                for direction in self.roadnet_dict['inter'][toNode]['dir_of_end_roads']:
                    if self.roadnet_dict['inter'][toNode]['dir_of_end_roads'][direction] == edge_id:
                        self.roadnet_dict['inter'][toNode]['dir_of_end_roads'][direction] = -1
                        
                for direction in self.roadnet_dict['inter'][fromNode]['dir_of_start_roads']:
                    if self.roadnet_dict['inter'][fromNode]['dir_of_start_roads'][direction] == edge_id:
                        self.roadnet_dict['inter'][fromNode]['dir_of_start_roads'][direction] = -1
                        
                continue
            
            self.roadnet_dict['inter'][fromNode]['start_roads'].append(edge_id)
            self.roadnet_dict['inter'][toNode]['end_roads'].append(edge_id)
            
            from_node_lat, from_node_lon = self.roadnet_dict['inter'][fromNode]['lat'], self.roadnet_dict['inter'][fromNode]['lon']
            to_node_lat, to_node_lon = self.roadnet_dict['inter'][toNode]['lat'], self.roadnet_dict['inter'][toNode]['lon']
            length = self._calc_road_length(from_node_lat, from_node_lon, to_node_lat, to_node_lon)
            
            self.roadnet_dict['road'][edge_id] = {'start_inter':fromNode, 'end_inter':toNode, 'roadlen':length, 'lane_num':3, 'max_speed':16.67}

        # drop nodes which do not connected to any edges
        for inter in self.roadnet_dict['inter']:
            if len(self.roadnet_dict['inter'][inter]['start_roads']) == 0 and len(self.roadnet_dict['inter'][inter]['end_roads']) == 0:
                del self.roadnet_dict['inter'][inter]
                continue
            
            if len(self.roadnet_dict['inter'][inter]['dir_of_end_roads']) == 0:
                self.roadnet_dict['inter'][inter]['dir_of_end_roads'] = {'north':-1, 'east':-1, 'south':-1, 'west':-1}
            
            if len(self.roadnet_dict['inter'][inter]['dir_of_start_roads']) == 0:
                self.roadnet_dict['inter'][inter]['dir_of_start_roads'] = {'north':-1, 'east':-1, 'south':-1, 'west':-1}
    
    
    def _get_edge_num(self):
        max_edge_id = -1
        vis_edge_set = set()
        edge_num = 0
        for road_id in self.roadnet_dict['road']:
            if road_id > max_edge_id:
                max_edge_id = road_id
                
            if road_id in vis_edge_set:
                continue
            
            edge_num += 1
            vis_edge_set.add(road_id)
            vis_edge_set.add(self._get_reversed_road(road_id))
            
        return edge_num, max_edge_id
            
    
    def _get_angle(self, lat_1, lon_1, lat_2, lon_2):
        # given (lat_1, lon_1) of from_node and (lat_2, lon_2) of to_node,
        # return the angle_1(rad in [0, 2*PI)) from from_node to to_node and the angle_2 from to_node to from_node
        pi = math.pi
        if lat_1 == lat_2:
            if lon_1 < lon_2:
                return [0.5 * pi, 1.5 * pi]
            else:
                return [1.5 * pi, 0.5 * pi]
        
        if lon_1 == lon_2:
            if lat_1 < lat_2:
                return [0, pi]
            else:
                return [pi, 0]
        
        angle_1 = math.atan( (lon_1 - lon_2) / (lat_1 - lat_2) )
        if angle_1 < 0:
            angle_1 = angle_1 + pi
        if lon_2 > lon_1:
            angle_2 = angle_1 + pi
            return [angle_1, angle_2]
        else:
            angle_2 = angle_1
            angle_1 = angle_1 + pi
            return [angle_1, angle_2]
    
    
    def _get_loss(self, angle_edge, dir_index):
        dir_list = ['north', 'east', 'south', 'west']
        direction = dir_list[dir_index]
        angle = angle_edge.angle
        
        pi = math.pi
        
        if direction == 'north':
            loss = abs(angle - 0.5*pi)
            if loss > pi:
                loss = 2*pi - loss
        
        elif direction == 'east':
            loss = abs(angle - 0)
            if loss > pi:
                loss = 2*pi - loss
        
        elif direction == 'south':
            loss = abs(angle - 1.5*pi)
            if loss > pi:
                loss = 2*pi - loss
        
        elif direction == 'west':
            loss = abs(angle - pi)
            if loss > pi:
                loss = 2*pi - loss
                
        return loss


    def _get_loss_sum(self, angle_edge_list, dirIndex_tuple):
        loss_sum = 0
        for i in range(len(dirIndex_tuple)):
            loss_sum = loss_sum + self._get_loss(angle_edge_list[i], dirIndex_tuple[i])
            
        return loss_sum


    def _determine_dir(self, angle_edge_list):
        dir_list = ['north', 'east', 'south', 'west']
        edge_num = len(angle_edge_list)
        if edge_num > 4: # no more than 4 approaches
            edge_num = 4
        dir_index = [0, 1, 2, 3]
        dirIndex_list = list(permutations(dir_index, edge_num))
        
        min_loss = 4*math.pi
        min_loss_tuple = ()
        for i in range(len(dirIndex_list)):
            loss = self._get_loss_sum(angle_edge_list, dirIndex_list[i])
            if loss < min_loss:
                min_loss = loss
                min_loss_tuple = dirIndex_list[i]
                
        dir_dict = {}
        for i in range(edge_num):
            dir_dict[dir_list[min_loss_tuple[i]]] = angle_edge_list[i].edge_id
                
        dir_list = []
        
        if 'north' in dir_dict:
            dir_list.append(dir_dict['north'])
        else:
            dir_list.append(-1)
            
        if 'east' in dir_dict:
            dir_list.append(dir_dict['east'])
        else:
            dir_list.append(-1)
            
        if 'south' in dir_dict:
            dir_list.append(dir_dict['south'])
        else:
            dir_list.append(-1)
            
        if 'west' in dir_dict:
            dir_list.append(dir_dict['west'])
        else:
            dir_list.append(-1)
            
        return dir_list


    def _get_reversed_road(self, road):
        road = int(road)
        if road == -1:
            return -1
        if road % 2 == 0:
            return road + 1
        else:
            return road - 1
    
    
    def _calc_road_length(self, lat_1, lon_1, lat_2, lon_2):
        #given latitude and longitude of two nodes, return distance between them
        earth_radius = 6371393 # meter
        PI = math.pi
        meter_per_lon = 2 * PI * earth_radius / 360
        lat_rad = lat_1 * PI / 180
        meter_per_lat = meter_per_lon * math.cos(lat_rad)
        lat_dist = (lat_1 - lat_2) * meter_per_lat
        lon_dist = (lon_1 - lon_2) * meter_per_lon
        length = math.sqrt(lat_dist*lat_dist + lon_dist*lon_dist)
        return length
    
    
    def Output_roadnet_dict(self, output_path):
        # output roadnet.pkl
        with open(output_path + '/roadnet_dict.pkl', 'wb') as pkl_f:
            pickle.dump(self.roadnet_dict, pkl_f)
    
    
    def CBEngine_data_transform(self, output_path):
        # transform data to CBEngine format using roadnet_dict
        print('Transform data to CBEngine format......')
        with open(output_path + '/raw_roadnet.txt', 'w') as f:
            self._CBEngine_write_inter_data(f)
            if self.isDirected == True:
                self._directed_CBEngine_write_road_data(f)
                self._directed_CBEngine_write_signal_data(f)
            else:
                self._undirected_CBEngine_write_road_data(f)
                self._undirected_CBEngine_write_signal_data(f)
        
        # # if is Cityflow_json:        
        # with open(output_path + '/mapping.json', 'w') as f:
        #     f.write(json.dumps(self.json_rename_dict))

        print('Done')
        print("CBEngine intersection number: ", len(self.roadnet_dict['inter']))
        print("CBEngine road number: ", len(self.roadnet_dict['road']))

    
    def _CBEngine_write_inter_data(self, f):
        # write intersection dataset
        inter_num = len(self.roadnet_dict['inter'])
        f.write('{}\n'.format(inter_num))
        
        inter_data_form = '{latitude} {longitude} {inter_id} {signalized}\n'
        for inter_id in self.roadnet_dict['inter']:
            inter_data = self.roadnet_dict['inter'][inter_id]
            inter_data_str = inter_data_form.format(latitude = inter_data['lat'],
                                                    longitude = inter_data['lon'],
                                                    inter_id = inter_id,
                                                    signalized = inter_data['sign'])
            f.write(inter_data_str)
    
    
    def _undirected_CBEngine_write_road_data(self, f):
        # write road dataset
        edge_num, max_edge_id = self._get_edge_num()
        f.write('{}\n'.format(edge_num))
        
        road_data_form = '{from_inter_id} {to_inter_id} {length} {speed_limit} {dir1_num_lane} {dir2_num_lane} {dir1_id} {dir2_id}\n{dir1_mov}\n{dir2_mov}\n'
        for i in range(int(max_edge_id / 2) + 1):
            
            if (2 * i) not in self.roadnet_dict['road'] or (2*i + 1) not in self.roadnet_dict['road']:
                continue
            
            road_data = self.roadnet_dict['road'][2 * i]
            reversed_road_data = self.roadnet_dict['road'][2*i + 1]
            road_data_str = road_data_form.format(from_inter_id = road_data['start_inter'],
                                                  to_inter_id = road_data['end_inter'],
                                                  length = road_data['roadlen'],
                                                  speed_limit = road_data['max_speed'],
                                                  dir1_num_lane = road_data['lane_num'],
                                                  dir2_num_lane = reversed_road_data['lane_num'],
                                                  dir1_id = 2 * i,     
                                                  dir2_id = 2*i + 1,
                                                  dir1_mov = '1 0 0 0 1 0 0 0 1',
                                                  dir2_mov = '1 0 0 0 1 0 0 0 1')
            f.write(road_data_str)
    
    
    def _directed_CBEngine_write_road_data(self, f):
        # write road dataset
        edge_num, max_edge_id = self._get_edge_num()
        f.write('{}\n'.format(edge_num))
        
        road_data_form = '{from_inter_id} {to_inter_id} {length} {speed_limit} {dir1_num_lane} {dir2_num_lane} {dir1_id} {dir2_id}\n{dir1_mov}\n{dir2_mov}\n'
        for i in range(int(max_edge_id / 2) + 1):
            
            if (2 * i) not in self.roadnet_dict['road'] and (2*i + 1) not in self.roadnet_dict['road']:
                continue
            
            if (2 * i) in self.roadnet_dict['road']:
                road_data = self.roadnet_dict['road'][2 * i]
                road_data_str = road_data_form.format(from_inter_id = road_data['start_inter'],
                                                      to_inter_id = road_data['end_inter'],
                                                      length = road_data['roadlen'],
                                                      speed_limit = road_data['max_speed'],
                                                      dir1_num_lane = road_data['lane_num'],
                                                      dir2_num_lane = road_data['lane_num'],
                                                      dir1_id = 2 * i,
                                                      dir2_id = ((2*i + 1) if (2*i + 1) in self.roadnet_dict['road'] else -1),
                                                      dir1_mov = '1 0 0 0 1 0 0 0 1',
                                                      dir2_mov = '1 0 0 0 1 0 0 0 1')
                f.write(road_data_str)
            else:
                road_data = self.roadnet_dict['road'][2*i + 1]
                road_data_str = road_data_form.format(from_inter_id = road_data['start_inter'],
                                                      to_inter_id = road_data['end_inter'],
                                                      length = road_data['roadlen'],
                                                      speed_limit = road_data['max_speed'],
                                                      dir1_num_lane = road_data['lane_num'],
                                                      dir2_num_lane = road_data['lane_num'],
                                                      dir1_id = -1,
                                                      dir2_id = 2*i + 1,
                                                      dir1_mov = '1 0 0 0 1 0 0 0 1',
                                                      dir2_mov = '1 0 0 0 1 0 0 0 1')
                f.write(road_data_str)
                
    
    def _undirected_CBEngine_write_signal_data(self, f):
        # write traffic signal dataset
        inter_num = len(self.roadnet_dict['inter'])
        f.write('{}\n'.format(inter_num))
        
        TS_data_form = '{inter_id} {n_road} {e_road} {s_road} {w_road}\n'
        for inter_id in self.roadnet_dict['inter']:
            dir_of_roads = self.roadnet_dict['inter'][inter_id]['dir_of_end_roads']
            north_road = self._get_reversed_road(dir_of_roads['north'])
            east_road = self._get_reversed_road(dir_of_roads['east'])
            south_road = self._get_reversed_road(dir_of_roads['south'])
            west_road = self._get_reversed_road(dir_of_roads['west'])
            TS_data_str = TS_data_form.format(inter_id = inter_id,
                                              n_road = north_road,
                                              e_road = east_road,
                                              s_road = south_road,
                                              w_road = west_road)
            f.write(TS_data_str)
    
    
    def _directed_CBEngine_write_signal_data(self, f):
        # write traffic signal dataset
        inter_num = len(self.roadnet_dict['inter'])
        f.write('{}\n'.format(inter_num))
        
        TS_data_form = '{inter_id} {n_road} {e_road} {s_road} {w_road}\n'
        for inter_id in self.roadnet_dict['inter']:
            dir_of_roads = self.roadnet_dict['inter'][inter_id]['dir_of_start_roads']
            north_road = dir_of_roads['north']
            east_road = dir_of_roads['east']
            south_road = dir_of_roads['south']
            west_road = dir_of_roads['west']
            TS_data_str = TS_data_form.format(inter_id = inter_id,
                                              n_road = north_road,
                                              e_road = east_road,
                                              s_road = south_road,
                                              w_road = west_road)
            f.write(TS_data_str)
        
        
    def _CityFlow_data_transform(self, output_path):
        # TODO: transform data to Cityflow format using roadnet_dict
        pass
    
    
    def _SUMO_data_transform(self, output_path):
        # TODO: transform data to SUMO format using roadnet_dict
        pass

    def CBEngine_flow_transform(self, input_path, mapping_path, output_path):

        # print("The flow intervals devide 2.5")

        with open(input_path, 'r') as file:
            flows = json.load(file)
        with open(mapping_path, 'r') as file:
            mapping = json.load(file)
        
        file = open(output_path+'/CBEngine_flow.txt', 'w')
        file.write("{}\n".format(len(flows)))
        for flow in flows:
            interval = max(int(flow['interval']), 1) # devide 2.5
            start = int(flow['startTime'])
            end = int(flow['endTime'])
            route = flow['route']
            length = len(route)

            if end == start:
                end += 1

            file.write("{} ".format(start))
            file.write("{} ".format(10800 if (end<start) else end))
            file.write("{}\n".format(interval))
            file.write("{}\n".format(length))
            for id in range(length):
                if (id == length - 1):
                    file.write("{}\n".format(mapping['road'][route[id]]))
                else:
                    file.write("{} ".format(mapping['road'][route[id]]))
        file.close()
        


        
    
    