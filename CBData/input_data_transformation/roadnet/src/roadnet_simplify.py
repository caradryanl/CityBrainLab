'''
Simplify roadnet

Dependent libraries:
    os,
    pandas,
    osm2gmns (pip install osm2gmns),
    shapely,
    math,
    time,
    numpy

Input: node_path, edge_from_to_path, edge_level_path, output_path, reserved_edge_level_set, has_columns
    node_path: path of node.csv
        node.csv: with columns = ['node_id', 'lat', 'lon']
            'node_id': OSM ID of node
            'lat': latitude of node
            'lon': longitude of node
    
    edge_from_to_path: path of edge_from_to.csv
        edge_from_to.csv: with columns = ['edge_id', 'from_node_id', 'to_node_id']
            'edge_id': OSM ID of edge
            'from_node_id': OSM ID of beginning intersection of edge
            'to_node_id': OSM ID of endding intersection of edge
    
    edge_level_path: path of edge_level.csv
        edge_level.csv: with columns = ['edge_id', 'level']
            !! Note: Each edge in edge_from_to_csv whose 'edge_id' must be in edge_level.csv !!
            'edge_id': OSM ID of edge
            'level': level of edge according to OSM, such as 'motorway', 'trunk'
                (You can learn about details in https://wiki.openstreetmap.org/wiki/Map_features#Roads)

    output_path: the path of output
        e.g. "D:/simplified"
            simplified node.csv and edge.csv will output to it
    
    reserved_edge_level_set: set which contains edge levels will be reserved
    
    has_columns: True or False, represents whether .csv files has column labels or not. If False, make sure .csv files data is in order as above mentioned.
    
Output: simplified node.csv and edge.csv
    !! Note: Every node and edge is renamed !!
'''

import os
import pandas as pd
import osm2gmns as og
from shapely.geometry.linestring import LineString
import math
import numpy as np
import time


class Roadnet_simplify:
    
    def __init__(self, node_path, edge_from_to_path, edge_level_path, output_path, reserved_edge_level_set, has_columns=True):
        self.src_node_df = pd.read_csv(node_path)
        self.src_edge_from_to_df = pd.read_csv(edge_from_to_path)
        self.src_edge_level_df = pd.read_csv(edge_level_path)
        self.output_path = output_path # the directory which will store some files generated during processing and output
        
        if has_columns == False:
            self._add_columns()
        
        # drop duplicates
        self.src_node_df.drop_duplicates(subset='node_id', inplace=True)
        self.src_edge_level_df.drop_duplicates(subset='edge_id', inplace=True)
        
        self.reserved_edge_level_set = reserved_edge_level_set
        
        self._mkdir()
    
        
    def _add_columns(self):
        # add columns
        first_node = self.src_node_df.columns
        first_edge_from_to = self.src_edge_from_to_df.columns
        first_edge_level = self.src_edge_level_df.columns

        self.src_node_df.loc[len(self.src_node_df)] = first_node
        self.src_edge_from_to_df.loc[len(self.src_edge_from_to_df)] = first_edge_from_to
        self.src_edge_level_df.loc[len(self.src_edge_level_df)] = first_edge_level

        self.src_node_df.columns = ['node_id', 'lon', 'lat']
        self.src_edge_from_to_df.columns = ['edge_id', 'from_node_id', 'to_node_id']
        self.src_edge_level_df.columns = ['edge_id', 'level']
        
        
    def _mkdir(self):
        os.makedirs(self.output_path + '/__osm2gmns__cache__/__input__', exist_ok=True)
        os.makedirs(self.output_path + '/__osm2gmns__cache__/__output__', exist_ok=True)
        
        
    def _get_reserved_edges(self):
        # reserve edges whose level is in reserved_edge_set
        reserved_edge_set = set()
        
        for i in range(len(self.src_edge_level_df)):
            edge_id = self.src_edge_level_df.loc[i, 'edge_id']
            edge_level = self.src_edge_level_df.loc[i, 'level']
            if edge_level in self.reserved_edge_level_set:
                reserved_edge_set.add(edge_id)
        
        reserved_node_set = set()
        reserved_edge_rows = []
        
        for i in range(len(self.src_edge_from_to_df)):
            edge_id = self.src_edge_from_to_df.loc[i, 'edge_id']
            fromNode = self.src_edge_from_to_df.loc[i, 'from_node_id']
            toNode = self.src_edge_from_to_df.loc[i, 'to_node_id']
            if edge_id in reserved_edge_set:
                reserved_edge_rows.append(list(self.src_edge_from_to_df.loc[i]))
                reserved_node_set.add(fromNode)
                reserved_node_set.add(toNode)
        self.edge_df = pd.DataFrame(reserved_edge_rows, columns=self.src_edge_from_to_df.columns)
    
        reserved_node_rows = []
        
        for i in range(len(self.src_node_df)):
            node_id = self.src_node_df.loc[i, 'node_id']
            if node_id in reserved_node_set:
                reserved_node_rows.append(list(self.src_node_df.loc[i]))
        self.node_df = pd.DataFrame(reserved_node_rows, columns=self.src_node_df.columns)

        print('reserved nodes: ', len(self.node_df))
        print('reserved edges: ', len(self.edge_df))


    def _calc_road_length(self, lat_1, lon_1, lat_2, lon_2):
        #given latitude and longitude of two nodes, return distance between them
        earth_radius = 6371393 # meter
        PI = math.pi
        meter_per_lon = 2 * PI * earth_radius / 360
        lat_rad = lat_1 * PI / 180
        meter_per_lat = meter_per_lon * math.cos(lat_rad)
        lat_dist = (lat_1 - lat_2) * meter_per_lat
        lon_dist = (lon_1 - lon_2) * meter_per_lon
        len = math.sqrt(lat_dist*lat_dist + lon_dist*lon_dist)
        return len
    

    def _convert_to_osm2gmns_input(self):
        self._convert_to_osm2gmns_node_csv()
        self._convert_to_osm2gmns_edge_csv()
    
    
    def _convert_to_osm2gmns_node_csv(self):
        osm2gmns_node_rows = []
        
        for i in range(len(self.node_df)):
            node = self.node_df.loc[i]
            node_id = np.int64(node['node_id'])
            lat = node['lat']
            lon = node['lon']
            row = []
            
            # node_id
            row.append(node_id)
            
            # osm_node_id
            row.append(node_id)
            
            # x_coord
            row.append(lat)
            
            # y_coord
            row.append(lon)
            
            # osm_highway
            row.append('traffic_signal')
            
            # ctrl_type
            row.append('signal')
            
            osm2gmns_node_rows.append(row)
        
        osm2gmns_node_df = pd.DataFrame(osm2gmns_node_rows,
            columns=['node_id', 'osm_node_id', 'x_coord', 'y_coord', 'osm_highway', 'ctrl_type'])
        osm2gmns_node_df.to_csv(self.output_path + '/__osm2gmns__cache__/__input__/__node__.csv')


    def _convert_to_osm2gmns_edge_csv(self):
        # get osm2gmns_roadnet_dict
        osm2gmns_roadnet_dict = { 'node':{}, 'edge':{} }

        for i in range(len(self.node_df)):
            node = self.node_df.loc[i]
            id = node['node_id']
            lat = node['lat']
            lon = node['lon']
            
            osm2gmns_roadnet_dict['node'][id] = {'lat':lat, 'lon': lon}

        for i in range(len(self.edge_df)):
            edge = self.edge_df.loc[i]
            line_string_id = edge['edge_id']
            fromNode = edge['from_node_id']
            toNode = edge['to_node_id']
            id = i
            self.edge_df.loc[i, 'edge_id'] = id # rename edge
            
            if fromNode not in osm2gmns_roadnet_dict['node'] or toNode not in osm2gmns_roadnet_dict['node']:
                continue
            
            lat_1, lon_1 = osm2gmns_roadnet_dict['node'][fromNode]['lat'], osm2gmns_roadnet_dict['node'][fromNode]['lon']
            lat_2, lon_2 = osm2gmns_roadnet_dict['node'][toNode]['lat'], osm2gmns_roadnet_dict['node'][toNode]['lon']
            length = self._calc_road_length(lat_1, lon_1, lat_2, lon_2)
            
            osm2gmns_roadnet_dict['edge'][id] = {'from_node_id':fromNode, 'to_node_id':toNode, 
                                                 'length': length, 'line_string_id':line_string_id}
        
        # get osm2gmns edge DataFrame
        osm2gmns_edge_rows = []
        
        for i in range(len(self.edge_df)):
            edge = self.edge_df.loc[i]
            edge_id = edge['edge_id']
            fromNode = edge['from_node_id']
            toNode = edge['to_node_id']
            
            row = []
            
            if edge_id not in osm2gmns_roadnet_dict['edge']:
                continue
            
            # link_id
            row.append(i)
            
            # osm_way_id
            row.append(osm2gmns_roadnet_dict['edge'][edge_id]['line_string_id'])
            
            # from_node_id
            row.append(fromNode)
            
            # to_node_id
            row.append(toNode)
            
            # length
            row.append(osm2gmns_roadnet_dict['edge'][edge_id]['length'])
            
            # lanes
            row.append(3)
            
            # free_speed
            row.append(16.67)
            
            # geometry
            lat_1, lon_1 = osm2gmns_roadnet_dict['node'][fromNode]['lat'], osm2gmns_roadnet_dict['node'][fromNode]['lon']
            lat_2, lon_2 = osm2gmns_roadnet_dict['node'][toNode]['lat'], osm2gmns_roadnet_dict['node'][toNode]['lon']
            row.append(LineString([(lat_1, lon_1), (lat_2, lon_2)]))
            
            # allowed_uses
            row.append('auto')
            
            osm2gmns_edge_rows.append(row)
            
        osm2gmns_edge_df = pd.DataFrame(osm2gmns_edge_rows,
            columns=['link_id', 'osm_way_id', 'from_node_id', 'to_node_id', 'length', 'lanes', 'free_speed', 'geometry', 'allowed_uses'])
        osm2gmns_edge_df.to_csv(self.output_path + '/__osm2gmns__cache__/__input__/__link__.csv')
    
        
    def _consolidate_intersections(self):
        # consolidate the complex intersections
        # learn about details in https://osm2gmns.readthedocs.io/en/latest/index.html
        roadnet = og.loadNetFromCSV(node_file=self.output_path + '/__osm2gmns__cache__/__input__/__node__.csv', link_file=self.output_path + '/__osm2gmns__cache__/__input__/__link__.csv')
        og.consolidateComplexIntersections(roadnet, auto_identify=True, int_buffer=20)
        og.outputNetToCSV(roadnet, output_folder=self.output_path + '/__osm2gmns__cache__/__output__')
    
    
    def _format_osm2gmns_csv(self):
        # format node.csv
        self.new_node_df = pd.read_csv(self.output_path + '/__osm2gmns__cache__/__output__/node.csv')
        self.new_node_df.rename(columns = {'x_coord':'lat', 'y_coord':'lon'}, inplace=True)
        self.new_node_df = self.new_node_df[ ['node_id', 'lat', 'lon'] ]

        # format link.csv
        self.new_edge_df = pd.read_csv(self.output_path + '/__osm2gmns__cache__/__output__/link.csv')
        self.new_edge_df.rename(columns = {'link_id':'edge_id'}, inplace=True)
        self.new_edge_df = self.new_edge_df[ ['edge_id', 'from_node_id', 'to_node_id'] ]
    
    
    def _get_node_degree(self):
        self.node_degree_dict = {} # key: node_id, value: degree of the node
        self.node_degree_3plus_set = set() # nodes whose degree greater than or equal to 3
        
        for i in range(len(self.new_edge_df)):
            edge = self.new_edge_df.loc[i]
            fromNode = edge['from_node_id']
            toNode = edge['to_node_id']
                
            self.new_edge_df.loc[i, 'edge_id'] = i # rename edge
            
            if fromNode not in self.node_degree_dict:
                self.node_degree_dict[fromNode] = 1
            else:
                self.node_degree_dict[fromNode] += 1
                if self.node_degree_dict[fromNode] >= 3:
                    self.node_degree_3plus_set.add(fromNode)
            
            if toNode not in self.node_degree_dict:
                self.node_degree_dict[toNode] = 1
            else:
                self.node_degree_dict[toNode] += 1
                if self.node_degree_dict[toNode] >= 3:
                    self.node_degree_3plus_set.add(toNode)
                    
    
    def _get_roadnet_dict(self):
        self.roadnet_dict = { 'node':{}, 'edge':{} }
        
        for i in range(len(self.new_edge_df)):
            edge = self.new_edge_df.loc[i]
            id = edge['edge_id']
            fromNode = edge['from_node_id']
            toNode = edge['to_node_id']
            
            if fromNode not in self.roadnet_dict['node']:
                self.roadnet_dict['node'][fromNode] = {'roads':[id]}
            else:
                self.roadnet_dict['node'][fromNode]['roads'].append(id)
                
            if toNode not in self.roadnet_dict['node']:
                self.roadnet_dict['node'][toNode] = {'roads':[id]}
            else:
                self.roadnet_dict['node'][toNode]['roads'].append(id)
            
            self.roadnet_dict['edge'][id] = {'node_1': fromNode, 'node_2': toNode}
    
    
    def _find_endNode(self, node):
        edge_set = set(self.roadnet_dict['node'][node]['roads'])
        endNode_set = set()
        
        for edge in edge_set:
            if edge in self.vis_edge_set:
                continue
            
            cur_edge = edge
            self.vis_edge_set.add(cur_edge)
            cur_node = node
            
            # find next node
            if self.roadnet_dict['edge'][cur_edge]['node_1'] == cur_node:
                cur_node = self.roadnet_dict['edge'][cur_edge]['node_2']
            else:
                cur_node = self.roadnet_dict['edge'][cur_edge]['node_1']
            
            while self.node_degree_dict[cur_node] == 2: # neither intersection nor ending node
                # find next edge
                road_list = self.roadnet_dict['node'][cur_node]['roads']
                if cur_edge == road_list[0]:
                    cur_edge = road_list[1]
                else:
                    cur_edge = road_list[0]
                self.vis_edge_set.add(cur_edge)
                
                # find next node
                if self.roadnet_dict['edge'][cur_edge]['node_1'] == cur_node:
                    cur_node = self.roadnet_dict['edge'][cur_edge]['node_2']
                else:
                    cur_node = self.roadnet_dict['edge'][cur_edge]['node_1']
                    
            endNode_set.add(cur_node)
        
        return endNode_set
        
    
    def _merge_edges(self):
        self.final_edge_df = pd.DataFrame(columns=self.new_edge_df.columns)
        
        self.vis_edge_set = set()
        self.node_edge_dict = {'fromNode_edge_dict':{}, 'toNode_edge_dict':{}}
        reserved_node_set = set()
        
        new_node_degree_dict = {}
        
        for node in self.node_degree_3plus_set:
            reserved_node_set.add(node)
            endNode_set = self._find_endNode(node)
            for endNode in endNode_set:
                
                if (node in new_node_degree_dict and new_node_degree_dict[node] >= 4) or (endNode in new_node_degree_dict and new_node_degree_dict[endNode] >= 4):
                    continue
                
                idx = len(self.final_edge_df)
                self.final_edge_df.loc[idx, 'edge_id'] = idx
                self.final_edge_df.loc[idx, 'from_node_id'] = node
                self.final_edge_df.loc[idx, 'to_node_id'] = endNode
                
                reserved_node_set.add(endNode)
                
                if node in new_node_degree_dict:
                    new_node_degree_dict[node] += 1
                else:
                    new_node_degree_dict[node] = 1
                    
                if endNode in new_node_degree_dict:
                    new_node_degree_dict[endNode] += 1
                else:
                    new_node_degree_dict[endNode] = 1
                
                
                if node not in self.node_edge_dict['fromNode_edge_dict']:
                    self.node_edge_dict['fromNode_edge_dict'][node] = [idx]
                else:
                    self.node_edge_dict['fromNode_edge_dict'][node].append(idx)
                
                if endNode not in self.node_edge_dict['toNode_edge_dict']:
                    self.node_edge_dict['toNode_edge_dict'][endNode] = [idx]
                else:
                    self.node_edge_dict['toNode_edge_dict'][endNode].append(idx)
                
        self.final_node_df = pd.DataFrame(columns=self.new_node_df.columns)
        
        for i in range(len(self.new_node_df)):
            node = self.new_node_df.loc[i]
            node_id = node['node_id']
            lat = node['lat']
            lon = node['lon']
            
            if node_id in new_node_degree_dict:
                idx = len(self.final_node_df)
                
                if node_id in self.node_edge_dict['fromNode_edge_dict']:
                    for edge_id in self.node_edge_dict['fromNode_edge_dict'][node_id]:
                        self.final_edge_df.loc[edge_id, 'from_node_id'] = idx + 1
                
                if node_id in self.node_edge_dict['toNode_edge_dict']:
                    for edge_id in self.node_edge_dict['toNode_edge_dict'][node_id]:
                        self.final_edge_df.loc[edge_id, 'to_node_id'] = idx + 1
                
                self.final_node_df.loc[idx, 'node_id'] = idx + 1
                self.final_node_df.loc[idx, 'lat'] = lat
                self.final_node_df.loc[idx, 'lon'] = lon

        print("number of new nodes: ", len(self.final_node_df))
        print("number of new edges: ", len(self.final_edge_df))
        
        
    def _output_result(self):
        self.final_node_df.rename(columns={'lat':'latitude', 'lon':'longitude'}, inplace=True)
        
        self.final_node_df.to_csv(self.output_path + '/node.csv')
        self.final_edge_df.to_csv(self.output_path + '/edge.csv')
        
    
    def _rmdir(self):
        os.remove(self.output_path + '/__osm2gmns__cache__/__input__/__node__.csv')
        os.remove(self.output_path + '/__osm2gmns__cache__/__input__/__link__.csv')
        if len(os.listdir(self.output_path + '/__osm2gmns__cache__/__input__')) == 0:
            os.rmdir(self.output_path + '/__osm2gmns__cache__/__input__')
        
        os.remove(self.output_path + '/__osm2gmns__cache__/__output__/node.csv')
        os.remove(self.output_path + '/__osm2gmns__cache__/__output__/link.csv')
        if len(os.listdir(self.output_path + '/__osm2gmns__cache__/__output__')) == 0:
            os.rmdir(self.output_path + '/__osm2gmns__cache__/__output__')
        
        if len(os.listdir(self.output_path + '/__osm2gmns__cache__')) == 0:
            os.rmdir(self.output_path + '/__osm2gmns__cache__')


    def simplify_roadnet(self):
        start_time = time.time()
        print('Simplify roadnet starts.\t', time.strftime('%Y-%m-%d %H:%M:%S', time.localtime()))
        
        print('Getting reserved edges......')
        self._get_reserved_edges()
        print('Done')
        
        print('Converting to OSM2gmns input form......')
        self._convert_to_osm2gmns_input()
        print('Done')
        
        print('Consolidating complex intersections......')
        self._consolidate_intersections()
        print('Done')
        
        print('Formating OSM2gmns files......')
        self._format_osm2gmns_csv()
        print('Done')
        
        print('Getting degree information of nodes......')
        self._get_node_degree()
        print('Done')
        
        print('Getting dictionary of roadnet......')
        self._get_roadnet_dict()
        print('Done')
        
        print('Merging edges......')
        self._merge_edges()
        print('Done')
        
        print('Outputing results......')
        self._output_result()
        print('Done')
        
        self._rmdir()
        
        print('Simplify roadnet finishes.\t', time.strftime('%Y-%m-%d %H:%M:%S', time.localtime()))
        end_time = time.time()
        print('Runtime: ', end_time - start_time)
        
    