import pandas as pd
import pickle
import os
import csv

road_df = pd.read_csv("../roadnet-match/output/road/road-temp.txt", sep='\t', header=None, index_col=False)
road_list = road_df.values.tolist()
road_dict = {}
for road_item in road_list:
    if road_item[0] in road_dict:
        road_dict[road_item[0]][road_item[1]]= road_item[2]
    else:
        road_dict[road_item[0]] = {road_item[1] : road_item[2]}

OD_dict = {}
iter = 0
for filename in os.listdir("../roadnet-match/output/vehicle"):
    path_df= pd.read_csv("../roadnet-match/output/vehicle/" + filename, sep='\t')
    path_list = path_df.values.tolist()
    path = []

    for road_info in path_list:
        if len(path) == 0:
            if road_info[3] != -1:
                path.append([road_info[1], road_info[2]])
            else:
                continue
        else:
            if path[-1][0] == road_info[1] and path[-1][1] == road_info[2]:           # if in the same road
                continue
            elif path[-1][1] == road_info[1] and road_info[3] != -1 and path[-1][0] != road_info[2]:                  # if can be linked
                path.append([road_info[1], road_info[2]])
            else:
                road_path = []
                for road in path:
                    road_path.append(road_dict[road[0]][road[1]])
                if len(road_path) > 0:
                    origin = road_path[0]
                    dest = road_path[-1]
                    if origin in OD_dict:
                        if dest in OD_dict[origin]:
                            OD_dict[origin][dest].append(road_path)
                        else:
                            OD_dict[origin][dest] = [road_path]
                    else:
                        OD_dict[origin] = {dest : [road_path]}
                path.clear()
                if road_info[3] != -1:
                    path.append([road_info[1], road_info[2]])

    road_path = []
    for road in path:
        road_path.append(road_dict[road[0]][road[1]])
    if len(road_path) > 0:
        origin = road_path[0]
        dest = road_path[-1]
        if origin in OD_dict:
            if dest in OD_dict[origin]:
                OD_dict[origin][dest].append(road_path)
            else:
                OD_dict[origin][dest] = [road_path]
        else:
            OD_dict[origin] = {dest : [road_path]}
    path.clear()
    iter = iter  + 1
    if iter % 1000 == 0:
        print(iter)
    if iter >= 10000:
        break

max_origin = 0
max_dest = 0
max_OD = 0 
max_different_traj = 0
for origin in OD_dict.keys():
    for dest in OD_dict[origin].keys():
        if len(set(str(x) for x in OD_dict[origin][dest])) > max_different_traj:
            max_OD = len(OD_dict[origin][dest])
            max_different_traj = len(set(str(x) for x in OD_dict[origin][dest]))
            max_origin = origin
            max_dest = dest

print("origin: {}".format(max_origin))
print("dest: {}".format(max_dest))
print("traj num: {}".format(max_OD))
print("traj variety: {}".format(max_different_traj))


with open("./output/Binomial.csv", "w") as f:
    curr_num = 1
    oid = 1
    writer = csv.writer(f)
    writer.writerow(["", "oid", "ent", "sectionId"])
    for path in OD_dict[max_origin][max_dest]:
        ent = 1
        for road in path:
            writer.writerow([curr_num, oid, ent, road])
            curr_num += 1
            ent += 1
        oid += 1

with open("./output/Single_OD_traj.pkl", "wb") as f:
    pickle.dump(OD_dict[max_origin][max_dest], f)