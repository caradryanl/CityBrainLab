import pickle
import pandas as pd

road_df = pd.read_csv("../roadnet-match/output/road/road-temp.txt", header=None, index_col=False, sep='\t')
road_list = road_df.values.tolist()

adj_dict = {}

for road in road_list:
    if road[0] in adj_dict.keys():
        adj_dict[road[0]].append([road[1], road[2]])
    else:
        adj_dict[road[0]] = [[road[1], road[2]]]

net_in = []
net_out = []
for adj in adj_dict.keys():
    if len(adj_dict[adj]) == 1:
        net_in.append(adj_dict[adj][0][1])
        for inter in adj_dict[adj_dict[adj][0][0]]:
            if inter[0] == adj:
                net_out.append(inter[1])
                break

with open("./output/net-in.pkl", "wb") as f:
    pickle.dump(net_in, f)

with open("./output/net-out.pkl", "wb") as f:
    pickle.dump(net_out, f)