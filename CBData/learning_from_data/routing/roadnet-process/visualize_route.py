import pandas as pd
from pandas import DataFrame
from pandas import Series
import numpy as np
import osmnx as ox
import networkx as nx
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.colors import LinearSegmentedColormap
from sklearn.cluster import KMeans
import pickle

# datas = ["Origin", "Untrained", "Generated"]
datas = ["Generated"]

roadnet_df = pd.read_csv("./road-coord-temp.txt", header=None, index_col=False, sep='\t')
roadnet_list = roadnet_df.values.tolist()
dir_id_to_coord = dict(zip([int(a[4]) for a in roadnet_list], [a[:4] for a in roadnet_list]))

fig_list = []

for data in datas:
    trajs = []
    with open("./Single-OD-{}.pkl".format(data), "rb") as f:
        trajs = pickle.load(f)

    trajs = [traj[1 : -1] for traj in trajs]
    trajs = trajs[0 : 118]

    routes = []

    wrong_routes = 0
    is_wrong_routes = False
    iter = 0

    print("There are {} trajs".format(len(trajs)))

    tracks = []
    max_coord = [-90, -180]
    min_coord = [90, 180]
    for traj in trajs:
        track_one = []
        if len(traj) == 0:
            break
        for dir in traj:
            track_one.append([dir_id_to_coord[dir][0], dir_id_to_coord[dir][1]])
        track_one.append([dir_id_to_coord[traj[-1]][2], dir_id_to_coord[traj[-1]][3]])
        max_coord = np.max([max_coord, np.max(track_one, axis=0)], axis=0)
        min_coord = np.min([min_coord, np.min(track_one, axis=0)], axis=0)
        tracks.append(track_one)

    place = ox.graph_from_bbox(min_coord[0], max_coord[0], min_coord[1], max_coord[1], network_type='drive')

    for track in tracks:
        is_wrong_routes = False

        lat = [coord[0] for coord in track]
        lng = [coord[1] for coord in track]

        nn = ox.nearest_nodes(place, lng, lat)
        route = []
        for i in range(1, len(nn)):
            route_small = ox.shortest_path(place, nn[i - 1], nn[i], weight='length')
            if route_small is not None and len(route_small) > 1:
                if len(route) == 0:
                    route = route_small
                else:
                    if route[-1] != route_small[0]:
                        is_wrong_routes = True
                        if len(route) > 0:
                            routes.append(route)
                        route = route_small
                        # break
                    else:
                        route.pop()
                        route = route + route_small
        if is_wrong_routes or len(route) == 0:
            wrong_routes = wrong_routes + 1
        else:
            routes.append(route)
        iter = iter + 1
        print("iter: {}, wrong routes: {}".format(iter, wrong_routes))

    ox.plot_graph_routes(place, routes)