import numpy
import pickle

def process_segmentation():
    with open('./raw_data/vehicle-in.pkl','rb') as file:
        vehs_raw = pickle.load(file)
    with open('./raw_data/avg-speed.pkl','rb') as file:
        speeds_raw = pickle.load(file)
    
    data = {}

    for k, vehs in vehs_raw.items():
        speeds = speeds_raw[k]

        id_seg = 0
        data_seg = {'veh':{}, 'speed':{}}
        id = 0

        while(id < len(vehs)):
            while(id < len(vehs) and vehs[id] < 0):
                id += 1
            tmp_vehs = []
            tmp_speeds = []
            while(id < len(vehs) and vehs[id] >= 0):
                tmp_vehs.append(vehs[id])
                tmp_speeds.append(speeds[id])
                id += 1
            data_seg['veh'][id_seg] = tmp_vehs
            data_seg['speed'][id_seg] = tmp_speeds
            id_seg += 1
        
        data[k] = data_seg
    print(data)
    with open('./data/data_segmented.pkl','wb') as file:
        pickle.dump(data, file)


def generate_traffic_unit_time(route, volume, start_time, end_time, scale=5):
    data = []
    if volume == 0:
        return data

    veh_num = volume * scale
    veh_freq = int((end_time - start_time) / veh_num)
    
    data.append(start_time)
    data.append(end_time)
    data.append(veh_freq)
    data.append(len(route))
    for road in route:
        data.append(road)

    return data

def generate_traffic(data_path, output_path):
    '''
        [22.7316, 113.853, 22.7287, 113.86]: 4192, 4193
    '''

    with open(data_path,'rb') as file:
        data = pickle.load(file)

    vehs = data['[22.7316, 113.853, 22.7287, 113.86]']['veh'][0][120: 180] 
    speeds = data['[22.7316, 113.853, 22.7287, 113.86]']['speed'][0][120: 180]
    flow_data = {}

    for time_stamp, volume in enumerate(vehs):
        start_time = time_stamp * 60
        end_time = (time_stamp+1) * 60
        route = [4192]
        flow = generate_traffic_unit_time(route, volume, start_time, end_time)
        if len(flow) > 0:
            flow_data[time_stamp] = flow
        
    file = open(output_path+'flow.txt', 'w')
    file.write("{}\n".format(len(flow_data)))
    for flow_data in flow_data.values():
        i = flow_data
        for j in range(len(i)):
            if (j == 2) or (j == 3) or (j == len(i) - 1):
                file.write("{}\n".format(i[j]))
            else:
                file.write("{} ".format(i[j]))
    file.close()
    
    print(speeds)
    with open(output_path+'speed.pkl','wb') as file:
        pickle.dump(speeds, file)


def main():
    data_path = './data/data_segmented.pkl'
    output_path = './data/'

    # process_segmentation()
    generate_traffic(data_path, output_path)

if __name__ == '__main__':
    main()