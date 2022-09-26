# 

class RemoveSignal:
    
    def __init__(self, input_path, output_path):
        self.input_path = input_path
        self.output_path = output_path

    def rm_signal(self):
        roadnet_dict = {'inter':[], 'road':[], 'signal':[], 'reserved_signal':set()}

        with open(self.input_path, 'r') as in_f:
            # read intersections
            inter_num = int(in_f.readline().rstrip('\n'))
            for inter in range(inter_num):
                roadnet_dict['inter'].append(in_f.readline())
            
            # read roads
            road_num = int(in_f.readline().rstrip('\n'))
            for road in range(road_num):
                road_line_1 = in_f.readline()
                road_line_2 = in_f.readline()
                road_line_3 = in_f.readline()
                roadnet_dict['road'].append(road_line_1 + road_line_2 + road_line_3)
                
            # read traffic signals
            signal_num = int(in_f.readline().rstrip('\n'))
            for signal in range(signal_num):
                signal_line = in_f.readline()
                signal_list = [int(x) for x in signal_line.rstrip('\n').split(' ')]
                no_road_count = 0
                for road in signal_list[1 : ]:
                    if road == -1:
                        no_road_count += 1
                        
                if no_road_count <= 1: # reserve
                    roadnet_dict['reserved_signal'].add(signal_list[0])
                    roadnet_dict['signal'].append(signal_line)
                # else remove


        with open(self.output_path, 'w') as out_f:
            # write intersections
            out_f.write("{}\n".format(len(roadnet_dict['inter'])))
            for inter_line in roadnet_dict['inter']:
                inter_list = [float(x) for x in inter_line.rstrip('\n').split(' ')]
                if int(inter_list[2]) in roadnet_dict['reserved_signal']:
                    out_f.write(inter_line)
                else:
                    out_f.write("{lat} {lon} {id} 0\n".format(lat=inter_list[0],
                                                            lon=inter_list[1],
                                                            id=int(inter_list[2])))
            
            # write roads
            out_f.write("{}\n".format(len(roadnet_dict['road'])))
            for road_line in roadnet_dict['road']:
                out_f.write(road_line)
            
            # write traffic signals
            out_f.write("{}\n".format(len(roadnet_dict['signal'])))
            for signal_line in roadnet_dict['signal']:
                out_f.write(signal_line)
            