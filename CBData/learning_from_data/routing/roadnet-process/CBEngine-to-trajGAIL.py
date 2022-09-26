road_dict = {}
trajGAIL_roadnet = {}
with open("../roadnet-match/road/OpenEngine-roadnet-shenzhen.txt", "r") as f:
    lines = f.readlines()
    count = 0
    for line in lines:
        line = line.rstrip('\n').split(' ')
        if('' in line):
            line.remove('')

        if(len(line) == 1):
            count += 1
        else:
            if (count == 1):
                continue
            elif (count == 2):
                if (len(line) == 8):
                    road_dict[line[-2]] = line[-1]
                    road_dict[line[-1]] = line[-2]
                    trajGAIL_roadnet[line[-2]] = ["-1"] * 3
                    trajGAIL_roadnet[line[-1]] = ["-1"] * 3
            else:
                index_map = [0, 1, 2, 3, 4, 1, 2, 3, 4]
                for i in range(1, 5):
                    if (int(line[i]) == -1):
                        continue
                    else:
                        trajGAIL_roadnet[road_dict[line[i]]][0] = line[index_map[i + 3]]
                        trajGAIL_roadnet[road_dict[line[i]]][1] = line[index_map[i + 2]]
                        trajGAIL_roadnet[road_dict[line[i]]][2] = line[index_map[i + 1]]

key_list = trajGAIL_roadnet.keys()
with open("./output/Network.txt", "w") as f:
    for road in key_list:
        for i in range(3):
            if int(trajGAIL_roadnet[road][i]) == -1: 
                continue
            f.write(road + ' ' + str(i + 1) + ' ' + trajGAIL_roadnet[road][i] + '\n')