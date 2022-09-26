import cityflow
import json
import time


def main():
	max_epoch = 3600
	roadnet_json_path = "./data/roadnet.json"

	with open(roadnet_json_path, 'r') as f:
		inter_list = json.load(f)['intersections']
	inters_dict = {}
	for inter in inter_list:
		inters_dict[inter['id']] = len(inter['trafficLight']['lightphases'])

	eng = cityflow.Engine(config_file="./cfg/config.json", thread_num=20)
	start_time = time.time()
	max_vehicle_num = 0

	for step in range(max_epoch):
		for inter_id in inters_dict:
			if inters_dict[inter_id] > 0:
				eng.set_tl_phase(inter_id, ((int(eng.get_current_time()) // 30) % inters_dict[inter_id]))
		total_sum = len(eng.get_vehicles(include_waiting=True))
		eng.get_lane_vehicles()
		vehicle_num = eng.get_vehicle_count()
		if vehicle_num > max_vehicle_num:
			max_vehicle_num = vehicle_num
		# print('t: {}, v: {}'.format(step, vehicle_num))
		print('t: {}, v: {}, total_sum: {}'.format(step, vehicle_num, total_sum))
		eng.next_step()

	end_time = time.time()
	print('Runtime: {}, max vehicle num: {}'.format(end_time - start_time, max_vehicle_num))


if __name__ == '__main__':
	main()
