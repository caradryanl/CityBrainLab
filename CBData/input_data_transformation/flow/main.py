


from trafficGenerator import Traffic_generator
import argparse

parser = argparse.ArgumentParser(description="OpenEngine Args")
parser.add_argument(
    "--numveh",
    type=int,
    default=10000,
    help="the total number of vehicles that appear.",
)
args = parser.parse_args()


def main():
    main_folder = "./data/"
    output_folder = './output/'
    city = "nanchang"
    traffic_duration = 3600
    numveh = args.numveh
    
    generator = Traffic_generator(city, main_folder, output_folder, numveh, traffic_duration=traffic_duration)
    generator.initialize(cpus=4)

if __name__ == '__main__':
    main()