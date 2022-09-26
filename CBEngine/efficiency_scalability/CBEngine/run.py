import os, sys
from wrapper import Wrapper
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cbengine

def main():
    roadnet_file = './data/roadnet.txt'
    flow_file = './data/flow.txt'
    wrapper = Wrapper(roadnet_file, flow_file)
    wrapper.test()
    pass

if __name__ == '__main__':
    main()