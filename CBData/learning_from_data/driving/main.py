import pickle
from trainer import Trainer
from dataSource import DataSource

def main():
    label_path = './data/speed.pkl'
    roadnet_path = './data/roadnet.txt'
    flow_path = './data/flow.txt'
    cfg_path = './cfgs/test.cfg'

    with open(label_path,'rb') as file:
        label = pickle.load(file)
    label = label
    label = [x/3.6 for x in label]
    source = DataSource(label, interval=20)

    trainer = Trainer(cfg_path, roadnet_path, flow_path, label)
    trainer.train(source, 3)
    # config = {'max_acc_': 2.0, 'min_acc_': 5.0}
    # trainer.compute_res(config)
    

if __name__ == '__main__':
    main()