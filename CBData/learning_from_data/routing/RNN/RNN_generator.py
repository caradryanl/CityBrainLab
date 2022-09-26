# Refer to https://github.com/benchoi93/TrajGAIL

import os
import pickle
import argparse
import numpy as np
from mdp import shortestpath
from models.behavior_clone.rnn_predictor import *

def argparser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gangnam', default = False, action = "store_true" )
    parser.add_argument("--max-length",default = 60,type=int)
    parser.add_argument("--num-trajs",default=1000,type=int)
    parser.add_argument("--trained-epoches",default=980,type=int)
    return parser.parse_args()

args = argparser()

origins = []
destinations = []
with open("./data/Network_in_out/net-in.pkl", "rb") as f:
    origins = pickle.load(f)
with open("./data/Network_in_out/net-out.pkl", "rb") as f:
    destinations = pickle.load(f)

sw = shortestpath.ShortestPath("data/Network.txt",origins, destinations)
dataset_list = ["Single_OD"]
list_all_data = []
for dataset in dataset_list:
    for data0 in os.listdir(os.path.join("data",dataset)):
        list_all_data.append(os.path.join("data",dataset,data0))

N_STATES = sw.n_states
for data0 in list_all_data:
    
    print("current data :: {}".format(data0))
    # data0=list_all_data[0]
    trajs = sw.import_demonstrations(data0)

    datainfo = data0.split(os.sep)

    RNNMODEL = RNN_predictor(sw.states, 256, pad_idx = -1)
    RNNMODEL.load_state_dict(torch.load(os.path.join("Result",datainfo[-2],
    'RNN_{}_{}_{}.pth'.format(datainfo[-2], datainfo[-1].split(".")[0], args.trained_epoches)),
    map_location=torch.device('cpu')))

    generated_rnn = RNNMODEL.unroll_trajectories(sw.start, sw.terminal, args.num_trajs, args.max_length)
    find_state = lambda x: RNNMODEL.states[x]
    np_find_state= np.vectorize(find_state)
    generated_rnn = np_find_state(generated_rnn.numpy())

    import nltk

    exp_trajs = [[i.cur_state for i in x]+[x[-1].next_state] for x in trajs]

    generated_rnn = [[x for x in list(x) if x != -1] for x in generated_rnn]

    bleu_result = np.zeros(shape=(args.num_trajs , 4))
    meteor_result = np.zeros(shape=(args.num_trajs , 4))

    def test_bleu(i):
        return (nltk.translate.bleu(exp_trajs,generated_rnn[i])) 

    def test_meteor(i):
        temp_exp_trajs = [[str(i) for i in x] for x in exp_trajs]

        return (nltk.translate.meteor_score.meteor_score(temp_exp_trajs, [str(x) for x in generated_rnn[i]])) 

    for i in range(args.num_trajs):
        bleu_result[i,:] = test_bleu(i)
        meteor_result[i,:] = test_meteor(i)

    bleu_result_sum = [0 for i in range(len(bleu_result[0, :]))]
    meteor_result_sum = [0 for i in range(len(meteor_result[0, :]))]
    for i in range(args.num_trajs):
        bleu_result_sum = np.sum([bleu_result_sum, bleu_result[i, :]], axis=0).tolist()
        meteor_result_sum = np.sum([meteor_result_sum, meteor_result[i, :]], axis=0).tolist()
    print([x / args.num_trajs for x in bleu_result_sum])
    print([x / args.num_trajs for x in meteor_result_sum])
    with open("./Result/Generated_OD/Single_OD.pkl", "wb") as f:
        pickle.dump(generated_rnn, f)
    break