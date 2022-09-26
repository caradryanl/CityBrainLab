from statistics import mean
import matplotlib
# matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import pickle
import numpy as np
from calculate import calc

cases = ["Loss", "Bleu", "Meteor"]

iters = 50

plt.figure(figsize=(26, 7))

for case in cases:
    result = []
    with open("Single-OD-RNN-{}.pkl".format(case), "rb") as f:
        result = pickle.load(f)

    list = []
    if case == "Loss":
        list = result
    else:
        list = calc(result)

    list = list[: int(len(list) / 2)]
    x = np.linspace(0, iters, len(list))

    plt.subplot(1, 3, cases.index(case) + 1)
    # plt.title(case, fontsize = 30)
    plt.xlabel("Iteration", fontsize = 36)
    plt.ylabel(case, fontsize = 36)
    plt.plot(x, list, color='deepskyblue', linewidth=2)
    plt.scatter(x, list, c='none', marker='o', s=24, edgecolors='deepskyblue', linewidth=2)
    plt.grid(axis='y')
    plt.tick_params(labelsize= 31)

plt.subplots_adjust(left=0.04, right=0.98, top=0.98, bottom=0.16)

plt.savefig("./fig/fig.png")
plt.savefig("./fig/fig.pdf")