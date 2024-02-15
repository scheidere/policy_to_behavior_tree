from main import *
import os
import sys
import pickle
import time
import json
import matplotlib.pyplot as plt

def read_results(filename):

    data = np.load(filename)
    print(data)

    return data

def plot(percent_increase_array,domain):
    fig, ax = plt.subplots(1,1)

    img = ax.imshow(percent_increase_array,extent=[.5, 4.5, 0.5, 4.5],origin='lower')

    x_label_list = ['1', '2', '3', '4']
    y_label_list = ['1', '2', '3', '4']

    ax.set_xticks([1, 2, 3, 4])
    ax.set_yticks([1, 2, 3, 4])

    ax.set_xticklabels(x_label_list)
    ax.set_yticklabels(y_label_list)

    plt.xlabel('False Negative Penalty')
    plt.ylabel('False Positive Penalty')
    if domain == 'i':
        plt.title('Infant Domain - Constant Probability')
    elif domain == 'm':
        plt.title('Marine Domain - Constant Probability')

    cbar = fig.colorbar(img)
    cbar.set_label("Percent Increase in Reward")

    #plt.savefig(str(start_time) + '_fnfp_penalty_results')
    plt.show() #only this or savefig works, one at a time


if __name__ == "__main__":

    domain = input("domain ('i' or 'm'): ")

    directory = "imshow_data/"
    data = read_results(directory+"imshow_probability_array_"+ domain + ".npy")

    print(data.max(),data.min())
    print(data.sum()/data.size)

    plot(data,domain)