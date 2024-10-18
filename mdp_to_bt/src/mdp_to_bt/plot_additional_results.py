#!/usr/bin/env python3

from main import *
import os
import sys
import pickle
import time
import json
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

def read_results(filename):

    data = np.load(filename)
    print("%s...\n" %filename)
    print(data)

    return data

def read_all_results(directories):

    results = []

    for d in directories:

        for f in os.listdir(d):
            path = os.path.join(d, f)
            results.append(read_results(path))


    return results

def plot_prob(prob_data, domain):

    # prob_data refers to results for the set of domains with varied penalty magnitudes (16)

    fig, ax = plt.subplots(1,1)

    img = ax.imshow(prob_data,extent=[.5, 4.5, 0.5, 4.5],origin='lower')

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

def plot_prob_test(prob_data, pen_data, domain):
    fig = plt.figure(figsize=(8, 8))  # Adjusted figure size to better fit plots and colorbar
    gs = gridspec.GridSpec(2, 1, height_ratios=[4, 1], hspace=0.4)  # Adjust space between plots

    # Calculate the positions for centering
    left_position = 0.25  # Centered left position (0.25 = 25% from left of the figure)
    width = 0.5          # Width of each subplot (50% of the figure)
    
    # Plot the original data
    ax0 = plt.subplot(gs[0])
    img1 = ax0.imshow(prob_data, extent=[0.5, 4.5, 0.5, 4.5], origin='lower', aspect='equal')

    # Set the position for the first subplot
    ax0.set_position([left_position, 0.45, width, 0.4])  # [left, bottom, width, height]

    # Configure the first subplot
    x_label_list_prob = ['1', '2', '3', '4']
    y_label_list = ['1', '2', '3', '4']
    
    ax0.set_xticks([1, 2, 3, 4])
    ax0.set_yticks([1, 2, 3, 4])
    ax0.set_xticklabels(x_label_list_prob, fontsize=14)
    ax0.set_yticklabels(y_label_list, fontsize=14)
    ax0.set_xlabel('False Negative Penalty', fontsize=14)
    ax0.set_ylabel('False Positive Penalty', fontsize=14)

    if domain == 'i':
        ax0.set_title('Infant Domain - Constant Probability', fontsize=14)
    elif domain == 'm':
        ax0.set_title('Marine Domain - Constant Probability', fontsize=14)

    ax1 = plt.subplot(gs[1])
    pen_data_reshaped = np.array(pen_data).reshape(1, -1)
    img2 = ax1.imshow(pen_data_reshaped, aspect='equal', extent=[0.5, 4.5, 0, 1], origin='lower', cmap='viridis')

    # Set the position for the second subplot
    ax1.set_position([left_position, 0.2, width, 0.1])  # [left, bottom, width, height]

    # Configure the second subplot
    ax1.set_xticks([1, 2, 3, 4])
    x_label_list_pen = ['10%', '20%', '30%', '40%']
    ax1.set_xticklabels(x_label_list_pen, fontsize=14)
    ax1.set_yticks([])
    ax1.set_xlabel('Action Effect Uncertainty', fontsize=14)

    if domain == 'i':
        ax1.set_title('Infant Domain - Constant Penalty', fontsize=14)
    elif domain == 'm':
        ax1.set_title('Marine Domain - Constant Penalty', fontsize=14)

    # Add a vertical colorbar to the right of the plots
    cbar_ax = fig.add_axes([0.8, 0.1, 0.02, 0.8])  # Adjust position and size
    cbar = fig.colorbar(img1, cax=cbar_ax)
    for t in cbar.ax.get_yticklabels():
        t.set_fontsize(14)
    cbar.set_label("Runtime (in seconds)", fontsize=14)


    #print(np.min(prob_data), np.min(pen_data))
    #print(np.max(prob_data), np.max(pen_data))
    combined_data = np.concatenate((prob_data.flatten(), pen_data.flatten()))
    combined_mean = np.mean(combined_data)
    print("Mean: ", combined_mean)
    print("std dev: ", np.std(combined_data))
    print("min max:")
    print(np.min(combined_data), np.max(combined_data))

    plt.show()


# def plot_prob_test(prob_data, pen_data, domain):
#     fig = plt.figure(figsize=(12, 8))  # Adjusted figure size to better fit plots and colorbar
#     gs = gridspec.GridSpec(2, 1, height_ratios=[4, 1], hspace=0.4)  # Adjust space between plots

#     # Plot the original data
#     ax0 = plt.subplot(gs[0])
#     img1 = ax0.imshow(prob_data, extent=[0.5, 4.5, 0.5, 4.5], origin='lower', aspect='equal')

#     # Configure the first subplot
#     x_label_list_prob = ['1', '2', '3', '4']
#     y_label_list = ['1', '2', '3', '4']
    
#     ax0.set_xticks([1, 2, 3, 4])
#     ax0.set_yticks([1, 2, 3, 4])
#     ax0.set_xticklabels(x_label_list_prob)
#     ax0.set_yticklabels(y_label_list)
#     ax0.set_xlabel('False Negative Penalty')
#     ax0.set_ylabel('False Positive Penalty')
    
#     if domain == 'i':
#         ax0.set_title('Infant Domain - Constant Probability')
#     elif domain == 'm':
#         ax0.set_title('Marine Domain - Constant Probability')

#     # # Annotate each cell in prob_data
#     # for i in range(prob_data.shape[0]):
#     #     for j in range(prob_data.shape[1]):
#     #         ax0.text(j + 0.5, i + 0.5, f'{prob_data[i, j]:.2f}', 
#     #                  ha='center', va='center', color='black')

#     ax1 = plt.subplot(gs[1])
#     pen_data_reshaped = np.array(pen_data).reshape(1, -1)  # reshape to (1, 4)
#     img2 = ax1.imshow(pen_data_reshaped, aspect='equal', extent=[0.5, 4.5, 0, 1], origin='lower', cmap='viridis')

#     # Configure the second subplot
#     ax1.set_xticks([1, 2, 3, 4])
#     x_label_list_pen = ['10%','20%','30%','40%']
#     ax1.set_xticklabels(x_label_list_pen)
#     ax1.set_yticks([])
#     ax1.set_yticklabels([])
#     ax1.set_xlabel('Action Effect Uncertainty (%)')
#     if domain == 'i':
#         ax1.set_title('Infant Domain - Constant Penalty')
#     elif domain == 'm':
#         ax1.set_title('Marine Domain - Constant Penalty')


#     # Add a vertical colorbar to the right of the plots
#     cbar_ax = fig.add_axes([0.92, 0.1, 0.02, 0.8])  # Adjust position and size
#     cbar = fig.colorbar(img1, cax=cbar_ax)
#     cbar.set_label("Runtime (in seconds)")

#     plt.show()

    #### OPTION 2
    # import numpy as np
    # import matplotlib.pyplot as plt
    # import matplotlib.gridspec as gridspec

    # # Generate randomly populated arrays
    # #data1 = np.random.rand(10,10)*10 
    # #data2 = np.random.rand(10,10)*10 - 7.5
    # data1 = prob_data
    # data2 = pen_data

    # # Find minimum of minima & maximum of maxima
    # minmin = np.min([np.min(data1), np.min(data2)])
    # maxmax = np.max([np.max(data1), np.max(data2)])

    # # Create a figure with GridSpec layout
    # fig = plt.figure(figsize=(10, 8))
    # gs = gridspec.GridSpec(2, 2, width_ratios=[1, 0.05], height_ratios=[1, 1], wspace=0.3, hspace=0.3)

    # # Plot the first data
    # ax1 = plt.subplot(gs[0, 0])
    # im1 = ax1.imshow(data1, vmin=minmin, vmax=maxmax, extent=(0.5, 4.5, 0, 1), aspect='auto', cmap='viridis')
    # ax1.set_title('Constant Probability')

    # # Plot the second data
    # ax2 = plt.subplot(gs[1, 0])
    # im2 = ax2.imshow(data2, vmin=minmin, vmax=maxmax, extent=(0.5, 4.5, 0, 1), aspect='auto', cmap='viridis')
    # ax2.set_title('Constant Penalty')

    # # Add a vertical colorbar on the right side
    # cbar_ax = plt.subplot(gs[:, 1])  # Colorbar axis takes up all rows in the second column
    # cbar = fig.colorbar(im1, cax=cbar_ax)
    # cbar.set_label("Runtime (seconds)")

    # # Show the plot
    # plt.show()


def plot_pen(pen_data):

    # pen_data refers to results for the set of domains with varied action effect uncertainty (4)

    pass

if __name__ == "__main__":

    domain = input("domain ('i' or 'm'): ")

    directory = "imshow_data/"

    i_pen_path = directory+"i_pen/"
    i_prob_path = directory+"i_prob/"
    m_pen_path = directory+"m_pen/"
    m_prob_path = directory+"m_prob/"

    if domain == "i":
        pen_path, prob_path = i_pen_path, i_prob_path
        prob_runtime = read_results(prob_path+"imshow_i_prob_runtimes.npy")
        pen_runtime = read_results(pen_path+"imshow_i_pen_runtimes.npy")
        prob_bt_rewards = read_results(prob_path+"imshow_i_prob_bt_rewards.npy")
        pen_bt_rewards = read_results(pen_path+"imshow_i_pen_bt_rewards.npy")
        prob_raw_rewards = read_results(prob_path+"imshow_i_prob_raw_rewards.npy")
        pen_raw_rewards = read_results(pen_path+"imshow_i_pen_raw_rewards.npy")

    elif domain == "m":
        pen_path, prob_path = m_pen_path, m_prob_path
        prob_runtime = read_results(prob_path+"imshow_m_prob_runtimes.npy")
        pen_runtime = read_results(pen_path+"imshow_m_pen_runtimes.npy")
        prob_bt_rewards = read_results(prob_path+"imshow_m_prob_bt_rewards.npy")
        pen_bt_rewards = read_results(pen_path+"imshow_m_pen_bt_rewards.npy")
        prob_raw_rewards = read_results(prob_path+"imshow_m_prob_raw_rewards.npy")
        pen_raw_rewards = read_results(pen_path+"imshow_m_pen_raw_rewards.npy")


    # directories = [pen_path,prob_path]
    # input("hi")

    # Get data
    #[pen_bt, pen_raw, pen_runtime, prob_bt, prob_raw, prob_runtime] = read_all_results(directories)

    # For now let's just plot runtimes (first just for prob)
    print(prob_runtime)
    print(pen_runtime)
    #plot_prob(prob_runtime, domain)
    plot_prob_test(prob_runtime, pen_runtime, domain)

    if domain=="i":
        prob_bt_rewards_int = np.round(prob_bt_rewards).astype(int)
        prob_raw_rewards_int = np.round(prob_raw_rewards).astype(int)
        pen_bt_rewards_int = np.round(pen_bt_rewards).astype(int)
        pen_raw_rewards_int = np.round(pen_raw_rewards).astype(int)
        print(prob_bt_rewards_int)
        print(prob_raw_rewards_int)
        print(pen_bt_rewards_int)
        print(pen_raw_rewards_int)
    elif domain=="m":
        print(prob_bt_rewards)
        print(prob_raw_rewards)
        print(pen_bt_rewards)
        print(pen_raw_rewards)

    print("BT results: ")
    combined_data = np.concatenate((prob_bt_rewards.flatten(), pen_bt_rewards.flatten()))
    combined_mean = np.mean(combined_data)
    print("Mean: ", combined_mean)
    print("std dev: ", np.std(combined_data))
    print("min max:")
    print(np.min(combined_data), np.max(combined_data))

    print("Raw results: ")
    combined_data2 = np.concatenate((prob_raw_rewards.flatten(), pen_raw_rewards.flatten()))
    combined_mean2 = np.mean(combined_data2)
    print("Mean: ", combined_mean2)
    print("std dev: ", np.std(combined_data2))
    print("min max:")
    print(np.min(combined_data2), np.max(combined_data2))
