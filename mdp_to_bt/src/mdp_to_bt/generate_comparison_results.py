from main import *
import os
import sys
import pickle
import time

import matplotlib.pyplot as plt

def compare_policies_testing(prob_domain_path, det_domain_path, problem_path, output_path, do_prints = False):

    # Run main with probabilistic domain; Save policy and mdp problem
    os.system("python3 main.py " + prob_domain_path + " " + problem_path)
    
    # Extract probabilistic policy
    file = open(output_path+'policy.p','rb')
    prob_policy = pickle.load(file)
    file.close()

def compare_policies(prob_domain_path, det_domain_path, problem_path, output_path, do_prints = False):

    # Path to policy and mdp_problem pickle files
    #output_path = "/home/scheidee/new_bt_generation_ws/src/bt_generation/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/"

    #partial_path = "/home/scheidee/new_bt_generation_ws/src/bt_generation/policy_to_behavior_tree/pypddl-parser/pypddl-parser/pddl"
    
    # Path to PPDDL domain and problem files
    # prob_domain_path = probabilistic_domain_path
    # det_domain_path = partial_path + '/marine/both_false_penalty/domain_deterministic.ppddl'
    # problem_path = partial_path + "/marine/problems/problem1.ppddl"

    # Run main with deterministic domain; Save policy
    os.system("python3 main.py " + det_domain_path + " " + problem_path)

    # Extract deterministic policy
    file = open(output_path+'policy.p','rb')
    det_policy = pickle.load(file)
    file.close()
    #print('d', det_policy)

    # Run main with probabilistic domain; Save policy and mdp problem
    os.system("python3 main.py " + prob_domain_path + " " + problem_path)
    
    # Extract probabilistic policy
    file = open(output_path+'policy.p','rb')
    prob_policy = pickle.load(file)
    file.close()
    #print('p', prob_policy)

    # Extract probabilistic mdp problem
    file = open(output_path+'mdp_problem.p','rb')
    mdp_problem = pickle.load(file)
    file.close()
    #print('mdp', mdp_problem)

    # Compare probabilistic and deterministic policies on same world (i.e. mdp problem)
    num_trials = 100
    print('Probabilistic path', prob_domain_path)
    print('Probabilistic: ', prob_policy)
    avg_reward_prob, prob_rewards = get_average_reward(num_trials, mdp_problem, prob_policy)
    print('\nAverage reward over %d trials: %f' % (num_trials, avg_reward_prob))
    print('Deterministic path', det_domain_path)
    print('Deterministic: ', det_policy)
    avg_reward_det, det_rewards = get_average_reward(num_trials, mdp_problem, det_policy)
    print('\nAverage reward over %d trials: %f' % (num_trials, avg_reward_det))

    #percentIncrease(avg_reward_prob, avg_reward_det)
    per_diff, difference = percentDifference(avg_reward_prob, avg_reward_det) # + if prob better, - if det better

    #return avg_reward_prob, avg_reward_det
    return per_diff, difference, avg_reward_prob, avg_reward_det

def percentIncrease(prob_avg_rew, det_avg_rew):

    small = min(prob_avg_rew,det_avg_rew)
    less_small = max(prob_avg_rew,det_avg_rew)

    if small == det_avg_rew and small < 0:

        percent = 100*(less_small - small)/abs(small)

    else: # small > 0 or small < 0 but is prob not det

        percent =  100*(less_small - small)/small

    print('%f is %f percent higher than %f' %(less_small,percent,small))

    return percent

def get_policies_rewards(det_policy, prob_policy, mdp_problem):

    # Get deterministic policy and probabilistic policy reward on same probabilistic problem

    num_trials = 100
    #print('Probabilistic: ', prob_policy)
    avg_reward_prob = get_average_reward(num_trials, mdp_problem, prob_policy)
    #print('\nAverage reward over %d trials: %f' % (num_trials, avg_reward_prob))
    #print('Deterministic: ', det_policy)
    avg_reward_det = get_average_reward(num_trials, mdp_problem, det_policy)
    #print('\nAverage reward over %d trials: %f' % (num_trials, avg_reward_det))

    return avg_reward_det, avg_reward_prob

def get_deterministic_policy(domain_path, problem_path, output_path):

    output_path = "/home/emily/auro_ws/src/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/"

    # Run main with deterministic domain; Save policy
    os.system("python3 main.py " + domain_path + " " + problem_path)

    # Extract deterministic policy
    file = open(output_path+'policy.p','rb')
    det_policy = pickle.load(file)
    file.close()
    #print('d', det_policy)

    return det_policy

def get_probabilistic_policy(domain_path, problem_path, output_path):

    output_path = "/home/emily/auro_ws/src/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/"

    # Also returns mdp problem both the probabilistic and deterministic policy

    # Run main with probabilistic domain; Save policy and mdp problem
    os.system("python3 main.py " + domain_path + " " + problem_path)
    
    # Extract probabilistic policy
    file = open(output_path+'policy.p','rb')
    prob_policy = pickle.load(file)
    file.close()
    #print('p', prob_policy)

    # Extract probabilistic mdp problem
    file = open(output_path+'mdp_problem.p','rb')
    mdp_problem = pickle.load(file)
    file.close()
    #print('mdp', mdp_problem)

    return prob_policy, mdp_problem


def percentDifference(prob_avg_rew, det_avg_rew):

    # small = min(prob_avg_rew,det_avg_rew)
    # less_small = max(prob_avg_rew,det_avg_rew)

    # percent =  100*(less_small - small)/small
    # print('%f is %f percent higher than %f' %(less_small,percent,small))

    difference = prob_avg_rew - det_avg_rew

    dec_per_diff = (prob_avg_rew - det_avg_rew)/det_avg_rew

    per_diff = 100*dec_per_diff

    if per_diff < 0 and det_avg_rew < prob_avg_rew:

        per_diff = abs(per_diff)

    print('diff (+ if prob higher) ', per_diff)

    return per_diff, difference

def get_penalty_results(domain):

    start_time = round(time.time())

    # Path to policy and mdp_problem pickle files
    output_path = "/home/emily/auro_ws/src/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/" # Laptop

    if domain == 'm':
        # Marine
        pddl_path = "/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/marine/"
        
        test_fp_penalty = "false_positive_penalty/" # constant probability
        test_np_penalty = "false_negative_penalty/" # constant probability
        test_both_penalty = "both_false_penalty/" # constant probability
        # test_fp_probability = "false_positive_probability_range" # constant penalty
        # test_np_probability = "false_negative_probability_range" # constant penalty

        # Path to PPDDL domain and problem files
        det_domain_path = pddl_path + test_both_penalty + "domain_deterministic.ppddl"
        problem_path = pddl_path + "problems/problem1.ppddl"
        path_to_prob_domains = pddl_path + test_both_penalty


    if domain == 'i':
        # Infant
        pddl_path = "/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/infant_mobility/" # Laptop
        path_to_prob_domains = pddl_path + "penalty_p30const/" #+ "penalty_p20const/"
        problem_path = pddl_path + "problems/problem3.ppddl"
        det_domain_path = path_to_prob_domains + "domain_deterministic.ppddl"


    domain_files = os.listdir(path_to_prob_domains)
    ###domain_files = ['fn1fp1.ppddl']#,'fn2fp2.ppddl']#,'fn2fp2.ppddl'] # for testing

    # Init array to be plotted to show percent increase for each false positive/negative magnitude combination
    n = 4
    percent_increase_array = np.zeros((n,n))


    for file in domain_files:

        if file != 'domain_deterministic.ppddl':

            print(file)
            
            fn_num,fp_num = int(float(file[2])), int(float(file[5])) # fn will be x-axis, fp will be y-axis (i in array)
            j,i = fn_num - 1, fp_num - 1

            prob_domain_path = path_to_prob_domains + file

            per_diff, difference, p, d = compare_policies(prob_domain_path, det_domain_path, problem_path, output_path)

            # Add percent to unflipped array
            percent_increase_array[i][j] = per_diff

    # Flip i's so that plot increases from lower left, not upper left corner
    # SKIPPING FLIP FOR NOW BECAUSE IMSHOW EXPECTS ARRAY INDEXING ALREADY
    #percent_increase_array = np.flip(percent_increase_array,axis=0) 

    print(percent_increase_array)
    np.save("imshow_probability_array_" + domain + ".npy",percent_increase_array)

    # OLD PLOTTING WAY
    # plt.figure(figsize=(12,4))
    # plt.subplot(132)
    # im=plt.imshow(percent_increase_array, cmap = 'viridis', interpolation='nearest')
    # plt.title('Percent Increase of Our Method over Baseline', y=1.02, fontsize=12)
    # plt.xlabel('False Negative Penalty')
    # plt.ylabel('False Positive Penalty')
    # plt.xticks(range(n))
    # plt.yticks(range(n))
    # clb = plt.colorbar(im,fraction=0.046, pad=0.04)
    #clb.ax.set_title('Percent Increase') # needs to be vertical to not overlap with main title

    # NEW PLOTTING WAY (Also in plot_penalty.py)
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

    fig.colorbar(img)

    #plt.savefig(str(start_time) + '_fnfp_penalty_results')
    plt.show() #only this or savefig works, one at a time


def get_probability_results(domain):

    start_time = round(time.time())

    # Notes for domain/problem combos
    #   domain p30 in probability2/ goes with problem2
    #   domain p30 in probability3/, p30_constraints or p30_constraints_consts all go with problem2_constraints

    if domain == 'm':
        # Marine
        pddl_path = "/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/marine/" # Laptop
        path_to_prob_domains = pddl_path + 'probability_fn2fp2/'
        problem_path = pddl_path + "problems/problem1.ppddl"

    if domain == 'i':
        # Infant
        pddl_path = "/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/infant_mobility/" # Laptop

        #path_to_prob_domains = pddl_path + "probability2/" #old
        #problem_path = pddl_path + "problems/problem2.ppddl" #infant (old)

        # New
        # path_to_prob_domains = pddl_path + "probability3/"
        # problem_path = pddl_path + "problems/problem2_constraint.ppddl"

        # Double new
        path_to_prob_domains = pddl_path + "probability4/" #FINAL
        #problem_path = pddl_path + "problems/problem2_constraint.ppddl"
        problem_path = pddl_path + "problems/problem3.ppddl" # has orientation object added, FINAL

    # Both
    det_domain_path = path_to_prob_domains + "domain_deterministic.ppddl" # Use with marine domain
    output_path = "/home/emily/auro_ws/src/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/" # Laptop

    #test_file = 'p30.ppddl' #p30_or_test.ppddl' #p30.ppddl' #'p30_constraints_consts.ppddl' #'p30_constraints.ppddl'
    #test_file = 'p30.ppddl' #marine

    domain_files = os.listdir(path_to_prob_domains)
    domain_files.sort()

    # Init list to be plotted in histogram (y axis)
    percent_increase_list = []
    labels = []
    difference_list = [] # diff between p and d avg rewards
    probabilistic_avg_rew_list = []
    deterministic_avg_rew_list = []

    for file in domain_files:
        print('file', file)
        #input('press enter')

        if file != 'domain_deterministic.ppddl': # and file == test_file: #will need to remove this test_file bit for full run

            print(file)
            label = file[1:3]
            
            prob_domain_path = path_to_prob_domains + file

            per_diff, difference, p, d = compare_policies(prob_domain_path, det_domain_path, problem_path, output_path)
            ###per_diff = compare_policies_testing(prob_domain_path, det_domain_path, problem_path, output_path)

            percent_increase_list.append(per_diff)
            labels.append(label)
            difference_list.append(difference)
            probabilistic_avg_rew_list.append(p)
            deterministic_avg_rew_list.append(d)

    print(percent_increase_list)
    print(labels)
    print(difference_list)
    print('p rewards', probabilistic_avg_rew_list)
    print('d rewards', deterministic_avg_rew_list)
    p_vals = [int(val) for val in probabilistic_avg_rew_list]
    d_vals = [int(val) for val in deterministic_avg_rew_list]


    # NEW PLOTTING WAY (Also in plot_penalty.py)
    plt.bar(labels, percent_increase_list, align='center')
    plt.gca().set_xticks(labels)

    for i in range(len(labels)):
        plt.annotate('p:'+str(p_vals[i])+' d:'+str(d_vals[i]), xy=(labels[i],percent_increase_list[i]), ha='center', va='bottom')


    plt.xlabel('Action Effect Uncertainty') # Likelihood of action failure
    plt.ylabel('Percent Increase')
    #plt.title('Infant Domain: All rewards 2 except bubbles (3) and penalties -2')
    if domain == 'i':
        plt.title('Infant Domain - Constant Penalty') # All rewards 2 except 3 for bubbles
    elif domain == 'm':
        plt.title('Marine Domain - Constant Penalty')

    #plt.savefig(str(start_time) + '_probability_results')
    plt.show() #only this or savefig works, one at a time



def main():

    method = input('To generate penalty or probability plots, respectively, type pen or prob: ')
    domain = input("Choose either the marine (type 'm') or infant (type 'i') domain: ")

    if method == "pen":
        get_penalty_results(domain)
    elif method == "prob":
        get_probability_results(domain)


if __name__ == "__main__":

    # Note: You will need to change three paths, two in this file, one in main
    # when using a new comp

    #Go to main.py and comment out simplification of bt because it is not needed for plotting
    #and slows it down

    main()
