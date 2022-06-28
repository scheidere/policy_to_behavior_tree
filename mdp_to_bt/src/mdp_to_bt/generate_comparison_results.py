


from main import *
import os
import sys
import pickle
import time

import matplotlib.pyplot as plt




def plot(domain1,domain2):

    avg_reward_prob, avg_reward_det = compare_policies()



def compare_policies(probabilistic_domain_path, do_prints = False):

    # Path to policy and mdp_problem pickle files
    output_path = "/home/scheidee/new_bt_generation_ws/src/bt_generation/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/"

    partial_path = "/home/scheidee/new_bt_generation_ws/src/bt_generation/policy_to_behavior_tree/pypddl-parser/pypddl-parser/pddl"
    
    # Path to PPDDL domain and problem files
    prob_domain_path = probabilistic_domain_path
    det_domain_path = partial_path + '/marine/both_false_penalty/domain_deterministic.ppddl'
    problem_path = partial_path + "/marine/problems/problem1.ppddl"

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
    avg_reward_prob = get_average_reward(num_trials, mdp_problem, prob_policy)
    print('\nAverage reward over %d trials: %f' % (num_trials, avg_reward_prob))
    print('Deterministic path', det_domain_path)
    print('Deterministic: ', det_policy)
    avg_reward_det = get_average_reward(num_trials, mdp_problem, det_policy)
    print('\nAverage reward over %d trials: %f' % (num_trials, avg_reward_det))

    #percentIncrease(avg_reward_prob, avg_reward_det)
    per_diff = percentDifference(avg_reward_prob, avg_reward_det) # + if prob better, - if det better

    #return avg_reward_prob, avg_reward_det
    return per_diff

def percentIncrease(prob_avg_rew, det_avg_rew):

    small = min(prob_avg_rew,det_avg_rew)
    less_small = max(prob_avg_rew,det_avg_rew)

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

    output_path = "/home/scheidee/new_bt_generation_ws/src/bt_generation/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/"

    # Run main with deterministic domain; Save policy
    os.system("python3 main.py " + domain_path + " " + problem_path)

    # Extract deterministic policy
    file = open(output_path+'policy.p','rb')
    det_policy = pickle.load(file)
    file.close()
    #print('d', det_policy)

    return det_policy

def get_probabilistic_policy(domain_path, problem_path, output_path):

    output_path = "/home/scheidee/new_bt_generation_ws/src/bt_generation/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/"

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

    dec_per_diff = (prob_avg_rew - det_avg_rew)/det_avg_rew

    per_diff = 100*dec_per_diff

    print('diff (+ if prob higher) ', per_diff)

    return per_diff

def main():

    start_time = round(time.time())

    pddl_path = "/home/scheidee/new_bt_generation_ws/src/bt_generation/policy_to_behavior_tree/pypddl-parser/pypddl-parser/pddl/marine/"

    test_fp_penalty = "false_positive_penalty/" # constant probability
    test_np_penalty = "false_negative_penalty/" # constant probability
    test_both_penalty = "both_false_penalty/" # constant probability
    # test_fp_probability = "false_positive_probability_range" # constant penalty
    # test_np_probability = "false_negative_probability_range" # constant penalty

    path_to_prob_domains = pddl_path + test_both_penalty

    domain_files = os.listdir(path_to_prob_domains)
    ###domain_files = ['fn1fp1.ppddl']#,'fn2fp2.ppddl']#,'fn2fp2.ppddl'] # for testing

    # Init array to be plotted to show percent increase for each false positive/negative magnitude combination
    n = 4
    percent_increase_array = np.zeros((n,n))

    # Path to policy and mdp_problem pickle files
    output_path = "/home/scheidee/new_bt_generation_ws/src/bt_generation/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/"
    
    # Path to PPDDL domain and problem files
    det_domain_path = pddl_path + test_both_penalty + "domain_deterministic.ppddl"
    problem_path = pddl_path + "problems/problem1.ppddl"

    for file in domain_files:

        if file != 'domain_deterministic.ppddl':

            print(file)
            
            fn_num,fp_num = int(float(file[2])), int(float(file[5])) # fn will be x-axis, fp will be y-axis (i in array)
            j,i = fn_num - 1, fp_num - 1

            prob_domain_path = path_to_prob_domains + file

            per_diff = compare_policies(prob_domain_path)

            # Add percent to unflipped array
            percent_increase_array[i][j] = per_diff

    # Flip i's so that plot increases from lower left, not upper left corner
    # SKIPPING FLIP FOR NOW BECAUSE IMSHOW EXPECTS ARRAY INDEXING ALREADY
    #percent_increase_array = np.flip(percent_increase_array,axis=0) 

    print(percent_increase_array)

    plt.figure(figsize=(12,4))

    plt.subplot(132)
    im=plt.imshow(percent_increase_array, cmap = 'viridis', interpolation='nearest')
    plt.title('Percent Increase of Our Method over Baseline', y=1.02, fontsize=12)
    plt.xlabel('False Negative Penalty')
    plt.ylabel('False Positive Penalty')
    plt.xticks(range(n))
    plt.yticks(range(n))
    clb = plt.colorbar(im,fraction=0.046, pad=0.04)
    #clb.ax.set_title('Percent Increase') # needs to be vertical to not overlap with main title
    plt.savefig(str(start_time) + '_fnfp_penalty_results')
    #plt.show() #only this or savefig works, one at a time





def test():


    pddl_path = "/home/scheidee/new_bt_generation_ws/src/bt_generation/policy_to_behavior_tree/pypddl-parser/pypddl-parser/pddl/marine/"

    test_fp_penalty = "false_positive_penalty/" # constant probability
    test_np_penalty = "false_negative_penalty/" # constant probability
    test_both_penalty = "both_false_penalty/" # constant probability
    # test_fp_probability = "false_positive_probability_range" # constant penalty
    # test_np_probability = "false_negative_probability_range" # constant penalty

    ####path_to_prob_domains = pddl_path + test_both_penalty

    # Reverting to old for testing
    path_to_prob_domains = pddl_path + 'old_testing/'
    det_domain_path = pddl_path + 'old_testing/domain_deterministic.ppddl'

    domain_files = os.listdir(path_to_prob_domains)

    # Init array to be plotted to show percent increase for each false positive/negative magnitude combination
    percent_increase_array = np.zeros((4,4))

    # Path to policy and mdp_problem pickle files
    output_path = "/home/scheidee/new_bt_generation_ws/src/bt_generation/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/"
    
    # Path to PPDDL domain and problem files
    ####det_domain_path = pddl_path + "domain_deterministic.ppddl"
    problem_path = pddl_path + "problems/problem1.ppddl"

    det_policy = get_deterministic_policy(det_domain_path, problem_path, output_path)
    print('det', det_policy)

    ##file = 'fn4fp4.ppddl' #23% worse than det
    #file = 'fn1fp1.ppddl' #0.6% worse
    file = 'domain6.ppddl'

    prob_domain_path = path_to_prob_domains + file

    prob_policy, mdp_problem = get_probabilistic_policy(prob_domain_path, problem_path, output_path)
    print('prob', prob_policy)

    avg_reward_prob, avg_reward_det = get_policies_rewards(det_policy, prob_policy, mdp_problem)
    print('det r', avg_reward_det, '\nprob r', avg_reward_prob)

    percent_diff_wrt_det = percentDifference(avg_reward_prob, avg_reward_det)
    print('percent above (+) or below (-) prob rew is above det rew', percent_diff_wrt_det)




if __name__ == "__main__":


    main()
    #test()