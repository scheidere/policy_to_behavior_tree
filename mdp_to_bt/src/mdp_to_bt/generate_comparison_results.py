


from main import *
import os
import sys
import pickle




def plot(domain1,domain2):

	avg_reward_prob, avg_reward_det = compare_policies()



def compare_policies(arg1, arg2, do_prints = False):

    # Path to policy and mdp_problem pickle files
    output_path = "/home/scheidee/new_bt_generation_ws/src/bt_generation/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/"
    
    # Path to PPDDL domain and problem files
    partial_path = "/home/scheidee/new_bt_generation_ws/src/bt_generation/policy_to_behavior_tree/pypddl-parser/pypddl-parser/pddl"
    # domain_path = partial_path + "/marine/" + arg1 # OLD
    # problem_path = partial_path + "/marine/problems/" + arg2 # OLD

    prob_domain_path = partial_path + "/marine/" + arg1 + '.ppddl'
    det_domain_path = partial_path + "/marine/domain_deterministic.ppddl"
    problem_path = partial_path + "/marine/problems/" + arg2 + '.ppddl'

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
    print('Probabilistic: ', prob_policy)
    avg_reward_prob = get_average_reward(num_trials, mdp_problem, prob_policy)
    print('\nAverage reward over %d trials: %f' % (num_trials, avg_reward_prob))
    print('Deterministic: ', det_policy)
    avg_reward_det = get_average_reward(num_trials, mdp_problem, det_policy)
    print('\nAverage reward over %d trials: %f' % (num_trials, avg_reward_det))

    percentIncrease(avg_reward_prob, avg_reward_det)

    return avg_reward_prob, avg_reward_det

def percentIncrease(prob_avg_rew, det_avg_rew):

    small = min(prob_avg_rew,det_avg_rew)
    less_small = max(prob_avg_rew,det_avg_rew)

    percent =  100*(less_small - small)/small
    print('%f is %f percent higher than %f' %(less_small,percent,small))

    return percent

def main():

	pddl_path = "/home/scheidee/new_bt_generation_ws/src/bt_generation/policy_to_behavior_tree/pypddl-parser/pypddl-parser/pddl/marine/"

	test_fp_penalty = "false_positive_penalty_range" # constant probability
	test_np_penalty = "false_negative_penalty_range" # constant probability
	test_fp_probability = "false_positive_probability_range" # constant penalty
	test_np_probability = "false_negative_probability_range" # constant penalty