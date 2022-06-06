
from main import *
import os
import sys
import pickle

# This file is meant for comparing a probabilistic domain result and deterministic domain result
# on the same probabilistic world, i.e. mdp problem


def main(arg1, arg2):

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


if __name__ == "__main__":

    # Input probabilistic domain
    prob_domain = input("Choose one of the marine domains: domain, domain2, domain3... ")
    problem = 'problem1' # This is irrelevant

    main(prob_domain, problem)

    # OLD
    # Run like this, for example: "python3 evaluate_policy.py domain_deterministic.ppddl problem1.ppddl"
    # main(sys.argv[1],sys.argv[2])


