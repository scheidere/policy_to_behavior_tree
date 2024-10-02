from main import *
import os
import sys
import pickle
import time
import random
import roslaunch
from evaluate_mdp_policy import *


def get_file_paths(domain_option):

    if domain_option == "m":
        pddl_path = "/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine_sensitivity/"

    elif domain_option == "i":

        pddl_path = "/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/infant_sensitivity/"

    problem_path = pddl_path + "problem.ppddl"

    return pddl_path, problem_path


def evaluate(mdp_problem_final, path_to_domains, problem_path, file, f):

    domain_path = path_to_domains + file
    # domain  = PDDLParser.parse(domain_path)
    # problem = PDDLParser.parse(problem_path)

    print("python3 main.py " + domain_path + " " + problem_path)
    os.system("python3 main.py " + domain_path + " " + problem_path)

    # To confirm main was run correctly and saving appropriate results, pull domain and problem as follows
    f_dp = open("/home/emily/Desktop/more_AURO_results/domain_and_problem.txt", "r")
    lines = f_dp.readlines()
    dp = lines[0].strip()
    pp = lines[1].strip()

    # Add domain and problem to analysis tracking file
    f.write(dp+ "\n")
    f.write(pp+ "\n")

    # Get policy
    policy_path = "/home/emily/auro_ws/src/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/policy.p"
    with open(policy_path, 'rb') as f_policy:
        policy = pickle.load(f_policy)
    print(str(policy))
    f.write(str(policy)+"\n")

    # Get reward in "real"/correct MDP
    reward_from_policy = evaluate_mdp_policy_sensitivity(mdp_problem_final, policy)
    f.write(str(reward_from_policy)+"\n")


def run(domain_option, group):

    # Create output file
    f = open("/home/emily/Desktop/more_AURO_results/sensitivity_analysis_info.txt", "w+")

    path_to_domains, problem_path = get_file_paths(domain_option)

    # # Run domain without mistakes in it COMMENTED OUT FOR TESTING
    final_domain_path = path_to_domains+"final_domain.ppddl"
    print("python3 main.py " + final_domain_path + " " + problem_path)
    os.system("python3 main.py " + final_domain_path + " " + problem_path)

    # To confirm main was run correctly and saving appropriate results, pull domain and problem as follows
    f_dp = open("/home/emily/Desktop/more_AURO_results/domain_and_problem.txt", "r")
    lines = f_dp.readlines()
    dp = lines[0].strip()
    pp = lines[1].strip()

    # Add domain and problem to analysis tracking file
    f.write(dp+ "\n")
    f.write(pp+ "\n")

    # Get mdp_problem from that (for use when evaluating the other policies)
    mdp_path = "/home/emily/auro_ws/src/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/mdp_problem.p"
    with open(mdp_path, 'rb') as f_mdp:
        mdp_problem_final = pickle.load(f_mdp)
    print("mdp_problem_final.P", mdp_problem_final.actions_with_params)

    # Get policy
    policy_path = "/home/emily/auro_ws/src/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/policy.p"
    with open(policy_path, 'rb') as f_policy:
        policy_final = pickle.load(f_policy)
    print(str(policy_final))
    f.write(str(policy_final)+"\n")

    # Get reward in "real"/correct MDP
    final_reward_from_policy = evaluate_mdp_policy_sensitivity(mdp_problem_final, policy_final)
    f.write(str(final_reward_from_policy)+"\n")

    path_to_domain_group = path_to_domains+group+"/"
    #print(path_to_domain_group)
    domain_files = os.listdir(path_to_domain_group)
    for file in domain_files:

        evaluate(mdp_problem_final, path_to_domain_group, problem_path, file, f)


if __name__ == "__main__":

    domain_option = input("'i' or 'm'?")
    group = input("Pick group to evaluate: missing_precond, added_wrong_precond, penalty_higher, reward_higher, likelihood_higher, or likelihood_lower?")

    run(domain_option, group)
