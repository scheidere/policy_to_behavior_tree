#!/usr/bin/env python3

import pickle
from evaluate_mdp_policy import *

# Testing infant domain "mistake" policies in comparison to policy from final domain which should be optimal
# Let's examine i_missing_precond first



def num_actions_differ(final_policy, other_policy):

    num_differences = 0
    for j in range(len(final_policy)):

        if final_policy[j] != other_policy[j]:
            num_differences+=1

    return num_differences

def temp():
    # #testttt = (1, 0, 0, 1, 0, 4, 2, 0, 0, 1, 6, 1, 1, 6, 4, 2, 6, 2, 8, 0, 0, 0, 0, 0, 0, 0, 7, 1, 0, 0, 1, 0, 3, 2, 0, 0, 1, 6, 1, 1, 6, 3, 2, 6, 2, 8, 0, 0, 0, 0, 0, 0, 0, 7)
    # # Loop
    # # Pull what should be worse policy from domains with mistakes
    # one_p = (0, 0, 0, 0, 0, 4, 0, 0, 0, 1, 6, 1, 1, 6, 4, 2, 6, 2, 8, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 3, 0, 0, 0, 1, 6, 1, 1, 6, 3, 2, 6, 2, 8, 0, 0, 0, 0, 0, 0, 0, 7)
    # four_p = (0, 0, 0, 4, 4, 4, 0, 0, 0, 1, 6, 1, 4, 4, 4, 2, 6, 2, 8, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 3, 3, 3, 0, 0, 0, 1, 6, 1, 3, 3, 3, 2, 6, 2, 8, 0, 0, 0, 0, 0, 0, 0, 7)
    # all_p = (0, 0, 0, 4, 4, 4, 0, 0, 0, 6, 6, 6, 4, 4, 4, 6, 6, 6, 8, 8, 8, 0, 0, 0, 7, 7, 7, 0, 0, 0, 3, 3, 3, 0, 0, 0, 6, 6, 6, 3, 3, 3, 6, 6, 6, 8, 8, 8, 0, 0, 0, 7, 7, 7)
    pass

def marine():

    mdp_path = "/home/emily/Desktop/mdp_problem_final_marine.p"
    with open(mdp_path, 'rb') as f_mdp:
        mdp_problem_final = pickle.load(f_mdp)

    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine_sensitivity/final_domain.ppddl
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine_sensitivity/problem.ppddl
    final_p = (5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2)

    return mdp_problem_final, final_p

def infant():

    mdp_path = "/home/emily/Desktop/mdp_problem_final_infant.p"
    with open(mdp_path, 'rb') as f_mdp:
        mdp_problem_final = pickle.load(f_mdp)

    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/infant_sensitivity/final_domain.ppddl
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/infant_sensitivity/problem.ppddl
    final_p = (1, 0, 0, 1, 0, 4, 2, 0, 0, 1, 6, 1, 1, 6, 4, 2, 6, 2, 8, 0, 0, 0, 0, 0, 0, 0, 7, 1, 0, 0, 1, 0, 3, 2, 0, 0, 1, 6, 1, 1, 6, 3, 2, 6, 2, 8, 0, 0, 0, 0, 0, 0, 0, 7)

    
    return mdp_problem_final, final_p

def added_wrong_precond_i():

    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/infant_sensitivity/added_wrong_precond/one.ppddl
    one_p = (1, 1, 1, 1, 1, 4, 2, 2, 2, 1, 6, 1, 1, 6, 4, 2, 6, 2, 8, 0, 0, 0, 0, 0, 0, 0, 7, 1, 0, 0, 1, 0, 3, 2, 0, 0, 1, 6, 1, 1, 6, 3, 2, 6, 2, 8, 0, 0, 0, 0, 0, 0, 0, 7)
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/infant_sensitivity/added_wrong_precond/four.ppddl
    four_p = (2, 2, 2, 2, 2, 4, 2, 2, 2, 2, 6, 2, 2, 6, 2, 2, 6, 2, 8, 0, 0, 0, 0, 0, 0, 0, 7, 1, 0, 0, 1, 0, 3, 2, 0, 0, 1, 6, 1, 1, 6, 1, 2, 6, 2, 8, 0, 0, 0, 0, 0, 0, 0, 7)
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/infant_sensitivity/added_wrong_precond/all.ppddl
    all_p = (4, 2, 2, 2, 2, 4, 2, 2, 4, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 3, 2, 0, 0, 1, 6, 1, 1, 6, 1, 2, 6, 2, 8, 0, 0, 0, 0, 0, 0, 0, 7)


    return one_p, four_p, all_p

def missing_precond_i():

    # Changed order for ease of copy/pasting from output file, it is correct

    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/infant_sensitivity/missing_precond/four.ppddl
    four_p = (0, 0, 0, 4, 4, 4, 0, 0, 0, 1, 6, 1, 4, 4, 4, 2, 6, 2, 8, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 3, 3, 3, 0, 0, 0, 1, 6, 1, 3, 3, 3, 2, 6, 2, 8, 0, 0, 0, 0, 0, 0, 0, 7)
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/infant_sensitivity/missing_precond/one.ppddl
    one_p = (0, 0, 0, 0, 0, 4, 0, 0, 0, 1, 6, 1, 1, 6, 4, 2, 6, 2, 8, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 3, 0, 0, 0, 1, 6, 1, 1, 6, 3, 2, 6, 2, 8, 0, 0, 0, 0, 0, 0, 0, 7)
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/infant_sensitivity/missing_precond/all.ppddl
    all_p = (0, 0, 0, 4, 4, 4, 0, 0, 0, 6, 6, 6, 4, 4, 4, 6, 6, 6, 8, 8, 8, 0, 0, 0, 7, 7, 7, 0, 0, 0, 3, 3, 3, 0, 0, 0, 6, 6, 6, 3, 3, 3, 6, 6, 6, 8, 8, 8, 0, 0, 0, 7, 7, 7)

    return one_p, four_p, all_p


def reward_higher_i():

    # Changed order for ease of copy/pasting from output file, it is correct

    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/infant_sensitivity/reward_higher/four.ppddl
    four_p = (1, 0, 0, 1, 0, 4, 2, 0, 0, 1, 1, 1, 1, 1, 4, 2, 2, 2, 8, 0, 0, 0, 0, 4, 0, 0, 7, 1, 0, 0, 1, 0, 3, 2, 0, 0, 1, 1, 1, 1, 1, 3, 2, 2, 2, 8, 0, 0, 0, 0, 3, 0, 0, 7)
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/infant_sensitivity/reward_higher/one.ppddl
    one_p = (1, 0, 0, 1, 0, 4, 2, 0, 0, 1, 6, 1, 1, 6, 4, 2, 6, 2, 8, 0, 0, 0, 0, 0, 0, 0, 7, 1, 0, 0, 1, 0, 3, 2, 0, 0, 1, 6, 1, 1, 6, 3, 2, 6, 2, 8, 0, 0, 0, 0, 0, 0, 0, 7)
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/infant_sensitivity/reward_higher/all.ppddl
    all_p = (1, 0, 0, 1, 0, 4, 2, 0, 0, 1, 6, 1, 1, 6, 4, 2, 6, 2, 8, 0, 0, 0, 0, 4, 0, 0, 7, 1, 0, 0, 1, 0, 3, 2, 0, 0, 1, 6, 1, 1, 6, 3, 2, 6, 2, 8, 0, 0, 0, 0, 3, 0, 0, 7)

    return one_p, four_p, all_p

def penalty_higher_i():

    # Changed order for ease of copy/pasting from output file, it is correct

    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/infant_sensitivity/penalty_higher/four.ppddl
    four_p = (8, 5, 3, 0, 5, 4, 0, 5, 7, 8, 6, 0, 0, 6, 4, 0, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 7, 8, 5, 3, 0, 5, 3, 0, 5, 7, 8, 6, 0, 0, 6, 3, 0, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 7)
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/infant_sensitivity/penalty_higher/one.ppddl
    one_p = (1, 1, 1, 1, 1, 4, 2, 2, 2, 1, 6, 1, 1, 6, 4, 2, 6, 2, 8, 0, 0, 0, 0, 0, 0, 0, 7, 1, 1, 1, 1, 1, 3, 2, 2, 2, 1, 6, 1, 1, 6, 3, 2, 6, 2, 8, 0, 0, 0, 0, 0, 0, 0, 7)
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/infant_sensitivity/penalty_higher/all.ppddl
    all_p = (8, 3, 3, 0, 3, 4, 0, 1, 7, 8, 6, 0, 0, 6, 4, 0, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 7, 8, 3, 3, 0, 3, 3, 0, 1, 7, 8, 6, 0, 0, 6, 3, 0, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 7)

    return one_p, four_p, all_p

def likelihood_lower_i():

    # Changed order for ease of copy/pasting from output file, it is correct

    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/infant_sensitivity/uncertainty_lower/four.ppddl
    four_p = (8, 5, 3, 0, 5, 4, 0, 5, 7, 8, 6, 0, 0, 6, 4, 0, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 7, 8, 5, 3, 0, 5, 3, 0, 5, 7, 8, 6, 0, 0, 6, 3, 0, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 7)
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/infant_sensitivity/uncertainty_lower/one.ppddl
    one_p = (1, 1, 1, 1, 1, 4, 2, 2, 2, 1, 6, 1, 1, 6, 4, 2, 6, 2, 8, 0, 0, 0, 0, 0, 0, 0, 7, 1, 1, 1, 1, 1, 3, 2, 2, 2, 1, 6, 1, 1, 6, 3, 2, 6, 2, 8, 0, 0, 0, 0, 0, 0, 0, 7)
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/infant_sensitivity/uncertainty_lower/all.ppddl
    all_p = (8, 3, 3, 0, 3, 4, 0, 1, 7, 8, 6, 0, 0, 6, 4, 0, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 7, 8, 3, 3, 0, 3, 3, 0, 1, 7, 8, 6, 0, 0, 6, 3, 0, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 7)

    return one_p, four_p, all_p

def likelihood_higher_i():
    # Changed order for ease of copy/pasting from output file, it is correct
    
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/infant_sensitivity/uncertainty_higher/four.ppddl
    four_p = (8, 0, 0, 1, 0, 4, 2, 0, 7, 8, 1, 1, 1, 1, 4, 2, 2, 7, 8, 0, 0, 0, 0, 4, 0, 0, 7, 8, 0, 0, 1, 0, 3, 2, 0, 7, 8, 1, 1, 1, 1, 3, 2, 2, 7, 8, 0, 0, 0, 0, 3, 0, 0, 7)
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/infant_sensitivity/uncertainty_higher/one.ppddl
    one_p = (1, 0, 0, 1, 0, 4, 2, 0, 0, 1, 6, 1, 1, 6, 4, 2, 6, 2, 8, 0, 0, 0, 0, 0, 0, 0, 7, 1, 0, 0, 1, 0, 3, 2, 0, 0, 1, 6, 1, 1, 6, 3, 2, 6, 2, 8, 0, 0, 0, 0, 0, 0, 0, 7)
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/infant_sensitivity/uncertainty_higher/all.ppddl
    all_p = (8, 0, 0, 1, 0, 4, 2, 0, 7, 8, 6, 1, 1, 6, 4, 2, 6, 7, 8, 0, 0, 0, 0, 4, 0, 0, 7, 8, 0, 0, 1, 0, 3, 2, 0, 7, 8, 6, 1, 1, 6, 3, 2, 6, 7, 8, 0, 0, 0, 0, 3, 0, 0, 7)

    return one_p, four_p, all_p

def added_wrong_precond_m():

    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine_sensitivity/added_wrong_precond/one.ppddl
    one_p = (5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2)
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine_sensitivity/added_wrong_precond/three.ppddl
    three_p = (5, 5, 3, 6, 5, 5, 3, 6, 0, 0, 3, 6, 0, 0, 3, 6, 5, 5, 3, 6, 5, 5, 3, 6, 0, 0, 0, 6, 0, 0, 0, 6, 0, 0, 3, 6, 0, 0, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 3, 6, 0, 0, 3, 6, 4, 4, 3, 6, 1, 1, 1, 6)
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine_sensitivity/added_wrong_precond/all.ppddl
    all_p = (0, 5, 0, 0, 0, 5, 0, 0, 0, 0, 3, 6, 0, 0, 3, 6, 0, 5, 0, 0, 0, 5, 0, 0, 0, 0, 3, 6, 0, 0, 3, 6, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 3, 6, 2, 2, 3, 6, 0, 0, 0, 0, 0, 0, 0, 0, 4, 4, 3, 6, 1, 1, 3, 6)
    

    return one_p, three_p, all_p

def missing_precond_m():

    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine_sensitivity/missing_precond/one.ppddl
    one_p = (5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2)
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine_sensitivity/missing_precond/three.ppddl
    three_p = (5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2)
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine_sensitivity/missing_precond/all.ppddl
    all_p =(5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2)

    return one_p, three_p, all_p


def reward_higher_m():

    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine_sensitivity/reward_higher/one.ppddl
    one_p = (5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0, 6, 0, 0, 0, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 1, 6, 2, 2, 2, 2, 2, 2, 2, 2)
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine_sensitivity/reward_higher/three.ppddl
    three_p = (5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0, 6, 0, 0, 0, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 1, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 4, 6, 1, 1, 1, 6, 2, 2, 2, 2, 2, 2, 2, 2)
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine_sensitivity/reward_higher/all.ppddl
    all_p = (5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 3, 6, 0, 0, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 1, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 1, 6, 2, 2, 2, 2, 2, 2, 2, 2)

    return one_p, three_p, all_p

def penalty_higher_m():

    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine_sensitivity/penalty_higher/one.ppddl
    one_p = (5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2)
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine_sensitivity/penalty_higher/three.ppddl
    three_p = (5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2)
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine_sensitivity/penalty_higher/all.ppddl
    all_p = (5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2)

    return one_p, three_p, all_p

def likelihood_lower_m():

    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine_sensitivity/likelihood_lower/one.ppddl
    one_p = (5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2)
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine_sensitivity/likelihood_lower/three.ppddl
    three_p = (5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 6, 2, 2, 2, 6, 5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 6, 2, 2, 2, 6, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 6, 2, 2, 2, 6, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 6, 2, 2, 2, 6)
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine_sensitivity/likelihood_lower/all.ppddl
    all_p = (5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2)

    return one_p, three_p, all_p

def likelihood_higher_m():

    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine_sensitivity/likelihood_higher/one.ppddl
    one_p = (5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 3, 6, 0, 0, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2)   
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine_sensitivity/likelihood_higher/three.ppddl
    three_p = (5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 3, 6, 0, 0, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 1, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 1, 6, 2, 2, 2, 2, 2, 2, 2, 2)
    #/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine_sensitivity/likelihood_higher/all.ppddl
    all_p = (5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 0, 5, 3, 6, 0, 0, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 3, 6, 1, 1, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2)

    return one_p, three_p, all_p


if __name__ == "__main__":

    #mdp_problem_final, final_p = marine()
    #one_p, half_p, all_p = added_wrong_precond_m()
    #one_p, half_p, all_p = missing_precond_m()
    #one_p, half_p, all_p = reward_higher_m()
    #one_p, half_p, all_p = penalty_higher_m()
    #one_p, half_p, all_p = likelihood_lower_m()
    #one_p, half_p, all_p = likelihood_higher_m()

    mdp_problem_final, final_p = infant()
    #one_p, half_p, all_p = added_wrong_precond_i()
    #one_p, half_p, all_p = missing_precond_i()
    #one_p, half_p, all_p = reward_higher_i()
    #one_p, half_p, all_p = penalty_higher_i()
    #one_p, half_p, all_p = likelihood_lower_i()
    one_p, half_p, all_p = likelihood_higher_i()



    f_reward, f_std_err = evaluate_mdp_policy_sensitivity(mdp_problem_final, final_p)
    o_reward, o_std_err = evaluate_mdp_policy_sensitivity(mdp_problem_final, one_p)
    h_reward, h_std_err = evaluate_mdp_policy_sensitivity(mdp_problem_final, half_p)
    a_reward, a_std_err = evaluate_mdp_policy_sensitivity(mdp_problem_final, all_p)

    print("Final reward, num differences: ", f_reward, ", ", num_actions_differ(final_p,final_p))
    print("Std err in reward: ", f_std_err)
    print("One_p reward, num differences: ", o_reward, ", ", num_actions_differ(final_p,one_p))
    print("Std err in reward: ", o_std_err)
    print("Half_p reward, num differences: ", h_reward, ", ", num_actions_differ(final_p,half_p))
    print("Std err in reward: ", h_std_err)
    print("All_p reward, num differences: ", a_reward, ", ", num_actions_differ(final_p,all_p))
    print("Std err in reward: ", a_std_err)







