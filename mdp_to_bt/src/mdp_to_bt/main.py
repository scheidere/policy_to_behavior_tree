#!/usr/bin/env python3
from ppddl_to_matrices import *
#from matrices_to_policy import solve
from matrices_to_policy import *
import matrices_to_policy
from policy_to_bt import *
from simplify import * # NEW WAY
from evaluate_mdp_policy import *

import mdptoolbox
import numpy as np
import mdptoolbox.example

import argparse
import sys
#sys.path.append('../../../pypddl-parser/pypddl-parser')
from pypddl_parser.pddlparser import PDDLParser
from pypddl_parser.literal import Literal #used for isinstance()

import itertools
from itertools import product
import numpy as np
import copy
import time
import datetime
import yaml


np.set_printoptions(threshold=sys.maxsize) # So you can see matrices without truncation

import pickle

def main(domain, problem, config):

    # f = open("/home/scheidee/Desktop/AURO_results/raw_policy.txt", "w+")

    test = False
    save_raw_policy_bt = True

    # Method components
    do_simplification = config['do_simplification'] # True for full method
    ignore_dontcares = config['ignore_dontcares'] # True for full method
    default_action_order = config['default_action_order'] # True for full method

    domain_name = domain.name

    start_time_ppddl_to_matrices = time.time()
    # Convert to MDP, i.e. generate transition probability matrix P and reward matrix R
    P, R, states, actions_with_params = getPandR(domain,problem)
    ppddl_to_matrices_runtime = time.time() - start_time_ppddl_to_matrices

    if test:
        # print('The follow matrices represent the transition probabilities\n and rewards for all state transitions: ')
        print(('P:\n', P, '\n'))
        print(('R:\n', R, '\n'))

        #print(P.shape)
        #return
        for i in range(P.shape[0]):
            for j in range(P.shape[1]):
                row = P[i][j]
                #print(row)
                check = sum(row)
                #print(check)
                if check != 1:
                    print(('oh no, sum is not 1!', check, i, j))

        #return

    # Set value iteration as our method of solving the MDP for a policy, denoted by 'v'
    solver = 'v' # Note that Q-learning has bugs in the MDPToolbox, hence sticking to value iteration

    if test:
        print((type(P), shape(P)))
        print((type(R), shape(R)))

    start_time_solver = time.time()
    # Solve for a policy (and also return updated action list, only including those actions that actually appear in the policy)
    policy = matrices_to_policy.solve(solver,P,R)
    solver_runtime = time.time() - start_time_solver
    f = open("/home/scheidee/Desktop/AURO_results/bla.txt", "w+")
    f.write(str(policy)+"\n")

    # Uncomment line below to (re)generate readable policy to a file (see saveReadablePolicy in ppddl_to_matrices.py for file location)
    saveReadablePolicy(policy,states,actions_with_params)

    
    if save_raw_policy_bt:
        # Convert policy to BT and save as
        # print('\n++++++++++++++++++\n')
        # print('Raw policy behavior tree:\n')
        p2bt = PolicyToBT(states, actions_with_params, policy)
        raw_policy_bt = p2bt.behavior_tree

    # Translate policy to readable form
    # choice = input('Press r to print policy in readable form. To skip press any other key.')
    choice = 'Nope'
    if choice == 'r':
        #readPolicy(policy,states,actions_with_params)
        checkPolicyPreconditions(policy,states,actions_with_params) #Value iteration passes, Q-learning fails

        eval(input('Scroll up to read the policy. Press return to simplify and evaluate.'))

    # Simplify the policy using Boolean logic and the resulting behavior tree in a .tree file
    if do_simplification:
        print('Simplified policy behavior tree:\n')
        test_time_start = time.time()
        simplify = Simplify(states, actions_with_params, policy, domain, problem, ignore_dontcares, default_action_order, f)
        simplified_policy_bt = simplify.bt
        simplification_runtime = simplify.simplification_runtime
        policy_to_bt_runtime = simplify.policy_to_bt_runtime
        test_runtime = time.time()-test_time_start
    else:
        print('SKIPPING SIMPLIFICATION!')
        simplification_runtime = None
        policy_to_bt_runtime = None
        test_runtime = None
        #input('hi')

    # Save the behavior trees in .tree files in behavior_tree/config
    print('Saving behavior trees to files...\n')
    if save_raw_policy_bt:
        #raw_policy_bt.write_config('/home/parallels/auro_ws/src/policy_to_behavior_tree/behavior_tree/config/AURO_final_synthesized_BTs/raw_policy_bt.tree')
        raw_policy_bt.write_config('/home/scheidee/auro_ws/src/policy_to_behavior_tree/behavior_tree/config/AURO_final_synthesized_BTs/raw_policy_bt_hi.tree')
    if do_simplification:
        #simplified_policy_bt.write_config('/home/parallels/auro_ws/src/policy_to_behavior_tree/behavior_tree/config/AURO_final_synthesized_BTs/final_synth_bt.tree')
        simplified_policy_bt.write_config('/home/scheidee/auro_ws/src/policy_to_behavior_tree/behavior_tree/config/AURO_final_synthesized_BTs/final_synth_bt_hi.tree')
    else:
        print('SKIPPING SAVE OF SIMPLIFIED POLICY WHILE GENERATING RESULTS')
        raw_policy_bt.evaluate_bt_compactness()
        #input('hi')

    evaluate_for_reward = False

    mdp_problem = None
    if evaluate_for_reward:
        # Evaluate the policy (simplified policy is equivalent, by definition)
        mdp_problem = MDP_Problem(P, R, states, actions_with_params)
        # if domain == prob_domain:
        #     prob_mdp_problem = mdp_problem # to use in fairly evaluating policies
        reward = evaluate_mdp_policy(mdp_problem, policy)
        print(("\nReward: %f\n" % reward))

    pickle_start = time.time()
    # Desktop
    #path = "/home/scheidee/new_bt_generation_ws/src/bt_generation/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/"
    # Laptop (Mac OS)
    #path = "/home/scheidee/bt_synthesis_ws/src/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/"
    #path = "/home/parallels/auro_ws/src/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/"
    path = "/home/scheidee/auro_ws/src/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/"
    pickle.dump(policy, open( path + "policy.p", "wb" ) )
    pickle_runtime = time.time() - pickle_start
    print(('pickle runtime: ', pickle_runtime))

    # Only save mdp problem for probabilistic world, 
    # which will be used in evaluate_policy.py to compare probabilistic and deterministic policies
    if 'deterministic' not in domain_name and mdp_problem:
        pickle.dump(mdp_problem, open( path + "mdp_problem.p", "wb" ) )

    return mdp_problem, policy, ppddl_to_matrices_runtime, solver_runtime, simplification_runtime, policy_to_bt_runtime, test_runtime # TO BE REMOVED


def run():

    rospy.init_node('mdp_to_bt')
    config_filename = rospy.get_param('~config')
    print('config_filename', config_filename)
    garbage_string = "_parameters.yaml"
    if garbage_string in config_filename:
        current_method = config_filename.replace(garbage_string, '')

    now = datetime.datetime.now()
    start_time_milli = int(time.time()*1000) #milliseconds

    # Create output file
    #f = open("/home/parallels/Desktop/AURO_results/bla.txt", "w+")
    f = open("/home/scheidee/Desktop/AURO_results/bla.txt", "w+")

    # Get the domain and problem
    domain_path = rospy.get_param('~domain')
    problem_path = rospy.get_param('~problem')
    domain  = PDDLParser.parse(domain_path)
    problem = PDDLParser.parse(problem_path)

    # Get config file
    rospack = rospkg.RosPack()
    filepath = rospack.get_path('mdp_to_bt') + "/config/" + rospy.get_param('~config')
    with open(filepath, 'r') as stream:
        config = yaml.safe_load(stream)

    mdp_problem, policy, ppddl_to_matrices_runtime, solver_runtime, simplification_runtime, policy_to_bt_runtime, test_runtime = main(domain, problem, config)


if __name__ == '__main__':

    run()

    # rospy.init_node('mdp_to_bt')
    # config_filename = rospy.get_param('~config')
    # print('config_filename', config_filename)
    # garbage_string = "_parameters.yaml"
    # if garbage_string in config_filename:
    #     current_method = config_filename.replace(garbage_string, '')

    # start_time = time.time()

    # # This method can (in theory) be run with or without ros
    # # The non-ros version is deprecated
    # # The most updated version is setup to run with "roslaunch main.launch config:=final"
    # # You must change the variable below, depending on which method you would like to use
    # use_ros = True 

    # # Define domain and problem to consider (they represent an MDP)
    # #print('\nFor the following domain and problem: \n\n')
    # if use_ros:
    #     domain_path = rospy.get_param('~domain')
    #     problem_path = rospy.get_param('~problem')

    # else:
    #     ## If running with 'python3 main.py <DOMAIN> <INSTANCE>'
    #     ## Domain refers to domain path, instance refers to problem path
    #     # This is deprecated, do not use
    #     print('i am here')
    #     args = parse()
    #     domain_path = args.domain
    #     problem_path = args.problem

    # domain  = PDDLParser.parse(domain_path)
    # problem = PDDLParser.parse(problem_path)

    # print(('Solving ', domain.name, '...'))

    # mdp_problem, policy, ppddl_to_matrices_runtime, solver_runtime, simplification_runtime, policy_to_bt_runtime, test_runtime = main(domain, problem)
