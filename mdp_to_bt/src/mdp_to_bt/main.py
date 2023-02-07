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
sys.path.append('../../../pypddl-parser/pypddl-parser')
from pddlparser import PDDLParser
from literal import Literal #used for isinstance()

import itertools
from itertools import product
import numpy as np
import copy
import time

np.set_printoptions(threshold=sys.maxsize) # So you can see matrices without truncation

import pickle

def parse():
    usage = 'python3 main.py <DOMAIN> <INSTANCE>'
    description = 'pypddl-parser is a PDDL parser built on top of ply.'
    parser = argparse.ArgumentParser(usage=usage, description=description)

    parser.add_argument('domain',  type=str, help='path to PDDL domain file')
    parser.add_argument('problem', type=str, help='path to PDDL problem file')

    return parser.parse_args()

def main(domain, problem):

    test = False # Making True breaks something

    domain_name = domain.name

    start_time_ppddl_to_matrices = time.time()
    # Convert to MDP, i.e. generate transition probability matrix P and reward matrix R
    P, R, states, actions_with_params = getPandR(domain,problem)
    ppddl_to_matrices_runtime = time.time() - start_time_ppddl_to_matrices

    # print('actions_with_params', actions_with_params)

    if test:
        # print('The follow matrices represent the transition probabilities\n and rewards for all state transitions: ')
        print('P:\n', P, '\n')
        print('R:\n', R, '\n')

        #print(P.shape)
        #return
        for i in range(P.shape[0]):
            for j in range(P.shape[1]):
                row = P[i][j]
                #print(row)
                check = sum(row)
                #print(check)
                if check != 1:
                    print('oh no, sum is not 1!', check, i, j)

        return

    # Set value iteration as our method of solving the MDP for a policy, denoted by 'v'
    solver = 'v' # Note that Q-learning has bugs in the MDPToolbox, hence sticking to value iteration

    if test:
        print(type(P), shape(P))
        print(type(R), shape(R))

    start_time_solver = time.time()
    # print('time before solver: ', start_time_solver)
    # Solve for a policy (and also return updated action list, only including those actions that actually appear in the policy)
    policy = matrices_to_policy.solve(solver,P,R)
    solver_runtime = time.time() - start_time_solver
    # print('time after solver: ', time.time())
    # print('solver runtime: ', solver_runtime)
    # input('hiiiii')

    save_raw_policy_bt = False
    if save_raw_policy_bt:
        # Convert policy to BT and save as
        # print('\n++++++++++++++++++\n')
        # print('Raw policy behavior tree:\n')
        p2bt = PolicyToBT(states, actions_with_params, policy)
        raw_policy_bt = p2bt.behavior_tree

    # Translate policy to readable form
    # choice = input('Press r to print policy in readable form. To skip press any other key.')
    choice = 'nope'
    if choice == 'r':
        #readPolicy(policy,states,actions_with_params)
        checkPolicyPreconditions(policy,states,actions_with_params) #Value iteration passes, Q-learning fails

        input('Scroll up to read the policy. Press return to simplify and evaluate.')

    # Simplify the policy using Boolean logic and the resulting behavior tree in a .tree file
    print('Simplified policy behavior tree:\n')
    test_time_start = time.time()
    simplify = Simplify(states, actions_with_params, policy, domain, problem)
    simplified_policy_bt = simplify.bt
    simplification_runtime = simplify.simplification_runtime
    policy_to_bt_runtime = simplify.policy_to_bt_runtime
    test_runtime = time.time()-test_time_start
    # print('SIMPLIFICATION HAS BEEN COMMENTED OUT TO SAVE TIME')
    # print('Policy is equivalent, simplification result not used for plotting results')

    # Save the behavior trees in .tree files in behavior_tree/config
    print('Saving behavior trees to files...\n')
    if save_raw_policy_bt:
        raw_policy_bt.write_config('../../../behavior_tree/config/final_synthesized_BTs/raw_policy_bt.tree')
    #print('SKIPPING SAVE OF SIMPLIFIED POLICY WHILE GENERATING RESULTS')
    #simplified_policy_bt.write_config('../../../behavior_tree/config/final_synthesized_BTs/infant/simplified_bt_final.tree')
    # simplified_policy_bt.write_config('../../../behavior_tree/config/final_synthesized_BTs/marine/simplified_bt_final.tree')
    simplified_policy_bt.write_config('../../../behavior_tree/config/final_synthesized_BTs/final_synth_cdrc_music_started_test2.tree')

    evaluate_for_reward = False

    mdp_problem = None
    if evaluate_for_reward:
        # Evaluate the policy (simplified policy is equivalent, by definition)
        mdp_problem = MDP_Problem(P, R, states, actions_with_params)
        # if domain == prob_domain:
        #     prob_mdp_problem = mdp_problem # to use in fairly evaluating policies
        reward = evaluate_mdp_policy(mdp_problem, policy)
        print("\nReward: %f\n" % reward)

    # Desktop
    #path = "/home/scheidee/new_bt_generation_ws/src/bt_generation/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/"
    # Laptop
    pickle_start = time.time()
    path = "/home/scheidee/bt_synthesis_ws/src/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/"
    pickle.dump(policy, open( path + "policy.p", "wb" ) )
    pickle_runtime = time.time() - pickle_start
    print('pickle runtime: ', pickle_runtime)

    # Only save mdp problem for probabilistic world, 
    # which will be used in evaluate_policy.py to compare probabilistic and deterministic policies
    if 'deterministic' not in domain_name and mdp_problem:
        pickle.dump(mdp_problem, open( path + "mdp_problem.p", "wb" ) )

    return mdp_problem, policy, ppddl_to_matrices_runtime, solver_runtime, simplification_runtime, policy_to_bt_runtime, test_runtime # TO BE REMOVED


if __name__ == '__main__':

    start_time = time.time()

    # Define domain and problem to consider (they represent an MDP)
    #print('\nFor the following domain and problem: \n\n')
    args = parse()
    domain  = PDDLParser.parse(args.domain)
    problem = PDDLParser.parse(args.problem)

    print(domain)
    print(problem)
    #input('wait to look at domain and problem')

    print('Solving ', domain.name, '...')

    mdp_problem, policy, ppddl_to_matrices_runtime, solver_runtime, simplification_runtime, policy_to_bt_runtime, test_runtime = main(domain, problem)

    # time_elapsed = time.time() - start_time
    # print("Total time elapsed: ", time_elapsed)
    # print("PPDDL to matrices runtime percentage: ", ppddl_to_matrices_runtime/time_elapsed)
    # print("Solver runtime percentage: ", solver_runtime/time_elapsed)
    # print("Simplification runtime percentage: ", simplification_runtime/time_elapsed)
    # print("Policy to BT runtime percentage: ", policy_to_bt_runtime/time_elapsed)
    # print('Test runtime percentage: ', test_runtime/time_elapsed, test_runtime)

    # f = open('runtime_results.txt','a')
    # t_str = "Total time elapsed: " + str(time_elapsed) + "\n"
    # f.write(t_str)
    # # Alg. 2: PPDDL to MDP
    # ppddl_to_mdp_str1 = "PPDDL to matrices runtime: " + str(ppddl_to_matrices_runtime) + "\n"
    # ppddl_to_mdp_str2 = "PPDDL to matrices runtime percentage: " + str(ppddl_to_matrices_runtime/time_elapsed) + "\n"
    # f.write(ppddl_to_mdp_str1)
    # f.write(ppddl_to_mdp_str2)
    # # Solver
    # solver_str1 = "Solver runtime: " + str(solver_runtime) + "\n"
    # solver_str2 = "Solver runtime percentage: " + str(solver_runtime/time_elapsed) + "\n"
    # f.write(solver_str1)
    # f.write(solver_str2)
    # # Alg. 3: Simplify policy
    # simp_str1 = "Simplification runtime: " + str(simplification_runtime) + "\n"
    # simp_str2 = "Simplification runtime percentage: " + str(simplification_runtime/time_elapsed) + "\n"
    # f.write(simp_str1)
    # f.write(simp_str2)
    # # Alg. 4: Policy to BT
    # polbt_str1 = "Policy to BT runtime: " + str(policy_to_bt_runtime) + "\n"
    # polbt_str2 = "Policy to BT runtime percentage: " + str(policy_to_bt_runtime/time_elapsed) + "\n"
    # f.write(polbt_str1)
    # f.write(polbt_str2)

    # # These are runtime percentages being saved
    # f1 = open('ppddl_to_mdp_runtimes.txt', 'a')
    # bla1 = str(ppddl_to_matrices_runtime/time_elapsed) + "\n"
    # f1.write(bla1)
    # f2 = open('solver_runtimes.txt', 'a')
    # bla2 = str(solver_runtime/time_elapsed) + "\n"
    # f2.write(bla2)
    # f3 = open('simplification_runtimes.txt','a')
    # bla3 = str(simplification_runtime/time_elapsed) + "\n"
    # f3.write(bla3)
    # f4 = open('policy_to_bt_runtimes.txt','a')
    # bla4 = str(policy_to_bt_runtime/time_elapsed) + "\n"
    # f4.write(bla4)