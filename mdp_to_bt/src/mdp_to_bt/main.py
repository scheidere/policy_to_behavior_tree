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

    test = False

    domain_name = domain.name

    # Convert to MDP, i.e. generate transition probability matrix P and reward matrix R
    P, R, states, actions_with_params = getPandR(domain,problem)

    # print('actions_with_params', actions_with_params)

    if test:
        # print('The follow matrices represent the transition probabilities\n and rewards for all state transitions: ')
        # print('P:\n', P, '\n')
        # print('R:\n', R, '\n')

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

    # Solve for a policy (and also return updated action list, only including those actions that actually appear in the policy)
    policy = matrices_to_policy.solve(solver,P,R)

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
    simplify = Simplify(states, actions_with_params, policy, domain, problem)
    simplified_policy_bt = simplify.bt

    # Save the behavior trees in .tree files in behavior_tree/config
    print('Saving behavior trees to files...\n')
    raw_policy_bt.write_config('../../../behavior_tree/config/raw_policy_bt.tree')
    simplified_policy_bt.write_config('../../../behavior_tree/config/simplified_bt.tree')
    
    # Evaluate the policy (simplified policy is equivalent, by definition)
    mdp_problem = MDP_Problem(P, R, states, actions_with_params)
    # if domain == prob_domain:
    #     prob_mdp_problem = mdp_problem # to use in fairly evaluating policies
    reward = evaluate_mdp_policy(mdp_problem, policy)
    print("\nReward: %f\n" % reward)

    # Desktop
    #path = "/home/scheidee/new_bt_generation_ws/src/bt_generation/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/"
    # Laptop
    path = "/home/scheidee/bt_synthesis_ws/src/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/"
    pickle.dump(policy, open( path + "policy.p", "wb" ) )

    # Only save mdp problem for probabilistic world, 
    # which will be used in evaluate_policy.py to compare probabilistic and deterministic policies
    if 'deterministic' not in domain_name:
        pickle.dump(mdp_problem, open( path + "mdp_problem.p", "wb" ) )

    return mdp_problem, policy # TO BE REMOVED


if __name__ == '__main__':

    # Define domain and problem to consider (they represent an MDP)
    #print('\nFor the following domain and problem: \n\n')
    args = parse()
    domain  = PDDLParser.parse(args.domain)
    problem = PDDLParser.parse(args.problem)

    print(domain)
    print(problem)
    #input('wait to look at domain and problem')

    print('Solving ', domain.name, '...')

    mdp_problem, policy = main(domain, problem)

