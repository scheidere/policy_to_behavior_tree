from ppddl_to_matrices import *
#from matrices_to_policy import solve
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


def parse():
    usage = 'python3 main.py <DOMAIN> <INSTANCE>'
    description = 'pypddl-parser is a PDDL parser built on top of ply.'
    parser = argparse.ArgumentParser(usage=usage, description=description)

    parser.add_argument('domain',  type=str, help='path to PDDL domain file')
    parser.add_argument('problem', type=str, help='path to PDDL problem file')

    return parser.parse_args()

def main(domain, problem):

    # Convert to MDP, i.e. generate transition probability matrix P and reward matrix R
    print('The follow matrices represent the transition probabilities\n and rewards for all state transitions: ')
    P, R, states, actions_with_params = getPandR(domain,problem)

    print('P:\n', P, '\n')
    print('R:\n', R, '\n')

    # print('actions_with_params: ')
    # for action in actions_with_params:
    #     print(action)

    # Set value iteration as our method of solving the MDP for a policy, denoted by 'v'
    solver = 'v' # Note that Q-learning has bugs in the MDPToolbox, hence sticking to value iteration

    print(type(P), shape(P))
    print(type(R), shape(R))

    input('wait')

    # Solve for a policy
    policy = matrices_to_policy.solve(solver,P,R)
    #policy = solve(solver,P,R)
    # if domain == prob_domain:
    #     print('Probabilistic')
    #     prob_policy = policy
    # elif domain == det_domain:
    #     print('Deterministic')
    #     det_policy = policy

    input('wait2')

    # Convert policy to BT and save as
    print('\n++++++++++++++++++\n')
    print('Raw policy behavior tree:\n')
    p2bt = PolicyToBT(states, actions_with_params, policy)
    raw_policy_bt = p2bt.behavior_tree

    # Translate policy to readable form
    # choice = input('Press r to print policy in readable form. To skip press any other key.')
    choice = 'nope'
    if choice == 'r':
        #readPolicy(policy,states,actions_with_params)
        checkPolicyPreconditions(policy,states,actions_with_params) #Value iteration passes, Q-learning fails

        input('Scroll up to read the policy. Press return to simplify and evaluate.')

    # print('Simplifying policy...')
    # print('old policy', policy)
    # Simplify the policy using Boolean logic and the resulting behavior tree in a .tree file
    print('Simplified policy behavior tree:\n')
    print("COMMENTED OUT SIMPLIFICATION BC OF MARINE BUG with domain.ppddl")
    # simplify = Simplify(states, actions_with_params, policy, domain, problem)
    # simplified_policy_bt = simplify.bt

    # Save the behavior trees in .tree files in behavior_tree/config
    print('Saving behavior trees to files...\n')
    raw_policy_bt.write_config('../../../behavior_tree/config/raw_policy_bt.tree')
    # simplified_policy_bt.write_config('../../../behavior_tree/config/simplified_bt.tree')
    
    # Evaluate the policy (simplified policy is equivalent, by definition)
    mdp_problem = MDP_Problem(P, R, states, actions_with_params)
    # if domain == prob_domain:
    #     prob_mdp_problem = mdp_problem # to use in fairly evaluating policies
    reward = evaluate_mdp_policy(mdp_problem, policy)
    print("\nReward: %f\n" % reward)


    # print('Now lets compare performance of the policies learned from the deterministc vs the probabilistic domains')
    # # print('Probabilistic: ', prob_policy)
    # # print('Deterministic: ', det_policy)
    # num_trials = 100
    # getAverageRewardInSameWorld(prob_mdp_problem, prob_policy, det_policy)

    #input('If you want to get the average reward, press return.')
    # Get average reward over a certain number of runs
    #num_trials = 100
    #avg_reward = get_average_reward(num_trials, mdp_problem, policy)
    #print('\nAverage reward over %d trials: %f' % (num_trials, avg_reward))
    return mdp_problem # TO BE REMOVED

def test(domain):

    print('test')
    print(domain)



if __name__ == '__main__':

    # Define domain and problem to consider (they represent an MDP)
    print('\nFor the following domain and problem: \n\n')
    args = parse()
    domain  = PDDLParser.parse(args.domain)
    problem = PDDLParser.parse(args.problem)

    print(domain)

    main(domain, problem)