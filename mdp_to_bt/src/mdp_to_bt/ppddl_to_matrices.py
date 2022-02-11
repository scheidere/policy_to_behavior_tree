# This file is part of pypddl-PDDLParser.

# pypddl-parser is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# pypddl-parser is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with pypddl-parser.  If not, see <http://www.gnu.org/licenses/>.

# Emily Scheide
# Use of pyppdl-parser + additions

# NOTE THIS FILE IS CURRENTLY BEING USED AS MAIN FILE 
# FOR RUNNING PPDDL TO SIMPLIFIED BEHAVIOR TREE METHOD
# (This is because of simplicity with pointing to the pddl files, but I will update this and create a main file)

from policy_to_bt import *
from simplify_bt import *

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

def dictproduct(dct):
    for t in product(*dct.values()):
        yield dict(zip(dct.keys(), t))

def getParamCombos(action):

    combo_list = []

    # Dict of param name keys, with possible value list as arg for each
    param_values_dict = {}
    for param in action.params:

        param_values_dict[param._name] = problem.objects[param.type]

    #print('param val dict', param_values_dict)

    # Get all combinations of param values in dictionaries, congregate in list
    combo_list = list(dictproduct(param_values_dict))

    return combo_list

def getStateList():

    # Get all states (valid and invalid)

    single_state = []
    for i in range(len(domain.predicates)):
        #print('Predicate is %s' % str(domain.predicates[i]))
        for variable_type in domain.types:
            #print(variable_type)
            if variable_type in str(domain.predicates[i]):
                #print('This predicate has variable type %s' % variable_type)
                for value in problem.objects[variable_type]:

                    state_sub_list = [str(domain.predicates[i].name),value,1]
                    single_state.append(state_sub_list)

    states = []
    for tup in list(itertools.product([0,1],repeat=len(single_state))):
        # tup = (0,1,0,0) for example, representing (False, True, False, False)
        #print(tup)

        # Distribute True/False options for each condition (predicate) in a state
        full_state = copy.deepcopy(single_state)
        for i in range(len(single_state)):
            full_state[i][-1] = tup[i]

        states.append(full_state)

    #print('states ', states)

    # Remove invalid states per constraints in domain
    if domain.constraints:
        states = removeInvalidStates(states)


    return states

def removeInvalidStates(state_list):

    # This would need to have more constraint types added to be entirely complete

    valid_states_list = []
    removal_indices = [] # Indices of all invalid states

    # For each constraint, remove states that do not satisfy 
    for i in range(len(domain.constraints)):
        constraint = domain.constraints[i]
        print('Constraint: ', constraint)
        print(constraint._name)
        print(constraint._literal._predicate._name)
        print(''.join(map(str, constraint._params)))

        # Check each state for failure, per kind of constraint (currently only at-most-once)
        for j in range(len(state_list)):
            state = state_list[j]
            print('state ', state)
            if constraint._name == 'at-most-once':
                # Find all terms in state with the literal that is in the constraint
                # Simplest case (one literal, one param, two param values): [literal,param value 1, 0],[literal,param value 2, 1]
                # Only 0 and 1, or 1 and 0 are allowed

                # Satisfy constraint: One 1, rest are 0s
                # Fail to satisfy: All zeros, or more than one 1
                num=None
                found_a_one = False
                for term in state:
                    if term[0] == constraint._literal._predicate._name:
                        if num==None:
                            num=term[-1] # 0 or 1
                            print('num ',num)
                            continue
                        elif num==1 and term[-1]==num: # Found another 1, so constraint is not satisfied
                            removal_indices.append(j)
                            print("More than one 1 found")
                            break
                        elif num==0:
                            if term[-1] == 1: # found a one
                                if not found_a_one:
                                    found_a_one = True # found first 1
                                else: # found second 1, constraint not satisfied
                                    removal_indices.append(j)
                                    print("More than one 1 found*")
                                    break


                # After searching whole state
                if num == 0 and not found_a_one: # If all zeros -> failure
                    removal_indices.append(j)
                    print('all zeros')
 


    # Only retain valid states
    for idx in range(len(state_list)):
        if idx not in removal_indices:
            # Then state at idx in list is valid
            valid_states_list.append(state_list[idx])

    return valid_states_list


def testRemoveInvalidStates():

    states = getStateList()

    for s in states:
        print(s)

    print('{{{{{{{{{{{{')

    valid = removeInvalidStates(states)
    for s in valid:
        print(s)


def getComboArgValues(args,combo_dict):

    # E.g. args = ['?x','?a','?b']

    values = []
    for arg in args:
        values.append(combo_dict[arg])

    return values



def preconditionSatisfied(combo_dict,start_state,action,test=False):

    # Check that parameter values in combo match start state per action precondition 
    for precond in action.precond:

        if precond._predicate.name != '=':
            match_found = False
            
            precond_args = precond._predicate.args
            if test:
                print(precond._predicate.name, precond_args)

            # Search start state for precondition predicate name where true
            for term in start_state:
                # True is defined by term[-1] == 1
                if term[0] == precond._predicate.name and term[-1] == 1: # Check robot-at is True

                    # Get param values in start state for current term (associated with given predicate)
                    term_arg_vals = term[1:-1]
                    if test:
                        print('term arg vals', term_arg_vals)

                    # Get parameter values to compare with from the param_combo given
                    combo_arg_vals = getComboArgValues(precond_args,combo_dict)

                    if test:
                        print('Comparing %s to %s' % (term_arg_vals,combo_arg_vals))

                    # Compare start state term with combo
                    if term_arg_vals != combo_arg_vals:
                        # Term does not match arg vals in give combo
                        if test:
                            print('Discrepancy found')
                        else:
                            pass
                    else:
                        if test:
                            print('Match found') # precondition is satisfied
                        match_found = True
                        continue

            if not match_found: # Precondition fails for given state and param combo
                if test:
                    print('Failure because no match found between state and param combo')
                return False

        # Check equation preconditions with combo parameter values
        elif precond._predicate.name == '=':
            args = precond._predicate.args

            if precond.is_positive(): # =

                if combo_dict[args[0]] != combo_dict[args[1]]:
                    if test:
                        print('Failure due to params not being equal...')
                    return False
            else: # !=

                if combo_dict[args[0]] == combo_dict[args[1]]:
                    if test:
                        print('Failure due to params being equal...')
                    return False


    return True

def equal(state1, state2):

    #need to check if states have all the same terms
    #even if the terms are not in the same order

    #returns True or False

    for i in range(len(state1)):

        term1 = state1[i]

        if term1 not in state2:
            #print(term1)
            #print(state2)
            return False

    return True


def testEqual():

    # State 1 and state 2 are actually the same, same terms and same order (Should return TRUE)
    state1 = [['robot-at', 'left-cell', 1], ['robot-at', 'right-cell', 1], ['dirty-at', 'left-cell', 1], ['dirty-at', 'right-cell', 1]]
    state2 = [['robot-at', 'left-cell', 1], ['robot-at', 'right-cell', 1], ['dirty-at', 'left-cell', 1], ['dirty-at', 'right-cell', 1]]
    print(equal(state1,state2))

    # States 3 and 4 are equal but the terms are in different orders (should return TRUE)
    state3 = [['robot-at', 'left-cell', 0], ['robot-at', 'right-cell', 1], ['dirty-at', 'left-cell', 1], ['dirty-at', 'right-cell', 0]]
    state4 = [['robot-at', 'right-cell', 1], ['dirty-at', 'right-cell', 0], ['dirty-at', 'left-cell', 1], ['robot-at', 'left-cell', 0]]
    print(equal(state3,state4))

    # States 5 and 6 are not equal, one term does not match (should return FALSE)
    state5 = [['robot-at', 'left-cell', 1], ['robot-at', 'right-cell', 1], ['dirty-at', 'left-cell', 1], ['dirty-at', 'right-cell', 1]]
    state6 = [['robot-at', 'left-cell', 1], ['robot-at', 'right-cell', 0], ['dirty-at', 'left-cell', 1], ['dirty-at', 'right-cell', 1]]
    print(equal(state5,state6))


def getStateIndex(state, states):

    for i in range(len(states)):

        if equal(states[i],state):

            #print("match")
            #print(states[i])
            #print(state)

            return i

def testGetStateIndex():

    states = getStateList()

    state = [['robot-at', 'right-cell', 1], ['dirty-at', 'right-cell', 0], ['dirty-at', 'left-cell', 1], ['robot-at', 'left-cell', 0]]

    print(getStateIndex(state,states))

def outcome(param_values, start_state, action, test=False):

    precond_satisfied = True

    # List of lists
    # Each sub list includes an outcome state with probability p and reward r, [state,p,r]
    outcome_list = []

    # Initialize state terms left over after given action is taken in the start state
    unchanged_state_terms = copy.deepcopy(start_state)

    if not preconditionSatisfied(param_values,start_state,action):

        outcome_sublist = [unchanged_state_terms,1.0,0]
        outcome_list.append(outcome_sublist)
        precond_satisfied = False

    else:

        replacement_state_terms = [] # [state,p,r]

        probabilistic = True

        literals = []
        for effect in action.effects:

            if effect[0] == 1.0: # Non-probabilistic effect
                # Initialize state terms left over after given action is taken in the start state
                #unchanged_state_terms = copy.deepcopy(start_state)

                #replacement_state_terms = [] # [state,p,r]
                prob = 1.0
                reward = 0
                #print('not probabilistic')
                #literals.append(effect[1])

                probabilistic = False

                predicate = effect[1]._predicate
                predicate_name = predicate.name
                params = predicate._args
                values = []
                for param in params:
                    values.append(param_values[param])
                
                # Search for relevant terms in start state, those that will be affected
                for i in range(len(start_state)):
                    term = start_state[i]

                    # Find term that will change due to effect of taking action in start state
                    if term[0] == predicate_name and term[1:-1] == values:

                        unchanged_state_terms.remove(term)

                        # Initialize term for end state, state that results from action occurance
                        new_term = copy.deepcopy(term[0:-1])

                        if effect[1].is_positive():
                            new_term.append(1)
                        else:
                            new_term.append(0)

                        replacement_state_terms.append(new_term)


            else: # Probabilistic effect

                for term in effect:

                    #print('term',term)
                    prob = term[0]
                    #print('prob: ', prob)
                    literals = []
                    for t1 in term[1:]:
                        #print('t1 ', t1)
                        if len(term) > 1:

                            # in case t1 = ('reward', 2) for e.g.
                            if t1[0] == 'reward':
                                reward = t1[1]
                                #print('reward t1 len > 1', reward)

                            else:
                                for t2 in t1:
                                    if isinstance(t2,Literal):
                                        #print('literal t2 ', t2)
                                        literal = t2
                                        literals.append(literal)
                                    elif t2:
                                        if t2[0] == 'reward':
                                            reward = t2[1]
                                            #print('reward t2', reward)
                        else:
                            if t1[0] == 'reward': #only contains a reward term, no state changes
                                reward = t1[1]
                                #print('reward t1 len = 1', reward)
                                #end_state = unchanged_state_terms # full start state
                                #outcome_sublist = outcome_sublist = [end_state,prob,reward]
                                #print('outcome_sublist 2', outcome_sublist)
                            else: # literals
                                #print('literal t1 ', t1)
                                literal = t1
                                literals.append(literal)

                    # print('PROB', prob)
                    # print('REWARD ', reward)
                    # print('literals ', literals)

                    
                    outcome_sublist = getOutcomeSublist(literals,prob,reward,start_state,param_values,is_probabilistic=True)
                    #print(outcome_sublist)
                    outcome_list.append(outcome_sublist)

        if not probabilistic:
            end_state = unchanged_state_terms + replacement_state_terms
            if equal(end_state,start_state):
                print('end is same as start state')
            outcome_sublist = [end_state,prob,reward]
            #print(outcome_sublist)
            #print('non probabilitistic outcome', outcome_sublist)
            ##getOutcomeSublist(literals, prob, reward, start_state, param_values,is_probabilistic=False)
            outcome_list.append(outcome_sublist)

        if test:
            print('Outcome list: ', outcome_list)


    return outcome_list, precond_satisfied

def getOutcomeSublist(literals, prob, reward, start_state, param_values, is_probabilistic=False):

    #print('agh lit', literals)

    if not literals: # No state changes, just prob and reward
        end_state = copy.deepcopy(start_state)
        outcome_sublist = [end_state,prob,reward]
        #print('outcome_sublist 1', outcome_sublist)
        return outcome_sublist

    if is_probabilistic:
        # Initialize state terms left over after given action is taken in the start state
        unchanged_state_terms = copy.deepcopy(start_state)

        replacement_state_terms = [] # [state,p,r]

    # [outcome_state, prob, reward]
    for literal in literals:
        predicate = literal._predicate
        #print('predicate', predicate)
        predicate_name = predicate.name
        params = predicate._args
        values = []
        for param in params:
            values.append(param_values[param])
    
        #print('values', values)
        # Search for relevant terms in start state, those that will be affected
        for i in range(len(start_state)):
            term = start_state[i]
            #print('start state term', term)

            # Find term that will change due to effect of taking action in start state
            if term[0] == predicate_name and term[1:-1] == values:

                #print('match')
                #print('unchanged_state_terms',unchanged_state_terms)

                unchanged_state_terms.remove(term)

                # Initialize term for end state, state that results from action occurance
                new_term = copy.deepcopy(term[0:-1])

                if literal.is_positive():
                    new_term.append(1)
                else:
                    new_term.append(0)

                replacement_state_terms.append(new_term)
                #print('replacement_state_terms', replacement_state_terms)

        end_state = replacement_state_terms + unchanged_state_terms
        outcome_sublist = [end_state,prob,reward]
        #print('outcome_sublist 2', outcome_sublist)

    return outcome_sublist

def updatePandR(outcome_list):

    # outcome_list, i.e. output of outcome()
    # each element is an outcome_sublist

    #for outcome_sublist in outcome_list:

        # [outcome_state,prob,reward]

        #??? might not want this, harder to update
        #actually should work on get index from state with terms in different order
        #so equal function and getStateIndex

    pass

def preconditionSatisfiedActionParams(action, combo_dict, test=True):

    #Need to remove things like move(left,left) and move(right,right) by checking preconditions

    #Might also be able to merge with preconditionSatisfied, but not sure what is most elegant option

    #use this in getPandR to ammend the actions_with_params list to only contain valid actions_with_params

    #then update loops so you only consider those actions for p and r

    # Check equation preconditions with combo parameter values
    for precond in action.precond:
        if precond._predicate.name == '=':
            args = precond._predicate.args

            if precond.is_positive(): # =

                if combo_dict[args[0]] != combo_dict[args[1]]:
                    if test:
                        print('Failure due to params not being equal...')
                    return False
            else: # !=

                if combo_dict[args[0]] == combo_dict[args[1]]:
                    if test:
                        print('Failure due to params being equal...')
                    return False

    return True

def getActionsWithParamsList():

    # Get all action/param combos

    # Initialize actions with parameters list (used for reading policy)
    actions_with_params = []

    # Loop through all actions in domain
    for action in domain.operators:

        # Get all possible combos of action parameter values
        param_combos = getParamCombos(action)

        for combo_dict in param_combos:

            # Make sure parameters pass the preconditions that can be checked at this stage
            # e.g. params == or != to each other or something
            if preconditionSatisfiedActionParams(action,combo_dict):

                #actions_with_params.append([action.name,combo_dict])
                actions_with_params.append([action,combo_dict])

    #print(actions_with_params)
    return actions_with_params


def getPandR():

    # Init main lists, NxN arrays will be added to (later converted to 3d numpy array)
    P,R = [],[]

    # Get valid states
    states = getStateList()

    # Initialize actions with parameters list (used for reading policy)
    actions_with_params = []

    # Number of states
    N = len(states)
    print('N: ',N)

    # Get valid action/param combos
    actions_with_params = getActionsWithParamsList()

    for action_with_params in actions_with_params:

        action, combo_dict = action_with_params

        # Init two NxN arrays with zeros (one for P_a and one for R_a)
        p, r = np.zeros((N,N)), np.zeros((N,N))

        # Loop through start states s, indexing with i
        for i in range(len(states)):
            start_state = states[i]

            # print('+++++++++++++++++++')
            # print('action ',action)
            # print('params ', combo_dict)
            # print('start state ', start_state)

            # Get list of [end state, prob, reward] terms, given action and start state
            # Also return only the actions_with_params that satisfy preconds
            outcome_list, precond_satisfied = outcome(combo_dict,start_state,action)
            #print("outcome_list: ",outcome_list)

            # if precond_satisfied: # i.e. list not empty
            #     print(action.name, combo_dict)
            #     actions_with_params.append([action.name,combo_dict])

            # Update NxN matrices, p and r according to outcome
            for outcome_sublist in outcome_list:

                #print('outcome_sublist', outcome_sublist)

                # Get index of end state where outcome_sublist = [end state, prob, reward]
                #print(outcome_sublist[0])
                j = getStateIndex(outcome_sublist[0],states)
                #print('j', j)

                p[j,i] = outcome_sublist[1]
                r[j,i] = outcome_sublist[2]
                
        # Add NxN matrices to lists P and R
        P.append(p)
        R.append(r)


    # # Loop through all actions in domain
    # for action in domain.operators:

    #     # Get all possible combos of action parameter values
    #     param_combos = getParamCombos(action)
    #     #print('param combos', param_combos)

    #     # For combo in possible combos, update P and R with NxN matrix for given action/param combo
    #     for combo_dict in param_combos:

    #         # Add info to readbility list (including invalid param combos e.g. move(left,left) which fails preconditions)
    #         #??? issue, only want action/param combos added to this if valid per precondtions
    #         actions_with_params.append([action.name,combo_dict])

    #         # Init two NxN arrays with zeros (one for P_a and one for R_a)
    #         p, r = np.zeros((N,N)), np.zeros((N,N))

    #         # Loop through start states s, indexing with i
    #         for i in range(len(states)):
    #             start_state = states[i]

    #             print('+++++++++++++++++++')
    #             print('action ',action)
    #             print('params ', combo_dict)
    #             print('start state ', start_state)

    #             # Get list of [end state, prob, reward] terms, given action and start state
    #             # Also return only the actions_with_params that satisfy preconds
    #             outcome_list, precond_satisfied = outcome(combo_dict,start_state,action)
    #             print("outcome_list: ",outcome_list)

    #             # if precond_satisfied: # i.e. list not empty
    #             #     print(action.name, combo_dict)
    #             #     actions_with_params.append([action.name,combo_dict])

    #             # Update NxN matrices, p and r according to outcome
    #             for outcome_sublist in outcome_list:

    #                 #print('outcome_sublist', outcome_sublist)

    #                 # Get index of end state where outcome_sublist = [end state, prob, reward]
    #                 #print(outcome_sublist[0])
    #                 j = getStateIndex(outcome_sublist[0],states)
    #                 #print('j', j)

    #                 p[j,i] = outcome_sublist[1]
    #                 r[j,i] = outcome_sublist[2]
                    
    #         # Add NxN matrices to lists P and R
    #         P.append(p)
    #         R.append(r)

    # Convert list of matrices to 3d numpy array
    P = np.dstack(P).transpose()
    R = np.dstack(R).transpose()
    actions = []
    for action in domain.operators:
        actions.append(action.name)
    #print('Actions: ', actions)
    #print('Param combos: ', param_combos)
    #print('Probability transition matrix P:')
    #print2DArraysFrom3DArray(P)
    # print('Reward matrix R:')
    #print2DArraysFrom3DArray(R)
    # print(P.shape)
    # print('R:\n',R)
    # print(R.shape)

    return P, R, states, actions_with_params

def test_outcome():

    states = getStateList()

    for action in domain.operators:

        combos = getParamCombos(action)

        for start_state in states:

            for combo_dict in combos:

                #if preconditionSatisfied(combo_dict,start_state,action): 
                    # ??? move precond satisfied to inside outcome function
                    # when precond not satisfied for certain action/param combo/state 
                    # you still want to add a 1 at i,j where i=j for the unchanged start state

                print('\n')
                print('Start state: ', start_state)
                print('Combo dict: ', combo_dict)
                print('Precondition satisfied? ', preconditionSatisfied(combo_dict,start_state,action))
                print('Action: ', action.name)
                outcome_list = outcome(combo_dict,start_state,action)
                for o in outcome_list:
                    print(o)

                print('\n')

def precondSatisfiedTest():

    states = getStateList()

    for action in domain.operators:
    #action = domain.operators[0]
    #print('Action: ', action)

        param_combos = getParamCombos(action)

        for i in range(len(states)):
            start_state = states[i]
            

            for combo_dict in param_combos:

                print('Action: ', action)

                print('Start: ', start_state)

                print('Param combo: ', combo_dict)

                print('Precond satisfied? %s\n' % preconditionSatisfied(combo_dict,start_state,action,test=True))


def solve(solver,P,R):

    # Get transition probabality and reward matrices from PPDDL

    if solver == 'v':
        val_it = mdptoolbox.mdp.ValueIteration(P, R, 0.96)
        val_it.run()
        policy = val_it.policy
        print('Policy: ', val_it.policy)
    elif solver == 'q':
        q_learn = mdptoolbox.mdp.QLearning(P, R, 0.96)
        q_learn.run()
        policy = q_learn.policy
        print('Policy: ', q_learn.policy)
    else:
        print("That is not a valid solver...")

    return policy

def readPolicy(policy,states,actions_with_params):

    print('Policy in readable form: ')

    for i in range(len(policy)):

        print(states[i])
        print(actions_with_params[policy[i]][0].name,actions_with_params[policy[i]][1],'\n')

if __name__ == '__main__':

    # Define domain and problem to consider (they represent an MDP)
    print('For the following domain and problem: ')
    args = parse()

    domain  = PDDLParser.parse(args.domain)
    problem = PDDLParser.parse(args.problem)

    print(domain)
    print(problem)

    # Convert to MDP, i.e. generate transition probability matrix P and reward matrix R
    print('The follow matrices represent the transition probabilities\n and rewards for all state transitions: ')
    P, R, states, actions_with_params = getPandR()

    print('P:\n', P, '\n',P.shape)
    print('R:\n', R, '\n',R.shape)

    print('actions_with_params: ')
    for action in actions_with_params:
        print(action)

    # Choose method of solving for a policy given P and R
    solver = input("Enter value iteration (v) or Q-learning (q): ") 

    # Solve for a policy
    policy = solve(solver,P,R)

    # Translate policy to readable form
    readPolicy(policy,states,actions_with_params)

    # Convert policy to behavior tree
    # p2bt = PolicyToBT(states, actions_with_params, policy)

    # # Save behavior tree in a file
    # p2bt.behavior_tree.write_config('output_config/raw_policy_output_bt.tree') # need to copy output_bt.tree to behavior_tree/src/behavior_tree/config/
    # p2bt.behavior_tree.write_config('../../../behavior_tree/config/raw_policy_output_bt.tree') # Needed here to show in rqt

    # # To visualize the behavior tree, navigate to the behavior tree package
    # # The output file will be here: your_workspace/src/policy_to_behavior_tree/behavior_tree/config
    # # Run 'roslaunch behavior_tree show_tree.launch'
    # # Change .tree file on line 45 of show_tree.py in behavior_tree/src/behavior_tree 

    # # Simplify the behavior tree via conflict (remove irrelevant conditions/decorators and combine same-action subtrees)
    # simplify = SimplifyBT(p2bt.behavior_tree)
    # final_bt = simplify.simplified_bt
    # final_bt.write_config('output_config/final_simplified_output_bt_SIMPLIFYTEST.tree')
    # final_bt.write_config('../../../behavior_tree/config/final_simplified_output_bt_SIMPLIFYTEST.tree')


    # #print('TESTING TESTING TESTING')
    # #testRemoveInvalidStates()

    # print('+++++++++++++++++++++++++++++++')
    # #precondSatisfiedTest() # this does show issue with move(left,left)
    # getActionsWithParamsList()
