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
# Adding this file Nov 2021 to do an intial pass at translating a ppddl parsed object
# to mdp probability transition and reward matrices, P and R

# This file uses the parser, but is not a part of it

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

    return states

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

    # param_values is a dict that represents some of the info in the start state
    # This function should only be called when param_values and start_state pass preconditionSatisified

    # List of lists
    # Each sub list includes an outcome state with probability p and reward r, [state,p,r]
    outcome_list = []

    # Initialize state terms left over after given action is taken in the start state
    unchanged_state_terms = copy.deepcopy(start_state)

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
            print('not probabilistic')
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


    return outcome_list

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



def getPandR():

    # Init main lists, NxN arrays will be added to (later converted to 3d numpy array)
    P,R = [],[]

    # Get states
    states = getStateList()

    # Number of states
    N = len(states)
    print('N: ',N)

    # Loop through all actions in domain (first try with just action move)
    for action in domain.operators:

        # Get all possible combos of action parameter values
        param_combos = getParamCombos(action)
        #print('param combos', param_combos)
        

        # For combo in possible combos, update P and R with NxN matrix for given action/param combo
        for combo_dict in param_combos:

            # Init two NxN arrays with zeros (one for P_a and one for R_a)
            p, r = np.zeros((N,N)), np.zeros((N,N))

            # Loop through start states s, indexing with i
            for i in range(len(states)):
                start_state = states[i]

                if preconditionSatisfied(combo_dict,start_state,action):

                    #print('Start state: ', start_state)
                    #print('Combo dict: ', combo_dict)
                    #print('Action: ', action.name)
                    outcome_list = outcome(combo_dict,start_state,action)
                    #print('Outcome list: ', outcome_list)

                    # Update NxN matrices, p and r according to outcome
                    for outcome_sublist in outcome_list:

                        #print('outcome_sublist', outcome_sublist)

                        # Get index of end state where outcome_sublist = [end state, prob, reward]
                        #print(outcome_sublist[0])
                        j = getStateIndex(outcome_sublist[0],states)
                        #print('j', j)

                        p[i,j] = outcome_sublist[1]
                        r[i,j] = outcome_sublist[2]
                        



            # Add NxN matrices to lists P and R
            P.append(p)
            R.append(r)

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

    return P, R, states, actions, param_combos


def print2DArraysFrom3DArray(array):

    for i in range(array.shape[2]):
        N = array.shape[0] # should be same for 0 or 1
        print(array[:N,:N,i])


def test_outcome():

    states = getStateList()

    for action in domain.operators:

        combos = getParamCombos(action)

        for start_state in states:

            for combo_dict in combos:

                if preconditionSatisfied(combo_dict,start_state,action): 
                    ??? move precond satisfied to inside outcome function
                    when precond not satisfied for certain action/param combo/state 
                    you still want to add a 1 at i,j where i=j for the unchanged start state

                    print('\n')
                    print('Start state: ', start_state)
                    print('Combo dict: ', combo_dict)
                    print('Action: ', action.name)
                    outcome_list = outcome(combo_dict,start_state,action)
                    for o in outcome_list:
                        print(o)

                    print('\n')

def precondSatisfiedTest():

    states = getStateList()

    action = domain.operators[0]
    print('Action: ', action)

    param_combos = getParamCombos(action)

    for i in range(len(states)):
        start_state = states[i]
        print('Start: ', start_state)

        for combo_dict in param_combos:

            print('Param combo: ', combo_dict)

            print('Precond satisfied? %s\n' % preconditionSatisfied(combo_dict,start_state,action,test=True))


def solve(solver,P,R):

    # Get transition probabality and reward matrices from PPDDL


    if solver == 'v':
        val_it = mdptoolbox.mdp.ValueIteration(P, R, 0.96)
        val_it.run()
        print('Policy: ', val_it.policy)
    elif solver == 'q':
        q_learn = mdptoolbox.mdp.QLearning(P, R, 0.96)
        q_learn.run()
        print('Policy: ', q_learn.policy)
    else:
        print("That is not a valid solver...")   
        

if __name__ == '__main__':

    # Define domain and problem to consider (they represent an MDP)
    print('For the following domain and problem: ')
    args = parse()

    domain  = PDDLParser.parse(args.domain)
    problem = PDDLParser.parse(args.problem)

    print(domain)
    print(problem)

    # # Convert to MDP, i.e. generate transition probability matrix P and reward matrix R
    # print('The follow matrices represent the transition probabilities\n and rewards for all state transitions: ')
    # P, R, states, actions, param_combos = getPandR()

    # print('P:\n', P, '\n',P.shape)
    # print('R:\n', R, '\n',R.shape)

    # # Choose method of solving for a policy given P and R
    # solver = input("Enter value iteration (v) or Q-learning (q): ") 

    # # Solve for a policy
    # solve(solver,P,R)


    test_outcome()


  
