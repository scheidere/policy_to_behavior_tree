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


import argparse

from pddlparser import PDDLParser
#from literal import Literal #used for isinstance()

import itertools
from itertools import product
import numpy as np
import copy

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

def preconditionSatisfied(combo_dict,start_state,action):

    # combo is combo dict

    #print('==========')

    #print('start state %s\n' % start_state)
    #print('combo %s\n' % combo_dict)

    # Check that parameter values in combo match start state per action precondition 
    for precond in action.precond:
        #print(precond)
        #print(precond._predicate.args)
        if precond._predicate.name != '=':
            #print('Precond name: %s' % precond._predicate.name)
            # Search state for name, e.g. 'robot-at'
            for term in start_state:
                if term[0] == precond._predicate.name and term[-1] == 1: # Check robot-at is True
                    #print(term)
                    # Determine if True for correct arg(s), e.g. ?x or ?x,?y
                    arg_terms = term[1:-1]
                    #print('arg_terms',arg_terms)
                    for i in range(len(precond._predicate.args)):
                        arg = precond._predicate.args[i]
                        #print(i,arg,combo_dict[arg],arg_terms[i])
                        if arg_terms[i] != combo_dict[arg]: 
                            print('Failure due to combo not matching start state given action preconditions')
                            return False

        # Check equation preconditions with combo parameter values
        elif precond._predicate.name == '=':
            args = precond._predicate.args
            #print('args: ', args)
            if precond.is_positive(): # =
                #print('pos')
                if combo_dict[args[0]] != combo_dict[args[1]]:
                    print('Failure due to params not being equal...')
                    return False
            else: # !=
                #print('neg')
                #print(args[0],args[1])
                if combo_dict[args[0]] == combo_dict[args[1]]:
                    print('Failure due to params being equal...')
                    return False


    return True

    
    #print('==========')

def outcomeIsEndState(param_values,start_state, end_state,action):

    #print('In outcomeIsEndState...')

    #print('combo_dict', param_values)

    # Done for 'move', does not include reward or probs
    outcome = []
    for effect in action.effects:

        outcome.append([effect[1]._predicate.name])
        params = effect[1]._predicate._args
        for param in params:
            #print(param)
            param_value = param_values[param]
            #print(param_value)
            outcome[-1].append(param_value)

        if effect[1].is_positive():
            outcome[-1].append(1)
        else:
            outcome[-1].append(0)

    #print('Outcome: ', outcome)

    # Check if outcome is satisfied by end state
    non_outcome_term_indices = list(range(len(end_state))) # will denote parts of state that should remain same
    for term in outcome:
        if term not in end_state:
            print('Could not find %s term\n' % term)
            return False
        else: # term is in end_state
            term_index = end_state.index(term)
            non_outcome_term_indices.remove(term_index)

    # At this point, the outcome is satisfied in the end state
    # However, the rest of the start state unaffected by the action, must remain the same
    # So now, check that non-outcome portion is the same in start and end states
    for index in non_outcome_term_indices:
        if start_state[index] != end_state[index]:
            print('State changed more than just from the action outcome\n')
            return False


    return True

    #print('End outcomeIsEndState')


def getPandR():

    # Init main lists, NxN arrays will be added to (later converted to 3d numpy array)
    P,R = [],[]

    # Get states
    states = getStateList()

    # Number of states
    N = len(states)
    print('N: ',N)

    # Loop through all actions in domain (first try with just action move)
    for action in [domain.operators[0]]:

        print('+++++++++++')
        print('Action: ', action)

        # Get all possible combos of action param values
        param_combos = getParamCombos(action)
        print('param combos', param_combos)
        #print(len(param_combos))

        ##preconditionSatisfied(action,param_combos[1],states[5]) NOT DONE

        # For combo in possible combos (ASSUMING ALL PRECOND SATISFIED ALREADY FOR NOW)
        for combo_dict in param_combos:

            # Init  two NxN arrays with zeros (one for P_a and one for R_a)
            p, r = np.zeros((N,N)), np.zeros((N,N))

            # if combo valid in precondition
            #preconditionSatisfied(action,combo)
            #if preconditionSatisfied(action, combo):

            # Loop through start states s, indexing with i
            for i in range(len(states)):
                start_state = states[i]

                # If precondition is satisfied for action(combo) in start state
                ##if preconditionSatisfied(action, combo, start_state): ASSUMING PRECOND SATISFIED FOR TESTING

                # Loop through end states s', indexing with j
                for j in range(len(states)):
                    end_state = states[j]
                    # If outcome matches s'
                    print('Start state: ', start_state)
                    print('Action: ', action.name)
                    print('End state: ', end_state)
                    if outcomeIsEndState(combo_dict,start_state,end_state,action):
                        # Valid transition, add probability (1 if not specified), add reward if any
                        print('Valid transition\n')

                    # Else not valid
                    #    Invalid transtion, probability remains 0 as initialized, add reward maybe (???)
        print('+++++++++++')

def outcomeIsEndStateTest():

    states = getStateList()

    # Success
    print('Success Scenario: ')
    start_state = states[5]
    print(start_state)
    end_state = states[9]
    print(end_state)

    action = domain.operators[0]
    print('Action: ', action)

    combos = getParamCombos(action)
    combo_dict = combos[2]
    print('Combo: ', combo_dict)

    print('Outcome: %s\n' % outcomeIsEndState(combo_dict,start_state,end_state,action))

    # Failure
    print('Failure Scenario 1: End state too different, not just changes from taking action in start state')
    start_state = states[5]
    print(start_state)
    end_state = states[8]
    print(end_state)

    action = domain.operators[0]
    print('Action: ', action)

    combos = getParamCombos(action)
    combo_dict = combos[2]
    print('Combo: ', combo_dict)

    print('Outcome: %s\n' % outcomeIsEndState(combo_dict,start_state,end_state,action))

    print('Failure Scenario 2: End state is not what happens when taking action in start state given')
    start_state = states[5]
    print(start_state)
    end_state = states[5]
    print(end_state)

    action = domain.operators[0]
    print('Action: ', action)

    combos = getParamCombos(action)
    combo_dict = combos[2]
    print('Combo: ', combo_dict)

    print('Outcome: %s\n' % outcomeIsEndState(combo_dict,start_state,end_state,action))


def precondSatisfiedTest():

    states = getStateList()

    # Satisfied test
    print('Success Scenario: ')
    start_state = states[5]
    print('Start: ', start_state)
    # end_state = states[9]
    # print('End: ', end_state)

    action = domain.operators[0]
    print('Action: ', action)

    combos = getParamCombos(action)
    combo_dict = combos[2]
    print('Param combo: ', combo_dict)

    print('Precond satisfied? %s\n' % preconditionSatisfied(combo_dict,start_state,action))

    # Not satisfied test (two ways to fail)

    # Combo matches start state but action preconditions are not satisfied (robot-at(?x) fails)
    print('Failure Scenario 1: Preconditions not satisfied (robot-at(?x) fails)')
    start_state = states[5]
    print('Start: ', start_state)

    action = domain.operators[0]
    print('Action: ', action)

    combos = getParamCombos(action)
    combo_dict = combos[1]
    print('Param combo: ', combo_dict)

    print('Precond satisfied? %s\n' % preconditionSatisfied(combo_dict,start_state,action))

    # Combo matches start state but action preconditions are not satisfied (?x != ?y fails)
    print('Failure Scenario 2: Preconditions not satisfied (?x != ?y fails)')
    start_state = states[5]
    print('Start: ', start_state)
    end_state = states[9]
    print('End: ', end_state)

    action = domain.operators[0]
    print('Action: ', action)

    combos = getParamCombos(action)
    combo_dict = combos[3]
    print('Param combo: ', combo_dict)

    print('Precond satisfied? %s\n' % preconditionSatisfied(combo_dict,start_state,action))





if __name__ == '__main__':
    args = parse()

    domain  = PDDLParser.parse(args.domain)
    problem = PDDLParser.parse(args.problem)

    print(domain)
    print(problem)

    ??? add preconditionSatisfied calls back into main getPandR function
    then add actual calls to functions that pull correct probabilities and rewards to add to matrices

    #getPandR()

    outcomeIsEndStateTest()

    #precondSatisfiedTest()
