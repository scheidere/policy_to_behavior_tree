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
from literal import Literal #used for isinstance()

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

    pass

def getStateIndex(state, states):

    for i in range(len(states)):

        if equal(states[i],state):

            return i

def outcome(param_values, start_state, action, test=False):

    # param_values is a dict that represents some of the info in the start state
    # This function should only be called when param_values and start_state pass preconditionSatisified

    # List of lists
    # Each sub list includes an outcome state with probability p and reward r, [state,p,r]
    outcome_list = []

    # Initialize state terms left over after given action is taken in the start state
    unchanged_state_terms = copy.deepcopy(start_state)

    replacement_state_terms = [] # [state,p,r]

    reward = 0
    prob = 0
    for effect in action.effects:

        if effect[0] == 1.0: # Non-probabilistic effect
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
                print(term)
                prob = term[0]
                print('prob: ', prob)
                for t1 in term[1:]:
                    print('t1 ', t1)
                    if len(t1) > 1:

                        # in case t1 = ('reward', 2) for e.g.
                        if t1[0] == 'reward':
                            reward = t1[1]
                            print('reward t1', reward)

                        else:
                            for t2 in t1:
                                if isinstance(t2,Literal):
                                    print('literal t2 ', t2)
                                elif t2:
                                    if t2[0] == 'reward':
                                        reward = t2[1]
                                        print('reward t2', reward)
                    else:
                        if t1[0] == 'reward':
                            reward = t2[1]
                            print('reward ', reward)



    #print('replacement_state_terms ', replacement_state_terms)
    #print('unchanged_state_terms ', unchanged_state_terms)

    end_state = replacement_state_terms + unchanged_state_terms
    outcome_sublist = [end_state,1.0,0]
    print('outcome_sublist ', outcome_sublist)





    return outcome_list

def test_outcome():

    states = getStateList()

    # print('Scenario 1:')
    # start_state = states[5]
    # print('start state ', start_state)
    # end_state = states[9]
    # print('expected end state ', end_state)

    # action = domain.operators[0]
    # print('Action: ', action)

    # combos = getParamCombos(action)
    # combo_dict = combos[2]
    # print('Combo: ', combo_dict)

    # outcome(combo_dict,start_state,action)

    print('Scenario 2:')
    action = domain.operators[0]
    print('Action: ', action)
    combos = getParamCombos(action)
    for start_state in states:

        for combo_dict in combos:

            if preconditionSatisfied(combo_dict,start_state,action):

                print('\n')
                print('Start state: ', start_state)
                print('Combo dict: ', combo_dict)
                print('Action: ', action.name)
                #??? add more prints, combo, action, etc?
                outcome(combo_dict,start_state,action)

                print('\n')


    #outcome(combo_dict,start_state,action)

def outcomeIsEndState(param_values,start_state, end_state,action,test=False):

    #print('In outcomeIsEndState...')

    #print('combo_dict', param_values)

    # Done for 'move', does not include reward or probs
    state_terms_changed = []
    for effect in action.effects:

        if effect[0] == 1.0: # Non-probabilistic effect
            #print('Non-probabilistic effect')
            state_terms_changed.append([effect[1]._predicate.name])
            params = effect[1]._predicate._args
            for param in params:
                #print(param)
                param_value = param_values[param]
                #print(param_value)
                state_terms_changed[-1].append(param_value)

            if effect[1].is_positive():
                state_terms_changed[-1].append(1)
            else:
                state_terms_changed[-1].append(0)
        else:
            print('Probabilistic effect')

            effect[5]

        
    print('state_terms_changed: ', state_terms_changed)
    #print('Outcome: ', outcome)

    # Check if outcome is satisfied by end state
    non_outcome_term_indices = list(range(len(end_state))) # will denote parts of state that should remain same
    for term in state_terms_changed:
        if term not in end_state:
            if test:
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
            if test:
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

        #print('+++++++++++')
        #print('Action: ', action)

        # Get all possible combos of action param values
        param_combos = getParamCombos(action)
        #print('param combos', param_combos)
        #print(len(param_combos))

        ##preconditionSatisfied(action,param_combos[1],states[5]) NOT DONE

        # For combo in possible combos (ASSUMING ALL PRECOND SATISFIED ALREADY FOR NOW)
        for combo_dict in param_combos:

            # Init  two NxN arrays with zeros (one for P_a and one for R_a)
            p, r = np.zeros((N,N)), np.zeros((N,N))

            # Loop through start states s, indexing with i
            for i in range(len(states)):
                start_state = states[i]

                if preconditionSatisfied(combo_dict,start_state,action):

                    valid_transition = False

                    # Loop through end states s', indexing with j
                    for j in range(len(states)):
                        end_state = states[j]
                        # If outcome matches s'

                        if outcomeIsEndState(combo_dict,start_state,end_state,action):
                        # Valid transition, add probability (1 if not specified), add reward if any

                            print('Start state: ', start_state)
                            print('Combo dict: ', combo_dict)
                            print('Action: ', action.name)
                            print('End state: ', end_state,'\n')





                    # Else not valid
                    # print('Start state: ', start_state)
                    # print('Combo dict: ', combo_dict)
                    # print('Action: ', action.name)
                    # print('End state: ', end_state)
                    # print('Valid transtion: %s\n' % valid_transition)

                    #    Invalid transtion, probability remains 0 as initialized, add reward maybe (???)

        #print('+++++++++++')

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

def outcomeIsEndStateTest2():

    states = getStateList()

    action = domain.operators[0]
    print('Action: ', action)

    param_combos = getParamCombos(action)

    count = 0
    for i in range(len(states)):
        start_state = states[i]
        #print('Start: ', start_state)

        for combo_dict in param_combos:

            #print('Param combo: ', combo_dict)

            for j in range(len(states)):
                end_state = states[j]
                #print('Precond satisfied? %s\n' % preconditionSatisfied(combo_dict,start_state,action,test=False))
                if preconditionSatisfied(combo_dict,start_state,action,test=False):
                    count += 1
                    outcome = outcomeIsEndState(combo_dict,start_state,end_state,action,test=False)

                    if outcome:
                        print('Combo dict: ', combo_dict)
                        print('Start: ', start_state)
                        print('End: ', end_state)
                        print('Outcome: %s\n' % outcomeIsEndState(combo_dict,start_state,end_state,action,test=True))

                    # print('Combo dict: ', combo_dict)
                    # print('Start: ', start_state)
                    # print('End: ', end_state)
                    # print('Outcome: %s\n' % outcomeIsEndState(combo_dict,start_state,end_state,action,test=True))

    print('Count: ', count)


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

    print('Precond satisfied? %s\n' % preconditionSatisfied(combo_dict,start_state,action,test=True))

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

    print('Precond satisfied? %s\n' % preconditionSatisfied(combo_dict,start_state,action,test=True))

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

    print('Precond satisfied? %s\n' % preconditionSatisfied(combo_dict,start_state,action,test=True))


def precondSatisfiedTest2():

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
            






if __name__ == '__main__':
    args = parse()

    domain  = PDDLParser.parse(args.domain)
    problem = PDDLParser.parse(args.problem)

    print(domain)
    print(problem)

    #getPandR() #??? fix this, prints valid transition when end state is wrong

    #outcomeIsEndStateTest()
    #outcomeIsEndStateTest2()

    #precondSatisfiedTest()
    #precondSatisfiedTest2()

    #test_outcome()
    states = getStateList()
    print('Scenario 1:')
    start_state = states[5]
    print('start state ', start_state)
    end_state = states[9]
    print('expected end state ', end_state)

    action = domain.operators[1] #[0] = move
    print('Action: ', action)

    combos = getParamCombos(action)
    combo_dict = combos[1] #[2] is correct for states[5] and action 0
    print('Combo: ', combo_dict)

    outcome(combo_dict,start_state,action)

    print('=======================')
    print(isinstance('reward',Literal))
