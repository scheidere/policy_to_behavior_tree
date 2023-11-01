#!/usr/bin/env python
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
from simplify import * # NEW WAY
from evaluate_mdp_policy import *

import mdptoolbox
import numpy as np
import mdptoolbox.example

import argparse
import sys
#sys.path.append('../../../pypddl-parser/pypddl-parser')
#sys.path.append('../../../pypddl_parser/src/pypddl_parser')
from behavior_tree.behavior_tree import * # testing only
from pypddl_parser.pddlparser import PDDLParser
from pypddl_parser.literal import Literal #used for isinstance()

import itertools
from itertools import product
import numpy as np
import copy

np.set_printoptions(threshold=sys.maxsize) # So you can see matrices without truncation

def dictproduct(dct):
    for t in product(*list(dct.values())):
        yield dict(list(zip(list(dct.keys()), t)))

def getParamCombos(action,problem):

    #print('In getParamCombos')

    combo_list = []

    # Dict of param name keys, with possible value list as arg for each
    param_values_dict = {}
    for param in action.params:

        param_values_dict[param._name] = problem.objects[param.type]

    #print('param val dict', param_values_dict)

    # Get all combinations of param values in dictionaries, congregate in list
    combo_list = list(dictproduct(param_values_dict))

    return combo_list

def getStateList(domain,problem):

    #input('IN GET STATE LIST')

    # print('In getStateList')
    # print('domain types', domain.types)
    print(('HELLO', problem.objects))

    # Get all states (valid and invalid)

    single_state = []
    for i in range(len(domain.predicates)):
        print(('Predicate is %s' % str(domain.predicates[i])))
        for variable_type in domain.types:
            print(('variable type ', variable_type))
            if variable_type in str(domain.predicates[i]):
                print(('This predicate has variable type %s' % variable_type))
                print(('problem.objects[variable_type]', problem.objects[variable_type]))
                for value in problem.objects[variable_type]:

                    print(('problem object ', value))

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
        states = removeInvalidStates(states,domain)


    for i in range(len(states)):
        print((i, '', states[i],'\n'))
    #input('wait states')

    #input('OUT GET STATE LIST')

    return states

def removeInvalidStates(state_list,domain):

    # This would need to have more constraint types added to be entirely complete

    print(('BEFORE ', state_list))

    valid_states_list = []
    removal_indices = [] # Indices of all invalid states

    # For each constraint, remove states that do not satisfy 
    for i in range(len(domain.constraints)):
        constraint = domain.constraints[i]
        # print('Constraint: ', constraint)
        # print(constraint._name)
        # print(constraint._literal._predicate._name)
        # print(''.join(map(str, constraint._params)))

        # Check each state for failure, per kind of constraint (currently only at-most-once)
        for j in range(len(state_list)):
            state = state_list[j]
            #print('state ', state)
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
                            #print('num ',num)
                            continue
                        elif num==1 and term[-1]==num: # Found another 1, so constraint is not satisfied
                            removal_indices.append(j)
                            #print("More than one 1 found")
                            break
                        elif num==0:
                            if term[-1] == 1: # found a one
                                if not found_a_one:
                                    found_a_one = True # found first 1
                                else: # found second 1, constraint not satisfied
                                    removal_indices.append(j)
                                    #print("More than one 1 found*")
                                    break


                # After searching whole state
                if num == 0 and not found_a_one: # If all zeros -> failure
                    removal_indices.append(j)
                    #print('all zeros')
 


    # Only retain valid states
    for idx in range(len(state_list)):
        if idx not in removal_indices:
            # Then state at idx in list is valid
            valid_states_list.append(state_list[idx])

    return valid_states_list


def testRemoveInvalidStates(domain,problem):

    states = getStateList(domain,problem)

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

def preconditionSatisfiedNoParams(start_state, action, combo_dict=None, test=False):

    # Still need to add and test OR option


    print('Not OR - precondsatnoparam')
    # Check that parameter values in combo match start state per action precondition 
    for precond in action.precond:

        if test:
            print(('precond type ', type(precond)))
            print(('precond._predicate', precond._predicate))
            print(('precond._predicate.name', precond._predicate.name))

        # for term in precond._predicate._args:
        #     print('arg', term._value)

        if precond._predicate.name != '=':

            print('Not =')

            match_found = False
            
            ##precond_args = precond._predicate.args

            precond_args = []
            for arg in precond._predicate._args:
                precond_args.append(arg._value)
            #if test:
            print(('hi',precond._predicate.name, precond_args))

            # Search start state for precondition predicate name where true
            for term in start_state:
                # True is defined by term[-1] == 1

                # First, check that the state term is the same as the precondition term
                if term[0] == precond._predicate.name:

                    print(('matches term[0]', term[0], precond._predicate.name))
                    print(('precond is pos', precond.is_positive()))
                    print(('precond is neg', precond.is_negative()))
                    print(('term[-1]', term[-1]))

                    # Second, check that the Boolean value is the same for both
                    if precond.is_positive() and term[-1] == 1 or precond.is_negative() and term[-1] == 0:

                        print('found true preciate in state that matches precond term')

                        # Third, check that the parameters/args are the same
                        term_arg_vals = term[1:-1] 
                        print(('term_arg_vals ', term_arg_vals))
                        print(('compared to precond_args: ', precond_args))

                        # Compare start state term with combo
                        if term_arg_vals != precond_args:
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

        # Otherwise deal with == scenario, (how does this work with constants? - currently not needed)
        #elif: ...


    return True

def getPrecondArgValues(precond, combo_dict):

    print(('In getPrecondArgValues',precond,precond._predicate.args))

    vals = []
    for arg in precond._predicate.args:
        #print('arg', arg, type(arg))
        # If arg is a constant
        if not isinstance(arg,str):
            vals.append(arg._value)
        else:
            vals.append(combo_dict[arg])

    print(('VALS', vals))

    return vals

def usesConstant(precond_vals, combo_dict):

    # Set flag that denotes whether current precond uses constant or combo_dict element
    has_constant = False

    if combo_dict:
        for val in precond_vals:
            if val not in list(combo_dict.values()):
                # Found one that is a constant
                has_constant = True
                return True
    else: # if no combo dict, constants involved
        return True

    return has_constant



def preconditionSatisfied(start_state,action,combo_dict=None,test=False):

    # If action params, assumes no constants (for now)
    #if combo_dict:

    print(('HMMMMMM', type(action.precond)))
    if combo_dict:  
        print(('combo_dict', combo_dict))

    #print('type', action.precond.type)
    # if action.precond.type == Literal:
    #     print('IT BE A LITERAL')
    #     input('waaaa')

    if action.precond[0] == 'or':
        precond_lst = action.precond[1]
    else:
        precond_lst = action.precond

    print(('PRECOND LST', precond_lst))
    #input('wut')

    for i in range(len(precond_lst)):

        precond = precond_lst[i]

        if precond._predicate.name != '=':
            match_found = False

            #print('pre val', precond._predicate.args[0]._value)
            #print(getPrecondArgValues(precond))
            precond_vals = getPrecondArgValues(precond, combo_dict)

            # Set flag that denotes whether current precond uses constant or combo_dict element
            has_constant = usesConstant(precond_vals, combo_dict)

            if combo_dict and not has_constant: # Combo_dict only
                precond_args = precond._predicate.args
                print(('no constants', precond_args))
            else: # Constants and combo_dict or constants only, but current precond term uses constant
                precond_args = precond_vals
                print(('both',))

            # else: # Constants ony
            #     # precond_args = []
            #     # for arg in precond._predicate._args:
            #     #     precond_args.append(arg._value)
            #     precond_args = precond_vals


            # Search start state for precondition predicate name where true
            for term in start_state:

                # First, check that the state term is the same as the precondition term
                if term[0] == precond._predicate.name:

                    # Second, check that the Boolean value is the same for both
                    if precond.is_positive() and term[-1] == 1 or precond.is_negative() and term[-1] == 0:

                        # Third, check that the parameters/args are the same
                        term_arg_vals = term[1:-1] # if are no parameters this is = [] or has dummy var like 'x'

                        # for arg in precond_args:
                        #     print('precond_arg', arg._value)
                        # input('heeyy')


                        if combo_dict and not has_constant: # No constant invovled
                            # Get parameter values to compare with from the param_combo given
                            compare_vals = getComboArgValues(precond_args,combo_dict)
                        else: # Constant invovled
                            compare_vals = precond_args

                        # Compare start state term with args from precond
                        if term_arg_vals == compare_vals:

                            match_found = True

                            if action.precond[0] == 'or':
                                return True
                            else: #precondition is not an OR
                                break
            
            # Non-or case: Precondition fails for given state and param combo
            if not match_found and not action.precond[0]=='or':
                return False



        # Check equation preconditions with combo parameter values
        elif precond._predicate.name == '=' and combo_dict: # Currently does not support constants in domain
            args = precond._predicate.args

            if precond.is_positive(): # =, need equality

                if combo_dict[args[0]] == combo_dict[args[1]] and action.precond == 'or':
                    return True # Match found, only one needed for OR

                elif combo_dict[args[0]] != combo_dict[args[1]]:
                    return False # Match not found once

            else: # !=, need inequality

                if combo_dict[args[0]] != combo_dict[args[1]] and action.precond == 'or':
                    return True # Correct inequality, so match, only one needed for OR

                elif combo_dict[args[0]] == combo_dict[args[1]]:
                    return False # Match when should be different

    # If after checking all predicates in the OR list, no match found, then precondition not satisfied
    if action.precond[0] == 'or':
        return False

    return True


def preconditionSatisfied_old(start_state,action,combo_dict=None,test=True):

    print('IN precondSatisfied')
    # print('start_state ', start_state)
    # print('action ', action)

    print(('combo_dict ', combo_dict))


    if combo_dict:

        # If ANY precond matches the given state, the OR precond is satisfied
        if action.precond[0] == 'or':

            print(('Is OR ', action.precond[1], len(action.precond[1])))

            for i in range(len(action.precond[1])):

                print(('Looking at precond', i))

                precond = action.precond[1][i]

                if precond._predicate.name != '=':
                    match_found = False

                    print((i, 'match found', match_found))
                    
                    precond_args = precond._predicate.args
                    if test:
                        print(('predicate and args: ',precond._predicate.name, precond_args))

                    # Search start state for precondition predicate name where true
                    for term in start_state:
                        # True is defined by term[-1] == 1

                        if test:
                            print((type(term[0])))

                        # First, check that the state term is the same as the precondition term
                        if term[0] == precond._predicate.name:

                            print(('state term matches precond ', term[0], precond._predicate.name))
                            print(('precond is pos, term[-1]', precond.is_positive(), term[-1]))

                            # Second, check that the Boolean value is the same for both
                            if precond.is_positive() and term[-1] == 1 or precond.is_negative() and term[-1] == 0:

                                print(("boolean val matches (i.e. 0 in term and 'not' in precond) ", term[-1]))

                                # Third, check that the parameters/args are the same
                                term_arg_vals = term[1:-1] # if are no parameters this is = [] or has dummy var like 'x'
                                if test:
                                    print(('term_arg_vals: ', term_arg_vals))

                                if combo_dict: # if there are parameters to compare at all (might be None)
                                    # Get parameter values to compare with from the param_combo given
                                    combo_arg_vals = getComboArgValues(precond_args,combo_dict)
                                else:
                                    combo_arg_vals = None

                                if test:
                                    print(('Comparing %s to %s' % (term_arg_vals,combo_arg_vals)))

                                # Compare start state term with combo
                                if term_arg_vals != combo_arg_vals:
                                    # Term does not match arg vals in give combo
                                    if test:
                                        print('Discrepancy found')
                                    else:
                                        pass
                                else:
                                    #if test:
                                    print('Match found') # precondition is satisfied
                                    match_found = True
                                    return True # Only need a single match needed for an OR
                                    #continue

                # Check equation preconditions with combo parameter values
                elif precond._predicate.name == '=':
                    args = precond._predicate.args

                    if precond.is_positive(): # =

                        if combo_dict[args[0]] == combo_dict[args[1]]:
                            return True # Match found, only one needed for OR
                    else: # !=

                        if combo_dict[args[0]] != combo_dict[args[1]]:
                            return True


            # If after checking all predicates in the OR list, no match found, then precondition not satisfied
            return False

        else: # precondition is not an OR
            print(('Not OR', action.precond))
            # Check that parameter values in combo match start state per action precondition 
            for precond in action.precond:

                if test:
                    print(('precond type ', type(precond)))
                    print(('precond._predicate', precond._predicate))
                    print(('precond._predicate.name', precond._predicate.name))
                    # for term in precond._predicate._args:
                    #     print('arg', term._value)

                if precond._predicate.name != '=':

                    if test:
                        print('Not =')

                    match_found = False
                    
                    precond_args = precond._predicate.args
                    if test:
                        print((precond._predicate.name, precond_args))

                    # Search start state for precondition predicate name where true
                    for term in start_state:
                        # True is defined by term[-1] == 1

                        if test:
                            print((type(term[0])))

                        # First, check that the state term is the same as the precondition term
                        if term[0] == precond._predicate.name:

                            # Second, check that the Boolean value is the same for both
                            if precond.is_positive() and term[-1] == 1 or precond.is_negative() and term[-1] == 0:

                                # Third, check that the parameters/args are the same
                                term_arg_vals = term[1:-1] # if are no parameters this is = [] or has dummy var like 'x'
                                #print('term_arg_vals ', term_arg_vals)
                                if test:
                                    print(('term_arg_vals: ', term_arg_vals))

                                if combo_dict: # if there are parameters to compare at all (might be None)
                                    # Get parameter values to compare with from the param_combo given
                                    combo_arg_vals = getComboArgValues(precond_args,combo_dict)
                                else:
                                    combo_arg_vals = None

                                if test:
                                    print(('Comparing %s to %s' % (term_arg_vals,combo_arg_vals)))

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
                                    break #continue


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

    else:
        return preconditionSatisfiedNoParams(start_state,action)

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
    print((equal(state1,state2)))

    # States 3 and 4 are equal but the terms are in different orders (should return TRUE)
    state3 = [['robot-at', 'left-cell', 0], ['robot-at', 'right-cell', 1], ['dirty-at', 'left-cell', 1], ['dirty-at', 'right-cell', 0]]
    state4 = [['robot-at', 'right-cell', 1], ['dirty-at', 'right-cell', 0], ['dirty-at', 'left-cell', 1], ['robot-at', 'left-cell', 0]]
    print((equal(state3,state4)))

    # States 5 and 6 are not equal, one term does not match (should return FALSE)
    state5 = [['robot-at', 'left-cell', 1], ['robot-at', 'right-cell', 1], ['dirty-at', 'left-cell', 1], ['dirty-at', 'right-cell', 1]]
    state6 = [['robot-at', 'left-cell', 1], ['robot-at', 'right-cell', 0], ['dirty-at', 'left-cell', 1], ['dirty-at', 'right-cell', 1]]
    print((equal(state5,state6)))


def getStateIndex(state, states):

    for i in range(len(states)):

        if equal(states[i],state):

            #print("match")
            #print(states[i])
            #print(state)

            return i

def testGetStateIndex(domain,problem):

    states = getStateList(domain,problem)

    state = [['robot-at', 'right-cell', 1], ['dirty-at', 'right-cell', 0], ['dirty-at', 'left-cell', 1], ['robot-at', 'left-cell', 0]]

    #print(getStateIndex(state,states))

def outcome(start_state, action, param_values = None, test=False):

    # param_values same as combo_dict

    #input('entering new outcome function')

    precond_satisfied = True

    # List of lists
    # Each sub list includes an outcome state with probability p and reward r, [state,p,r]
    outcome_list = []

    # Initialize state terms left over after given action is taken in the start state
    unchanged_state_terms = copy.deepcopy(start_state)

    #test (remove these lines after you can access the else below)
    action_effects = action.effects[0] # Indexed to remove duplicate outer list
    #print('Action effect: ', action_effects)
    #input('Waiting')


    if not preconditionSatisfied(start_state,action, combo_dict=param_values):

        print('Precondition not satisfied...')

        outcome_sublist = [unchanged_state_terms,1.0,0]
        outcome_list.append(outcome_sublist)
        precond_satisfied = False

    else:

        #input('precond satisfied')

        replacement_state_terms = [] # [state,p,r]

        action_effects = action.effects[0]
        #print('action effects ', action_effects)

        probs = [] # checking sum to 1

        for i in range(len(action_effects)):

            #input('effect %s' %i)

            term = action_effects[i]

            literals = []

            reward = 0

            if isinstance(term, float):
                prob = term

                probs.append(prob)

                #print('prob: ', prob)

                effects = action_effects[i+1]
                #print('effects: ', effects)

                for effect_term in effects:

                    if isinstance(effect_term, Literal):
                        #input('is literal')

                        literals.append(effect_term)

                    elif effect_term[0] == 'reward':
                        #print('is reward')

                        reward = effect_term[1]

                outcome_sublist = getOutcomeSublist(literals,prob,reward,start_state,param_values,is_probabilistic=True)
                # print('1 ', start_state)
                # print('2 ', outcome_sublist)
                outcome_list.append(outcome_sublist)

        print(('prob check list', probs))
        #if sum(probs) != 1:
        if sum(probs) < .999999 or sum(probs) > 1.000001:
            print(('error found with probs above, sum is not 1: ', sum(probs)))
            eval(input('wait'))
        print(('outcome list', outcome_list))
        if len(outcome_list) != len(probs):
            print('FOUND ERROR')
            eval(input('wait'))

    # if precond_satisfied:
    #     input('Wait')

    #print('exiting new outcome')
    #print('outcome list ', outcome_list)
    return outcome_list, precond_satisfied


def getOutcomeSublist(literals, prob, reward, start_state, param_values, is_probabilistic=False, test=False):

    # literals are the predicates (either with params in param_values or with constants) that define outcome

    print(('agh lit', literals))

    #print('LOOOOOOOOOOK prob ', prob)

    print('In getOutcomeSublist')

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

        print(('PARAMS', params))

        if param_values:
            print(('p vals test', param_values, list(param_values.values())))

        for param in params:

            print(('param', param))

            if param_values:
                # First, consider cases where no constants involved
                if param in list(param_values.values()) or param in list(param_values.keys()):
                    values.append(param_values[param])
                else: # Cases where constants and params
                    values.append(param._value)
            else: # Involves a constant
                values.append(param._value)


        print(('values ', values))
    
        # Search for relevant terms in start state, those that will be affected
        for i in range(len(start_state)):
            term = start_state[i]
            #print('start state term', term)

            # Find main term that will change due to effect of taking action in start state
            if term[0] == predicate_name and term[1:-1] == values:

                print(('match!!!', term, predicate_name, values))
                #print('unchanged_state_terms',unchanged_state_terms)

                unchanged_state_terms.remove(term)

                # Initialize term for end state, state that results from action occurance
                new_term = copy.deepcopy(term[0:-1])

                if literal.is_positive():
                    new_term.append(1)
                else:
                    new_term.append(0)

                new_terms, unchanged_state_terms = clearOutcomeConflicts(new_term, start_state, unchanged_state_terms)

                for term in new_terms:
                    replacement_state_terms.append(term)
                #print('replacement_state_terms', replacement_state_terms)


        end_state = replacement_state_terms + unchanged_state_terms
        outcome_sublist = [end_state,prob,reward]
        #print('prob type', type(prob))
        #print('outcome_sublist 2', outcome_sublist)

    return outcome_sublist


def clearOutcomeConflicts(new_term, start_state, unchanged_state_terms):

    # new_term[0] denotes the name of the condition that explicitly got changed
    # by the action effect, given the precondition

    # If new_term[-1] is 1, then term used to be [predicate name, args, 0]
    # If new_term[-1] is 0, then .. [predicate name, args, 1] (Tougher case)

    print('In clearOutcomeConflicts')
    print(('unchanged_state_terms', unchanged_state_terms))

    new_terms = [new_term]

    print(('NNNEEWWWW TEERRMMM', new_term))

    for i in range(len(start_state)):
        term = start_state[i]
        print(('term',term))

        # Find term that could conflict with change (excluding the one that was explicitly changed itself)
        if term[0] == new_term[0] and term[1:-1] != new_term[1:-1]:

            next_new_term = copy.deepcopy(term[0:-1])

            # If new value is 1, set all same named terms' value to 0 (only one can be True)
            if new_term[-1] == 1: # Prevents multiple ones
                next_new_term.append(0)
                unchanged_state_terms.remove(term)
                new_terms.append(next_new_term)
            elif new_term[-1] == 0:
                # This is more complex and can be avoided by specifying change in domain effect explicitly

                # One case is that all would be zero, which is a logical failure
                pass

    print(('new_terms', new_terms))
    #input('look at new terms')


    return new_terms, unchanged_state_terms

def preconditionSatisfiedActionParams(action, combo_dict, test=False):


    # TODO: DOUBLE CHECK THAT 'OR' LOGIC WORKS CORRECTLY

    # Need to remove things like move(left,left) and move(right,right) by checking preconditions
    # Might also be able to merge with preconditionSatisfied, but not sure what is most elegant option
    # use this in getPandR to ammend the actions_with_params list to only contain valid actions_with_params
    # then update loops so you only consider those actions for p and r

    #print(action,'\n',combo_dict)
    #input('hi')

    # First consider OR case (Need ANY)
    # if action.precond[0] == 'or': # precond = ['or', [predicate1, predicate2, ...]]

    #     for precond in action.precond[1]:

    #         if precond._predicate.name == '=':
    #             args = precond._predicate.args

    #             if precond.is_positive(): # =

    #                 if combo_dict[args[0]] == combo_dict[args[1]]:

    #                     print('True')
    #                     return True

    #             else:

    #                 if combo_dict[args[0]] != combo_dict[args[1]]:

    #                     print('True')
    #                     return True

    #         print('False')
    #         return False



    # Next consider the SINGLE or AND case (original method, OR lines added above)
    #else: # (Need ALL)
    # Check equation preconditions with combo parameter values
    if action.precond[0]== 'or':

        precond_list = action.precond[1]

    else:

        precond_list = action.precond

    for precond in precond_list:

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

def getActionsWithParamsList(domain,problem):

    # Has bugs with infant domain
    # input('HELLO')

    # Get all action/param combos
    # print('In getActionsWithParamsList')

    # Initialize actions with parameters list (used for reading policy)
    actions_with_params = []

    # Loop through all actions in domain
    for action in domain.operators:

        # print('action: ', action)

        # Get all possible combos of action parameter values
        param_combos = getParamCombos(action,problem)
        # print('param_combos ', param_combos)

        for combo_dict in param_combos:

            # Make sure parameters pass the preconditions that can be checked at this stage
            # e.g. params == or != to each other or something
            if preconditionSatisfiedActionParams(action,combo_dict):

                #actions_with_params.append([action.name,combo_dict])
                actions_with_params.append([action,combo_dict])

    # print(actions_with_params)
    # print('end')
    # input('bye')
    return actions_with_params

def getActions(domain):

    actions = []

    for action in domain.operators:

        actions.append(action)

    return actions


def getPandR(domain,problem):

    print("In getPandR")
    #input('wait')

    combo_dict = None

    # Init main lists, NxN arrays will be added to (later converted to 3d numpy array)
    P,R = [],[]

    # Get valid states
    states = getStateList(domain,problem)
    # print('States: ')
    # for state in states:
    #     print(state)
    # input('look at states for conflicts!')

    # Initialize actions with parameters list (used for reading policy)
    #actions_with_params = []

    # Number of states
    N = len(states)
    #print('Number of states, N: ',N)

    if problem != None:

        # print('Problem exists')

        # Get valid action/param combos
        actions = getActionsWithParamsList(domain,problem) # old var name: actions_with_params
        #print('actions: ', actions)
        # input('yo1')
        # print('actions: ')
        # for action in actions:
        #     print(action[0]._name, action[1])
        # input('yo2')

        # for a in actions:
        #     print(a)

        # input("YOU ARE HERE")

    else:
        #print('Problem is None')
        actions = getActions(domain)
        #print('actions: ', actions)

    for action_term in actions:

        print(('action_term', action_term))

        # if problem != None:
        #     action, combo_dict = action_term # action term is [action, params]
        # else:
        #     action = action_term

        if problem != None and action_term[1]!={}:
            action, combo_dict = action_term # action term is [action, params]
        elif problem == None:
            action = action_term
        else:
            action = action_term[0]

        # if combo_dict == {}: #when params are empty e.g. :parameters ()

        #     TODO need to accomodate for combo_dict being empty bc no params, but constants being used instead

        # Init two NxN arrays with zeros (one for P_a and one for R_a)
        p, r = np.zeros((N,N)), np.zeros((N,N))

        # Loop through start states s, indexing with i
        for i in range(len(states)):
            start_state = states[i]

            print('+++++++++++++++++++')
            print(('action ',action))
            if combo_dict:
                print(('params ', combo_dict))
            else:
                print('no params')
            print(('start state ', start_state))

            # Get list of [end state, prob, reward] terms, given action and start state
            # Also return only the actions_with_params that satisfy preconds
            if combo_dict:
                #print('combo_dict')
                outcome_list, precond_satisfied = outcome(start_state,action,param_values=combo_dict)
            else: # no parameters
                #print('no params')
                outcome_list, precond_satisfied = outcome(start_state,action)

            # Update NxN matrices, p and r according to outcome
            print(('outcome_list ', outcome_list))
            print('==============================')
            #input('wait')
            all_js = [] #debugging
            j_duplicate_tracker = {} # Init tracker that sorts outcome_sublists together, by their associated j (same outcome state)
            for idx in range(len(outcome_list)):

                outcome_sublist = outcome_list[idx]

                # Get index of end state where outcome_sublist = [end state, prob, reward]
                #print(outcome_sublist[0])
                j = getStateIndex(outcome_sublist[0],states)
                all_js.append(j)
                if j not in list(j_duplicate_tracker.keys()):
                    j_duplicate_tracker[j] = [outcome_sublist]
                else:
                    j_duplicate_tracker[j].append(outcome_sublist)

            #print('j track', j_duplicate_tracker)
            p_sum_check = []
            r_sum_check = []
            # Make sure to count all outcome_sublists, which may have the same outcome state, indexed by j
            for j in list(j_duplicate_tracker.keys()):

                outcome_sublist_j_group = j_duplicate_tracker[j]

                # Sum probability and reward for outcomes that share j
                p_sum = 0
                r_sum = 0
                for outcome_sublist in outcome_sublist_j_group:

                    p_sum += outcome_sublist[1]
                    r_sum += outcome_sublist[2]
              
                p[j,i] = p_sum
                r[j,i] = r_sum
                p_sum_check.append(p_sum)
                r_sum_check.append(r_sum)
            #print('p', p_sum_check)
            #print('r', r_sum_check)
                #input('Wait')

            # if len(np.unique(all_js)) != len(all_js):
            #     print(all_js)
            #     print(np.unique(all_js))
            #     input('found overwrite issue')

        # Check for p row that does not sum to 1 (it should because it is a probability distribution)!
        # if sum(p) != 1:
        #     input('shooooot', sum(p))
                
        # Add NxN matrices to lists P and R
        P.append(p)
        R.append(r)

    # Convert list of matrices to 3d numpy array
    P = np.dstack(P).transpose()
    R = np.dstack(R).transpose()
    # actions = []
    # for action in domain.operators:
    #     actions.append(action.name)

    #input("YOU ARE HERE NOW")


    return P, R, states, actions

def test_outcome(domain):

    # Need to confirm this test is updated, so it can actually be used to test...

    states = getStateList()

    for action in domain.operators:

        combos = getParamCombos(action,domain)

        for start_state in states:

            for combo_dict in combos:

                #if preconditionSatisfied(combo_dict,start_state,action): 
                    # ??? move precond satisfied to inside outcome function
                    # when precond not satisfied for certain action/param combo/state 
                    # you still want to add a 1 at i,j where i=j for the unchanged start state

                print('\n')
                print(('Start state: ', start_state))
                print(('Combo dict: ', combo_dict))
                print(('Action: ', action.name))
                ps = preconditionSatisfied(start_state,action, combo_dict=combo_dict)
                print(('Precondition satisfied? ', ps))
                outcome_list = outcome(start_state,action,param_values=combo_dict)
                for o in outcome_list:
                    print(o)
                

                print('\n')


def precondSatisfiedTest(domain,problem):

    # Need to confirm this test is updated, so it can actually be used to test...

    states = getStateList()

    for action in domain.operators:
    #action = domain.operators[0]
    #print('Action: ', action)

        param_combos = getParamCombos(action,problem)

        for i in range(len(states)):
            start_state = states[i]
            

            for combo_dict in param_combos:

                print(('Action: ', action))

                print(('Start: ', start_state))

                print(('Param combo: ', combo_dict))

                print(('Precond satisfied? %s\n' % preconditionSatisfied(start_state,action, combo_dict=param_values,test=True)))


def solve(solver,P,R):

    # Get transition probabality and reward matrices from PPDDL

    if solver == 'v':
        val_it = mdptoolbox.mdp.ValueIteration(P, R, 0.96)
        val_it.run()
        policy = val_it.policy
        print(('\nPolicy: ', val_it.policy))
    elif solver == 'q':
        q_learn = mdptoolbox.mdp.QLearning(P, R, 0.96)
        q_learn.run()
        policy = q_learn.policy
        print(('\nPolicy: ', q_learn.policy))
    else:
        print("That is not a valid solver...")

    return policy

def readPolicy(policy,states,actions_with_params):

    print('Policy in readable form: ')

    for i in range(len(policy)):

        print(('Index in policy: ', i))
        print(('Action index: ', policy[i]))
        print((states[i]))
        #print(actions_with_params[policy[i]][0].name,actions_with_params[policy[i]][1],'\n')
        print((actions_with_params[policy[i]][0],actions_with_params[policy[i]][1],'\n'))

def checkPolicyPreconditions(policy, states, actions_with_params):

    # Used to make the policy readable and check for issues with preconditions

    print('Policy in readable form: \n')

    for i in range(len(policy)):

        print('====================')
        print(('Index in policy: ', i))
        print(('Action index: ', policy[i]))
        print(('State: \n', states[i]))
        action = actions_with_params[policy[i]][0]
        print(('\nAction: \n %s' % action))
        param_values = actions_with_params[policy[i]][1]
        print(('Params: \n', param_values))
        precondsatisfied = preconditionSatisfied(states[i],action, combo_dict=param_values)
        print(('\nPreconditions satisfied? %r' % precondsatisfied))
        if not precondsatisfied:
            eval(input('Found failure'))

def saveReadablePolicy(policy, states, actions_with_params):

    # Save readable and basic "state: action" policy form to file f
    f = open("/home/scheidee/Desktop/AURO_results/raw_policy.txt", "w+")
    fa = open("/home/scheidee/Desktop/AURO_results/raw_policy_actions.txt", "w+")

    for i in range(len(policy)):

        f.write("State count: %d\n"%(i+1))
        f.write("State: %s\n" %str(states[i]))
        action = actions_with_params[policy[i]][0]
        f.write("Action: %s\n" %str(action.name))
        fa.write("%s\n" %str(action.name))



def get_average_reward(num_runs, mdp_problem, policy):

    # This might be redundant with evaluate_mdp_policy in evaluate_mdp_policy.py

    print(('Getting average reward over %d trials... It should take about a minute.' % num_runs))

    rewards = []
    for i in range(num_runs):

        reward = evaluate_mdp_policy(mdp_problem, policy)
        #print(i,': ', reward)

        rewards.append(reward)


    return sum(rewards)/len(rewards), rewards


def getAverageRewardInSameWorld():

    # domain3
    p_policy = (5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 4, 6, 1, 1, 1, 6, 2, 2, 2, 2, 1, 1, 1, 1, 4, 4, 4, 6, 1, 1, 1, 6, 2, 2, 2, 2, 1, 1, 1, 1)
    
    # domain2
    #p_policy = (5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0, 6, 0, 0, 0, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 4, 6, 1, 1, 1, 1, 4, 4, 4, 2, 1, 1, 1, 1, 4, 4, 4, 6, 1, 1, 1, 1, 4, 4, 4, 2, 1, 1, 1, 1)
    d_policy = (5, 5, 3, 6, 5, 5, 3, 6, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0, 6, 0, 0, 0, 6, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 4, 6, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 4, 6, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2)

    num_runs = 100

    p_avg_reward = get_average_reward(num_runs, mdp_problem, p_policy)
    print(('\nAverage reward over %d trials for probabilistic policy: %f' % (num_runs, p_avg_reward)))
    d_avg_reward = get_average_reward(num_runs, mdp_problem, d_policy)
    print(('\nAverage reward over %d trials for deterministic policy: %f' % (num_runs, d_avg_reward)))




if __name__ == '__main__':

    # Define domain and problem to consider (they represent an MDP)
    print('\nFor the following domain and problem: \n\n')
    args = parse()
    print((type(args.domain)))
    print((args.problem))
    eval(input('wait'))
    domain  = PDDLParser.parse(args.domain)
    problem = PDDLParser.parse(args.problem)

    # Hard-coded
    # prob_domain = PDDLParser.parse('../../../pypddl-parser/pypddl-parser/pddl/marine/domain3.ppddl')
    # problem = PDDLParser.parse('../../../pypddl-parser/pypddl-parser/pddl/marine/problems/problem1.ppddl')

    # det_domain = PDDLParser.parse('../../../pypddl-parser/pypddl-parser/pddl/marine/domain_deterministic.ppddl')

    # print(prob_domain)
    # print('deterministic:\n', det_domain)
    # print(problem)

    # domains = [prob_domain,det_domain]

    # for domain in domains:

    #     main()


    # domain = PDDLParser.parse('../../../pypddl-parser/pypddl-parser/pddl/marine/domain3.ppddl')
    # problem = PDDLParser.parse('../../../pypddl-parser/pypddl-parser/pddl/marine/problems/problem1.ppddl')

    print(domain)
    print(problem)

    mdp_problem = main()

    # print('Now lets compare performance of the policies learned from the deterministc vs the probabilistic domains')
    # # print('Probabilistic: ', prob_policy)
    # # print('Deterministic: ', det_policy)
    # num_trials = 100
    # getAverageRewardInSameWorld(prob_mdp_problem, prob_policy, det_policy)

    # HARDCODED FOR SPEED
    getAverageRewardInSameWorld()
