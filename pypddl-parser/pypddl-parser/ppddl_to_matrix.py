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

import itertools
import numpy as np
import copy

def parse():
    usage = 'python3 main.py <DOMAIN> <INSTANCE>'
    description = 'pypddl-parser is a PDDL parser built on top of ply.'
    parser = argparse.ArgumentParser(usage=usage, description=description)

    parser.add_argument('domain',  type=str, help='path to PDDL domain file')
    parser.add_argument('problem', type=str, help='path to PDDL problem file')

    return parser.parse_args()

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

    print('states ', states)

    return states

def removeInvalidStates(states):

    # ALERT! THIS IS NOT GENERALIZED YET. ONLY WORKS FOR VACUUM TOY PROBLEM

    # This function needs to remove states where predicates that can only have one True/False at a time, show up twice
    # An example of this is robot-at, where it shows up twice, but sometimes is True True or False False (not possible)
    # This is possible for dirty-at

    # For now we assume this repitition is okay because the solver will learn these states are not possible

    # However, if this poses an issue, I think I could rely on the init part of the problem file to determine 
    # which predicates can be True OR False and which can be True and True, False and False, or either or


    # For now, we are doing a cheat way which is not generalizeable past the vacuum problem
    new_states = copy.deepcopy(states)
    for i in range(len(states)):
        state = states[i]
        #print('state', state)
        #print(state[0][1],state[0][2])
        #print(state[1][1],state[1][2])
        if state[0][2] == state[1][2]:
            # if robot-at left and robot-at right
            # this both cannot be true or both be false at the same time
            # the robot must be somewhere, and only one cell at a time

            #print('Removing state...')
            new_states.remove(state)
        else:
            #print('State valid')
            pass

    print('Valid states only: ', new_states)
    print('Number of valid states: ', len(new_states))
    return new_states

def getPossibleParamValues(states):

    # Currently only for action 0!

    param_value_dict = {}

    for p in domain.operators[0].params:
        for t in domain.types:
            if p.type == t:
                #print("Found parameter (%s) of type (%s) " % (p,t))
                #print('Possible values: ', problem.objects[t])
                param_value_dict[p.name] = problem.objects[t]

    print('Parameters/values for action move: ', param_value_dict)
    return param_value_dict

def getStartStates(states,param_value_dict):

    # Currently only for action 0!

    print(domain.operators[0].params[0].name)

    print(domain.operators[0].params[0].type)

    #print(domain.operators[0].precond)
    #for s in states:
    state = states[0]
    print('Consider this state: ', state)
    for precond in domain.operators[0].precond:
        #print('1',precond._predicate.name)
        for term in state:

            # E.g. robot-at(?x)
            if term[0] == precond._predicate.name and term[2] == 1:
                current_arg_values = term[1:-1]
                print('Current %s value: %s' % (precond._predicate._args, current_arg_values))

            #elif precond._predicate
                            #    print('bla')
        #if domain.operators[0].params[0].name in precond.__str__():
        #print(precond.__str__())

    start_state_param_value_dict = copy.deepcopy(param_value_dict)

    print('Testing new way...')
    for param_value_1 in param_value_dict['?x']:
        print('Param 1 value is: ', param_value_1)
        for param_value_2 in param_value_dict['?y']:
            print('Param 2 value is: ', param_value_2)
            for precondition in domain.operators[0].precond:

                print('Considering %s precondition...' %precondition)
                print(precondition.is_positive())

                if precondition._predicate.name == '=':

                    # ==
                    if precondition.is_positive():
                        print('=')

                    #!=
                    else:
                        print('!=')


        # NOT DONE BUT LEAVING THIS FUNCTION FOR NOW
        #elif 

        #for param in precond._predicate._args:

        #    print('Consider parameter %s' %param)


def checkTransition(start_state,end_state):

    # Only set up for action 0, i.e. move

    # This function checks whether you can go from start to end state via action move

    x,y = getParams(start_state, end_state)

    if x == y:
        return False

    return True

def checkTransition2(start_state,end_state,action):

    # action i in the form domain.operators[i]

    print('===========')
    print('start_state', start_state)


    # Will we need this forbidden values concept or should I get rid of it?
    # It was supposed to be for effects that include a (not (this)) type statement
    # Not sure it has enough context to be used, might need to rely on preconditions

    end_values_dict = {} # Values we want to see in end state
    ##forbidden_values_dict = {} # Values stated with a 'not' in the action effect
    print('end_state ', end_state)
    for effect in action.effects:
        # print(effect) # e.g. (1.0, robot-at(?y))
        #literal = effect[1] # robot-at(?y)
        predicate = effect[1]._predicate # bc effect[1] is a literal
        # print(predicate.name)
        # print(effect[1].is_positive()) # If false, means it is (not robot-at(?x)) for example
        # print(predicate.args)
        params = predicate.args # Get params from predicate term
        for term in end_state:
            if term[0] ==  predicate.name and term[2] == 1:
                # e.g. robot-at, left-cell, 1 given predicate name robot-at
                param_values = term[1:-1] # Need to ensure order always matches params here
                for i in range(len(params)): # Get each param (i.e. key) from the list
                    ##if effect[1].is_positive(): # We want this predicate/param pair to be True in end state
                    end_values_dict[str(params[i])] = [param_values[i],effect[0]] # Include probability
                    ##else:
                    ##    forbidden_values_dict[str(params[i])] = [param_values[i],effect[0]]
    #print('Goal ', end_values_dict)
    ##print('Anti-goal: ', forbidden_values_dict)

    start_values_dict = {}

    for precondition in action.precond:

        # For preconditions like robot-at(?x)
        if precondition._predicate.name != '=': # Probably needs to be updated for generalizability
            for predicate in domain.predicates: #robot-at or dirty-at
                if precondition._predicate.name == predicate.name:

                    # Just x or y in our case, but could be multiple
                    params = precondition._predicate.args # ['?x']          
                    #print('aha!', precondition._predicate.args)

                    # So now we have a predicate such as robot-at
                    # And we have an arg, i.e. ['?x']
                    for term in start_state:
                        if term[0] == predicate.name and term[2] == 1:
                            param_values = term[1:-1] # param value where robot-at is True (1)
                            #print('param_value', param_values)

                            for i in range(len(params)): # Get each param (i.e. key) from the list
                                start_values_dict[str(params[i])] = param_values[i]
    
                            # ???
                            # add a dict just in case there are multiple params to return from precondition
                            # use effects part of action and end_state to get y (potentially multiple again)
                            # Start with assuming just one, i.e. y and compare results to hardcoded version
                            # after that, test with other action


        # For preconditions like x != y (how do you know where to get x/y from, start or end state???)
        # (Check this: could it ever be robot-at(bla) == dirty-at(bla2) or something?)
        # or is it always just a param like x
        elif precondition._predicate.name == '=':
            params = precondition._predicate.args
            #print(params)
            if precondition.is_positive(): # =
                #print('==')

                # Transition is invalid if x and y are not equal
                if start_values_dict[params[0]] != end_values_dict[params[1]]:
                    print('Invalid transition!')
                    return False


            else: # !=
                #print('!=')
                # Transition is invalid if x and y are equal
                print(params[0], start_values_dict[params[0]]) # start_values_dict has just bla form
                print(params[1], end_values_dict[params[1]][0]) #NOTE: end_values_dict has [bla,1] form
                if start_values_dict[params[0]] == end_values_dict[params[1]][0]:
                    print('Invalid transition!')
                    return False
    

    #print('Start ', start_values_dict)

    print('Valid transition!')
    return True

    
    #elif precondition._predicate.name == '=':


def getParams(start_state,end_state):

    # Harcoded for action move

    for term in start_state:
        if term[0] == 'robot-at':
            if term[2] == 1:
                # Then it is true the robot is at the location x
                x = term[1]

    for term in end_state:
        if term[0] == 'robot-at':
            if term[2] == 1:
                # Then it is true the robot is at the location y
                y = term[1]

    return x,y

def getProbabilityMatrixforActionMove(states):

    print('Creating NxN array for action %s...' % domain.operators[0].name)
    num_states = len(states)
    probability_transition_matrix = np.zeros((num_states,num_states))

    # All possible values for each parameter
    param_value_dict = getPossibleParamValues(states)

    # Get start states from action
    #getStartStates(states,param_value_dict) #ignoring this for now

    for i in range(len(states)):
        s = states[i]
        for j in range(len(states)):
            new_s = states[j]
            print('Start: ', s)
            print('End: ', new_s)
            if checkTransition(s,new_s):
                # Valid transition!
                print('Valid transition for move action given')
                probability_transition_matrix[i][j] = 1
                print('P: ', 1)
            else:
                print('Not valid')


    print("P: ", probability_transition_matrix)
    return probability_transition_matrix

    # All for second parameter

    # Loop version

def getParams2(start_state,end_state):

    # Harcoded for action move

    for term in start_state:
        if term[0] == 'robot-at':
            if term[2] == 1:
                # Then it is true the robot is at the location x
                x = term[1]

    for term in end_state:
        if term[0] == 'robot-at':
            if term[2] == 1:
                # Then it is true the robot is at the location y
                y = term[1]

    return x,y

def getProbabilityMatrixforActionMove2(states):

    print('Creating NxN array for action %s...' % domain.operators[0].name)
    num_states = len(states)
    probability_transition_matrix = np.zeros((num_states,num_states))

    # All possible values for each parameter
    param_value_dict = getPossibleParamValues(states)

    # Get start states from action
    #getStartStates(states,param_value_dict) #ignoring this for now

    for i in range(len(states)):
        s = states[i]
        for j in range(len(states)):
            new_s = states[j]
            print('Start: ', s)
            print('End: ', new_s)
            if checkTransition(s,new_s):
                # Valid transition!
                print('Valid transition for move action given')
                probability_transition_matrix[i][j] = 1
                print('P: ', 1)
            else:
                print('Not valid')


    print("P: ", probability_transition_matrix)
    return probability_transition_matrix


def getProbabilityMatrix(states):

    # returns a numpy array of shape (2,8,8)
    # if there are 8 possible states
    # and 2 possible actions

    #Single action array
    # Get number of states
    N = len(states)
    print('size',N)

    # Create probability transition matrix for given action
    single = np.zeros((N,N))
    print(single.shape)

    # Logically I know that state transitions generally
    # in the vacuum problem are only valid if there is only one change
    # i.e. dirty to not dirty at one single location

    possible_param_value_sets = []

    # we must look at a given action
    for i in range(len(domain.operators)):
        print('Creating NxN array for action %s...' % domain.operators[i].name)
        #print('Parameters: ', domain.operators[i].params[0])

        for p in range(len(domain.operators[i].params)):
            print(domain.operators[i].params[p])

        #print(domain.operators[i].effects)

        # Get initial possible states based on preconditions
        if domain.operators[i].precond:
            #Get subset of starting states
            print('yes precondition')
            print('precondition: ' + str(domain.operators[i].precond))

            # Get start states, s
            for state in states:
                pass

            # Get end states, s'

            #possible_states = ???

        else:
            print('No preconditions, action applicable in all states')
            possible_states = states # redundant, just for now

    # look at the effect, and translate that effect


    # if the effect does not include probabilistic, it will be probaility 1

    # for s in states:
    #     for s_prime in states:



    # if probability in domain.action:
    #     then we have a non 1 or 0 probability

    # return
    #todo


def getPandR():
    pass


if __name__ == '__main__':
    args = parse()

    domain  = PDDLParser.parse(args.domain)
    problem = PDDLParser.parse(args.problem)

    print(domain)
    print(problem)

    #print("++++++++++++++++++++++++++++++")
    #print(domain.operators[0].effects)
    #print(domain.predicates[1])
    #print("++++++++++++++++++++++++++++++")
    #print(problem.objects[domain.types[0]])
    states = getStateList()
    states = removeInvalidStates(states)
    for state in states:
        print(state)

    # Run this line below for hardcoded example for action move
    #getProbabilityMatrixforActionMove(states)


    # Testing generalization with 'move' action
    checkTransition2(states[0],states[1],domain.operators[0]) # Invalid state pair
    checkTransition2(states[0],states[4],domain.operators[0]) # Valid state pair



    

