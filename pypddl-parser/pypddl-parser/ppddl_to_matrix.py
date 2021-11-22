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

def getStartStates(states):

    # Currently only for action 0!

    print(domain.operators[0].params[0].name)

    print(domain.operators[0].params[0].type)

    #print(domain.operators[0].precond)
    #for s in states:
    state = states[0]
    print('Consider this state: ', state)
    for precond in domain.operators[0].precond:
        print('1',precond._predicate.name)
        for term in state:
            if term[0] == precond._predicate.name:
                pass
                ???
            #    print('bla')
        #if domain.operators[0].params[0].name in precond.__str__():
        #print(precond.__str__())


def getProbabilityMatrixforActionMove(states):

    print('Creating NxN array for action %s...' % domain.operators[0].name)

    # All possible values for each parameter
    param_value_dict = getPossibleParamValues(states)

    # Get start states from action
    getStartStates(states)

    for s in states:
        for new_s in states:
            pass




    # All for second parameter

    # Loop version


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

    #getProbabilityMatrix(states)
    getProbabilityMatrixforActionMove(states)
    

