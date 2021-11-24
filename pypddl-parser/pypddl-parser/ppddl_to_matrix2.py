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

def getPossibleParamValues(action):

    # Currently only for action 0!

    param_value_dict = {}

    for p in action.params:
        for t in domain.types:
            if p.type == t:
                #print("Found parameter (%s) of type (%s) " % (p,t))
                #print('Possible values: ', problem.objects[t])
                param_value_dict[p.name] = problem.objects[t]

    print('Parameters/values: ', param_value_dict)
    return param_value_dict

def getParamCombos(action):

    combo_list = []

    # List of lists, where each sublist is a param's possible values
    param_values_lists = []
    for param in action.params:

        param_values_lists.append(problem.objects[param.type])

    #print(param_values_lists)

    # Get all combos with one elment from each list 
    # (so 3 element combos if 3 lists, i.e. 3 parameters)
    combo_list = list(itertools.product(*param_values_lists))

    #print(combo_list)

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

def preconditionSatisfied(action,combo):


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

        print('Action: ', action)

        # Get all possible combos of action param values
        param_combos = getParamCombos(action)

        # For combo in possible combos
        for combo in param_combos:

            # Init  two NxN arrays with zeros (one for P_a and one for R_a)
            p, r = np.zeros((N,N)), np.zeros((N,N))

            # if combo valid in precondition
            if preconditionSatisfied(action, combo):
                
                # Loop through start states s, indexing with i
                    # Loop through end states s', indexing with j
                        # Consider start state s and action with current combo params to get expected outcome
                        # If outcome matches s'
                            # Valid transition, add probability (1 if not specified), add reward if any
                        # Else not valid
                            # Invalid transtion, probability remains 0 as initialized, add reward maybe (???)

if __name__ == '__main__':
    args = parse()

    domain  = PDDLParser.parse(args.domain)
    problem = PDDLParser.parse(args.problem)

    print(domain)
    print(problem)

    getPandR()