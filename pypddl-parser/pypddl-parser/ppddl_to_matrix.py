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

def parse():
    usage = 'python3 main.py <DOMAIN> <INSTANCE>'
    description = 'pypddl-parser is a PDDL parser built on top of ply.'
    parser = argparse.ArgumentParser(usage=usage, description=description)

    parser.add_argument('domain',  type=str, help='path to PDDL domain file')
    parser.add_argument('problem', type=str, help='path to PDDL problem file')

    return parser.parse_args()

def getStateList():

    # Each element in the list will be a tuple? ()
    # if there are two predicates each with one variable (with two options): 4 states?
    # e.g. tuple = ((robot-at cell1),(dirty-at cell2))
    # state_list = []
    # for i in range(len(domain.predicates)):
    #     for j in range(len(problem.objects)):
    #         print(i)
    #         print(domain.predicates[i])
    #         state_list.append((str(domain.predicates[i]),problem.objects[j]))
    # print(state_list)

    single_state = []
    for i in range(len(domain.predicates)):
        print('Predicate is %s' % str(domain.predicates[i]))
        for variable_type in domain.types:
            #print(variable_type)
            if variable_type in str(domain.predicates[i]):
                print('This predicate has variable type %s' % variable_type)
                for value in problem.objects[variable_type]:

                    state_sub_list = [str(domain.predicates[i]),value,1]
                    single_state.append(state_sub_list)

    print(single_state)
    print(single_state[0][-1])

    states = []
    for tup in list(itertools.product([0,1],repeat=len(single_state))):
        # tup = (0,1,0,0) for example, representing (False, True, False, False)
        print(tup)

        for i in range(len(single_state)):
            full_state = single_state.copy()
            full_state[i][-1] = tup[i]
        states.append(full_state)
        print(full_state)

    #print(states)
    print(len(states))

def removeInvalidStates():

    # This function needs to remove states where predicates that can only have one True/False at a time, show up twice
    # An example of this is robot-at, where it shows up twice, but sometimes is True True or False False (not possible)
    # This is possible for dirty-at

    # For now we assume this repitition is okay because the solver will learn these states are not possible

    # However, if this poses an issue, I think I could rely on the init part of the problem file to determine 
    # which predicates can be True OR False and which can be True and True, False and False, or either or
    pass



if __name__ == '__main__':
    args = parse()

    domain  = PDDLParser.parse(args.domain)
    problem = PDDLParser.parse(args.problem)

    print(domain)
    print(problem)

    print("++++++++++++++++++++++++++++++")
    #print(domain.operators[0].effects)
    #print(domain.predicates[1])
    print("++++++++++++++++++++++++++++++")
    print(problem.objects[domain.types[0]])
    getStateList()
