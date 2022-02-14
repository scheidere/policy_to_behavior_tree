#!/usr/bin/env python3

from sympy import *
from sympy.logic import SOPform
import numpy as np
from itertools import product



class Simplify:
    def __init__(self, states, actions_with_params, policy, domain, problem):

        self.states = states
        self.actions = actions_with_params # action names with parameters
        self.policy = policy
        self.domain = domain
        self.problem = problem

        # Get condition names with parameters! E.g. ['robot-at', {'?x': 'left-cell'}]
        self.conditions = self.getConditionsWithParamsList()

        self.run()
        #self.test()

    def test(self):

        for state in states:
            print(state)
            print(self.getNumericalState(state))

        for condition_predicate in self.domain.predicates:
            self.getParams(condition_predicate)

        print(self.getConditionsWithParamsList())

    def dictproduct(self, dct):
        for t in product(*dct.values()):
            yield dict(zip(dct.keys(), t))

    def getParams(self, condition):

        # condition is a predicate

        #print('+++++++++hello+++++++++')

        combo_list = []

        # Dict of param name keys, with possible value list as arg for each
        param_values_dict = {}
        for param in condition._args:

            param_values_dict[param._name] = self.problem.objects[param.type]

        #print('param val dict', param_values_dict)

        # Get all combinations of param values in dictionaries, congregate in list
        combo_list = list(self.dictproduct(param_values_dict))

        #print('combo_list: ', combo_list)

        return combo_list

    def getConditionsWithParamsList(self):

        # Get all action/param combos

        # Initialize conditions with parameters list (used for reading policy)
        conditions_with_params = []

        # Loop through all conditions in domain
        for condition_predicate in self.domain.predicates:

            # Get all possible combos of condition parameter values
            param_combos = self.getParams(condition_predicate)

            for combo_dict in param_combos:

                #actions_with_params.append([action.name,combo_dict])
                conditions_with_params.append([condition_predicate.name,combo_dict])

        #print(actions_with_params)
        return conditions_with_params

    # def getConditions(self, domain):

    #     condition_names = []

    #     for pred in domain.predicates:

    #         #print(pred.name)
    #         condition_names.append(pred.name)

    #     return condition_names

    def setConditionSymbols(self):

        # Issue: symbols() does not like when cond has 

        conds_string = ''

        for i in range(len(self.conditions)):

            #print(self.conditions[i][1])
            #print(self.conditions[i][1].values)

            string = ''
            lst = list(self.conditions[i][1].values())
            for j in range(len(lst)):
                el = lst[j]
                if j > 0:
                    string += '_'
                string += el

            cond = self.conditions[i][0] + '_' + string

            #print('cond ', cond)

            conds_string += cond

            if i <= len(self.conditions)-2:
                conds_string += ', '

        #print(conds_string)

        c = symbols(conds_string)

        return c

    def getNumericalState(self, state):

        # Return 0/1 representation of a state
        # Length of state should be as long as the number of conditions

        # Init numeric state with zeros
        numeric_state = [int(n) for n in np.zeros((len(self.conditions))).tolist()]

        for cond in self.conditions:

            values_list = list(cond[1].values())

            for i in range(len(state)):

                term = state[i]

                if term[0] == cond[0] and term[1:-1] == values_list and term[2] == 1:

                    # Found True
                    numeric_state[i] = 1

        return numeric_state


    def getActionNum(self, action):

        for i in range(len(self.actions)):

            action_with_params = self.actions[i]

            if action_with_params == action:

                return i


    def getActionStates(self, action_num):

        # For a particular action, enumerate all states where this action is selected

        states_with_given_action = []
        #states_test = []

        # Get action num
        #action_num = self.getActionNum(action)

        # Find all state indexes in policy that have action num
        state_indices = [index for index, element in enumerate(self.policy) if element == action_num]

        for idx in state_indices:

            states_with_given_action.append(self.getNumericalState(self.states[idx]))
            #states_test.append(self.states[idx])

        return states_with_given_action #, states_test

    def test(self):

        # This test seems to pass

        # Extract the conditions from the MDP, set them as an array of symbols
        c = self.setConditionSymbols()
        print('c: ', c)

        print('Policy: ', self.policy)
        print('actions ', self.actions)

        for i in range(len(self.actions)):

            action = self.actions[i]

            #states_with_given_action = self.getActionStates(action)
            print('action ', action)
            print('i ',i)
            print('action num ', self.getActionNum(i))
            a = self.getActionStates(i)
            print('policy ', self.policy)
            print('all states ')
            for state in self.states:
                print(self.getNumericalState(state))
            #print('states ')
            # for state in b:
            #     print(state)
            print('states ', a)

    def buildSubtree(self, terms, action):

        # This is not set up for a BT yet, just printing as is for now

        # Look at each subtree one at a time
        for term in terms:

            print("==============")
            print("subtree(")

            # Get the conditions for this subtree
            conditions = term.args

            # Look at each condition one at a time
            for condition in conditions:

                # Determine if it is positive or negative
                if type(condition) == Not:

                    # Not decorator with condition
                    name = str(condition.args[0])

                    print("\t NOT", name)

                else:

                    # Condition
                    name = str(condition)
                    print("\t",name)

            # Add the action at the end of the subtree
            print("\t",action)

            print(")")


    def buildFullTree(self, subtrees):

        bt = BehaviorTree('')
        bt.root = Fallback()



    def run(self):

        # I think the dont_cares determination is more complex
        # Am i looping the wrong way?

        # Extract the conditions from the MDP, set them as an array of symbols
        c = self.setConditionSymbols()
        print('c: ', c)

        dontcares = []
        ##for i in range(len(self.policy)):
        for i in range(len(self.actions)):

            ##print('i ', i)

            action = self.actions[i]

            action_num = self.getActionNum(action)

            if i >= 1:
                dontcares += prev_minterms

            #action_num = self.policy[i]
            print(self.policy)
            print(action_num)

            # Get list of numeric states the policy states action a should be taken in
            minterms = self.getActionStates(action_num) # switch to take action num

            prev_minterms = minterms

            print('minterms ', minterms)
            print('dontcares ', dontcares)

            # Get the sum-of-product representation
            sop = SOPform(c, minterms, dontcares)
            print('sop', sop)

            # Simplify it
            sop_simplify = to_dnf(sop,simplify=True)
            print('sop_simplify', sop_simplify)

            # Extract the subtrees
            terms = sop_simplify.args

            self.buildSubtree(terms,action)



