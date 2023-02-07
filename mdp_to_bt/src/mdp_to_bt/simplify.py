#!/usr/bin/env python3

from sympy import *
from sympy.logic import SOPform
from sympy.core.symbol import *
from sympy.logic.boolalg import *
import numpy as np
from behavior_tree.behavior_tree import *
from itertools import product
import itertools
import time



class Simplify:
    def __init__(self, states, actions_with_params, policy, domain, problem):

        simplification_start_time = time.time()
        self.states = states
        self.actions = actions_with_params # all possible actions in the domain
        self.policy = policy
        self.action_nums_in_policy = list(np.unique(self.policy))
        self.domain = domain
        self.problem = problem
        self.simplification_runtime = None
        self.policy_to_bt_runtime = None

        # simplification_start_time = time.time()
        # Get condition names with parameters! E.g. ['robot-at', {'?x': 'left-cell'}]
        self.conditions = self.getConditionsWithParamsList()

        self.run(simplification_start_time)
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

        # Get correct indices from the original action list, which was used to denote the policy with action nums

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

    def findInitialDontCares(self):

        #print("findInitialDontCares")
        
        valid_states = [];

        # Get all states that have an associated action
        for idx in range(len(self.states)):

            valid_states.append(self.getNumericalState(self.states[idx]))

        #print('valid_states', valid_states)

        # Find all states that are not in the above states list
        dontcares = []
        
        for tup in list(itertools.product([0,1],repeat=len(valid_states[0]))):
            # tup = (0,1,0,0) for example, representing (False, True, False, False)
            #print(tup)

            current_state = list(tup)

            # Distribute True/False options for each condition (predicate) in a state
            if not (current_state in valid_states):
                dontcares.append(current_state)

        #print('dontcares', dontcares)

        return dontcares

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

    def buildSubtree(self, sop_simplify, action):

        # Create list to contain a single or multiple (if OR) subtrees
        new_subtrees = []

        # Extract the subtrees from the logic
        if type(sop_simplify) == Or:

            # "Or" found, therefore must be multiple subtrees
            terms = sop_simplify.args
            #print('OR')

        elif type(sop_simplify) == And:

            # "And" found, therefore must be a single subtree
            # Put the subtree in a list so it is compatible with the following loop
            terms = [sop_simplify]
            #print('And')

        elif type(sop_simplify) == Not:

            #print('Not')

            # The tree is a single subtree with a Not decorator
            # Process this case separately
            #terms = [] # So the following loop is skipped
            terms = [sop_simplify]
            # print("==============")
            # print("subtree [single NOT] (")
            name = str(sop_simplify.args[0])
            # print("\t NOT", name)
            # print("\t",action[0].name, action[1]) #.name added
            # print(")")

        elif type(sop_simplify) == Symbol:

            #print('Symbol')
            # The tree is a single subtree with a single condition
            # Process this case separately
            terms = [] # So the following loop is skipped
            # print("==============")
            # print("subtree [single Condition] (")
            name = str(sop_simplify)
            # print("\t",name)
            # print("\t",action[0].name, action[1])
            # print(")")

        elif type(sop_simplify) == BooleanFalse:

            # This case should be impossible
            ERROR

        elif type(sop_simplify) == BooleanTrue:

            # This action has no conditions
            terms = [] # So the following loop is skipped
            # print("==============")
            # print("subtree [always] (")
            # print("\t",action[0].name, action[1])
            # print(")")
            action_node = self.createActionNode(action)
            return [action_node]

        else:

            # This case should be impossible
            ERROR



        # Look at each subtree one at a time
        for term in terms:

            # Each "term" is either an And, Condition, or Decorator+Condition
            # Treat each case slightly differently

            # Create root of subtree, which is a sequence node
            sequence_node = Sequence()
            
            # Get the conditions for this subtree
            if type(term) == And:

                # "And" found, iterate through the list of conditions
                # print("==============")
                # print("subtree [multiple Conditions] (")
                
                conditions = term.args

            else:

                # "Not" or "Condition" found
                # Wrap it in a list, then proceed with the following loop
                # print("==============")
                # print("subtree [single Condition/Not] (")
                conditions = [term]

            # Look at each condition one at a time
            for condition in conditions:

                # Determine if it is positive or negative
                if type(condition) == Not:

                    # Not decorator with condition #
                    #name = str(condition.args[0])
                    #print("\t NOT", name)

                    # Get condition label
                    condition_label = str(condition.args[0])

                    # Make condition node
                    condition_node = Condition(condition_label)

                    # Make decorator node
                    decorator_node = NotDecorator()
                    decorator_node.add_child(condition_node)

                    # Add decorator to subtree
                    sequence_node.children.append(decorator_node)

                else:

                    # Condition #

                    # Get condition label
                    condition_label = str(condition)
                    condition_node = Condition(condition_label)
                    #print("\t",condition_label)

                    # Add condition to subtree
                    sequence_node.children.append(condition_node)



            # Add the action at the end of the subtree
            # print("\t",action[0].name, action[1])
            # print(")")

            action_node = self.createActionNode(action)

            # Add action to subtree
            sequence_node.children.append(action_node)

            new_subtrees.append(sequence_node)

        #print(len(new_subtrees))
        #print(new_subtrees)
        return new_subtrees    

        # print('++++++++++++++++++++++++++end')

    def createActionNode(self, action):

        # Get action label with parameters
        action_name = action[0].name
        params = action[1]
        action_label = action_name + '('
        for key in params.keys():
            variable = key[1:]
            value = params[key]
            action_label = action_label + variable + ': ' + value + ', '
        action_label = action_label[:-2] # remove extra comma and space
        action_label = action_label + ')'
        #print('action label', action_label)

        # Make action node
        action_node = Action(action_label)
        #print(action_node)

        return action_node


    def buildFullTree(self, subtrees):

        bt = BehaviorTree('')
        bt.root = Fallback()

        #self.printBT(bt)

        for subtree in subtrees:

            #print('subtree', subtree)

            bt.root.children.append(subtree)
            #self.printBT(bt)

        return bt

    def printBT(self, bt):
        print('++++++++++++++++++\n')
        bt.generate_nodes_list()
        #print('!!!!!!!!!!!!!!!', type(bt.root))
        #print(bt.nodes)
        for node in bt.nodes:
            print(node.label)
        print('\nNumber of subtrees: ', len(bt.root.children))
        print('\n++++++++++++++++++\n')

    def run(self, simplification_start_time):

        # Create list for subtrees
        subtrees = []

        # I think the dont_cares determination is more complex
        # Am i looping the wrong way?

        # Extract the conditions from the MDP, set them as an array of symbols
        c = self.setConditionSymbols()
        # print('c: ', c)

        # dontcares = []
        dontcares = self.findInitialDontCares()
        #print('init dontcares', dontcares)

        #print('self.actions', self.actions)

        ##for i in range(len(self.policy)):
        #for i in range(len(self.actions)):
        first_iter = True
        for action_num in self.action_nums_in_policy:

            #print('i ', i)

            #action = self.actions[i]
            # print('action', action)
            action = self.actions[action_num]

            # print('action: ', action)

            # action_num = self.getActionNum(action)
            #print('action_num', action_num)

            #print('action num', action_num)

            #if i >= 1:
            if not first_iter:
                dontcares += prev_minterms

            #print('new dontcares', dontcares)

            #action_num = self.policy[i]
            #print(self.policy)
            #print(action_num)

            # Get list of numeric states the policy states action a should be taken in
            minterms = self.getActionStates(action_num) # switch to take action num
            #print('minterms', minterms)

            prev_minterms = minterms

            # print('minterms ', minterms)
            # print('dontcares ', dontcares)

            # Get the sum-of-product representation
            sop = SOPform(c, minterms, dontcares)
            # print('sop', sop)
            #print('sop', sop)

            # Simplify it
            sop_simplify = to_dnf(sop,simplify=True)
            #print('sop_simplify', sop_simplify)
            # print('sop_simplify', sop_simplify)
            #print('sop_simplify',sop_simplify)

            subtree_list = self.buildSubtree(sop_simplify,action)
            #print('out', subtree_list)
            #print('len',len(subtree_list))
            # if len(subtree_list) < 2:
            #     subtrees.append(subtree_list[0])
            # else:
            #     subtrees.extend(subtree_list)
            for s in subtree_list:
                subtrees.append(s)
                print(subtrees)
                print(len(subtrees))

            first_iter = False

        self.simplification_runtime = time.time() - simplification_start_time

        policy_to_bt_start_time = time.time()
        self.bt = self.buildFullTree(subtrees)
        self.policy_to_bt_runtime = time.time() - policy_to_bt_start_time
        self.printBT(self.bt)
        



