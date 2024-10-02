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
import copy


class Simplify:
    def __init__(self, states, actions_with_params, policy, domain, problem, ignore_dontcares, default_action_order, f):

        simplification_start_time = time.time()
        self.states = states
        self.actions = actions_with_params # all possible actions in the domain
        self.policy = policy
        self.action_nums_in_policy = list(np.unique(self.policy))
        self.domain = domain
        self.problem = problem
        self.simplification_runtime = None
        self.policy_to_bt_runtime = None
        self.ignore_dontcares = ignore_dontcares
        self.default_action_order = default_action_order

        self.f = f

        #f = open("/home/scheidee/Desktop/AURO_results/bla.txt", "w+") # for reorder debugging
        self.f.write("action nums in policy: " + str(self.action_nums_in_policy)+"\n")
        self.f.write("actions: \n")
        for i in range(len(self.actions)):
            #print(action)
            action = self.actions[i][0]
            self.f.write(str(i) + ": " + str(action)+"\n")
        #f.write("actions: " + str(self.actions)+"\n")
        #input("actions above")

        self.reorder_actions()
        f.write("action nums in policy 2: " + str(self.action_nums_in_policy)+"\n")

        # simplification_start_time = time.time()
        # Get condition names with parameters! E.g. ['robot-at', {'?x': 'left-cell'}]
        self.conditions = self.getConditionsWithParamsList()

        self.run(simplification_start_time)
        #self.test()

    def reorder_actions(self):

        print('action order default', self.action_nums_in_policy)
        reordered_action_nums_in_policy = copy.deepcopy(self.action_nums_in_policy)
        if not self.default_action_order: # Reorder actions randomly
            print('Reordering actions to determine whether it changes simplification results')
            while self.action_nums_in_policy==reordered_action_nums_in_policy:
                random.shuffle(reordered_action_nums_in_policy)

            # Now with new order, save it
            self.action_nums_in_policy = reordered_action_nums_in_policy

        print('reordered action order', self.action_nums_in_policy)

        # Must reorder actions list as well to match the new action num order???
        # new_actions = []
        # for n in self.action_nums_in_policy:
        #     #self.f.write(str(self.actions[n][0]))
        #     new_actions.append(self.actions[n])


        # self.actions = new_actions

        # self.f.write("actions: \n")
        # for i in range(len(new_actions)):
        #     action = new_actions[i][0]
        #     self.f.write(str(i) + ": " + str(action)+"\n")



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

        #print(conditions_with_params)
        return conditions_with_params


    def setConditionSymbols(self):

        conds_string = ''

        for i in range(len(self.conditions)):

            string = ''
            lst = list(self.conditions[i][1].values())
            for j in range(len(lst)):
                el = lst[j]
                if j > 0:
                    string += ','
                string += el

            #cond = self.conditions[i][0] + '_' + string
            cond = self.conditions[i][0] + '{' + string + '}'

            conds_string += cond

            if i <= len(self.conditions)-2:
                conds_string += ', '

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

    def getConditionsFromOR(self, or_sop_simplify):

        # There is a sympy issue where if the term only has one element, it counts as 0 args
        # This means that single arg terms get lost

        self.f.write("----In getConditionsFromOR----\n")

        # Given sop_simply with at least one OR | in it
        # return list of all conditions that appear at least once (not per term, just at all)
        conditions = []
        for term in or_sop_simplify.args:
            self.f.write("term: %s\n" %term)
            self.f.write("term.args: %s\n" %str(len(term.args)))
            self.f.write("c args: \n")

            if not len(term.args): # Found bug where single arg gets counted as 0
                self.f.write("0\n")
                c = term
                if c not in conditions:
                    conditions.append(c)
            elif len(term.args) == 1: # This seems to be !condition cases
                # An issue arises where term.args returns only condition, without the "!"
                c = term
                if c not in conditions:
                    conditions.append(c)
            else:
                self.f.write(str(len(term.args))+"\n")
                for c in term.args:
                    self.f.write(str(c) + "\n")
                    if c not in conditions:
                        conditions.append(c)

        self.f.write("----Out of getConditionsFromOR\n")


        return conditions

    def getConditionsInAllORTerms(self, all_conds, or_terms):

        self.f.write("!!!!! In get getConditionsInAllORTerms !!!!!\n")
        self.f.write('all_conds ' + str(all_conds) + "\n")
        self.f.write('or_terms ' + str(or_terms) + "\n")

        common_conditions = []
        condition_in_all_terms = True
        for c in all_conds:
            #self.f.write('idk' + str(c)+"\n")
            for term in or_terms:
                if not (len(term.args)): # Checking for 0 len bug
                    c_in_term = c == term

                elif len(term.args) == 1: # This seems to be !condition cases
                    # An issue arises where term.args returns only condition, without the "!"
                    c_in_term = c == term
                else:
                    c_in_term = c in term.args
                #self.f.write("c vs term "+str(c) + " " +str(term.args) + str(c_in_term) + "\n")
                if not c_in_term:
                    break

            #self.f.write('y '+ str(c_in_term) + " " + str(c) + "\n")
            if c_in_term:
                #self.f.write('yep' + str(c)+"\n")
                common_conditions.append(c)


        self.f.write("!!! Out of common conditions function !!!\n")


        return common_conditions

    def getUniqueConditionListsInORTerms(self, common_conditions, or_terms):

        # Record conditions that are not common to all terms, every term

        self.f.write("In unique func\n")

        unique_condition_lists = [] # Will be a list of lists, each list of one or more condition terms

        for term in or_terms:
            term_list = []

            # if not len(term.args): # Found bug where single arg gets counted as 0
            #     c = term
            #     if c not in common_conditions:
            #         term_list.append(c)
            # elif len(term.args) == 1: # This seems to be !condition cases
            #     # An issue arises where term.args returns only condition, without the "!"
            #     c = term
            #     if c not in common_conditions:
            #         term_list.append(c)
            self.f.write("term: %s\n" %term)
            self.f.write("len(term.args): %s\n" %len(term.args))
            for c in term.args:
                self.f.write("c: %s\n" %c)
            if not len(term.args) or len(term.args)==1: # Found bug where single arg gets counted as 0
                self.f.write("0 or 1 " + str(term) +"\n")
                c = term
                if c not in common_conditions:
                    term_list.append(c)
            else:
                self.f.write(str(len(term.args))+"\n")
                for c in term.args:
                    self.f.write("0 hi\n")
                    if c not in common_conditions:
                        term_list.append(c)

            unique_condition_lists.append(term_list)

        self.f.write("End unique func\n")

        return unique_condition_lists


    def buildSubtree(self, sop_simplify, action):

        self.f.write("+++++++++++++=in buildSubtree for this action: %s\n" %action[0])

        # Create list to contain a single or multiple (if OR) subtrees
        new_subtrees = []

        # Extract the subtrees from the logic
        if type(sop_simplify) == Or:

            self.f.write("Or\n")


            # BUG FIX IN PROGRESS - COMMENTED OUT FOR MEETING 10/23
            self.f.write(str(sop_simplify)+"\n")

            # "Or" found, therefore must be multiple subtrees
            or_terms = sop_simplify.args 
            all_conds = self.getConditionsFromOR(sop_simplify)
            self.f.write('all conds ' + str(all_conds) + "\n")

            common_conditions = self.getConditionsInAllORTerms(all_conds, or_terms)
            self.f.write("common conditions: "+str(common_conditions)+"\n")

            unique_condition_lists = self.getUniqueConditionListsInORTerms(common_conditions, or_terms)

            self.f.write("unique_condition_lists: " + str(unique_condition_lists) + "\n")

            sequence_node = Sequence()
            self.f.write("Created sequence node: " + str(sequence_node) + "\n")

            # Add common condition nodes beneath the sequence node
            for condition in common_conditions:

                self.f.write("condition: %s\n" %str(condition))

                sequence_node = self.createConditionNode(condition, sequence_node)

            self.f.write("Checking sequence node children (common conditions)...\n")
            self.f.write("++++Sequence node has the following children:\n")
            for child in sequence_node.children:
                self.f.write(child.label + "\n")
            self.f.write("Done listing child nodes for sequence++++\n")

            # Add unique conditions lists to fallback
            fallback_node = Fallback()
            for lst in unique_condition_lists:
                new_sequence_node = Sequence()
                if len(lst) == 1:
                    fallback_node = self.createConditionNode(lst[0], fallback_node)
                else:
                    for c in lst:
                        new_sequence_node = self.createConditionNode(c, new_sequence_node)
                    
                    fallback_node.children.append(new_sequence_node)

            sequence_node.children.append(fallback_node)

            # Add the action beneath the sequence node last
            action_node = self.createActionNode(action)
            self.f.write("action_node: %s, %s\n" %(str(action_node),action_node.label))
            sequence_node.children.append(action_node) # Add action to subtree
            new_subtrees.append(sequence_node) # Add subtree to list
            
            #conditions = sop_simplify.args # This is wrong. Fix in progress above, currently commented out.
            self.f.write("+++++==== END ++++====")
            return new_subtrees
            #print('OR')


        elif type(sop_simplify) == And:

            self.f.write("And\n")
            self.f.write(str(sop_simplify))

            # "And" found, therefore must be a single subtree
            # Put the subtree in a list so it is compatible with the following loop
            #terms = [sop_simplify]
            conditions = sop_simplify.args
            #print('And')

        elif type(sop_simplify) == Not:

            self.f.write("Not\n")
            #print('Not')

            # The tree is a single subtree with a Not decorator
            # Process this case separately
            #terms = [] # So the following loop is skipped
            #terms = [sop_simplify]
            # print("==============")
            # print("subtree [single NOT] (")
            conditions = [sop_simplify]
            name = str(sop_simplify.args[0])
            # print("\t NOT", name)
            # print("\t",action[0].name, action[1]) #.name added
            # print(")")

        elif type(sop_simplify) == Symbol:

            self.f.write("Symbol\n")

            #print('Symbol')
            # The tree is a single subtree with a single condition
            # Process this case separately
            #terms = [] # So the following loop is skipped
            conditions = [sop_simplify] # single condition, ADDED due to missing action in final tree, aka missing subtree
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

            self.f.write("BooleanTrue\n")

            # This action has no conditions
            #terms = [] # So the following loop is skipped
            # print("==============")
            # print("subtree [always] (")
            # print("\t",action[0].name, action[1])
            # print(")")
            action_node = self.createActionNode(action)
            return [action_node] # don't need a sequence node, it would be redundant

        else:

            # This case should be impossible
            ERROR

        #self.f.write("Does Symbol case get here? 1\n")
        #self.f.write("terms: %s\n" %str(terms))

        # For creating a subtree for an action with conditions (Or, And, Not, Symbol) 
        # Create root of subtree, which is a sequence node
        sequence_node = Sequence()

        # Add condition nodes beneath the sequence node
        for condition in conditions:

            self.f.write("condition: %s\n" %str(condition))

            sequence_node = self.createConditionNode(condition, sequence_node)

        # Add the action beneath the sequence node last
        action_node = self.createActionNode(action)
        self.f.write("action_node: %s, %s\n" %(str(action_node),action_node.label))
        sequence_node.children.append(action_node) # Add action to subtree
        new_subtrees.append(sequence_node) # Add subtree to list

        # # Look at each term and build subtree
        # for term in terms:

        #     self.f.write("term: %s\n" %str(term))

        #     # Each "term" is either an And, Condition, or Decorator+Condition
        #     # Treat each case slightly differently

        #     # Create root of subtree, which is a sequence node
        #     sequence_node = Sequence()
            
        #     # Get the conditions for this subtree
        #     if type(term) == And:

        #         # "And" found, iterate through the list of conditions
        #         # print("==============")
        #         # print("subtree [multiple Conditions] (")
                
        #         conditions = term.args

        #     else:

        #         # "Not" or "Condition" found
        #         # Wrap it in a list, then proceed with the following loop
        #         # print("==============")
        #         # print("subtree [single Condition/Not] (")
        #         conditions = [term]

        #     # Look at each condition one at a time
        #     for condition in conditions:

        #         self.f.write("condition: %s\n" %str(condition))

        #         sequence_node = self.createConditionNode(condition, sequence_node)

        # # Look at each subtree one at a time
        # for term in terms:

        #     self.f.write("term: %s\n" %str(term))

        #     # Each "term" is either an And, Condition, or Decorator+Condition
        #     # Treat each case slightly differently

        #     # Create root of subtree, which is a sequence node
        #     sequence_node = Sequence()
            
        #     # Get the conditions for this subtree
        #     if type(term) == And:

        #         # "And" found, iterate through the list of conditions
        #         # print("==============")
        #         # print("subtree [multiple Conditions] (")
                
        #         conditions = term.args

        #     else:

        #         # "Not" or "Condition" found
        #         # Wrap it in a list, then proceed with the following loop
        #         # print("==============")
        #         # print("subtree [single Condition/Not] (")
        #         conditions = [term]

        #     # Look at each condition one at a time
        #     for condition in conditions:

        #         self.f.write("condition: %s\n" %str(condition))

        #         # Determine if it is positive or negative
        #         if type(condition) == Not:

        #             # Not decorator with condition #
        #             #name = str(condition.args[0])
        #             #print("\t NOT", name)

        #             # Get condition label
        #             condition_label = str(condition.args[0])

        #             # Make condition node
        #             condition_node = Condition(condition_label)

        #             # Make decorator node
        #             decorator_node = NotDecorator()
        #             decorator_node.add_child(condition_node)

        #             # Add decorator to subtree
        #             sequence_node.children.append(decorator_node)

        #         else:

        #             # Condition #

        #             # Get condition label
        #             condition_label = str(condition)
        #             condition_node = Condition(condition_label)
        #             #print("\t",condition_label)

        #             # Add condition to subtree
        #             sequence_node.children.append(condition_node)



            # Add the action at the end of the subtree
            # print("\t",action[0].name, action[1])
            # print(")")

            # self.f.write("Does Symbol case get here? 2\n")

            # action_node = self.createActionNode(action)
            # self.f.write("action_node: %s, %s\n" %(str(action_node),action_node.label))

            # # Add action to subtree
            # sequence_node.children.append(action_node)

            # new_subtrees.append(sequence_node)

            # self.f.write("=========end buildSubtree==========\n")

        #print(len(new_subtrees))
        #print(new_subtrees)
        self.f.write("+++++out+++++\n")
        return new_subtrees    

        # print('++++++++++++++++++++++++++end')

    def createConditionNode(self, condition, node):

        # Determine if it is positive or negative
        if type(condition) == Not:

            # Not decorator with condition #
            #name = str(condition.args[0])
            #print("\t NOT", name)

            # Get condition label
            condition_label = str(condition.args[0])
            print('cond label 1 ' + condition_label + "\n")

            # Make condition node
            condition_node = Condition(condition_label)

            # Make decorator node
            decorator_node = NotDecorator()
            decorator_node.add_child(condition_node)

            # Add decorator to subtree
            node.children.append(decorator_node)

        else:

            # Condition #

            # Get condition label
            condition_label = str(condition)
            print('cond label 2 ' + condition_label + "\n")
            condition_node = Condition(condition_label)
            #print("\t",condition_label)

            # Add condition to subtree
            node.children.append(condition_node)

        return node

    def createActionNode(self, action):

        # Get action label with parameters
        action_name = action[0].name
        params = action[1]
        action_label = None
        params_included = False
        if params.keys():
            action_label = action_name
            for key in params.keys():
                variable = key[1:]
                value = params[key]
                if variable != 'x' or value !='x':
                    action_label = action_label + variable + ': ' + value + ', '
                    params_included = True
            if action_label and params_included:
                action_label = action_label[:-2] # remove extra comma and space
                action_label = '(' + action_label + ')'

        if not action_label: # no params
            action_label = action_name

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

    # def evaluateBTCompactness(self, bt):

    #     print('EVALUATING BT COMPACTNESS')

    #     # Determine complexity of the sentence "reading" the bt results in
    #     # Count total number of nodes
    #     # Count conditions and actions

    #     # Populate bt.nodes
    #     bt.generate_nodes_list()

    #     print('len(bt.nodes)',len(bt.nodes))

    #     total_num_nodes = 0
    #     num_action_nodes = 0
    #     num_condition_nodes = 0
    #     for node in bt.nodes:
    #         total_num_nodes+=1
    #         print(node.label)
    #         if isinstance(node,Action):
    #             print(node.label, 'is Action')
    #             num_action_nodes+=1
    #         elif isinstance(node,Condition):
    #             print(node.label,'is Condition')
    #             num_condition_nodes+=1

    #     print("Total number of nodes: %d" %total_num_nodes)
    #     print("Total number of action nodes: %d" %num_action_nodes)
    #     print("Total number of condition nodes: %d" %num_condition_nodes)

    #     print('DONE WITH COMPACTNESS EVAL')



    def run(self, simplification_start_time):

        # Create list for subtrees
        subtrees = []

        # I think the dont_cares determination is more complex
        # Am i looping the wrong way?

        # Extract the conditions from the MDP, set them as an array of symbols
        c = self.setConditionSymbols()
        # print('c: ', c)

        # dontcares = []
        if self.ignore_dontcares:
            dontcares = self.findInitialDontCares()
        else:
            print('we care now 1')
            dontcares = [] # Not removing dontcares, so don't need to find them
        #print('init # dontcares', len(dontcares))
        #input('look up 1')

        #print('self.actions', self.actions)
        self.f.write("========run() debug==============\n")
        ##for i in range(len(self.policy)):
        #for i in range(len(self.actions)):
        first_iter = True
        for action_num in self.action_nums_in_policy:

            #print('i ', i)

            #action = self.actions[i]
            # print('action', action)
            action = self.actions[action_num]

            self.f.write(str(action[0])+"\n")

            # print('action: ', action)

            # action_num = self.getActionNum(action)
            #print('action_num', action_num)

            #print('action num', action_num)

            #if i >= 1:
            if not first_iter:
                if self.ignore_dontcares:
                    dontcares += prev_minterms
                else:
                    print('we care now 2')
                    dontcares = [] # Not removing dontcares, so don't need to add them

            #print('new # dontcares', len(dontcares))
            #input('look up 2')

            #action_num = self.policy[i]
            #print(self.policy)
            #print(action_num)

            # Get list of numeric states the policy states action a should be taken in
            minterms = self.getActionStates(action_num) # switch to take action num
            #print('minterms', minterms)
            self.f.write(str(minterms)+"\n")

            prev_minterms = minterms

            # print('minterms ', minterms)
            # print('dontcares ', dontcares)

            # Get the sum-of-product representation
            sop = SOPform(c, minterms, dontcares)
            # print('sop', sop)
            #print('sop', sop)
            self.f.write("sop: "+str(sop)+"\n")

            # Simplify it
            sop_simplify = to_dnf(sop,simplify=True,force=True)
            #print('sop_simplify', sop_simplify)
            # print('sop_simplify', sop_simplify)
            #print('sop_simplify',sop_simplify)
            self.f.write("sop_simplify: "+str(sop_simplify)+"\n")

            #THE PROBLEM IS HERE, I think my BT building method loses disarm for the marine domain for "final" and another for "reorder"
            subtree_list = self.buildSubtree(sop_simplify,action)
            #print('out', subtree_list)
            #print('len',len(subtree_list))
            # if len(subtree_list) < 2:
            #     subtrees.append(subtree_list[0])
            # else:
            #     subtrees.extend(subtree_list)

            #self.f.write("\n\n++++++++++++++++\n\n")
            for s in subtree_list:
                subtrees.append(s)
                print(subtrees)
                print(len(subtrees))
                #self.f.write("%s\n" %str(s))

            first_iter = False

        self.simplification_runtime = time.time() - simplification_start_time

        policy_to_bt_start_time = time.time()
        self.bt = self.buildFullTree(subtrees)
        self.policy_to_bt_runtime = time.time() - policy_to_bt_start_time
        self.printBT(self.bt)
        #self.bt.evaluateBTCompactness()
        



