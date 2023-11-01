#!/usr/bin/env python3

import numpy as np
from behavior_tree.behavior_tree import *
from itertools import product

class PolicyToBT:
    def __init__(self, states, actions, policy, goal=None):

        self.states = states
        self.actions = actions # actions with params list
        self.policy = policy # e.g. (0,0,1,1,0,0,0,0)

        self.run()

    def run(self):

        self.behavior_tree = BehaviorTree('')

        # Add root '?'
        self.behavior_tree.root = Fallback()

        # Add subtree for each action in policy
        self.behavior_tree.root = self.convert_policy_to_subtrees(self.behavior_tree.root)

        #self.printBT(self.behavior_tree)

    def printBT(self, bt):
        print('++++++++++++++++++\n')
        bt.generate_nodes_list()
        for node in bt.nodes:
            print(node.label)
        print('\nNumber of subtrees: ', len(bt.root.children))
        print('\n++++++++++++++++++\n')

    def convert_policy_to_subtrees(self, root):

        print("+++++++++++++++++++++++++++++++++++++++++++\n\n")

        # Add a subtree for each action
        for i in range(len(self.states)):

            state = self.states[i]
            
            # Add subtree sequence root
            sequence = Sequence()
            root.children.append(sequence)

            # Add conditions to describe each state
            for j in range(len(state)):
                term = state[j] # e.g. ['robot-at', 'left-cell', 1]
                condition_label = term[0] + '{' + term[1] + '}'
                condition = Condition(condition_label)
                if term[2]: # condition True
                    root.children[-1].children.append(condition)
                else: # False
                    decorator = NotDecorator()
                    decorator.add_child(condition)
                    root.children[-1].children.append(decorator)


                #print(condition_label)
            
            # Add action associated with each state
            action_num = self.policy[i]
            action_with_params_term = self.actions[action_num]

            action_name = action_with_params_term[0].name
            #print("action_with_params_term name "+ str(action_name) + "\n")
            params = action_with_params_term[1] # dictionary
            #print('params ', params.keys)
            action_label = None
            params_included = False
            if params.keys():
                #print("hiiiii")
                action_label = action_name
                for key in params.keys():
                    variable = key[1:]
                    #print('var '+ str(variable)+"\n")
                    value = params[key]
                    #print('val '+ str(value)+"\n")
                    if variable != 'x' or value !='x':
                        action_label = action_label + variable + ': ' + value + ', '
                        params_included = True
                if action_label and params_included:
                    action_label = action_label[:-2] # remove extra comma and space
                    print('um '+action_label+"\n")
                    action_label = '(' + action_label + ')'

            if not action_label: # no params
                action_label = action_name

            action = Action(action_label)
            root.children[-1].children.append(action)

            print(action_label)


        print("=============================================\n\n")



        return root


if __name__ == "__main__":
    # Vacuum example

    #??? call this this main file
    p2bt = PolicyToBT(states, actions, policy)


