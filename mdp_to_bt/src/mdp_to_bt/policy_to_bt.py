#!/usr/bin/env python3

import numpy as np
from behavior_tree.behavior_tree import *
from itertools import product

class PolicyToBT:
    def __init__(self, states, actions, policy, goal=None):

        self.states = states
        self.actions = actions
        self.policy = policy # e.g. (0,0,1,1,0,0,0,0)

        self.run()

    def run(self):

        self.behavior_tree = BehaviorTree('')

        # Add root '?'
        self.behavior_tree.root = Fallback()

        # Add subtree for each action in policy
        self.behavior_tree.root = self.convert_policy_to_subtrees(self.behavior_tree.root)

        self.printBT(self.behavior_tree)

    def printBT(self, bt):
        bt.generate_nodes_list()
        for node in bt.nodes:
            print(node.label)
        print(len(bt.root.children))

    def convert_policy_to_subtrees(self, root):

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
            
            # Add action associated with each state
            action_num = self.policy[i]
            action_with_params_term = self.actions[action_num]
            action_name = action_with_params_term[0]
            params = action_with_params_term[1] # dictionary
            print('params ', params.keys)
            action_label = action_name + '('
            for key in params.keys():
                variable = key[1:]
                value = params[key]
                action_label = action_label + variable + ': ' + value + ', '
            action_label = action_label[:-2] # remove extra comma and space
            action_label = action_label + ')'
            action = Action(action_label)
            root.children[-1].children.append(action)

        return root


if __name__ == "__main__":
    # Vacuum example

    #??? call this this main file
    p2bt = PolicyToBT(states, actions, policy)


