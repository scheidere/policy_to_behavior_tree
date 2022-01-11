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

    def exportBT(bt, include_nodes=None):  
        # Create a Word that represents this BT  
        # But only include nodes that include_nodes[node_idx]==True

        # Setup a stack data structure (similar to nodes_worklist)
        # Do this for both keeping track of nodes and for number of tabs
        nodes_stack = []
        level_stack = []
        nodes_stack.append(bt.root) #push
        level_stack.append(0)

        char_list = []

        prev_level = 0

        num_include_nodes = 0
        for i in include_nodes:
            if i:
                num_include_nodes += 1
        # print("exportBT num_include_nodes", num_include_nodes)
        # print("exportBT len(include_nodes)", len(include_nodes))

        # Do the traversal, using the stack to help
        while len(nodes_stack) != 0:

            # Pop a node off the stack
            current_node = nodes_stack.pop() #pop
            level = level_stack.pop()
            # print(current_node.__class__.__name__)

            if include_nodes == None:
                include_node = True
            else:
                node_index = bt.nodes.index(current_node)
                include_node = include_nodes[node_index]

            if include_node:
                if level > prev_level:
                    char_list.append(Character('('))
                elif level < prev_level:
                    for i in xrange(prev_level-level):
                        char_list.append(Character(')'))
                
                label = bt.get_node_text(current_node)
                char_list.append(Character(label))

                # Add all children to the stack
                for child_idx in reversed(range(len(current_node.children))):
                    nodes_stack.append(current_node.children[child_idx]) #push
                    level_stack.append(level+1)

                prev_level = level

        while prev_level > 0:
            char_list.append(Character(')'))
            prev_level -= 1

        # Close the file
        new_word = Word(char_list)
        return new_word

if __name__ == "__main__":
    # Vacuum example

    #??? call this this main file
    p2bt = PolicyToBT(states, actions, policy)


