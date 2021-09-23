#!/usr/bin/env python3

import numpy as np
from behavior_tree.behavior_tree import *
from itertools import product

class PolicyToBT:
    def __init__(self, actions, conditions, policy, goal=None):

        self.actions = actions
        self.conditions = conditions # e.g. at_left, left_dirty, right_dirty

        self.policy = policy # e.g. (0,0,1,1,0,0,0,0)
        self.goal = goal # e.g. Both cells clean, states 7 or 8

        self.run()

    def run(self):

        self.behavior_tree = BehaviorTree('')

        # Add root '?'
        self.behavior_tree.root = Fallback()

        # Add goal condition at left-most position in tree, beneath root
        if self.goal:
            self.add_goal_subtree()

        # Add subtree for each action in policy
        self.behavior_tree.root = self.convert_policy_to_subtrees(self.behavior_tree.root)

        self.printBT(self.behavior_tree)

    def printBT(self, bt):
        bt.generate_nodes_list()
        for node in bt.nodes:
            print(node.label)
        print(len(bt.root.children))

    def add_goal_subtree(self):
        # Call this first after root is defined so it is left-most subtree
        
        # Add a sequence as a subtree root
        sequence = Sequence()
        self.behavior_tree.root.children.append(sequence)
        
        # Add goal condition(s) beneath the newest sequence
        for goal in self.goal:
            goal_condition_label, goal_status = goal[0], goal[1]
            goal_condition = Condition(goal_condition_label)
            if goal_status:
                self.behavior_tree.root.children[-1].children.append(goal_condition)
            elif not goal_status:
                decorator = NotDecorator()
                decorator.add_child(goal_condition)
                self.behavior_tree.root.children[-1].children.append(decorator)
        

    def convert_policy_to_subtrees(self, root):
        
        count = 0

        # Add a subtree for each action
        for bool_state in list(product([True,False],repeat=len(self.conditions))):
            
            # Add subtree sequence root
            sequence = Sequence()
            root.children.append(sequence)

            # Add conditions to describe each state
            for i in range(len(bool_state)):
                status = bool_state[i]
                condition_label = self.conditions[i]
                condition = Condition(condition_label)
                if status: # True
                    root.children[-1].children.append(condition)
                elif not status: # False
                    decorator = NotDecorator()
                    decorator.add_child(condition)
                    root.children[-1].children.append(decorator)
            
            # Add action associated with each state
            action_num = self.policy[count]
            action_label = self.actions[action_num]
            action = Action(action_label)
            root.children[-1].children.append(action)

            count += 1

        return root
            

if __name__ == "__main__":
    # Vacuum example
    actions = ['clean','move']
    conditions = ['at_left','left_dirty','right_dirty']
    policy = (0,0,1,1,0,0,0,0)
    goal = (('left_dirty',False),('right_dirty',False))
    p2bt = PolicyToBT(actions, conditions, policy, goal)

