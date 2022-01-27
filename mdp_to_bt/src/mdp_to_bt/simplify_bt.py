#!/usr/bin/env python3

import numpy as np
from behavior_tree.behavior_tree import *

class SimplifyBT:

    def __init__(self, tree):
        
        self.tree = tree
        #self.action_names = action_names
        #self.condition_names = condition_names # Order matters, same as appears in state

        self.initSimplifiedBT()
        self.simplifyBT()

    def initSimplifiedBT(self):

        simplified_bt = BehaviorTree('')
        simplified_bt.root = Fallback()
        self.simplified_bt = simplified_bt

    def simplifyBT(self):

        # Need to update this so that the list of subtrees gets updated (or check this is happening)
        # as pairs are simplified, we might still want to compare a simplified one to a not yet simplified one

        print('subtrees 0', self.tree.root.children)

        for i in range(len(self.tree.root.children)):
            print('*',i)
            #if i < len(self.tree.root.children):
            subtree_1 = self.tree.root.children[i]
            # else:
            #     return

            for j in range(len(self.tree.root.children)):

                if j != i: # Don't compare a subtree to itself
                    #if j < len(self.tree.root.children):
                    subtree_2 = self.tree.root.children[j]
                    # else:
                    #     return

                    # Attempt to combine pair of subtrees if they have the same action
                    if self.hasSameAction(subtree_1,subtree_2):

                        new_subtree = self.simplifySubtreePair(subtree_1,subtree_2)

                        print(new_subtree)

                        if new_subtree: # if no simplification possible, new_subtree will be None

                            # Remove subtree 1 and 2 from the behavior tree
                            #self.tree.root.children.remove(subtree_1)
                            #self.tree.root.children.remove(subtree_2)

                            #print('subtrees 1', self.tree.root.children, len(self.tree.root.children))

                            # Get new subtree location based on subtree 1 and 2 locations
                            # Ignoring for now since order doesn't matter (in this case)
                            # Just use left-most subtree position out of subtree 1 and 2
                            #most_left_index = min(i,j)
                            #print(most_left_index)

                            # Add new subtree at specified location
                            #self.tree.root.children.insert(most_left_index, new_subtree)
                            self.simplified_bt.root.children.append(new_subtree)

                            #print('subtrees 2', self.tree.root.children, len(self.tree.root.children))
                        else:
                            if subtree_1 not in self.simplified_bt.root.children:
                                self.simplified_bt.root.children.append(subtree_1)
                            if subtree_2 not in self.simplified_bt.root.children:
                                self.simplified_bt.root.children.append(subtree_2)

        #self.simplified_bt = self.tree


    def hasSameAction(self,subtree_1,subtree_2):

        action_1 = subtree_1.children[-1]
        action_2 = action_2 = subtree_2.children[-1]

        return action_2.label == action_1.label

    def convertPairToBinaryArray(self, subtree_pair):

        # Create array 
        # Rows are for each subtree
        # Cols are for each condition (always same number and order in each subtree)
        # Difference is which are alone (condition, denoted by 1) 
        # or beneath decorator nodes (denoted by 0)

        num_conditions = len(subtree_pair[0].children)-1 # only one action
        conditions_vs_decorators = np.zeros((2,num_conditions))

        for i in range(2):

            subtree = subtree_pair[i] # sequence node and child conditions, decorators, and action

            for j in range(len(subtree.children)-1):

                child = subtree.children[j] # a condition, decorator or action

                if isinstance(child, Condition):

                    #print(child.label + ' is condition')

                    conditions_vs_decorators[i,j] = 1

                elif isinstance(child, Decorator):

                    #print(child.label + child.children[0].label + ' is decorator')

                    conditions_vs_decorators[i,j] = 0

        return conditions_vs_decorators

    def simplifySubtreePair(self, subtree_1, subtree_2):

        # Use boolean logic (. is "and" and + is "or")
        # Distributivity of and over or: X . ( Y + Z ) = ( X . Y ) + ( X . Z)
        # Complementation 2 (notA + A = 1)

        subtree_pair = [subtree_1, subtree_2]

        # Initialize consolidated subtree to return after simplification
        output_subtree = Sequence()

        conditions_vs_decorators = self.convertPairToBinaryArray(subtree_pair)
        print(conditions_vs_decorators)
        #print(conditions_vs_decorators)

        num_conflicts = 0 # only one allowed, for simplification this way to be possible
        # Look for conflicts in condition/decorator appearance between subtrees in group
        for i in range(conditions_vs_decorators.shape[1]):

            print('i',i)
            print('num_conflicts', num_conflicts)

            # If number of conflicts is more than 1, this pair cannot be simplified in this way
            if num_conflicts > 1:
                return None

            col = conditions_vs_decorators[:,i]
            print('col',col)
            print(np.unique(col))

            # Conflict found between same condition in each subtree (e.g. decorator vs condition)
            if len(np.unique(col)) > 1: # np.unqiue returns list of unique vals, so if conflict it is [0,1] 

                print('conflict at i=',i)
                num_conflicts += 1
            else:

                # Keep condition/decorator for simplified/consolidated subtree (since it appears in both subtree in pair)
                output_subtree.children.append(subtree_pair[0].children[i])

        # Append action that defines this group (all subtrees in group have same action)
        # k will be last child, which should be the action
        output_subtree.children.append(subtree_pair[0].children[i+1]) 


        return output_subtree

   


if __name__ == '__main__':

    #bt = BehaviorTree('simplify_input_test.tree')
    bt = BehaviorTree('output_config/raw_output_bt.tree')

    simplify_bt = SimplifyBT(bt)

    final_bt = simplify_bt.simplified_bt

    final_bt.write_config('output_config/simplify_output_test.tree')
