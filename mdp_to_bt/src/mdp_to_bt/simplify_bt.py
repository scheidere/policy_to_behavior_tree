#!/usr/bin/env python3

import numpy as np
from behavior_tree.behavior_tree import *
from itertools import combinations

class SimplifyBT:

    CONDITION = 1
    DECORATOR = 0
    MISSING = 2

    def __init__(self, tree):
        
        self.tree = tree
        #self.action_names = action_names
        #self.condition_names = condition_names # Order matters, same as appears in state

        #self.initSimplifiedBT()
        self.simplifyBT()

    def initSimplifiedBT(self):

        simplified_bt = BehaviorTree('')
        simplified_bt.root = Fallback()
        self.simplified_bt = simplified_bt

    def printBT(self, bt):
        bt.generate_nodes_list()
        for node in bt.nodes:
            print(node.label)
        print(len(bt.root.children))

    def printSubtree(self, subtree):

        printable_subtree = BehaviorTree('')
        printable_subtree.root = subtree
        self.printBT(printable_subtree)

    def simplifyBT(self):

        # Need to update this so that the list of subtrees gets updated (or check this is happening)
        # as pairs are simplified, we might still want to compare a simplified one to a not yet simplified one
        
        found_simplification = True
        previous_bt = self.tree
        count = 0
        while found_simplification:

            print('\n\n')
            print('++++++ ITERATION %s ++++++' %count)
            print('\n\n')

            print('Number of subtrees: ', len(previous_bt.root.children))

            found_simplification = False


            unique_subtree_pairs = list(combinations(previous_bt.root.children,2))
            self.initSimplifiedBT()
            
            already_included = [] # subtrees already included
            #print('subtrees 0', self.tree.root.children)

            for subtree_1, subtree_2 in unique_subtree_pairs:

                # Attempt to combine pair of subtrees if they have the same action
                if self.hasSameAction(subtree_1,subtree_2):

                    #print(subtree_1.children[-1].label) # print action

                    new_subtree = self.simplifySubtreePair(subtree_1,subtree_2)
                    #print('TESTING')
                    #if new_subtree:
                    #    self.printSubtree(new_subtree)

                    if new_subtree: # if no simplification possible, new_subtree will be None

                        # Simplification occured, adding new subtree
                        found_simplification = True
                        self.simplified_bt.root.children.append(new_subtree)
                        #print('new subtree added', new_subtree)

                    else:
                        # Simplification did not occur, no new subtree to add

                        if subtree_1 not in already_included:
                            self.simplified_bt.root.children.append(subtree_1)
                            #print('adding subtree 1')
                            already_included.append(subtree_1)
                        else:
                            found_simplification = True # Did not keep subtree 1
                        if subtree_2 not in already_included:
                            self.simplified_bt.root.children.append(subtree_2)
                            #print('adding subtree 2')
                            already_included.append(subtree_2)
                        else:
                            found_simplification = True # Did not keep subtree 2

            # Add subtrees that cannot be simplified 
            # because they are the only subtrees with a certain action individually
            for a in range(len(previous_bt.root.children)):
                subtree_a = previous_bt.root.children[a]
                unique = True
                action_label_a = subtree_a.children[-1].label
                for b in range(len(previous_bt.root.children)):
                    subtree_b = previous_bt.root.children[b]
                    action_label_b = subtree_b.children[-1].label

                    if action_label_a == action_label_b and a != b:
                        #print('not unique')
                        unique = False

                if unique: # add subtree
                    #print(unique)
                    self.simplified_bt.root.children.append(subtree_a)

            # Reset previous bt
            previous_bt = self.simplified_bt

            print('CURRENT SIMPLIFIED BEHAVIOR TREE: ')
            self.printBT(previous_bt)

            input('Ready to continue? ')
            count+=1

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

        #num_conditions = len(subtree_pair[0].children)-1 # only one action
        #conditions_vs_decorators = np.zeros((2,num_conditions))

        conditions_vs_decorators = {}

        # 1 = condition, 0 = decorator, 2 = unknown/irrelevant


        subtree_1 = subtree_pair[0]
        subtree_2 = subtree_pair[1]

        for i in range(len(subtree_1.children)-1):

            child = subtree_1.children[i]

            if isinstance(child, Condition):

                label = child.label
                conditions_vs_decorators[label] = [self.CONDITION,self.MISSING]

            elif isinstance(child, Decorator):

                label = child.children[0].label
                conditions_vs_decorators[label] = [self.DECORATOR,self.MISSING]


        for j in range(len(subtree_2.children)-1):

            child = subtree_2.children[j]

            if isinstance(child, Condition):

                label = child.label
                if label in conditions_vs_decorators.keys():
                    conditions_vs_decorators[label][1] = self.CONDITION
                else:
                    conditions_vs_decorators[label] = [self.MISSING,self.CONDITION]

            elif isinstance(child, Decorator):

                label = child.children[0].label
                if label in conditions_vs_decorators.keys():
                    conditions_vs_decorators[label][1] = self.DECORATOR
                else:
                    conditions_vs_decorators[label] = [self.MISSING,self.DECORATOR]


        # for i in range(2):

        #     subtree = subtree_pair[i] # sequence node and child conditions, decorators, and action

        #     for j in range(len(subtree.children)-1):

        #         child = subtree.children[j] # a condition, decorator or action

        #         if isinstance(child, Condition):

        #             #print(child.label + ' is condition')

        #             conditions_vs_decorators[i,j] = 1

        #         elif isinstance(child, Decorator):

        #             #print(child.label + child.children[0].label + ' is decorator')

        #             conditions_vs_decorators[i,j] = 0

        return conditions_vs_decorators

    def getConditionFromSubtree(self, subtree, condition_label):

        for child in subtree.children:

            if isinstance(child, Condition):

                label = child.label

            elif isinstance(child, Decorator):

                label = child.children[0].label

            if label == condition_label:
                return child # condition or decorator with given label


    def simplifySubtreePair(self, subtree_1, subtree_2):

        # Use boolean logic (. is "and" and + is "or")
        # Distributivity of and over or: X . ( Y + Z ) = ( X . Y ) + ( X . Z)
        # Complementation 2 (notA + A = 1)

        print('+++++++++++++++++++++')
        print('Input subtree 1: \n')
        self.printSubtree(subtree_1)
        print('Input subtree 2: \n')
        self.printSubtree(subtree_2)
        print('+++++++++++++++++++++')

        subtree_pair = [subtree_1, subtree_2]

        # Initialize consolidated subtree to return after simplification
        output_subtree = Sequence()

        conditions_vs_decorators = self.convertPairToBinaryArray(subtree_pair)
        print(conditions_vs_decorators)

        num_known_conflicts = 0 # only one allowed, for simplification this way to be possible
        num_unknown_conflicts = 0

        # Look for conflicts in condition/decorator appearance between subtrees in group
        for node_label in conditions_vs_decorators.keys():

            # np.unqiue returns list of unique vals, so if conflict it is [unique_val_1, unique_val_2]
            unique_values = np.unique(conditions_vs_decorators[node_label])
            #print('unique_values: ', unique_values)
            #print(len(unique_values))
            if len(unique_values) > 1: # [0,1], [0,2], [1,2]

                if list(unique_values) == [0,1]: # includes [0,1] and [1,0] cases because np.unique has same order for both
                    # Known conflict found
                    num_known_conflicts += 1

                else: # [0,2],[2,0],[1,2],[2,1]
                    num_unknown_conflicts += 1

            else: 
                # Keep condition/decorator for simplified/consolidated subtree (since it appears in both subtree in pair)
                node_to_keep = self.getConditionFromSubtree(subtree_pair[0], node_label)
                output_subtree.children.append(node_to_keep)

        # Append action that defines this group (all subtrees in group have same action)
        # k will be last child, which should be the action
        action = subtree_pair[0].children[-1]
        output_subtree.children.append(action) 

        # If number of known conflicts is more than 1, this pair cannot be simplified in this way
        if num_known_conflicts > 1:
            print('====================')
            print('Pair cannot be simplified, more than one known conflict found')
            print('====================')
            return None

        # If number of known conflicts is at least 1 and there are any unknown conflicts, this pair cannot be simplified
        elif num_known_conflicts == 1 and num_unknown_conflicts > 0:
            print('====================')
            print('Pair cannot be simplified, a known conflict AND unknown conflict were found')
            print('====================')
            return None

        # num_conflicts = 0 # only one allowed, for simplification this way to be possible
        # # Look for conflicts in condition/decorator appearance between subtrees in group
        # for i in range(conditions_vs_decorators.shape[1]):

        #     #print('i',i)
        #     #print('num_conflicts', num_conflicts)

        #     # If number of conflicts is more than 1, this pair cannot be simplified in this way
        #     if num_conflicts > 1:
        #         return None

        #     col = conditions_vs_decorators[:,i]
        #     #print('col',col)
        #     #print(np.unique(col))

        #     # Conflict found between same condition in each subtree (e.g. decorator vs condition)
        #     if len(np.unique(col)) > 1: # np.unqiue returns list of unique vals, so if conflict it is [0,1] 

        #         #print('conflict at i=',i)
        #         num_conflicts += 1
        #     else:

        #         # Keep condition/decorator for simplified/consolidated subtree (since it appears in both subtree in pair)
        #         output_subtree.children.append(subtree_pair[0].children[i])

        # # Append action that defines this group (all subtrees in group have same action)
        # # k will be last child, which should be the action
        # output_subtree.children.append(subtree_pair[0].children[i+1]) 

        print('====================')
        print('Output subtree: \n')
        self.printSubtree(output_subtree)
        print('Known conflicts found: ', num_known_conflicts)
        print('Unknown conflicts found: ', num_unknown_conflicts)
        print('====================')

        return output_subtree

   


if __name__ == '__main__':

    #bt = BehaviorTree('simplify_input_test.tree')
    bt = BehaviorTree('output_config/raw_output_bt.tree')

    simplify_bt = SimplifyBT(bt)

    final_bt = simplify_bt.simplified_bt

    final_bt.write_config('output_config/simplify_output_test.tree')
