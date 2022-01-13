#!/usr/bin/env python3

import numpy as np
from behavior_tree.behavior_tree import *

class SimplifyBT:

    def __init__(self, tree):
        
        self.tree = tree
        #self.action_names = action_names
        #self.condition_names = condition_names # Order matters, same as appears in state

        self.initSimplifiedBT()
        self.run()

    def initSimplifiedBT(self):

    	simplified_bt = BehaviorTree('')
    	simplified_bt.root = Fallback()
    	self.simplified_bt = simplified_bt

    def run(self):

    	# Group subtrees by the action in them
    	subtree_groups = self.groupSubtreesByAction()

    	# Find conflicting conditions/decorators, resolve conflicts, return new behavior tree
    	self.simplifyViaConflict(subtree_groups)


    def groupSubtreesByAction(self):

    	# Returns a dict with action names as the keys
    	# and a list of subtrees (each a behavior tree instance)
    	# that all have that action

    	#actions_already_considered = [] # Used to keep track of which actions have already been used to define a group

    	subtree_groups = {} # Dict of lists, where each list is a group of subtrees with the same action

    	# for action_name in self.action_names:

    	# 	# First check that action is in tree (might not have been learned as a part of the policy)
    	# 	if action_name in subtree_groups.keys():

	    # 		subtree_groups[action_name] = []

	    # 		for subtree in self.tree.root.children:

	    # 			subtree_action = self.getSubtreeActionName(subtree)

	    # 			if subtree_action_name == action_name:

	    # 				subtree_groups[action_name].append(subtree)

    	# return subtree_groups

    	for i in range(len(self.tree.root.children)):

    		subtree_1 = self.tree.root.children[i]

    		#num_children = len(subtree.children)

    		# Action is always last
    		#action_num = num_children-1
    		action_1 = subtree_1.children[-1] # Does -1 work?, if not use num_children and action_num commented above

    		if action_1 not in subtree_groups.keys(): # Ensure action not already considered
    			subtree_groups[action_1] = [subtree_1]

    			for j in range(len(self.tree.root.children)):

    				if j != i: # Don't compare a subtree to itself
	    				subtree_2 = self.tree.root.children[j]

	    				action_2 = subtree_2.children[-1]

	    				if action_2 == action_1:
	    					subtree_groups[action_1].append(subtree_2)


    	return subtree_groups


    # def getSubtreeActionName(self,subtree):

    # 	for child in subtree.root.children: # issue? subtree might be just a node rather than full bt, get rid of root?
    # 		if isinstance(child, Action):
    # 			return child.label

    def getSubtreeAction(self,subtree): # don't actually use this, maybe delete

    	for child in subtree.children:
    		if isinstance(child, Action):
    			return child

    def combineSameActionSubtrees(self,subtree_group):

    	# Consolidate subtrees with the same action via comparison of contitions and decorators between them

    	# Initialize consolidated subtree to return after simplification
    	output_subtree = Sequence()

    	# Rows are for each subtree
    	# Cols are for each condition (always same number and order in each subtree)
    	# Difference is which are alone or beneath decorator nodes
    	conditions_vs_decorators = np.zeros((len(subtree_group),len(subtree_group[0].children)))

    	for i in range(len(subtree_group)):

    		subtree = subtree_group[i]

    		for j in range(len(subtree.children)):

    			child = subtree.children[j]

    			if isinstance(child, Condition):

    				conditions_vs_decorators[i,j] = 1

    			elif isinstance(child, Decorator):

    				conditions_vs_decorators[i,j] = 0


    	# Look for conflicts in condition/decorator appearance between subtrees in group
    	for k in range(conditions_vs_decorators.shape[1]):

    		row = conditions_vs_decorators[:,k]

    		if len(np.unique(row)) == 1: # All same node type if len = 1

    			# This means the condition or decorator/condition remains the same for all subtrees in group
    			# Therefore, it is relevance and it should be retained

    			# Keep condition/decorator for simplified/consolidated subtree
    			output_subtree.children.append(subtree_group[0].children[k])

    		else:

    			# This means that there is a condition in one subtree 
    			# and a decorator above that condition in another
    			# Thus, this condition should not determine whether the action
    			# the subtrees have in common should happen
    			pass

    	# Append action that defines this group (all subtrees in group have same action)
    	# k will be last child, which should be the action
    	output_subtree.children.append(subtree_group[0].children[k]) 


    	return output_subtree

    def simplifyViaConflict(self, subtree_groups):

    	# Simplify behavior tree by comparing subtrees pertaining to the same action
    	# Combine simplified/consolidated subtrees into a single simplified behavior tree

    	for action in subtree_groups:

    		subtree_group = subtree_groups[action]

    		# Compare and consolidate subtrees in same group (grouped by action)
    		group_subtree = self.combineSameActionSubtrees(subtree_group)

    		# Add output group subtree to final simplified behavior tree
    		self.simplified_bt.root.children.append(group_subtree)



if __name__ == '__main__':

	bt = get from config file

	simplify_bt = SimplifyBT(bt)

	final_bt = simplify_bt.simplified_bt

	final_bt.write_config('output_config/test.tree')
