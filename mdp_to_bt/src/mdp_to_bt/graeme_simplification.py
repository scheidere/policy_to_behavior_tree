from sympy import *
from sympy.logic import SOPform


# Extract the conditions from the MDP, set them as an array of symbols
c = symbols('is_dirty, is_stuck, has_object, found_object')
print('c', c)

# For a particular action, enumerate all states where this action is selected
action = "move"
minterms = [[0,0,0,0],[0,1,0,0],[0,1,1,0],[0,1,1,1],[0,1,0,0],[0,1,0,1],[1,0,0,0],[1,0,0,0],[1,0,1,1]]

# Add don't cares for states that are already covered by subtrees to the left
# This should simplify this subtree, since the logic doesn't care about these states
dontcares = [[1,1,1,0],[1,1,1,1],[1,1,0,0]]

# Get the sum-of-product representation
sop = SOPform(c, minterms, dontcares)
print('sop', sop)

# Simplify it
sop_simplify = to_dnf(sop,simplify=True)
print('sop_simplify', sop_simplify)

# Extract the subtrees
terms = sop_simplify.args

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

