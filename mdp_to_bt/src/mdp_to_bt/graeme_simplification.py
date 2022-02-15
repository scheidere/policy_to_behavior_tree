from sympy import *
from sympy.logic import SOPform
from sympy.core.symbol import *
from sympy.logic.boolalg import *


# Extract the conditions from the MDP, set them as an array of symbols
c = symbols('is_dirty, is_stuck, has_object, found_object')
print('c', c)

# For a particular action, enumerate all states where this action is selected
action = "move"
# minterms = [[0,0,0,0],[0,1,0,0],[0,1,1,0],[0,1,1,1],[0,1,0,0],[0,1,0,1],[1,0,0,0],[1,0,0,0],[1,0,1,1]]
minterms = [[0,0,0,0],[0,0,0,1],[0,0,1,0],[0,0,1,1],[0,1,0,0],[0,1,0,1],[0,1,1,0],[0,1,1,1],[1,1,0,1],]

# Add don't cares for states that are already covered by subtrees to the left
# This should simplify this subtree, since the logic doesn't care about these states
dontcares = [[1,1,1,0],[1,1,1,1],[1,1,0,0]]

# Get the sum-of-product representation
sop = SOPform(c, minterms, dontcares)
print('sop', sop)

# Simplify it
sop_simplify = to_dnf(sop,simplify=True)
print('sop_simplify', sop_simplify)

print(type(sop_simplify))

# Extract the subtrees from the logic
if type(sop_simplify) == Or:

	# "Or" found, therefore must be multiple subtrees
	terms = sop_simplify.args

elif type(sop_simplify) == And:

	# "And" found, therefore must be a single subtree
	# Put the subtree in a list so it is compatible with the following loop
	terms = [sop_simplify]

elif type(sop_simplify) == Not:

	# The tree is a single subtree with a Not decorator
	# Process this case separately
	terms = [] # So the following loop is skipped
	print("==============")
	print("subtree1(")
	name = str(sop_simplify.args[0])
	print("\t NOT", name)
	print("\t",action)
	print(")")

elif type(sop_simplify) == Symbol:

	# The tree is a single subtree with a single condition
	# Process this case separately
	terms = [] # So the following loop is skipped
	print("==============")
	print("subtree2(")
	name = str(sop_simplify)
	print("\t",name)
	print("\t",action)
	print(")")

elif type(sop_simplify) == BooleanFalse:

	# This case is impossible
	ERROR

elif type(sop_simplify) == BooleanTrue:

	# This action has no conditions
	terms = [] # So the following loop is skipped
	print("==============")
	print("subtree3(")
	print("\t",action)
	print(")")

else:
	ERROR



# Look at each subtree one at a time
for term in terms:

	# Each "term" is either an And, Condition, or Decorator+Condition
	# Treat each case slightly differently
	
	# Get the conditions for this subtree
	if type(term) == And:

		# "And" found, iterate through the list of conditions
		print("==============")
		print("subtree4(")
		
		conditions = term.args

	else:

		# "Not" or "Condition" found
		# Wrap it in a list, then proceed with the following loop
		print("==============")
		print("subtree5(")
		conditions = [term]

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

