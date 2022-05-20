from policy_to_bt import *
from simplify import * # NEW WAY
from evaluate_mdp_policy import *
from ppddl_to_matrices import *

import mdptoolbox
import numpy as np
import mdptoolbox.example

import argparse
import sys
sys.path.append('../../../pypddl-parser/pypddl-parser')
from pddlparser import PDDLParser
from literal import Literal #used for isinstance()

import itertools
from itertools import product
import numpy as np
import copy
import os


def getArgs():

	# NOTE: You MUST name your DETERMINISTIC domain ppddl file with the world 'deterministic' in it

	args = sys.argv

	domains = args[1:-1] # e.g. ['determistic_domain.ppddl', 'probabilistic_domain.ppddl']
	problem = args[-1]
	print(domains,problem)

	return domains, problem

# def getSpecifications(domains_filenames,problem_filename):

# 	for domain_filename in domains_filenames:
# 		if 'deterministic' in domain_filename:
# 			print('found deterministic', domain_filename)
# 			det_domain = PDDLParser.parse(domain_filename)
# 		else:
# 			domain = PDDLParser.parse(domain_filename)

# 	problem = PDDLParser.parse(problem_filename)

# 	return det_domain, domain, problem


if __name__ == '__main__':

	# Method 1: Just run ppddl_to_matrices.py with each domain and the problem
	domain_filenames, problem_filename = getArgs()
	for domain_filename in domain_filenames:
		os.system("python3 ppddl_to_bt.py %s %s" %(domain_filename,problem_filename))


	# Method 2: Import all from ppddl_to_matrices and call functions

	# det_domain, domain, problem = getSpecifications(domain_filenames, problem_filename)

	# print(det_domain)
	# print(domain)
	# print(problem)