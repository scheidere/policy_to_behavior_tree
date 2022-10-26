import numpy as np
import os
import json
from behavior_tree.behavior_tree import *


def get_intermediate_average_gain(filename,datatype):

	if datatype == 'l':
		lst = read_txt_results(filename)
		return sum(lst)/len(lst)
	else:
		arr = read_numpy_results(filename)
		return arr.sum()/arr.size



def read_txt_results(filename):

	f = open(filename,'r')
	contents = f.readlines()
	data = json.loads(contents[0].strip('\n'))
	#print(data)

	return data


def read_numpy_results(filename):

    data = np.load(filename)
    #print(data)

    return data


def get_overall_average_gain():

	averages = []

	# 8 lists represent the constant penalty experiment results
	directorys = ["box_and_whisker_i_data","box_and_whisker_m_data"]
	for directory in directorys:
		files = os.listdir(directory)
		for file in files:
			#print(file)
			avg = get_intermediate_average_gain(directory+'/'+file,'l')
			averages.append(avg)

	#print(averages)

	# 2 arrays represent the constant probability experiment results
	directory = "imshow_data/"
	files = os.listdir(directory)
	for file in files:
		avg = get_intermediate_average_gain(directory+file,'m')
		averages.append(avg)

	#print(averages)

	return sum(averages)/len(averages)


def get_exec_time():

	marine = "python3 main.py ../../../pypddl-parser/pypddl-parser/pddl/marine/final_display_bt/penalty-reward_mag_match.ppddl ../../../pypddl-parser/pypddl-parser/pddl/marine/problems/problem1.ppddl"
	infant = "python3 main.py ../../../pypddl-parser/pypddl-parser/pddl/infant_mobility/final_display_bt/penalty_reward_mag_match.ppddl ../../../pypddl-parser/pypddl-parser/pddl/infant_mobility/problems/problem3.ppddl"

	os.system(marine)
	input('Press enter')
	os.system(infant)

def count_nodes(config_dir):

	bt = BehaviorTree(config_dir)
	# print(bt.nodes)
	# print(len(bt.nodes))

	return len(bt.nodes)

def count_all_nodes():

	final_infant_dir = "/home/scheidee/bt_synthesis_ws/src/policy_to_behavior_tree/behavior_tree/config/final_synthesized_BTs/infant/penalty-reward-mag-match-varied"
	# final_marine_dir = "/home/scheidee/bt_synthesis_ws/src/policy_to_behavior_tree/behavior_tree/config/final_synthesized_BTs/marine/final-penalty-reward-mag-match7030"

	infant_raw_nodes = count_nodes(final_infant_dir+'/raw_policy_bt.tree')
	infant_simp_nodes = count_nodes(final_infant_dir+'/final_clean.tree')
	marine_raw_nodes = count_nodes(final_marine_dir+'/raw_policy_bt.tree')
	marine_simp_nodes = count_nodes(final_marine_dir+'/final_clean.tree')

	print('Nodes in raw infant bt: ', infant_raw_nodes)
	print('Nodes in simplified infant bt: ', infant_simp_nodes)
	print('Nodes in raw marine bt: ', marine_raw_nodes)
	print('Nodes in simplified marine bt: ', marine_simp_nodes)


if __name__ == "__main__":

	# Average percent increase of prob over det
	average_gain = get_overall_average_gain()
	# print("Average gain: ", average_gain)

	# input('Press enter')

	# Range of execution time (approx. 1 to 50s)
	#get_exec_time()

	# Number of nodes in:
	# raw policy and simplified bt for
	# marine and infant domain
	final_infant_dir = "/home/scheidee/bt_synthesis_ws/src/policy_to_behavior_tree/behavior_tree/config/final_synthesized_BTs/infant/penalty-reward-mag-match-varied"
	raw_infant_tree = "raw_policy_bt.tree"
	simplified_infant_tree = "final_clean.tree"
	final_marine_dir = "/home/scheidee/bt_synthesis_ws/src/policy_to_behavior_tree/behavior_tree/config/final_synthesized_BTs/marine/final-penalty-reward-mag-match7030"
	raw_marine_tree = "raw_policy_bt.tree"
	simplified_marine_tree = "final_clean.tree"

	count_all_nodes()
