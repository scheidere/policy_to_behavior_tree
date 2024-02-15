from main import *
import os
import sys
import pickle
import time
import json
import matplotlib.pyplot as plt


# In this file, we display the probability (penalty constant) comparison results
# between input probabiliistic domain and deterministic version of domain

def get_probability_results(domain):

    start_time = round(time.time())

    # Notes for domain/problem combos
    #   domain p30 in probability2/ goes with problem2
    #   domain p30 in probability3/, p30_constraints or p30_constraints_consts all go with problem2_constraints

    if domain == 'm':
        # Marine
        pddl_path = "/home/scheidee/bt_synthesis_ws/src/policy_to_behavior_tree/pypddl-parser/pypddl-parser/pddl/marine/" # Laptop
        path_to_prob_domains = pddl_path + 'probability_fn2fp2/'
        problem_path = pddl_path + "problems/problem1.ppddl"

    if domain == 'i':
        # Infant
        pddl_path = "/home/scheidee/bt_synthesis_ws/src/policy_to_behavior_tree/pypddl-parser/pypddl-parser/pddl/infant_mobility/" # Laptop
        #pddl_path = "/home/scheidee/new_bt_generation_ws/src/bt_generation/policy_to_behavior_tree/pypddl-parser/pypddl-parser/pddl/infant_mobility/" # Desktop

        #path_to_prob_domains = pddl_path + "probability2/" #old
        #problem_path = pddl_path + "problems/problem2.ppddl" #infant (old)

        # New
        # path_to_prob_domains = pddl_path + "probability3/"
        # problem_path = pddl_path + "problems/problem2_constraint.ppddl"

        # Double new
        path_to_prob_domains = pddl_path + "probability4/" #FINAL
        #problem_path = pddl_path + "problems/problem2_constraint.ppddl"
        problem_path = pddl_path + "problems/problem3.ppddl" # has orientation object added, FINAL

    # Both
    det_domain_path = path_to_prob_domains + "domain_deterministic.ppddl" # Use with marine domain
    output_path = "/home/scheidee/bt_synthesis_ws/src/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/" # Laptop
    #output_path = "/home/scheidee/new_bt_generation_ws/src/bt_generation/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/" # Desktop

    #test_file = 'p30.ppddl' #p30_or_test.ppddl' #p30.ppddl' #'p30_constraints_consts.ppddl' #'p30_constraints.ppddl'
    #test_file = 'p30.ppddl' #marine

    domain_files = os.listdir(path_to_prob_domains)
    domain_files.sort()

    # FOR TESTING
    f = open("box_whisker_data.txt", "a")
    for d in domain_files:

    	if d != "domain_deterministic.ppddl":

    		filename = "box_and_whisker_" + domain + "_data/box_whisker_" + domain + "_data_" + d[:-6] + ".txt"
    		f = open(filename, "a")

    		prob_domain_path = path_to_prob_domains + d

    		avg_reward_prob, prob_rewards, avg_reward_det, det_rewards = compare_policies(prob_domain_path, det_domain_path, problem_path, output_path)

    		per_diffs = getData(prob_rewards, det_rewards)

    		print(per_diffs,len(per_diffs))
    		f.write(str(per_diffs))

    # Init list to be plotted in histogram (y axis)
    # percent_increase_list = []
    # labels = []
    # difference_list = [] # diff between p and d avg rewards
    # probabilistic_avg_rew_list = []
    # deterministic_avg_rew_list = []

    # for file in domain_files:
    #     print('file', file)
    #     #input('press enter')

    #     if file != 'domain_deterministic.ppddl': # and file == test_file: #will need to remove this test_file bit for full run

    #         print(file)
    #         label = file[1:3]
            
    #         prob_domain_path = path_to_prob_domains + file

    #         per_diff, difference, p, d = compare_policies(prob_domain_path, det_domain_path, problem_path, output_path)
    #         ###per_diff = compare_policies_testing(prob_domain_path, det_domain_path, problem_path, output_path)

    #         percent_increase_list.append(per_diff)
    #         labels.append(label)
    #         difference_list.append(difference)
    #         probabilistic_avg_rew_list.append(p)
    #         deterministic_avg_rew_list.append(d)

    # print(percent_increase_list)
    # print(labels)
    # print(difference_list)
    # print('p rewards', probabilistic_avg_rew_list)
    # print('d rewards', deterministic_avg_rew_list)
    # p_vals = [int(val) for val in probabilistic_avg_rew_list]
    # d_vals = [int(val) for val in deterministic_avg_rew_list]


    # NEW PLOTTING WAY (Also in plot_penalty.py)
    # plt.bar(labels, percent_increase_list, align='center')
    # plt.gca().set_xticks(labels)

    # for i in range(len(labels)):
    #     plt.annotate('p:'+str(p_vals[i])+' d:'+str(d_vals[i]), xy=(labels[i],percent_increase_list[i]), ha='center', va='bottom')


    # plt.xlabel('Action Effect Uncertainty') # Likelihood of action failure
    # plt.ylabel('Percent Increase')
    # #plt.title('Infant Domain: All rewards 2 except bubbles (3) and penalties -2')
    # if domain == 'i':
    #     plt.title('Infant Domain - Constant Penalty') # All rewards 2 except 3 for bubbles
    # elif domain == 'm':
    #     plt.title('Marine Domain - Constant Penalty')

    # #plt.savefig(str(start_time) + '_probability_results')
    # plt.show() #only this or savefig works, one at a time

def compare_policies(prob_domain_path, det_domain_path, problem_path, output_path, do_prints = False):

    # Path to policy and mdp_problem pickle files
    #output_path = "/home/scheidee/new_bt_generation_ws/src/bt_generation/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/"

    #partial_path = "/home/scheidee/new_bt_generation_ws/src/bt_generation/policy_to_behavior_tree/pypddl-parser/pypddl-parser/pddl"
    
    # Path to PPDDL domain and problem files
    # prob_domain_path = probabilistic_domain_path
    # det_domain_path = partial_path + '/marine/both_false_penalty/domain_deterministic.ppddl'
    # problem_path = partial_path + "/marine/problems/problem1.ppddl"

    # Run main with deterministic domain; Save policy
    os.system("python3 main.py " + det_domain_path + " " + problem_path)

    # Extract deterministic policy
    file = open(output_path+'policy.p','rb')
    det_policy = pickle.load(file)
    file.close()
    #print('d', det_policy)

    # Run main with probabilistic domain; Save policy and mdp problem
    os.system("python3 main.py " + prob_domain_path + " " + problem_path)
    
    # Extract probabilistic policy
    file = open(output_path+'policy.p','rb')
    prob_policy = pickle.load(file)
    file.close()
    #print('p', prob_policy)

    # Extract probabilistic mdp problem
    file = open(output_path+'mdp_problem.p','rb')
    mdp_problem = pickle.load(file)
    file.close()
    #print('mdp', mdp_problem)

    # Compare probabilistic and deterministic policies on same world (i.e. mdp problem)
    num_trials = 100
    print('Probabilistic path', prob_domain_path)
    print('Probabilistic: ', prob_policy)
    avg_reward_prob, prob_rewards = get_average_reward(num_trials, mdp_problem, prob_policy)
    print('\nAverage reward over %d trials: %f' % (num_trials, avg_reward_prob))
    print('Deterministic path', det_domain_path)
    print('Deterministic: ', det_policy)
    avg_reward_det, det_rewards = get_average_reward(num_trials, mdp_problem, det_policy)
    print('\nAverage reward over %d trials: %f' % (num_trials, avg_reward_det))

    #percentIncrease(avg_reward_prob, avg_reward_det)
    #per_diff, difference = percentDifference(avg_reward_prob, avg_reward_det) # + if prob better, - if det better

    #return avg_reward_prob, avg_reward_det
    return avg_reward_prob, prob_rewards, avg_reward_det, det_rewards

def percentDifference(prob_rew, det_rew):

    # small = min(prob_avg_rew,det_avg_rew)
    # less_small = max(prob_avg_rew,det_avg_rew)

    # percent =  100*(less_small - small)/small
    # print('%f is %f percent higher than %f' %(less_small,percent,small))

    difference = prob_rew - det_rew

    dec_per_diff = (prob_rew - det_rew)/det_rew

    per_diff = 100*dec_per_diff

    if per_diff < 0 and det_rew < prob_rew:

        per_diff = abs(per_diff)

    print('diff (+ if prob higher) ', per_diff)

    return per_diff, difference


def getData(p_rewards, d_rewards):


	per_diffs = []
	for i in range(len(p_rewards)):

		# Get % increase values (p - d); if negative, means method did better with d that time
		per_diff, diff = percentDifference(p_rewards[i], d_rewards[i])
		per_diffs.append(per_diff)


	return per_diffs # should have 100 elements


def plot(per_increase_data, p_avg, d_avg):

	ax1.boxplot(per_increase_data)

	# plt.show()

def test1():

	data = [1,2,3,4,5,6,7,8,9,10]
	p_avg = 8
	d_avg = 6

	plot(data,p_avg,d_avg)

def read_data(filename):

	f = open(filename,'r')
	contents = f.readlines()
	data = json.loads(contents[0].strip('\n'))
	#print(data)

	return data

def plot_all(domain):

	directory = "box_and_whisker_" + domain + "_data"

	# fig1, ax1 = plt.subplots()
	# ax1.set_title('Basic Plot')

	labels = []
	all_data = []
	files = os.listdir(directory)
	files.sort()
	for f in files:

		print(f)
		labels.append(f[-6:-4])
		data = read_data(directory+"/"+f)
		print(data[-1])
		print(sum(data)/len(data))
		all_data.append(data)
		# plt.boxplot(data)
		# plt.show()

	print(labels)
	print(len(all_data))
	#print(all_data)
	plt.boxplot(all_data)
	plt.show()

def plot_all2(domain):

	directory = "box_and_whisker_" + domain + "_data"

	# fig1, ax1 = plt.subplots()
	# ax1.set_title('Basic Plot')

	labels = []
	all_data = []
	files = os.listdir(directory)
	files.sort()
	for f in files:

		print(f)
		labels.append(f[-6:-4])
		data = read_data(directory+"/"+f)
		print(data[-1])
		print(sum(data)/len(data))
		all_data.append(data)
		# plt.boxplot(data)
		# plt.show()

	print(labels)
	print(len(all_data))
	#print(all_data)
	# fig = plt.figure(figsize =(10, 7))
	# ax = fig.add_subplot(111)
	fig, ax = plt.subplots(1,1)
	bp = ax.boxplot(all_data, patch_artist = True,
                notch ='True', vert = 1)

	colors = ['#0C7BDC', '#0C7BDC',
          '#0C7BDC', '#0C7BDC']
 
	for patch, color in zip(bp['boxes'], colors):
		patch.set_facecolor(color)

	if domain == 'm':
		plt.title("Marine Domain - Constant Penalty")
	else:
		plt.title("Infant Domain - Constant Penalty")
	plt.xlabel("Transition Probability")
	plt.ylabel("Percent Increase in Reward")
	ax.set_xticklabels(labels)

	# changing color and linewidth of
	# whiskers
	for whisker in bp['whiskers']:
	    whisker.set(color ='black',
	                linewidth = 1.5,
	                linestyle =":")
	 
	# changing color and linewidth of
	# caps
	for cap in bp['caps']:
	    cap.set(color ='black',
	            linewidth = 2)
	 
	# changing color and linewidth of
	# medians
	for median in bp['medians']:
	    median.set(color ='#FFC20A',
	               linewidth = 3)

	# changing style of fliers
	for flier in bp['fliers']:
	    flier.set(marker ='D',
	              color ='#e7298a',
	              alpha = 0.5)

	plt.show() 





def generate_data(domain):
	# To generate data, run the following two lines (doesn't auto save data yet)
	get_probability_results(domain)

if __name__ == "__main__":

	#domain = input("Choose either the marine (type 'm') or infant (type 'i') domain: ")

	#generate_data('i')

	#Comment out simplification in main

	plot_all2('m')


