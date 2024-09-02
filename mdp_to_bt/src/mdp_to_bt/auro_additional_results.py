from main import *
import os
import sys
import pickle
import time
import random
import roslaunch
import matplotlib.pyplot as plt


def get_file_paths(domain_option, group_option):

    if domain_option == "m":
        pddl_path = "/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/marine/"
        problem_path = pddl_path + "problems/problem1.ppddl"

        if group_option == "pen": # const penalty, varied action effect uncertainty
            path_to_domains = pddl_path + "probability_fn2fp2/" # Group of 4 IF we exclude deterministic domain

        elif group_option == "prob": # const action effect uncertainty 30%, varied penalty
            path_to_domains = pddl_path + "both_false_penalty/" # Group of 16

    elif domain_option == "i":
        pddl_path = "/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/infant_mobility/"
        problem_path = pddl_path + "problems/problem3.ppddl"

        if group_option == "pen": # const penalty, varied action effect uncertainty
            path_to_domains = pddl_path + "probability4/" # Group of 4 IF we exclude deterministic domain
            
        elif group_option == "prob": # const action effect uncertainty 30%, varied penalty
            path_to_domains = pddl_path + "penalty_p30const/" # Group of 16


    return path_to_domains, problem_path


def init_arrays(group_option):

    if group_option == "pen": #4
        runtimes = np.zeros((4,1))
        raw_rewards = np.zeros((4,1))
        bt_rewards = np.zeros((4,1))
    elif group_option == "prob": # 16
        runtimes = np.zeros((4,4))
        raw_rewards = np.zeros((4,4))
        bt_rewards = np.zeros((4,4))

    return runtimes, raw_rewards, bt_rewards

# def update_arrays(group_option):

#     if group_option == "pen": #4

#     elif group_option == "prob": # 16

def update_array(file, array, new_value):

    if group_option == "pen": #4
        # files are p10, p20, p30, p40
        i = int(file[1:-3][0])-1
        print(i)
        j=0

    elif group_option == "prob": # 16
        fn_num,fp_num = int(float(file[2])), int(float(file[5])) # fn will be x-axis, fp will be y-axis (i in array)
        j,i = fn_num - 1, fp_num - 1

    array[i][j] = new_value
    
    return array

def get_action_index(action_label, actions_with_params):


    # loop elements in self.actions_with_params, for same label
    # return index of matching element

    #print("in get_action_index")
    for i in range(len(actions_with_params)):
        action = actions_with_params[i][0]
        #print(action.name)
        if action.name == action_label:
            #print(i)
            return i

    input("Not able to find action, so cannot return index!")

# Define the evaluation function
def evaluate_once(bt_actions, raw_actions, P, R, states, actions_with_params, num_trials, num_steps_per_trial):

    # Ensure both bt and raw policies are triggered through same random seed
    # which will be a different seed each trial so trials are distinct
    trial_seed = np.random.randint(0, 10000)
    bt_rng = np.random.RandomState(trial_seed)
    raw_rng = np.random.RandomState(trial_seed)

    # Accumulate reward over the "mission" for a set number of steps
    bt_reward = 0
    raw_reward = 0

    # Get random start state
    state_idx = random.randint(0,len(states)-1)
    bt_state_idx = raw_state_idx = state_idx # both start at same state
    # print("state_idx", state_idx)

    # Do a number of steps
    for j in range(num_steps_per_trial):

        # Get action for that state from both bt policy and raw policy
        bt_action, raw_action = bt_actions[state_idx], raw_actions[state_idx]
        #print("state_idx: ", state_idx, ", bt: ", bt_action, ", raw: ", raw_action)

        # Convert those to action idx's
        bt_action_idx, raw_action_idx = get_action_index(bt_action, actions_with_params), get_action_index(raw_action, actions_with_params)

        if (bt_action_idx!=raw_action_idx):
            print("bt action idx: ", bt_action_idx)
            print("raw action idx: ", raw_action_idx)
            input("oopsie idx")


        # Get the next state using the transition model (should be the same for bt vs raw always, but this checks that independently)
        #bt_probabilities, raw_probabilities = self.P[bt_action_idx, state_idx], self.P[raw_action_idx, state_idx]
        bt_probabilities = P[bt_action_idx, bt_state_idx]
        raw_probabilities = P[raw_action_idx, raw_state_idx]
        if not np.all(bt_probabilities == raw_probabilities):
            print("bt_probabilities ", bt_probabilities, ", raw_probabilities ", raw_probabilities)
            input("oopsie probs")
        #bt_next_states_idx, raw_next_states_idx = range(0,len(bt_probabilities)), range(0,len(raw_probabilities))
        bt_next_states_idx = range(0,len(bt_probabilities))
        raw_next_states_idx = range(0,len(raw_probabilities))
        bt_next_state_idx = bt_rng.choice(bt_next_states_idx, p=bt_probabilities)
        raw_next_state_idx = raw_rng.choice(raw_next_states_idx, p=raw_probabilities)
        # print("next_state_idx", next_state_idx)
        if (bt_next_state_idx!=raw_next_state_idx):
            print("bt_next_state_idx: ", bt_next_state_idx, ", ", "raw_next_state_idx: ", raw_next_state_idx)
            input("oopsie next")


        # Get a reward according to the reward matrix
        bt_reward += R[bt_action_idx, bt_state_idx, bt_next_state_idx]
        raw_reward += R[raw_action_idx, raw_state_idx, raw_next_state_idx]
        if (bt_reward!=raw_reward):
            print("bt reward: ", bt_reward, ", ", "raw reward: ", raw_reward)
            input("oopsie reward")

        # Move to the next state
        bt_state_idx = bt_next_state_idx
        raw_state_idx = raw_next_state_idx
        if (bt_state_idx!=raw_state_idx):
            print("bt_state_idx: ", bt_state_idx, ", ", "raw_state_idx: ", raw_state_idx)
            input("oopsie next update")

    return bt_reward, raw_reward

def compare_raw_policy_and_bt_policy(bt_actions, raw_actions, P, R, states, actions_with_params):

    num_trials = 100
    num_steps_per_trial = 100

    
    # Evaluate multiple times to find average reward
    bt_reward_sum, raw_reward_sum = 0,0
    for i in range(num_trials):
      bt_reward, raw_reward = evaluate_once(bt_actions, raw_actions, P, R, states, actions_with_params, num_trials, num_steps_per_trial)
      if (bt_reward!=raw_reward):
        input("oopsie")

      bt_reward_sum += bt_reward
      raw_reward_sum += raw_reward

    bt_average_reward, raw_average_reward = bt_reward_sum/num_trials, raw_reward_sum/num_trials

    print("Final average bt reward: ", bt_average_reward)
    print("Final average raw reward: ", raw_average_reward)

    return bt_average_reward, raw_average_reward

def examine_domain_group(domain_option, group_option):

    # Locate group of domains (with associated problem)
    path_to_domains, problem_path = get_file_paths(domain_option, group_option)

    domain_files = os.listdir(path_to_domains)
    domain_files.sort()

    runtimes, raw_rewards, bt_rewards = init_arrays(group_option)

    for file in domain_files:

        if file != 'domain_deterministic.ppddl':

            domain_path = path_to_domains + file
            domain  = PDDLParser.parse(domain_path)
            problem = PDDLParser.parse(problem_path)
            P, R, states, actions_with_params = getPandR(domain,problem)

            # Run main to get raw policy (just list of actions, where index in list denotes state)
            # Output in "/home/emily/Desktop/more_AURO_results/"
            os.system("python3 main.py " + domain_path + " " + problem_path)

            with open('/home/emily/Desktop/more_AURO_results/getPandR_outputs.p', 'rb') as f:
                P, R, states, actions_with_params = pickle.load(f)

            # Get runtime
            f_info = open("/home/emily/Desktop/more_AURO_results/info.txt", "r")
            lines = f_info.readlines()
            runtime = lines[2].strip()
            print(runtime)
            update_array(file, runtimes, runtime)
            # Add runtime to array

            # Get raw policy (action list)
            f_raw = open("/home/emily/Desktop/more_AURO_results/raw_policy_actions.txt", "r")
            raw_actions = f_raw.readlines()
            # Get raw policy reward

            # Run bt_to_policy_equivalency_test.py to translate the simplified bt from main back into policy (action list) form
            # Output in "/home/emily/Desktop/more_AURO_results/"
            os.system("python3 bt_to_policy_equivalency_test.py") # Assesses final_synth_bt.tree in "/home/emily/Desktop/more_AURO_results/"
            f_bt = open("/home/emily/Desktop/more_AURO_results/policy_actions_from_bt.txt", "r")
            bt_actions = f_bt.readlines()

            # Remove any new line characters at end of each action label in above lists
            for i in range(len(states)):
                bt_actions[i], raw_actions[i] = bt_actions[i].strip(), raw_actions[i].strip()


            bt_average_reward, raw_average_reward = compare_raw_policy_and_bt_policy(bt_actions, raw_actions, P, R, states, actions_with_params)

            update_array(file, raw_rewards, raw_average_reward)
            update_array(file, bt_rewards, bt_average_reward)


    print(runtimes)
    print(raw_rewards)
    print(bt_rewards)

    path = "/home/emily/auro_ws/src/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/imshow_data/"
    np.save(path + "imshow_" + domain_option + "_" + group_option + "_runtimes.npy",runtimes)
    np.save(path + "imshow_" + domain_option + "_" + group_option + "_raw_rewards.npy",raw_rewards)
    np.save(path + "imshow_" + domain_option + "_" + group_option + "_bt_rewards.npy",bt_rewards)


def run():

    #Examine all domain groups and generate final plots
    pass



if __name__ == "__main__":

    # Domain option "i" for infant mobility or "m" for marine
    domain_option = "i"

    # Group option "pen" or "prob"
    # "pen" for const penalty, varied action effect uncertainty (total of 4 domains)
    # "prob" for const action effect uncertainty, varied false positive and false negative penalties (total of 16 domains)
    group_option = "pen"

    examine_domain_group(domain_option, group_option)