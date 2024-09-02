#!/usr/bin/env python

import random
import numpy as np
import rospy
from ppddl_to_matrices import *
from pypddl_parser.pddlparser import PDDLParser


class CompareRewards():
    def __init__(self, path):

        # Parameters
        self.num_trials = 100
        self.num_steps_per_trial = 100
        self.path = path

        # Get the domain and problem
        with open(self.path+"domain_and_problem.txt", "r") as f:
            domain_path = f.readline().strip()  # Read the first line and remove any trailing newline characters
            problem_path = f.readline().strip()  # Read the second line and remove any trailing newline characters
        domain  = PDDLParser.parse(domain_path)
        problem = PDDLParser.parse(problem_path)

        self.P, self.R, self.states, self.actions_with_params = getPandR(domain,problem)
        self.run()

    def run(self):


        try:
            done = False
            while not rospy.is_shutdown() and not done:

                done = True

                # Pass path to policy actions from bt file and raw policy actions
                bt_reward, raw_reward = self.compare_rewards()

        except rospy.ROSInterruptException: pass

    def compare_rewards(self):

        # Define the evaluation function
        def evaluate_once():

            # Ensure both bt and raw policies are triggered through same random seed
            # which will be a different seed each trial so trials are distinct
            trial_seed = np.random.randint(0, 10000)
            bt_rng = np.random.RandomState(trial_seed)
            raw_rng = np.random.RandomState(trial_seed)

            # Accumulate reward over the "mission" for a set number of steps
            bt_reward = 0
            raw_reward = 0

            # Open both files
            with open(self.path + "policy_actions_from_bt.txt", "r") as bt_file, open(self.path + "raw_policy_actions.txt", "r") as raw_file:
                # state_idx = 1
                # for bt_line, raw_line in zip(bt_file, raw_file):
                #     bt_action = bt_line.strip()
                #     raw_action = raw_line.strip()
                #     print(state_idx," ",bt_action, ", ", raw_action, self.get_action_index(raw_action))
                #     state_idx+=1

                # Read actions into a list where indices are state indices
                bt_actions = bt_file.readlines()
                raw_actions = raw_file.readlines()

                # Remove any new line characters at end of each action label in above lists
                for i in range(len(self.states)):
                    bt_actions[i], raw_actions[i] = bt_actions[i].strip(), raw_actions[i].strip()

                # Get random start state
                state_idx = random.randint(0,len(self.states)-1)
                bt_state_idx = raw_state_idx = state_idx # both start at same state
                # print("state_idx", state_idx)

                # Do a number of steps
                for j in range(self.num_steps_per_trial):

                    # Get action for that state from both bt policy and raw policy
                    bt_action, raw_action = bt_actions[state_idx], raw_actions[state_idx]
                    #print("state_idx: ", state_idx, ", bt: ", bt_action, ", raw: ", raw_action)

                    # Convert those to action idx's
                    bt_action_idx, raw_action_idx = self.get_action_index(bt_action), self.get_action_index(raw_action)
            
                    if (bt_action_idx!=raw_action_idx):
                        print("bt action idx: ", bt_action_idx)
                        print("raw action idx: ", raw_action_idx)
                        input("oopsie idx")


                    # Get the next state using the transition model (should be the same for bt vs raw always, but this checks that independently)
                    #bt_probabilities, raw_probabilities = self.P[bt_action_idx, state_idx], self.P[raw_action_idx, state_idx]
                    bt_probabilities = self.P[bt_action_idx, bt_state_idx]
                    raw_probabilities = self.P[raw_action_idx, raw_state_idx]
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
                    bt_reward += self.R[bt_action_idx, bt_state_idx, bt_next_state_idx]
                    raw_reward += self.R[raw_action_idx, raw_state_idx, raw_next_state_idx]
                    if (bt_reward!=raw_reward):
                        print("bt reward: ", bt_reward, ", ", "raw reward: ", raw_reward)
                        input("oopsie reward")

                    # Move to the next state
                    bt_state_idx = bt_next_state_idx
                    raw_state_idx = raw_next_state_idx
                    if (bt_state_idx!=raw_state_idx):
                        print("bt_state_idx: ", bt_state_idx, ", ", "raw_state_idx: ", raw_state_idx)
                        input("oopsie next update")



            #see comments below for rest

            return bt_reward, raw_reward

        
        #return evaluate_once() # for testing



        # Evaluate multiple times to find average reward
        bt_reward_sum, raw_reward_sum = 0,0
        for i in range(self.num_trials):
          bt_reward, raw_reward = evaluate_once()
          if (bt_reward!=raw_reward):
            input("oopsie")

          bt_reward_sum += bt_reward
          raw_reward_sum += raw_reward

        bt_average_reward, raw_average_reward = bt_reward_sum/self.num_trials, raw_reward_sum/self.num_trials

        print("Final average bt reward: ", bt_average_reward)
        print("Final average raw reward: ", raw_average_reward)

        return bt_average_reward, raw_average_reward

    def get_action_index(self, action_label):


        # loop elements in self.actions_with_params, for same label
        # return index of matching element

        #print("in get_action_index")
        for i in range(len(self.actions_with_params)):
            action = self.actions_with_params[i][0]
            #print(action.name)
            if action.name == action_label:
                #print(i)
                return i

        input("Not able to find action, so cannot return index!")


if __name__ == "__main__":

    rospy.init_node("compare_rewards")

    path = "/home/emily/Desktop/more_AURO_results/marine_final/"
    
    cr = CompareRewards(path)
