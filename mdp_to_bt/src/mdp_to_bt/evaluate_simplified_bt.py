#! /usr/bin/env python
# Will also need to make a launch file for this right?

# Doesn't matter what order states are evaluated
# Need to evaluate all states only once, get action BT triggers, and get associated reward
# Just like in evaluate_mdp_policy.py, evaluate for 100 100-step trials


# Things we will need
# MDP problem class like in eval mdp policy file
# Conversion between state idx and list of conditions for triggering BT
# Conversion between triggered action and action idx
# The next state is determined by the transition model (that IS our simulator per se)
# and we need the indices to access that info 
# Once got next state idx from transition matrix, that becomes current state
# Which then gets converted to list of conditions that can be used to trigger BT
# then it returns action triggered, which is converted to action idx
# To transition matrix -> next state, etc. for 100 steps per trial


# How do we reset the BT between trials? Where do we want the Bt interface object to be?

import random
import numpy as np
from evaluate_mdp_policy import MDP_Problem
from pypddl_parser.pddlparser import PDDLParser
from ppddl_to_matrices import *
from bt_interface import *

# class EvaluateBT:
#     def __init__(self, bt):
#         self.bt = bt
#         self.bt_interface = BT_Interface(self.bt)


    # def evaluate_simplified_bt(self, mdp_problem): #, bt):

    #     # Parameters
    #     num_trials = 3 #100
    #     num_steps_per_trial = 3 #100

    #     self.init_bt()
    #     self.bt.print_BT()

    #     #print("States: ", mdp_problem.states[0])

    #     # Define the evaluation function
    #     def evaluate_once(mdp_problem): #, bt):

    #         #bt_interface = BT_Interface(bt)

    #         # print("================ evaluate_once ==============")
    #         # Accumulate reward over the "mission"
    #         reward = 0

    #         # Pick a start state
    #         state_idx = random.randint(0,len(mdp_problem.states)-1)
    #         state = mdp_problem.states[state_idx]

    #         # Do a number of steps
    #         for j in range(num_steps_per_trial):
    #             # print("j", j)

    #             #self.bt.tick()

    #             # Pick an action according to the bt
    #             action = self.get_action_from_bt(state)
    #             action_idx = self.get_action_idx(mdp_problem, action)
    #             # print("action_idx", action_idx) 

    #         return reward

    #     # Evaluate multiple times to find average reward
    #     reward_sum = 0
    #     for i in range(num_trials):
    #         reward = evaluate_once(mdp_problem) #, bt)

    #         reward_sum += reward

    #     average_reward = reward_sum / num_trials

    #     return average_reward


    # def get_condition_label(self, condition_term):

    #     num_c_terms = len(condition_term)-1 # Last term of state is 0/1 representing False/True
    #     print('num_c_terms:', num_c_terms)

    #     print("condition node keys: ", self.bt.condition_nodes.keys())

    #     for label in self.bt.condition_nodes.keys():
    #         print("label: ", label)
    #         found = True
    #         for j in range(num_c_terms): 

    #             if condition_term[j] not in label:
    #                 found = False
    #                 break

    #         # If here without break, all terms e.g. 'infant-orientation' and 'toward' in label so condition label found
    #         if found:
    #             return label

    # def get_condition_statuses(self):

    #     # Self.bt.condition_nodes is a dict where keys are conditions labels and values are instances
    #     # Returns statuses in a list of ALL condition node instances (not just by type)

    #     statuses = []
    #     for lst in list(self.bt.condition_nodes.values()):
    #         #print("lst", lst)
    #         for c in lst:
    #             #print("c", c)
    #             statuses.append(c.status.status)

    #     return statuses

    # def update_bt(self, state): #(bt_interface, bt, state):

    #     # Takes state format generated from getStatesList that pulls from the PPDDL domain and problem directly
    #     # Does include constraints

    #     for i in range(len(state)):

    #         c = state[i] # e.g. ['infant-orientation', 'toward', 1] but could have more than one param like 'toward'
    #         print("state[i]: ", c)

    #         c_label = self.get_condition_label(c) #(bt, c)
    #         print("c_label: ", c_label)
            
    #         #print('c_label', c_label)

    #         if state[i][-1]: #1
    #             boolean = True
    #         else:
    #             boolean = False
    #         self.bt_interface.setConditionStatus(c_label,boolean)


    # def get_action_from_bt(self, state):

    #     #[['has-object', 'x', 0], ['found-object', 'x', 0], ['found-mine', 'x', 0], 
    #     #['at-drop-off', 'x', 0], ['found-target-to-report', 'x', 0], ['in-comms-range', 'x', 0]]


    #     # Update BT with state
    #     self.update_bt(state) #(bt_interface, bt, state)

    #     # Check that update worked
    #     self.get_condition_statuses()

    #     self.bt.tick()

    #     # Get triggered action
    #     active_actions = self.bt.getActiveActions()
    #     if len(active_actions) > 1:
    #         input("NOOO, MORE THAN ONE ACTION ACTION! Check why.")
    #     elif len(active_actions)==0:
    #         input("No active actions")

    #     action = active_actions[0]
    #     print("Action: ", action)
    #     input("hi")

    #     return action


# This is for running with separate launch (because of issues running the original class stuff through main launch)
class EvaluateBT:
    def __init__(self, bt_path, domain, problem):
        self.bt = BehaviorTree(bt_path)
        self.bt_interface = BT_Interface(self.bt)
        self.P, self.R, self.states, self.actions_with_params = getPandR(domain,problem)
        self.num_trials = 1 #100
        self.num_steps_per_trial = 1 #100
        self.num_states = len(self.states)
        
    def init_bt(self):
        # print("BT_Interface initialising BT...")
        for node in self.bt.nodes:
            node.init_ros() # this just has pass in it....
            # print(node.label)
        # print("BT finished init")


    def run(self):

        # Have main rospy loop in here and call reward from bt function and reward from policy function in it

        # Make sure you have removed the rospy loop from the reward from bt function and it works
        try:

            self.init_bt()

            while not rospy.is_shutdown():

                self.run_trial()
                input("Reward from bt above ")

        except rospy.ROSInterruptException: pass


    def reward_from_bt(self):

        # Testing first with the rospy loop here to mimic bt_to_policy_equivalency_test

        try:

            self.init_bt()

            current_state_eval_done = True # Trigger update to first state
            running_active_actions = None
            first_active_running_action = None

            count = 0
            # saved = False
            done = False
            old_running_active_actions = None
            update_state = True
            state = None
            state_count = 0

            # Pick a start state
            state_idx = random.randint(0,len(self.states)-1)

            while not rospy.is_shutdown():


                # Update state
                if state_count < self.num_states and update_state:
                    
                    state = self.states[state_idx]
                    self.update_bt(state)
                    state_count +=1
                    # f.write("State count: %d\n" %state_count)
                    # f.write("State: %s\n" %str(state))
                    update_state = False
                    prev = first_active_running_action
                    first_active_running_action = None
                    # active_actions = self.bt.getActiveActions()
                    # running_active_actions = self.get_running_actions_from_active_actions(active_actions)
                    # print('post update', running_active_actions)
                    # print('post update', state)

                elif state_count == self.num_states and not done:
                    print("DONE!!!")
                    done = True

                self.bt.tick()
                count += 1

                # Monitor active running actions
                active_actions = self.bt.getActiveActions()
                print("active actions: ", str(active_actions))
                #input('hi')
                running_active_actions = self.get_running_actions_from_active_actions(active_actions)

                if count%100: #is this the buffer so that the action is obtained correctly?

                    # print('running_active_actions', running_active_actions)
                    # print('state', state)

                    if running_active_actions and not first_active_running_action:
                        print('active_actions', active_actions)
                        print('running_active_actions', running_active_actions)
                        first_active_running_action = running_active_actions[0]
                        first_active_running_action = first_active_running_action.split("(",1)[0] # removing (x: x), leaving label

                        print(state)
                        print(first_active_running_action)
                        update_state = True

                        # Not sure if this stuff is still needed, seems redundant
                        # This will be where we need to convert action to action idx
                        # which can then be used to get the reward from R (and below the transition to the next state from P)
                        action_idx = self.get_action_index(first_active_running_action)
                        input("hi")


                        # Get the next state using the transition model
                        probabilities = self.P[action_idx, state_idx]
                        next_states_idx = range(0,len(probabilities))
                        next_state_idx = np.random.choice(next_states_idx, p=probabilities)


        except rospy.ROSInterruptException: pass


    def get_action_label_from_bt(self):

        # Give buffer of 100 ticks, and return action from BT
        for i in range(100):
            self.bt.tick()

        # After buffer, get action
        active_actions = self.bt.getActiveActions()
        print("active actions: ", str(active_actions))
        #input('hi')
        running_active_actions = self.get_running_actions_from_active_actions(active_actions)
        first_active_running_action = running_active_actions[0]
        first_active_running_action = first_active_running_action.split("(",1)[0]
        
        return first_active_running_action


    def run_trial(self): # Like evaluate once in evaluate_mdp_policy

        # print("================ evaluate_once ==============")
        # Accumulate reward over the "mission" for certain number of steps
        reward = 0

        # Pick a start state
        state_idx = random.randint(0,len(self.states)-1)
        
        # Do a number of steps
        for j in range(self.num_steps_per_trial):

            state = self.states[state_idx]
            self.update_bt(state)

            # Get action from bt
            action_label = self.get_action_label_from_bt() # includes buffer of 100 ticks

            # Get action index
            action_idx = self.get_action_index(action_label)

            # Get the next state using the transition model
            probabilities = self.P[action_idx, state_idx]
            next_states_idx = range(0,len(probabilities))
            next_state_idx = np.random.choice(next_states_idx, p=probabilities)
            # print("next_state_idx", next_state_idx)

            # Get a reward according to the reward matrix
            reward += self.R[action_idx, state_idx, next_state_idx]
            # print("accumulated reward", reward)

            # Move to the next state
            state_idx = next_state_idx

        return reward


    def get_action_index(self, action_label):


        # loop elements in self.actions_with_params, for same label
        # return index of matching element

        print("in get_action_index")
        for i in range(len(self.actions_with_params)):
            action = self.actions_with_params[i][0]
            print(action.name)
            if action.name == action_label:
                print(i)
                return i

        input("Not able to find action, so cannot return index!")

    def get_running_actions_from_active_actions(self, active_actions):

        # There should only ever be one active running action (ignoring parallel nodes)
        running_actions = []
        for a_label in active_actions:
            action_node_list = self.bt.action_nodes[a_label]
            status = action_node_list[0].status.status
            if status == 1:
                #print('status is 1')
                running_actions.append(a_label)

        return running_actions # Plz tell me there is only one


    def update_bt(self, state): #(bt_interface, bt, state):

        # Takes state format generated from getStatesList that pulls from the PPDDL domain and problem directly
        # Does include constraints

        for i in range(len(state)):

            c = state[i] # e.g. ['infant-orientation', 'toward', 1] but could have more than one param like 'toward'
            print("state[i]: ", c)

            c_label = self.get_condition_label(c) #(bt, c)
            print("c_label: ", c_label)
            
            #print('c_label', c_label)

            if state[i][-1]: #1
                boolean = True
            else:
                boolean = False
            self.bt_interface.setConditionStatus(c_label,boolean)

    def get_condition_label(self, condition_term):

        num_c_terms = len(condition_term)-1 # Last term of state is 0/1 representing False/True
        print('num_c_terms:', num_c_terms)

        print("condition node keys: ", self.bt.condition_nodes.keys())

        for label in self.bt.condition_nodes.keys():
            print("label: ", label)
            found = True
            for j in range(num_c_terms): 

                if condition_term[j] not in label:
                    found = False
                    break

            # If here without break, all terms e.g. 'infant-orientation' and 'toward' in label so condition label found
            if found:
                return label



if __name__ == "__main__":

    rospy.init_node("reward_equality_demonstration",anonymous=True) # between bt and policy
    graphviz_pub = rospy.Publisher('behavior_tree_graphviz', String, queue_size=1)
    compressed_pub = rospy.Publisher('behavior_tree_graphviz_compressed', String, queue_size=1)


    # Domain and problem
    domain_path = "/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/infant/final_domain.ppddl"
    problem_path = "/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/infant/problem.ppddl"

    # Get the domain and problem
    domain  = PDDLParser.parse(domain_path)
    problem = PDDLParser.parse(problem_path)

    # Raw policy (for that domain and problem)

    # Final simplified bt (for that domain and problem)
    bt_final_path = "/home/emily/auro_ws/src/policy_to_behavior_tree/behavior_tree/config/AURO_final_synthesized_BTs/infant/final_synth_bt.tree"


    ebt = EvaluateBT(bt_final_path,domain,problem)
    ebt.run()