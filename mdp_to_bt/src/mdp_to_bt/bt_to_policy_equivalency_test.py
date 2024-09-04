#!/usr/bin/env python

from ppddl_to_matrices import getStateList
import behavior_tree.behavior_tree_graphviz as gv
import rospy
from bt_interface import *
from behavior_tree_msgs.msg import Status, Active
import zlib
import copy
import itertools
from statistics import mean
from pypddl_parser.pddlparser import PDDLParser
from bt_interface import *

import time

class CompareBTPolicy():
    def __init__(self, bt_path, domain, problem):
        self.bt = BehaviorTree(bt_path)
        self.bt_interface = BT_Interface(self.bt)
        policy = self.bt_to_policy(self.bt, domain, problem) #works

    def init_bt(self):
        # print("BT_Interface initialising BT...")
        for node in self.bt.nodes:
            node.init_ros() # this just has pass in it....
            # print(node.label)
        # print("BT finished init")


    def bt_to_policy(self, bt, domain, problem):

        try:
            f = open("/home/emily/Desktop/more_AURO_results/policy_from_btTESTING.txt", "w+")
            fa = open("/home/emily/Desktop/more_AURO_results/policy_actions_from_btTESTING.txt", "w+")
            fb = open("/home/emily/Desktop/more_AURO_results/running_active_actionsTEST.txt", "w+")
            print('In bt_to_policy +++++++++++++++++++===')
            
            self.init_bt()
            states = getStateList(domain, problem)
            states.reverse()  # Reverse list so pop works correctly
            num_states = len(states)
            state_count = 0
            update_state = True
            state = None
            done = False

            while not rospy.is_shutdown() and not done:

                if len(states) > 0 and update_state:
                    state = states.pop()
                    self.update_bt(state)
                    state_count += 1
                    f.write(f"State count: {state_count}\n")
                    f.write(f"State: {state}\n")
                    update_state = False


                self.bt.tick()

                active_actions = self.bt.getActiveActions()
                print("State: ", state)
                print("state index: ", state_count)
                print("active_actions: ", active_actions)
                #input("hmm ")
                running_active_actions = self.get_running_actions_from_active_actions(active_actions)
                print("Running active actions: ", running_active_actions)
                active_conditions = self.bt.getActiveConditions()

                if running_active_actions:
                    first_active_running_action = running_active_actions[0].split("(", 1)[0]
                    print("first active running action: ", first_active_running_action)
                    #input("hmm 2")
                    f.write(f"Action: {first_active_running_action}\n")
                    fa.write(f"{first_active_running_action}\n")
                    fb.write(f"State count: {state_count}\n")
                    fb.write(f"State: {state}\n")
                    fb.write(f"BT conditions: {active_conditions}\n")
                    fb.write(f"Active actions: {active_actions}\n")
                    fb.write(f"Running active actions: {running_active_actions}\n")
                    fb.write(f"Action: {first_active_running_action}\n")
                    update_state = True

                # If have checked all states, stop
                if state_count == num_states and not done:
                    print("All states processed. Exiting...")
                    done = True


            f.close()
            fa.close()

            print('End bt_to_policy +++++++++++++++++++===')
            

        except rospy.ROSInterruptException: pass


    def print_bt_cond_statuses(self):

        state_from_bt = []
        #print('yee', self.bt.condition_nodes)
        for c in self.bt.condition_nodes.keys():
            #print('test c', c)
            c_lst = self.bt.condition_nodes[c]
            status = c_lst[0].status.status
            state_from_bt.append([c,status])

        print(state_from_bt)

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

    def print_active_action_statuses(self, active_actions):

        statuses = []
        for a_label in active_actions:
            action_node_list = self.bt.action_nodes[a_label]
            status = action_node_list[0].status.status
            #print(a_label, status)
            statuses.append(status)

        return statuses

    def get_condition_statuses(self):

        # Self.bt.condition_nodes is a dict where keys are conditions labels and values are instances
        # Returns statuses in a list of ALL condition node instances (not just by type)

        statuses = []
        for lst in list(self.bt.condition_nodes.values()):
            #print("lst", lst)
            for c in lst:
                #print("c", c)
                statuses.append(c.status.status)

        return statuses

    def get_c_label(self, c_label_pure):

        # Condition labels in the BT currently contain "x" and "_"
        # We need to match these for update statuses 
        # Assumes condition names don't overlap 

        #print("YEE", self.bt.condition_nodes.keys())

        for label in self.bt.condition_nodes.keys():
            if c_label_pure in label:
                #print("YO", c_label_pure,label)
                return label
        return None # ERROR

    def get_condition_label(self, condition_term):

        num_c_terms = len(condition_term)-1 # Last term of state is 0/1 representing False/True

        for label in self.bt.condition_nodes.keys():
            found = True
            for j in range(num_c_terms): 

                if condition_term[j] not in label:
                    found = False
                    break

            # If here without break, all terms e.g. 'infant-orientation' and 'toward' in label so condition label found
            if found:
                return label

    def update_bt(self, state):

        # update this to match fix in compactness, needs to account for params or will break for infant domain and others like it

        # #follow method in compactness eval except from pddl state format not my format for state

        # #print("condies%s\n" %self.bt.condition_nodes.keys()) # have x param references in names

        # for i in range(len(state)):

        #     c_label_pure = state[i][0] # pure meaning 'found_mine' as opposed to the 'found_mine_x' found in the tree due to params
        #     c_label = self.get_c_label(c_label_pure)
        #     #print('c_label %s'%str(c_label))

        #     if state[i][-1]: #1
        #         boolean = True
        #     else:
        #         boolean = False
        #     self.bt_interface.setConditionStatus(c_label,boolean)

        # return

        # Update below!
        # Takes state format generated from getStatesList that pulls from the PPDDL domain and problem directly
        # Does include constraints
        print("In update_bt")
        print("State: ", state)

        for i in range(len(state)):

            c = state[i] # e.g. ['infant-orientation', 'toward', 1] but could have more than one param like 'toward'

            print("c: ", c)

            c_label = self.get_condition_label(c)
            
            print('c_label', c_label)

            if state[i][-1]: #1
                boolean = True
            else:
                boolean = False
            self.bt_interface.setConditionStatus(c_label,boolean)


        # Check bt state
        #active_conditions = self.bt.getActiveConditions()
        #print("Active conditions: ", active_conditions)
        #input("u")

        #input("waiting")


if __name__ == "__main__":

    rospy.init_node("compare_bt_to_policy")
    graphviz_pub = rospy.Publisher('behavior_tree_graphviz', String, queue_size=1)
    compressed_pub = rospy.Publisher('behavior_tree_graphviz_compressed', String, queue_size=1)

    # infant
    # bt_final_path = "/home/emily/auro_ws/src/policy_to_behavior_tree/behavior_tree/config/AURO_final_synthesized_BTs/infant/final_synth_bt.tree"
    # domain_path = "/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/infant/final_domain.ppddl"
    # problem_path = "/home/emily/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/infant/problem.ppddl"

    # marine
    #path = "/home/emily/Desktop/more_AURO_results/marine_final_1/"

    # infant
    path = "/home/emily/Desktop/more_AURO_results/infant_final/"


    bt_path = path + "final_synth_bt.tree"
    #bt_path = path + "raw_policy_bt.tree" # This takes painfully long, do not do it
    with open(path+"domain_and_problem.txt", "r") as f:
        domain_path = f.readline().strip()  # Read the first line and remove any trailing newline characters
        problem_path = f.readline().strip()  # Read the second line and remove any trailing newline characters


    # Get the domain and problem
    domain  = PDDLParser.parse(domain_path)
    problem = PDDLParser.parse(problem_path)

    cbtp = CompareBTPolicy(bt_path,domain,problem)
    #cbtp = CompareBTPolicy(bt_deterministic_path,domain,problem)