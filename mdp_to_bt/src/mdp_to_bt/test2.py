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



class Test():
	def __init__(self, bt_path, domain, problem):

		# Inits
		self.bt_path = bt_path
		self.domain = domain
		self.problem = problem

		# Get states
		self.states = getStateList(domain, problem)
        self.states.reverse()
        self.num_states = len(states)

        # Generate policy from BT (recreating BT every state to reset activity)
        self.run()

    def run(self):

    	for i in range(self.num_states):

    		state = self.states[i]
    		cbtpol = CompareBTPolicy(self.bt_path, self.domain, self.problem) # Re-init BT
    		cbtpol.get_first_active_running_action(state) # Save action






class CompareBTPolicy():
    def __init__(self, bt_path, domain, problem):
        self.bt = BehaviorTree(bt_path)
        self.bt_interface = BT_Interface(self.bt)
        #policy = self.bt_to_policy(self.bt, domain, problem)

    def init_bt(self):
        # print("BT_Interface initialising BT...")
        for node in self.bt.nodes:
            node.init_ros() # this just has pass in it....
            # print(node.label)
        # print("BT finished init")

    def get_first_active_running_action(state):

    	# 


    def bt_to_policy3(self, bt, domain, problem):

        try:

            f = open("/home/scheidee/Desktop/AURO_results/policy_from_bt.txt", "w+")
            fa = open("/home/scheidee/Desktop/AURO_results/policy_actions_from_bt.txt", "w+")
            # f.write("hi\n")

            self.init_bt()
            states = getStateList(domain, problem)
            states.reverse() # reversing the list order so pop chooses them in order below
            num_states = len(states)
            #states = states[:5]
            state0 = states[0]            
            state_count = 0

            current_state_eval_done = True # Trigger update to first state
            running_active_actions = None
            first_active_running_action = None

            count = 0
            # saved = False
            done = False
            old_running_active_actions = None
            update_state = True

            while not rospy.is_shutdown():

                # self.bt.tick()
                # count += 1

                # Update state
                if len(states) >= 1 and update_state:
                    state = states.pop()
                    self.update_bt(state)
                    state_count +=1
                    f.write("State count: %d\n" %state_count)
                    f.write("State: %s\n" %str(state))
                    update_state = False
                    prev = first_active_running_action
                    first_active_running_action = None
                elif state_count == num_states and not done:
                    print("DONE")
                    done = True
                else:
                	self.bt.tick()
                	count += 1

                # Monitor active running actions
                active_actions = self.bt.getActiveActions()
                running_active_actions = self.get_running_actions_from_active_actions(active_actions)

                if count%100 and running_active_actions and not first_active_running_action:
                    print(running_active_actions)
                    first_active_running_action = running_active_actions[0]
                    first_active_running_action = first_active_running_action.split("(",1)[0] # removing (x: x)
                    f.write("Action: %s\n" %str(first_active_running_action))
                    fa.write("%s\n" %str(first_active_running_action))
                    update_state = True

  

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

    def update_bt(self, state):

        #follow method in compactness eval except from pddl state format not my format for state

        #print("condies%s\n" %self.bt.condition_nodes.keys()) # have x param references in names

        for i in range(len(state)):

            c_label_pure = state[i][0] # pure meaning 'found_mine' as opposed to the 'found_mine_x' found in the tree due to params
            c_label = self.get_c_label(c_label_pure)
            #print('c_label %s'%str(c_label))

            if state[i][-1]: #1
                boolean = True
            else:
                boolean = False
            self.bt_interface.setConditionStatus(c_label,boolean)

        return


if __name__ == "__main__":

    rospy.init_node("compare_bt_to_policy")
    graphviz_pub = rospy.Publisher('behavior_tree_graphviz', String, queue_size=1)
    compressed_pub = rospy.Publisher('behavior_tree_graphviz_compressed', String, queue_size=1)

    # Path to bt config
    bt_final_path = "/home/scheidee/auro_ws/src/policy_to_behavior_tree/behavior_tree/config/AURO_final_synthesized_BTs/bt_final.tree"
    bt_reorder_path = "/home/scheidee/auro_ws/src/policy_to_behavior_tree/behavior_tree/config/AURO_final_synthesized_BTs/bt_reorder.tree"

    # Path to domain file
    domain_path = "/home/scheidee/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine/final_domain.ppddl"

    # Path to problem file
    problem_path = "/home/scheidee/auro_ws/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine/problem.ppddl"

    # Get the domain and problem
    domain  = PDDLParser.parse(domain_path)
    problem = PDDLParser.parse(problem_path)

    cbtp = CompareBTPolicy(bt_final_path,domain,problem)