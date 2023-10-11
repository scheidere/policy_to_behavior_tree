#!/usr/bin/env python

from ppddl_to_matrices import getStateList
import rospy
from bt_interface import *
from behavior_tree_msgs.msg import Status, Active
import zlib
import copy
import itertools
from statistics import mean
from pypddl_parser.pddlparser import PDDLParser
from bt_interface import *

class CompareBTPolicy():
    def __init__(self, bt_path, domain, problem):
        self.bt = BehaviorTree(bt_path)
        self.bt_interface = BT_Interface(self.bt)
        policy = self.bt_to_policy(self.bt, domain, problem)

    def init_bt(self):
        # print("BT_Interface initialising BT...")
        for node in self.bt.nodes:
            node.init_ros() # this just has pass in it....
            # print(node.label)
        # print("BT finished init")

    def bt_to_policy(self, bt, domain, problem):

        try:

            self.init_bt()
            states = getStateList(domain, problem)

            test_state = states[7] # explore then report
            #test_state = states[17]

            #print(test_state)

            # evaluate bt in this state, i.e. change condition statuses
            # and print first action to become active as a result

            # start each of these tests from bt completely inactive/reset


            policy = [] # list of action names (idk which nums they would be anyway)

            active_actions = self.bt.getActiveActions()
            print("BEFORE", active_actions)

            self.print_bt_cond_statuses()

            while not rospy.is_shutdown():

                self.bt.tick()
                self.update_bt(test_state)
                new_active_actions = self.bt.getActiveActions()
                if active_actions != new_active_actions:
                    active_actions = new_active_actions
                    print("++++++++++")
                    print('active actions: %s' %str(active_actions))
                    #statuses = self.get_condition_statuses()
                    #print('conds: %s\n' %str(statuses))
                    self.print_bt_cond_statuses()
                    print("==========")

        except rospy.ROSInterruptException: pass


        return policy

    def print_bt_cond_statuses(self):

        state_from_bt = []
        #print('yee', self.bt.condition_nodes)
        for c in self.bt.condition_nodes.keys():
            #print('test c', c)
            c_lst = self.bt.condition_nodes[c]
            status = c_lst[0].status.status
            state_from_bt.append([c,status])

        print(state_from_bt)

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