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
        policy = self.bt_to_policy(self.bt, domain, problem)

    def init_bt(self):
        # print("BT_Interface initialising BT...")
        for node in self.bt.nodes:
            node.init_ros() # this just has pass in it....
            # print(node.label)
        # print("BT finished init")

    def bt_to_policy(self, bt, domain, problem):

        try:

            # just for testing remote launch
            t = round(time.time())

            # f_path = "/home/scheidee/Desktop/AURO_results/policy_from_bt_YOO" + str(t) + ".txt"
            # fa_path = "/home/scheidee/Desktop/AURO_results/policy_actions_from_bt_YOO" + str(t) + ".txt"
            # f = open(f_path, "w+")
            # fa = open(fa_path, "w+")
            f = open("/home/scheidee/Desktop/AURO_results/policy_from_bt.txt", "w+")
            fa = open("/home/scheidee/Desktop/AURO_results/policy_actions_from_bt.txt", "w+")
            # f.write("hi\n")

            self.init_bt()
            states = getStateList(domain, problem)
            states.reverse() # reversing the list order so pop chooses them in order below
            num_states = len(states)
            #states = states[:5]   
            state0 = states[-1] # -1 or 0???            
            state_count = 0

            current_state_eval_done = True # Trigger update to first state
            running_active_actions = None
            first_active_running_action = None

            count = 0
            # saved = False
            done = False
            old_running_active_actions = None
            update_state = True
            state = None

            while not rospy.is_shutdown():

                self.bt.tick()
                count += 1

                # if intermediate_update_state:


                #     update_state = True
                #     intermediate_update_state = False

                # Update state
                if len(states) >= 1 and update_state:
                    # active_actions = self.bt.getActiveActions()
                    # running_active_actions = self.get_running_actions_from_active_actions(active_actions)
                    # print('pre update', running_active_actions)
                    # if state:
                    #     print('pre update', state)
                    # else:
                    #     print('no state yet')
                    state = states.pop()
                    self.update_bt(state)
                    state_count +=1
                    f.write("State count: %d\n" %state_count)
                    f.write("State: %s\n" %str(state))
                    update_state = False
                    prev = first_active_running_action
                    first_active_running_action = None
                    # active_actions = self.bt.getActiveActions()
                    # running_active_actions = self.get_running_actions_from_active_actions(active_actions)
                    # print('post update', running_active_actions)
                    # print('post update', state)

                elif state_count == num_states and not done:
                    print("DONE!!!")
                    done = True


                # Monitor active running actions
                active_actions = self.bt.getActiveActions()
                running_active_actions = self.get_running_actions_from_active_actions(active_actions)

                if count%100:

                    # print('running_active_actions', running_active_actions)
                    # print('state', state)

                    if running_active_actions and not first_active_running_action:
                        print(running_active_actions)
                        first_active_running_action = running_active_actions[0]
                        first_active_running_action = first_active_running_action.split("(",1)[0] # removing (x: x)
                        f.write("Action: %s\n" %str(first_active_running_action))
                        fa.write("%s\n" %str(first_active_running_action))
                        print(state)
                        print(first_active_running_action)
                        update_state = True
                        # intermediate_update_state = True
                    # elif not running_active_actions and first_active_running_action:
                    #     update_state = True # Only update state if there are no running active actions for state update clarity?


        except rospy.ROSInterruptException: pass


    def bt_to_policy2(self, bt, domain, problem):

        try:

            f = open("/home/scheidee/Desktop/AURO_results/policy_from_bt.txt", "w+")
            # f.write("hi\n")

            self.init_bt()
            states = getStateList(domain, problem)
            #states.reverse() # reversing the list order so pop chooses them in order below
            #states = states[:5]
            states = states[:5]
            state = None
            state_count = 0

            current_state_eval_done = True # Trigger update to first state
            running_active_actions = None
            first_active_running_action = None

            count = 0
            # saved = False
            exit_print = False
            old_running_active_actions = None

            while not rospy.is_shutdown():

                self.bt.tick()
                count += 1

                if not state:
                    print('change', state, len(states))
                    state = states.pop() # This pulls the states in order because the list has been reversed above
                    self.update_bt(state)
                    state_count += 1

                if count%100 == 0:
                    # f.write(str(state_count))
                    # if not saved and len(states) >= 1:
                    #     print('changin', len(states))
                    #     state = states.pop() # This pulls the states in order because the list has been reversed above
                    #     self.update_bt(state)
                    #     state_count += 1
                    #     saved = False
                    # elif not states and not exit_print and not running_active_actions: 
                    #     print("BT to policy check complete.")
                    #     exit_print = True

                    if not len(states) and not exit_print:
                        print("BT to policy check complete.")
                        exit_print = True
                        first_active_running_action = None
                
                    active_actions = self.bt.getActiveActions()
                    running_active_actions = self.get_running_actions_from_active_actions(active_actions)

                    # An active action is now running, so record it
                    if running_active_actions and not first_active_running_action:
                        print('running_active_actions', running_active_actions)
                        first_active_running_action = running_active_actions[0]

                    # No more running active actions, save previously recorded action and update to new state
                    if not running_active_actions and first_active_running_action:
                        print('Saving %s\n' %first_active_running_action)
                        f.write("%d %s: %s\n" %(state_count, str(state), first_active_running_action))
                        first_active_running_action = None # Reset
                        if len(states) >= 1:
                            print('changin', state, len(states))
                            state = states.pop() # This pulls the states in order because the list has been reversed above
                            self.update_bt(state)
                            first_active_running_action = None
                            state_count +=1



                    # new_running_active_actions = self.get_running_actions_from_active_actions(active_actions)

                    # if new_running_active_actions != running_active_actions:
                    #     running_active_actions = new_running_active_actions
                    #     print('running_active_actions', running_active_actions)
                    #     # If there are active running actions, and you haven't saved first running active action yet
                    #     if not first_active_running_action and running_active_actions: 
                    #         first_active_running_action = running_active_actions[0] # Save it
                    # elif new_running_active_actions == []: # Same as previous, staying at [], i.e., no running active actions
                    #     # Now that changes are done, do the save and move on to next state
                    #     #print(saved)

                    #     if first_active_running_action: # If there is one to save, i.e., not None, save it
                    #         # Save first active and running action with state
                    #         print('Saving %s\n' %first_active_running_action, len(states))
                    #         # first_active_running_action = running_active_actions[0] # should only be 1 because no parallel nodes
                    #         f.write("%d %s: %s\n" %(state_count, str(state), first_active_running_action))
                            
                    #         # Now that the first running active action for the last state was saved, update the state (if any left)
                    #         if len(states) >= 1:
                    #             print('changin', len(states))
                    #             state = states.pop() # This pulls the states in order because the list has been reversed above
                    #             self.update_bt(state)
                    #             first_active_running_action = None
                    #             state_count +=1





        except rospy.ROSInterruptException: pass

    def bt_to_policy_old(self, bt, domain, problem):

        try:

            f = open("/home/scheidee/Desktop/AURO_results/policy_from_bt.txt", "w+")

            self.init_bt()
            states = getStateList(domain, problem)
            states.reverse() # reversing the list order so pop chooses them in order below
            states = states[:2]

            #test_state = states[34]
            #test_state = states[17]


            #print(test_state)

            # evaluate bt in this state, i.e. change condition statuses
            # and print first action to become active as a result

            # start each of these tests from bt completely inactive/reset


            policy = [] # list of action names (idk which nums they would be anyway)

            active_actions = self.bt.getActiveActions()
            active_action_statuses = self.print_active_action_statuses(active_actions)
            print("BEFORE", active_actions)

            self.print_bt_cond_statuses()

            # first keep statuses all 0s for a while to give me a chance to get rqt up to see the change
            wait_for_rqt_count_threshold = 20000

            first_active_running_action = None

            current_state_eval_done = True # To trigger first state change immediately

            state_count = 0

            saved = False


            count = 0
            while not rospy.is_shutdown():

                count += 1

                if current_state_eval_done:
                    # reset state and thus begin next evaluation
                    if len(states) >= 1:
                        state = states.pop() # This pulls the states in order because the list has been reversed above
                        self.update_bt(state)
                        state_count += 1
                        saved = False
                    else: 
                        input("BT to policy check complete.")

                    current_state_eval_done = False

                self.bt.tick()
                #if count >= wait_for_rqt_count_threshold: # ONLY FOR RQT (to leave time to pull up window and view tree prior to status changes)
                # To use the delay line above for rqt, move everything up (aka not including) the rviz block
                #self.update_bt(test_state) # only for single state testing, not full bt to policy regeneration runs
                new_active_actions = self.bt.getActiveActions()
                new_active_action_statuses = self.print_active_action_statuses(active_actions)
                if active_actions != new_active_actions or new_active_action_statuses != active_action_statuses:
                    active_actions = new_active_actions
                    active_action_statuses = new_active_action_statuses
                    print("++++++++++")
                    print('Count: %d' %count)
                    print('active actions (+statuses): %s (%s)\n' %(str(active_actions),active_action_statuses))

                    #self.print_active_action_statuses(active_actions)
                    #statuses = self.get_condition_statuses()
                    #print('conds: %s\n' %str(statuses))
                    self.print_bt_cond_statuses()
                    running_active_actions = self.get_running_actions_from_active_actions(active_actions)
                    print('running_active_actions', running_active_actions)
                    print('2 active actions (+statuses): %s (%s)\n' %(str(active_actions),active_action_statuses))
                    # Save first active running action if you haven't already
                    if running_active_actions and not first_active_running_action:
                        if len(running_active_actions) == 1:
                            print("HELLO")
                            first_active_running_action = running_active_actions[0]
                        else: # Check for more than 1 running action at a time, without parallel nodes, there should only ever be 1
                            input("ERROR - Line 85 - bt_to_policy_equivalency_test.py")
                    elif not first_active_running_action and count%100 == 0: # No running active actions and haven't saved one
                        print('HERE')
                        # For cases like state 17, where it results in same outcome as baseline 0 state, so no actions listed as running
                        # Added this else to catch that, and avoid just returning None
                        first_active_running_action = active_actions[0] # first action that became active (presumably was running given baseline state)
                    
                    if first_active_running_action and not saved:
                        f.write("%d %s: %s\n" %(state_count, str(state), first_active_running_action))
                        current_state_eval_done = True # Trigger change to next state during next iteration
                        saved = True
                        print("ANSWER: %s\n" %first_active_running_action)
                        print('Count: %d' %count)
                        print("==========")

                # # This block is seemingly redundant with self.bt.tick() but seems to be needed for the rqt plugin
                source = gv.get_graphviz(self.bt)
                source_msg = String()
                source_msg.data = source
                graphviz_pub.publish(source_msg) 

                compressed = String()
                compressed.data = zlib.compress(source.encode("utf-8"))
                compressed_pub.publish(compressed)

                count += 1
                #print('Count %d\n' %count)
                #self.print_bt_cond_statuses()

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