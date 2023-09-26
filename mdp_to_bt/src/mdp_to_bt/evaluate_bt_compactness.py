#!/usr/bin/env python

import rospy
from behavior_tree.behavior_tree import *
import behavior_tree.behavior_tree_graphviz as gv
from bt_interface import *
from behavior_tree_msgs.msg import Status, Active
import zlib
import copy



class EvaluateBTCompactness():
    def __init__(self, config_filename):
        self.config_filename = config_filename
        self.bt = BehaviorTree(config_filename)
        self.bt_at_previous_tick = None
        self.old_statuses = None
        self.prev_active_actions = None
        self.prev_active_conditions = None
        #self.run()
        self.bt_interface = BT_Interface(self.bt)
        self.less_complex_run()

    def init_bt(self):
        # print("BT_Interface initialising BT...")
        for node in self.bt.nodes:
            node.init_ros() # this just has pass in it....
            # print(node.label)
        # print("BT finished init")

    def tick_bt(self):
        self.bt.tick() #root.tick(True)

        source = gv.get_graphviz(self.bt)
        source_msg = String()
        source_msg.data = source
        graphviz_pub.publish(source_msg) 

        compressed = String()
        compressed.data = zlib.compress(source.encode("utf-8"))
        compressed_pub.publish(compressed)

    def print_condition_info(self):

        print(self.bt.condition_nodes)
        for key in self.bt.condition_nodes:
            for condition_node in self.bt.condition_nodes[key]: # Might be multiple instances of a certain type (key) of condition
                print('\ninstance: ' + condition_node.label)
                print("is_active: " + str(condition_node.is_active))
                print("status: " + str(condition_node.status.__str__()))

    def get_all_node_activities(self):
        ## subscribe to the topics
        pass

    def get_all_node_statuses(self):

        statuses = []
        for n in self.bt.nodes:
            statuses.append(n.status.status)

        return statuses

    def get_condition_statuses(self):

        # Self.bt.condition_nodes is a dict where keys are conditions labels and values are instances
        # Returns statuses in a list of ALL condition node instances (not just by type)

        statuses = []
        for lst in list(self.bt.condition_nodes.values()):
            for c in lst:
                statuses.append(c.status.status)

        return statuses

    def at_node_status_equilibrium(self, old_statuses, current_statuses):
        '''
            - Returns True if no change between current and previous statuses of every node in BT
            - Else returns False
        '''

        return old_statuses == current_statuses

    def at_action_activity_equilibrium(self, prev_active_actions, active_actions):

        for i in range(len(active_actions)):
            pass

    def record_active_conds_per_active_action(self, active_actions, active_conditions, active_conds_per_active_actions):

        # For a certain BT static state
        
        for a in active_actions:
            active_conds_per_active_actions[a].append(len(active_conditions))

        return active_conds_per_active_actions

    def define_state(self):

        # Initial state, all conditions have a status of Failure

        state = {}
        for c in self.condition_labels:
            state[c] = False

        return state

    def redefine_state(self, state, change_index):

        # The change_index denotes which type of condition we are changing the status of
        # T->F or F->T

        # print('change_index: %d' %change_index)
        # print(state)
        # print("hi" + str(self.condition_labels[change_index]))

        change_label = self.condition_labels[change_index]
        if state[change_label] == True:
            state[change_label] = False
        elif state[change_label] == False:
            # print('CHANGE')
            state[change_label] = True

        # print(state)
        return state

    def update_bt(self, state):

        # Condition label, T/F
        for c_label in list(state.keys()):
            self.bt_interface.setConditionStatus(c_label, state[c_label])

        #rospy.loginfo("updating BT given new state") # Could probably just print instead

    def less_complex_run(self):
        try:
            
            self.init_bt()
            self.condition_labels = list(self.bt.condition_nodes.keys())
            state = self.define_state()

            # Create output file
            f = open("/home/scheidee/Desktop/AURO_results/activity_results.txt", "w+")

            count = 1
            while not rospy.is_shutdown():
                #states = [[]] 
                #for state in states:
                #print('Tick %d' % count) 
                
                self.bt.tick()
                c_stats = self.get_condition_statuses()
                active_actions = self.bt.getActiveActions()
                num_active_a = len(active_actions)
                active_conditions = self.bt.getActiveConditions()
                num_active_c = len(active_conditions)
                f.write("Active actions: " + str(active_actions) + ", %d\n" %num_active_a)
                f.write("Active conditions: " + str(active_conditions) + ", %d\n" %num_active_c)
                f.write("Current condition statuses (from BT): " + str(c_stats) + "\n")
                #print(state)
                f.write("======\n")
                f.write("Changing state (locally, i.e., independent of BT)\n")
                f.write("Before: " + str(state) +"\n")
                if count == 1:
                    state = self.redefine_state(state,2)
                    state = self.redefine_state(state,4)
                #state = self.redefine_state(state,0)
                # print(state)
                f.write("After: " + str(state) +"\n")
                f.write("++++++\n")

                self.update_bt(state) 

                # This block is seemingly redundant with self.bt.tick() but seems to be needed for the rqt plugin
                source = gv.get_graphviz(self.bt)
                source_msg = String()
                source_msg.data = source
                graphviz_pub.publish(source_msg) 

                compressed = String()
                compressed.data = zlib.compress(source.encode("utf-8"))
                compressed_pub.publish(compressed)

                #rospy.spin()
                count += 1

        except rospy.ROSInterruptException: pass

    def run(self):

        # deprecated

        try:
            
            self.init_bt()

            # Create output file
            f = open("/home/scheidee/Desktop/AURO_results/activity_results.txt", "w+")

            #self.print_condition_info()

            # node_label = 'test'
            # node = Action(node_label)
            # message_type = Status
            # message_data = Status.SUCCESS
            # #message_data = 'NO MESSAGE'
            # print(node, message_type, message_data)
            # input('yo')

            #self.bt.print_BT()
            #self.bt.changeConditionStatus()
            #bt.write_config('onr_example2.tree') # need to manually add in decorator nodes to config file/ implement it dur
            count = 1
            while not rospy.is_shutdown():  
                #print('Tick %d' % count) 
                
                self.bt.tick()

                current_statuses = self.get_all_node_statuses()
                current_condition_statuses = self.get_condition_statuses()
                active_actions = self.bt.getActiveActions()
                num_active_a = len(active_actions)
                active_conditions = self.bt.getActiveConditions()
                num_active_c = len(active_conditions)

                if count == 1:
                    active_conds_per_active_actions = {}
                    action_labels = list(self.bt.action_nodes.keys())
                    for k in action_labels:
                        active_conds_per_active_actions[k] = []
                    print(active_conds_per_active_actions)
                    f.write("Tick %d\n" %count)
                    #f.write("Node statuses: " + str(current_statuses) + "\n") # all node statuses
                    f.write('Condition order: ' + str(list(self.bt.condition_nodes.keys())) + "\n")
                    f.write("Condition statuses: " + str(current_condition_statuses)+ "\n") # just condition statuses
                    f.write("Active actions: " + str(active_actions) + ", %d\n" %num_active_a)
                    f.write("Active conditions: " + str(active_conditions) + ", %d\n" %num_active_c)
                    f.write("Tracker (list of active condition nums per active action): \n" + str(active_conds_per_active_actions) + "\n")
                    f.write("+++++++++++++++++++++ EVALUATION BELOW +++++++++++++++++++++\n")

                elif count > 1:
                    if not self.at_node_status_equilibrium(self.old_statuses, current_statuses):
                        f.write("Tick %d\n" %count)
                        #f.write("Node statuses: " + str(current_statuses) + "\n") # all node statuses
                        f.write("Condition statuses: " + str(current_condition_statuses)+ "\n") # just condition statuses
                        f.write("Active actions: " + str(active_actions) + ", %d\n" %num_active_a)
                        f.write("Active conditions: " + str(active_conditions) + ", %d\n" %num_active_c)
                        # print("Tick %d\n" %count)
                        # print("Node statuses: " + str(current_statuses) + "\n")
                        # print("Active actions: " + str(active_actions) + "\n")
                        # print("Active conditions: " + str(active_conditions) + "\n")

                        active_conds_per_active_actions = self.record_active_conds_per_active_action(active_actions, active_conditions, active_conds_per_active_actions)
                        f.write("Tracker: " + str(active_conds_per_active_actions) + "\n")


                # if self.bt_at_previous_tick:
                #     self.at_node_activity_equilibrium()

                #self.bt_at_previous_tick = copy.deepcopy(self.bt) # how do i copy this without breaking things 
                # self.bt.print_BT()

                # Print when statuses of any nodes change in the tree
                # current_statuses = self.get_all_node_statuses()
                # active_actions = self.bt.getActiveActions()
                # active_conditions = self.bt.getActiveConditions()

                # if self.old_statuses and not self.at_node_status_equilibrium(self.old_statuses, current_statuses):
                #     f.write("Tick %d\n" %count)
                #     f.write('old' + str(self.old_statuses))
                #     print('old', self.old_statuses)
                #     print('new', current_statuses)
                #     print("++++++++++ CHANGE OCCURRED ++++++++++")

                

                # # Print when node activity change occurs
                # if self.prev_active_actions and self.prev_active_actions != active_actions:  
                #     print('prev_active_actions ', self.prev_active_actions)
                #     print('active_actions ', active_actions)
                # if self.prev_active_conditions and self.prev_active_conditions != active_conditions:  
                #     print('prev_active_conditions ', self.prev_active_conditions)
                #     print('active_conditions ', active_conditions)

                # This block is redundant with self.bt.tick() but seems to be needed 
                source = gv.get_graphviz(self.bt)
                source_msg = String()
                source_msg.data = source
                graphviz_pub.publish(source_msg) 

                compressed = String()
                compressed.data = zlib.compress(source.encode("utf-8"))
                compressed_pub.publish(compressed)

                self.old_statuses = copy.deepcopy(current_statuses)
                self.prev_active_actions = copy.deepcopy(active_actions)
                self.prev_active_conditions = copy.deepcopy(active_conditions)
                count += 1

        except rospy.ROSInterruptException: pass

if __name__ == "__main__":

    rospy.init_node("eval_bt_compactness")
    graphviz_pub = rospy.Publisher('behavior_tree_graphviz', String, queue_size=1)
    compressed_pub = rospy.Publisher('behavior_tree_graphviz_compressed', String, queue_size=1)

    # path_to_bts = "/home/parallels/auro_ws/src/policy_to_behavior_tree/behavior_tree/config/AURO_final_synthesized_BTs"
    path_to_bts = "/home/scheidee/auro_ws/src/policy_to_behavior_tree/behavior_tree/config/AURO_final_synthesized_BTs"

    marine_simplified_bt = "/marine/final_synth_bt.tree"

    config_filename = path_to_bts + marine_simplified_bt

    ebtc = EvaluateBTCompactness(config_filename)


