#!/usr/bin/env python

import rospy
from behavior_tree.behavior_tree import *
import behavior_tree.behavior_tree_graphviz as gv
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
        self.run()

    def init_bt(self):
        # print("BT_Interface initialising BT...")
        for node in self.bt.nodes:
            node.init_ros() # this just has pass in it....
            # print(node.label)
        # print("BT finished init")

    def tick_bt(self):
        self.bt.tick() #root.tick(True)

        # if self.bt_at_previous_tick: # i.e., it don't check during first tick, no comparison
        #     self.at_node_activity_equilibrium(self.bt_at_previous_tick)

        source = gv.get_graphviz(self.bt)
        source_msg = String()
        source_msg.data = source
        graphviz_pub.publish(source_msg) 

        compressed = String()
        compressed.data = zlib.compress(source.encode("utf-8"))
        compressed_pub.publish(compressed)

        #self.bt_at_previous_tick = copy.deepcopy(self.bt)


    # def get_publish_function(widget, button, other_buttons, node, message_type, message_data):
    #     pub = rospy.Publisher(node.get_subscriber_name(), message_type, queue_size=1)

    # def active_callback(msg):
    #     id_container.id = msg.id

    # def add_publish_function():
    #     def publish_function():
    #         if message_data != 'NO MESSAGE':
    #             msg = message_type()
    #             if isinstance(msg, Bool):
    #                 msg.data = message_data
    #             elif isinstance(msg, Status):
    #                 msg.status = message_data
    #                 msg.id = id_container.id
    #             pub.publish(msg)

    def get_publish_function(self, node, message_type, message_data):
                pub = rospy.Publisher(node.get_subscriber_name(), message_type, queue_size=1)
                class IDContainer:
                    def __init__(self):
                        self.id = 0
                id_container = IDContainer()
                def active_callback(msg):
                    id_container.id = msg.id
                if isinstance(message_type(), Status):
                    print(node.get_publisher_name())
                    sub = rospy.Subscriber(node.get_publisher_name(), Active, active_callback)
                def add_publish_function():
                    def publish_function():
                        if message_data != 'NO MESSAGE':
                            msg = message_type()
                            if isinstance(msg, Bool):
                                msg.data = message_data
                            elif isinstance(msg, Status):
                                msg.status = message_data
                                msg.id = id_container.id
                            pub.publish(msg)

    def print_condition_info(self):

        print(self.bt.condition_nodes)
        for key in self.bt.condition_nodes:
            for condition_node in self.bt.condition_nodes[key]: # Might be multiple instances of a certain type (key) of condition
                print('\ninstance: ' + condition_node.label)
                print("is_active: " + str(condition_node.is_active))
                print("status: " + str(condition_node.status.__str__()))

    def changeNodeActivtyOrStatus(self):

        # I think this should make the given node active as well as assign the noted status
        # first test node label: "drop-off(x: x)"

        for node in self.bt.nodes:
            if node.label == 'drop-off(x: x)':
                if isinstance(node, Action):
                    message_data = Status.SUCCESS
                    self.get_publish_function(node, Status, message_data)

                elif isinstance(node, Condition):
                    message_data = True
                    self.get_publish_function(node, Bool, message_data)


    # def at_node_activity_equilibrium(self):
    #     '''
    #      - Returns Bool denoting whether or not there has been a change in any node's status or activity
    #      between current and previous tick
    #      - Returns True if no change
    #     '''
    #     print('================== CHECKING IF AT EQUIL ===================')
    #     for i in range(len(self.bt.nodes)):
    #         n1 = self.bt_at_previous_tick.nodes[i]
    #         n2 = self.bt.nodes[i]
    #         # print(n1.label, n1.status.status)
    #         # print(n2.label, n2.status.status)
    #         if n1.status.status != n2.status.status:
    #             print(n1.label, n1.status.status)
    #             print(n2.label, n2.status.status)
    #             print('==========/========== NOT AT EQUIL ==========/==========')
    #             return False

    #     print('================== AT EQUIL ===================')
    #     return True



        # for n1 in self.bt_at_previous_tick.nodes:
        #     for n2 in self.bt.nodes:
        #         if n1.status != n2.status:
        #             print(n1.label, n1.status.status.__str__())
        #             print(n2.label, n2.status.status)
        #             print("not same")
        #             print('==========/========== NOT AT EQUIL ==========/==========')
        #             return False
        #         else:
        #             print(n1.label, n1.status.status)
        #             print(n2.label, n2.status.status)
        #             print("same")

        # print('================== AT EQUIL ===================')
        #return True

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



    def run(self):

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


