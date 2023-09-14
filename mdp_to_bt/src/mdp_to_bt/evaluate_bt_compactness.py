#!/usr/bin/env python

import rospy
from behavior_tree.behavior_tree import *
import behavior_tree.behavior_tree_graphviz as gv
from behavior_tree_msgs.msg import Status, Active
import zlib


class EvaluateBTCompactness():
    def __init__(self, config_filename):
        self.config_filename = config_filename
        self.bt = BehaviorTree(config_filename)
        self.run()

    # def init_bt(self):
    #     # print("BT_Interface initialising BT...")
    #     for node in self.bt.nodes:
    #         node.init_ros() # this just has pass in it....
    #         # print(node.label)
    #     # print("BT finished init")

    def tick_bt(self):
        self.bt.tick() #root.tick(True)

        source = gv.get_graphviz(self.bt)
        source_msg = String()
        source_msg.data = source
        graphviz_pub.publish(source_msg) 

        compressed = String()
        compressed.data = zlib.compress(source.encode("utf-8"))
        compressed_pub.publish(compressed)

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

    def test(self):
        def get_publish_function(node, message_type, message_data):
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

    def run(self):

        try:
            
            #self.init_bt()

            self.print_condition_info()

            #self.bt.print_BT()
            #self.bt.changeConditionStatus()
            #bt.write_config('onr_example2.tree') # need to manually add in decorator nodes to config file/ implement it dur
            while not rospy.is_shutdown():   
                #self.tick_bt()
                source = gv.get_graphviz(self.bt)
                source_msg = String()
                source_msg.data = source
                graphviz_pub.publish(source_msg) 

                compressed = String()
                compressed.data = zlib.compress(source.encode("utf-8"))
                compressed_pub.publish(compressed)
                print("++++++++++++++++")
                self.print_condition_info()
                print("++++++++++++++++")

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


