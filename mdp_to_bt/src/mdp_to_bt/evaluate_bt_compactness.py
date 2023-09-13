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

    def init_bt(self):
        # print("BT_Interface initialising BT...")
        for node in self.bt.nodes:
            node.init_ros()
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

    def run(self):

        try:
            
            self.init_bt()
            #self.bt.print_BT()
            self.bt.makeActionActive("report")
            #bt.write_config('onr_example2.tree') # need to manually add in decorator nodes to config file/ implement it dur
            while not rospy.is_shutdown():   
                 self.tick_bt()
            #     self.bt.countActiveConditions()

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


