#!/usr/bin/env python

import rospy
import yaml
import rospkg

from behavior_tree.behavior_tree import *
from behavior_tree_msgs.msg import Status, Active

import behavior_tree.behavior_tree_graphviz as gv
import zlib

def init_bt(bt):
    # print("BT_Interface initialising BT...")
    for node in bt.nodes:
        node.init_ros()
        # print(node.label)
    # print("BT finished init")

def tick_bt(bt):
    bt.tick()#root.tick(True)

    source = gv.get_graphviz(bt)
    source_msg = String()
    source_msg.data = source
    graphviz_pub.publish(source_msg) 

    compressed = String()
    #compressed.data = zlib.compress(source)
    compressed.data = zlib.compress(source.encode("utf-8"))
    compressed_pub.publish(compressed)

if __name__ == '__main__':

	# Initialise the node
    rospy.init_node('show_tree')
    # Get the config file etc

    graphviz_pub = rospy.Publisher('behavior_tree_graphviz', String, queue_size=1)
    compressed_pub = rospy.Publisher('behavior_tree_graphviz_compressed', String, queue_size=1)

    try:

        # marine
        #path = "/home/emily/Desktop/more_AURO_results/marine_final_1/"

        # infant
        path = "/home/emily/Desktop/more_AURO_results/infant_final/"


        bt_path = path + "final_synth_bt.tree"
        bt = BehaviorTree(bt_path)

        init_bt(bt)
        #bt.write_config('onr_example2.tree') # need to manually add in decorator nodes to config file/ implement it dur
        while not rospy.is_shutdown():   
        	tick_bt(bt)

    except rospy.ROSInterruptException: pass