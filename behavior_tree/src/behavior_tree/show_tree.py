#!/usr/bin/env python

import rospy
import yaml
import rospkg

from behavior_tree import *
from behavior_tree_msgs.msg import Status, Active

import behavior_tree_graphviz as gv
import zlib
import sys

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
    compressed.data = zlib.compress(source)
    compressed_pub.publish(compressed)

if __name__ == '__main__':

    # Get behavior tree we want to show from command line argument
    print(sys.argv[0]) # file name
    print(sys.argv[1]) # argument

    bt_tree_file = sys.argv[1]

	# Initialise the node
    rospy.init_node('show_tree')
    # Get the config file etc
    rospack = rospkg.RosPack()
    filepath = rospack.get_path('behavior_tree') + "/config/" #+ rospy.get_param('~config')

    graphviz_pub = rospy.Publisher('behavior_tree_graphviz', String, queue_size=1)
    compressed_pub = rospy.Publisher('behavior_tree_graphviz_compressed', String, queue_size=1)

    try:

        #bt = BehaviorTree(filepath+ 'output_bt.tree')
	    #bt = BehaviorTree(filepath+ 'raw_output_bt.tree')
        #bt = BehaviorTree(filepath+ 'test_final_output_bt.tree')
        #bt = BehaviorTree(filepath+ 'human_final_bt.tree')
        bt = BehaviorTree(filepath+bt_tree_file)

        includes = [True, True, True, True, True, True, True, True, True, True,\
            True, True, True, True, True,True, True, True, True, True,\
            True, True, True, True, True,True, True, True, True, True]

        init_bt(bt)
        #bt.write_config('onr_example2.tree') # need to manually add in decorator nodes to config file/ implement it dur
        while not rospy.is_shutdown():   
        	tick_bt(bt)

    except rospy.ROSInterruptException: pass
