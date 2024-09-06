#!/usr/bin/python3
import zlib


import behavior_tree_graphviz as gv
import cv2
import rospy
from std_msgs.msg import Bool, String


import behavior_tree as bt




class BehaviorTreeNode:
    def __init__(self, config_filename):
        self.tree = bt.BehaviorTree(config_filename)
        for node in self.tree.nodes:
            node.init_ros()


graphviz_msg = String()
compressed_msg = String()




def timer_callback(event):
    global graphviz_msg
    global compressed_msg
    changed = node.tree.tick()  # root.tick(True)
    # print('CHANGED: ', changed, only_publish_on_change)


    if True: # if changed
        source = gv.get_graphviz(node.tree)
        print("source ", source)
        graphviz_msg.data = source
        compressed_msg.data = zlib.compress(source.encode("utf-8"))


        if only_publish_on_change:
            graphviz_pub.publish(graphviz_msg)
            compressed_pub.publish(compressed_msg)


    if not only_publish_on_change:
        graphviz_pub.publish(graphviz_msg)
        compressed_pub.publish(compressed_msg)
    """
    img = gv.get_graphviz_image(source)
    cv2.imshow('img', img)
    cv2.waitKey(1)
    """




if __name__ == "__main__":
    rospy.init_node("behavior_tree_node")

    # Retrieve the parameter
    print("hiiiiiiiiiiii")
    config_filename = rospy.get_param("~config", "")
    print(f"Configuration file path: {config_filename}")  # Debugging line

    only_publish_on_change = False

    node = BehaviorTreeNode(config_filename)

    graphviz_pub = rospy.Publisher("behavior_tree_graphviz", String, queue_size=1)
    compressed_pub = rospy.Publisher("behavior_tree_graphviz_compressed", String, queue_size=1)
    timer = rospy.Timer(rospy.Duration(0.05), timer_callback)

    rospy.spin()


# #!/usr/bin/python
# import rospy
# from std_msgs.msg import String, Bool
# import behavior_tree as bt
# import behavior_tree_graphviz as gv
# import cv2
# import zlib

# class BehaviorTreeNode:
#     def __init__(self, config_filename):
#         self.tree = bt.BehaviorTree(config_filename)
#         for node in self.tree.nodes:
#             node.init_ros()

# def timer_callback(event):
#     node.tree.tick()#root.tick(True)

#     source = gv.get_graphviz(node.tree)
#     source_msg = String()
#     source_msg.data = source
#     graphviz_pub.publish(source_msg)

#     compressed = String()
#     compressed.data = zlib.compress(source)
#     compressed_pub.publish(compressed)
#     '''
#     img = gv.get_graphviz_image(source)
#     cv2.imshow('img', img)
#     cv2.waitKey(1)
#     '''

# if __name__ == '__main__':
#     rospy.init_node('behavior_tree_node')
    
#     config_filename = rospy.get_param('~config', '')
    
#     node = BehaviorTreeNode(config_filename)

#     graphviz_pub = rospy.Publisher('behavior_tree_graphviz', String, queue_size=1)
#     compressed_pub = rospy.Publisher('behavior_tree_graphviz_compressed', String, queue_size=1)
#     timer = rospy.Timer(rospy.Duration(0.05), timer_callback)

#     rospy.spin()
