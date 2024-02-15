#!/usr/bin/env python

import roslaunch
import rospy

import subprocess

def main():


    # Run bt_to_policy_equivalency_test.py (compare.launch) from here

    # print("Starting roscore")
    # roscore = subprocess.Popen('roscore')
    # time.sleep(1)  # wait a bit to be sure the roscore is really launched


    print("Initializating node")

    rospy.init_node("compare_bt_to_policy", anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/scheidee/auro_ws/src/policy_to_behavior_tree/mdp_to_bt/launch/compare.launch"])

    print("Launching")
    launch.start()
    rospy.loginfo("started")

    

    rospy.sleep(10)
    print("Shutting down")
    launch.shutdown()

    # The below does not happen

    print("Launching 2")
    rospy.init_node("compare_bt_to_policy", anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/scheidee/auro_ws/src/policy_to_behavior_tree/mdp_to_bt/launch/compare.launch"])
    launch.start()
    rospy.sleep(10)
    print("Shutting down 2")
    launch.shutdown()


    # After the works, make new launch file to point to file that saves just one action given a state

def main2():

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    cli_args1 = ['mdp_to_bt', 'compare.launch']
    cli_args2 = ['mdp_to_bt', 'compare.launch']
    roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0]

    roslaunch_file2 = roslaunch.rlutil.resolve_launch_arguments(cli_args2)[0]

    launch_files = [roslaunch_file1, roslaunch_file2]

    parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

    parent.start()

    # Doesn't work because the nodes have to be different, seems to be simultaneous execution



if __name__ == "__main__":

    main2()