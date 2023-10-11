#!/usr/bin/env python

import rospy
from behavior_tree.behavior_tree import *
import behavior_tree.behavior_tree_graphviz as gv
from bt_interface import *
from behavior_tree_msgs.msg import Status, Active
import zlib
import copy
import itertools
from statistics import mean


class EvaluateBTCompactness():
    def __init__(self, config_filename):
        self.config_filename = config_filename
        self.bt = BehaviorTree(config_filename)
        self.bt_at_previous_tick = None
        self.old_statuses = None
        self.prev_active_actions = None
        self.prev_active_conditions = None
        self.bt_interface = BT_Interface(self.bt)
        #self.less_complex_run()
        self.run()


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

    #def redefine_state(self, state, change_index):
    def redefine_state(self, state, new_bools):

        # The change_index denotes which type of condition we are changing the status of
        # T->F or F->T

        # print('change_index: %d' %change_index)
        # print(state)
        # print("hi" + str(self.condition_labels[change_index]))

        for i in range(len(state)):
            new_bool = new_bools[i]
            state[self.condition_labels[i]] = new_bool

        # change_label = self.condition_labels[change_index]
        # if state[change_label] == True:
        #     state[change_label] = False
        # elif state[change_label] == False:
        #     # print('CHANGE')
        #     state[change_label] = True

        # print(state)
        return state


    def get_all_bool_combos(self, n):

        return [list(i) for i in itertools.product([False,True], repeat=n)]


    def update_bt(self, state):

        # Condition label, T/F
        for c_label in list(state.keys()):
            self.bt_interface.setConditionStatus(c_label, state[c_label])

        #rospy.loginfo("updating BT given new state") # Could probably just print instead

    def get_active_action_and_condition_nodes(self):

        # instances of active action or condition nodes, including duplicates of the same type (different location in tree)

        active_action_nodes = []
        active_condition_nodes = []
        for node in self.bt.nodes:
            if node.is_active and isinstance(node,Action):
                active_action_nodes.append(node)
            elif node.is_active and isinstance(node,Condition):
                active_condition_nodes.append(node)

        return active_action_nodes, active_condition_nodes

    def count_condition_node_instances(self):

        condition_count = 0
        for lst in self.bt.condition_nodes.values():
            condition_count += len(lst)

        return condition_count

    def count_action_node_instances(self):

        action_count = 0
        for lst in self.bt.action_nodes.values():
            action_count += len(lst)

        return action_count

    def run(self):
        try:

            # Create output file
            f = open("/home/scheidee/Desktop/AURO_results/activity_results.txt", "w+")
            f.write("Config: %s\n" %self.config_filename)
            f.write("Total number of condition nodes in tree: %d \n" %(self.count_condition_node_instances()))
            f.write("Total number of action nodes in tree: %d \n" %(self.count_action_node_instances()))

            self.init_bt()
            self.condition_labels = list(self.bt.condition_nodes.keys())
            state = self.define_state()
            n = len(self.condition_labels) # get number of condition types
            all_test_bools = self.get_all_bool_combos(n) # list of lists, each represents a state
            #all_test_bools = [[False, False, True, False, True],[False, False, True, True, True]]
            conditions_per_actions = [] # active conditions per active actions
            episode_c_per_a_avgs = [] # active c per a
            all_states_evaluated = False

            f.write('Examining %d state(s)\n' %len(all_test_bools))
            print('Examining %d state(s)\n' %len(all_test_bools))

            tick_count = 1 # Overall tick counter
            episode = 1
            while not rospy.is_shutdown():
                
                # Every tick start
                self.bt.tick()
                current_statuses = self.get_all_node_statuses()
                c_stats = self.get_condition_statuses()
                active_actions = self.bt.getActiveActions()
                num_active_a = len(active_actions)
                active_conditions = self.bt.getActiveConditions()
                num_active_c = len(active_conditions)


                if tick_count == 1: # First tick ONLY

                    # Get a set of bools to define the state and remove from list
                    current_bools = all_test_bools.pop()

                    # Get state based on current bools
                    state = self.redefine_state(state, current_bools)

                    # Update BT given new state
                    self.update_bt(state)

                    print("Episode %d" %episode)


                elif tick_count%10000==0: # Evaluate result of each state change over 10k ticks

                    # Get average number of condition nodes per action nodes, C/A
                    avg_c_per_a = mean(conditions_per_actions)
                    episode_c_per_a_avgs.append(avg_c_per_a) # Retain episode averages to be averaged at end

                    # Reset raw C/A list
                    conditions_per_actions = []

                    # Get new bools for definition of new state (as long as list not empty) and remove from list
                    if all_test_bools:
                        current_bools = all_test_bools.pop()
                    else: # All states have been evaluated
                        all_states_evaluated = True

                    # Get state based on current bools
                    state = self.redefine_state(state, current_bools)

                    # Update BT given new state
                    self.update_bt(state)

                    episode+=1
                    print("Episode %d" %episode)
                    

                else: # Most ticks

                    # Get number of active condition nodes per active action nodes, C/A

                    # Number of active condition/action nodes
                    active_a_nodes, active_c_nodes = self.get_active_action_and_condition_nodes()

                    # Check if c,a pair are unique, only save if so for efficiency
                    # Add ^^^ later if necessary to alleviate slowness

                    # Add C/A to list for current state/10k tick episode
                    if active_a_nodes: # Check at least one action is active, otherwise ignore
                        conditions_per_actions.append(len(active_c_nodes)/len(active_a_nodes))


                if all_states_evaluated:

                    # Get final average C/A
                    avg_active_conditions_per_active_action = mean(episode_c_per_a_avgs)
                    f.write("Number of episodes: %d\n" %len(episode_c_per_a_avgs))
                    f.write("Average active conditions per action per episode: \n%s\n" %str(episode_c_per_a_avgs))
                    f.write("The average number of active conditions per active action is: %f\n" % avg_active_conditions_per_active_action)
                    print("Evaluation complete.")
                    rospy.spin()

                # # This block is seemingly redundant with self.bt.tick() but seems to be needed for the rqt plugin
                source = gv.get_graphviz(self.bt)
                source_msg = String()
                source_msg.data = source
                graphviz_pub.publish(source_msg) 

                compressed = String()
                compressed.data = zlib.compress(source.encode("utf-8"))
                compressed_pub.publish(compressed)

                tick_count += 1

        except rospy.ROSInterruptException: pass



if __name__ == "__main__":

    rospy.init_node("eval_bt_compactness")
    graphviz_pub = rospy.Publisher('behavior_tree_graphviz', String, queue_size=1)
    compressed_pub = rospy.Publisher('behavior_tree_graphviz_compressed', String, queue_size=1)

    # path_to_bts = "/home/parallels/auro_ws/src/policy_to_behavior_tree/behavior_tree/config/AURO_final_synthesized_BTs"
    path_to_bts = "/home/scheidee/auro_ws/src/policy_to_behavior_tree/behavior_tree/config/AURO_final_synthesized_BTs"

    marine_simplified_bt = "/marine/final_synth_bt.tree"
    marine_raw_bt = "/marine/raw_policy_bt.tree"
    marine_simplified_reorder_bt = "/marine/final_synth_bt_reorder.tree" # random action order reordered for simplification
    marine_simplified_cares_bt = "/marine/final_synth_bt_cares.tree" # not removing dontcares

    infant_simplified_bt = "/infant/final_synth_bt.tree"
    infant_raw_bt = "/infant/raw_policy_bt.tree"

    tree = marine_simplified_bt
    #tree = marine_raw_bt
    #tree = marine_simplified_reorder_bt
    #tree = marine_simplified_cares_bt

    #tree = infant_simplified_bt 
    #tree = infant_raw_bt

    config_filename = path_to_bts + tree

    ebtc = EvaluateBTCompactness(config_filename)


