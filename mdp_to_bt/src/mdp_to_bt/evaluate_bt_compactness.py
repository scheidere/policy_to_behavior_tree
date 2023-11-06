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
import time
from ppddl_to_matrices import getStateList
from pypddl_parser.pddlparser import PDDLParser



class EvaluateBTCompactness():
    def __init__(self, config_filename, domain_path, problem_path):
        self.config_filename = config_filename
        self.domain  = PDDLParser.parse(domain_path)
        self.problem = PDDLParser.parse(problem_path)
        self.domain_path = domain_path
        self.problem_path = problem_path
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


    # def get_c_label(self, c_label_pure):

    #     # Condition labels in the BT currently contain "x" and "_"
    #     # We need to match these for update statuses 
    #     # Assumes condition names don't overlap 

    #     #print("YEE", self.bt.condition_nodes.keys())

    #     for label in self.bt.condition_nodes.keys():
    #         if c_label_pure in label:
    #             #print("YO", c_label_pure,label)
    #             return label
    #     return None # ERROR


    def get_condition_label(self, condition_term):

        num_c_terms = len(condition_term)-1 # Last term of state is 0/1 representing False/True

        for label in self.bt.condition_nodes.keys():
            found = True
            for j in range(num_c_terms): 

                if condition_term[j] not in label:
                    found = False
                    break

            # If here without break, all terms e.g. 'infant-orientation' and 'toward' in label so condition label found
            if found:
                return label


    def update_bt(self, state):

        # Takes state format generated from getStatesList that pulls from the PPDDL domain and problem directly
        # Does include constraints

        for i in range(len(state)):

            c = state[i] # e.g. ['infant-orientation', 'toward', 1] but could have more than one param like 'toward'

            c_label = self.get_condition_label(c)
            
            #print('c_label', c_label)

            if state[i][-1]: #1
                boolean = True
            else:
                boolean = False
            self.bt_interface.setConditionStatus(c_label,boolean)

        #input("ji")


    # def update_bt(self, state):

    #     # This works with the bool state format, but I had to go back to using the domain/problem to get the state list
    #     # This is because the constraints were being ignored (they are in the domain)

    #     # Condition label, T/F
    #     for c_label in list(state.keys()):
    #         print('c_label ' + c_label + "\n")
    #         self.bt_interface.setConditionStatus(c_label, state[c_label])

    #     #input("hiey")

    #     #rospy.loginfo("updating BT given new state") # Could probably just print instead

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
            start = time.time()

            # Create output file
            f = open("/home/scheidee/Desktop/AURO_results/activity_results.txt", "w+")
            f.write("Config: %s\n" %self.config_filename)
            f.write("Domain: %s\n" %self.domain_path)
            f.write("Problem: %s\n" %self.problem_path)
            f.write("Total number of condition nodes in tree: %d \n" %(self.count_condition_node_instances()))
            f.write("Total number of action nodes in tree: %d \n" %(self.count_action_node_instances()))

            self.init_bt()

            states = getStateList(self.domain, self.problem)

            conditions_per_actions = [] # active conditions per active actions
            episode_c_per_a_avgs = [] # active c per a
            all_states_evaluated = False

            f.write('Examining %d state(s)\n' %len(states))
            print('Examining %d state(s)\n' %len(states))

            tick_count = 1 # Overall tick counter
            episode = 1
            while not rospy.is_shutdown():

                active_actions = self.bt.getActiveActions()
                active_conditions = self.bt.getActiveConditions()
                # print('active_actions: ' + str(active_actions) + "\n") # do see more than one over 10k tick episodes... Not sure why, sticking to 100 ticks
                # print('active_conditions: ' + str(active_conditions) + "\n")

                if tick_count == 1: # First tick ONLY

                    # Get next state
                    state = states.pop()

                    # Update BT given new state
                    self.update_bt(state)


                elif tick_count%100==0: # Evaluate result of each state change over 10k ticks --> changed to 100

                    print("Episode %d" %episode)

                    print('state ' + str(state) + "\n")
                    print(len(states))

                    # Get average number of condition nodes per action nodes, C/A
                    avg_c_per_a = mean(conditions_per_actions)
                    episode_c_per_a_avgs.append(avg_c_per_a) # Retain episode averages to be averaged at end

                    # Reset raw C/A list
                    conditions_per_actions = []


                    if len(states) > 0:
                        state = states.pop()
                        self.update_bt(state)
                    else:
                        all_states_evaluated = True

                    episode+=1                    

                else: # Most ticks

                    # Get number of active condition nodes per active action nodes, C/A
                    self.bt.tick()

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

            t = time.time() - start
            print("Time: %f\n" %t)
            f.write("Time: " + str(t) + "\n")

        except rospy.ROSInterruptException: pass

    def run_old(self):

        try:
            start = time.time()

            # Create output file
            f = open("/home/scheidee/Desktop/AURO_results/activity_results.txt", "w+")
            f.write("Config: %s\n" %self.config_filename)
            f.write("Domain: %s\n" %self.domain)
            f.write("Problem: %s\n" %self.problem)
            f.write("Total number of condition nodes in tree: %d \n" %(self.count_condition_node_instances()))
            f.write("Total number of action nodes in tree: %d \n" %(self.count_action_node_instances()))

            self.init_bt()
            # self.condition_labels = list(self.bt.condition_nodes.keys())
            # state = self.define_state()
            # n = len(self.condition_labels) # get number of condition types
            # all_test_bools = self.get_all_bool_combos(n) # list of lists, each represents a state
            #all_test_bools = [[False, False, True, False, True],[False, False, True, True, True]]
            

            states = getStateList(self.domain, self.problem)

            conditions_per_actions = [] # active conditions per active actions
            episode_c_per_a_avgs = [] # active c per a
            all_states_evaluated = False

            # f.write('Examining %d state(s)\n' %len(all_test_bools))
            # print('Examining %d state(s)\n' %len(all_test_bools))

            f.write('Examining %d state(s)\n' %len(states))
            print('Examining %d state(s)\n' %len(states))

            tick_count = 1 # Overall tick counter
            episode = 0
            while not rospy.is_shutdown():
                
                # Every tick start
                self.bt.tick()
                current_statuses = self.get_all_node_statuses()
                c_stats = self.get_condition_statuses()
                active_actions = self.bt.getActiveActions()
                num_active_a = len(active_actions)
                active_conditions = self.bt.getActiveConditions()
                num_active_c = len(active_conditions)
                #print('active_actions: ' + str(active_actions) + "\n") # do see more than one over 10k tick episodes... Not sure why, sticking to 100 ticks
                #print('active_conditions: ' + str(active_conditions) + "\n")


                if tick_count == 1: # First tick ONLY

                    # Get a set of bools to define the state and remove from list
                    #current_bools = all_test_bools.pop()

                    # Get state based on current bools
                    #state = self.redefine_state(state, current_bools)

                    # Get next state
                    state = states.pop()

                    # Update BT given new state
                    self.update_bt(state)
                    self.bt.tick()

                    print("Episode %d" %episode)


                elif tick_count%1000==0: # Evaluate result of each state change over 10k ticks --> changed to 100

                    print('state ' + str(state) + "\n")
                    print(len(states))

                    self.bt.tick()

                    # Get average number of condition nodes per action nodes, C/A
                    avg_c_per_a = mean(conditions_per_actions)
                    episode_c_per_a_avgs.append(avg_c_per_a) # Retain episode averages to be averaged at end

                    # Reset raw C/A list
                    conditions_per_actions = []

                    # Get new bools for definition of new state (as long as list not empty) and remove from list
                    # if all_test_bools:
                    #     current_bools = all_test_bools.pop()
                    # else: # All states have been evaluated
                    #     all_states_evaluated = True

                    if len(states) == 0:
                        all_states_evaluated = True

                    # Get state based on current bools
                    #state = self.redefine_state(state, current_bools)
                        
                    # Update BT given new state

                    self.update_bt(state)
                    #self.bt.tick()

                    episode+=1
                    print("Episode %d" %episode)
                    

                else: # Most ticks

                    # Get number of active condition nodes per active action nodes, C/A
                    self.bt.tick()

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

            t = time.time() - start
            print("Time: %f\n" %t)
            f.write("Time: " + str(t) + "\n")

        except rospy.ROSInterruptException: pass



if __name__ == "__main__":

    rospy.init_node("eval_bt_compactness")

    # This ros param "tree" should be one of the names below, e.g., marine_simplified_bt or infant_raw_bt
    tree = rospy.get_param('~tree')
    print('Evaluating compactness of %s\n' %tree)

    # Get the domain and problem (to be used just to get the state list with condition constraints considered to remove invalide states)
    domain_path = rospy.get_param('~domain')
    problem_path = rospy.get_param('~problem')

    graphviz_pub = rospy.Publisher('behavior_tree_graphviz', String, queue_size=1)
    compressed_pub = rospy.Publisher('behavior_tree_graphviz_compressed', String, queue_size=1)

    # path_to_bts = "/home/parallels/auro_ws/src/policy_to_behavior_tree/behavior_tree/config/AURO_final_synthesized_BTs"
    path_to_bts = "/home/scheidee/auro_ws/src/policy_to_behavior_tree/behavior_tree/config/AURO_final_synthesized_BTs"

    marine_simplified_bt = "/marine/final_synth_bt.tree" # NEED TO REGEN post bug fix
    marine_raw_bt = "/marine/raw_policy_bt.tree" # NEED TO REGEN post bug fix
    marine_simplified_reorder_bt = "/marine/final_synth_bt_reorder.tree" # random action order reordered for simplification # NEED TO REGEN post bug fix
    marine_simplified_cares_bt = "/marine/final_synth_bt_cares.tree" # not removing dontcares # NEED TO REGEN post bug fix
    marine_simplified_deterministic_bt = "/marine/final_synth_bt_deterministic.tree" # final method, but deterministic version of specification
    marine_raw_deterministic_bt = "/marine/raw_policy_bt_deterministic.tree"

    infant_simplified_bt = "/infant/final_synth_bt.tree"
    infant_raw_bt = "/infant/raw_policy_bt.tree"
    # todo

    #tree = marine_simplified_bt
    #tree = marine_raw_bt
    #tree = marine_simplified_reorder_bt
    #tree = marine_simplified_cares_bt
    #tree = marine_simplified_deterministic_bt
    #tree = marine_raw_deterministic_bt # todo

    #tree = infant_simplified_bt 
    #tree = infant_raw_bt

    # To run this: 
    # roslaunch mdp_to_bt eval.launch tree:=/infant/name.tree domain:=... problem:=...
    # roslaunch mdp_to_bt eval.launch tree:=/marine/name.tree domain:=... problem:=...

    # tree = "/raw_policy_bt_MARINE.tree"
    config_filename = path_to_bts + tree

    ebtc = EvaluateBTCompactness(config_filename, domain_path, problem_path)


