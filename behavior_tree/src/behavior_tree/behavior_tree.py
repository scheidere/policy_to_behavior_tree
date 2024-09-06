#!/usr/bin/python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
from behavior_tree_msgs.msg import Status, Active
import sys
import rospkg

#================================================================================================================
# ----------------------------------------------- Return Status  ------------------------------------------------
#================================================================================================================

class ReturnStatus:
    def __init__(self, status):
        if status == Status.FAILURE or status == Status.RUNNING or status == Status.SUCCESS:
            self.status = status
        else:
            print(('Invalid return status: ' + status +', defaulting to FAILURE.'))
            self.status = Status.FAILURE

    def __eq__(self, other):
        return self.status == other.status
    
    def __ne__(self, other):
        return self.status != other.status
    
    def __str__(self):
        return self.status

ReturnStatus.FAILURE = ReturnStatus(Status.FAILURE)
ReturnStatus.RUNNING = ReturnStatus(Status.RUNNING)
ReturnStatus.SUCCESS = ReturnStatus(Status.SUCCESS)

#================================================================================================================
# ----------------------------------------------- Base Node Class -----------------------------------------------
#================================================================================================================

class Node(object):
    max_wait_time = 1.0
    def __init__(self, label):
        self.label = label
        self.children = []
        self.is_active = False
        self.status = ReturnStatus.FAILURE
        #self.max_wait_time = 1.0
    
    def add_child(self, node):
        self.children.append(node)

    def tick(self, active, traversal_count):
        print(('tick not implemented for node ' + self.label))
    
    def init_ros(self):
        pass

    def print_node(self):
        print((self.label))

#================================================================================================================
# ---------------------------------------------- Control Flow Nodes ---------------------------------------------
#================================================================================================================

class ControlFlowNode(Node):
    label = 'Unlabeled Control Flow Node'
    def __init__(self):
        Node.__init__(self, self.label)

class Fallback(ControlFlowNode):
    label = '?'
    
    def __init__(self):
        ControlFlowNode.__init__(self)
    
    def tick(self, active, traversal_count):
        if not active:
            for child in self.children:
                child.tick(False, traversal_count)
        else:
            self.status = ReturnStatus.FAILURE
            for child in self.children:
                if self.status == ReturnStatus.FAILURE:
                    child.tick(True, traversal_count)
                    self.status = child.status
                else:
                    child.tick(False, traversal_count)
        
        self.is_active = active

class Sequence(ControlFlowNode):
    label = '->' #u'\u2192' # arrow
    
    def __init__(self):
        ControlFlowNode.__init__(self)
    
    def tick(self, active, traversal_count):
        if not active:
            for child in self.children:
                child.tick(False, traversal_count)
        else:
            self.status = ReturnStatus.SUCCESS
            for child in self.children:
                if self.status == ReturnStatus.SUCCESS:
                    child.tick(True, traversal_count)
                    self.status = child.status
                else:
                    child.tick(False, traversal_count)
        
        self.is_active = active

class Parallel(ControlFlowNode):
    label = '||'

    def __init__(self, child_success_threshold):
        ControlFlowNode.__init__(self)
        self.child_success_threshold = child_success_threshold

    def tick(self, active, traversal_count):
        if not active:
            for child in self.children:
                child.tick(False, traversal_count)
        else:
            child_success_count = 0
            child_failure_count = 0
            for child in self.children:
                child.tick(True, traversal_count)
                if child.status == ReturnStatus.SUCCESS:
                    child_success_count += 1
                elif child.status == ReturnStatus.FAILURE:
                    child_failure_count += 1

            if child_success_count >= self.child_success_threshold:
                self.status = ReturnStatus.SUCCESS
            elif child_failure_count >= len(self.children) - self.child_success_threshold + 1:
                self.status = ReturnStatus.FAILURE
            else:
                self.status = ReturnStatus.RUNNING

        self.is_active = active
        
        
#================================================================================================================
# ----------------------------------------------- Execution Nodes -----------------------------------------------
#================================================================================================================

def get_condition_topic_name(name):
    topic_name = name.lower().replace(' ', '_') + '_success'
    return topic_name

class ExecutionNode(Node):
    def __init__(self, label):
        Node.__init__(self, label)
        self.publisher = None
        self.subscriber = None
        self.status_modification_time = None
    
    def init_ros(self):
        self.init_publisher()
        self.init_subscriber()

    def get_publisher_name(self):
        pass
    
    def init_publisher(self):
        pass

    def get_subscriber_name(self):
        raise NotImplementedError
        
    def init_subscriber(self):
        raise NotImplementedError

    def set_status(self, status):
        self.status = status
        self.status_modification_time = rospy.Time.now()
    
    def get_status_age(self):
        if self.status_modification_time == None:
            return float('inf')
        else:
            return (rospy.Time.now() - self.status_modification_time).to_sec()

class Condition(ExecutionNode):
    def __init__(self, label):
        ExecutionNode.__init__(self, label)
        self.current_traversal_count = -1
        self.is_newly_active = False

    def tick(self, active, traversal_count):
        if self.current_traversal_count != traversal_count:
            self.current_traversal_count = traversal_count
        else:
            if self.is_active:
                return
        
        # Condition activity/status before
        # if self.is_active == False and active == True and self.status == ReturnStatus.FAILURE:
        #     self.set_status(ReturnStatus.FAILURE)#RUNNING)
        # if self.get_status_age() > self.max_wait_time:
        #     self.set_status(ReturnStatus.FAILURE)

        # Condition activity/status now
        if self.is_active == False and active == True:
            self.is_newly_active = True
            if self.status == ReturnStatus.FAILURE:
                self.set_status(ReturnStatus.FAILURE)
        else:
            self.is_newly_active = False

        # if self.get_status_age() > self.max_wait_time:
        #     self.set_status(ReturnStatus.FAILURE)

        # ACTION ACTIVITY/STATUS COPY - TO BE DELETED
        # if self.is_active == False and active == True:# and self.status == ReturnStatus.FAILURE:
        #     self.set_status(ReturnStatus.RUNNING)
        #     self.is_newly_active = True
        # else:
        #     self.is_newly_active = False
        
        self.is_active = active

    def get_publisher_name(self):
        pub_name = self.label.lower().replace(' ', '_') + '_active'
        illegal_list = ['(',')',':','-']
        for illegal_char in illegal_list:
            pub_name = pub_name.replace(illegal_char,'')
        return pub_name

    def init_publisher(self):
        #self.publisher = rospy.Publisher(self.get_publisher_name(), Bool, queue_size=1)
        self.publisher = rospy.Publisher(self.get_publisher_name(), Active, queue_size=1)
        
    def get_subscriber_name(self):
        #sub_name = self.label.lower().replace(' ', '_') + '_success'
        sub_name = get_condition_topic_name(self.label)
        #illegal_list = ['(',')',':','-','_']
        #for illegal_char in illegal_list:
        #    sub_name = sub_name.replace(illegal_char,'')
        return sub_name
    
    def init_subscriber(self):
        self.subscriber = rospy.Subscriber(self.get_subscriber_name(), Bool, self.callback)

    def callback(self, msg):
        if msg.data:
            self.set_status(ReturnStatus.SUCCESS)
        else:
            self.set_status(ReturnStatus.FAILURE)

    def publish_active_msg(self, active_id): # Added for AURO
        if self.publisher != None:
            #active_msg = Bool()
            #active_msg.data = self.is_active
            active_msg = Active()
            active_msg.active = self.is_active
            active_msg.id = active_id
            self.current_id = active_id
            self.publisher.publish(active_msg)
        else:
            rospy.logerr('')

class Action(ExecutionNode):
    def __init__(self, label):
        ExecutionNode.__init__(self, label)
        self.current_traversal_count = -1
        self.is_newly_active = False
        self.current_id = 0

    def tick(self, active, traversal_count):
        #print(self.label + ' ticked ' + str(self.current_traversal_count) + ' ' + str(self))
        if self.current_traversal_count != traversal_count:
            self.current_traversal_count = traversal_count
        else:
            if self.is_active:
                return
        
        # if we have just become active, set the status to running
        if self.is_active == False and active == True:# and self.status == ReturnStatus.FAILURE:
            self.set_status(ReturnStatus.RUNNING)
            self.is_newly_active = True
        else:
            self.is_newly_active = False
        # if we haven't received a message within the timeout period, set the status to failed
        # if self.get_status_age() > self.max_wait_time:
        #     self.set_status(ReturnStatus.FAILURE)
        
        self.is_active = active

    def get_publisher_name(self):
        pub_name = self.label.lower().replace(' ', '_') + '_active'
        illegal_list = ['(',')',':','-']
        for illegal_char in illegal_list:
            pub_name = pub_name.replace(illegal_char,'')
        return pub_name
    
    def init_publisher(self):
        #self.publisher = rospy.Publisher(self.get_publisher_name(), Bool, queue_size=1)
        self.publisher = rospy.Publisher(self.get_publisher_name(), Active, queue_size=1)
    
    def get_subscriber_name(self):
        sub_name = self.label.lower().replace(' ', '_') + '_status'
        illegal_list = ['(',')',':','-']
        for illegal_char in illegal_list:
            sub_name = sub_name.replace(illegal_char,'')
        return sub_name
    
    def init_subscriber(self):
        self.subscriber = rospy.Subscriber(self.get_subscriber_name(), Status, self.callback)
    
    def callback(self, msg):
        if self.is_active:
            if msg.id != self.current_id:
                print((self.label + ' Incorrect ID', msg.id, self.current_id, self.is_active))
                return
            self.set_status(ReturnStatus(msg.status))
            if msg.status != self.status.status:
                print(('Invalid return status "' + msg.status + '" for node ' + self.label))

    def publish_active_msg(self, active_id):
        if self.publisher != None:
            #active_msg = Bool()
            #active_msg.data = self.is_active
            active_msg = Active()
            active_msg.active = self.is_active
            active_msg.id = active_id
            self.current_id = active_id
            self.publisher.publish(active_msg)
        else:
            rospy.logerr('')

#================================================================================================================
# ----------------------------------------------- Decorator Nodes -----------------------------------------------
#================================================================================================================

class Decorator(Node):
    def __init__(self, label):
        Node.__init__(self, label)
    
    def add_child(self, node):
        if len(self.children) == 0:
            self.children.append(node)
        else:
            raise Exception('A Decorator can have only one child.')


class NotDecorator(Decorator):
    def __init__(self):
        Decorator.__init__(self, '!')

    def add_child(self, node):
        if isinstance(node, Condition):
            super(NotDecorator, self).add_child(node)
        else:
            raise Exception('Not Decorators must have a Condition node child.')
    
    def tick(self, active, traversal_count):
        child = self.children[0]
        child.tick(active, traversal_count)
        if child.status == ReturnStatus.SUCCESS:
            self.status = ReturnStatus.FAILURE
        elif child.status == ReturnStatus.FAILURE:
            self.status = ReturnStatus.SUCCESS
        
        self.is_active = active

def get_decorator(label):
    if label == '!':
        return NotDecorator()

#================================================================================================================
# ------------------------------------------------ Behavior Tree ------------------------------------------------
#================================================================================================================

class BehaviorTree:
    def __init__(self, config_filename):
        self.root = None
        self.nodes = []
        self.node_text = []
        self.active_ids = {}
        self.active_condition_ids = {}
        self.traversal_count = 0

        #self.flag = 1
        self.num_child = ''

        # Parse the input file
        # Leave tree empty if no filename given
        if config_filename != '':
            self.parse_config(config_filename)

        Node.max_wait_time = rospy.get_param('~timeout', 1.0)

        self.active_actions_pub = rospy.Publisher('active_actions', String, queue_size=1)
        self.active_conditions_pub = rospy.Publisher('active_conditions', String, queue_size=1) # Added for AURO results; BT compactness eval
        self.defineActionNodes()
        self.defineConditionNodes()

    # def at_node_activity_equilibrium(self, bt_at_previous_tick):
    #     '''
    #      - Returns Bool denoting whether or not there has been a change in any node's status or activity
    #      between current and previous tick
    #     '''
    #     print('================== CHECKING IF AT EQUIL ===================')
    #     at_equil = True
    #     for n1 in bt_at_previous_tick.nodes:
    #         for n2 in self.nodes:
    #             if n1.status != n2.status:
    #                 print(n1.status.__str__() + n2.status.__str__() + "not same")
    #                 return False
    #             else:
    #                 print(n1.status.__str__() + n2.status.__str__() + "same")

    #     print('================== AT EQUIL ===================')
    #     return False

    def generate_nodes_list(self):
        # This function fills in self.nodes
        # Note that if parse_config is used, this is not necessary (done internally)
        # But if the tree is generated by other means, it might be easier to use this function
        #
        # self.root should already be defined!
        #
        # Depth-first traversal to produce pre-ordering

        # Empty the list, since it's going to be completed regenerated here
        self.nodes = []

        # Setup a stack data structure (similar to nodes_worklist)
        nodes_stack = []
        nodes_stack.append(self.root) #push

        # Do the traversal, using the stack to help
        while len(nodes_stack) != 0:
            current_node = nodes_stack.pop()
            self.nodes.append(current_node)
            for child_idx in reversed(list(range(len(current_node.children)))):
                nodes_stack.append(current_node.children[child_idx]) #push


    def parse_config(self, config_filename):
        rospack = rospkg.RosPack()
        
        fin = open(config_filename)
        nodes_worklist = []
        prev_tabs = 0
        
        lines = fin.read().split('\n')
        
        include = 'include'
        did_include = True

        while did_include:
            did_include = False
            for i in range(len(lines)):
                line = lines[i]
                tabs = len(line) - len(line.lstrip('\t'))
                label = line.strip()
            
                if len(label) >= len(include) and label[:len(include)] == include:
                    did_include = True
                    filename = label.split(include)[-1].strip()
                    while True:
                        try:
                            start = filename.index('$(')
                            end = filename.index(')')
                        except:
                            break
                        find_statement = filename[start:end]
                        package_name = find_statement.split('find')[-1].strip()
                        filename = filename[0:start] + rospack.get_path(package_name) + filename[end+1:]
                        subtree_lines = open(filename).read().split('\n')
                        add_tabs = tabs*'\t'
                        subtree_lines = [add_tabs+x for x in subtree_lines]
                        del lines[i]
                        lines = lines[:i] + subtree_lines + lines[i:]
        
        for line in lines:
            if len(line) == 0:
                continue
            
            tabs = len(line) - len(line.lstrip('\t'))
            label = line.strip()
            node = None

            if label[:2] == '->':
                node = Sequence()
            elif label[0] == '?':
                node = Fallback()   
            elif label[0] == '|':
                arguments = [x.strip() for x in label[3:].split(',')]
                node = Parallel(int(arguments[0]))
                self.num_child = int(arguments[0])
            elif label[0] == '(':
                node_label = label.replace('(', '').replace(')', '')
                node = Condition(node_label)
                self.node_text = node_label
                self.active_condition_ids[node_label] = 0 # Added for AURO
            elif label[0] == '[':
                node_label = label.replace('[', '').replace(']', '')
                node = Action(node_label)
                self.node_text = node_label
                self.active_ids[node_label] = 0
            elif label[0] == '<':
                node_label = label.replace('<', '').replace('>', '')
                node = get_decorator(node_label)
                self.node_text = node_label

            self.nodes.append(node)
            
            if self.root == None:
                self.root = node
                nodes_worklist.append(node)
                continue
            
            if tabs == prev_tabs+1:
                parent = nodes_worklist[-1]
                parent.add_child(node)
            else:
                for i in range(prev_tabs - tabs + 1):
                    nodes_worklist.pop()
                parent = nodes_worklist[-1]
                parent.add_child(node)
                
            nodes_worklist.append(node)
            prev_tabs = tabs
            # self.write_config(nodes_worklist, prev_tabs, self.node_text)


    # def write_config(self, nodes_worklist, prev_tabs, node_text):
        
    #     rootnode = self.which_node(nodes_worklist[0].__class__.__name__,node_text)
    #     #change the file name each time you want to generate a new config
    #     f = open("test.tree","a")
    #     if self.flag == 1:
    #         self.flag = 0
    #         f.write(rootnode + '\n')
    #     for object in nodes_worklist[prev_tabs:]:
    #         node_type = object.__class__.__name__
    #         node = self.which_node(node_type, node_text)
    #         indent = ''
    #         for i in range(prev_tabs):
    #             indent = indent + '\t'
    #         f.write(indent + node + '\n')
    #     f.close()

    
    def write_config(self, output_filename):
        # Method: Do a depth-first traversal over the tree
        # to produce a pre-ordering
        # Implementation: iteration not recursion
        #
        # pseudocode adapted from https://en.wikipedia.org/wiki/Depth-first_search :
        '''
        open file
        let S be a stack
        S.push(root)
        while S is not empty
            current_node = S.pop()
            file.write( current_node.tabs )
            file.write( current_node.label )
            file.write( '\n' )
            for child in current_node.children in !reverse! order
                S.push(child)
        close file
        '''

        # Open the output file
        f = open(output_filename,"w")
        print(output_filename)
        import os
        #print(os.path.abspath(os.getcwd()))
        # Setup a stack data structure (similar to nodes_worklist)
        # Do this for both keeping track of nodes and for number of tabs
        nodes_stack = []
        tabs_stack = []
        nodes_stack.append(self.root) #push
        tabs_stack.append(0) #push

        # Do the traversal, using the stack to help
        while len(nodes_stack) != 0:

            # Pop a node off the stack
            current_node = nodes_stack.pop() #pop
            tabs = tabs_stack.pop() #pop

            # Write a line to the file
            indent = ''
            for i in range(tabs):
                indent = indent + '\t'
            
            label = self.get_node_text(current_node) 

            f.write( indent + label + '\n' )

            # Add all children to the stack
            for child_idx in reversed(list(range(len(current_node.children)))):
                nodes_stack.append(current_node.children[child_idx]) #push
                tabs_stack.append(tabs + 1) #push

        # Close the file
        f.close()

    # def which_node(self, node_type, node_text):

    #     node = node_type
    #     if node == 'Sequence':
    #         node = '->'
    #     elif node == 'Fallback':
    #         node = '?'
    #     elif node == 'Parallel': 
    #         node = '|| ' + str(self.num_child)
    #     elif node == 'Condition':
    #         node = '(' + node_text + ')'
    #     elif node == 'Action':
    #         node = '[' + node_text + ']'
    #     elif node == 'Decorator':
    #         node = '<' + node_text + '>'
    #     return node

    def get_node_text(self, node):

        node_type = node.__class__.__name__

        if node_type == 'Sequence':
            node_label = '->'
        elif node_type == 'Fallback':
            node_label = '?'
        elif node_type == 'Parallel': 
            node_label = '|| ' + str( node.child_success_threshold )
        elif node_type == 'Condition':
            node_text = node.label
            node_label = '(' + node_text + ')'
        elif node_type == 'Action':
            node_text = node.label
            node_label = '[' + node_text + ']'
        elif node_type == 'Decorator' or node_type == 'NotDecorator':
            node_text = node.label
            node_label = '<' + node_text + '>'
        else:
            node_label = ''
        return node_label

    def tick(self):
        if self.root != None:
            #print('begin tick')
            self.root.tick(True, self.traversal_count)
            #print()
            self.traversal_count += 1
            active_actions = String()
            active_actions.data = ''
            active_conditions = String() # AURO
            active_conditions.data = '' # AURO

            # Make sure that if there are more than one of the same action, if any are active, then active should be published
            unique_action_nodes = {}
            for node in self.nodes:
                #print("node: ", node, node.label)
                if isinstance(node, Action):
                    #print("unique_action_nodes ", unique_action_nodes.keys(), "\n")
                    #print("action node: ", node.label, ", ", node.is_active, ", ", node.is_newly_active, "\n")
                    #print("active ids ", self.active_ids)
                    if node.label not in list(unique_action_nodes.keys()) or node.is_active:
                        #if node.is_active:
                        unique_action_nodes[node.label] = node
                        if node.label not in self.active_ids.keys():
                                #print("Adding node label to active ids ", node.label)
                                self.active_ids[node.label] = 0
                        if node.is_newly_active:
                            self.active_ids[node.label] += 1
                    #else:
                    #    unique_action_nodes[node.label] = node
            
            #print("active ids 2 ", self.active_ids)         
            for label, node in unique_action_nodes.items():
                #active_msg = Bool()
                #active_msg.data = node.is_active
                #node.publisher.publish(active_msg)
                node.publish_active_msg(self.active_ids[node.label])

                if node.is_active:
                    active_actions.data += label + ', '



            active_actions.data = active_actions.data[:-2] # strip the final comma and space
            self.active_actions_pub.publish(active_actions)

            # For AURO results, track active conditions as well
            unique_condition_nodes = {}
            for node in self.nodes:
                if isinstance(node, Condition):
                    if node.label not in list(unique_condition_nodes.keys()) or node.is_active:
                        #if node.is_active:
                        unique_condition_nodes[node.label] = node
                        if node.label not in self.active_condition_ids.keys():
                                self.active_condition_ids[node.label] = 0
                        if node.is_newly_active:
                            self.active_condition_ids[node.label] += 1
                    #else:
                    #    unique_action_nodes[node.label] = node
                        
            for label, node in unique_condition_nodes.items():
                #active_msg = Bool()
                #active_msg.data = node.is_active
                #node.publisher.publish(active_msg)
                node.publish_active_msg(self.active_condition_ids[node.label])

                if node.is_active:
                    active_conditions.data += label + ', '

            active_conditions.data = active_conditions.data[:-2] # strip the final comma and space
            self.active_conditions_pub.publish(active_conditions)

    def at_node_activity_equilibrium(self, bt_at_previous_tick):
        '''
         - Returns Bool denoting whether or not there has been a change in any node's status or activity
         between current and previous tick
        '''
        print('================== CHECKING IF AT EQUIL ===================')
        at_equil = True
        for n1 in bt_at_previous_tick.nodes:
            for n2 in self.bt.nodes:
                if n1.status != n2.status:
                    print(n1.status.__str__() + n2.status.__str__() + "not same")
                    return False
                else:
                    print(n1.status.__str__() + n2.status.__str__() + "same")

        print('================== AT EQUIL ===================')
        return False

    def print_BT(self):
        for node in self.nodes:
            node.print_node()
    
    def evaluate_bt_compactness(self):

        print('EVALUATING BT COMPACTNESS - Total node count (does not consider node activity)')

        # Determine complexity of the sentence "reading" the bt results in
        # Count total number of nodes
        # Count conditions and actions

        # Populate bt.nodes
        self.generate_nodes_list()

        print(('len(self.nodes) where self is BT',len(self.nodes)))

        total_num_nodes = 0
        num_action_nodes = 0
        num_condition_nodes = 0
        for node in self.nodes:
            total_num_nodes+=1
            print((node.label))
            if isinstance(node,Action):
                print((node.label, 'is Action'))
                num_action_nodes+=1
            elif isinstance(node,Condition):
                print((node.label,'is Condition'))
                num_condition_nodes+=1

        print(("Total number of nodes: %d" %total_num_nodes))
        print(("Total number of action nodes: %d" %num_action_nodes))
        print(("Total number of condition nodes: %d" %num_condition_nodes))

        print('DONE WITH COMPACTNESS EVAL - Total node count (does not consider node activity)')

    def defineActionNodes(self):

        #print("in defineActionNodes")

        self.action_nodes = dict()

        if not self.nodes:
            print("defineActionNodes: bt empty!")
        else:
            for n in self.nodes:
                if n.__class__ == Action:
                    if n.label not in self.action_nodes:

                        # Create empty list
                        self.action_nodes[n.label] = []

                    # Add it to the dictionary
                    self.action_nodes[n.label].append(n)

        # print("action node values: \n")
        # print(self.action_nodes.keys())
        # print(self.action_nodes.values())
        # input('defineActionNodes end')


    def defineConditionNodes(self):

        self.condition_nodes = dict()

        if not self.nodes:
            print("defineConditionNodes: bt empty!")
        else:
            for n in self.nodes:
                if n.__class__ == Condition:
                    #print(n.label,"n.label")
                    if n.label not in self.condition_nodes:

                        # Create empty list
                        self.condition_nodes[n.label] = []

                    # Add it to the dictionary
                    self.condition_nodes[n.label].append(n)


    def getActiveActions(self):
        # returns list of all actions that are active currently as a list of strings (i.e. names)
        # Pulled from bt_interface in MCDAGS work

        active_actions = []

        print("in getActiveActions")
        print("action node values: \n")
        print(self.action_nodes.keys())
        print(self.action_nodes.values())
        
        for n in list(self.action_nodes.values()):
            is_active = False
            for node in n:
                if node.is_active:
                    is_active = True
            if is_active:
                active_actions.append(n[0].label)

        return active_actions

    def getActiveConditions(self):
        # returns list of all actions that are active currently as a list of strings (i.e. names)
        # Pulled from bt_interface in MCDAGS work

        active_conditions = []
        
        for n in list(self.condition_nodes.values()):
            is_active = False
            for node in n:
                if node.is_active:
                    is_active = True
            if is_active:
                active_conditions.append(n[0].label)

        return active_conditions

    def changeConditionStatus(self): 

        # likely need to fix label generation (remove (x: x) stuff)

        # I think this method of setting an action to false manually will not work
        # rqt doesnt work currently, but I dont think it would help if it did
        # because setting an action to active in the GUI doesnt make the correct conditions change their status
        # it is all manual

        # So it seems we would have to loop through or do something differently
        # I would assume the answer lies in the tick function somewhere since that's traversal

        # for a in self.action_nodes:
        #     print(a)

        # print(self.active_ids)
        # for n in self.nodes:
        #     if n.label == action_label:
        #         n.is_active = True
        #     print(n.label, n.is_active)


        # Condtion stuff below

        print(self.condition_nodes)
        for key in self.condition_nodes:
            print(key, " - ", self.condition_nodes[key], "\n")
            condition_node = self.condition_nodes[key]
        #first_condition = self.condition_nodes[0]
        #print('first condition', first_condition.label , first_condition)


if __name__ == '__main__':
    pass
    #BehaviorTree(sys.argv[1])
    
    #Change the path to the config file you want to parse and then convert back
    #BehaviorTree("../../../behavior_executive/config/search_rescue3.tree")
    
    #For the test config file I included if in the same directory
    #BehaviorTree("config_filename.tree")

