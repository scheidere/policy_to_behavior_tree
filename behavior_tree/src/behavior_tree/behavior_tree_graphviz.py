#!/usr/bin/python
#import graphviz
import cv2
import sys
import behavior_tree.behavior_tree as bt
import os

import sys
#reload(sys)  
#sys.setdefaultencoding('utf8')


def get_graphviz(tree):
    nodes_worklist = [tree.root]

    gv = 'digraph G {\n'

    fontsize = "40"
    counts = {bt.Condition: 0, bt.Action: 0, bt.Fallback: 0, bt.Sequence: 0, bt.Parallel: 0, bt.Decorator: 0}
    node_names = {}
    
    while len(nodes_worklist) > 0:
        node = nodes_worklist.pop()
        
        style = ''
        if node.is_active:
            style += 'penwidth=2 color=black style=filled '
            if node.status == bt.ReturnStatus.SUCCESS:
                style += 'fillcolor=yellowgreen'#'fillcolor=green'
            elif node.status == bt.ReturnStatus.RUNNING:
                style += 'fillcolor=lightblue'
            elif node.status == bt.ReturnStatus.FAILURE:
                style += 'fillcolor=indianred2' #'fillcolor=#FF0000'
		#print(style)
        else:
            style += 'penwidth=2 '
            if node.status == bt.ReturnStatus.SUCCESS:
                style += 'color=yellowgreen'#'color=green'
            elif node.status == bt.ReturnStatus.RUNNING:
                style += 'color=lightblue'
            elif node.status == bt.ReturnStatus.FAILURE:
                style += 'color=indianred2'#'color=#FF0000'
        
        if isinstance(node, bt.Condition):
            name = 'condition_%d' % (counts[bt.Condition])
            gv += '\t%s [label="%s" %s fontsize="%s"]\n' % (name, node.label, style, fontsize)
            counts[bt.Condition] += 1
        elif isinstance(node, bt.Action):
            name = 'action_%d' % (counts[bt.Action])
            gv += '\t%s [label="%s" shape=square %s fontsize="%s"]\n' % (name, node.label, style, fontsize)
            counts[bt.Action] += 1
        elif isinstance(node, bt.Fallback):
            name = 'fallback_%d' % (counts[bt.Fallback])
            gv += '\t%s [label="%s" shape=square %s fontsize="%s"]\n' % (name, node.label, style, fontsize)
            counts[bt.Fallback] += 1
        elif isinstance(node, bt.Sequence):
            name = 'sequence_%d' % (counts[bt.Sequence])
            gv += '\t%s [label="%s" shape=square %s fontsize="%s"]\n' % (name, node.label, style, fontsize)
            counts[bt.Sequence] += 1
        elif isinstance(node, bt.Parallel):
            name = 'parallel_%d' % (counts[bt.Parallel])
            gv += '\t%s [label="%s" shape=parallelogram %s fontsize="%s"]\n' % (name, node.label, style, fontsize)
            counts[bt.Parallel] += 1
        elif isinstance(node, bt.Decorator):
            name = 'decorator_%d' % (counts[bt.Decorator])
            gv += '\t%s [label="%s" shape=diamond %s fontsize="%s"]\n' % (name, node.label, style, fontsize)
            counts[bt.Decorator] += 1
            
        node_names[node] = name
	#print(gv)
        
        for child in node.children:
            nodes_worklist.append(child)
    
    gv += '\n\tordering=out;\n\n'
    
    nodes_worklist.append(tree.root)
    while len(nodes_worklist) > 0:
        node = nodes_worklist.pop()
        
        for child in node.children:
            gv += '\t%s -> %s;\n' % (node_names[node], node_names[child])
            nodes_worklist.append(child)
        
    
    gv += '}\n'
    
    return gv

'''
def get_graphviz_image(gv_source, filename='temp'):
    s = graphviz.Source(gv_source, filename=filename, format='png', engine='dot')
    s.render()
    
    img = cv2.imread(filename + '.png')
    return img
'''

if __name__ == '__main__':
    tree = bt.BehaviorTree(sys.argv[1])
    source = get_graphviz(tree)
    
'''
    if (len(sys.argv)) > 2:
        get_graphviz_image(source, os.path.basename(sys.argv[2]).split('.')[0])
'''
