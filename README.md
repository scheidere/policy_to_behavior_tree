# Behavior Tree Synthesis for MDPs through Probabilistic PDDL 

This repo provides a method of translating an MDP represented by a Probabilistic PDDL (PPDDL) specification into behavior tree (BT) form, including a policy simplification step based on Boolean albebra. This method is applicable to domains with nondeterministic actions, unlike previous methods that only relate PDDL to BTs.

In the future, we plan to extend this work to accommodate POMDPs and multi-robot sytems.

## Description

There are five main stages to this method:

* PPDDL specification, i.e. a domain.ppddl and a problem.ppddl file that relate to each other.

* Conversion of the PPDDL specification that represents an MDP to equivalent, but solvable matrix form: the transition probability matrix, and the reward matrix.

* A Q-learning algorithm that solves the MDP for a policy.

* A policy simplification method using Boolean algebra rules that removes redundancies in the policy, preparing it for conversion to behavior tree form.

* A method of conversion to a behavior tree to make the policy readable.

## Getting Started

### Dependencies

* Ubuntu 18.04 Bionic Beaver 
* ROS Melodic
* See import almost inevitable errors for library dependencies (lol)

### Installation

* Create a catkin workspace like [this](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
* Navigate to the src directory in your workspace and clone this repo

### Executing the program

* Navigate to the following location: 
'''~/your_workspace/src/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt'''
* Locate your domain and problem file paths, which we will refer to as domain-path and problem-path
* Open another terminal window and run '''roscore'''
* Run the main method:
'''python3 ppddl_to_matrices.py domain-path problem-path'''

## Help

* Remember to '''source devel/setup.bash''' in the root of your workspace with every new terminal window

## Acknowledgments

* [Related work: PDDL to BT](https://arxiv.org/abs/2101.01964)
* [Basic PPDDL parser](https://github.com/thiagopbueno/pypddl-parser)
* [Python Toolbox MDP Solver](https://pymdptoolbox.readthedocs.io/en/latest/api/mdp.html)
* [Boolean Algebra](https://docs.sympy.org/latest/modules/logic.html)






