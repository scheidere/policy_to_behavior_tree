# Behavior Tree Synthesis for MDPs through Probabilistic PDDL 

This repository provides a method of behavior tree synthesis from an MDP specified in Probabilistic PDDL (PPDDL) by a user. This synthesis occurs through the translation of the PPDDL MDP specification into a solveable form, the solving of that MDP for the optimal policy, a policy simplification step using Boolean albebra, and a unique conversion method from this simplified policy to a compact, but equivalent, behavior tree. This method is applicable to domains with nondeterministic actions, unlike previous methods that only relate PDDL to BTs.

In the future, we plan to extend this work to accommodate POMDPs and multi-robot sytems.

## Description

There are five main stages to this method:

* PPDDL specification, i.e., a domain.ppddl and a problem.ppddl file that relate to each other and collectively represent an MDP.

* Conversion of the PPDDL MDP specification to the equivalent, but solvable matrix form: the transition probability matrix, and the reward matrix.

* A value iteration algorithm that solves the MDP for the optimal policy.

* A policy simplification method using Boolean algebra rules that removes redundancies in the policy, preparing it for conversion to behavior tree form.

* A method of conversion from the simplified optimal policy to a functionally equivalent and compact behavior tree, with the objective of increasing interpretability.

## Getting Started

### Dependencies

* Ubuntu 20.04 Focal Fossa and ROS Noetic

### Installation

* Create a catkin workspace like [this](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
* Navigate to the src directory in your workspace and clone this repository

### Executing the program 

* What is the domain path? Specify a domain or pick one like the following: ```~/your_workspace/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine/final_domain.ppddl```

* What is the problem path? Specify or locate the associated problem. The following is associated with the above domain: ```~/your_workspace/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine/problem.ppddl```

* To run the **full method**: ```roslaunch main.launch config:=final domain:=domain-path problem:=problem-path``` where domain-path and problem-path are as defined above.

* The generated behavior tree before and after simplification will be saved as ```raw_policy_bt.tree``` and ```final_synth_bt.tree``` respectively. You will need to update their paths in main.py here: ```raw_policy_bt.write_config('your-path')``` and```simplified_policy_bt.write_config('your-path')```.

* Next, similarly update the path variable in main.py where intermediate data will be stored. Currently, the path points to the mdp_to_bt/src/mdp_to_bt/policy_eval_output/ directory.

* To run the **method without policy simplification**: ```roslaunch main.launch config:=no_simp domain:=domain-path problem:=problem-path```

## Acknowledgments

* [Related work: PDDL to BT](https://arxiv.org/abs/2101.01964)
* [Basic PPDDL parser](https://github.com/thiagopbueno/pypddl-parser)
* [Python Toolbox MDP Solver](https://pymdptoolbox.readthedocs.io/en/latest/api/mdp.html)
* [Boolean Algebra Support](https://docs.sympy.org/latest/modules/logic.html)






