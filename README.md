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

* Ubuntu 20.04 Focal Fossa and ROS Noetic

### Installation

* Create a catkin workspace like [this](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
* Navigate to the src directory in your workspace and clone this repo

### Executing the program 

* What is the domain path? Specify a domain or pick one like the following: ```~/your_workspace/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine/final_domain.ppddl```

* What is the problem path? Specify or locate the associated problem. The following is associated with the above domain: ```~/your_workspace/src/policy_to_behavior_tree/pypddl_parser/src/pypddl_parser/pddl/AURO/marine/problem.ppddl```

* To run the **full method**: ```roslaunch mdp_to_bt main.launch config:=final domain:=domain-path problem:=problem-path``` where domain-path and problem-path are as defined above.

* The generated behavior tree before and after simplification will be saved as ```raw_policy_bt.tree``` and ```final_synth_bt.tree``` respectively. You will need to update their paths in main.py here: ```raw_policy_bt.write_config('your-path')``` and```simplified_policy_bt.write_config('your-path')```.

* Next, similarly update the path variable in main.py where intermediate data will be stored. Currently, the path points to the mdp_to_bt/src/mdp_to_bt/policy_eval_output/ directory.

* To run the **method without policy simplification**: ```roslaunch mdp_to_bt main.launch config:=no_simp domain:=domain-path problem:=problem-path```

## Executing the program (deprecated)

* Navigate to the following location: 
```~/your_workspace/src/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt```
* Locate your domain and problem file paths, which we will refer to as domain-path and problem-path.
* Open another terminal window and run ```roscore```.
* Run the main method:
```python3 main.py domain-path problem-path```
* The solved for policy and simplified policy will both be converted to and saved as behavior trees. You can access these .tree files in ```~/your_workspace/src/policy_to_behavior_tree/behavior_tree/config```. They are raw_policy_bt.tree and simplified_bt.tree.
* To visualize these trees, ```rosrun behavior_tree show_tree.py file_name```, where ```file_name``` is the .tree file you want to show. Once this is running, run ```rqt``` in a new terminal window.
* To evaluate a probabilistic domain against the deterministic version (given ```roscore``` is running) run:
```python3 policy_comparison.py```

## Help

* Remember to ```source devel/setup.bash``` in the root of your workspace with every new terminal window if you haven't automated this.

## Acknowledgments

* [Related work: PDDL to BT](https://arxiv.org/abs/2101.01964)
* [Basic PPDDL parser](https://github.com/thiagopbueno/pypddl-parser)
* [Python Toolbox MDP Solver](https://pymdptoolbox.readthedocs.io/en/latest/api/mdp.html)
* [Boolean Algebra Support](https://docs.sympy.org/latest/modules/logic.html)






