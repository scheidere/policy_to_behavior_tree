
import random
import numpy as np

# Take as input an mdp problem and an mdp policy
# Evaluate this policy using random sampling

class MDP_Problem:
	def __init__(self, P, R, states, actions_with_params):
		self.P = P
		self.R = R
		self.states = states
		self.actions_with_params = actions_with_params


def evaluate_mdp_policy(mdp_problem, mdp_policy):

	# Parameters
	num_trials = 100
	num_steps_per_trial = 100

	# Define the evaluation function
	def evaluate_once(mdp_problem, mdp_policy):
		
		# print("================ evaluate_once ==============")
		# Accumulate reward over the "mission"
		reward = 0

		# Pick a start state
		state_idx = random.randint(0,len(mdp_problem.states)-1)
		# print("state_idx", state_idx)

		# Do a number of steps
		for j in range(num_steps_per_trial):
			# print("j", j)

			# Pick an action according to the policy
			# print("mdp_policy", mdp_policy)
			action_idx = mdp_policy[state_idx]
			# print("action_idx", action_idx)

			# Get the next state using the transition model
			probabilities = mdp_problem.P[action_idx, state_idx]
			next_states_idx = range(0,len(probabilities))
			next_state_idx = np.random.choice(next_states_idx, p=probabilities)
			# print("next_state_idx", next_state_idx)

			# Get a reward according to the reward matrix
			reward += mdp_problem.R[action_idx, state_idx, next_state_idx]
			# print("accumulated reward", reward)

			# Move to the next state
			state_idx = next_state_idx

		return reward

	# Evaluate multiple times to find average reward
	reward_sum = 0
	for i in range(num_trials):
		reward = evaluate_once(mdp_problem, mdp_policy)

		reward_sum += reward

	average_reward = reward_sum / num_trials

	return average_reward