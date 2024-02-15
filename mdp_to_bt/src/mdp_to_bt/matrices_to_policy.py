import mdptoolbox
import numpy as np


# class SolveForPolicy:
#     def __init__(self, solver, P, R, actions_with_params):

#         self.run(solver, P, R, actions_with_params)

#     def run(self, solver, P, R, actions_with_params):

#         # Solve for policy
#         self.policy = self.solve(solver, P, R)

#         # Remove any action terms for actions that do not appear in the new policy
#         self.filtered_actions, self.filtered_action_nums  = self.filterActionsWithParams(self.policy, actions_with_params)

#     def solve(self, solver,P,R):

#         # Get transition probabality and reward matrices from PPDDL

#         if solver == 'v':
#             val_it = mdptoolbox.mdp.ValueIteration(P, R, 0.96)
#             val_it.run()
#             policy = val_it.policy
#             print('\nPolicy: ', val_it.policy)
#         elif solver == 'q':
#             q_learn = mdptoolbox.mdp.QLearning(P, R, 0.96)
#             q_learn.run()
#             policy = q_learn.policy
#             print('\nPolicy: ', q_learn.policy)
#         else:
#             print("That is not a valid solver...")

#         return policy

#     def filterActionsWithParams(self, policy, actions_with_params):

#         print(actions_with_params, len(actions_with_params))

#         unique_action_nums_in_policy = list(np.unique(policy))

#         new_actions_with_params = [actions_with_params[i] for i in unique_action_nums_in_policy]
#         updated_action_num_order = [i for i in unique_action_nums_in_policy]

#         print(new_actions_with_params, len(new_actions_with_params))

#         return new_actions_with_params, updated_action_num_order


def solve(solver,P,R):

    # Get transition probabality and reward matrices from PPDDL

    if solver == 'v':
        val_it = mdptoolbox.mdp.ValueIteration(P, R, 0.96)
        val_it.run()
        policy = val_it.policy
        print('\nPolicy: ', val_it.policy)
    elif solver == 'q':
        q_learn = mdptoolbox.mdp.QLearning(P, R, 0.96)
        q_learn.run()
        policy = q_learn.policy
        print('\nPolicy: ', q_learn.policy)
    else:
        print("That is not a valid solver...")

    return policy