#!/usr/bin/env python3

import mdptoolbox
import numpy as np
import mdptoolbox.example


class MDP:
    def __init__(self, method='vacuum'):

	# Lists to track order of states and actions in cols and rows of matrices
        #self.states = states # list of tuples, S
        #self.actions = actions # list of strings, A

        self.method = method

	# Define transition probabilities and associated rewards; (A,S,S) shape
        self.transition_probs, self.rewards = self.get_probs_and_rewards()


    def get_probs_and_rewards(self):
        # Methods: small, vacuum or infant

        if method == 'small':
            P,R = mdptoolbox.example.small()
            return P, R

        elif self.method == 'vacuum':
            P = np.array([[[0., 0., 1., 0., 0., 0., 0. , 0.],
            [0., 0., 0., 1., 0., 0., 0. , 0.],
            [0., 0., 1., 0., 0., 0., 0. , 0.],
            [0., 0., 0., 1., 0., 0., 0. , 0.],
            [0., 0., 0., 0., 0., 0., 1. , 0.],
            [0., 0., 0., 0., 0., 0., 0. , 1.],
            [0., 0., 0., 0., 0., 0., 1. , 0.],
            [0., 0., 0., 0., 0., 0., 0. , 1.]],

            [[0., 1., 0., 0., 0., 0., 0. , 0.],
            [1., 0., 0., 0., 0., 0., 0. , 0.],
            [0., 0., 0., 0., 0., 1., 0. , 0.],
            [0., 0., 0., 0., 1., 0., 0. , 0.],
            [0., 0., 0., 1., 0., 0., 0. , 0.],
            [0., 0., 1., 0., 0., 0., 0. , 0.],
            [0., 0., 0., 0., 0., 0., 0. , 1.],
            [0., 0., 0., 0., 0., 0., 1. , 0.]]])

            R = np.array([[[0., 0., 0., 0., 0., 0., 0. , 0.],
            [0., 0., 0., 0., 0., 0., 0. , 0.],
            [0., 0., 0., 0., 0., 0., 0. , 0.],
            [0., 0., 0., 0., 0., 0., 0. , 0.],
            [0., 0., 0., 0., 0., 0., 1. , 0.],
            [0., 0., 0., 0., 0., 0., 0. , 1.],
            [0., 0., 0., 0., 0., 0., 1. , 0.],
            [0., 0., 0., 0., 0., 0., 0. , 1.]],

            [[0., 0., 0., 0., 0., 0., 0. , 0.],
            [0., 0., 0., 0., 0., 0., 0. , 0.],
            [0., 0., 0., 0., 0., 0., 0. , 0.],
            [0., 0., 0., 0., 0., 0., 0. , 0.],
            [0., 0., 0., 0., 0., 0., 0. , 0.],
            [0., 0., 0., 0., 0., 0., 0. , 0.],
            [0., 0., 0., 0., 0., 0., 0. , 1.],
            [0., 0., 0., 0., 0., 0., 1. , 0.]]])

            return P, R

        elif self.method == 'infant':
            print('Not implemented yet')
            return None
      
        
        print('Method does not exist')
        return None    









def main(method, solver):

    mdp = MDP(method)
    P = mdp.transition_probs
    R = mdp.rewards
    print("Transition probs: ", P)
    print("Rewards: ", R)

    if solver == 'v':
        val_it = mdptoolbox.mdp.ValueIteration(P, R, 0.96)
        val_it.run()
        print('Policy: ', val_it.policy)
    elif solver == 'q':
        q_learn = mdptoolbox.mdp.QLearning(P, R, 0.96)
        q_learn.run()
        print('Policy: ', q_learn.policy)
    else:
        print("That is not a valid policy...")


    
if __name__ == "__main__":

    method = input("Enter small, vacuum, or infant: ")
    solver = input("Enter value iteration (v) or Q-learning (q): ")
    main(method, solver)
