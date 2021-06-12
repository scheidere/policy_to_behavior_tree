#!/usr/bin/env python3

import mdptoolbox

import mdptoolbox.example


class MDP:
    def __init__(self, states, actions, method='vacuum'):

	# Lists to track order of states and actions in cols and rows of matrices
        self.states = states # list of tuples, S
        self.actions = actions # list of strings, A

	# Define transition probabilities and associated rewards; (A,S,S) shape
        self.transition_probs, self.rewards = self.get_probs_and_rewards(method='vacuum')


    def get_transition_probs(self, method='vacuum'):
        # Methods: small, vacuum or infant

        if method == 'small':
            P,R = mdptoolbox.example.small()
            return P,R
        elif method == 'vacuum':
            P = np.array()
            R = ...
        elif method == 'infant':
            print('Not implemented yet')
            return None
      
        
        print('Method does not exist')
	return None    









def main(method):

    vacuum_states = [('vac left', 'dirt left', 'dirt right'), ('vac right', 'dirt left', 'dirt right')
                     ('vac left', 'no dirt left', 'dirt right'), ('vac right', 'dirt left', 'dirt right'),
                     ('vac left', 'dirt left', 'dirt right'),('vac right', 'dirt left', 'dirt right'),
                     ('vac left', 'dirt left', 'dirt right')('vac right', 'dirt left', 'dirt right')]


if __name__ == "__main__":

    method = input("Enter small, vacuum, or infant")
    main(method)
