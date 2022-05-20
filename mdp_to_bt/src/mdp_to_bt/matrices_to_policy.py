import mdptoolbox


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