#!/usr/bin/env python3

print('Example that does not work')
states = []
sing_state = [['bla',1],['bla2',1]]
for tup in [(0,1),(1,0)]:
    print(tup)
    full_state = sing_state.copy()
    
    print('1', states)

    for i in range(len(sing_state)):
        full_state[i][-1] = tup[i]

    states.append(full_state.copy())
    print('2', states)

# This works!
print('Example that works')
states = []
test_state = [1,1]
for i in range(2):

    full_state = test_state.copy()

    print('1', states)

    full_state[i] = 3

    states.append(full_state.copy())
    print('2', states)