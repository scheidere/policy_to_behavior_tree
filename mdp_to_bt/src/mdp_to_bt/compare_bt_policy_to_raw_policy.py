#!/usr/bin/env python3


def compare():

    f3 = open("/home/scheidee/Desktop/AURO_results/bt_pol_to_raw_pol_comparison_results.txt", "w+")

    # f1 = open("/home/scheidee/Desktop/AURO_results/raw_policy_actions.txt","r")
    # f2 = open("/home/scheidee/Desktop/AURO_results/policy_actions_from_bt.txt", "r")

    with open(r"/home/scheidee/Desktop/AURO_results/raw_policy_actions_deterministic.txt","r") as f1:
        f1_lines = f1.readlines()
        f1_num = len(f1_lines)

    with open(r"/home/scheidee/Desktop/AURO_results/policy_actions_from_bt_deterministic.txt","r") as f2:
        f2_lines = f2.readlines()
        f2_num = len(f2_lines)

    if f1_num != f2_num:
        input("THE NUMBER OF STATES DO NOT MATCH - ERROR")
    else:
        print('Same number of states: %s\n' %f1_num)

    match_count = 0
    for i in range(f1_num):
        l1 = f1_lines[i]
        l2 = f2_lines[i]
        if l1 in l2:
            #print("match %s, %s\n" %(l1,l2))
            match_count += 1
        else:
            print("No match at state %s\n" %(i+1))

    print("Match count: %s\n" %match_count)

    # count lines in f1 (should be same) and f2
    # check they are same, record num

    # for i in range(num):
    #   if f1 line in f2 line:

    # for line in f1













if __name__ == "__main__":

    compare()