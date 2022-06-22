


from main import *
import os
import sys
import pickle




def plot(domain1,domain2):

    avg_reward_prob, avg_reward_det = compare_policies()



def compare_policies(probabilistic_domain_path, do_prints = False):

    # Path to policy and mdp_problem pickle files
    output_path = "/home/scheidee/new_bt_generation_ws/src/bt_generation/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/"
    
    # Path to PPDDL domain and problem files
    prob_domain_path = probabilistic_domain_path
    det_domain_path = partial_path + "/marine/domain_deterministic.ppddl"
    problem_path = partial_path + "/marine/problems/problem1.ppddl"

    # Run main with deterministic domain; Save policy
    os.system("python3 main.py " + det_domain_path + " " + problem_path)

    # Extract deterministic policy
    file = open(output_path+'policy.p','rb')
    det_policy = pickle.load(file)
    file.close()
    #print('d', det_policy)

    # Run main with probabilistic domain; Save policy and mdp problem
    os.system("python3 main.py " + prob_domain_path + " " + problem_path)
    
    # Extract probabilistic policy
    file = open(output_path+'policy.p','rb')
    prob_policy = pickle.load(file)
    file.close()
    #print('p', prob_policy)

    # Extract probabilistic mdp problem
    file = open(output_path+'mdp_problem.p','rb')
    mdp_problem = pickle.load(file)
    file.close()
    #print('mdp', mdp_problem)

    # Compare probabilistic and deterministic policies on same world (i.e. mdp problem)
    num_trials = 100
    print('Probabilistic: ', prob_policy)
    avg_reward_prob = get_average_reward(num_trials, mdp_problem, prob_policy)
    print('\nAverage reward over %d trials: %f' % (num_trials, avg_reward_prob))
    print('Deterministic: ', det_policy)
    avg_reward_det = get_average_reward(num_trials, mdp_problem, det_policy)
    print('\nAverage reward over %d trials: %f' % (num_trials, avg_reward_det))

    percentIncrease(avg_reward_prob, avg_reward_det)

    return avg_reward_prob, avg_reward_det

def get_deterministic_policy(domain_path, problem_path, output_path):

    # Run main with deterministic domain; Save policy
    os.system("python3 main.py " + domain_path + " " + problem_path)

    # Extract deterministic policy
    file = open(output_path+'policy.p','rb')
    det_policy = pickle.load(file)
    file.close()
    #print('d', det_policy)

    return det_policy

def get_probabilistic_policy(domain_path, problem_path, output_path):

    # Also returns mdp problem both the probabilistic and deterministic policy

    # Run main with probabilistic domain; Save policy and mdp problem
    os.system("python3 main.py " + domain_path + " " + problem_path)
    
    # Extract probabilistic policy
    file = open(output_path+'policy.p','rb')
    prob_policy = pickle.load(file)
    file.close()
    #print('p', prob_policy)

    # Extract probabilistic mdp problem
    file = open(output_path+'mdp_problem.p','rb')
    mdp_problem = pickle.load(file)
    file.close()
    #print('mdp', mdp_problem)

    return prob_policy, mdp_problem


def percentIncrease(prob_avg_rew, det_avg_rew):

    small = min(prob_avg_rew,det_avg_rew)
    less_small = max(prob_avg_rew,det_avg_rew)

    percent =  100*(less_small - small)/small
    print('%f is %f percent higher than %f' %(less_small,percent,small))

    return percent

def main():

    pddl_path = "/home/scheidee/new_bt_generation_ws/src/bt_generation/policy_to_behavior_tree/pypddl-parser/pypddl-parser/pddl/marine/"

    test_fp_penalty = "false_positive_penalty/" # constant probability
    test_np_penalty = "false_negative_penalty/" # constant probability
    test_both_penalty = "both_false_penalty/" # constant probability
    # test_fp_probability = "false_positive_probability_range" # constant penalty
    # test_np_probability = "false_negative_probability_range" # constant penalty

    path_to_prob_domains = pddl_path + test_both_penalty

    domain_files = os.listdir(path_to_prob_domains)

    # Init array to be plotted to show percent increase for each false positive/negative magnitude combination
    percent_increase_array = np.zeros((4,4))

    # Path to policy and mdp_problem pickle files
    output_path = "/home/scheidee/new_bt_generation_ws/src/bt_generation/policy_to_behavior_tree/mdp_to_bt/src/mdp_to_bt/policy_eval_output/"
    
    # Path to PPDDL domain and problem files
    det_domain_path = pddl_path + "domain_deterministic.ppddl"
    problem_path = pddl_path + "problems/problem1.ppddl"

    det_policy = get_deterministic_policy(det_domain_path, problem_path, output_path)

    count = 0
    for file in domain_files:

        if count < 1:
        
            #print(file)
            #fn_num,fp_num = file[2], file[5]
            #print(fn_num, fp_num)

            prob_domain_path = path_to_prob_domains + file

            print(prob_domain_path)

            prob_policy, mdp_problem = get_probabilistic_policy(prob_domain_path, problem_path, output_path)

            print(file,'\n',prob_policy)

            count += 1
        else:
            break



if __name__ == "__main__":


    main()