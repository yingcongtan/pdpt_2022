from src.util import read_pickle
from src.pdotw_mip import eval_pdotw_sol
import pickle
import numpy as np
import itertools
import json

DATA_DIR = '/home/tan/Documents/GitHub/pdpt_2022/'

case_num = 1
verbose = 1
undelivered_cargos = {}
SEED = 0

def unique_pair(list1, list2):
    unique_combinations = []
 
    # Getting all permutations of list_1
    # with length of list_2
    permut = itertools.permutations(list1, len(list2))
    
    # zip() is called to pair each permutation
    # and shorter list element into combination
    for comb in permut:
        zipped = zip(comb, list2)
        unique_combinations.append(list(zipped))
    
    return unique_combinations

def sort_(dict, ascending = True):
    assert ascending in [True,False]
    # idx = dict[:, 1].argsort()

    idx = dict[:, 1].astype(np.double).argsort()
    if ascending == True:
        dict = dict[idx, :]
    else:
        dict = dict[idx[::-1], :]
    dict_sorted = list(dict[:,0])

    return dict_sorted

def manal_break_for_insepction():
    abc = 1 
    assert abc==2


def read_inisol(case_num):

    filename = DATA_DIR + f'/out/case{case_num}initSol.txt'
    
    truck_yCycle_file, truck_used_file, truck_route_file, \
    cargo_route_file, S_sol_file, A_sol_file, D_sol_file, \
    Sb_sol_file, Ab_sol_file, Db_sol_file = read_route_solution_PDPT(filename, verbose = 0)

    undel_cargos = [key for key, value in cargo_route_file.items() if len(value)==0]


    return undel_cargos 



def sort_truck_by_distance_from_cargo(cargo_key, cargo_list, truck_list, truck_route_list, edge_distance, verbose = 0):
    csize, c_lb_time, c_ub_time, c_origin, c_dest = cargo_list[cargo_key]
    total_distance = {}
    for t_key, t_value in truck_list.items():
        if verbose > 0: print(f'\tconsider truck [{t_key}]')
        route_ = truck_route_list[t_key]
        distance = []
        if len(route_) >= 2:
            # collect a list of distances from a node on truck route  to cargo' origin
            distance.append(sum([edge_distance[(route_[i], c_origin)] for i in range(len(route_))]))
            # collect a list of distances from cargo' origin from a node on truck route 
            distance.append(sum([edge_distance[(c_origin, route_[i] )] for i in range(1, len(route_))]))
            # collect a list of distances from a node on truck route  to cargo' destination
            distance.append(sum([edge_distance[(route_[i], c_dest)] for i in range(1, len(route_))]))
            # collect a list of distances from cargo' destination from a node on truck route 
            distance.append(sum([edge_distance[(c_dest, route_[i] )] for i in range(1, len(route_))]))

            total_distance[t_key]= sum(distance)
            if verbose > 0: print(f'\ttotal distance between cargo [{cargo_key}] and truck [{t_key}] is [{total_distance[t_key]}]')
        else:
            total_distance[t_key]= 100000 # give a arbitrarily large number so t_key will not be considered in best_insertion
            if verbose > 0: print(f'\ttruck[{t_key}] is not used, assign an arbitarily large number 100000 so that it is less favorable')

    dist = np.array([ [k, v] for k, v in total_distance.items()]).reshape(-1,2)
    dist_sorted = sort_(dist, ascending = True)

    return total_distance, dist_sorted

def sort_truck_by_workhr_slackness(truck_list, truck_yCycle_file, DT_sol, DT_b_sol, verbose = 0):
    workhr_slack = {}
    for t_key, t_value in truck_list.items():
        t_origin, t_dest, t_max_workhr, t_max_cap = t_value
        total_workhr = DT_b_sol[(t_dest, t_key)] - DT_b_sol[(t_origin, t_key)] if t_key in truck_yCycle_file\
                            else DT_sol[(t_dest, t_key)] - DT_sol[(t_origin, t_key)]
        workhr_slack[t_key] = t_max_workhr - total_workhr
        if verbose> 0:
            print(f'truck [{t_key}] departuring its dest [{t_dest}] at {total_workhr}')
            print(f'   with max_worktime [{t_max_workhr}], total slack in workhr is [{workhr_slack[t_key]}]')

    slack = np.array([ [k, v] for k, v in workhr_slack.items()]).reshape(-1,2)
    # slack = slack[slack[:,1].nonzero()]
    idx_nonzero = slack[:,1].astype(np.int32).nonzero()[0]
    slack = slack[idx_nonzero,:]
    slack_sorted = sort_(slack, ascending = False)

    return workhr_slack, slack_sorted


def rank_truck_by_slacknesS_distance(pdpt_ins, solution_file, verbose = 0):
    sorted_truck_list = {}
    truck_yCycle_file, truck_used_file, truck_route_file, \
    cargo_route_file, S_sol_file, A_sol_file, D_sol_file, \
    Sb_sol_file, Ab_sol_file, Db_sol_file = solution_file

    undelivered_cargos = [key for key, value in cargo_route_file.items() if len(value)==0]


    cargo_list = pdpt_ins['cargo']
        # truck['nb_truck'] = ['departure_node', 'arrival_node', 'max_worktime', 'max_capacity']
    truck_list = pdpt_ins['truck']
    edge_shortest = pdpt_ins['edge_shortest']

    distance_sorted = {}
    for cargo_key in undelivered_cargos:
        if verbose>0:
            print(f'Cargo [{cargo_key}] is not delivered, we consider it for best_insertion')

            print(f'   +++ Find trucks with available workhr and sort in workhr_slackness')
        truck_slack, slack_sorted = sort_truck_by_workhr_slackness(truck_list, truck_yCycle_file, 
                                                        D_sol_file, Db_sol_file, verbose = 0)
        if verbose>0:
            print(f'\tsorted truck list trucks {[[t_key, truck_slack[t_key]] for t_key in slack_sorted]}')

            print(f'   +++ Find trucks with available workhr and sort in distances from the cargo')
        truck_list_ = {t_key: truck_list[t_key] for t_key in slack_sorted}
        truck_route_file_ = {t_key: truck_route_file[t_key] for t_key in slack_sorted}
        truck_dis, dis_sorted = sort_truck_by_distance_from_cargo(cargo_key, cargo_list, truck_list_, 
                                                        truck_route_file_, edge_shortest, verbose = 0)
        if verbose>0:
            print(f'\tsorted truck list trucks {[[t_key, truck_dis[t_key]] for t_key in dis_sorted]}')


        total_score = np.array([[t_key, len(slack_sorted)*2\
                                - np.where(slack_sorted == np.array(t_key))[0][0]\
                                    - np.where(dis_sorted == np.array(t_key))[0][0]]
                            for t_key in slack_sorted])
        total_score_sorted = sort_(total_score, False)
        if verbose>0:
            print(f'   +++ Rank truck based on slackness and distance:')
            print(f'\t[{total_score_sorted}]')
        sorted_truck_list[cargo_key] = total_score_sorted

    return sorted_truck_list, truck_slack, truck_dis


def checking_capacity_constraints(node_, S_sol, truck_list, truck_key, cargo_size, verbose = 0):

    flag = None
    if truck_list[truck_key][-1] - S_sol[(node_, truck_key)] > cargo_size:
        if verbose > 0:
            print(f'\t    -- truck capacity constraint satisfied')
            verbose +=1
        flag =  True
                                
    else: 
        if verbose > 0:
            print(f'\t    -- truck capacity constraint failed')

        flag =  False

    if verbose > 1:
        print(f'\t    LOG capacity constraints check')
        print(f'\t    -- total size of cargos leave {node_} is [{S_sol[(node_, truck_key)]}]')
        print(f'\t    -- available capacity is {truck_list[truck_key][-1] - S_sol[(node_, truck_key)]}')
        print(f'\t    -- cargo size capacity is {cargo_size}')
    return flag



## TODO - worktime constraint check

def insertion_cost(node_curr, node_next, truck_key, 
                   cargo_key, c_origin, c_dest, c_size, 
                   edge_shortest, loading_coe,
                   A_, D_, best_insert, verbose = 0):
    A_[(c_origin, truck_key)] = D_[(node_curr, truck_key)] + edge_shortest[(node_curr, c_origin)]
    D_[(c_origin, truck_key)] = A_[(c_origin, truck_key)] + int(np.ceil(c_size*loading_coe))
    cost = D_[(c_origin, truck_key)] + edge_shortest[(c_origin, node_next)] - A_[(node_next, truck_key)]

    if len(best_insert.items()) == 0:
        if verbose > 0:
            print(f'\tfound a new insert position {node_curr} with better cost {cost}')
        best_insert['node'] = node_curr
        best_insert['cost'] = cost
        verbose +=1


    elif best_insert['cost'] > cost:
        if verbose > 0:
            print(f'\tfound a new insert position {node_curr} with better cost {cost}')
        best_insert['node'] = node_curr
        best_insert['cost'] = cost
        verbose +=1

    if verbose > 1:
        print(f'\t    Checking work time constraints')
        print(f'\t    -- leave [{node_curr}] at {D_[(node_curr, truck_key)]}')
        print(f'\t    -- edge_shortest from {node_curr} to origin of cargo {cargo_key} at {c_origin} is {edge_shortest[(node_curr, c_origin)]}')
        print(f'\t    -- arrive cargo origin [{c_origin}] at {D_[(node_curr, truck_key)] + edge_shortest[(node_curr, c_origin)]}')
        print(f'\t    -- leaving cargo origin [{c_origin}] at {A_[(c_origin, truck_key)] + int(np.ceil(c_size*loading_coe))}')
        print(f'\t    -- original arrival time the next node [{node_next}] at {A_[(node_next, truck_key)]}')
        print(f'\t    -- entering the next node [{node_next}] at {D_[(c_origin, truck_key)] + edge_shortest[(c_origin, node_next)]}')

    return node_curr, cost

def find_all_candidate_truck_pair(truck_list, sorted_truck_list_, truck_route_file):
    truck_pair = {}
    for i in range(len(sorted_truck_list_)):
        t_key1 = sorted_truck_list_[i]
        for j in range(i+1, len(sorted_truck_list_)):
            t_key2 = sorted_truck_list_[j]
            intersect = list(set(truck_route_file[t_key1]).intersection(set(truck_route_file[t_key2])))
            if t_key1 != t_key2 and len(intersect) > 0:
                truck_pair[(t_key1, t_key2)] = []
                common_node = list(intersect)
                for node_ in common_node:
                    if (node_ != truck_list[t_key1][0] and node_ != truck_list[t_key2][0]) or (node_ != truck_list[t_key1][1] and node_ != truck_list[t_key2][1]):
                        truck_pair[(t_key1, t_key2)].append(node_)
                if len(truck_pair[(t_key1, t_key2)])==0:
                    truck_pair.pop('key', None)
    return truck_pair


def randomly_pick_one_truck_pair(truck_pair, used_truck_pair):
    np.random.seed(SEED)
    flag = True
    keys = list(truck_pair.keys())
    while flag == True:
        idx = int(np.random.randint(0, len(truck_pair), 1))
        if keys[idx] not in used_truck_pair:
            flag = False

    return keys[idx]

def return_best_insertion(pdpt_ins, A_, Ab_, D_, Db_, S_, Sb_,
                                cargo_key, cargo_list, 
                                truck_key, truck_list, 
                                route_to_search, search_index, best_insert, verbose = 0):

    edge_shortest = pdpt_ins['edge_shortest']
    loading_coe = pdpt_ins['constant']['loading_variation_coefficient']
    c_size, c_lb_time, c_ub_time, c_origin, c_dest = cargo_list[cargo_key] 

    for i_ in range(len(route_to_search)-1)[::-1]:
        node_curr = route_to_search[i_]
        node_next = route_to_search[i_+1]            
        if checking_capacity_constraints(node_curr, S_, truck_list, truck_key, c_size ) == True:
            insert_node, insert_cost = insertion_cost(node_curr, node_next, truck_key, 
                                                        cargo_key, c_origin, c_dest, c_size, 
                                                        edge_shortest, loading_coe,
                                                        A_, D_, best_insert, verbose = verbose)
        else:
            break

    return best_insert


# filename = DATA_DIR + f'/out/case{case_num}initSol.txt'

def best_insert(pdpt_ins, solution_file, verbose = 0):
    truck_yCycle_file, truck_used_file, truck_route_file, \
    cargo_route_file, S_sol_file, A_sol_file, D_sol_file, \
    Sb_sol_file, Ab_sol_file, Db_sol_file = solution_file

    if verbose > 0:
        print('======= START Sorting all trucks before best_insertion')
    sorted_truck_list, truck_slackness, _ = rank_truck_by_slacknesS_distance(pdpt_ins, solution_file, verbose)
    if verbose > 0:
        print('======= END Sorting all trucks before best_insertion\n')

    truck_list = pdpt_ins['truck']
    cargo_list = pdpt_ins['cargo']
    constant = pdpt_ins['constant']
    loading_coe = constant['loading_variation_coefficient']

    edge_shortest = pdpt_ins['edge_shortest']
    undelivered_cargos = [key for key, value in cargo_route_file.items() if len(value)==0]

    best_insertion = {}
    for cargo_key in undelivered_cargos:
        best_insertion[cargo_key] = {}
        turck_pair = {}
        used_truck_pair = []
        if verbose > 0:
            print(f'Cargo [{cargo_key}] is not delivered, we consider it for best_insertion')
        # cargo['nb_cargo'] = ['size', 'lb_time', 'ub_time', 'departure_node', 'arrival_node']
        c_size, c_lb_time, c_ub_time, c_origin, c_dest = cargo_list[cargo_key]

        sorted_truck_list_ = sorted_truck_list[cargo_key]
        truck_pair_all = find_all_candidate_truck_pair(truck_list, sorted_truck_list_, truck_route_file)
        truck_pair_key = randomly_pick_one_truck_pair(truck_pair_all, used_truck_pair)
        t_key1, t_key2 = truck_pair_key
        if verbose > 0:
            print(f'Pick two trucks:\n   {t_key1, truck_route_file[t_key1]}\n   {t_key2, truck_route_file[t_key2]}')
        common_nodes = truck_pair_all[truck_pair_key]
        if verbose > 0:
            print(f'All common nodes: {common_nodes}')

        for node_ in common_nodes:
            if verbose > 0:
                print(f'++++ START Search on common node: {node_}')
            best_insertion[cargo_key][node_]={}
            for truck_key in [t_key1, t_key2]:
                best_insertion[cargo_key][node_][truck_key]={}
                if verbose > 0:
                    print(f'   ++++ START Search on truck [{truck_key}]')
                idx_ = truck_route_file[truck_key].index(node_)
                route_before, route_after = truck_route_file[truck_key][:idx_+1], truck_route_file[truck_key][idx_:]

                if verbose > 0:
                    print(f'      ++++ Search along route {route_before} before {node_}')
                A_, Ab_ = A_sol_file.copy(), Ab_sol_file.copy()
                D_, Db_ = D_sol_file.copy(), Db_sol_file.copy()
                S_, Sb_ = S_sol_file.copy(), Sb_sol_file.copy()
                best_insertion_ = {}
                search_index = range(len(route_before)-1)[::-1]
                best_insertion_ = return_best_insertion(pdpt_ins, A_, Ab_, D_, Db_, S_, Sb_,
                                                                        cargo_key, cargo_list, 
                                                                        truck_key, truck_list, 
                                                                        route_before, search_index, 
                                                                        best_insertion_, verbose = verbose)
                                                                        
                best_insertion[cargo_key][node_][truck_key]['before'] = best_insertion_
                if verbose > 0:
                    print(f'      ++++ Search along route {route_after} after {node_}')

                best_insertion_ = {}
                search_index = range(len(route_after)-1)
                best_insertion_ = return_best_insertion(pdpt_ins, A_, Ab_, D_, Db_, S_, Sb_,
                                                                        cargo_key, cargo_list, 
                                                                        truck_key, truck_list, 
                                                                        route_after, search_index, 
                                                                        best_insertion_, verbose = verbose)
                best_insertion[cargo_key][node_][truck_key]['after'] = best_insertion_
                if verbose > 0:
                    print(f'   ++++ END Search on truck [{truck_key}]')
                    print('    ===============================')
            if verbose > 0:
                print(f'   ++++ END Search on common node: {node_}')

    return best_insertion





if __name__ == "__main__":

    print('Nothing Happen')


