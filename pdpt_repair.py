import sys, os

src_dir_ = '/home/tan/Documents/GitHub/pdpt_2022/src'
sys.path.insert(1, src_dir_)

from util import read_pickle
from pdotw_mip import eval_pdotw_sol
from pdpt_best_insertion import find_all_candidate_truck_pair, randomly_pick_one_truck_pair,\
                                return_best_insertion, rank_truck_by_slackness_distance

import pickle
import numpy as np
import itertools
import json

DATA_DIR = '/home/tan/Documents/GitHub/pdpt_2022/'

    # ini_sol.pickle
    # res = {'MIP': {'x_sol': x_sol_total,
    #                'y_sol': y_sol_total,
    #                'S_sol': S_sol_total,
    #                'D_sol': D_sol_total,
    #                'A_sol': A_sol_total,
    #                'Sb_sol': Sb_sol_total,
    #                'Db_sol': Db_sol_total,
    #                'Ab_sol': Ab_sol_total,
    #                'runtime': runtime_pdotw,
    #               },
    #        'route': {'truck_yCycle':list(created_truck_yCycle_total.keys()),
    #                  'used_truck': truck_used_total,
    #                  'truck_route': truck_route,
    #                  'cargo_route': cargo_route,
    #                 },
    #        'cost': {'truck_cost' : truck_cost,
    #                 'travel_cost' : travel_cost,
    #                 },
    #       }


def best_insert(pdpt_ins, ini_sol, verbose = 0):
    '''
    return best_insertion[cargo_key][node][truck_key]

    The best_insert function follows the pesudo-code as below:

        For each undelivered_cargos:
            Find a pair of trucks, T1, T2
            Find common nodes visited by T1 and T2
            For each node in the list of common nodes


    
    '''
    truck_route = ini_sol['route']['truck_route']
    cargo_route = ini_sol['route']['cargo_route']
    S_sol = ini_sol['MIP']['S_sol']
    A_sol = ini_sol['MIP']['A_sol']
    D_sol = ini_sol['MIP']['D_sol']


    Sb_sol = ini_sol['MIP']['Sb_sol']
    Ab_sol = ini_sol['MIP']['Ab_sol']
    Db_sol = ini_sol['MIP']['Db_sol']


    if verbose > 0:
        print('======= START Sorting all trucks before best_insertion')
    sorted_truck_list, truck_slackness, _ = rank_truck_by_slackness_distance(pdpt_ins, ini_sol, verbose)
    if verbose > 0:
        print('======= END Sorting all trucks before best_insertion\n')

    truck_list = pdpt_ins['truck']
    cargo_list = pdpt_ins['cargo']
    constant = pdpt_ins['constant']
    loading_coe = constant['loading_variation_coefficient']

    edge_shortest = pdpt_ins['edge_shortest']
    undelivered_cargos = [key for key, value in cargo_route.items() if len(value)==0]

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
        truck_pair_all = find_all_candidate_truck_pair(truck_list, sorted_truck_list_, truck_route)
        truck_pair_key = randomly_pick_one_truck_pair(truck_pair_all, used_truck_pair)
        t_key1, t_key2 = truck_pair_key

        if verbose > 0:
            print(f'Pick two trucks:\n   {t_key1, truck_route[t_key1]}\n   {t_key2, truck_route[t_key2]}')
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
                idx_ = truck_route[truck_key].index(node_)
                route_before, route_after = truck_route[truck_key][:idx_+1], truck_route[truck_key][idx_:]

                if verbose > 0:
                    print(f'      ++++ Search along route {route_before} before {node_}')
                A_, Ab_ = A_sol.copy(), Ab_sol.copy()
                D_, Db_ = D_sol.copy(), Db_sol.copy()
                S_, Sb_ = S_sol.copy(), Sb_sol.copy()
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


    return False



def pdpt_sol_repair(case_num, dir_, verbose = 0):

    ini_sol_file = os.path.join(dir_, 'out','iniSol', f'case{case_num}_iniSol.pkl')
    pdpt_ins_file = os.path.join(dir_, 'data', f'case{case_num}.pkl')

    pdpt_ins = read_pickle(pdpt_ins_file)
    ini_sol = read_pickle(ini_sol_file)

    best_insertion = best_insert(pdpt_ins, ini_sol, verbose = 0)

    print(best_insertion)


if __name__ == "__main__":

    print('Nothing Happen')


