import matplotlib.pyplot as plt
import numpy as np
from math import sqrt
import os, json

from util import generate_node_cargo_size_change, calculate_single_truck_deviation, ConsoleLogger
from util import read_route_solution_PDPT, read_pdpt_pickle
from pdpt_repair import best_insert
from pdpt_ini_sol import solve_pdotw_mip
import pickle
from pathlib import Path

# Please update dir_ to the folder where you wan to save files
dir_ = '/home/tan/Documents/GitHub/pdpt_2022/toy'


def toy_example():
    ins = {}
    num_node = 16
    constant = {'truck_fixed_cost': 30000.0, 
                'truck_running_cost': 50.0, 
                'cargo_reloading_cost': 0.5, 
                'node_fixed_time': 11.0, 
                'loading_variation_coefficient': 0.0094598
               }
    loc = [[0.00,  0.00], #node 1
        [1.50, -0.50], #node 2
        [2.00, -0.75], #node 3
        [2.50, -0.75], #node 4
        [3.50, -0.75], #node 5
        [4.00, -0.50], #node 6
        [5.00, -0.75], #node 7
        [5.20, -1.75], #node 8
        [3.00, -0.50], #node 9
        [4.50,  0.00], #node 10
        [1.00, -1.25], #node 11
        [0.75, -1.75], #node 12
        [0.25, -2.00], #node 13
        [2.50, -1.75], #node 14
        [3.00, -2.50], #node 15
        [3.75, -2.00], #node 16
        ]

    node_list = [f'N{i+1}' for i in range(num_node)]

    # cargo['nb_cargo'] = ['size', 'lb_time', 'ub_time', 'departure_node', 'arrival_node']
    cargo = {'C1': [10, 0, 100, 0,  1], #cargo 1
            'C2': [10, 0, 100, 10, 11], #cargo 2
            'C3': [10, 0, 100,  9, 12], #cargo 3
            'C4': [10, 0, 100,  2,  6], #cargo 4
            'C5': [10, 0, 100,  8,  3], #cargo 5
            'C6': [10, 0, 100,  4,  6], #cargo 6
            'C7': [10, 0, 100, 13, 15], #cargo 7
            'C8': [10, 0, 100,  2, 14], #cargo 8
            }
    
    

    # truck['nb_truck'] = ['departure_node', 'arrival_node', 'max_worktime', 'max_capacity']

    truck = {'T1':[ 0,  7, 100, 100],
             'T2':[ 9, 12, 100, 100],
             'T3':[ 3, 15, 100, 100],
            }

    edge_shortest = {(i,j): round(sqrt((loc[i][0]-loc[j][0])**2 +(loc[i][1]-loc[j][1])**2),2)for i in range(num_node) for j in range(num_node)}
    node_cargo_size_change = generate_node_cargo_size_change(node_list, cargo)
    single_truck_deviation = calculate_single_truck_deviation(truck, cargo, edge_shortest)

    dict_ = {'constant': constant,
             'cargo': cargo,
             'truck': truck,
             'nodes': node_list,
             'node_cargo_size_change': node_cargo_size_change,
             'edge_shortest': edge_shortest,
             'single_truck_deviation': single_truck_deviation,
             'loc': loc
            }
    path_ = dir_+'/toy.pkl'
    with open(path_, 'wb') as pickle_file:
        pickle.dump(dict_, pickle_file)

    return dict_


def toy_ini_sol(dir_, greedy_initialization, verbose = 0):


    # for case_num in range(1, 6, 1):
    if verbose > 0:
        print('=========== START READ RAW DATA FROM CSV TO PICKLE FILES ===========')

    with open(dir_+'/toy.pkl', 'rb') as pkl_file:
            pdpt_ins = pickle.load(pkl_file)

    if verbose >0: 
        print('=========== END  =========== \n')


    if verbose > 0:
        print('=========== START CONSTRUCT INITIAL SOLUTION BY SOLVING MULTIPLE PDOTW PROBLEMS ===========')
    path_ = dir_+'/toy'
    Path(dir_+'/toy_gurobi').mkdir(parents=True, exist_ok=True)

    res = solve_pdotw_mip(pdpt_ins, path_, greedy_initialization, verbose)
    if verbose >0: 
        print('=========== END INITIAL SOLUTION  =========== \n')

    filename = dir_+'/toy_initSol_all.pkl'
    with open(filename, 'wb') as f:
        pickle.dump(res, f)

def toy_best_insertion(dir_, verbose = 0):
    filename = dir_ + '/toy_initSol.txt'
    pdpt_ins = read_pdpt_pickle(filename, verbose = verbose-1) 


    truck_yCycle_file, truck_used_file, truck_route_file, \
    cargo_route_file, S_sol_file, A_sol_file, D_sol_file, \
    Sb_sol_file, Ab_sol_file, Db_sol_file = read_route_solution_PDPT(filename, verbose = 0)

    # print(cargo_route_file)
    temp = [key  for key, value in cargo_route_file.items() if len(value)==0]
    print(temp)
    if len(temp) > 0:
        print('======= There are undelivered parcel {temp}, START best insertion heuristic\n')
        solution_file = (truck_yCycle_file, truck_used_file, truck_route_file, \
                        cargo_route_file, S_sol_file, A_sol_file, D_sol_file, \
                        Sb_sol_file, Ab_sol_file, Db_sol_file)

        best_insertion = best_insert(pdpt_ins, solution_file, 0)

    best_insert_file = dir_ + '/toy_repair.json'
    with open(best_insert_file, 'w') as json_file:
        json.dump(best_insertion, json_file)

    print('======= END best insertion heuristic\n')

    if verbose > 0:
        for cargo_key in best_insertion.keys():
            print(f' Cargo [{cargo_key}]')
            for node_ in best_insertion[cargo_key].keys():
                for truck_key in best_insertion[cargo_key][node_].keys():
                    print(f'   node [{node_}] Truck[{truck_key}]')
                    for key, value in best_insertion[cargo_key][node_][truck_key].items(): 
                        print(f'\t {key}: {value}')



def main():
    # ins = toy_example()


    # fig, ax = plt.subplots(1,1, figsize=(10,5))
    # ax.plot(*np.array(ins['loc']).T, 'o',ms =10, mfc='None', mec='k', mew=2)
    # ax.axis('off')
    # fig.savefig(dir_+'/toy.png', dpi=150, transparent=True)

    # toy_ini_sol(dir_, greedy_initialization = False, verbose = 0)


if __name__ ==  "__main__":
    main()