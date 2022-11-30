import os, sys, random, time, pickle
src_dir_ = '/home/tan/Documents/GitHub/pdpt_2022/src'
sys.path.insert(1, src_dir_)

import numpy as np
from util import generate_node_cargo_size_change, read_pickle, group_cycle_truck, manual_stop, ConsoleLogger
from pathlib import Path
from tvopdpt import tvopdpt_milp_gurobi
dir_ = '/home/tan/Documents/GitHub/pdpt_2022/'
num_ins = 10

def find_key(list_dict, value):
    return  [k for item_ in list_dict for k, v in item_.items() if value in v]

def solve_tvopdpt_mip(ins,  # dict contains the data of pdpt instance,
                     path_, # file where all data of pdotw solutions are saved
                     optimize_pdotw_routes = True,
                     max_runtime = 100,
                     verbose = 0):  

    # load data from ins
    selected_truck = ins['truck']
    selected_cargo = ins['cargo']
    selected_node = ins['nodes']
    selected_edge = ins['edge_shortest']    
    # edges = ins['edges']
    # nodes = ins['nodes']
    constant = ins['constant']
    node_cargo_size_change = ins['node_cargo_size_change']
    # path_shortest = ins['path_shortest']
    single_truck_deviation = ins['single_truck_deviation']


    # edge_shortest, path_shortest = replace_edge_by_shortest_length_nx(nodes, edges)
    # single_truck_deviation = calculate_single_truck_deviation(truck, cargo, edge_shortest)
    
    start_time = time.time()
    print(f'========= START [PDOTW with truck ========= ')

    
    created_truck = selected_truck.copy()
    
    # nodes in the cluster
    # Note. cargo['nb_cargo'] = ['size', 'lb_time', 'ub_time','departure_node', 'arrival_node']
    # truck['nb_truck'] = ['departure_node', 'arrival_node', 'max_worktime', 'max_capacity']
    

    if verbose >0:
        print(f'+++ Preprocess data to instantiate a PDOTW MIP')
    if verbose > 2:
        print(f'    [selected_cargo] size: {len(selected_cargo)}')
        for key, value in selected_cargo.items():
            print(f'        {key, value}')
        print(f'    [created_truck] size: {len(created_truck)}')
        for key, value in created_truck.items():
            print(f'       {key, value}')
    
    ### Need to update node_cargo_size_change 
    node_cargo_size_change = generate_node_cargo_size_change(selected_node, selected_cargo)

    ### group cycle and non-cycle trucks
    created_truck_yCycle, created_truck_nCycle, selected_truck = group_cycle_truck(created_truck)  
    
    if verbose > 2:
        print('    [created_truck_yCycle]', created_truck_yCycle)
        print('    [created_truck_nCycle]', created_truck_nCycle)


    if verbose >0:
        print(f'+++ Solve TVOPDPT MILP in Gurobi')
    ### use gurobi to solve the GROW origin PDPTW
    # Note. the pdotw_mip_gurobi function is desgined to take the same arguments as pdpt function
    # but some parameters
    gurobi_log_file = os.path.join(path_, f'gurobi.log')


    obj_val_MP, runtime_MP, gurobi_res = tvopdpt_milp_gurobi(constant, 
                                                           selected_cargo, 
                                                           selected_truck, 
                                                           created_truck_yCycle, 
                                                           created_truck_nCycle,
                                                           selected_node, 
                                                           selected_edge, 
                                                           node_cargo_size_change, 
                                                           max_runtime, 
                                                           gurobi_log_file, 
                                                           verbose = 1)


    x_sol, z_sol, u_sol, w_sol, S_sol, D_sol, A_sol, Sb_sol, Db_sol, Ab_sol = gurobi_res
    # Index k for truck is pre-defined
    #x_sol: x^k_{ij}, if truck k visit edge (i,j) or not
    #z_sol: z^{kr}_{ij}, if truck k visit edge (i,j) with cargo r
    #u_sol: u^r_i, if cargo r is transfered at node i
    #y_sol: y^k_r, if parcel r is carried by truck k
    #S_sol: x^k_i, total size of cargos on truck k at node i
    #D_sol: D^k_i, depature time of truck k at node i
    #A_sol: A^k_i, arrival time of truck k at node i

    res = {'obj_val_MP': obj_val_MP,
           'runtime_MP': runtime_MP,
           'x_sol':  x_sol,
           'z_sol':  z_sol,
           'u_sol':  u_sol,
           'w_sol':  w_sol,
           'S_sol':  S_sol,
           'D_sol':  D_sol,
           'A_sol':  A_sol,
           'Sb_sol': Sb_sol,
           'Db_sol': Db_sol,
           'Ab_sol': Ab_sol
          }

    return res


def select_subroutes(ins, cargo_route_file, truck_pairs_to_try, seed=0, verbose=0):


    # load data from ins
    truck = ins['truck']
    cargo = ins['cargo']
    edge_shortest = ins['edge_shortest']
    selected_truck = {}
    selected_node = []
    selected_edge = {}
    selected_cargo = {}

    cargo_to_truck_assignment = [{key: list(set([v[0] for v in value]))} for key, value in cargo_route_file.items()]

    
    # for truck_key in truck_keys_shuffle[:2]:
    for truck_key in truck_pairs_to_try:
        if verbose >0:
            print(f'========= START [PDOTW with truck {truck_key}] ========= ')

        truck_value = truck[truck_key]

        selected_truck[truck_key] = truck[truck_key]

        cargo_keys = find_key(cargo_to_truck_assignment, truck_key)
        if len(cargo_keys) > 0:
            for c_key in cargo_keys: 
                selected_cargo[c_key] = cargo[c_key]

        if truck_value[0] not in selected_node:
            selected_node.append(truck_value[0])
        if truck_value[1] not in selected_node:
            selected_node.append(truck_value[1])
    # add undelivered cargos
    for cargo_key, cargo_route in cargo_route_file.items():
        if len(cargo_route) == 0:
                selected_cargo[cargo_key] = cargo[cargo_key]


    for v in selected_cargo.values():
        if v[3] not in selected_node:
            selected_node.append(v[3])
        if v[4] not in selected_node:
            selected_node.append(v[4])


    edges_ = list(set([(i,j) for i in selected_node for j in selected_node]))

    for i,j in edges_:
        selected_edge[(i,j)] = edge_shortest[(i,j)]


    return (selected_cargo, selected_truck, selected_node, selected_edge)

def prepare_subproblem(pdpt_ins):

    truck_list = pdpt_ins['truck']
    cargo_list = pdpt_ins['cargo']
    node_list = pdpt_ins['nodes']
    edge_list = pdpt_ins['edge_shortest']    
    # edges = ins['edges']
    # nodes = ins['nodes']
    constant = pdpt_ins['constant']
    node_cargo_size_change = pdpt_ins['node_cargo_size_change']

def main():

    case_num, ins_idx, num_trucks = 1, 0, 2
    pdpt_ins_filename = os.path.join(dir_, f'data/case{case_num}', f'case{case_num}_truck{num_trucks}_ins{ins_idx+1}.pkl')
    print(f'===== START team orienteering pdpt\n      {pdpt_ins_filename}')
    pdpt_ins = read_pickle(pdpt_ins_filename)

    path_ = os.path.join(dir_, 'toy/top_pdpt')
    Path(path_).mkdir(parents=True, exist_ok=True)
    logfile = os.path.join(path_, f'pdpt_exp.log')
    ConsoleLogger.start(logfile, mode='w', time_format='[%m-%d %H:%M:%S]')

    res = solve_pdotw_mip(pdpt_ins,  # dict contains the data of pdpt instance,
                          path_, # file where all data of pdotw solutions are saved
                          optimize_pdotw_routes = True,
                          max_runtime = 100,
                          verbose = 0)

    res_filename = os.path.join(dir_, f'data/case{case_num}', f'case{case_num}_truck{num_trucks}_ins{ins_idx+1}_res.pkl')
    with open(res_filename, 'wb') as f:
        pickle.dump(res, f)

    ConsoleLogger.stop()

if __name__ == "__main__":
        

    main()