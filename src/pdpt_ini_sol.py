from util import read_case, generate_node_cargo_size_change, read_constant
from util import read_pdpt_csv_to_pickle, read_pdpt_pickle, group_cycle_truck,  store_route_solution_PDPT, read_route_solution_PDPT
from pdotw_mip import pdotw_mip_gurobi, postprocess_solution_oneTruck
import time
import random
import pickle
import numpy as np
from pathlib import Path

# DATA_DIR = '/home/tan/Documents/PDPT_src/data'


def initialization_pdotw(ins, greedy_sorting_truck = False, seed = 0, verbose = 0):
    cargo = ins['cargo']
    truck = ins['truck']

    created_truck_yCycle_total = {}
    created_truck_nCycle_total = {}
    created_truck_all_total = {}
    node_list_truck_hubs_total = {}

    # store PDPT solution
    x_sol_total = {}
    y_sol_total = {}
    S_sol_total = {}
    D_sol_total = {}
    A_sol_total = {}
    Sb_sol_total = {}
    Db_sol_total = {}
    Ab_sol_total = {} 

    # store PDPT obj
    cost_cargo_size_value_total = {}
    cost_cargo_number_value_total = {}
    cost_travel_value_total = {}
    cost_deviation_value_total = {}

    # track truck and cargo used in PDOTW solutions
    truck_used_total = []
    truck_route = {}
    cargo_delivered_total = {} 
    cargo_undelivered_total = {} 
    lb_truck = {}
    cargo_route = {}
    cargo_truck_total_all = {}
    cargo_in_truck_total = {}

    for c_key, _ in cargo.items():
        cargo_truck_total_all[c_key] = -1
        cargo_route[c_key] = []


    ### Globally using truck_route and lb_truck
    for t_key, t_value in truck.items():
        truck_route[t_key] = []
        truck_route[t_key].append(t_value[0])
        lb_truck[t_key] = 0

    selected_truck = {}
    for t_key, t_value in truck.items():
        selected_truck[t_key] = t_value
    selected_cargo = {}
    for c_key, c_value in cargo.items():
        selected_cargo[c_key] = c_value

    if greedy_sorting_truck == False:
        if verbose > 0:
            print('Randomly shuffle truck list')
        truck_keys_shuffle = list(selected_truck.keys())
        random.Random(seed).shuffle(truck_keys_shuffle)

    elif greedy_sorting_truck == True:
        if verbose > 0:
            print('Greedily sort trucks accoding to capacity')
                    #improve truck shuffle
        capacity = np.array([ [k, v[-1]] for k, v in selected_truck.items()]).reshape(-1,2)
        idx = capacity[:, 1].argsort()
        capacity = capacity[idx[::-1], :]
        truck_key_sort_by_capacity = list(capacity[:,0])
        truck_keys_shuffle = truck_key_sort_by_capacity



    res = ( (created_truck_yCycle_total, created_truck_nCycle_total, created_truck_all_total, node_list_truck_hubs_total),
            (x_sol_total, y_sol_total, S_sol_total, D_sol_total, A_sol_total, Sb_sol_total, Db_sol_total, Ab_sol_total), 
            (cost_cargo_size_value_total, cost_cargo_number_value_total, cost_travel_value_total, cost_deviation_value_total),
            (truck_used_total, truck_route, cargo_delivered_total, cargo_undelivered_total, lb_truck, cargo_route, cargo_truck_total_all, cargo_in_truck_total)
    )

    return res, truck_keys_shuffle, selected_truck, selected_cargo

# Here we solve multiple PDOTW MIP's to construct an intial solution for PDPT
def solve_pdotw_mip(ins,  # dict contains the data of pdpt instance,
                    path_, # file where all data of pdotw solutions are saved
                    greedy_initialization = False,
                    verbose = 0):  

    res, truck_keys_shuffle, selected_truck, selected_cargo = initialization_pdotw(ins, greedy_initialization, verbose)


    # load data from ins
    truck = ins['truck']
    cargo = ins['cargo']
    # edges = ins['edges']
    nodes = ins['nodes']
    constant = ins['constant']
    node_cargo_size_change = ins['node_cargo_size_change']
    edge_shortest = ins['edge_shortest']
    # path_shortest = ins['path_shortest']
    single_truck_deviation = ins['single_truck_deviation']

    created_truck_yCycle_total, created_truck_nCycle_total, created_truck_all_total, node_list_truck_hubs_total = res[0]
    x_sol_total, y_sol_total, S_sol_total, D_sol_total, A_sol_total, Sb_sol_total, Db_sol_total, Ab_sol_total = res[1]
    cost_cargo_size_value_total,  cost_cargo_number_value_total, cost_travel_value_total, cost_deviation_value_total = res[2]
    truck_used_total, truck_route, cargo_delivered_total, cargo_undelivered_total, lb_truck, cargo_route, cargo_truck_total_all, cargo_in_truck_total = res[3]

    # edge_shortest, path_shortest = replace_edge_by_shortest_length_nx(nodes, edges)
    # single_truck_deviation = calculate_single_truck_deviation(truck, cargo, edge_shortest)
    time_start_all = time.time()

    for truck_key in truck_keys_shuffle:
        if verbose >0:
            print(f'========= START [PDOTW with truck {truck_key}] ========= ')

        tv = selected_truck[truck_key]
        
        created_truck = {}
        node_list_truck_hubs = {}
        created_truck[truck_key] = tv
        
        # nodes in the cluster
        # Note. cargo['nb_cargo'] = ['size', 'lb_time', 'ub_time','departure_node', 'arrival_node']
        # truck['nb_truck'] = ['departure_node', 'arrival_node', 'max_worktime', 'max_capacity']

        selected_node = []
        for v in selected_cargo.values():
            if v[3] not in selected_node:
                selected_node.append(v[3])
            if v[4] not in selected_node:
                selected_node.append(v[4])
        if tv[0] not in selected_node:
            selected_node.append(tv[0])
        if tv[1] not in selected_node:
            selected_node.append(tv[1])
        
        # edges in the cluster
        selected_edge = {}
        for i in selected_node:
            for j in selected_node:
                selected_edge[(i,j)] = edge_shortest[(i,j)]
        
        node_list_truck_hubs[truck_key] = selected_node.copy()
        node_list_truck_hubs_total[truck_key] = selected_node.copy()
        assert len(created_truck) == len(node_list_truck_hubs), "Inconsistent truck numbers"

        if verbose >0:
            print(f'+++ Preprocess data to instantiate a PDOTW MIP')
        if verbose > 1:
            print(f'    [selected_cargo] size: {len(selected_cargo)}')
            for key, value in selected_cargo.items():
                print(f'        {key, value}')
            print(f'    [created_truck] size: {len(created_truck)}')
            for key, value in created_truck.items():
                print(f'       {key, value}')
            print(f'    [node_list_truck_hubs] size: {len(node_list_truck_hubs)}')
            for key, value in node_list_truck_hubs.items():
                print(f'       {key, value}')
        
        ### Need to update node_cargo_size_change 
        node_cargo_size_change = \
        generate_node_cargo_size_change(selected_node, selected_cargo)

        ### group cycle and non-cycle trucks
        created_truck_yCycle, created_truck_nCycle, created_truck_all = \
        group_cycle_truck(created_truck)  
        
        if verbose > 1:
            print('    [created_truck_yCycle]', created_truck_yCycle)
            print('    [created_truck_nCycle]', created_truck_nCycle)

        if verbose > 1: 
            print('    [The created_truck_yCycle]')  
        for key, value in created_truck_yCycle.items():
            if verbose > 1: 
                print(f'       {key, value}')
            created_truck_yCycle_total[key] = value
        if verbose > 1: 
            print('    [The created_truck_nCycle]') 
        for key, value in created_truck_nCycle.items():
            if verbose > 1: 
                print(f'       {key, value}')
            created_truck_nCycle_total[key] = value
        if verbose > 1: 
            print('    [The created_truck_all]') 
        for key, value in created_truck_all.items():
            if verbose > 1: 
                print(f'       {key, value}')
            created_truck_all_total[key] = value

        if verbose >0:
            print(f'+++ Solve PDOTW MIP in Gurobi')
        ### use gurobi to solve the GROW origin PDPTW
        # Note. the pdotw_mip_gurobi function is desgined to take the same arguments as pdpt function
        # but some parameters
        gurobi_log_file = path_ + f'_gurobi/truck{truck_key}.log'
        print('gurobi_log_file', gurobi_log_file)
        obj_val_MP, runtime_MP, \
        x_sol, z_sol, y_sol, S_sol, D_sol, A_sol, \
        Sb_sol, Db_sol, Ab_sol, \
        slack_clb_sol, slack_cub_sol, \
        slack_tlb_sol, slack_tub_sol, \
        cost_cargo_size_value, cost_cargo_number_value, \
        cost_travel_value, cost_deviation_value, \
        _, _, \
        _, _ = \
        pdotw_mip_gurobi(constant, truck, 
        selected_cargo, single_truck_deviation,
        created_truck_yCycle, created_truck_nCycle, created_truck_all,
        node_list_truck_hubs, selected_edge, node_cargo_size_change,
        {}, 100, gurobi_log_file, 1)

        ### if origin stage subproblem for the current truck is feasible
        if obj_val_MP >= 0:
            if verbose >0:
                print(f'+++ Postprocee Gurobi solution if a feasible solution is found')
            cost_cargo_size_value_total[truck_key] = cost_cargo_size_value
            cost_cargo_number_value_total[truck_key] = cost_cargo_number_value
            cost_travel_value_total[truck_key] = cost_travel_value
            cost_deviation_value_total[truck_key] = cost_deviation_value

            for key, value in x_sol.items():
                x_sol_total[key] = value
            for key, value in y_sol.items():
                y_sol_total[key] = value
            for key, value in S_sol.items():
                S_sol_total[key] = value
            for key, value in D_sol.items():
                D_sol_total[key] = value
            for key, value in A_sol.items():
                A_sol_total[key] = value
            for key, value in Sb_sol.items():
                Sb_sol_total[key] = value
            for key, value in Db_sol.items():
                Db_sol_total[key] = value
            for key, value in Ab_sol.items():
                Ab_sol_total[key] = value

            ### post-process the solution
            truck_used, cargo_delivered, cargo_undelivered, \
            cargo_truck_total, cargo_in_truck = \
            postprocess_solution_oneTruck(constant, cargo, truck, 
            edge_shortest, selected_cargo, 
            created_truck_yCycle, created_truck_nCycle, created_truck_all,
            node_list_truck_hubs, 
            x_sol, z_sol, y_sol, S_sol, D_sol, A_sol, Sb_sol, Db_sol, Ab_sol, 
            0, {}, {},
            {}, {}, {}, 
            truck_route, lb_truck, cargo_route)
            
            for truck_ in truck_used:
                if truck_ not in truck_used_total:
                    truck_used_total.append(truck_)

            for c_key, c_value in cargo_delivered.items():
                cargo_delivered_total[c_key] = c_value

            for c_key, c_value in cargo_undelivered.items():
                cargo_undelivered_total[c_key] = c_value

            for c_key, c_value in cargo_truck_total.items():
                if v != -1:
                    cargo_truck_total_all[c_key] = c_value
            for t_key, t_value in cargo_in_truck.items():
                cargo_in_truck_total[t_key] = t_value.copy()

            ### Remove carried cargo ######
            for key, value in y_sol.items():
                if value == 1:
                    del selected_cargo[key[1]]

        print(f'========= END [PDOTW with truck {truck_key}] ========= \n')

    res =((created_truck_yCycle_total, created_truck_nCycle_total, created_truck_all_total, node_list_truck_hubs_total),
          (x_sol_total, y_sol_total, S_sol_total, D_sol_total, A_sol_total, Sb_sol_total, Db_sol_total, Ab_sol_total),
          (cost_cargo_size_value_total,  cost_cargo_number_value_total, cost_travel_value_total, cost_deviation_value_total),
          (truck_used_total, truck_route, cargo_delivered_total, cargo_undelivered_total, lb_truck, cargo_route, cargo_truck_total_all, cargo_in_truck_total))

    pdpt_res_file = path_ +'initSol.txt'
    if verbose > 0:
        print('+++ Saving PDPT initial solutino to {pdpt_res_file}')
    store_route_solution_PDPT(pdpt_res_file, cargo, created_truck_yCycle_total, 
    created_truck_nCycle_total, truck, 
    truck_used_total, truck_route, cargo_route, x_sol_total, y_sol_total, 
    S_sol_total, D_sol_total, A_sol_total, 
    Sb_sol_total, Db_sol_total, Ab_sol_total)

    print('+++ Summary of the initial solution')
    print('\nThe final length of selected_cargo is:', len(selected_cargo))
    removed_cargo = []
    for c in selected_cargo.keys():
        removed_cargo.append(c)

    print('\nThe truck_used_total:', len(truck_used_total))
    print(truck_used_total)

    assert len(truck_used_total) <= len(truck) , \
    "len(truck_used_total) > len(truck)"

    print('\nThe truck_route:', len(truck_route))
    for t, v in truck_route.items():
        print(t, v)
    assert len(truck) == len(truck_route), "len(truck) != len(truck_route)"

    print('\nThe cargo_delivered_total:', len(cargo_delivered_total))
    for t, v in cargo_delivered_total.items():
        print(t, v)
    print('\nThe cargo_undelivered_total:', len(cargo_undelivered_total))
    for t, v in cargo_undelivered_total.items():
        print(t, v)


    print('\nThe lb_truck:', len(lb_truck))
    for t, v in lb_truck.items():
        print(t, v)
    assert len(truck) == len(lb_truck), "len(truck) != len(lb_truck)"

    print('\nThe cargo_truck_total_all:', len(cargo_truck_total_all))
    for t, v in cargo_truck_total_all.items():
        print(t, v)
    assert len(cargo_truck_total_all) == len(cargo), \
    "len(cargo_truck_total_all) != len(cargo)"

    print('\nThe cargo_in_truck_total:', len(cargo_in_truck_total))
    for t, v in cargo_in_truck_total.items():
        print(t, v)

    print('\nThe cargo_route:', len(cargo_route))
    for c, v in cargo_route.items():
        print(c, v)
    assert len(cargo) == len(cargo_route), "len(cargo) != len(cargo_route)"

    print('\nThe removed_cargo after the greedy approach are:', len(removed_cargo))
    print(removed_cargo)

    time_end_all = time.time()
    print('\nThe total runtime is:', time_end_all - time_start_all)


    return res



def pdpt_ini_sol(case_num, dir_, greedy_initialization, verbose = 0):

    # for case_num in range(1, 6, 1):
    if verbose > 0:
        print('=========== START READ RAW DATA FROM CSV TO PICKLE FILES ===========')
    pdpt_ins_file = dir_+'/data/case' + str(case_num) +'.pkl'
    pdpt_ins = read_pdpt_pickle(pdpt_ins_file, verbose = verbose-1) 
    if verbose >0: print('=========== END  =========== \n')

    if verbose > 0:
        print('=========== START CONSTRUCT INITIAL SOLUTION BY SOLVING MULTIPLE PDOTW PROBLEMS ===========')
    Path(dir_+'/out/case'+ str(case_num) +'_gurobi').mkdir(parents=True, exist_ok=True)

    path_ = dir_+'/out/case' + str(case_num) 
    res = solve_pdotw_mip(pdpt_ins, path_, greedy_initialization, verbose)
    if verbose >0: print('=========== END INITIAL SOLUTION  =========== \n')

    filename = dir_+'/out/case' + str(case_num) +'initSol_all.pkl'
    with open(filename, 'wb') as f:
        pickle.dump(res, f)
    



    # truck_yCycle_file, truck_used_file, truck_route_file, \
    # cargo_route_file, S_sol_file, A_sol_file, D_sol_file, \
    # Sb_sol_file, Ab_sol_file, Db_sol_file = \
    # read_route_solution_PDPT(filename, verbose = verbose)
