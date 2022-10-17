import sys, os

src_dir_ = '/home/tan/Documents/GitHub/pdpt_2022/src'
sys.path.insert(1, src_dir_)

from util import generate_node_cargo_size_change
from util import read_pickle, group_cycle_truck
from pdotw_mip import pdotw_mip_gurobi, postprocess_solution_pdotw, eval_pdotw_sol
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
    truck_per_cargo = {}
    cargo_in_truck = {}

    for c_key, _ in cargo.items():
        truck_per_cargo[c_key] = -1
        cargo_route[c_key] = []


    ### Globally using truck_route and lb_truck
    for t_key, t_value in truck.items():
        truck_route[t_key] = []
        truck_route[t_key].append(t_value[0])
        lb_truck[t_key] = 0


    if greedy_sorting_truck == False:
        if verbose > 0:
            print('Randomly shuffle truck list')
        truck_keys_shuffle = list(truck.keys())
        random.Random(seed).shuffle(truck_keys_shuffle)

    elif greedy_sorting_truck == True:
        if verbose > 0:
            print('Greedily sort trucks accoding to capacity')
                    #improve truck shuffle
        capacity = np.array([ [k, v[-1]] for k, v in truck.items()]).reshape(-1,2)
        idx = capacity[:, 1].argsort()
        capacity = capacity[idx[::-1], :]
        truck_key_sort_by_capacity = list(capacity[:,0])
        truck_keys_shuffle = truck_key_sort_by_capacity



    res = ( (created_truck_yCycle_total, created_truck_nCycle_total, created_truck_all_total, node_list_truck_hubs_total),
            (x_sol_total, y_sol_total, S_sol_total, D_sol_total, A_sol_total, Sb_sol_total, Db_sol_total, Ab_sol_total), 
            (cost_cargo_size_value_total, cost_cargo_number_value_total, cost_travel_value_total, cost_deviation_value_total),
            (truck_used_total, truck_route, cargo_delivered_total, cargo_undelivered_total, lb_truck, cargo_route, truck_per_cargo, cargo_in_truck)
    )

    return res, truck_keys_shuffle, truck, cargo


# Here we solve multiple PDOTW MIP's to construct an intial solution for PDPT
def solve_pdotw_mip(ins,  # dict contains the data of pdpt instance,
                    path_, # file where all data of pdotw solutions are saved
                    greedy_initialization = False,
                    optimize_pdotw_routes = True,
                    verbose = 0):  

    res, truck_keys_shuffle, selected_truck, selected_cargo = initialization_pdotw(ins, greedy_initialization, verbose = verbose)

    # load data from ins
    truck = ins['truck']
    cargo = ins['cargo']
    # edges = ins['edges']
    # nodes = ins['nodes']
    constant = ins['constant']
    node_cargo_size_change = ins['node_cargo_size_change']
    edge_shortest = ins['edge_shortest']
    # path_shortest = ins['path_shortest']
    single_truck_deviation = ins['single_truck_deviation']

    created_truck_yCycle_total, created_truck_nCycle_total, created_truck_all_total, node_list_truck_hubs_total = res[0]
    x_sol_total, y_sol_total, S_sol_total, D_sol_total, A_sol_total, Sb_sol_total, Db_sol_total, Ab_sol_total = res[1]
    cost_cargo_size_value_total,  cost_cargo_number_value_total, cost_travel_value_total, cost_deviation_value_total = res[2]
    truck_used_total, truck_route, cargo_delivered_total, cargo_undelivered_total, lb_truck, cargo_route, truck_per_cargo, cargo_in_truck = res[3]

    # edge_shortest, path_shortest = replace_edge_by_shortest_length_nx(nodes, edges)
    # single_truck_deviation = calculate_single_truck_deviation(truck, cargo, edge_shortest)
    
    runtime_pdotw = []
    for truck_key in truck_keys_shuffle:
        start_time = time.time()
        print(f'========= START [PDOTW with truck {truck_key}] ========= ')

        
        created_truck = {}
        created_truck[truck_key] = tuple(selected_truck[truck_key])

        node_list_truck_hubs = {}
        
        # nodes in the cluster
        # Note. cargo['nb_cargo'] = ['size', 'lb_time', 'ub_time','departure_node', 'arrival_node']
        # truck['nb_truck'] = ['departure_node', 'arrival_node', 'max_worktime', 'max_capacity']

        selected_node = []
        for v in selected_cargo.values():
            if v[3] not in selected_node:
                selected_node.append(v[3])
            if v[4] not in selected_node:
                selected_node.append(v[4])
        if created_truck[truck_key][0] not in selected_node:
            selected_node.append(created_truck[truck_key][0])
        if created_truck[truck_key][1] not in selected_node:
            selected_node.append(created_truck[truck_key][1])
        
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
        if verbose > 2:
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
        
        if verbose > 2:
            print('    [created_truck_yCycle]', created_truck_yCycle)
            print('    [created_truck_nCycle]', created_truck_nCycle)

        if verbose > 2: 
            print('    [The created_truck_yCycle]')  
        for key, value in created_truck_yCycle.items():
            if verbose > 2: 
                print(f'       {key, value}')
            created_truck_yCycle_total[key] = value
        if verbose > 2: 
            print('    [The created_truck_nCycle]') 
        for key, value in created_truck_nCycle.items():
            if verbose > 2: 
                print(f'       {key, value}')
            created_truck_nCycle_total[key] = value
        if verbose > 2: 
            print('    [The created_truck_all]') 
        for key, value in created_truck_all.items():
            if verbose > 2: 
                print(f'       {key, value}')
            created_truck_all_total[key] = value

        if verbose >0:
            print(f'+++ Solve PDOTW MIP in Gurobi')
        ### use gurobi to solve the GROW origin PDPTW
        # Note. the pdotw_mip_gurobi function is desgined to take the same arguments as pdpt function
        # but some parameters
        gurobi_log_file = path_ + f'_gurobi/truck{truck_key}.log'

        obj_val_MP, runtime_MP, \
        x_sol, _, y_sol, S_sol, D_sol, A_sol, \
        Sb_sol, Db_sol, Ab_sol, \
        cost_cargo_size_value, cost_cargo_number_value, \
        cost_travel_value, cost_deviation_value\
        = pdotw_mip_gurobi(constant, None,   # note, by seting y_sol = None, we are solving PDOTW to maximize # of cargos we deliver
                            selected_cargo, single_truck_deviation,
                            created_truck_yCycle, created_truck_nCycle, created_truck_all,
                            node_list_truck_hubs, selected_edge, node_cargo_size_change,
                            100, gurobi_log_file, verbose = 1)

        # Index k for truck is pre-defined
        #x_sol: x^k_{ij}, if truck k visit edge (i,j) or not
        #y_sol: y^k_r, if parcel r is carried by truck k
        #s_sol: x^k_i, total size of cargos on truck k at node i
        #D_sol: D^k_i, depature time of truck k at node i
        #A_sol: A^k_i, arrival time of truck k at node i


        ### if origin stage subproblem for the current truck is feasible
        if obj_val_MP >= 0:
            if optimize_pdotw_routes == True:
                print(f'+++ reoptimize pdotw routes to reduce the travel cost')
                delivered_cargo = {cargo_key: cargo[cargo_key] for truck_key, cargo_key in y_sol.keys()}
                
                # nodes in the cluster
                # Note. cargo['nb_cargo'] = ['size', 'lb_time', 'ub_time','departure_node', 'arrival_node']
                # truck['nb_truck'] = ['departure_node', 'arrival_node', 'max_worktime', 'max_capacity']

                selected_node = []
                for v in delivered_cargo.values():
                    if v[3] not in selected_node:
                        selected_node.append(v[3])
                    if v[4] not in selected_node:
                        selected_node.append(v[4])
                if created_truck[truck_key][0] not in selected_node:
                    selected_node.append(created_truck[truck_key][0])
                if created_truck[truck_key][1] not in selected_node:
                    selected_node.append(created_truck[truck_key][1])

                # edges in the cluster
                    selected_edge = {}
                    for i in selected_node:
                        for j in selected_node:
                            selected_edge[(i,j)] = edge_shortest[(i,j)]
                    
                    node_list_truck_hubs[truck_key] = selected_node.copy()
                    assert len(created_truck) == len(node_list_truck_hubs), "Inconsistent truck numbers"
                    node_cargo_size_change = \
                    generate_node_cargo_size_change(selected_node, delivered_cargo)

                    ### group cycle and non-cycle trucks
                    created_truck_yCycle, created_truck_nCycle, created_truck_all = \
                    group_cycle_truck(created_truck) 

                obj_val_MP, runtime_MP, \
                x_sol, _, y_sol, S_sol, D_sol, A_sol, \
                Sb_sol, Db_sol, Ab_sol, \
                cost_cargo_size_value, cost_cargo_number_value, \
                cost_travel_value, cost_deviation_value\
                = pdotw_mip_gurobi(constant, y_sol,   # note, by seting y_sol = y_sol, we are minimizing the travling distance of PDOTW solution
                                    delivered_cargo, single_truck_deviation,
                                    created_truck_yCycle, created_truck_nCycle, created_truck_all,
                                    node_list_truck_hubs, selected_edge, node_cargo_size_change,
                                    100, gurobi_log_file, verbose = 1)

            else:
                print(f'+++ Save PDOTW solutions directly, skip route reoptimization')

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
            postprocess_solution_pdotw(cargo, truck, 
            selected_cargo, created_truck_all,
            node_list_truck_hubs, 
            x_sol, y_sol, truck_route, cargo_route, verbose = verbose-1)
            
            for truck_ in truck_used:
                if truck_ not in truck_used_total:
                    truck_used_total.append(truck_)

            for c_key, c_value in cargo_delivered.items():
                cargo_delivered_total[c_key] = c_value

            for c_key, c_value in cargo_undelivered.items():
                cargo_undelivered_total[c_key] = c_value

            for c_key, c_value in cargo_truck_total.items():
                if v != -1:
                    truck_per_cargo[c_key] = c_value
            for t_key, t_value in cargo_in_truck.items():
                cargo_in_truck[t_key] = t_value.copy()

            ### Remove carried cargo ######
            for key, value in y_sol.items():
                if value == 1:
                    del selected_cargo[key[1]]

        runtime_pdotw.append(time.time()-start_time)
        print(f'========= END [PDOTW with truck {truck_key}] ========= \n')

    # truck_cost, travel_cost = eval_pdotw_sol(constant, edge_shortest, truck_used_total, truck_route, 1)

    travel_cost = sum([value for value in cost_travel_value_total.values()])
    truck_cost = constant['truck_fixed_cost']*len(truck_used_total)

    res = {'MIP': {'x_sol': x_sol_total,
                   'y_sol': y_sol_total,
                   'S_sol': S_sol_total,
                   'D_sol': D_sol_total,
                   'A_sol': A_sol_total,
                   'Sb_sol': Sb_sol_total,
                   'Db_sol': Db_sol_total,
                   'Ab_sol': Ab_sol_total,
                   'runtime': runtime_pdotw,
                  },
           'route': {'truck_yCycle':list(created_truck_yCycle_total.keys()),
                     'cargo_in_truck':cargo_in_truck,
                     'used_truck': truck_used_total,
                     'truck_route': truck_route,
                     'cargo_route': cargo_route,
                    },
           'cost': {'truck_cost' : truck_cost,
                    'travel_cost' : travel_cost,
                    },
          }
    removed_cargo = list(selected_cargo.keys())

    print('+++ Summary of the initial solution')

    print('    The truck_cost:', truck_cost)
    print('    The travel cost:', travel_cost)
    print('    The total cost:', truck_cost + travel_cost)

    print(f'    [{len(removed_cargo)}] undelivered cargo: {removed_cargo}')
    print(f'    The total runtime: [{sum(runtime_pdotw)}]')

    print(f'    The truck_used_total: [{len(truck_used_total)}]')
    print(truck_used_total)

    assert len(truck_used_total) <= len(truck) , \
    "len(truck_used_total) > len(truck)"

    if verbose > 0:

        print(f'The truck_route: [{len(truck_route)}]')
        for t, v in truck_route.items():
            print(t, v)
        assert len(truck) == len(truck_route), "len(truck) != len(truck_route)"

        print(f'The cargo_route: [{len(cargo_route)}]')
        for c, v in cargo_route.items():
            print(c, v)
        assert len(cargo) == len(cargo_route), "len(cargo) != len(cargo_route)"

        if verbose>2:
            print(f'    The cargo_delivered_total: [{len(cargo_delivered_total)}]')
            print([cargo_key for cargo_key in cargo_delivered_total.keys()])
            
            print(f'The cargo_undelivered_total: [{len(cargo_undelivered_total)}]')
            print([cargo_key for cargo_key in cargo_undelivered_total.keys()])

            print(f'The truck_per_cargo: [{len(truck_per_cargo)}]')
            for t, v in truck_per_cargo.items():
                print(t, v)
            assert len(truck_per_cargo) == len(cargo), "len(truck_per_cargo) != len(cargo)"

            print(f'The cargo_in_truck: [{len(cargo_in_truck)}]')
            for t, v in cargo_in_truck.items():
                print(t, v)



    return res



def pdpt_ini_sol(case_num, dir_, greedy_initialization, optimize_pdotw_route, verbose = 0):

    # for case_num in range(1, 6, 1):
    if verbose > 0:
        print('=========== START READ RAW DATA FROM CSV TO PICKLE FILES ===========')
    pdpt_ins_file = dir_+'/data/case' + str(case_num) +'.pkl'
    pdpt_ins = read_pickle(pdpt_ins_file, verbose = verbose-2) 
    if verbose >0: print('=========== END  =========== \n')

    if verbose > 0:
        print('=========== START CONSTRUCT INITIAL SOLUTION BY SOLVING MULTIPLE PDOTW PROBLEMS ===========')

    path_ = os.path.join(dir_,'out','iniSol') + f'/case{case_num}' 
    Path(path_ +'_gurobi').mkdir(parents=True, exist_ok=True)
    res = solve_pdotw_mip(pdpt_ins, path_, greedy_initialization, optimize_pdotw_route, verbose)
    if verbose >0: print('=========== END INITIAL SOLUTION  =========== \n')
    
    filename = os.path.join(dir_,'out','iniSol')+f'/case{case_num}_iniSol.pkl'
    with open(filename, 'wb') as f:
        pickle.dump(res, f)

    