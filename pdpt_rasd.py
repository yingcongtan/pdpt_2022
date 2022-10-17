import sys, os

src_dir_ = '/home/tan/Documents/GitHub/pdpt_2022/src'
sys.path.insert(1, src_dir_)

from pdpt_route_schedule import gurobi_master_cycle, greedy_fix_MP, MP_to_SP, cpo_sub, greedy_heuristic, time_checker_cluster, calculate_SP_cost, postprocess_pdpt_solution
from util import read_pickle, group_cycle_truck, manual_stop
from pdpt_best_insertion import unique_pair
import numpy as np
import random, sys, time
import pickle

def find_key(list_dict, value):
    return  [k for item_ in list_dict for k, v in item_.items() if value in v]


def convert_pdotw_sol_to_pdpt_sol(pdpt_ins, pdotw_sol, verbose = 0): 


    edge_shortest = pdpt_ins['edge_shortest']
    truck_list = pdpt_ins['truck']
    cargo_list = pdpt_ins['cargo']
    node_list = pdpt_ins['nodes']
    constant = pdpt_ins['constant']

    # Index k for truck is pre-defined
    # x[(i, j, truck_)]
    #   x_sol: x^k_{ij}, if truck k visit edge (i,j) or not
    # y[(truck_, cargo_)]
    #   y_sol: y^k_r, if parcel r is carried by truck k
    # S[(node_, truck_)]
    #   s_sol: x^k_i, total size of cargos on truck k at node i
    #D_sol: D^k_i, depature time of truck k at node i
    #A_sol: A^k_i, arrival time of truck k at node i
    x_sol = pdotw_sol['MIP']['x_sol']
    y_sol = pdotw_sol['MIP']['y_sol']
    S_sol = pdotw_sol['MIP']['S_sol']
    D_sol = pdotw_sol['MIP']['D_sol']
    Db_sol = pdotw_sol['MIP']['Db_sol']

    # x[(i, j, truck_)], same from PDOTW
        # x_sol: x^k_{ij}, if truck k visit edge (i,j) or not
    # y[(truck_, cargo_)], same from PDOTW
        # y_sol: y^k_r, if parcel r is carried by truck k
    # s[truck_], 
    # = 1 if exist, n1, n2, truck_, cargo, s.t. x[(n1,n2,truck_)] ==1 and y[(trcuk_, cargo_)] == 1 
        # s_sol: s^k, if truck k i used or not
    # z[(edge_[0], edge_[1], truck_, cargo_)]
    # = 1 if exist, n1, n2, truck_, cargo, s.t. x[(n1,n2,truck_)] ==1 and y[(trcuk_, cargo_)] == 1 
        #z_sol: z^{kr}_{ij}, if truck k carries parcel r visit edge (i, j) or not
    # u[(node_, cargo_)]
        #u_sol: y^r_i, if parcel r is transfered at node i
    # D[(node_, truck_)], same from PDOTW
        # D_sol: D^k_i, depature time of truck k at node i
        # D_sol for destinations of cycle trucks 
    z_sol = {}
    u_sol = {}

    for edge in edge_shortest.keys():
        for truck_key in truck_list.keys():
            for cargo_key in cargo_list.keys():
                if (edge[0], edge[1], truck_key) in x_sol.keys():
                    if (truck_key, cargo_key) in y_sol.keys():
                        if x_sol[(edge[0], edge[1], truck_key)] == 1 and y_sol[(truck_key, cargo_key)] == 1:
                            z_sol[(edge[0], edge[1], truck_key, cargo_key)] = 1

                        else:
                            z_sol[(edge[0], edge[1], truck_key, cargo_key)] = 0
                    else:
                        y_sol[(truck_key, cargo_key)] = 0
                        z_sol[(edge[0], edge[1], truck_key, cargo_key)] = 0
                else:
                    x_sol[(edge[0], edge[1], truck_key)] = 0
                    z_sol[(edge[0], edge[1], truck_key, cargo_key)] = 0

    for node_ in node_list:
        for cargo_key in cargo_list.keys():
            u_sol[(node_, cargo_key)] = 0

    s_sol = {key: 1 if sum([y_sol[key, c_key] for c_key in cargo_list.keys()]) >0 else 0 for key in truck_list.keys()}


    runtime = 100

    selected_cargo = cargo_list.copy()
    selected_edge = edge_shortest.copy()
    selected_node = node_list.copy()
    selected_truck = truck_list.copy()
    created_truck_yCycle, created_truck_nCycle, created_truck_all = \
    group_cycle_truck(truck_list)
    truck_MP, truck_nodes, truck_nodes_index, cargo_in, cargo_out, \
    transfer_nodes, cargo_unload, cargo_load = \
    MP_to_SP(constant, selected_cargo, 
        created_truck_yCycle, created_truck_nCycle, created_truck_all, 
        selected_edge, selected_node,
        x_sol, s_sol, z_sol, y_sol, u_sol, D_sol)  
    
    # Evaluating the feasibility of SP
    feasibility_SP, g_sol, h_sol, D_sol = \
    cpo_sub(constant, selected_cargo, 
            created_truck_yCycle, created_truck_nCycle, created_truck_all, 
            selected_edge, selected_node,
            truck_MP, truck_nodes, truck_nodes_index, 
            cargo_in, cargo_out,
            transfer_nodes, cargo_unload, cargo_load,
            runtime)
    if feasibility_SP != 'Feasible':
        manual_stop()

    truck_cost, travel_cost, transfer_cost = \
    calculate_SP_cost(constant, selected_cargo, selected_edge, 
        truck_MP, truck_nodes, 
        cargo_in, cargo_out, transfer_nodes)

    subroutes = (selected_cargo, selected_truck, selected_node, selected_edge)

    truck_used, cargo_delivered, cargo_undelivered, \
    trucks_per_cargo, cargo_in_truck, truck_route, cargo_route = \
    postprocess_pdpt_solution(subroutes, x_sol, s_sol, y_sol, z_sol, u_sol, verbose = verbose)



    res = {'MP': {'x_sol': x_sol,
                  's_sol': s_sol,
                  'z_sol': z_sol,
                  'y_sol': y_sol,
                  'u_sol': u_sol,
                  'D_sol': D_sol,
                  'Db_sol': Db_sol,
                 },
            'SP':{'g_sol': g_sol,
                  'h_sol': h_sol,
                  'D_sol': D_sol,
                 },
            'route':{'truck_route': truck_route,
                     'cargo_route': cargo_route,
                    },
            'cost':{'truck_cost': truck_cost,
                    'travel_cost': travel_cost,
                    'transfer_cost': transfer_cost,
                    }
            }

    return res

def select_subroutes(ins, cargo_route_file, truck_pairs_to_try, seed=0, verbose=0):


    # load data from ins
    truck = ins['truck']
    cargo = ins['cargo']
    # edges = ins['edge']
    # nodes = ins['nodes']
    # constant = ins['constant']
    # node_cargo_size_change = ins['node_cargo_size_change']
    edge_shortest = ins['edge_shortest']
    # path_shortest = ins['path_shortest']
    # single_truck_deviation = ins['single_truck_deviation']
    selected_truck = {}
    selected_node = []
    selected_edge = {}
    selected_cargo = {}

    cargo_to_truck_assignment = [{key: list(set([v[0] for v in value]))} for key, value in cargo_route_file.items()]

    # print(cargo_to_truck_assignment)
    
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
           
def pdpt_route_schedule_decomposition(path_, ins, subroutes, verbose):
    """
    The main function of PDPT decomposition
    input: ct = count
    
    output:
        infeasible_clusters
        removed_cargo_MP
        removed_cargo_SP

    Three steps to handle removed cargos:
        1. add a removed cargo to a different cluster, resolve the MP-SP
        2. add all removed cargos to an unused truck
        3. LNS
    """    



    truck = ins['truck']
    constant = ins['constant']
    # single_truck_deviation = ins['single_truck_deviation']
        
    selected_cargo, selected_truck, selected_node, selected_edge = subroutes
    print('selected_truck', selected_truck)

    # Setting parameters for clustering cargo and truck 
    runtime = 100
    # count_node_cluster = 10
    # conflict_obj_coefficient = 10000
    
      
    ###### Solve the MP+SP for each cluster ######
    
    time_start = time.time()
    
    infeasible_clusters = []
    removed_cargo_MP = []
    removed_cargo_SP = []
    # solution_cluster = {}   


    ###### Initialize data structures for the solution of the current cluster ######

    # solution_created_truck = {}

    ### group cycle and non-cycle trucks
    created_truck_yCycle, created_truck_nCycle, created_truck_all = \
    group_cycle_truck(selected_truck) 


    ###### Master Problem Solved by Gurobi MIP ######

    # no variable ordering
    # heuristic = 0.2

    #x_sol: x^k_{ij}, if truck k visit edge (i,j) or not
    #s_sol: s^k, if truck k i used or not
    #z_sol: z^{kr}_{ij}, if truck k carries parcel r visit edge (i, j) or not
    #y_sol: y^k_r, if parcel r is carried by truck k
    #u_sol: y^r_i, if parcel r is transfered at node i
    #D_sol: D^k_i, depature time of truck k at node i

    gurobi_log_file = path_ + f'_gurobi/rasd_MP.log'
    print(gurobi_log_file)

    obj_val_MP, runtime_MP, \
    x_sol, s_sol, z_sol, y_sol, u_sol, D_sol, Db_sol, \
    cost_truck_value, cost_travel_value, cost_transfer_value = \
    gurobi_master_cycle(constant, selected_cargo, 
        created_truck_yCycle, created_truck_nCycle, created_truck_all, 
        selected_edge, selected_node, runtime*1, gurobi_log_file, verbose)

    if obj_val_MP < 0:
        print(f'MP infeasible, stop solving, skip to the next iter of RASD')

        # gurobi_log_file_ = path_ + f'_gurobi/rasd_MP_repair.log'

        # print(f'MP infeasible, apply greedy heuristics to fix MP')
        # infeasible_clusters.append([selected_truck.keys()])
        # selected_cargo, created_truck_all, selected_edge, selected_node, \
        # selected_cargo_removed, obj_val_MP, runtime_MP, \
        # x_sol, s_sol, z_sol, y_sol, u_sol, D_sol = \
        # greedy_fix_MP(constant, selected_cargo, 
        #     created_truck_yCycle, created_truck_nCycle, created_truck_all, 
        #     selected_edge, selected_node, runtime, gurobi_log_file_)

        # # find removed cargo from MP
        # for c in selected_cargo_removed.keys():
        #     removed_cargo_MP.append([c, -1])
        
        return False, {}
    else:
        print(f'MP feasible, move to SP')



    
    ###### The MP should be feasible here ######


    ###### Subproblem Solved by CPO CP ######

    # convert MP solution to CP parameters
    print(f'Convert MP solution to SP parameters')

    truck_MP, truck_nodes, truck_nodes_index, cargo_in, cargo_out, \
    transfer_nodes, cargo_unload, cargo_load = \
    MP_to_SP(constant, selected_cargo, 
        created_truck_yCycle, created_truck_nCycle, created_truck_all, 
        selected_edge, selected_node,
        x_sol, s_sol, z_sol, y_sol, u_sol, D_sol)  
    

    # Evaluating the feasibility of SP
    feasibility_SP, g_sol, h_sol, D_sol = \
    cpo_sub(constant, selected_cargo, 
        created_truck_yCycle, created_truck_nCycle, created_truck_all, 
        selected_edge, selected_node,
        truck_MP, truck_nodes, truck_nodes_index, 
        cargo_in, cargo_out,
        transfer_nodes, cargo_unload, cargo_load,
        runtime)

    if feasibility_SP != 'Feasible':
        print(f'SP infeasible, stop solving, skip to the next iter of RASD')

        # # Greedy heuristic for cargo removal
        # selected_removed_cargo, \
        # truck_MP, truck_nodes, truck_nodes_index, \
        # cargo_in, cargo_out, \
        # transfer_nodes, cargo_unload, cargo_load = \
        # greedy_heuristic(constant, selected_cargo, 
        #     created_truck_yCycle, created_truck_nCycle, created_truck_all, 
        #     selected_edge, selected_node,
        #     truck_MP, truck_nodes, truck_nodes_index,
        #     cargo_in, cargo_out,
        #     transfer_nodes, cargo_unload, cargo_load)

        # # find removed cargo from SP
        # for c in selected_removed_cargo:
        #     removed_cargo_SP.append([c, -1])
        return False, {}
    else:
        print(f'SP Feasible, Check time constraint')
        if time_checker_cluster(constant, selected_cargo, created_truck_all, 
                            selected_edge, truck_MP, truck_nodes, 
                            cargo_unload, cargo_load, g_sol, h_sol, D_sol):
            print('Passing time constraint check!')
        else:
            print('Failed time constraint check, exit!!!!')
            print(f'Failed time constraint check, skip to the next iter of RASD')
            return False, {}
            # sys.exit()
        
    # Calculate SP costs: truck cost + traveling cost + transfer cost
    print(f'Compute cost after solving SP')

    truck_cost, travel_cost, transfer_cost = \
    calculate_SP_cost(constant, selected_cargo, selected_edge, 
        truck_MP, truck_nodes, 
        cargo_in, cargo_out, transfer_nodes)


    

    truck_used, cargo_delivered, cargo_undelivered, \
    trucks_per_cargo, cargo_in_truck, truck_route, cargo_route =  postprocess_pdpt_solution(subroutes, x_sol, s_sol, y_sol, z_sol, u_sol, verbose)
    
    if verbose >0:
        print(f'===== summary of post-processing [{selected_truck.keys()}] route+schedule solution')
        print(f'+++ cargo to truck assignment')
        for key, value in cargo_in_truck.items():
            print(f'      {key}: {value}')
        print(f'+++ truck to cargo assignment')
        for key, value in trucks_per_cargo.items():
            print(f'      {key}: {value}')
        print(f'+++ truck route')
        for key, value in truck_route.items():
            print(f'      {key}: {value}')
        print(f'+++ cargo route')
        for key, value in cargo_route.items():
            print(f'      {key}: {value}')

    res = {'MP': {'x_sol': x_sol,
                  's_sol': s_sol,
                  'z_sol': z_sol,
                  'y_sol': y_sol,
                  'u_sol': u_sol,
                  'D_sol': D_sol,
                  'Db_sol': Db_sol,
                 },
            'SP':{'g_sol': g_sol,
                  'h_sol': h_sol,
                  'D_sol': D_sol,
                 },
            'route':{'truck_route': truck_route,
                     'cargo_route': cargo_route,
                    },
            'cost':{'truck_cost': truck_cost,
                    'travel_cost': travel_cost,
                    'transfer_cost': transfer_cost,
                    }
            }

    return True, res
    # return (x_sol, s_sol, z_sol, y_sol, u_sol, D_sol, Db_sol),\
    #        (g_sol, h_sol, D_sol), (truck_route, cargo_route),\
    #        (truck_cost, travel_cost, transfer_cost)



def pdpt_rasd(dir_, case_num, num_iteration, seed=0, verbose = 0):
    max_iter = 10

    pdpt_ins_filename = os.path.join(dir_, f'data/case{case_num}.pkl')
    pdpt_ini_sol_res_filename = os.path.join(dir_, 'out', 'iniSol', f'case{case_num}_iniSol.pkl')

    pdpt_ins = read_pickle(pdpt_ins_filename)
    # truck_list = pdpt_ins['truck']

    pdotw_sol = read_pickle(pdpt_ini_sol_res_filename)
    cargo_route = pdotw_sol['route']['cargo_route']
    best_cost = sum([c_ for c_ in pdotw_sol['cost'].values()])

    cargos_in_truck = {}
    for truck_key in truck_list.keys():
        cargos_in_truck[truck_key] = [ cargo_key  for cargo_key, c_route_ in cargo_route.items() if truck_key in [r_[0] for r_ in c_route_] ]

    truck_key_list = np.array([[truck_key, len(values)] for truck_key, values in cargos_in_truck.items() if len(values)>0], dtype=object)

    truck_key_list = truck_key_list[truck_key_list[:, 1].argsort()]  # sort by day

    truck_list = {}
    for truck_key in truck_key_list[:,0]:
        truck_list[truck_key] = pdpt_ins['truck'][truck_key]

    # pdpt_sol = convert_pdotw_sol_to_pdpt_sol(pdpt_ins, pdotw_sol, verbose = verbose-2)
    # best_cost = sum([c_ for c_ in pdpt_sol['cost'].values()])

    pdpt_sol={}


    # truck_pairs_to_try = unique_pair(list(truck_list.keys()), list(truck_list.keys()))

    truck_list_keys = list(truck_list.keys())
    truck_pairs_to_try = list(set([(truck_list_keys[i], truck_list_keys[j]) 
                                    for i in range(len(truck_list_keys)) 
                                        for j in range(i+1, len(truck_list_keys))
                                            if truck_list_keys[i] in pdotw_sol['route']['used_truck'] and truck_list_keys[j] in pdotw_sol['route']['used_truck']]))
    
    random.Random(seed).shuffle(truck_pairs_to_try)

    random.Random(seed).shuffle(truck_list_keys)

    # print('unique pair of trucks to instantiate PDPT subproblems')
    selected_truck_pairs = []
    num_success_rasd_iter = 0
    iter = 0
    while num_success_rasd_iter < num_iteration and iter <=max_iter:
        # subroutes = select_subroutes(pdpt_ins, pdotw_sol['route']['cargo_route'], pdotw_sol['route']['used_truck'], verbose)
        subroutes = select_subroutes(pdpt_ins, pdotw_sol['route']['cargo_route'], truck_pairs_to_try[iter], seed=0, verbose=0)
        print(f'++++++ instantitate PDPT subproblem on truck {list(subroutes[1].keys())}')
        print(subroutes[-2])
        feasibility_flag, res = pdpt_route_schedule_decomposition(dir_+f'/out/case{case_num}', pdpt_ins, subroutes, verbose = 1)
    
        if feasibility_flag == True:
            cost = sum([c_ for c_ in res['cost'].values()])

            if cost < best_cost:
                print('find better_solution in RASD')
                pdpt_sol = res
                num_success_rasd_iter += 1
        iter += 1
        selected_truck_pairs.append(list(subroutes[1].keys()))
        # truck_pairs_to_try.remove(list(subroutes[1].keys()))


    pdpt_sol_filename = os.path.join(dir_, f'out/case{case_num}_rasdSol.pkl')
    with open(pdpt_sol_filename, 'wb') as pickle_file:
        pickle.dump(pdpt_sol, pickle_file)
    
    


def example():
    return None


if __name__ == '__main__':

    example()