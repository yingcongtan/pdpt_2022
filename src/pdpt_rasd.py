from pdpt_route_schedule import gurobi_master_cycle, greedy_fix_MP, MP_to_SP, cpo_sub, greedy_heuristic, time_checker_cluster, calculate_SP_cost, capacity_checker_cluster
from util import read_pdpt_pickle, read_route_solution_PDPT, group_cycle_truck 
import numpy as np
import random, sys, time
import pickle

def find_key(list_dict, value):
    return  [k for item_ in list_dict for k, v in item_.items() if value in v]

def initialization_rasd(ins, greedy_sorting_truck = False, seed=0, verbose = 0):
    cargo = ins['cargo']
    truck = ins['truck']

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


    return truck_keys_shuffle, selected_truck, selected_cargo    


def select_subroutes(ins, cargo_route_file, verbose):

    truck_keys_shuffle, selected_truck, selected_cargo = initialization_rasd(ins, False, verbose)

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
    
    for truck_key in truck_keys_shuffle[:2]:
    # for truck_key in ['T10','T1','T19','T5','T2','T7','T16','T12','T9','T18','T8','T6','T4','T15','T3','T14','T17','T11','T13']:
    # truck list from Jason's jupyter notebook
        if verbose >0:
            print(f'========= START [PDOTW with truck {truck_key}] ========= ')

        truck_value = truck[truck_key]

        selected_truck[truck_key] = truck[truck_key]

        cargo_keys = find_key(cargo_to_truck_assignment, truck_key)
        if len(cargo_keys) > 0:
            for c_key in cargo_keys: 
                selected_cargo[c_key] = cargo[c_key]

        
        for v in selected_cargo.values():
            if v[3] not in selected_node:
                selected_node.append(v[3])
            if v[4] not in selected_node:
                selected_node.append(v[4])
        if truck_value[0] not in selected_node:
            selected_node.append(truck_value[0])
        if truck_value[1] not in selected_node:
            selected_node.append(truck_value[1])

    edges_ = list(set([(i,j) for i in selected_node for j in selected_node]))

    for i,j in edges_:
        selected_edge[(i,j)] = int(edge_shortest[(i,j)])

    return selected_cargo, selected_truck, selected_node, selected_edge

def postprocess_pdpt_solution(subroutes, x_sol, y_sol, z_sol, u_sol, verbose = 0):

    selected_cargo, selected_truck, selected_node, selected_edge = subroutes

    #x_sol: x^k_{ij}, if truck k visit edge (i,j) or not
    #s_sol: s^k, if truck k i used or not
    #z_sol: z^{kr}_{ij}, if truck k carries parcel r visit edge (i, j) or not
    #y_sol: y^k_r, if parcel r is carried by truck k
    #u_sol: y^r_i, if parcel r is transfered at node i
    #D_sol: D^k_i, depature time of truck k at node i
    
    truck_used = []         # list of trucks used in the solution
    
    # dictionary, for each truck_key, there is a list of cargo carried that was on this truck. E.g., {'T1": ['C1', 'C2', ...]}
    cargo_in_truck = {}   
    for truck_key in selected_truck.keys():
        cargo_in_truck[truck_key] = []

    # dictionary, for each cargo_key, there is a list of truck that carried this cargo. E.g., {'C1": ['T1', 'T2', ...]}
    trucks_per_cargo = {} 
    for cargo_key in selected_cargo.keys():
        trucks_per_cargo[cargo_key] = []

    cargo_delivered = []   # list of delivered cargo
    cargo_undelivered = [] # list of undelivered cargo


    truck_route = {}
    for truck_key in selected_truck.keys():
        truck_route[truck_key] = []
    cargo_route = {}
    for cargo_key in selected_cargo.keys():
        cargo_route[cargo_key] = []


    # postprocess cargo_in_truck and truck_used
    for truck_key in selected_truck:
        cargo_in_truck[truck_key] = []
        truck_used_flag = 0
        # Generate cargo_in_truck
        for cargo_key in selected_cargo.keys():
            if y_sol[(truck_key, cargo_key)] == 1:
                if verbose > 0: 
                    print('    The cargo {} has been carried by truck {}'.format(cargo_key, truck_key))
                truck_used_flag += 1
                cargo_in_truck[truck_key].append(cargo_key)
        if truck_used_flag > 0:
            
            # put the truck_key into used trucks
            truck_used.append(truck_key)

    # postprocess truck_route
    for truck_key in selected_truck:
        source = selected_truck[truck_key][0] # starting from the origin node of each truck
        for node_ in selected_node:
            if node_ != source and x_sol[(source, node_, truck_key)] == 1: # find node_ such that x[source, node_, truck_key] == 1
                if len(truck_route[truck_key]) == 0: # append source as the first node
                        truck_route[truck_key].append(source)
                truck_route[truck_key].append(node_)
                source = node_
                break
        while source != selected_truck[truck_key][1]: # terminate when reach the arival node of each truck
            for node_ in selected_node:
                if node_ != source and x_sol[(source, node_, truck_key)] == 1: # find node_ such that x[source, node_, truck_key] == 1
                    truck_route[truck_key].append(node_)
                    source = node_
                    break

    # RECALL, cargo is a dictionary with the following format:
    # cargo['nb_cargo'] = ['size', 'lb_time', 'ub_time', 'departure_node', 'arrival_node']
    for cargo_key, cargo_value in selected_cargo.items():
        dest = cargo_value[-1]
        if sum([z_sol[(node_, dest, truck_key, cargo_key)] for node_ in selected_node for truck_key in selected_truck.keys() if node_!= dest]) == 1:
            truck_list = [truck_key for truck_key in selected_truck.keys()  # list of truck_key
                                    if y_sol[(truck_key, cargo_key)]  == 1  ]# if cargo_key is carried by truck_key i.e., y^k_r == 1
            source = cargo_value[3]
            while source != cargo_value[-1]: # terminate when reaching cargo arrival node
                for node_ in selected_node:
                    for truck_key in truck_list:
                        if node_!=source and y_sol[(truck_key, cargo_key)] == 1\
                                        and z_sol[(source, node_, truck_key, cargo_key)] == 1: #z_sol: z^{kr}_{ij}, if truck k carries parcel r visit edge (i, j) or not
                            trucks_per_cargo[cargo_key].append(truck_key)
                            if len(cargo_route[cargo_key]) == 0: # append source as the first node
                                cargo_route[cargo_key].append(source)
                            cargo_route[cargo_key].append((truck_key, node_))
                            source = node_
        else:
            cargo_undelivered.append(cargo_key)
            cargo_route[cargo_key] = []
            if verbose > 0: 
                print(f'+++ Failed to deliver cargo {cargo_key}')
 

    return truck_used, cargo_delivered, cargo_undelivered, \
           trucks_per_cargo, cargo_in_truck, truck_route, cargo_route 
           
def pdpt_route_schedule_decomposition(path_, ins, initial_solution, subroutes, verbose):
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

    truck_yCycle_file, truck_used_file, truck_route_file, \
    cargo_route_file, S_sol_file, A_sol_file, D_sol_file, \
    Sb_sol_file, Ab_sol_file, Db_sol_file = initial_solution
    
    truck = ins['truck']
    # cargo = ins['cargo']
    # edges = ins['edges']
    # node_list = ins['nodes']
    constant = ins['constant']
    # node_cargo_size_change = ins['node_cargo_size_change']
    # edge = ins['edge_shortest']
    # path_shortest = ins['path_shortest']
    # single_truck_deviation = ins['single_truck_deviation']
        
    selected_cargo, selected_truck, selected_node, selected_edge = subroutes

    for cargo_key in selected_cargo:
        cargo_route_file[cargo_key] = []
    for truck_key in selected_truck:
        truck_route_file[truck_key] = []
        truck_route_file[truck_key].append(truck[truck_key][0])

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

    obj_val_MP, runtime_MP, \
    x_sol, s_sol, z_sol, y_sol, u_sol, D_sol, Db_sol, \
    cost_truck_value, cost_travel_value, cost_transfer_value = \
    gurobi_master_cycle(constant, selected_cargo, 
        created_truck_yCycle, created_truck_nCycle, created_truck_all, 
        selected_edge, selected_node, runtime*1, gurobi_log_file, verbose)

    if obj_val_MP < 0:
        print(f'MP infeasible, apply greedy heuristics to fix MP')
        infeasible_clusters.append([selected_truck.keys()])
        selected_cargo, created_truck_all, selected_edge, selected_node, \
        selected_cargo_removed, obj_val_MP, runtime_MP, \
        x_sol, s_sol, z_sol, y_sol, u_sol, D_sol = \
        greedy_fix_MP(constant, selected_cargo, 
            created_truck_yCycle, created_truck_nCycle, created_truck_all, 
            selected_edge, selected_node, runtime)

        # find removed cargo from MP
        for c in selected_cargo_removed.keys():
            removed_cargo_MP.append([c, -1])
        
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
        print(f'SP infeasible, apply greedy heuristics to fix SP')

        # Greedy heuristic for cargo removal
        selected_removed_cargo, \
        truck_MP, truck_nodes, truck_nodes_index, \
        cargo_in, cargo_out, \
        transfer_nodes, cargo_unload, cargo_load = \
        greedy_heuristic(constant, selected_cargo, 
            created_truck_yCycle, created_truck_nCycle, created_truck_all, 
            selected_edge, selected_node,
            truck_MP, truck_nodes, truck_nodes_index,
            cargo_in, cargo_out,
            transfer_nodes, cargo_unload, cargo_load)

        # find removed cargo from SP
        for c in selected_removed_cargo:
            removed_cargo_SP.append([c, -1])
    else:
        print(f'SP Feasible, apply greedy heuristics to fix SP')
        if time_checker_cluster(constant, selected_cargo, created_truck_all, 
                            selected_edge, truck_MP, truck_nodes, 
                            cargo_unload, cargo_load, g_sol, h_sol, D_sol):
            print('Passing time constraint check!')
        else:
            print('Failed time constraint check, exit!!!!')

            sys.exit()
        
    # Calculate SP costs: truck cost + traveling cost + transfer cost
    print(f'Compute cost after solving SP')

    truck_cost, travel_cost, transfer_cost = \
    calculate_SP_cost(constant, selected_cargo, selected_edge, 
        truck_MP, truck_nodes, 
        cargo_in, cargo_out, transfer_nodes)


    

    truck_used, cargo_delivered, cargo_undelivered, \
    trucks_per_cargo, cargo_in_truck, truck_route, cargo_route =  postprocess_pdpt_solution(subroutes, x_sol, y_sol, z_sol, u_sol, verbose)
    
    if verbose >0:
        print(f'===== summary of post-processing [{selected_truck.keys()}] route+schedule solution')
        print(f'+++ cargo to truck assignment')
        for key, value in cargo_in_truck:
            print(f'      {key}: {value}')
        print(f'+++ truck to cargo assignment')
        for key, value in trucks_per_cargo:
            print(f'      {key}: {value}')
        print(f'+++ truck route')
        for key, value in truck_route:
            print(f'      {key}: {value}')
        print(f'+++ cargo route')
        for key, value in cargo_route:
            print(f'      {key}: {value}')

    return (x_sol, s_sol, z_sol, y_sol, u_sol, D_sol), (g_sol, h_sol, D_sol), (truck_route, cargo_route), (truck_cost, travel_cost, transfer_cost)


def pdpt_rasd(dir_, verbose = 0):

    iniSol_filename = dir_ + '/toyinitSol.txt'
    pdpt_ins = read_pdpt_pickle(dir_ +'/toy.pkl', verbose = verbose-1) 

    truck_yCycle_file, truck_used_file, truck_route_file, \
    cargo_route_file, S_sol_file, A_sol_file, D_sol_file, \
    Sb_sol_file, Ab_sol_file, Db_sol_file = read_route_solution_PDPT(iniSol_filename, verbose = 0)

    initial_solution = (truck_yCycle_file, truck_used_file, truck_route_file, \
                        cargo_route_file, S_sol_file, A_sol_file, D_sol_file, \
                        Sb_sol_file, Ab_sol_file, Db_sol_file)

    subroutes = select_subroutes(pdpt_ins, cargo_route_file, verbose)

    MP_sol, SP_sol, route_sol, costs = pdpt_route_schedule_decomposition(pdpt_ins, initial_solution, subroutes)

    res = {'MP': {'x_sol': MP_sol[0],
                  'x_sol': MP_sol[1],
                  'z_sol': MP_sol[2],
                  'y_sol': MP_sol[3],
                  'u_sol': MP_sol[4],
                  'D_sol': MP_sol[5],
                 },
            'SP':{'g_sol': SP_sol[0],
                  'h_sol': SP_sol[1],
                  'D_sol': SP_sol[-1],
                 },
            'route':{'truck_route': route_sol[0],
                     'cargo_route': route_sol[-1],
                    },
            'cost':{'truck_cost', costs[0],
                    'travel_cost', costs[1],
                    'transfer_cost', costs[2],}
            }
    
    res_filename = dir_ + '/toyimprove.pkl'
    with open(res_filename, 'wb') as pickle_file:
        pickle.dump(res, pickle_file)
    
    return res

    

def example():
    return None


if __name__ == '__main__':

    example()