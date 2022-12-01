import os, sys, random, time, pickle
src_dir_ = '/home/tan/Documents/GitHub/pdpt_2022/src'
sys.path.insert(1, src_dir_)
import statistics

import numpy as np
from util import generate_node_cargo_size_change, read_pickle, group_cycle_truck, manual_stop, ConsoleLogger
from pathlib import Path
from tvopdpt import tvopdpt_milp_gurobi
dir_ = '/home/tan/Documents/GitHub/pdpt_2022/'
num_ins = 10

def find_key(list_dict, value):
    return  [k for item_ in list_dict for k, v in item_.items() if value in v]

def solve_tvopdpt_mip(ins,  # dict contains the data of pdpt instance,
                     gurobi_log_file, # file where all data of pdotw solutions are saved
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
    print(f'   +++ with [{len(selected_truck.keys())}] Truck {list(selected_truck.keys())}  ')

    print(f'   +++ with [{len(selected_cargo.keys())}] cargo {list(selected_cargo.keys())}  ')
    print(f'   +++ with [{len(selected_node)}] cargo {selected_node}  ')

    # edge_shortest, path_shortest = replace_edge_by_shortest_length_nx(nodes, edges)
    # single_truck_deviation = calculate_single_truck_deviation(truck, cargo, edge_shortest)
    
    start_time = time.time()

    
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

    print(f'=== Result: total cargo [{len(selected_cargo.keys())}], MIP obj [{obj_val_MP}]')
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

# destory the solution by removing the route with the least number of cargo
def destory_sol(cargo_undelivered, 
                cargo_in_truck, 
                truck_key_list_cargo_sorted):
    remove_truck_key = truck_key_list_cargo_sorted[0,0]
    truck_key_list_cargo_sorted = truck_key_list_cargo_sorted[1:,:]
    cargo_undelivered.extend(cargo_in_truck[remove_truck_key])

    print(f'   +++ remove truck [{remove_truck_key}]')
    print(f'   +++ Thus, there are now [{len(cargo_undelivered)}] undelivered_cargo {cargo_undelivered}')


    return cargo_undelivered, truck_key_list_cargo_sorted

def rank_trucks(cargo_undelivered,
                single_truck_deviation,
                truck_key_list_cargo_sorted):
    truck_key_list_deviation= np.array([[truck_key, sum([ single_truck_deviation[(cargo_key,truck_key)] for cargo_key in cargo_undelivered])] for truck_key in truck_key_list_cargo_sorted[:,0]], dtype=object)

    truck_key_list_deviation_sorted = truck_key_list_deviation[truck_key_list_deviation[:, 1].argsort()]  # sort by number of cargo

    truck_list_score = {}
    i = 0
    for truck_key in truck_key_list_deviation_sorted[:,0]:
        truck_list_score[truck_key] = [np.where(truck_key_list_cargo_sorted[:,0] == truck_key)[0][0], np.where(truck_key_list_deviation_sorted[:,0] == truck_key)[0][0]]

    truck_score = np.array([[truck_key, sum(scores)] for truck_key, scores in truck_list_score.items()], dtype=object)
    truck_score_sorted = truck_score[truck_score[:, 1].argsort()]  # sort by number of cargo

    # return truck_score_sorted

    return truck_key_list_cargo_sorted


def prepare_subproblem(pdpt_ins,
                       cargo_in_truck,
                       cargo_undelivered,
                       truck_score_sorted,
                       truck_pairs_tried,
                       verbose = 0): 
    
    
    truck_list = pdpt_ins['truck']
    cargo_list = pdpt_ins['cargo']
    node_list = pdpt_ins['nodes']
    edge_list = pdpt_ins['edge_shortest']    
    constant = pdpt_ins['constant']
    single_truck_deviation = pdpt_ins['single_truck_deviation']

    selected_cargo = {}
    selected_truck = {}
    selected_edge = {}
    selected_node = []

    for cargo_key in cargo_undelivered:
        selected_cargo[cargo_key] = cargo_list[cargo_key]


    
    num_candidate_trucks = int(len(truck_score_sorted) /2)
    flag = False
    for truck_1 in truck_score_sorted[:num_candidate_trucks,0]:
        for truck_2 in truck_score_sorted[:num_candidate_trucks,0]:
            if truck_1 != truck_2 and [truck_1, truck_2] not in truck_pairs_tried:
                selected_truck[truck_1] = truck_list[truck_1]
                selected_truck[truck_2] = truck_list[truck_2]
                flag = True
                print(f'   +++ select truck [{truck_1}], truck [{truck_2}] to instantiate sub-problem')
                print(f'   +++ cargos assigned to [{truck_1}] in the current sol {cargo_in_truck[truck_1]}')
                print(f'   +++ cargos assigned to [{truck_2}] in the current sol {cargo_in_truck[truck_2]}')

            if flag == True:
                break
        if flag == True:
            break        

    if flag == True: # find a good pair of trucks 



        for truck_key in selected_truck.keys():
            for cargo_key in cargo_in_truck[truck_key]:
                selected_cargo[cargo_key] = cargo_list[cargo_key]
            if selected_truck[truck_key][0] not in selected_node:
                selected_node.append(selected_truck[truck_key][0])
            if selected_truck[truck_key][1] not in selected_node:
                selected_node.append(selected_truck[truck_key][1])
        for v in selected_cargo.values():
            if v[3] not in selected_node:
                selected_node.append(v[3])
            if v[4] not in selected_node:
                selected_node.append(v[4])
        edges_ = list(set([(i,j) for i in selected_node for j in selected_node]))

        for i,j in edges_:
            selected_edge[(i,j)] = edge_list[(i,j)]


        # print('selected_truck',  selected_truck)
        # print('selected_cargo',  selected_cargo)
        # print('selected_node',  selected_node)
        # print('selected_edge',  selected_edge)
        pdpt_subproblem = pdpt_ins.copy()

        pdpt_subproblem['truck'] = selected_truck
        pdpt_subproblem['cargo'] = selected_cargo
        pdpt_subproblem['nodes'] = selected_node
        pdpt_subproblem['edge_shortest']  = selected_edge  
    else:
        pdpt_subproblem = None


    return pdpt_subproblem


def lns_tvopdpt(case_num, max_iter = 20, max_runtime = 100):
    # truck_pairs_tried = [['T7', 'T19']]
    truck_pairs_tried = []

    pdpt_ins_filename = os.path.join(dir_, f'data/case{case_num}.pkl')
    pdpt_ini_sol_res_filename = os.path.join(dir_, 'out', 'iniSol', f'case{case_num}_iniSol.pkl')

    pdpt_ins = read_pickle(pdpt_ins_filename)
    truck_list = pdpt_ins['truck']

    pdotw_sol = read_pickle(pdpt_ini_sol_res_filename)
    cargo_route = pdotw_sol['route']['cargo_route']
    cargo_undelivered = [cargo_key for cargo_key, cargo_route_ in cargo_route.items() if len(cargo_route_) <1]


    cargo_in_truck = {}
    for truck_key in truck_list.keys():
        cargo_in_truck[truck_key] = [ cargo_key  for cargo_key, c_route_ in cargo_route.items() if truck_key in [r_[0] for r_ in c_route_] ]
        print(truck_key, cargo_in_truck[truck_key])

    truck_key_list_cargo = np.array([[truck_key, len(cargo_list)] for truck_key, cargo_list in cargo_in_truck.items() if len(cargo_list)>0], dtype=object)

    truck_key_list_cargo_sorted = truck_key_list_cargo[truck_key_list_cargo[:, 1].argsort()]  # sort by number of cargo

    cargo_undelivered, truck_key_list_cargo_sorted = destory_sol(cargo_undelivered, 
                cargo_in_truck, 
                truck_key_list_cargo_sorted)

    truck_score_sorted = rank_trucks(cargo_undelivered,
                                    pdpt_ins['single_truck_deviation'],
                                    truck_key_list_cargo_sorted)



    path_ = os.path.join(dir_, 'out','impSol_tvopdpt')
    Path(path_).mkdir(parents=True, exist_ok=True)

    rt_start = time.time()
    iter = 0
    flag = False
    while flag != True:
        print(f'[iter {iter+1}] truck_pairs_tried {truck_pairs_tried}')

        print(f'\n   ========= START solving TVOPDPT ========= ')
        subproblem_ins = prepare_subproblem(pdpt_ins,
                                            cargo_in_truck,
                                            cargo_undelivered,
                                            truck_score_sorted,
                                            truck_pairs_tried)
        if subproblem_ins is None:
            print('   +++ Cant find good subproblems, terminate')
            break
        else:
            truck_pairs_tried.append(list(subproblem_ins['truck'].keys()))

        
        # manual_stop()

        gurobi_log_file = os.path.join(dir_, 'out','impSol_tvopdpt', f'{case_num}_sp{iter+1}_gurobi.log')

        res = solve_tvopdpt_mip(subproblem_ins,  # dict contains the data of pdpt instance,
                                gurobi_log_file, # file where all data of pdotw solutions are saved
                                max_runtime = max_runtime,
                                verbose = 0)
        if time.time() - rt_start > 20*60:
            print('++++ reach time_limit, terminate')

            flag = True
            break
        if len(subproblem_ins['cargo'].keys()) == res['obj_val_MP']:
            print('++++ all cargo delivered, terminate')
            flag = True

            tvopdpt_res_filename = os.path.join(dir_, 'out','impSol_tvopdpt', f'case{case_num}_sp{iter+1}_res.pkl')
            tvopdpt_sp_filename = os.path.join(dir_, 'out','impSol_tvopdpt', f'case{case_num}_sp{iter+1}.pkl')

            with open(tvopdpt_res_filename, 'wb') as f:
                pickle.dump(res, f)
            with open(tvopdpt_sp_filename, 'wb') as f:
                pickle.dump(subproblem_ins, f)

        iter +=1
        if iter >= max_iter:
            print('++++ reach max_iter, terminate')
            flag = True
        # manual_stop()




def main():

    Path(os.path.join(dir_, 'out', 'impSol_tvopdpt')).mkdir(parents=True, exist_ok=True)
    logfile = os.path.join(dir_, 'out', 'pdpt_exp.log')

    ConsoleLogger.start(logfile, mode='w', time_format='[%m-%d %H:%M:%S]')

    for case_num in range(1,6,1):
    # for case_num in [3]:
        print(f'\n START LNS_TVOPDPT on case {case_num}========================')
        with ConsoleLogger.copy_to(os.path.join(dir_, 'out','impSol_tvopdpt', f'case{case_num}.log')):

            lns_tvopdpt(case_num)



    ConsoleLogger.stop()

if __name__ == "__main__":
        

    main()