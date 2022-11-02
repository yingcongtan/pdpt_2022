import os, sys, random, time, pickle
src_dir_ = '/home/tan/Documents/GitHub/pdpt_2022/src'
sys.path.insert(1, src_dir_)

import numpy as np
import random
from pathlib import Path


from util import read_pickle, group_cycle_truck, manual_stop, generate_node_cargo_size_change, calculate_single_truck_deviation
# from pdpt_route_schedule import gurobi_master_cycle, greedy_fix_MP, MP_to_SP, cpo_sub, greedy_heuristic, time_checker_cluster, calculate_SP_cost, postprocess_pdpt_solution

dir_ = '/home/tan/Documents/GitHub/pdpt_2022/'

case_num = 1

# num_trucks = 5
num_ins = 10

seed=0
np.random.seed(seed)
random.seed(seed)


def generate_random_ins(dir_, case_num, num_trucks, num_ins):

    def generate_ins_based_on_truck_list(pdpt_ins, selected_truck_list, cargos_in_truck):

        edge_shortest = pdpt_ins['edge_shortest']
        coor = pdpt_ins['coordinate']

        selected_cargo_list = {}
        selected_node_list = []

        for truck_key, truck_value in selected_truck_list.items():
            t_o, t_d = truck_value[:2]
            if t_o not in selected_node_list:
                    selected_node_list.append(t_o)
            if t_d not in selected_node_list:
                selected_node_list.append(t_d)
            # print(t_o, t_d)
            # print(truck_value)
            for cargo_key in cargos_in_truck[truck_key]:
                assert cargo_key not in list(selected_cargo_list.keys())
                selected_cargo_list[cargo_key] = pdpt_ins['cargo'][cargo_key]
                # print(selected_cargo_list[cargo_key] )
                c_o, c_d = selected_cargo_list[cargo_key][3:]

                # print(c_o, c_d)
                # manual_stop()
                if  c_o not in selected_node_list:
                    selected_node_list.append(c_o)
                if c_d not in selected_node_list:
                    selected_node_list.append(c_d)


        selected_edge_list = {}
        selected_node_coor_list = {}
        for node_i in selected_node_list:
            for node_j in selected_node_list:
                selected_edge_list[(node_i, node_j)] = edge_shortest[(node_i, node_j)]
                selected_node_coor_list[node_j] = coor[node_j]

        node_cargo_size_change = generate_node_cargo_size_change(selected_node_list, selected_cargo_list)
        single_truck_deviation = calculate_single_truck_deviation(selected_truck_list, selected_cargo_list, selected_edge_list)

        selected_truck_yCycle = {}
        selected_truck_nCycle = {}
        
        for trcuk_key, truck_value in selected_truck_list.items():
            # flag = random.Random(seed).random()
            if truck_value[0] == truck_value[1]:
                # modify the truck to be a cycle truck
                selected_truck_yCycle[trcuk_key] = (truck_value[0], truck_value[0], truck_value[2], truck_value[3])
            else:
                # keep the truck to be a non-cycle truck
                selected_truck_nCycle[trcuk_key] = (truck_value[0], truck_value[1], truck_value[2], truck_value[3])

        ins_ = {'constant': pdpt_ins['constant'],
                'cargo': selected_cargo_list,
                'truck': selected_truck_list,
                'nodes': selected_node_list,
                'node_cargo_size_change': node_cargo_size_change,
                'edge_shortest': selected_edge_list,
                'single_truck_deviation': single_truck_deviation,
                'truck_yCycle': selected_truck_yCycle,
                'truck_nCycle': selected_truck_nCycle,
                'coordinate': selected_node_coor_list,
                }
        return ins_

    Path(os.path.join(dir_, f'data/case{case_num}')).mkdir(parents=True, exist_ok=True)
            
    pdpt_ins_filename = os.path.join(dir_, f'data/case{case_num}.pkl')
    pdpt_ins = read_pickle(pdpt_ins_filename)
    truck_list = pdpt_ins['truck']

    pdotw_sol_res_filename = os.path.join(dir_, 'out', 'iniSol', f'case{case_num}_iniSol.pkl')
    pdotw_sol = read_pickle(pdotw_sol_res_filename)

    y_sol = pdotw_sol['MIP']['y_sol']

    for i in range(num_ins):
        path_ = os.path.join(dir_, f'data/case{case_num}', f'case{case_num}_truck{num_trucks}_ins{i+1}.pkl')
        if not os.path.isfile(path_):
            cargos_in_truck = {}

            for truck_key in pdotw_sol['route']['used_truck']:
                # cargos_in_truck[truck_key] = [ cargo_key  for cargo_key, c_route_ in cargo_route.items() if truck_key in [r_[0] for r_ in c_route_] ] # equivalent to the line below
                cargos_in_truck[truck_key] = [ keys[-1]  for keys, values in y_sol.items() if truck_key in keys and values ==1  ]
                # assert cargos_in_truck[truck_key].sort() == cargos_in_truck_[truck_key].sort()
                # print(f'truck_key, \n   cargos_in_truck[truck_key] :{cargos_in_truck[truck_key]}\n   cargos_in_truck_[truck_key]: {cargos_in_truck_[truck_key]}')

            selected_truck_key_list = random.sample(list(cargos_in_truck.keys()), num_trucks)

            print(f'     Selected trucks for new instances {selected_truck_key_list}')
            # print(cargos_in_truck.keys())

            selected_truck_list = {truck_key: truck_list[truck_key] for truck_key in selected_truck_key_list}

            ins = generate_ins_based_on_truck_list(pdpt_ins, selected_truck_list, cargos_in_truck)


            path_ = os.path.join(dir_, f'data/case{case_num}', f'case{case_num}_truck{num_trucks}_ins{i+1}.pkl')
            with open(path_, 'wb') as pickle_file:
                pickle.dump(ins, pickle_file)
        else:
            print('+++ path_ realy exist')



def main():
    for case_num in range(1,6,1):
        for num_trucks in [2, 3, 5]:
            print(f'+++ Generate new instance using data from case {case_num} with {num_trucks} trucks')
            
            generate_random_ins(dir_, case_num, num_trucks, num_ins)


if __name__ == "__main__":
        

    main()