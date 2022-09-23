import matplotlib.pyplot as plt
import numpy as np
from math import sqrt
import os, sys

# src_dir_ = '/home/tan/Documents/GitHub/pdpt_2022/src'
# sys.path.insert(1, src_dir_)
src_dir_ = '/home/tan/Documents/GitHub/pdpt_2022'
sys.path.insert(1, src_dir_)

from src.util import generate_node_cargo_size_change, calculate_single_truck_deviation, ConsoleLogger
from src.util import read_route_solution_PDPT, read_pickle
from src.pdotw_mip import eval_pdotw_sol

from pdpt_repair import best_insert
from pdpt_ini_sol import solve_pdotw_mip
from pdpt_rasd import select_subroutes, pdpt_route_schedule_decomposition
import pickle
from pathlib import Path
from matplotlib.lines import Line2D
import matplotlib.font_manager as font_manager
from math import ceil

# Please update dir_ to the folder where you wan to save files
dir_ = '/home/tan/Documents/GitHub/pdpt_2022/toy'


def toy_example():
    num_node = 16
    constant = {'truck_fixed_cost': 30000.0, 
                'truck_running_cost': 50.0, 
                'cargo_reloading_cost': 0.5, 
                'node_fixed_time': 11.0, 
                'loading_variation_coefficient': 0.0094598
               }
    # loc = [[0.00,  0.00], #node 1
    #     [1.50, -0.50], #node 2
    #     [2.00, -0.75], #node 3
    #     [2.50, -0.75], #node 4
    #     [3.50, -0.75], #node 5
    #     [4.00, -0.50], #node 6
    #     [5.00, -0.75], #node 7
    #     [5.20, -1.75], #node 8
    #     [3.00, -0.50], #node 9
    #     [4.50,  0.00], #node 10
    #     [1.00, -1.25], #node 11
    #     [0.75, -1.75], #node 12
    #     [0.25, -2.00], #node 13
    #     [2.50, -1.75], #node 14
    #     [3.00, -2.50], #node 15
    #     [3.75, -2.00], #node 16
    #     ]

    loc = [[0.00,  0.00], #node 1
           [150,  -50], #node 2
           [200,  -75], #node 3
           [250,  -75], #node 4
           [350,  -75], #node 5
           [400,  -50], #node 6
           [500,  -75], #node 7
           [520,  -175], #node 8
           [300,  -50], #node 9
           [450,   00], #node 10
           [100, -125], #node 11
           [ 75, -175], #node 12
           [ 25, -200], #node 13
           [250, -175], #node 14
           [300, -250], #node 15
           [375, -200], #node 16
           ]
    node_list = [f'N{i+1}' for i in range(num_node)]

    # cargo['nb_cargo'] = ['size', 'lb_time', 'ub_time', 'departure_node', 'arrival_node']
    # cargo = {'C1': [10, 0, 100, '0',  '1'], #cargo 1
    #         'C2': [10, 0, 100, '10', '11'], #cargo 2
    #         'C3': [10, 0, 100,  '9', '12'], #cargo 3
    #         'C4': [10, 0, 100,  '2',  '6'], #cargo 4
    #         'C5': [10, 0, 100,  '8',  '3'], #cargo 5
    #         'C6': [10, 0, 100,  '4',  '6'], #cargo 6
    #         'C7': [10, 0, 100, '13', '15'], #cargo 7
    #         'C8': [10, 0, 100,  '2', '14'], #cargo 8
    #         }
    cargo = {'C1': [100, 0, 10000, 0,  1], #cargo 1
             'C2': [100, 0, 10000, 10, 11], #cargo 2
             'C3': [100, 0, 10000,  9, 12], #cargo 3
             'C4': [100, 0, 10000,  2,  6], #cargo 4
             'C5': [100, 0, 10000,  8,  3], #cargo 5
             'C6': [100, 0, 10000,  4,  6], #cargo 6
             'C7': [100, 0, 10000, 13, 15], #cargo 7
             'C8': [100, 0, 10000,  2, 14] #cargo 8
            }
    

    # truck['nb_truck'] = ['departure_node', 'arrival_node', 'max_worktime', 'max_capacity']

    # truck = {'T1':[ '0',  '7', 100, 100],
    #          'T2':[ '9', '12', 100, 100],
    #          'T3':[ '3', '15', 100, 100],
    #         }
    truck = {'T1':[ 0,  7, 5000, 100],
             'T2':[ 9, 12, 5000, 100],
             'T3':[ 3, 15, 5000, 100]
            }
    # edge_shortest = {(i,j): round(sqrt((loc[i][0]-loc[j][0])**2 +(loc[i][1]-loc[j][1])**2),2) for i in range(num_node) for j in range(num_node)}
    edge_shortest = {(i,j): int(sqrt((loc[i][0]-loc[j][0])**2 +(loc[i][1]-loc[j][1])**2)) for i in range(num_node) for j in range(num_node)}
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


###############
#complete test, all functions related to initial solution phase seems to work fine
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

    filename = dir_+'/toy_initSol.pkl'
    with open(filename, 'wb') as f:
        pickle.dump(res, f)



############
#Incomplete
def toy_best_insertion(dir_, verbose = 0):
    pdpt_ins_filename = dir_ + '/toy.pkl'

    pdpt_ins = read_pickle(pdpt_ins_filename, verbose = verbose-1) 

    initialSol_filename = dir_ + '/toyinitSol.txt'
    truck_yCycle_file, truck_used_file, truck_route_file, \
    cargo_route_file, S_sol_file, A_sol_file, D_sol_file, \
    Sb_sol_file, Ab_sol_file, Db_sol_file = read_route_solution_PDPT(initialSol_filename, verbose = 0)

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

    else:
        print('No undelivered cargo, skip best_insertion')


def pdpt_rasd(dir_, verbose = 0):

    pdpt_ins = read_pickle(dir_ +'/toy.pkl', verbose = verbose-1) 


    ini_sol_res_filename = dir_ + '/toy_initSol.pkl'
    ini_sol_res = read_pickle(ini_sol_res_filename)

    # truck_yCycle_file, truck_used_file, truck_route_file, \
    # cargo_route_file, S_sol_file, A_sol_file, D_sol_file, \
    # Sb_sol_file, Ab_sol_file, Db_sol_file = read_route_solution_PDPT(iniSol_filename, verbose = 0)

    # initial_solution = (truck_yCycle_file, truck_used_file, truck_route_file, \
    #                     cargo_route_file, S_sol_file, A_sol_file, D_sol_file, \
    #                     Sb_sol_file, Ab_sol_file, Db_sol_file)

    subroutes = select_subroutes(pdpt_ins, ini_sol_res['cargo_route'], verbose)

    MP_sol, SP_sol, route_sol, costs = pdpt_route_schedule_decomposition(dir_+'/toy', pdpt_ins, ini_sol_res, subroutes, verbose = 0)

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
            'cost':{'truck_cost': costs[0],
                    'travel_cost': costs[1],
                    'transfer_cost': costs[2],}
            }
    
    res_filename = dir_ + '/toyimprove.pkl'
    with open(res_filename, 'wb') as pickle_file:
        pickle.dump(res, pickle_file)
    


################################################


def toy_eval_pdpt_sol(dir_, rasd_sol_filename, verbose = 0):
    pdpt_ins = read_pickle(dir_ +'/toy.pkl', verbose = verbose-1) 
    constant = pdpt_ins['constant']
    nodes = pdpt_ins['nodes']
    edge_shortest = pdpt_ins['edge_shortest']

    ini_sol_res_filename = dir_ + '/toy_initSol.pkl'
    ini_sol_res = read_pickle(ini_sol_res_filename)

    truck_used_file = ini_sol_res['used_truck']
    truck_route_file = ini_sol_res['truck_route']

    
    with open(rasd_sol_filename, 'rb') as pickle_file:
        rasd_sol=pickle.load(pickle_file)
        
    subroute_truck_route = rasd_sol['route']['truck_route']

    trucks_in_subroute = subroute_truck_route.keys()
    truck_not_in_subroute = [truck_key for truck_key in truck_used_file 
                                    if truck_key not in trucks_in_subroute]


    truck_cost, travel_cost = \
    eval_pdotw_sol(constant, edge_shortest,
    truck_not_in_subroute, truck_route_file)

    costs_rasd = [value for _, value in rasd_sol['cost'].items()]

    final_cost = truck_cost + travel_cost + sum(costs_rasd)


    return truck_cost + costs_rasd[0], travel_cost+costs_rasd[1], costs_rasd[-1]





def plot_instance(dir_, truck_colors, cargo_colors, font):

    pdpt_ins = read_pickle(dir_ +'/toy.pkl', verbose = 0) 

    # cargo['nb_cargo'] = ['size', 'lb_time', 'ub_time', 'departure_node', 'arrival_node']
    cargo_list = pdpt_ins['cargo']
    node_coor = pdpt_ins['loc']

    fig1, ax1 = plt.subplots(1,1, figsize=(10,5))
    ax1.axis('off')
    
    cargo_idx = 0
    for cargo_key, cargo_value in cargo_list.items():
        source, dest = int(cargo_value[-2]), int(cargo_value[-1])
        ax1.plot(*node_coor[source], 'o', color = cargo_colors[cargo_idx],
                 mec='k', ms=10)
        ax1.plot(*node_coor[dest], '^', color = cargo_colors[cargo_idx],
                 mec='k', ms=10)

        cargo_idx+=1
        
    legend_elements = [Line2D([0], [0],  marker='^', color='w', label='origin',
                          markerfacecolor='None', mec='k',  markersize=10),
                   Line2D([0], [0],  marker='o', color='w', label='destination',
                          markerfacecolor='None', mec='k', markersize=10)]
        
    ax1.legend(handles=legend_elements, loc='upper center', bbox_to_anchor=(0.5, -0.05),
              fancybox=True, shadow=True, ncol=2, prop=font)


    ax1.set_title('Toy Instance', size=18, font='serif')

    return fig1, ax1

def plot_init_Sol(dir_, truck_colors, cargo_colors, font):


    pdpt_ins = read_pickle(dir_ +'/toy.pkl', verbose = 0) 
    # cargo['nb_cargo'] = ['size', 'lb_time', 'ub_time', 'departure_node', 'arrival_node']
    cargo_list = pdpt_ins['cargo']
    node_coor = pdpt_ins['loc']
    constant = pdpt_ins['constant']
    edge_shortest = pdpt_ins['edge_shortest']

    ini_Sol_res_filename = dir_ + '/toy_initSol.pkl'

    ini_sol_res = read_pickle(ini_Sol_res_filename)

    truck_route = ini_sol_res['truck_route']
    truck_used = ini_sol_res['used_truck']


    truck_cost, travel_cost = eval_pdotw_sol(constant, edge_shortest,truck_used, truck_route)

    total_cost = truck_cost+travel_cost
    title = f'Init. Sol. [{truck_cost}+{travel_cost}+0]'

    fig1, ax1 = plt.subplots(1,1, figsize=(10,5))
    ax1.axis('off')
    
    cargo_idx = 0
    for cargo_key, cargo_value in cargo_list.items():
        source, dest = int(cargo_value[-2]), int(cargo_value[-1])
        ax1.plot(*node_coor[source], 'o', color = cargo_colors[cargo_idx],
                 mec='k', ms=10)
        ax1.plot(*node_coor[dest], '^', color = cargo_colors[cargo_idx],
                 mec='k', ms=10)

        cargo_idx+=1
        
    truck_idx = 0
    for truck_key, route_ in truck_route.items():
        for i in range(len(route_)-1):
            node_curr, node_next = int(route_[i]), int(route_[i+1])
            x0, y0 = node_coor[node_curr]
            dx, dy = (np.array(node_coor[node_next])-np.array(node_coor[node_curr]))
            ax1.arrow(x0, y0, dx, dy, color=truck_colors[truck_idx], head_width=.08,
                      length_includes_head = True, linewidth=2, linestyle=':',
                      alpha=0.3+truck_idx*0.1)
        truck_idx += 1
        
    legend_elements = [Line2D([0], [0],  marker='^', color='w', label='origin',
                          markerfacecolor='None', mec='k',  markersize=10),
                       Line2D([0], [0],  marker='o', color='w', label='destination',
                          markerfacecolor='None', mec='k', markersize=10)]
    
    for i in range(len(truck_colors)):
        legend_elements.append(Line2D([0], [0], color=truck_colors[i], 
                             lw=4, alpha=0.3+truck_idx*0.1, label=f'Truck {i+1}'))

    ax1.legend(handles=legend_elements, loc='upper center', bbox_to_anchor=(0.5, -0.05),
              fancybox=True, shadow=True, ncol=5, prop = font)
            

    
    return fig1, ax1, title

def plot_rasd_sol(dir_, truck_colors, cargo_colors, font):

    ini_sol_res_filename = dir_ + '/toy_initSol.pkl'
    ini_sol_res = read_pickle(ini_sol_res_filename)
    truck_route_file = ini_sol_res['truck_route']

    pdpt_ins = read_pickle(dir_ +'/toy.pkl', verbose = 0) 

    # cargo['nb_cargo'] = ['size', 'lb_time', 'ub_time', 'departure_node', 'arrival_node']
    cargo_list = pdpt_ins['cargo']
    node_coor = pdpt_ins['loc']

    
    filename = dir_ + '/toyimprove.pkl'
    with open(filename, 'rb') as pickle_file:
        rasd_sol=pickle.load(pickle_file)
    
    subroute_truck_route = rasd_sol['route']['truck_route']
    trucks_in_subroute = subroute_truck_route.keys()
    
    fig1, ax1 = plt.subplots(1,1, figsize=(10,5))
    ax1.axis('off')
    
    cargo_idx = 0
    for cargo_key, cargo_value in cargo_list.items():
        source, dest = int(cargo_value[-2]), int(cargo_value[-1])
        ax1.plot(*node_coor[source], 'o', color = cargo_colors[cargo_idx],
                 mec='k', ms=10, label='_nolegend_')
        ax1.plot(*node_coor[dest], '^', color = cargo_colors[cargo_idx],
                 mec='k', ms=10, label='_nolegend_')

        cargo_idx+=1

    truck_idx = 0
    for truck_key, route_ in truck_route_file.items():
        if truck_key in trucks_in_subroute:
            route_ = subroute_truck_route[truck_key]
            for i in range(len(route_)-1):
                node_curr, node_next = int(route_[i]), int(route_[i+1])
                x0, y0 = node_coor[node_curr]
                dx, dy = (np.array(node_coor[node_next])-np.array(node_coor[node_curr]))
                ax1.arrow(x0, y0, dx, dy, color=truck_colors[truck_idx], head_width=.08,
                          length_includes_head = True, linewidth=2, linestyle=':',
                          alpha=0.3+truck_idx*0.1)
            truck_idx += 1
        elif truck_key not in trucks_in_subroute:
            for i in range(len(route_)-1):
                node_curr, node_next = int(route_[i]), int(route_[i+1])
                x0, y0 = node_coor[node_curr]
                dx, dy = (np.array(node_coor[node_next])-np.array(node_coor[node_curr]))
                ax1.arrow(x0, y0, dx, dy, color=truck_colors[truck_idx], head_width=.08,
                          length_includes_head = True, linewidth=2, linestyle=':',
                          alpha=0.3+truck_idx*0.1)
            truck_idx += 1
            
    rasd_sol_filename = dir_ + '/toyimprove.pkl'
    truck_cost, travel_cost, transfer_cost = toy_eval_pdpt_sol(dir_, rasd_sol_filename)
    title = f'Sol. 1 iter RASD [{truck_cost}+{travel_cost}+{transfer_cost}]'

    legend_elements = [Line2D([0], [0],  marker='^', color='w', label='origin',
                          markerfacecolor='None', mec='k',  markersize=10),
                       Line2D([0], [0],  marker='o', color='w', label='destination',
                          markerfacecolor='None', mec='k', markersize=10)]
    
    for i in range(len(truck_colors)):
        legend_elements.append(Line2D([0], [0], color=truck_colors[i], 
                             lw=4, alpha=0.3+truck_idx*0.1, label=f'Truck {i+1}'))

    ax1.legend(handles=legend_elements, loc='upper center', bbox_to_anchor=(0.5, -0.05),
              fancybox=True, shadow=True, ncol=5, prop = font)

    
    return fig1, ax1, title

def main():

    truck_colors = ['b', 'r', 'g']
    cargo_colors = ['#8c510a', '#bf812d', '#dfc27d', '#f6e8c3',
                    '#f5f5f5','#c7eae5','#80cdc1','#35978f','#01665e']
    legend_font = font_manager.FontProperties(family='serif',style='normal', size=12)
    
    ins = toy_example()

    fig1, ax1 = plot_instance(dir_, None, cargo_colors, legend_font)
    fig1.savefig(dir_+'/toy.png', dpi=150, transparent=True)

    print('=========== Construct IniSol')
    toy_ini_sol(dir_, greedy_initialization = False, verbose = 0) 


    fig2, ax2, title_2 = plot_init_Sol(dir_, truck_colors, cargo_colors, legend_font)
    ax2.set_title(title_2, size=18, font='serif')
    fig2.savefig(dir_+'/toy_initSol.png', dpi=150, transparent=True)

    pdpt_rasd(dir_)

    fig3, ax3, title_3 = plot_rasd_sol(dir_, truck_colors, cargo_colors, legend_font)
    ax3.set_title(title_3, size=18, font='serif')

    fig3.savefig(dir_+'/toy_rasdSol.png', dpi=150, transparent=True)


if __name__ ==  "__main__":
    main()