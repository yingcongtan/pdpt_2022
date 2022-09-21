import pickle

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