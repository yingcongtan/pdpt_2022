import csv, os, sys
from pathlib import Path
import pickle
import threading
import datetime
import networkx as nx


# DATA_DIR = '/home/tan/Documents/PDPT_src/data'

def print_dict(dict):
    return  "\n".join("\t{}\t{}".format(k, v) for k, v in dict.items())


###################################################
# functions to read data from raw csv files
def read_constant(dir):
    """
    Read the Constant.csv
    return a dictionary: constant
    e.g., 
    constant['truck_fixed_cost'] = 30000
    constant['truck_running_cost'] = 50
    constant['cargo_reloading_cost'] = 0.5
    constant['node_fixed_time'] = 11
    constant['loading_variation_coefficient'] = 0.00946
    """
    constant = {}
    file_name = dir+"/Constant.csv"
    csv_reader = csv.reader(open(file_name))
    for line in csv_reader:
        if line[1] != 'value':
            constant[line[0]] = float(line[1])
    
    return constant

def read_cargo_model(dir):
    """
    Read the Cargo_model.csv
    return a dict<list>: cargo
    e.g., 
    cargo['nb_cargo'] = ['size', 'lb_time', 'ub_time',
                         'departure_node', 'arrival_node']
    cargo['c1'] = [915, 125, 762, 'N7', 'N22']
    """
    cargo = {}
    file_name = dir+"/Cargo_model.csv"
    csv_reader = csv.reader(open(file_name))
    # csv_reader = csv.reader(open("./Cargo_model.csv"))
    for line in csv_reader:
        if line[0] != 'nb_cargo':
            cargo[line[0]] = (int(line[1]), int(line[2]), 
                              int(line[3]), line[4], line[5])
            
    return cargo

def read_daiya_model(dir):
    """
    Read the Daiya_model.csv
    The file stores all information of trucks
    return a dict<list>: truck
    e.g.,
    truck['nb_truck'] = ['departure_node', 'arrival_node', 'max_worktime', 'max_capacity']
    truck[T1] = ['N7', 'N30', 780, 10000]
    """
    truck = {}
    # csv_reader = csv.reader(open("./Daiya_model.csv"))
    file_name = dir+"/Daiya_model.csv"
    csv_reader = csv.reader(open(file_name))
    for line in csv_reader:
        if line[0] != 'nb_truck':
            truck[line[0]] = (line[1], line[2], int(line[3]), int(line[4]))
            
    return truck

def read_edge_list(dir):
    """
    Read the Edge_list.csv
    return a dict<int>: edge
    Symmetrical
    e.g., 
    edge[('start_node', 'end_node')] = runtime
    edge[('N1', 'N2')] = 70
    edge[('N2', 'N1')] = 70
    Also, return a list of nodes:
        node = ['N1', 'N2', ...]
    """
    edge = {}
    node_list = []
    # csv_reader = csv.reader(open("./Edge_list.csv"))
    file_name = dir+"/Edge_list.csv"
    csv_reader = csv.reader(open(file_name))
    for line in csv_reader:
        if line[0] != 'start_node':
            if line[0] not in node_list:
                node_list.append(line[0])
            if line[1] not in node_list:
                node_list.append(line[1])
            edge[(line[0], line[1])] = int(line[2])
            # create 0 distance edge
            edge[(line[0], line[0])] = 0
    
    return node_list, edge

def read_case(dir):
    """
    Read all(4) files in a Case
    return a tuple (constant, cargo, truck, node_list, edge)
    """
    
    constant = read_constant(dir)
    cargo = read_cargo_model(dir)
    truck = read_daiya_model(dir)
    node_list, edge = read_edge_list(dir)
    
    return (constant, cargo, truck, node_list, edge)

def generate_node_cargo_size_change(node_list, cargo):
    """
    Generate node_cargo_size_change,
    which is very useful in the PDPTW or PDOTW model.

    Inputs:
        node_list = [N1, N2, ...]
        cargo[c] = (size, lb, ub, origin, destination)
    Outputs:
        node_cargo_size_change[(n, c)] =  0 if n is not oc / dc
        node_cargo_size_change[(n, c)] =  cargo_size if n is oc 
        node_cargo_size_change[(n, c)] = -cargo_size if n is dc
    """
    
    node_cargo_size_change = {}
    for n in node_list:
        for c in cargo.keys():
            if n == cargo[c][3]:
                node_cargo_size_change[(n, c)] = cargo[c][0]
            elif n == cargo[c][4]:
                node_cargo_size_change[(n, c)] = -cargo[c][0]
            else:
                node_cargo_size_change[(n, c)] = 0
    
    return node_cargo_size_change

###################################################


def calculate_truck_travel_cost(constant, edge_shortest,
    truck_used_file, truck_route_file):
    """ 
    Use truck_used_file to calculate truck_cost
    Use truck_route_file to calculate travel_cost
    """

    # For truck_cost
    truck_cost = len(truck_used_file) * constant['truck_fixed_cost']

    # For travel_cost
    travel_cost = 0
    for t in truck_used_file:
        route = truck_route_file[t]
        for i in range(len(route)-1):
            n1 = route[i]
            n2 = route[i+1]
            travel_cost += edge_shortest[(n1, n2)] * \
                constant['truck_running_cost']
    
    print('\tThe truck_cost based on the PDPT solution:', truck_cost)
    print('\tThe travel cost based on the PDPT solution:', travel_cost)
    print('\tThe total cost:', truck_cost + travel_cost)

    return truck_cost, travel_cost


#####################################################
# functions to process pdpt data

def store_route_solution_PDPT(filename, cargo, created_truck_yCycle_total, 
    created_truck_nCycle_total, truck, 
    truck_used, truck_route, cargo_route, x_sol_total, y_sol_total, 
    S_sol_total, D_sol_total, A_sol_total, 
    Sb_sol_total, Db_sol_total, Ab_sol_total):

    """
    Store solutions of PDPT to .txt
    Format:
    all cycle trucks
    T1 n1 S A D n2 S A D ...
    ...
    C1 T1 n1 T1 n2 ...
    ...
    """

    # filename = 'route_solution_PDPT.txt'
    
    with open(filename,'a') as f:

        ### First record all cycle trucks
        for t in truck.keys():
            if t in created_truck_yCycle_total.keys():
                f.write(str(t) + ' ')
        f.write('\n') 
        
        ### Second store truck route: (n, S, A, D)
        for t in truck.keys():
            # if the truck is used
            if t in truck_used:
                f.write(str(t) + ' ')
                # record the truck route
                for i in range(len(truck_route[t])):
                    n = truck_route[t][i]
                    if i == len(truck_route[t]) - 1:
                        if t in created_truck_yCycle_total.keys():
                            f.write(str(n) + ' ')
                            f.write(str(Sb_sol_total[(n, t)]) + ' ')
                            f.write(str(Ab_sol_total[(n, t)]) + ' ')
                            f.write(str(Db_sol_total[(n, t)]) + ' ')
                        else:
                            f.write(str(n) + ' ')
                            f.write(str(S_sol_total[(n, t)]) + ' ')
                            f.write(str(A_sol_total[(n, t)]) + ' ')
                            f.write(str(D_sol_total[(n, t)]) + ' ')
                    else:
                        f.write(str(n) + ' ')
                        f.write(str(S_sol_total[(n, t)]) + ' ')
                        f.write(str(A_sol_total[(n, t)]) + ' ')
                        f.write(str(D_sol_total[(n, t)]) + ' ')
                f.write('\n') 
            # if the truck is not used
            else:
                f.write(str(t) + ' ')
                # keep the truck route empty
                f.write('\n') 

        ### ADD A SEPARATOR
        f.write('...\n')

        ### Third store cargo route: (t, n)
        for c in cargo.keys():
            f.write(str(c) + ' ')
            # record the cargo route (implicitly include transfers)
            for pair in cargo_route[c]:
                f.write(str(pair[0]) + ' ')
                f.write(str(pair[1]) + ' ')
            f.write('\n') 
    
def read_route_solution_PDPT(filename, verbose = 0):

    """
    Read the file 'route_solution_PDPT.txt'
    Produce:
        truck_yCycle_file = []
        truck_used_file = []
        truck_route_file = {}
        cargo_route_file = {}
        S_sol_file = {}
        A_sol_file = {}
        D_sol_file = {}
        Sb_sol_file = {}
        Ab_sol_file = {}
        Db_sol_file = {}
    """

    # filename = 'route_solution_PDPT.txt'   
    with open(filename) as f: 

        # truck_yCycle_file
        truck_yCycle_file = []
        line = f.readline()
        arr = line.split()
        for t in arr:
            truck_yCycle_file.append(t)

        # truck_used_file
        # truck_route_file
        # all other sol
        truck_used_file = []
        truck_route_file = {}
        S_sol_file = {}
        A_sol_file = {}
        D_sol_file = {}
        Sb_sol_file = {}
        Ab_sol_file = {}
        Db_sol_file = {}

        line = f.readline()
        while line and line != '...\n':
            arr = line.split()
            t = arr[0]
            if len(arr) > 1:
                truck_used_file.append(t)
            truck_route_file[t] = []
            for i in range(1, len(arr)):
                if i % 4 == 1:
                    n = arr[i]
                    truck_route_file[t].append(n)
                if i % 4 == 2:
                    if (n,t) not in S_sol_file.keys():
                        S_sol_file[(n,t)] = int(arr[i])
                    else:
                        Sb_sol_file[(n,t)] = int(arr[i])
                if i % 4 == 3:
                    if (n,t) not in A_sol_file.keys():
                        A_sol_file[(n,t)] = int(arr[i])
                    else:
                        Ab_sol_file[(n,t)] = int(arr[i])
                if i % 4 == 0:
                    if (n,t) not in D_sol_file.keys():
                        D_sol_file[(n,t)] = int(arr[i])
                    else:
                        Db_sol_file[(n,t)] = int(arr[i])
            line = f.readline()
        
        # cargo_route_file
        cargo_route_file = {}

        line = f.readline()
        while line:
            arr = line.split()
            c = arr[0]
            cargo_route_file[c] = []
            for i in range(1, len(arr)):
                if i % 2 == 1:
                    t = arr[i]
                if i % 2 == 0:
                    n = arr[i]
                    cargo_route_file[c].append((t,n))
            line = f.readline()

    if verbose > 0:
        print(f'====== Summary of {filename}')
        print(f'The truck_yCycle_file, size: {len(truck_yCycle_file)}')
        print(truck_yCycle_file)
        print(f'The truck_used_file, size: {len(truck_used_file)}')
        print(truck_used_file)
        print(f'The truck_route_file, size: {len(truck_route_file)}')
        for t, v in truck_route_file.items():
            print(t, v)
        print(f'The cargo_route_file, size: {len(cargo_route_file)}')
        for c, v in cargo_route_file.items():
            print(c, v)
        print(f'The S_sol_file, size: {len(S_sol_file)}')
        for k, v in S_sol_file.items():
            print(k, v)
        print(f'The A_sol_file, size: {len(A_sol_file)}')
        for k, v in A_sol_file.items():
            print(k, v)
        print(f'The D_sol_file, size: {len(D_sol_file)}')
        for k, v in D_sol_file.items():
            print(k, v)
        print(f'The Sb_sol_file, size: {len(Sb_sol_file)}')
        for k, v in Sb_sol_file.items():
            print(k, v)
        print(f'The Ab_sol_file, size: {len(Ab_sol_file)}')
        for k, v in Ab_sol_file.items():
            print(k, v)
        print(f'The Db_sol_file, size: {len(Db_sol_file)}')
        for k, v in Db_sol_file.items():
            print(k, v)

    return truck_yCycle_file, truck_used_file, truck_route_file, \
           cargo_route_file, S_sol_file, A_sol_file, D_sol_file, \
           Sb_sol_file, Ab_sol_file, Db_sol_file

def group_cycle_truck(created_truck):
    """
    Group trucks whose origin and destination are the same
    from the created_truck
    Inputs:
        created_truck[T1] = (origin, destination, size, worktime)
    Outputs:
        created_truck_yCycle: same origin and destination
        created_truck_nCycle: different origin and destination
        created_truck_all = created_truck_yCycle + created_truck_nCycle
    """
    
    created_truck_yCycle = {}
    created_truck_nCycle = {}
    created_truck_all = {}
    
    for key, value in created_truck.items():
        # flag = random.Random(seed).random()
        if value[0] == value[1]:
            # modify the truck to be a cycle truck
            created_truck_yCycle[key] = \
            (value[0], value[0], value[2], value[3])
        else:
            # keep the truck to be a non-cycle truck
            created_truck_nCycle[key] = \
            (value[0], value[1], value[2], value[3])
    
    for key, value in created_truck_yCycle.items():
        created_truck_all[key] = value
    for key, value in created_truck_nCycle.items():
        created_truck_all[key] = value
    
    
    return created_truck_yCycle, created_truck_nCycle, created_truck_all

def replace_edge_by_shortest_length_nx(node_list, edge, verbose = 0):
    
    """
    This function is IMPORTANT!
    
    Since the triangle inequality may be violated,
    we replace the edge length of all pair nodes by
    the shortest path length between them
    by using networkX in a directed graph
    Function used in nx: all_pairs_dijkstra_path_length
    
    Inputs:
        node_list
        edge
    Outputs:
        edge_shortest[(N1, N2)] = shortest path length between N1 and N2
        path_shortest[(N1, N2)] = [N1, N3, N5, N2]
    """

    edge_shortest = {}
    path_shortest = {}

    
    G = nx.DiGraph()
    for i in node_list:
        G.add_node(i)
    
    for key, value in edge.items():
        G.add_edge(key[0], key[1], weight=value)
    
    # print('All edges in G:', G.edges.data())
    
    # use all_pairs_dijkstra to get all shortest path with length
    path = dict(nx.all_pairs_dijkstra_path(G))
    length = dict(nx.all_pairs_dijkstra_path_length(G))
    
    # print all node pairs that violate triangle inequality
    if verbose > 0:
        print('The node pairs that violate triangle inequality:')
        for i in length.keys():
            for j in length[i].keys():
                if len(path[i][j]) > 2:
                    print('Source:', i, 'Target:', j, 
                        'Path:', path[i][j], 'Length', length[i][j])

    for i in node_list:
        for j in node_list:
            edge_shortest[(i,j)] = length[i][j]
            path_shortest[(i,j)] = length[i][j]
            
            
    return edge_shortest, path_shortest

def calculate_single_truck_deviation(truck, cargo, edge_shortest, verbose = 0):
    """
    Calculate single-truck deviation for cargo
    
    single-truck deviation:
    if the truck picks up and delivers a cargo,
    the extra distance it needs to go through
    compared to not carrying the cargo
    """
    
    single_truck_deviation = {}
    
    # cargo['nb_cargo'] = ['size', 'lb_time', 'ub_time','departure_node', 'arrival_node']
    # truck['nb_truck'] = ['departure_node', 'arrival_node', 'max_worktime', 'max_capacity']
    for c_key,c_value in cargo.items():     
        for t_key,t_value in truck.items():
            deviation = \
            edge_shortest[(t_value[0], c_value[3])] + \
            edge_shortest[(c_value[3], c_value[4])] + \
            edge_shortest[(c_value[4], t_value[1])] - \
            edge_shortest[(t_value[0], t_value[1])]
            single_truck_deviation[(c_key,t_key)] = deviation
            if verbose > 0:
                print(f'   {c_key, t_key, deviation}')
    
    
    return single_truck_deviation


###################################################
# Functions to save and read raw data as a dict in pickle file
class PDPT_INS():
    def __init__(self,path):
        self.path = path
        self.constant = None
        self.cargo = None
        self.truck = None
        self.node_list = None
        self.edge = None
        self.node_cargo_size_change = None

    def read_raw_data(self):
        _ = read_constant(self.path)
        (constant, cargo, truck, node_list, edge) = read_case(self.path)
        node_cargo_size_change = generate_node_cargo_size_change(node_list, cargo)
        self.constant = constant
        self.cargo = cargo
        self.truck = truck
        self.nodes = node_list
        self.edges = edge
        self.node_cargo_size_change = node_cargo_size_change

def read_pdpt_csv_to_pickle(case_num, dir, verbose = 0):
    path = dir+'/raw/case' + str(case_num)

    def read_pdpt_csv(path, verbose = 0):
        ins = PDPT_INS(path)
        ins.read_raw_data()
        if verbose > 0:
            print(f'+++ Constant:\n{print_dict(ins.constant)}')
            print(f'+++ Cargo:\n{print_dict(ins.cargo)}')
            print(f'+++ Truck:\n{print_dict(ins.truck)}')
            print(f'+++ Nodes:\n{ins.nodes}')
            print(f'+++ Edges:\n{print_dict(ins.edges)}')
            print(f'+++ node_cargo_size_change:\n{print_dict(ins.node_cargo_size_change)}')

        return ins
    


    pdpt_ins = read_pdpt_csv(path, verbose)
    edge_shortest, path_shortest = replace_edge_by_shortest_length_nx(pdpt_ins.nodes, pdpt_ins.edges)
    single_truck_deviation = calculate_single_truck_deviation(pdpt_ins.truck, pdpt_ins.cargo, edge_shortest)
    dict_ = {'constant': pdpt_ins.constant,
             'cargo': pdpt_ins.cargo,
             'truck': pdpt_ins.truck,
             'nodes': pdpt_ins.nodes,
             'edges': pdpt_ins.edges,
             'node_cargo_size_change': pdpt_ins.node_cargo_size_change,
             'edge_shortest': edge_shortest,
             'path_shortest': path_shortest,
             'single_truck_deviation': single_truck_deviation,
            }
    path_ = dir+'/case' + str(case_num) +'.pkl'
    with open(path_, 'wb') as pickle_file:
        pickle.dump(dict_, pickle_file)

def read_pdpt_pickle(case_num, dir, verbose = 0):
    path_ = dir+'/case' + str(case_num) +'.pkl'

    with open(path_, 'rb') as pkl_file:
            ins = pickle.load(pkl_file)

    if verbose > 0:
        for key, value in ins.items():
            if isinstance(value, dict):
                print(f'+++ {key}:\n{print_dict(value)}')
            else:
                print(f'+++ {key}:\n{value}')

    return ins
###################################################


# The following function is adopted from ALMC project (Tan, Delong, Terekhove, Contreras, 2022)
class ConsoleLogger:
    """
    ConsoleLogger will capture anything written or printed to stdout and stderr,
    and direct a copy of that output to a file:
       >>> ConsoleLogger.start('test.log')
       >>> print("Hello World!")     # Written to console and test.log
       Hello World!
    Only create one instance of ConsoleLogger, usually at the start of a script.
    If a datetime-compatible time_format is specified, each printed line
    is prepended with a timestamp:
       >>> ConsoleLogger.start('test.log', time_format="[%m-%d %H:%M:%S]")
       >>> print("Hellow World!")     # Written to console and test.log
       [01-31 07:32:05] Hello World!
    An extra copy of portion of output can be logged to an extra file by using
    the echo_to method as a context manager:
       >>> with ConsoleLogger.copy_to('extra.log'):
       ...    print("Goodbye!")       # Written to console, test.log, and extra.log
       Goodbye!
    
    You can stop logging as well:
       >>> ConsoleLogger.stop()
    """
    _lock = threading.Lock()
    _logger = None

    class _Logger:
        def __init__(self, logfile, mode='w', time_format=None):
            self.streams = {
                'stdout': sys.stdout,
                'logfile': open(logfile, mode),
            }        
            self.time_format = time_format
            self.last_byte = None

        def write(self, s, /):
            with ConsoleLogger._lock:
                # If the first line written, or immediately follows a previously-written 
                # newline character, then prepend the time string before writing.
                if self.time_format:
                    if self.last_byte in (None, '\n', '\r'):
                        s = f"{datetime.datetime.now().strftime(self.time_format)} {s}"
                    self.last_byte = s[-1:]

                # Write the string to each output stream (console, logfile, possible extra logfile).
                for stream in self.streams.values():
                    count = stream.write(s)
                    stream.flush()

                # Return the number of bytes written, which should be the same for all streams.
                return count

        def flush(self):
            return

        def __enter__(self):
            with ConsoleLogger._lock:
                if 'extra_logfile' not in self.streams:
                    raise RuntimeError("Cannot enter ConsoleLogger context directly. Use copy_to.")
                return self

        def __exit__(self, exc_type, exc_value, traceback):
            with ConsoleLogger._lock:
                del self.streams['extra_logfile']
                return None

    @classmethod
    def start(cls, logfile, mode='w', time_format=None):
        with cls._lock:
            if cls._logger is not None: raise RuntimeError("ConsoleLogger seems to have already been started.")
            if sys.stdout is not sys.__stdout__: raise RuntimeError("sys.stdout seems to have already been modified.")
            if sys.stderr is not sys.__stderr__: raise RuntimeError("sys.stderr seems to have already been modified.")
            cls._logger = cls._Logger(logfile, mode=mode, time_format=time_format)
            sys.stdout = cls._logger
            sys.stderr = cls._logger

    @classmethod
    def stop(cls):
        with cls._lock:
            if cls._logger is None: raise RuntimeError("ConsoleLogger seems to not have been started, cannot call stop.")
            if sys.stdout is not cls._logger: raise RuntimeError("sys.stdout seems to have already been restored to defaults.")
            if sys.stderr is not cls._logger: raise RuntimeError("sys.stderr seems to have already been restored to defaults.")
            cls._logger = None
            sys.stdout = sys.__stdout__
            sys.stderr = sys.__stderr__

    @classmethod
    def copy_to(cls, extra_logfile, mode='w'):
        with cls._lock:
            if cls._logger is None: raise RuntimeError("ConsoleLogger seems to not have been started, cannot call copy_to.")
            cls._logger.streams['extra_logfile'] = open(extra_logfile, mode)
            return cls._logger