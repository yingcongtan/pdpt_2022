import sys, os, datetime, math
import numpy as np
from sortedcollections import SortedDict
import string   
import pandas as pd
import random

import time, sys

src_dir_ = '/home/tan/Documents/GitHub/pdpt_2022/src'
sys.path.insert(1, src_dir_)


from docplex.cp.model import end_of, start_of, CpoModel
import docplex.cp as cpotpimizer

from gurobipy import Model, GRB, quicksum
import csv
from util import group_cycle_truck


def time_checker_cluster(constant, selected_cargo, created_truck, 
    selected_edge, truck_MP, truck_nodes, 
    cargo_unload, cargo_load, g_sol, h_sol, D_sol):
    
    """
    Check all time-related constriants for trucks and cargo in a SP solution
    """
    
    ### For all trucks
    for t in truck_MP:
        
        # the origin of the truck t
        n1 = truck_nodes[t][0]
        n2 = truck_nodes[t][1]
        
        time_t = D_sol[(n1, n2, t)]
        if time_t < 0:
            print('Initial time of truck {} is wrong'.format(t))
            return False
        
        for c in cargo_load[(n1, t)]:
            if True or selected_cargo[c][3] == n1:
                if selected_cargo[c][1] > time_t:
                    print('cargo lower bound of {} at {} on {} is wrong'.format(c, n1, t))
                    print('selected_cargo[c][1]:', selected_cargo[c][1], 'time_t:', time_t)
                    return False
        time_origin = time_t
        time_t += selected_edge[(n1, n2)]
        
        
        # the middle nodes of the truck t
        if len(truck_nodes[t]) > 2:
            for i in range(1, len(truck_nodes[t])-1):
                n1 = truck_nodes[t][i]
                n2 = truck_nodes[t][i+1]

                for c in cargo_unload[(n1, t)]:
                    if True or selected_cargo[c][4] == n1:
                        if time_t > selected_cargo[c][2]:
                            print('The cargo upper bound of {} at {} on {} is wrong'.format(c, n1, t))
                            print('time_t:', time_t, 'selected_cargo[c][2]:', selected_cargo[c][2], )
                            return False

                time_t += constant['node_fixed_time']
                for c in cargo_unload[(n1, t)]:
                    time_t += int(np.ceil(selected_cargo[c][0] * constant['loading_variation_coefficient']))
                for c in cargo_load[(n1, t)]:
                    time_t += int(np.ceil(selected_cargo[c][0] * constant['loading_variation_coefficient']))

                if time_t > D_sol[(n1, n2, t)]:
                    print('The time of edge from {} to {} on {} is wrong'.format(n1, n2, t))
                    print('time_t:', time_t, 'D_sol[(n1, n2, t)]', D_sol[(n1, n2, t)])
                    return False
                for c in cargo_load[(n1, t)]:
                    if True or selected_cargo[c][3] == n1:
                        if max(time_t, D_sol[(n1, n2, t)]) < selected_cargo[c][1]:
                            print('The cargo lower bound of {} at {} on {} is wrong'.format(c, n1, t))
                            print('max(time_t, D_sol[(n1, n2, t)]):', max(time_t, D_sol[(n1, n2, t)]))
                            print('time_t:', time_t, 'D_sol[(n1, n2, t)]:', D_sol[(n1, n2, t)])
                            print('selected_cargo[c][1]:', selected_cargo[c][1])
                            return False
                time_t = max(time_t, D_sol[(n1, n2, t)])

                time_t += selected_edge[(n1, n2)]
        
        # the destination of truck t
        n1 = truck_nodes[t][-1]
        n2 = 'dummy'
        
        for c in cargo_unload[(n1, t)]:
            if True or selected_cargo[c][4] == n1:
                if time_t > selected_cargo[c][2]:
                    print('The cargo upper bound of {} at {} on {} is wrong'.format(c, n1, t))
                    print('time_t:', time_t, 'selected_cargo[c][2]', selected_cargo[c][2])
                    return False
        
        time_t += constant['node_fixed_time']
        for c in cargo_unload[(n1, t)]:
            time_t += int(np.ceil(selected_cargo[c][0] * constant['loading_variation_coefficient']))

        if time_t > D_sol[(n1, n2, t)]:
            print('The time of edge from {} to {} on {} is wrong'.format(n1, n2, t))
            print('time_t:', time_t, 'D_sol[(n1, n2, t)]:', D_sol[(n1, n2, t)])
            return False
        if time_t - time_origin > created_truck[t][2]:
            print('The truck max worktime of {} is wrong'.format(t))
            print('time_t - time_origin:', time_t - time_origin, 'created_truck[t][2]', created_truck[t][2])
            return False
    
    ### For cargo and transfers
    # for key, value in cargo_unload.keys(): # original line from Jason
    for key, value in cargo_unload.items():
        n = key[0]
        t1 = key[1]
        for c in value:
            for t2 in created_truck.keys():
                if t2 != t1:
                    if (n, t2) in cargo_load.keys() and c in cargo_load[(n, t2)]:
                        # if g_sol[(n, c)] + int(np.ceil(constant['loading_variation_coefficient'] * selected_cargo[cargo_][0])) > h_sol[(n, c)]: original line from jason
                        if g_sol[(n, c)] + int(np.ceil(constant['loading_variation_coefficient'] * selected_cargo[c][0])) > h_sol[(n, c)]:
                            print('The transfer time at {} of {} between {} and {} is wrong'.format(n, c, t1, t2))
                            # print('long one:', g_sol[(n, c)] + int(np.ceil(constant['loading_variation_coefficient'] * selected_cargo[cargo_][0]))) #original line from Jason

                            print('long one:', g_sol[(n, c)] + int(np.ceil(constant['loading_variation_coefficient'] * selected_cargo[c][0])))
                            print('h_sol[(n, c)]', h_sol[(n, c)])
                            return False    
    
    return True

def capacity_checker_cluster(selected_cargo, solution_created_truck, 
    truck_MP, truck_nodes, cargo_in, cargo_out):
    
    """
    Check the capacity constraint of all trucks and cargo in a cluster
    """
    
    for t in truck_MP:
        
        cap = solution_created_truck[t][3]
        
        # first the truck origin
        n = truck_nodes[t][0]
        cap_t = 0
        for c in cargo_out[(n, t)]:
            cap_t += selected_cargo[c][0]
            if cap_t > cap:
                prit('The capacityof {} at {} is violated!'.format(t, n))
                print('cap_t:', cap_t, 'cap:', cap)
                return False
        
        # second the rest of nodes including the truck destination
        for i in range(1, len(truck_nodes[t])):
            n = truck_nodes[t][i]
            cap_t = 0
            for c in cargo_in[(n, t)]:
                cap_t += selected_cargo[c][0]
                if cap_t > cap:
                    prit('The capacityof {} at {} is violated!'.format(t, n))
                    print('cap_t:', cap_t, 'cap:', cap)
                    return False
        
    return True


def gurobi_master_cycle(constant, selected_cargo,  
    created_truck_yCycle, created_truck_nCycle, created_truck_all,
    selected_edge, selected_node, runtime, 
    filename, verbose = 0):
    
    """
    Gurobi model for the master problem
    including cycle and non-cycle trucks
    all parameters of the case
    
    The most commonly used solver for the routing MP
    
    return 
    1. obj_val_MP: objective value
    2. runtime_MP: runtime
    3. x_sol, s_sol, z_sol, y_sol, u_sol, D_sol: values of variables
    """

    MP = Model("Gurobi MIP for master problem with cycles")
    
#     node_list = ['N'+str(i) for i in range(node_lb, node_ub+1)]
    node_list = selected_node
    
    
    ###### Decision Variables ######
    
    # binary variables x
    x = {}
    for edge_ in selected_edge.keys():
        for truck_ in created_truck_all.keys():
            if edge_[0] != edge_[1]:
                x[(edge_[0], edge_[1], truck_)] = MP.addVar(vtype=GRB.BINARY)
#             MP.setAttr("BranchPriority", x[(edge[0], edge[1], truck)], 9)
    
    # binary variables s
    s = {}
    for truck_ in created_truck_all.keys():
        s[truck_] = MP.addVar(vtype=GRB.BINARY)
    
    # binary variables z
    z = {}
    for edge_ in selected_edge.keys():
        for truck_ in created_truck_all.keys():
            for cargo_ in selected_cargo.keys():
                if edge_[0] != edge_[1]:
                    z[(edge_[0], edge_[1], truck_, cargo_)] = \
                    MP.addVar(vtype=GRB.BINARY)
#                 MP.setAttr("BranchPriority", 
#                            z[(edge[0], edge[1], truck, cargo)], 5)
    
    # binary variables y
    y = {}
    for truck_ in created_truck_all.keys():
        for cargo_ in selected_cargo.keys():
            y[(truck_, cargo_)] = MP.addVar(vtype=GRB.BINARY)
    
    # binary variables u
    u = {}
    for node_ in node_list:
        for cargo_ in selected_cargo.keys():
            u[(node_, cargo_)] = MP.addVar(vtype=GRB.BINARY)
            
    # integer variable D
    D = {}
    for node_ in node_list:
        for truck_ in created_truck_all.keys():
            D[(node_, truck_)] = MP.addVar(vtype=GRB.INTEGER, lb=0)
#             MP.setAttr("BranchPriority", D[(node, truck)], 1)

    # integer variable Db for destinations of cycle trucks
    Db = {}
    for truck_ in created_truck_yCycle.keys():
        node_ = created_truck_yCycle[truck_][1]
        Db[(node_, truck_)] = MP.addVar(vtype=GRB.INTEGER, lb=0)


    variables = (x, s, z, y, u, D, Db)
    
    
    ###### Constraints ######
    
    # Flow constraints (2.1)
    for truck_ in created_truck_all.keys():
        origin_truck = created_truck_all[truck_][0]
        MP.addConstr( 
            quicksum(x[(origin_truck, succ_node, truck_)] * 1
                     for succ_node in node_list
                     if succ_node != origin_truck) == s[truck_] 
        )
    # Flow constraints (2.2) 
    for truck_ in created_truck_nCycle.keys():
        origin_truck = created_truck_nCycle[truck_][0] 
        MP.addConstr( 
            quicksum(x[(succ_node, origin_truck, truck_)] * 1
                     for succ_node in node_list
                     if succ_node != origin_truck) == 0 
        )

    # Flow constraints (2.3)
    for truck_ in created_truck_all.keys():
        destination_truck = created_truck_all[truck_][1]
        MP.addConstr( 
            quicksum(x[(pred_node, destination_truck, truck_)] * 1
                     for pred_node in node_list
                     if pred_node != destination_truck) == s[truck_] 
        )    
    # Flow constraints (2.4)
    for truck_ in created_truck_nCycle.keys():
        destination_truck = created_truck_nCycle[truck_][1]
        MP.addConstr( 
            quicksum(x[(destination_truck, pred_node, truck_)] * 1
                     for pred_node in node_list
                     if pred_node != destination_truck) == 0 
        )
    
    # Flow constraints (2.5)
    for truck_ in created_truck_all.keys():
        origin_truck = created_truck_all[truck_][0]
        destination_truck = created_truck_all[truck_][1]
        for node_ in node_list:
            if node_ != origin_truck and node_ != destination_truck:
                MP.addConstr(
                    quicksum(x[(pred_node, node_, truck_)] * 1
                             for pred_node in node_list
                             if pred_node != node_) 
                    ==
                    quicksum(x[(node_, succ_node, truck_)] * 1
                             for succ_node in node_list
                             if succ_node != node_) 
                )
    
    # Flow constraints (2.6a)
    for truck_ in created_truck_nCycle.keys():
        for node1 in node_list:
            for node2 in node_list:
                if node1 != node2:
                    MP.addConstr(
                        x[(node1, node2, truck_)] + 
                        x[(node2, node1, truck_)]
                        <= s[truck_]
                    )
    # Flow constraints (2.6b)
    for truck_ in created_truck_yCycle.keys():
        origin_truck = created_truck_yCycle[truck_][0]
        destination_truck = created_truck_yCycle[truck_][1]
        for node1 in node_list:
            for node2 in node_list:
                if node1 != node2 and \
                   node1 != origin_truck and \
                   node2 != origin_truck and \
                   node1 != destination_truck and \
                   node2 != destination_truck:
                    MP.addConstr(
                        x[(node1, node2, truck_)] + 
                        x[(node2, node1, truck_)]
                        <= s[truck_]
                    )
                    
    for cargo_ in selected_cargo.keys():
    # cargo flow constraints (2.7)
        origin_cargo = selected_cargo[cargo_][3]
        MP.addConstr( 
            quicksum(z[(origin_cargo, succ_node, truck_, cargo_)] * 1
                     for succ_node in node_list 
                     if succ_node != origin_cargo
                     for truck_ in created_truck_all.keys())
                     == 1 
        )
    # cargo flow constraints (2.8)
        MP.addConstr( 
            quicksum(z[(succ_node, origin_cargo, truck_, cargo_)] * 1
                     for succ_node in node_list 
                     if succ_node != origin_cargo
                     for truck_ in created_truck_all.keys())
                     == 0 
        )
    # cargo flow constraints (2.9)
        destination_cargo = selected_cargo[cargo_][4]
        MP.addConstr( 
            quicksum(z[(pred_node, destination_cargo, truck_, cargo_)] * 1
                     for pred_node in node_list 
                     if pred_node != destination_cargo
                     for truck_ in created_truck_all.keys())
                     == 1 
        )
    # cargo flow constraints (2.10)
        MP.addConstr( 
            quicksum(z[(destination_cargo, pred_node, truck_, cargo_)] * 1
                     for pred_node in node_list 
                     if pred_node != destination_cargo
                     for truck_ in created_truck_all.keys())
                     == 0 
        )
    
    # cargo flow constraints (2.11)
    for cargo_ in selected_cargo.keys():
        origin_cargo = selected_cargo[cargo_][3]
        destination_cargo = selected_cargo[cargo_][4]
        for node_ in node_list:
            if node_ != origin_cargo and node_ != destination_cargo:
                MP.addConstr(
                    quicksum(z[(pred_node, node_, truck_, cargo_)] * 1
                             for pred_node in node_list 
                             if pred_node != node_
                             for truck_ in created_truck_all.keys())
                    ==
                    quicksum(z[(node_, succ_node, truck_, cargo_)] * 1
                             for succ_node in node_list 
                             if succ_node != node_
                             for truck_ in created_truck_all.keys())
                )
#     # cargo flow constraints (2.12)
#         for truck in created_truck.keys():
#             for node1 in node_list:
#                 for node2 in node_list:
#                     if node1 != node2:
#                         MP.addConstr(
#                             z[(node1, node2, truck, cargo)] + 
#                             z[(node2, node1, truck, cargo)]
#                             <= s[truck]
#                         )
    
#     # z-x-y constraints (2.13)
#     for cargo in selected_cargo.keys():
#         for truck in created_truck.keys():
#             for node1 in node_list:
#                 for node2 in node_list:
#                     if node1 != node2:
#                         MP.addConstr(
#                             2 * z[(node1, node2, truck, cargo)]
#                             <= x[(node1, node2, truck)] 
#                              + y[(truck, cargo)]
#                         )
    
    # z-x-y constraints (2.13a)
    for cargo_ in selected_cargo.keys():
        for truck_ in created_truck_all.keys():
            for node1 in node_list:
                for node2 in node_list:
                    if node1 != node2:
                        MP.addConstr(
                            z[(node1, node2, truck_, cargo_)]
                            <= x[(node1, node2, truck_)] 
                        )
    
    # z-x-y constraints (2.13b)
    for cargo_ in selected_cargo.keys():
        for truck_ in created_truck_all.keys():
            for node1 in node_list:
                for node2 in node_list:
                    if node1 != node2:
                        MP.addConstr(
                            z[(node1, node2, truck_, cargo_)]
                            <= y[(truck_, cargo_)]
                        )
    
    # capacity constraints (2.14)
    for truck_ in created_truck_all.keys():
        for node1 in node_list:
            for node2 in node_list:
                if node1 != node2:
                    MP.addConstr(
                        quicksum(z[(node1, node2, truck_, cargo_)]
                                 * selected_cargo[cargo_][0]
                                 for cargo_ in selected_cargo.keys())
                        <= created_truck_all[truck_][3] * s[truck_]
                    )
    
    # truck count constriants (2.15)
    for cargo in selected_cargo.keys():
        MP.addConstr(
            quicksum(y[(truck_, cargo_)] * 1
                     for truck_ in created_truck_all.keys())
            <= 3 * s[truck_]
        )
        
    # u-z constraints (2.16)
    for cargo_ in selected_cargo.keys():
        origin_cargo = selected_cargo[cargo_][3]
        destination_cargo = selected_cargo[cargo_][4]
        for truck_ in created_truck_all.keys():
            for node_ in node_list:    
                if node_ != origin_cargo and node_ != destination_cargo:
                    MP.addConstr(
                        u[(node_, cargo_)] >=
                        quicksum(z[(node_, succ_node, truck_, cargo_)] * 1
                                 for succ_node in node_list
                                 if succ_node != node_)
                        -
                        quicksum(z[(pred_node, node_, truck_, cargo_)] * 1
                                 for pred_node in node_list
                                 if pred_node != node_)
                    )
    
    # transfer count constraints (2.17)
    for cargo_ in selected_cargo.keys():
        origin_cargo = selected_cargo[cargo_][3]
        destination_cargo = selected_cargo[cargo_][4]
        MP.addConstr(
            quicksum(u[(node_, cargo_)] * 1
                     for node_ in node_list
                     if node_ != origin_cargo and node_ != destination_cargo)
            <= 2
        )
        
    for truck_ in created_truck_all.keys():
    # truck start time constraints (2.18) are implicitly expressed in the variable definition
    # truck time lower bound constraints (2.19)
        origin_truck = created_truck_all[truck_][0]
        for node_ in node_list:
            MP.addConstr(
                D[(node_, truck_)] >= D[(origin_truck, truck_)]
            )
#         MP.addConstr(
#             D[(origin_truck, truck)] >= 0
#         )
    # truck time upper bound constraints (2.20a)
    for truck_ in created_truck_nCycle.keys():
        destination_truck = created_truck_nCycle[truck_][1]
        for node_ in node_list:
            MP.addConstr(
                D[(node_, truck_)] <= D[(destination_truck, truck_)]
            )
    # truck time upper bound constraints (2.20b)
    for truck_ in created_truck_yCycle.keys():
        destination_truck = created_truck_yCycle[truck_][1]
        for node_ in node_list:
            MP.addConstr(
                D[(node_, truck_)] <= Db[(destination_truck, truck_)]
            )

    # truck working time constraints (2.21a)
    # 1 * lp should be fine
    # instead of 2 * lp
    for truck_ in created_truck_nCycle.keys():
        destination_truck = created_truck_nCycle[truck_][1]
        MP.addConstr(
            D[(destination_truck, truck_)] - 
            D[(origin_truck, truck_)] +
            quicksum(y[(truck_, cargo_)] * 
                     int(np.ceil(selected_cargo[cargo_][0] * 1 *
                         constant['loading_variation_coefficient']))
                     for cargo_ in selected_cargo.keys())
            <= created_truck_nCycle[truck_][2] # * s[truck] # truck max worktime
        )
    
    # truck working time constraints (2.21b)
    for truck_ in created_truck_yCycle.keys():
        destination_truck = created_truck_yCycle[truck_][1]
        MP.addConstr(
            Db[(destination_truck, truck_)] - 
            D[(origin_truck, truck_)] +
            quicksum(y[(truck_, cargo_)] * 
                     int(np.ceil(selected_cargo[cargo_][0] * 1 *
                         constant['loading_variation_coefficient']))
                     for cargo_ in selected_cargo.keys())
            <= created_truck_yCycle[truck_][2] # * s[truck] # truck max worktime
        )
    
    # Big-M constraints
    bigM = 2000
    
    for cargo_ in selected_cargo.keys():
        origin_cargo = selected_cargo[cargo_][3]
        destination_cargo = selected_cargo[cargo_][4]
        for truck_ in created_truck_all.keys():
    # cargo lower bound constraints (2.22)
            MP.addConstr(
                D[(origin_cargo, truck_)] >= 
                selected_cargo[cargo_][1] * # cargo lower bound
                quicksum(z[(origin_cargo, node_, truck_, cargo_)] * 1
                         for node_ in node_list
                         if node_ != origin_cargo)
            )
    # cargo upper bound constraints (2.23a)
        for truck_ in created_truck_all.keys():
            if destination_cargo != created_truck_all[truck_][1]:
                MP.addConstr(
                    D[(destination_cargo, truck_)] - 
                    constant['node_fixed_time'] 
                    <= 
                    selected_cargo[cargo_][2] + # cargo upper bound
                    bigM * (1 - 
                    quicksum(z[(node_, destination_cargo, truck_, cargo_)] * 1
                            for node_ in node_list
                            if node_ != destination_cargo)
                    )
                )
    # cargo upper bound constraints (2.23b)
        for truck_ in created_truck_nCycle.keys():
            if destination_cargo == created_truck_nCycle[truck_][1]:
                MP.addConstr(
                    D[(destination_cargo, truck_)] - 
                    constant['node_fixed_time'] 
                    <= 
                    selected_cargo[cargo_][2] + # cargo upper bound
                    bigM * (1 - 
                    quicksum(z[(node_, destination_cargo, truck_, cargo_)] * 1
                            for node_ in node_list
                            if node_ != destination_cargo)
                    )
                )
    # cargo upper bound constraints (2.23c)
        for truck_ in created_truck_yCycle.keys():
            if destination_cargo == created_truck_yCycle[truck_][1]:
                MP.addConstr(
                    Db[(destination_cargo, truck_)] - 
                    constant['node_fixed_time'] 
                    <= 
                    selected_cargo[cargo_][2] + # cargo upper bound
                    bigM * (1 - 
                    quicksum(z[(node_, destination_cargo, truck_, cargo_)] * 1
                            for node_ in node_list
                            if node_ != destination_cargo)
                    )
                )
    
    # travel constraints (cycle elimination) (2.24a)
    for truck_ in created_truck_all.keys():
        for node1 in node_list:
            for node2 in node_list:
                if node1 != node2:
                    if node2 != created_truck_all[truck_][1]:
                        MP.addConstr(
                            D[(node1, truck_)] +
                            s[truck_] * (constant['node_fixed_time'] +
                                        selected_edge[(node1, node2)]) 
                            <=
                            D[(node2, truck_)] +
                            bigM * (1 - x[(node1, node2, truck_)])
                        )

    # travel constraints (cycle elimination) (2.24b)
    for truck_ in created_truck_nCycle.keys():
        for node1 in node_list:
            node2 = created_truck_nCycle[truck_][1]
            if node1 != node2:
                MP.addConstr(
                    D[(node1, truck_)] +
                    s[truck_] * (constant['node_fixed_time'] +
                                selected_edge[(node1, node2)]) 
                    <=
                    D[(node2, truck_)] +
                    bigM * (1 - x[(node1, node2, truck_)])
                )

    # travel constraints (cycle elimination) (2.24c)
    for truck_ in created_truck_yCycle.keys():
        for node1 in node_list:
            node2 = created_truck_yCycle[truck_][1]
            if node1 != node2:
                MP.addConstr(
                    D[(node1, truck_)] +
                    s[truck_] * (constant['node_fixed_time'] +
                                selected_edge[(node1, node2)]) 
                    <=
                    Db[(node2, truck_)] +
                    bigM * (1 - x[(node1, node2, truck_)])
                )

    # cargo transfer time consistency (2.25a)
    # when cargo p is transferred from truck k1 to k2 at node i
    # the visiting time D_{i}^{k1} should be <= D_{i}^{k2}
    for cargo_ in selected_cargo.keys():
        origin_cargo = selected_cargo[cargo_][3]
        destination_cargo = selected_cargo[cargo_][4]
        for truck1 in created_truck_nCycle.keys():
            for truck2 in created_truck_all.keys():
                if truck1 != truck2:
                    for node_ in node_list:    
                        if node_ != origin_cargo and \
                           node_ != destination_cargo:
                            MP.addConstr(
                                D[(node_, truck1)] <=
                                D[(node_, truck2)] +
                                bigM * 
                                (2 - 
                                 quicksum(z[(pred_node, node_, 
                                             truck1, cargo_)]
                                          for pred_node in node_list
                                          if pred_node != node_) -
                                 quicksum(z[(node_, succ_node, 
                                             truck2, cargo_)]
                                          for succ_node in node_list
                                          if succ_node != node_) )
                            )
    
    # cargo transfer time consistency (2.25b)
    for cargo_ in selected_cargo.keys():
        origin_cargo = selected_cargo[cargo_][3]
        destination_cargo = selected_cargo[cargo_][4]
        for truck1 in created_truck_yCycle.keys():
            for truck2 in created_truck_all.keys():
                if truck1 != truck2:
                    for node_ in node_list:    
                        if node_ == created_truck_yCycle[truck1][1] and \
                           node_ != origin_cargo and \
                           node_ != destination_cargo:
                            MP.addConstr(
                                Db[(node_, truck1)] <=
                                D[(node_, truck2)] +
                                bigM * 
                                (2 - 
                                 quicksum(z[(pred_node, node_, 
                                             truck1, cargo_)]
                                          for pred_node in node_list
                                          if pred_node != node_) -
                                 quicksum(z[(node_, succ_node, 
                                             truck2, cargo_)]
                                          for succ_node in node_list
                                          if succ_node != node_) )
                            )
            
    # cargo transfer time consistency (2.25c)
    for cargo_ in selected_cargo.keys():
        origin_cargo = selected_cargo[cargo_][3]
        destination_cargo = selected_cargo[cargo_][4]
        for truck1 in created_truck_yCycle.keys():
            for truck2 in created_truck_all.keys():
                if truck1 != truck2:
                    for node_ in node_list:    
                        if node_ != created_truck_yCycle[truck1][1] and \
                           node_ != origin_cargo and \
                           node_ != destination_cargo:
                            MP.addConstr(
                                D[(node_, truck1)] <=
                                D[(node_, truck2)] +
                                bigM * 
                                (2 - 
                                 quicksum(z[(pred_node, node_, 
                                             truck1, cargo_)]
                                          for pred_node in node_list
                                          if pred_node != node_) -
                                 quicksum(z[(node_, succ_node, 
                                             truck2, cargo_)]
                                          for succ_node in node_list
                                          if succ_node != node_) )
                            )


    ###### Objective ######
    
    # truck cost: proportional to truck count
    cost_truck = quicksum(s[truck_] * constant['truck_fixed_cost']
                          for truck_ in created_truck_all.keys())
    
    # traveling cost: proportional to total travel time
    cost_travel = quicksum(x[(node1, node2, truck_)] * 
                           selected_edge[(node1, node2)] * 
                           constant['truck_running_cost']
                           for truck_ in created_truck_all.keys()
                           for node1 in node_list
                           for node2 in node_list
                           if node1 != node2)
    
    # transfer cost: proportional to transfer count
    cost_transfer = quicksum(u[(node_, cargo_)] *
                             np.ceil(selected_cargo[cargo_][0] *
                             constant['cargo_reloading_cost'])
                             for cargo_ in selected_cargo.keys()
                             for node_ in node_list
                             if node_ != selected_cargo[cargo_][3]
                             if node_ != selected_cargo[cargo_][4])
    
    
    ###### Integrate the model and optimize ######
    
#     MP.setAttr("BranchPriority", x, 1)
#     MP.setAttr("BranchPriority", z, 5)
#     MP.setAttr("BranchPriority", D, 9)
    MP.setObjective(cost_truck + cost_travel + cost_transfer)
    MP.modelSense = GRB.MINIMIZE
    MP.Params.timeLimit = runtime
    MP.Params.OutputFlag = 1
    MP.Params.LogFile = filename
    MP.Params.Heuristics = 0.2
#     MP.Params.VarBranch = 0
    MP.update()
    MP.Params.LogToConsole  = 0

    MP.optimize()
    
    # if infeasible
    if MP.Status == 3:
        print('\n********************** MP Infeasible Proved *********************\n')
        return -1, runtime, [], [], [], [], [], [], [], -1, -1, -1
    
    runtime_MP = MP.Runtime
    obj_val_MP = MP.objVal

    
    # if no objective value
    if float('inf') == obj_val_MP:
        print('\n********************** MP Infeasible *********************\n')
        return -1, runtime, [], [], [], [], [], [], [], -1, -1, -1
        
    
    print('\n*********************** MP Feasible **********************\n')
    print("The Gurobi obj value is %i" % obj_val_MP)
    print("The Gurobi runtime is %f" % runtime_MP)
    
    
    ###### Get solutions ######
    
    # store all values in a list: sol
    sol = []
    for z in MP.getVars():
        sol.append(int(z.x))
        
    # retrieve values from the list sol
    count = 0
    # binary variables x
    x_sol = {}
    for edge_ in selected_edge.keys():
        for truck_ in created_truck_all.keys():
            if edge_[0] != edge_[1]:
                x_sol[(edge_[0], edge_[1], truck_)] = sol[count]
                count += 1
    
    # binary variables s
    s_sol = {}
    for truck_ in created_truck_all.keys():
        s_sol[truck_] = sol[count]
        count += 1
    
    # binary variables z
    z_sol = {}
    for edge_ in selected_edge.keys():
        for truck_ in created_truck_all.keys():
            for cargo_ in selected_cargo.keys():
                if edge_[0] != edge_[1]:
                    z_sol[(edge_[0], edge_[1], truck_, cargo_)] = \
                    sol[count]   
                    count += 1
                
    # binary variables y
    y_sol = {}
    for truck_ in created_truck_all.keys():
        for cargo_ in selected_cargo.keys():
            y_sol[(truck_, cargo_)] = sol[count]
            count += 1
    
    # binary variables u
    u_sol = {}
    for node in node_list:
        for cargo in selected_cargo.keys():
            u_sol[(node, cargo)] = sol[count]
            count += 1
            
    # integer variable D
    D_sol = {}
    for node_ in node_list:
        for truck_ in created_truck_all.keys():
            D_sol[(node_, truck_)] = sol[count]
            count += 1
    # integer variable Db
    Db_sol = {}
    for truck_ in created_truck_yCycle.keys():
        node_ = created_truck_yCycle[truck_][1]
        Db_sol[(node_, truck_)] = sol[count]
        count += 1
            
    sol_val = (x_sol, s_sol, z_sol, y_sol, u_sol, D_sol, Db_sol)
    
    
    # truck cost: proportional to truck count
    cost_truck_value = 0
    for truck_ in created_truck_all.keys():
        cost_truck_value += s_sol[truck_] * constant['truck_fixed_cost']
    print('\ncost_truck_value:', cost_truck_value)
    
    # traveling cost: proportional to total travel time
    cost_travel_value = 0
    for truck_ in created_truck_all.keys():
        for node1 in node_list:
            for node2 in node_list:
                if node1 != node2:
                    cost_travel_value += x_sol[(node1, node2, truck_)] * \
                                        selected_edge[(node1, node2)] * \
                                        constant['truck_running_cost']
    print('cost_travel_value:', cost_travel_value)
    
    # transfer cost: proportional to transfer count
    cost_transfer_value = 0
    for cargo_ in selected_cargo.keys():
        for node_ in node_list:
            if node_ != selected_cargo[cargo_][3] and \
               node_ != selected_cargo[cargo_][4] and \
               u_sol[(node_, cargo_)] == 1:
                # print('u_sol is 1 with:', node_, cargo_)
                cost_transfer_value += u_sol[(node_, cargo_)] * \
                                        np.ceil(selected_cargo[cargo_][0] *
                                        constant['cargo_reloading_cost'])
    print('cost_transfer_value:', cost_transfer_value)    
    
    
    return obj_val_MP, runtime_MP, \
           x_sol, s_sol, z_sol, y_sol, u_sol, D_sol, Db_sol, \
           cost_truck_value, cost_travel_value, cost_transfer_value


def candidate_gen_removed_cargo(size_element, selected_cargo):
    """
    Generate candidates for removed cargo
    return 
        candidates: list of lists of a specific number of cargo
        the number is size_element
    """

    # set n as the total number of selected cargo
    n = len(selected_cargo)

    # convert cargo names to a list
    keys = list(selected_cargo.keys())

    # initialized the case for size_element==1
    candidates = []
    indices_dynamic = []
    for i in range(n):
        index = [i]
        indices_dynamic.append(index)

    # do the while loop until the size of elements match
    while len(indices_dynamic[0]) < size_element:
        head = indices_dynamic.pop(0)
        last_index = head[-1]
        if last_index+1 < n:
            for i in range(last_index+1, n):
                head_new = head + [i]
#                 print('head_new:', head_new)
                indices_dynamic.append(head_new)

    assert len(indices_dynamic[0]) == size_element, \
           "wrong number of candidates"

    # generate candidates with correct element size
    for indices in indices_dynamic:
        candidate = []
        for index in indices:
            candidate.append(keys[index])
        candidates.append(candidate)


    return candidates

def greedy_fix_MP(constant, selected_cargo, 
    created_truck_yCycle, created_truck_nCycle, created_truck_all, 
    selected_edge, selected_node, runtime, filename):
    
    """
    Greedy heuristic to remove a parcel
    then evaluating feasibility of the MP
    until MP is feasible
    
    Call 'gurobi_master_cycle' function in this function.
    
    Return: 
        removed_cargo: a minimal set of removed cargo
        feasible MP solution
    """    
        
    # try removing one cargo,
    # if not feasible, try two, ...
    obj_val_MP = -1
    size_element = 0
    while obj_val_MP < 0:
        
        # Update size_element
        size_element += 1
        
        candidates = candidate_gen_removed_cargo(size_element, selected_cargo)
        
        # try each candidate
        for candidate in candidates:
            print('Evaluating removing', candidate, 'now..................')
            
            # the following two data store removed node
            selected_node_removed = [] # element: node
            selected_cargo_removed = {} # key:cargo, value: selected_cargo[cargo]          
            
            # Delete cargo from selected_cargo
            for cargo_ in candidate:
                selected_cargo_removed[cargo_] = selected_cargo[cargo_]
                if selected_cargo[cargo_][3] not in selected_node_removed:
                    selected_node_removed.append(selected_cargo[cargo_][3])
                if selected_cargo[cargo_][4] not in selected_node_removed:
                    selected_node_removed.append(selected_cargo[cargo_][4])
                del selected_cargo[cargo_]   

            # remain useful nodes
            useful_node = []
            for cargo_ in selected_cargo.keys():
                if selected_cargo[cargo_][3] not in useful_node and \
                   selected_cargo[cargo_][3] in selected_node_removed:
                    useful_node.append(selected_cargo[cargo_][3])
                if selected_cargo[cargo_][4] not in useful_node and \
                   selected_cargo[cargo_][4] in selected_node_removed:
                    useful_node.append(selected_cargo[cargo_][4])           
            for truck_ in created_truck_all.keys():
                if created_truck_all[truck_][0] not in useful_node and \
                   created_truck_all[truck_][0] in selected_node_removed:
                    useful_node.append(created_truck_all[truck_][0])
                if created_truck_all[truck_][1] not in useful_node and \
                   created_truck_all[truck_][1] in selected_node_removed:
                    useful_node.append(created_truck_all[truck_][1])
            
            for node_ in useful_node:
                selected_node_removed.remove(node_)
            
            if len(selected_node_removed) > 0:
                for node_ in selected_node_removed:
                    print('Remove node', node_)
                    selected_node.remove(node_)

    
            ###### call gurobi to solve the MP ######
        

            runtime = 10
            obj_val_MP, runtime_MP, \
            x_sol, s_sol, z_sol, y_sol, u_sol, D_sol, Db_sol, \
            cost_truck_value, cost_travel_value, cost_transfer_value = \
            gurobi_master_cycle(constant, selected_cargo, 
                created_truck_yCycle, created_truck_nCycle, created_truck_all, 
                selected_edge, selected_node, runtime, filename)

            # if feasible, break
            if obj_val_MP > 0:
                print('******Removing cargo', selected_cargo_removed, 'makes MP feasible******')

                break

            # KEY RECOVER COMPONENT!
            # recover all the data to the states before removal

            for cargo_ in candidate:
                # selected_cargo
                selected_cargo[cargo_] = selected_cargo_removed[cargo_]
            
            for node_ in selected_node_removed:
                selected_node.append(node_)
            
            
    
    return selected_cargo, created_truck_all, selected_edge, selected_node, \
           selected_cargo_removed, obj_val_MP, runtime_MP, \
           x_sol, s_sol, z_sol, y_sol, u_sol, D_sol
 

def MP_to_SP(constant, selected_cargo, 
    created_truck_yCycle, created_truck_nCycle, created_truck_all, 
    selected_edge, selected_node,
    x_sol, s_sol, z_sol, y_sol, u_sol, D_sol, verbose = 0):
    
    """
    A very commonly used function!
    
    Once we have a feasible MP solution, 
    we will need this function to convert MP solution to SP parameters
    
    Preprocess the solution of MP
    generate parameters of SP
    return
    1. truck_MP: truck used in the MP solution, 
    2. truck_nodes: nodes visited by a truck, 
    3. truck_nodes_index: dictionary of node visited ranks for a truck, 
    4. cargo_in: cargos sent into a node by a truck, 
    5. cargo_out: cargos sent out of a node by a truck,
    6. trasfer_nodes: set of transfer nodes, 
    7. cargo_unload: cargos that are unloaded by a truck at a node, 
    8. cargo_load: cargos that are loaded by a truck at a node
    """
    
    print('')
    # truck_MP: truck used in the MP solution
    truck_MP = []
    for truck_ in created_truck_all.keys():
        if s_sol[truck_] == 1:
            if verbose >0:
                print('The truck', truck_, 'is used!')
            truck_MP.append(truck_)
    if verbose > 0:
        print(truck_MP)
    
    # create node_list
    node_list = selected_node
    
    # truck_nodes: nodes visited by a truck
    truck_nodes = {}
    for truck_ in truck_MP:
        truck_nodes[truck_] = []
        truck_nodes[truck_].append(created_truck_all[truck_][0])
        node_last = truck_nodes[truck_][-1]
        count_error = 0
        # append a node after the truck origin
        for node_ in node_list:
            if node_ not in truck_nodes[truck_]:
                if (node_last, node_, truck_) in x_sol.keys() and \
                   x_sol[(node_last, node_, truck_)] == 1:
                    truck_nodes[truck_].append(node_)
                    node_last = node_
        while node_last != created_truck_all[truck_][1]:
            count_error += 1
            for node_ in node_list:
                if node_ not in truck_nodes[truck_] or node_ == created_truck_all[truck_][1]:
                    if (node_last, node_, truck_) in x_sol.keys() and \
                       x_sol[(node_last, node_, truck_)] == 1:
                        truck_nodes[truck_].append(node_)
                        node_last = node_
                        break   
            if count_error >= 200:
                return [], {}, {}, \
                       {}, {}, \
                       [], {}, {}
        if verbose >0:
            print('Visited nodes of truck ', truck_, ' :', truck_nodes[truck_])
    
    # truck_nodes_index: dictionary of node visited ranks for a truck
    # WHEN THERE IS CYCLE, truck_nodes_index IS PROBLAMATIC!
    truck_nodes_index = {}
    for truck_ in truck_MP:
        truck_nodes_index[truck_] = {}
        truck_nodes_index[truck_][created_truck_all[truck_][0]] = 0
        for index in range(1, len(truck_nodes[truck_])):
            node_ = truck_nodes[truck_][index]
            if node_ != created_truck_all[truck_][0]:
                truck_nodes_index[truck_][node_] = index
    
    # cargo_in: cargos sent into a node by a truck
    cargo_in = {}
    # cargo_out: cargos sent out of a node by a truck
    cargo_out = {}
    for truck_ in truck_MP:
        for node_ in truck_nodes[truck_]:
            if (node_, truck_) not in cargo_in.keys():
                cargo_in[(node_, truck_)] = []
            if (node_, truck_) not in cargo_out.keys():
                cargo_out[(node_, truck_)] = []
            for cargo_ in selected_cargo.keys():
                for pred_node in truck_nodes[truck_]:
                    if pred_node != node_:
                        if (pred_node, node_, truck_, cargo_) in z_sol.keys() and \
                           z_sol[(pred_node, node_, truck_, cargo_)] == 1 and \
                           cargo_ not in cargo_in[(node_, truck_)]:
                            cargo_in[(node_, truck_)].append(cargo_)   
            for cargo_ in selected_cargo.keys():
                for succ_node in truck_nodes[truck_]:
                    if succ_node != node_:
                        if (node_, succ_node, truck_, cargo_) in z_sol.keys() and \
                           z_sol[(node_, succ_node, truck_, cargo_)] == 1 and \
                           cargo_ not in cargo_out[(node_, truck_)]:
                            cargo_out[(node_, truck_)].append(cargo_)
                            
    # transfer_nodes: set of transfer nodes
    transfer_nodes = []
    for cargo_ in selected_cargo.keys():
        for node_ in node_list:
            if u_sol[(node_, cargo_)] == 1:
                transfer_nodes.append((node_, cargo_))
    
    if verbose > 0:
        print('transfer_nodes:', transfer_nodes)
                
    # cargo_unload: cargos that are unloaded by a truck at a node
    cargo_unload = {}
    # cargo_load: cargos that are loaded by a truck at a node
    cargo_load = {}
    for truck_ in truck_MP:
        for node_ in truck_nodes[truck_]:
            if (node_, truck_) not in cargo_unload.keys():
                cargo_unload[(node_, truck_)] = []
            if (node_, truck_) not in cargo_load.keys():
                cargo_load[(node_, truck_)] = []
            for cargo_ in selected_cargo.keys():
                succ_sum = 0
                for succ_node in truck_nodes[truck_]:
                    if succ_node != node_:
                        succ_sum += z_sol[(node_, succ_node, truck_, cargo_)]
                pred_sum = 0
                for pred_node in truck_nodes[truck_]:
                    if pred_node != node_:
                        pred_sum += z_sol[(pred_node, node_, truck_, cargo_)]
                if pred_sum >= 1 and succ_sum == 0 and \
                   cargo_ not in cargo_unload[(node_, truck_)]:
                    cargo_unload[(node_, truck_)].append(cargo_)
                if pred_sum == 0 and succ_sum >= 1 and \
                   cargo_ not in cargo_load[(node_, truck_)]:
                    cargo_load[(node_, truck_)].append(cargo_)
    print('')
    
    
    return truck_MP, truck_nodes, truck_nodes_index, \
           cargo_in, cargo_out, \
           transfer_nodes, cargo_unload, cargo_load


def cpo_sub(constant, selected_cargo, 
    created_truck_yCycle, created_truck_nCycle, created_truck_all,  
    selected_edge, selected_node,
    truck_MP, truck_nodes, truck_nodes_index, cargo_in, cargo_out,
    transfer_nodes, cargo_unload, cargo_load,
    runtime):
    
    """
    CP model of the subproblem
    all variables are interval variables
    
    A very commonly used function for solving the scheduling SP!
    A satisfaction problem that CP solves fast
    
    return
    1. feasibility_SP: feasibility of the MP solution
    2. g_sol: variable values of g, 
    3. h_sol: variable values of h, 
    4. D_sol: variable values of D
    """
    
    ###### Initialize a CP model ######
    
    SP = CpoModel()
    
    
    ###### Decision variables ######
    
    # create node_list
#     node_list = ['N'+str(i) for i in range(node_lb, node_ub+1)]
    node_list = selected_node
    
    # g[(node, cargo)]: interval variable of the unloading operation
    g = {}
    for truck_ in truck_MP:
        for node_ in truck_nodes[truck_]:
            for cargo_ in cargo_unload[(node_, truck_)]:
                g[(node_, cargo_)] = SP.interval_var(
                size=int(np.ceil(
                         constant['loading_variation_coefficient'] * 
                         selected_cargo[cargo_][0])))
    
    # h[(node, cargo)]: interval variable of the loading operation
    h = {}
    for truck_ in truck_MP:
        for node_ in truck_nodes[truck_]:
            for cargo_ in cargo_load[(node_, truck_)]:
                h[(node_, cargo_)] = SP.interval_var(
                size=int(np.ceil(
                         constant['loading_variation_coefficient'] * 
                         selected_cargo[cargo_][0])))
    
    # D: interval variable of the traveling operation 
    D = {}
    dummy_node = 'dummy'
    for truck_ in truck_MP:
        for i in range(len(truck_nodes[truck_]) - 1):
            node1 = truck_nodes[truck_][i]
            node2 = truck_nodes[truck_][i+1]
            D[(node1, node2, truck_)] = \
            SP.interval_var(size = selected_edge[(node1, node2)])
            # SP.interval_var(size = int(np.ceil(selected_edge[(node1, node2)])))

        node_last = truck_nodes[truck_][-1]
        D[(node_last, dummy_node, truck_)] = SP.interval_var(size = 0)
        
    
    ###### Constraints ######
    
    ### ADD CONSTRAINTS REGARDING TWO TRUCKS AT TRANSFER NODES
    for pair in transfer_nodes:
        node_ = pair[0]
        cargo_ = pair[1]
        if cargo_ in selected_cargo.keys():
            SP.add(end_of(g[(node_, cargo_)])
                   <= start_of(h[(node_, cargo_)]) )
                        
        
    
    # precedence D and g (2.40)
    # arrival time of truck_ at node2 + fixed time at node2 
    # <= the start time of any unloading operations at node2 of truck_
    for truck_ in truck_MP:
        for i in range(len(truck_nodes[truck_]) - 1):
            node1 = truck_nodes[truck_][i]
            node2 = truck_nodes[truck_][i+1]
            for cargo_ in cargo_unload[(node2, truck_)]:
                SP.add(end_of(D[(node1, node2, truck_)]) + 
                       constant['node_fixed_time'] 
                       <= start_of(g[(node2, cargo_)]) )
                
    # precedence D and g (2.41)
    # departure time of truck_ at node1 
    # >= the end time of any unloading operations at node1 of truck_
    for truck_ in truck_MP:
        for i in range(1, len(truck_nodes[truck_]) - 1):
            node1 = truck_nodes[truck_][i]
            node2 = truck_nodes[truck_][i+1]
            for cargo_ in cargo_unload[(node1, truck_)]:
                SP.add(start_of(D[(node1, node2, truck_)])
                       >= end_of(g[(node1, cargo_)]) )
        # special case for truck destinations
        destination_truck = truck_nodes[truck_][-1]
        for cargo_ in cargo_unload[(destination_truck, truck_)]:
            SP.add(start_of(D[(destination_truck, dummy_node, truck_)])
                   >= end_of(g[(destination_truck, cargo_)]) )
    
    # precedence g and h (2.42)
    # unloading operations are ahead of loading operations
    # at a node other than the truck origin or truck destination
    for truck_ in truck_MP:
        for i in range(1, len(truck_nodes[truck_])-1):
            node_ = truck_nodes[truck_][i]
            for cargo1 in cargo_unload[(node_, truck_)]:
                for cargo2 in cargo_load[(node_, truck_)]:
                    SP.add(end_of(g[(node_, cargo1)])  
                           <= start_of(h[(node_, cargo2)]) )
    
    # precedence D and h (2.43)
    # departure time of truck_ at node1 
    # >= the end time of any loading operations at node1 of truck_
    for truck_ in truck_MP:
        for i in range(1, len(truck_nodes[truck_]) - 1):
            node1 = truck_nodes[truck_][i]
            node2 = truck_nodes[truck_][i+1]
            for cargo_ in cargo_load[(node1, truck_)]:
                SP.add(start_of(D[(node1, node2, truck_)])
                       >= end_of(h[(node1, cargo_)]) )
    
    #***** Some details of unloading and loading time ******
    # if a cargo p is loaded by a truck k at a node i 
    # (including the origin k^{+} of the truck k)
    # but the origin of the cargo p^{+} is not node i
    # then the loading time l_{p}^{+} should be accounted 
    # when calculating the departure time D_{i}^{k} of truck k at node i 
    # hence we should add a case to constraint (2.44)
    # which we call constraints (2.44)
    
    # precedence D and h (2.44)
    # departure time of truck_ at its origin 
    # >= the end time of any loading operations at the origin of truck_
    # only valid when the cargo is transfered to truck_
    for truck_ in truck_MP:
        origin_truck = truck_nodes[truck_][0]
        origin_next_truck = truck_nodes[truck_][1]
        for cargo_ in cargo_load[(origin_truck, truck_)]:
            origin_cargo = selected_cargo[cargo_][3]
            if origin_truck != origin_cargo:
                SP.add(start_of(D[(origin_truck, origin_next_truck, truck_)])
                       >= end_of(h[(origin_truck, cargo_)]) )
    
    # precedence D and h (2.45)
    # arrival time of truck_ at node2 + fixed time at node2 
    # <= the start time of any loading operations at node2
    for truck_ in truck_MP:
        for i in range(0, len(truck_nodes[truck_]) - 1 ):
            node1 = truck_nodes[truck_][i]
            node2 = truck_nodes[truck_][i+1]
            for cargo_ in cargo_load[(node2, truck_)]:
                SP.add(end_of(D[(node1, node2, truck_)])
                       + constant['node_fixed_time']
                       <= start_of(h[(node2, cargo_)]) )
    
    # precedence D and destination_cargo (2.46)
    # arrival time of truck_ at a cargo's destination
    # <= the cargo upper bound
    for cargo_ in selected_cargo.keys():
        destination_cargo = selected_cargo[cargo_][4]
        upper_tw_cargo = selected_cargo[cargo_][2]
        for truck_ in truck_MP:
            if (destination_cargo, truck_) in cargo_in.keys():
                if cargo_ in cargo_in[(destination_cargo, truck_)]:
                    if destination_cargo == created_truck_all[truck_][1] and \
                       truck_ in created_truck_yCycle.keys():
                        pred_node = truck_nodes[truck_][len(truck_nodes_index[truck_]) - 1]
                    else:
                        destination_index = truck_nodes_index[truck_][destination_cargo]
                        pred_node = truck_nodes[truck_][destination_index - 1]
                    SP.add(end_of(D[(pred_node, destination_cargo, truck_)]) <= upper_tw_cargo )
                    break
    
    # precedence D and origin_cargo (2.47)
    # departure time of truck_ at a cargo's origin
    # >= the cargo lower bound
    for cargo_ in selected_cargo.keys():
        origin_cargo = selected_cargo[cargo_][3]
        lower_tw_cargo = selected_cargo[cargo_][1]
        for truck_ in truck_MP:
            if (origin_cargo, truck_) in cargo_out.keys() and \
               cargo_ in cargo_out[(origin_cargo, truck_)]:
                origin_index = \
                truck_nodes_index[truck_][origin_cargo]
                succ_node = truck_nodes[truck_][origin_index + 1]
                SP.add(start_of(D[(origin_cargo, succ_node, truck_)])
                       >= lower_tw_cargo )
    
    # precedence D and origin_truck (2.48)
    # the departure time of truck_ >= 0
    for truck_ in truck_MP:
        origin_truck = truck_nodes[truck_][0]
        origin_next_truck = truck_nodes[truck_][1]
        SP.add(start_of(D[(origin_truck, origin_next_truck, truck_)])
               >= 0 )
        
    # precedence D, origin_truck, and destination_truck (2.49)
    # departure time of truck_ at its destination - departure time of truck_ at its origin
    # <= the maximum worktime of truck_
    for truck_ in truck_MP:
        origin_truck = truck_nodes[truck_][0]
        origin_next_truck = truck_nodes[truck_][1]
        destination_truck = truck_nodes[truck_][-1]
        worktime_truck = created_truck_all[truck_][2]
        SP.add(start_of(D[(destination_truck, dummy_node, truck_)]) -
               start_of(D[(origin_truck, origin_next_truck, truck_)])
               <= worktime_truck )
        
    # disjunctive unloading g (2.50)
    disjunctive_g = {}
    for truck_ in truck_MP:
        for i in range(1, len(truck_nodes[truck_])):
            node_ = truck_nodes[truck_][i]
            disjunctive_g[(node_, truck_)] = []
            for cargo_ in cargo_unload[(node_, truck_)]:
                disjunctive_g[(node_, truck_)].append(g[(node_, cargo_)])
            if len(disjunctive_g[(node_, truck_)]) > 1:
                SP.add( SP.no_overlap(disjunctive_g[(node_, truck_)]) )
    
    # disjunctive loading h (2.51)
    disjunctive_h = {}
    for truck_ in truck_MP:
        for i in range(1, len(truck_nodes[truck_]) - 1):
            node_ = truck_nodes[truck_][i]
            disjunctive_h[(node_, truck_)] = []
            for cargo_ in cargo_load[(node_, truck_)]:
                disjunctive_h[(node_, truck_)].append(h[(node_, cargo_)])
            if len(disjunctive_h[(node_, truck_)]) > 1:
                SP.add( SP.no_overlap(disjunctive_h[(node_, truck_)]) )
    
    # visiting order D (2.52)
    # + constant['node_fixed_time'] is very important!!!!
    for truck_ in truck_MP:
        for i in range(1, len(truck_nodes[truck_]) - 1):
            node0 = truck_nodes[truck_][i-1]
            node1 = truck_nodes[truck_][i]
            node2 = truck_nodes[truck_][i+1]
            SP.add(end_of(D[(node0, node1, truck_)])
                   + constant['node_fixed_time'] 
                   <= start_of(D[(node1, node2, truck_)]) )
    
    # visiting order destination D (2.53)
    # + constant['node_fixed_time'] is very important!!!!
    for truck_ in truck_MP:
        i = len(truck_nodes[truck_]) - 1
        node0 = truck_nodes[truck_][i-1]
        node1 = truck_nodes[truck_][i]
        SP.add(end_of(D[(node0, node1, truck_)])
               + constant['node_fixed_time']
               <= start_of(D[(node1, dummy_node, truck_)]) )
        
        
    ###### Solve the satisfaction SP ######
    
    # This param is needed for running SP.solve() in linux environment:
    # execfile='/opt/ibm/ILOG/CPLEX_Studio201/cpoptimizer/bin/x86-64_linux/cpoptimizer'

    SP_sol = SP.solve(TimeLimit = runtime, LogVerbosity = 'Verbose',
                     execfile='/opt/ibm/ILOG/CPLEX_Studio201/cpoptimizer/bin/x86-64_linux/cpoptimizer')
    
    feasibility_SP = SP_sol.get_solve_status()

    SP_sol.write('/home/tan/Documents/cp_log.log')
    SP_sol.get_search_status()

    conflict = cpotpimizer.solution.CpoRefineConflictResult(SP_sol)
    conflic_cont = conflict.get_all_member_constraints()
    conflict.print_conflict('/home/tan/Documents/cp_conflict.log')
    
    
    ###### Solution retriveal ######
    
    g_sol = {}
    h_sol = {}
    D_sol = {}
    
    # if feasible:
    if feasibility_SP == "Feasible":
        print('******************** SP Feasible **********************\n')
        
        # variable values of g
        for truck_ in truck_MP:
            for node_ in truck_nodes[truck_]:
                for cargo_ in cargo_unload[(node_, truck_)]:
                    if SP_sol.get_var_solution(g[(node_, cargo_)])\
                       is not None:
                        g_sol[(node_, cargo_)] = \
                        SP_sol.get_var_solution(g[(node_, cargo_)])\
                        .get_start()
            
        # variable values of h
        for truck_ in truck_MP:
            for node_ in truck_nodes[truck_]:
                for cargo_ in cargo_load[(node_, truck_)]:
                    if SP_sol.get_var_solution(h[(node_, cargo_)])\
                       is not None:
                        h_sol[(node_, cargo_)] = \
                        SP_sol.get_var_solution(h[(node_, cargo_)])\
                        .get_start()
            
        # variable values of D
        for truck_ in truck_MP:
            for i in range(len(truck_nodes[truck_]) - 1):
                node1 = truck_nodes[truck_][i]
                node2 = truck_nodes[truck_][i+1]
                if SP_sol.get_var_solution(D[(node1, node2, truck_)])\
                   is not None:
                    D_sol[(node1, node2, truck_)] = \
                    SP_sol.get_var_solution(D[(node1, node2, truck_)])\
                    .get_start()
            node_last = truck_nodes[truck_][-1]
            if SP_sol.get_var_solution(D[(node_last, dummy_node, truck_)])\
               is not None:
                D_sol[(node_last, dummy_node, truck_)] = \
                SP_sol.get_var_solution(D[(node_last, dummy_node, truck_)])\
                .get_start()
    
    # if infeasible:
    else:
        print('******************** SP Infeasible **********************\n')
    
    
    return feasibility_SP, g_sol, h_sol, D_sol

def greedy_heuristic(constant, selected_cargo,
    created_truck_yCycle, created_truck_nCycle, created_truck_all, 
    selected_edge, selected_node,
    truck_MP, truck_nodes, truck_nodes_index,
    cargo_in, cargo_out,
    transfer_nodes, cargo_unload, cargo_load):
    
    """
    Greedy heuristic to remove a violated parcel
    then evaluating feasibility of the SP
    until SP is feasible
    
    Call CP evaluation function in this function.
    This function is also commonly used!
    
    Return: 
        removed_cargo: a minimal set of removed cargo
        data related to feasible truck routes and 
                        feasible cargo routes
    """
    
    removed_cargo = []
    
#     print('\ncargo_unload:')
#     for key, value in cargo_unload.items():
#         print(key, value)
        
#     print('\ncargo_load:')
#     for key, value in cargo_load.items():
#         print(key, value)
        
    # try removing one cargo,
    # if not feasible, try two, ...
    feasibility_SP = ''
    size_element = 0
    while feasibility_SP != 'Feasible':
        
        # Update size_element
        size_element += 1
        
        candidates = candidate_gen_removed_cargo(size_element, selected_cargo)
        
        # try each candidate
        for candidate in candidates:
            print('Evaluating removing', candidate, 'now..................')
            
            # the following four dicts store removed cargo
            cargo_in_removed = {}
            cargo_out_removed = {}
            cargo_unload_removed = {}
            cargo_load_removed = {}
            
            # Remove cargo from four lists of cargos
            # These four lists all use [(node, truck)] as keys
            for cargo_ in candidate:
                # key: cargo, value: (node, truck)
                cargo_in_removed[cargo_] = []
                cargo_out_removed[cargo_] = []
                cargo_unload_removed[cargo_] = []
                cargo_load_removed[cargo_] = []
                
                for truck_ in truck_MP:
                    for node_ in truck_nodes[truck_]:
                        if cargo_ in cargo_in[(node_, truck_)]:
                            cargo_in[(node_, truck_)].remove(cargo_)
#                             print('remove {} from cargo_in[({},{})]'.format(cargo_, node_, truck_))
                            if (node_, truck_) not in cargo_in_removed[cargo_]:
                                cargo_in_removed[cargo_].append((node_, truck_))
#                             print('Remove cargo', cargo, 'from cargo_in')
                        if cargo_ in cargo_out[(node_, truck_)]:
                            cargo_out[(node_, truck_)].remove(cargo_)
#                             print('remove {} from cargo_out[({},{})]'.format(cargo_, node_, truck_))
                            if (node_, truck_) not in cargo_out_removed[cargo_]:
                                cargo_out_removed[cargo_].append((node_, truck_))
#                             print('Remove cargo', cargo, 'from cargo_out')
                        if cargo_ in cargo_unload[(node_, truck_)]:
                            cargo_unload[(node_, truck_)].remove(cargo_)
#                             print('remove {} from cargo_unload[({},{})]'.format(cargo_, node_, truck_))
                            if (node_, truck_) not in cargo_unload_removed[cargo_]:
                                cargo_unload_removed[cargo_].append((node_, truck_))
#                             print('Remove cargo', cargo, 'from cargo_unload')
                        if cargo_ in cargo_load[(node_, truck_)]:
                            cargo_load[(node_, truck_)].remove(cargo_)
#                             print('remove {} from cargo_load[({},{})]'.format(cargo_, node_, truck_))
                            if (node_, truck_) not in cargo_load_removed[cargo_]:
                                cargo_load_removed[cargo_].append((node_, truck_))
#                             print('Remove cargo', cargo, 'from cargo_load')
            
            # the following two data store removed node
            truck_nodes_removed = [] # element: (truck, node, index)
            selected_cargo_removed = {} # key:cargo, value: selected_cargo[cargo] 
            
            # Remove node from truck_nodes[truck] 
            # if cargo_unload[(node, truck)] and 
            # cargo_load[(node, truck)] are empty
            # and the node is not the origin or destination of the truck
            for truck_ in truck_MP:
                for node_ in truck_nodes[truck_]:
                    if node_ != created_truck_all[truck_][0] and \
                       node_ != created_truck_all[truck_][1] and \
                       cargo_unload[(node_, truck_)] == [] and \
                       cargo_load[(node_, truck_)] == []:
                        index = truck_nodes[truck_].index(node_)
                        truck_nodes[truck_].remove(node_)
#                         print('cargo_unload[({}, {})]'.format(node_, truck_), cargo_unload[(node_, truck_)])
#                         print('cargo_load[({}, {})]'.format(node_, truck_), cargo_load[(node_, truck_)])
                        print('node {} is removed from truck {}!'.format(node_, truck_))
                        truck_nodes_removed.append((truck_, node_, index))
#                         print('Remove node', node, 
#                               'from visited nodes of truck', truck)
                        del truck_nodes_index[truck_][node_]
                        # cargo_in[(node, truck)] == [] and\
                        # cargo_out[(node, truck)] == [] and\
            
            
            # Delete cargo from selected_cargo
            for cargo_ in candidate:
                selected_cargo_removed[cargo_] = selected_cargo[cargo_]
                del selected_cargo[cargo_]   
#                 print('Remove cargo', cargo, 'from selected_cargo')

            # Modify truck_nodes_index
            for truck_ in truck_MP:
                for index in range(len(truck_nodes[truck_])):
                    node_ = truck_nodes[truck_][index]
                    if node_ != created_truck_all[truck_][0]:
                        truck_nodes_index[truck_][node_] = index
                if truck_ in created_truck_nCycle.keys():
                    assert len(truck_nodes_index[truck_]) == len(truck_nodes[truck_])
            
            # print selected_cargo
#             for key, value in selected_cargo.items():
#                 print(key, value)
            
            # call CPO to solve the satisfaction SP
            runtime = 10
            feasibility_SP, g_sol, h_sol, D_sol = \
            cpo_sub(constant, selected_cargo,
                created_truck_yCycle, created_truck_nCycle, created_truck_all,  
                selected_edge, selected_node,
                truck_MP, truck_nodes, truck_nodes_index, cargo_in, cargo_out,
                transfer_nodes, cargo_unload, cargo_load,
                runtime)

            # if feasible, break
            if feasibility_SP == 'Feasible':
                removed_cargo = candidate
                print('******Removing cargo', candidate, 'makes SP feasible******')
                
                if time_checker_cluster(constant, selected_cargo, created_truck_all, 
                                    selected_edge, truck_MP, truck_nodes, 
                                    cargo_unload, cargo_load, g_sol, h_sol, D_sol):
                    print('\nThe time constraint of the current cluster is satisfied!')
                else:
                    sys.exit()
                
                break

            # KEY RECOVER COMPONENT!
            # recover all the data to the states before removal

            for cargo_ in candidate:
                # selected_cargo
                selected_cargo[cargo_] = selected_cargo_removed[cargo_]

                # cargo_in, cargo_out, cargo_unload, cargo_load
                # key: cargo, value: (node, truck)
                for pair in cargo_in_removed[cargo_]:
                    cargo_in[pair].append(cargo_)
                for pair in cargo_out_removed[cargo_]:
                    cargo_out[pair].append(cargo_)
                for pair in cargo_unload_removed[cargo_]:
                    cargo_unload[pair].append(cargo_)
                for pair in cargo_load_removed[cargo_]:
                    cargo_load[pair].append(cargo_)

            # truck_nodes
            for triplet in truck_nodes_removed:
                truck_nodes[triplet[0]].insert(triplet[2], triplet[1])

            # truck_nodes_index
            # Modify truck_nodes_index
            for truck_ in truck_MP:
                for index in range(len(truck_nodes[truck_])):
                    node_ = truck_nodes[truck_][index]
                    if node_ != created_truck_all[truck_][0]:
                        truck_nodes_index[truck_][node_] = index
                if truck_ in created_truck_nCycle.keys():
                    assert len(truck_nodes_index[truck_]) == len(truck_nodes[truck_])
    
    
    return removed_cargo, \
           truck_MP, truck_nodes, truck_nodes_index, \
           cargo_in, cargo_out, \
           transfer_nodes, cargo_unload, cargo_load


def calculate_SP_cost(constant, selected_cargo, selected_edge, 
    truck_MP, truck_nodes, 
    cargo_in, cargo_out, transfer_nodes):
    
    """
    Calculate costs of SP based on feasible SP solution
    
    Commonly used to calculate actual costs.
    
    Inputs:
        constant
        selected_cargo
        selected_edge
        truck_MP: used trucks
        truck_nodes: nodes visited by used trucks
        cargo_in
        cargo_out
        transfer_nodes: transfer nodes
    Outputs:
        truck_cost
        travel_cost
        transfer_cost
    """

    truck_cost, travel_cost, transfer_cost = 0, 0, 0

    ###### truck_cost ######

    for truck_ in truck_MP:
        truck_cost += constant['truck_fixed_cost']


    ###### travel_cost ######

    for truck_ in truck_MP:
        for i in range(len(truck_nodes[truck_])-1):
            travel_cost += constant['truck_running_cost'] * \
            selected_edge[(truck_nodes[truck_][i], truck_nodes[truck_][i+1])]


    ###### transfer_cost ######

    if len(transfer_nodes) > 0:
        for pair in transfer_nodes:
            node_ = pair[0]
            cargo_ = pair[1]
            for truck_1 in truck_MP:
                for truck_2 in truck_MP:
                    if truck_1 != truck_2:
                        if (node_, truck_1) in cargo_in.keys() and \
                           (node_, truck_2) in cargo_out.keys() and \
                           cargo_ in cargo_in[(node_, truck_1)] and \
                           cargo_ in cargo_out[(node_, truck_2)]:
                            transfer_cost += int(selected_cargo[cargo_][0] * \
                                                 constant['cargo_reloading_cost'])

    print('\nThe truck_cost:', truck_cost)
    print('The travel_cost:', travel_cost)
    print('The transfer_cost:', transfer_cost, '\n')
    
    return truck_cost, travel_cost, transfer_cost


    
def postprocess_pdpt_solution(subroutes, x_sol, s_sol, y_sol, z_sol, u_sol, verbose = 0):

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
        if s_sol[truck_key] == 1:
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
        else:
            truck_route[truck_key] = []

    # RECALL, cargo is a dictionary with the following format:
    # cargo['nb_cargo'] = ['size', 'lb_time', 'ub_time', 'departure_node', 'arrival_node']

    for cargo_key, cargo_value in selected_cargo.items():
        dest = cargo_value[-1]
        if sum([z_sol[(node_, dest, truck_key, cargo_key)] for node_ in selected_node for truck_key in selected_truck.keys() if node_!= dest]) == 1:
            truck_list = [truck_key for truck_key in selected_truck.keys()  # list of truck_key
                                        if y_sol[(truck_key, cargo_key)]  == 1  ]# if cargo_key is carried by truck_key i.e., y^k_r == 1
            n_curr = cargo_value[3]
            for n_next in selected_node:
                for truck_key in truck_list:
                    if n_next!=n_curr and z_sol[(n_curr, n_next, truck_key, cargo_key)] == 1:
                        if len(cargo_route[cargo_key]) == 0: # append source as the first node
                            cargo_route[cargo_key].append((truck_key, n_curr))
                        cargo_route[cargo_key].append((truck_key, n_next))
                        n_curr = n_next
                        break
                else:
                    continue
                break
            while n_curr != cargo_value[-1]:
                for n_next in selected_node:
                    for truck_key in truck_list:
                        if n_next!=n_curr and z_sol[(n_curr, n_next, truck_key, cargo_key)] == 1:
                            cargo_route[cargo_key].append((truck_key, n_next))
                            n_curr = n_next
                            break
                    else:
                        continue
                    break

    trucks_per_cargo = {cargo_key: [truck_key  for truck_key in selected_truck.keys() if y_sol[(truck_key, cargo_key)]==1 ] for cargo_key, cargo_value in selected_cargo.items()}





    return truck_used, cargo_delivered, cargo_undelivered, \
           trucks_per_cargo, cargo_in_truck, truck_route, cargo_route 



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


def greedy_heuristic(constant, selected_cargo,
    created_truck_yCycle, created_truck_nCycle, created_truck_all, 
    selected_edge, selected_node,
    truck_MP, truck_nodes, truck_nodes_index,
    cargo_in, cargo_out,
    transfer_nodes, cargo_unload, cargo_load):
    
    """
    Greedy heuristic to remove a violated parcel
    then evaluating feasibility of the SP
    until SP is feasible
    
    Call CP evaluation function in this function.
    This function is also commonly used!
    
    Return: 
        removed_cargo: a minimal set of removed cargo
        data related to feasible truck routes and 
                        feasible cargo routes
    """
    
    removed_cargo = []
    
#     print('\ncargo_unload:')
#     for key, value in cargo_unload.items():
#         print(key, value)
        
#     print('\ncargo_load:')
#     for key, value in cargo_load.items():
#         print(key, value)
        
    # try removing one cargo,
    # if not feasible, try two, ...
    feasibility_SP = ''
    size_element = 0
    while feasibility_SP != 'Feasible':
        
        # Update size_element
        size_element += 1
        
        candidates = candidate_gen_removed_cargo(size_element, selected_cargo)
        
        # try each candidate
        for candidate in candidates:
            print('Evaluating removing', candidate, 'now..................')
            
            # the following four dicts store removed cargo
            cargo_in_removed = {}
            cargo_out_removed = {}
            cargo_unload_removed = {}
            cargo_load_removed = {}
            
            # Remove cargo from four lists of cargos
            # These four lists all use [(node, truck)] as keys
            for cargo_ in candidate:
                # key: cargo, value: (node, truck)
                cargo_in_removed[cargo_] = []
                cargo_out_removed[cargo_] = []
                cargo_unload_removed[cargo_] = []
                cargo_load_removed[cargo_] = []
                
                for truck_ in truck_MP:
                    for node_ in truck_nodes[truck_]:
                        if cargo_ in cargo_in[(node_, truck_)]:
                            cargo_in[(node_, truck_)].remove(cargo_)
#                             print('remove {} from cargo_in[({},{})]'.format(cargo_, node_, truck_))
                            if (node_, truck_) not in cargo_in_removed[cargo_]:
                                cargo_in_removed[cargo_].append((node_, truck_))
#                             print('Remove cargo', cargo, 'from cargo_in')
                        if cargo_ in cargo_out[(node_, truck_)]:
                            cargo_out[(node_, truck_)].remove(cargo_)
#                             print('remove {} from cargo_out[({},{})]'.format(cargo_, node_, truck_))
                            if (node_, truck_) not in cargo_out_removed[cargo_]:
                                cargo_out_removed[cargo_].append((node_, truck_))
#                             print('Remove cargo', cargo, 'from cargo_out')
                        if cargo_ in cargo_unload[(node_, truck_)]:
                            cargo_unload[(node_, truck_)].remove(cargo_)
#                             print('remove {} from cargo_unload[({},{})]'.format(cargo_, node_, truck_))
                            if (node_, truck_) not in cargo_unload_removed[cargo_]:
                                cargo_unload_removed[cargo_].append((node_, truck_))
#                             print('Remove cargo', cargo, 'from cargo_unload')
                        if cargo_ in cargo_load[(node_, truck_)]:
                            cargo_load[(node_, truck_)].remove(cargo_)
#                             print('remove {} from cargo_load[({},{})]'.format(cargo_, node_, truck_))
                            if (node_, truck_) not in cargo_load_removed[cargo_]:
                                cargo_load_removed[cargo_].append((node_, truck_))
#                             print('Remove cargo', cargo, 'from cargo_load')
            
            # the following two data store removed node
            truck_nodes_removed = [] # element: (truck, node, index)
            selected_cargo_removed = {} # key:cargo, value: selected_cargo[cargo] 
            
            # Remove node from truck_nodes[truck] 
            # if cargo_unload[(node, truck)] and 
            # cargo_load[(node, truck)] are empty
            # and the node is not the origin or destination of the truck
            for truck_ in truck_MP:
                for node_ in truck_nodes[truck_]:
                    if node_ != created_truck_all[truck_][0] and \
                       node_ != created_truck_all[truck_][1] and \
                       cargo_unload[(node_, truck_)] == [] and \
                       cargo_load[(node_, truck_)] == []:
                        index = truck_nodes[truck_].index(node_)
                        truck_nodes[truck_].remove(node_)
#                         print('cargo_unload[({}, {})]'.format(node_, truck_), cargo_unload[(node_, truck_)])
#                         print('cargo_load[({}, {})]'.format(node_, truck_), cargo_load[(node_, truck_)])
                        print('node {} is removed from truck {}!'.format(node_, truck_))
                        truck_nodes_removed.append((truck_, node_, index))
#                         print('Remove node', node, 
#                               'from visited nodes of truck', truck)
                        del truck_nodes_index[truck_][node_]
                        # cargo_in[(node, truck)] == [] and\
                        # cargo_out[(node, truck)] == [] and\
            
            
            # Delete cargo from selected_cargo
            for cargo_ in candidate:
                selected_cargo_removed[cargo_] = selected_cargo[cargo_]
                del selected_cargo[cargo_]   
#                 print('Remove cargo', cargo, 'from selected_cargo')

            # Modify truck_nodes_index
            for truck_ in truck_MP:
                for index in range(len(truck_nodes[truck_])):
                    node_ = truck_nodes[truck_][index]
                    if node_ != created_truck_all[truck_][0]:
                        truck_nodes_index[truck_][node_] = index
                if truck_ in created_truck_nCycle.keys():
                    assert len(truck_nodes_index[truck_]) == len(truck_nodes[truck_])
            
            # print selected_cargo
#             for key, value in selected_cargo.items():
#                 print(key, value)
            
            # call CPO to solve the satisfaction SP
            runtime = 10
            feasibility_SP, g_sol, h_sol, D_sol = \
            cpo_sub(constant, selected_cargo,
                created_truck_yCycle, created_truck_nCycle, created_truck_all,  
                selected_edge, selected_node,
                truck_MP, truck_nodes, truck_nodes_index, cargo_in, cargo_out,
                transfer_nodes, cargo_unload, cargo_load,
                runtime)

            # if feasible, break
            if feasibility_SP == 'Feasible':
                removed_cargo = candidate
                print('******Removing cargo', candidate, 'makes SP feasible******')
                
                if time_checker_cluster(constant, selected_cargo, created_truck_all, 
                                    selected_edge, truck_MP, truck_nodes, 
                                    cargo_unload, cargo_load, g_sol, h_sol, D_sol):
                    print('\nThe time constraint of the current cluster is satisfied!')
                else:
                    sys.exit()
                
                break

            # KEY RECOVER COMPONENT!
            # recover all the data to the states before removal

            for cargo_ in candidate:
                # selected_cargo
                selected_cargo[cargo_] = selected_cargo_removed[cargo_]

                # cargo_in, cargo_out, cargo_unload, cargo_load
                # key: cargo, value: (node, truck)
                for pair in cargo_in_removed[cargo_]:
                    cargo_in[pair].append(cargo_)
                for pair in cargo_out_removed[cargo_]:
                    cargo_out[pair].append(cargo_)
                for pair in cargo_unload_removed[cargo_]:
                    cargo_unload[pair].append(cargo_)
                for pair in cargo_load_removed[cargo_]:
                    cargo_load[pair].append(cargo_)

            # truck_nodes
            for triplet in truck_nodes_removed:
                truck_nodes[triplet[0]].insert(triplet[2], triplet[1])

            # truck_nodes_index
            # Modify truck_nodes_index
            for truck_ in truck_MP:
                for index in range(len(truck_nodes[truck_])):
                    node_ = truck_nodes[truck_][index]
                    if node_ != created_truck_all[truck_][0]:
                        truck_nodes_index[truck_][node_] = index
                if truck_ in created_truck_nCycle.keys():
                    assert len(truck_nodes_index[truck_]) == len(truck_nodes[truck_])
    
    
    return removed_cargo, \
           truck_MP, truck_nodes, truck_nodes_index, \
           cargo_in, cargo_out, \
           transfer_nodes, cargo_unload, cargo_load

def time_checker_cluster(constant, selected_cargo, created_truck, 
    selected_edge, truck_MP, truck_nodes, 
    cargo_unload, cargo_load, g_sol, h_sol, D_sol):
    
    """
    Check all time-related constriants for trucks and cargo in a SP solution
    """
    
    ### For all trucks
    for t in truck_MP:
        
        # the origin of the truck t
        n1 = truck_nodes[t][0]
        n2 = truck_nodes[t][1]
        
        time_t = D_sol[(n1, n2, t)]
        if time_t < 0:
            print('Initial time of truck {} is wrong'.format(t))
            return False
        
        for c in cargo_load[(n1, t)]:
            if True or selected_cargo[c][3] == n1:
                if selected_cargo[c][1] > time_t:
                    print('cargo lower bound of {} at {} on {} is wrong'.format(c, n1, t))
                    print('selected_cargo[c][1]:', selected_cargo[c][1], 'time_t:', time_t)
                    return False
        time_origin = time_t
        time_t += selected_edge[(n1, n2)]
        
        
        # the middle nodes of the truck t
        if len(truck_nodes[t]) > 2:
            for i in range(1, len(truck_nodes[t])-1):
                n1 = truck_nodes[t][i]
                n2 = truck_nodes[t][i+1]

                for c in cargo_unload[(n1, t)]:
                    if True or selected_cargo[c][4] == n1:
                        if time_t > selected_cargo[c][2]:
                            print('The cargo upper bound of {} at {} on {} is wrong'.format(c, n1, t))
                            print('time_t:', time_t, 'selected_cargo[c][2]:', selected_cargo[c][2], )
                            return False

                time_t += constant['node_fixed_time']
                for c in cargo_unload[(n1, t)]:
                    time_t += int(np.ceil(selected_cargo[c][0] * constant['loading_variation_coefficient']))
                for c in cargo_load[(n1, t)]:
                    time_t += int(np.ceil(selected_cargo[c][0] * constant['loading_variation_coefficient']))

                if time_t > D_sol[(n1, n2, t)]:
                    print('The time of edge from {} to {} on {} is wrong'.format(n1, n2, t))
                    print('time_t:', time_t, 'D_sol[(n1, n2, t)]', D_sol[(n1, n2, t)])
                    return False
                for c in cargo_load[(n1, t)]:
                    if True or selected_cargo[c][3] == n1:
                        if max(time_t, D_sol[(n1, n2, t)]) < selected_cargo[c][1]:
                            print('The cargo lower bound of {} at {} on {} is wrong'.format(c, n1, t))
                            print('max(time_t, D_sol[(n1, n2, t)]):', max(time_t, D_sol[(n1, n2, t)]))
                            print('time_t:', time_t, 'D_sol[(n1, n2, t)]:', D_sol[(n1, n2, t)])
                            print('selected_cargo[c][1]:', selected_cargo[c][1])
                            return False
                time_t = max(time_t, D_sol[(n1, n2, t)])

                time_t += selected_edge[(n1, n2)]
        
        # the destination of truck t
        n1 = truck_nodes[t][-1]
        n2 = 'dummy'
        
        for c in cargo_unload[(n1, t)]:
            if True or selected_cargo[c][4] == n1:
                if time_t > selected_cargo[c][2]:
                    print('The cargo upper bound of {} at {} on {} is wrong'.format(c, n1, t))
                    print('time_t:', time_t, 'selected_cargo[c][2]', selected_cargo[c][2])
                    return False
        
        time_t += constant['node_fixed_time']
        for c in cargo_unload[(n1, t)]:
            time_t += int(np.ceil(selected_cargo[c][0] * constant['loading_variation_coefficient']))

        if time_t > D_sol[(n1, n2, t)]:
            print('The time of edge from {} to {} on {} is wrong'.format(n1, n2, t))
            print('time_t:', time_t, 'D_sol[(n1, n2, t)]:', D_sol[(n1, n2, t)])
            return False
        if time_t - time_origin > created_truck[t][2]:
            print('The truck max worktime of {} is wrong'.format(t))
            print('time_t - time_origin:', time_t - time_origin, 'created_truck[t][2]', created_truck[t][2])
            return False
    
    ### For cargo and transfers
    for key, value in cargo_unload.keys():
        n = key[0]
        t1 = key[1]
        for c in value:
            for t2 in created_truck.keys():
                if t2 != t1:
                    if (n, t2) in cargo_load.keys() and c in cargo_load[(n, t2)]:
                        if g_sol[(n, c)] + int(np.ceil(constant['loading_variation_coefficient'] * selected_cargo[cargo_][0])) > h_sol[(n, c)]:
                            print('The transfer time at {} of {} between {} and {} is wrong'.format(n, c, t1, t2))
                            print('long one:', g_sol[(n, c)] + int(np.ceil(constant['loading_variation_coefficient'] * selected_cargo[cargo_][0])))
                            print('h_sol[(n, c)]', h_sol[(n, c)])
                            return False    
    
    return True