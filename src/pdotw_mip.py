from gurobipy import Model, quicksum, GRB
import numpy as np

def eval_pdotw_sol(constant, edge_shortest,
    truck_used_file, truck_route_file, verbose = 0):
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
    if verbose > 0:
        print('\tThe truck_cost based on the PDPT solution:', truck_cost)
        print('\tThe travel cost based on the PDPT solution:', travel_cost)
        print('\tThe total cost:', truck_cost + travel_cost)

    return truck_cost, travel_cost

def postprocess_solution_pdotw(cargo, truck, 
    selected_cargo, created_truck_all,
    node_list_truck_hubs, 
    x_sol, y_sol,truck_route, cargo_route, verbose = 0):
    """
    ONLY ONE TRUCK IN THE SOLUTION

    Postprocess the solution of the PDPTW with oneTruck
    record all used/unused trucks/cargo
    
    Inputs:
        ... ...
    Outputs:
        they are useful 
        ... ...
    
    Since there is no slack in the PDPTW model, using A and Ab 
    is more accurate than using D and Db

    """
    
    
    ###### NEED CAREFUL CHECK ######
    ###### TOO MANY CASES ######
    
    
    ###### Preprocess the gurobi solution ######
    
    # trucks that are used in the origin stage
    truck_used = []
    # cargo that are delivered in the origin stage
    cargo_delivered = {}
    # cargo that are not delivered in the origin stage
    cargo_undelivered = {}
    # the truck carries cargo in the origin stage
    cargo_truck_origin = {}
    # cargo carried by specific truck
    cargo_in_truck = {}
    
    
    
    ###### TRUCKS ######
    
    ### For each truck, if it goes to its destination 
    ### without bringing any cargo, go back to the last node
    
    truck_ = list(created_truck_all.keys())[0]
    # print('Consider truck {} now ------'.format(truck_))
    v = created_truck_all[truck_]

    # first test whether the truck has carried any cargo
    cargo_in_truck[truck_] = []
    truck_used_flag = 0
    # Generate cargo_in_truck
    for c in selected_cargo.keys():
        if y_sol[(truck_, c)] == 1:
            if verbose > 0: 
                print('    The cargo {} has been carried by truck {}'.format(c, truck_))
            truck_used_flag += 1
            cargo_in_truck[truck_].append(c)
    
    ### if the truck is used in this solution
    if truck_used_flag > 0:
        
        # put the truck_ into used trucks
        truck_used.append(truck_)
        
        ### Evaluate whether the truck has arrived at its destination
        # if so
        if v[1] == truck[truck_][1]:
            if verbose > 0: 
                print('+++ Truck {}'.format(truck_), 'has arrived at its real destination!')
                
        # else: the truck would be considered in the hub problem
        else:
            if verbose > 0: 
                print('+++ Truck {}'.format(truck_), 'has NOT arrived at its real destination!')
            

    # if the truck is not used in this cluster
    # it just goes from its origin to its destination
    else:
        if verbose > 0: 
            print('+++ The {} is unused!'.format(truck_))
        
    
    ###### Generate truck routes for used trucks ######
    
    for truck_ in truck_used:
        source = created_truck_all[truck_][0]
        # do not need to specify created_truck_yCycle
        # with the following while loop structure
        for node_ in node_list_truck_hubs[truck_]:
            if node_ != source and x_sol[(source, node_, truck_)] == 1:
                truck_route[truck_].append(node_)
                source = node_
                break
        while source != created_truck_all[truck_][1]:
            for node_ in node_list_truck_hubs[truck_]:
                if node_ != source and x_sol[(source, node_, truck_)] == 1:
                    truck_route[truck_].append(node_)
                    source = node_
                    break


    if verbose > 0: 
        print('+++ The truck_route:')
    for key, value in truck_route.items():
        print(f'        {key, value}')
    
    ###### CARGO ######

    t = list(created_truck_all.keys())[0]
    # print('\nNow consider the truck', t)

    for c, v in selected_cargo.items():
        if y_sol[(t, c)] == 1:
            ### Evaluate whether the cargo has been delivered
            # if so
            if v[4] == cargo[c][4]:
                # if this cargo has been delivered
                # put it into cargo_delivered
                cargo_delivered[c] = []
                cargo_route_origin = []
                # locate the truck that carries the cargo
                for i in range(len(truck_route[t])):
                    if truck_route[t][i] == v[3]:
                        j = i
                        while truck_route[t][j] != v[4]:
                            cargo_route_origin.append(
                            truck_route[t][j])
                            j += 1
                        cargo_route_origin.append(
                        truck_route[t][j])
                        break
                # the cargo route is just a piece of the truck route
                cargo_delivered[c] = (t, cargo_route_origin.copy())
                for node_ in cargo_route_origin:
                    cargo_route[c].append((t, node_))

            # else: this cargo has not been delivered
            else:
                cargo_undelivered[c] = []
                cargo_route_origin = []
                # locate the truck that carries the cargo
                for i in range(len(truck_route[t])):
                    if truck_route[t][i] == v[3]:
                        j = i
                        while truck_route[t][j] != v[4]:
                            cargo_route_origin.append(
                            truck_route[t][j])
                            j += 1
                        cargo_route_origin.append(
                        truck_route[t][j])
                        break
                # the cargo route is just a piece of the truck route
                cargo_undelivered[c] = (t, cargo_route_origin.copy())
                if verbose > 0: 
                    print('+++ The undelivered cargo:', cargo_undelivered[c])
                for node_ in cargo_route_origin:
                    cargo_route[c].append((t, node_))
    
    ###### cargo_truck_origin ######
    
    t = list(created_truck_all.keys())[0]

    for c in selected_cargo.keys():
        cargo_truck_origin[c] = -1
        # if the c is carried by truck t
        if (t,c) in y_sol.keys() and y_sol[(t,c)] == 1:
            cargo_truck_origin[c] = t
                
            
    
    if verbose > 0:
        print('+++ Summary of postprocessing')
        print(f'   [The truck_used] size: {len(truck_used)}')
        print(f'       {truck_used}')

        print(f'   [The truck_route] size: {len(truck_route)}')
        for t_key, t_value in truck_route.items():
            print(f'       {t_key, t_value}')

        print(f'   [The cargo_delivered] size: {len(cargo_delivered)}')
        for t_key, t_value in cargo_delivered.items():
            print(f'       {t_key, t_value}')

        print(f'   [The cargo_undelivered] size: {len(cargo_undelivered)}')
        for t_key, t_value in cargo_undelivered.items():
            print(f'       {t_key, t_value}')

        print(f'   [The cargo_truck_origin] size: {len(cargo_truck_origin)}')
        for t_key, t_value in cargo_truck_origin.items():
            print(f'       {t_key, t_value}')

        print(f'   [The cargo_in_truck] size: {len(cargo_in_truck)}')
        for t_key, t_value in cargo_in_truck.items():
            print(f'       {t_key, t_value}')
            
            
    
    return truck_used, cargo_delivered, cargo_undelivered, \
           cargo_truck_origin, cargo_in_truck

def pdotw_mip_gurobi(constant, 
    selected_cargo, single_truck_deviation,
    created_truck_yCycle, created_truck_nCycle, created_truck_all,
    node_list_truck_hubs, selected_edge, node_cargo_size_change, 
    runtime, filename, verbose = 1):
    
    """
    This is the gurobi_PDPTW_cycle_node_list_oneTruck_deviation in jason's code (oneTruck_clean.ipynb)
    This is actually the PDOTW !!!
    
    There is ONLY ONE TRUCK in this PDtruck_entering_leave_T11PTW
             ALL REMAINING CARGO  in this PDPTW
    
    Gurobi model for pickup and delivery problem with time window (PDPTW)
    all parameters of the case.

    This model considers cycle and non-cycle trucks.
    Constraints on cycle trucks are very easy to be wrong
    Evaluate them carefully

    node_list_truck_hubs[T1] = [N1, N2, ...]

    node_cargo_size_change[(node_, cargo_)] =  0 if node_ is not oc / dc
    node_cargo_size_change[(node_, cargo_)] =  cargo_size if node_ is oc 
    node_cargo_size_change[(node_, cargo_)] = -cargo_size if node_ is dc 
    
    return 
    1. obj_val_MP: objective value
    2. runtime_MP: runtime
    3. x_sol, z_sol, y_sol, S_sol, D_sol, A_sol, 
       Sb_sol, Db_sol, Ab_sol: values of variables
        
    """

    MP = Model("Gurobi MIP for PDPTW")
    

    
    
    ###### Decision Variables ######
    ###### six types of decision variables ######
    
    # binary variables x
    # ==1 if the truck_ traverse an edge 
    x = {}
    for truck_ in created_truck_all.keys():
        for i in node_list_truck_hubs[truck_]:
            for j in node_list_truck_hubs[truck_]:
                if i != j:
                    x[(i, j, truck_)] = \
                    MP.addVar(vtype=GRB.BINARY)
    
#     # binary variables z
#     # ==1 if the truck_ is used in the solution
#     z = {}
#     for truck_ in created_truck_all.keys():
#         z[truck_] = MP.addVar(vtype=GRB.BINARY)
    
    # binary variables y
    # ==1 if cargo_ is carried by truck_
    y = {}
    for truck_ in created_truck_all.keys():
        for cargo_ in selected_cargo.keys():
            y[(truck_, cargo_)] = MP.addVar(vtype=GRB.BINARY)

    # Integer variables S
    # current total size of cargos on truck_ at node_
    S = {}
    Sb = {}
    for truck_ in created_truck_all.keys():
        for node_ in node_list_truck_hubs[truck_]:
            S[(node_, truck_)] = MP.addVar(vtype=GRB.INTEGER, lb=0,
                                           ub=created_truck_all[truck_][3])
    # if truck_ is a cycle truck and node_ is its destination
    # add a decision variable Sb
    # NOT A D VARIABLE ANYMORE
    for truck_ in created_truck_yCycle.keys():
        node_ = created_truck_yCycle[truck_][1]
        Sb[(node_, truck_)] = 0
    
    # integer variable D
    # departure time of truck_ at node_
    D = {}
    Db = {}
    for truck_ in created_truck_all.keys():
        for node_ in node_list_truck_hubs[truck_]:
            D[(node_, truck_)] = MP.addVar(vtype=GRB.INTEGER, lb=0)
    # if truck_ is a cycle truck and node_ is its destination
    # add a decision variable Ab
    for truck_ in created_truck_yCycle.keys():
        node_ = created_truck_yCycle[truck_][1]
        Db[(node_, truck_)] = MP.addVar(vtype=GRB.INTEGER, lb=0)
    
    # integer variable A
    # arrival time of truck_ at node_
    A = {}
    Ab = {}
    for truck_ in created_truck_all.keys():
        for node_ in node_list_truck_hubs[truck_]:
            A[(node_, truck_)] = MP.addVar(vtype=GRB.INTEGER, lb=0)
    # if truck_ is a cycle truck and node_ is its destination
    # add a decision variable Db
    for truck_ in created_truck_yCycle.keys():
        node_ = created_truck_yCycle[truck_][1]
        Ab[(node_, truck_)] = MP.addVar(vtype=GRB.INTEGER, lb=0)
    
        
    

    
    
    ###### Constraints ######
    ###### Distinguish cycle trucks and non-cycle trucks ######
    
    # Flow constraints (3.1)
    # the truck must start from its origin
    for truck_ in created_truck_all.keys():
        origin_truck = created_truck_all[truck_][0]
        if origin_truck in node_list_truck_hubs[truck_]:
            MP.addConstr( 
                quicksum(x[(origin_truck, succ_node, truck_)] * 1
                         for succ_node in node_list_truck_hubs[truck_]
                         if succ_node != origin_truck) == 1 
            )
        
    # Flow constraints (3.2)  
    # only applies to non-cycle trucks
    # no flow enters the origin of a non-cycle truck
    for truck_ in created_truck_nCycle.keys():
        origin_truck = created_truck_nCycle[truck_][0]
        if origin_truck in node_list_truck_hubs[truck_]:
            MP.addConstr( 
                quicksum(x[(succ_node, origin_truck, truck_)] * 1
                         for succ_node in node_list_truck_hubs[truck_]
                         if succ_node != origin_truck) == 0 
            )

    # Flow constraints (3.3)
    # the truck must end at its destination
    for truck_ in created_truck_all.keys():
        destination_truck = created_truck_all[truck_][1]
        if destination_truck in node_list_truck_hubs[truck_]:
            MP.addConstr( 
                quicksum(x[(pred_node, destination_truck, truck_)] * 1
                         for pred_node in node_list_truck_hubs[truck_]
                         if pred_node != destination_truck) == 1
            )    
        
    # Flow constraints (3.4)
    # only applies to non-cycle trucks
    # no flow departs from the destination of a non-cycle truck
    for truck_ in created_truck_nCycle.keys():
        destination_truck = created_truck_nCycle[truck_][1]
        if destination_truck in node_list_truck_hubs[truck_]:
            MP.addConstr( 
                quicksum(x[(destination_truck, pred_node, truck_)] * 1
                         for pred_node in node_list_truck_hubs[truck_]
                         if pred_node != destination_truck) == 0 
            )
    
    
    ### No cycle part below ----------------------------------------------
    
    # Flow constraints (3.5)
    # flow in = flow out
    # Don't consider origin_truck and destination_truck in this constraint
    for truck_ in created_truck_all.keys():
        origin_truck = created_truck_all[truck_][0]
        destination_truck = created_truck_all[truck_][1]
        for node_ in node_list_truck_hubs[truck_]:
            if node_ != origin_truck and node_ != destination_truck:
                MP.addConstr(
                    quicksum(x[(pred_node, node_, truck_)] * 1
                             for pred_node in node_list_truck_hubs[truck_]
                             if pred_node != node_) 
                    ==
                    quicksum(x[(node_, succ_node, truck_)] * 1
                             for succ_node in node_list_truck_hubs[truck_]
                             if succ_node != node_) 
                )
    
    # An edge is used at most once by a truck (3.6)
    # only apply for non-cycle trucks
    # and non-origin nodes for cycle trucks
    for truck_ in created_truck_nCycle.keys():
        for i in node_list_truck_hubs[truck_]:
            for j in node_list_truck_hubs[truck_]:
                if i != j:
                    MP.addConstr(
                        x[(i, j, truck_)] +
                        x[(j, i, truck_)]
                        <= 1
                    )
    for truck_ in created_truck_yCycle.keys():
        origin_truck = created_truck_yCycle[truck_][0]
        for i in node_list_truck_hubs[truck_]:
            for j in node_list_truck_hubs[truck_]:
                if i != j:
                    if i != origin_truck and j != origin_truck:
                        MP.addConstr(
                            x[(i, j, truck_)] +
                            x[(j, i, truck_)]
                            <= 1
                        )
            
    
    # origin_c is visited by truck_ if y[(truck_, c)] == 1 (3.9)
    for truck_ in created_truck_all.keys():
        for cargo_ in selected_cargo.keys():
            origin_cargo = selected_cargo[cargo_][3]
            if origin_cargo in node_list_truck_hubs[truck_]:
                MP.addConstr(
                    quicksum(x[(origin_cargo, node_, truck_)] * 1
                             for node_ in node_list_truck_hubs[truck_]
                             if node_ != origin_cargo)
                    >= 
                    y[(truck_, cargo_)]
                )
    
    # destination_c is visited by truck_ if y[(truck_, c)] == 1 (3.10)
    for truck_ in created_truck_all.keys():
        for cargo_ in selected_cargo.keys():
            destination_cargo = selected_cargo[cargo_][4]
            if destination_cargo in node_list_truck_hubs[truck_]:
                MP.addConstr(
                    quicksum(x[(node_, destination_cargo, truck_)] * 1
                             for node_ in node_list_truck_hubs[truck_]
                             if node_ != destination_cargo)
                    >= 
                    y[(truck_, cargo_)]
                )
    
    ### Capacity ----------------------------------------------------
    
    # capacity constraints (3.14)
    for truck_ in created_truck_all.keys():
        # if truck_ is a NON-cycle truck and node_ is its destination
        # then the truck capacity when departing its destination is 0
        if truck_ in created_truck_nCycle.keys():
            destination_truck = created_truck_all[truck_][1]
            MP.addConstr(
                S[(destination_truck, truck_)] 
                == 0
            )
            
    # Cumulative total size of a truck at a node (3.15)
    # be aware of whether the node is 
    # a cargo origin or cargo destination, or both
    bigM_capacity = 30000
    for truck_ in created_truck_all.keys():
        for node1 in node_list_truck_hubs[truck_]:
            for node2 in node_list_truck_hubs[truck_]:
                if node1 != node2:
                    # if truck_ is a cycle truck 
                    # and node2 is its destination
                    if truck_ in created_truck_yCycle.keys():
                        if node2 == created_truck_yCycle[truck_][1]:
                            MP.addConstr(
                                Sb[(node2, truck_)] - S[(node1, truck_)]
                                >= 
                                quicksum(y[(truck_, cargo_)] * 
                                node_cargo_size_change[(node2, cargo_)]
                                for cargo_ in selected_cargo.keys()
                                if selected_cargo[cargo_][4] == node2)
                                - bigM_capacity 
                                * (1 - x[(node1, node2, truck_)])
                            )
                        else:
                            MP.addConstr(
                                S[(node2, truck_)] - S[(node1, truck_)]
                                >= 
                                quicksum(y[(truck_, cargo_)] * 
                                node_cargo_size_change[(node2, cargo_)]
                                for cargo_ in selected_cargo.keys()
                                if selected_cargo[cargo_][4] == node2 \
                                or selected_cargo[cargo_][3] == node2)
                                - bigM_capacity 
                                * (1 - x[(node1, node2, truck_)])
                            )
                    # else
                    else:
                        if node2 == created_truck_nCycle[truck_][1]:
                            MP.addConstr(
                                S[(node2, truck_)] - S[(node1, truck_)]
                                >= 
                                quicksum(y[(truck_, cargo_)] * 
                                node_cargo_size_change[(node2, cargo_)]
                                for cargo_ in selected_cargo.keys()
                                if selected_cargo[cargo_][4] == node2)
                                - bigM_capacity 
                                * (1 - x[(node1, node2, truck_)])
                            )
                        else:
                            MP.addConstr(
                                S[(node2, truck_)] - S[(node1, truck_)]
                                >= 
                                quicksum(y[(truck_, cargo_)] * 
                                node_cargo_size_change[(node2, cargo_)]
                                for cargo_ in selected_cargo.keys()
                                if selected_cargo[cargo_][4] == node2 \
                                or selected_cargo[cargo_][3] == node2)
                                - bigM_capacity 
                                * (1 - x[(node1, node2, truck_)])
                            )
    # Change 20220911 TAN
    # Add total size of cargo <= M * sum_j x^k(i,j)
    for truck_ in created_truck_all.keys():
        for node1 in node_list_truck_hubs[truck_]:
            # if truck_ is a cycle truck 
            # and node2 is its destination
            MP.addConstr(
                S[(node1, truck_)]<= 
                bigM_capacity * quicksum(x[(node1, node2, truck_)]\
                        for node2 in node_list_truck_hubs[truck_] if node1 != node2))

    # total size of cargos at truck origins (3.16)  
    # Should be an equality constraint
    for truck_ in created_truck_all.keys():
        origin_truck = created_truck_all[truck_][0]
        MP.addConstr(
            S[(origin_truck, truck_)] == 
            quicksum(y[(truck_, cargo_)] * \
                     node_cargo_size_change[(origin_truck, cargo_)]
                     for cargo_ in selected_cargo.keys()
                     if selected_cargo[cargo_][3] == origin_truck)
        )
    
    ### Time --------------------------------------------------------
    # The arrival time of a truck at any node (even if not visited) 
    # is less than or equal to the departure time of a truck
    for truck_ in created_truck_all.keys():
        for node_ in node_list_truck_hubs[truck_]:
            if truck_ in created_truck_yCycle.keys() and \
               node_ == created_truck_yCycle[truck_][1]:
                MP.addConstr(
                    Ab[(node_, truck_)] 
                    <= Db[(node_, truck_)]
                )
                MP.addConstr(
                    A[(node_, truck_)] 
                    <= D[(node_, truck_)]
                )
            else:
                MP.addConstr(
                    A[(node_, truck_)] 
                    <= D[(node_, truck_)]
                )
    
    # loading and unloading time between arrival and departure (3.17)
    # Don't consider origin_truck in this constraint
    # but for cycle trucks, their origins are also their destinations
    # so we only consider their destination parts
    for truck_ in created_truck_all.keys():
        for node_ in node_list_truck_hubs[truck_]:
            # if truck_ is a cycle truck
            if truck_ in created_truck_yCycle.keys():
                # if node_ is its destination
                if node_ == created_truck_yCycle[truck_][1]:
                    MP.addConstr(
                        Ab[(node_, truck_)] +
                        constant['node_fixed_time'] +
                        quicksum(y[(truck_, cargo_)] * 
                                 int(np.ceil(selected_cargo[cargo_][0] * 
                                 constant['loading_variation_coefficient']))
                                 for cargo_ in selected_cargo.keys()
                                 if node_ == selected_cargo[cargo_][4])
                        <= Db[(node_, truck_)]
                    )
                else:
                    MP.addConstr(
                        A[(node_, truck_)] +
                        constant['node_fixed_time'] +
                        quicksum(y[(truck_, cargo_)] * 
                                 int(np.ceil(selected_cargo[cargo_][0] * 
                                 constant['loading_variation_coefficient']))
                                 for cargo_ in selected_cargo.keys()
                                 if node_ == selected_cargo[cargo_][3] \
                                 or node_ == selected_cargo[cargo_][4])
                        <= D[(node_, truck_)]
                    )
            # if truck_ is a non-cycle truck
            else:
                if node_ != created_truck_all[truck_][0]:
                    if node_ == created_truck_all[truck_][1]:
                        MP.addConstr(
                            A[(node_, truck_)] +
                            constant['node_fixed_time'] +
                            quicksum(y[(truck_, cargo_)] * 
                                     int(np.ceil(selected_cargo[cargo_][0] * 
                                     constant['loading_variation_coefficient']))
                                     for cargo_ in selected_cargo.keys()
                                     if node_ == selected_cargo[cargo_][4])
                            <= D[(node_, truck_)]
                        )
                    else:
                        MP.addConstr(
                            A[(node_, truck_)] +
                            constant['node_fixed_time'] +
                            quicksum(y[(truck_, cargo_)] * 
                                     int(np.ceil(selected_cargo[cargo_][0] * 
                                     constant['loading_variation_coefficient']))
                                     for cargo_ in selected_cargo.keys()
                                     if node_ == selected_cargo[cargo_][3] \
                                     or node_ == selected_cargo[cargo_][4])
                            <= D[(node_, truck_)]
                        )
    
    # bigM constraints for travel time on edge(i,j) (3.18) 
    # D[prev_node] + edge[(prev_node, curr_node)] <= A[curr_node]
    bigM_time = 2000
    for truck_ in created_truck_all.keys():
        for node1 in node_list_truck_hubs[truck_]:
            for node2 in node_list_truck_hubs[truck_]:
                if node1 != node2:
                    # if truck_ is a cycle truck and 
                    # node2 is its destination
                    if truck_ in created_truck_yCycle.keys() and \
                       node2 == created_truck_yCycle[truck_][1]:
                        MP.addConstr(
                            D[(node1, truck_)] +
                            selected_edge[(node1, node2)]
                            <= 
                            Ab[(node2, truck_)] +
                            bigM_time * (1 - x[(node1, node2, truck_)])
                        )
                    else:
                        MP.addConstr(
                            D[(node1, truck_)] +
                            selected_edge[(node1, node2)]
                            <= 
                            A[(node2, truck_)] +
                            bigM_time * (1 - x[(node1, node2, truck_)])
                        )
    
    # Earliest time window of cargos (3.19)
    for truck_ in created_truck_all.keys():
        for cargo_ in selected_cargo.keys():
            origin_cargo = selected_cargo[cargo_][3]
            if origin_cargo in node_list_truck_hubs[truck_]:
                MP.addConstr(
                    D[(origin_cargo, truck_)]
                    >= 
                    selected_cargo[cargo_][1] * y[(truck_, cargo_)]
                )
            
    # Latest time window of cargos (3.20)
    for truck_ in created_truck_all.keys():
        for cargo_ in selected_cargo.keys():
            destination_cargo = selected_cargo[cargo_][4]
            if destination_cargo in node_list_truck_hubs[truck_]:
                # if truck_ is a cycle truck and 
                # destination_cargo is its destination
                if truck_ in created_truck_yCycle.keys() and \
                   destination_cargo == created_truck_yCycle[truck_][1]:
                    MP.addConstr(
                        Ab[(destination_cargo, truck_)]
                        <= 
                        selected_cargo[cargo_][2] + 
                        bigM_time * (1 - y[(truck_, cargo_)])
                    )
                else:
                    MP.addConstr(
                        A[(destination_cargo, truck_)]
                        <= 
                        selected_cargo[cargo_][2] + 
                        bigM_time * (1 - y[(truck_, cargo_)])
                    )

    # maximum worktime of trucks (3.21)
    for truck_ in created_truck_all.keys():
        origin_truck = created_truck_all[truck_][0]
        destination_truck = created_truck_all[truck_][1]
        # if truck_ is a cycle truck
        if truck_ in created_truck_yCycle.keys():
            MP.addConstr(
                Db[(destination_truck, truck_)] - D[(origin_truck, truck_)]
                <= 
                created_truck_yCycle[truck_][2]  # z[truck_] * 
            )
            MP.addConstr(
                Db[(destination_truck, truck_)] - D[(origin_truck, truck_)]
                >= 
                0  # z[truck_] * 
            )
        else:
            MP.addConstr(
                D[(destination_truck, truck_)] - D[(origin_truck, truck_)]
                <= 
                created_truck_all[truck_][2]  # z[truck_] * 
            )
            MP.addConstr(
                D[(destination_truck, truck_)] - D[(origin_truck, truck_)]
                >= 
                0  # z[truck_] * 
            )

    
    # first pickup and then delivery (3.22)
    for truck_ in created_truck_all.keys():
        for cargo_ in selected_cargo.keys():
            origin_cargo = selected_cargo[cargo_][3]
            destination_cargo = selected_cargo[cargo_][4]
            if destination_cargo in node_list_truck_hubs[truck_]:
                # if truck_ is a cycle truck and 
                # destination_cargo is its destination
                if truck_ in created_truck_yCycle.keys() and \
                   destination_cargo == created_truck_yCycle[truck_][1]:
                    MP.addConstr(
                        Ab[(destination_cargo, truck_)] - 
                        D[(origin_cargo, truck_)]
                        >= 
                        selected_edge[(origin_cargo, destination_cargo)] -
                        bigM_time * (1 - y[(truck_, cargo_)])
                    )
                    MP.addConstr(
                        A[(origin_cargo, truck_)] - 
                        D[(destination_cargo, truck_)]
                        >= 
                        selected_edge[(destination_cargo, origin_cargo)] -
                        bigM_time * (1 - y[(truck_, cargo_)])
                    )
                else:
                    MP.addConstr(
                        A[(destination_cargo, truck_)] - 
                        D[(origin_cargo, truck_)]
                        >= 
                        selected_edge[(origin_cargo, destination_cargo)] -
                        bigM_time * (1 - y[(truck_, cargo_)])
                    )
            
            
            
            
    
    ###### Objective ######
    
    # cargo size cost: proportional to the total size of cargo carried by the only truck
    cost_cargo_size = quicksum(y[truck_, cargo_] * selected_cargo[cargo_][0] * 
                               constant['truck_running_cost'] * 1
                               for truck_ in created_truck_all.keys()
                               for cargo_ in selected_cargo.keys())
    
    # cargo number cost: proportional to the number of cargo carried by the only truck
    cost_cargo_number = quicksum(y[truck_, cargo_] * 
                                 constant['truck_running_cost'] * 1000
                                 for truck_ in created_truck_all.keys()
                                 for cargo_ in selected_cargo.keys())
    
    # # traveling cost: proportional to total travel time
    # cost_travel = quicksum(x[(node1, node2, truck_)] * 
    #                        selected_edge[(node1, node2)] * 
    #                        constant['truck_running_cost']
    #                        for truck_ in created_truck_all.keys()
    #                        for node1 in node_list_truck_hubs[truck_]
    #                        for node2 in node_list_truck_hubs[truck_]
    #                        if node1 != node2)
    
    # # deviation cost: proportional to total deviation of cargo carried by the only truck
    # # we don't include it in the objective
    # # ask Oksana about how to compute a deviation for a cargo and a truck
    # cost_deviation = quicksum(y[truck_, cargo_] * 1 *
    #                           single_truck_deviation[(cargo_, truck_)] *
    #                           constant['truck_running_cost']
    #                           for truck_ in created_truck_all.keys()
    #                           for cargo_ in selected_cargo.keys())
    
    
    ###### Integrate the model and optimize ######
#     cost_travel 
    MP.setObjective(cost_cargo_size + cost_cargo_number)
    MP.modelSense = GRB.MAXIMIZE
    MP.Params.timeLimit = runtime
    MP.Params.OutputFlag = 1
    MP.Params.LogFile = filename
    # set Params.Heuristics to 0.5 
    # such that it better finds feasible solution
    MP.Params.Heuristics = 0.5
    MP.Params.LogToConsole  = 0
    MP.update()
    MP.optimize()
    
    # if infeasible
    if MP.Status == 3:
        if verbose >0: print('+++ MIP [Infeasible Proved] ')
        return -1, runtime, [], [], [], [], [], [], [], [], [], \
               [], [], [], [], -1, -1, -1, -1, -1, -1, -1, -1
    
    runtime_MP = MP.Runtime
    obj_val_MP = MP.objVal

    
    # if no objective value
    if float('inf') == obj_val_MP:
        if verbose >0: print('+++ MIP [Infeasible] ')
        return -1, runtime, [], [], [], [], [], [], [], [], [], \
               [], [], [], [], -1, -1, -1, -1, -1, -1, -1, -1
        
    
    if verbose > 0:
        print('+++ MP [Feasible] ')
        print("   [Gurobi obj value] is %i" % obj_val_MP)
        print("   [The ]Gurobi runtime] is %f" % runtime_MP)
    
    
    
    ###### Get solutions ######
    
    # store all values in a list: sol
    sol = []
    for ss in MP.getVars():
        sol.append(int(ss.x))
        
    # retrieve values from the list sol
    count = 0
    # binary variables x
    x_sol = {}
    for truck_ in created_truck_all.keys():
        for i in node_list_truck_hubs[truck_]:
            for j in node_list_truck_hubs[truck_]:
                if i != j:
                    x_sol[(i, j, truck_)] = sol[count]
                    count += 1
                
    # binary variables y
    y_sol = {}
    for truck_ in created_truck_all.keys():
        for cargo_ in selected_cargo.keys():
            y_sol[(truck_, cargo_)] = sol[count]
            count += 1
    
    # integer variable S
    S_sol = {}
    Sb_sol = {}
    for truck_ in created_truck_all.keys():
        for node_ in node_list_truck_hubs[truck_]:
            S_sol[(node_, truck_)] = sol[count]
            count += 1
    # if truck_ is a cycle truck and node_ is its destination
    for truck_ in created_truck_yCycle.keys():
        node_ = created_truck_yCycle[truck_][1]
        Sb_sol[(node_, truck_)] = 0
            
    # integer variable D
    D_sol = {}
    Db_sol = {}
    for truck_ in created_truck_all.keys():
        for node_ in node_list_truck_hubs[truck_]:
            D_sol[(node_, truck_)] = sol[count]
            count += 1
    # if truck_ is a cycle truck and node_ is its destination
    for truck_ in created_truck_yCycle.keys():
        node_ = created_truck_yCycle[truck_][1]
        Db_sol[(node_, truck_)] = sol[count]
        count += 1
    
    # integer variable A
    A_sol = {}
    Ab_sol = {}
    for truck_ in created_truck_all.keys():
        for node_ in node_list_truck_hubs[truck_]:
            A_sol[(node_, truck_)] = sol[count]
            count += 1
    # if truck_ is a cycle truck and node_ is its destination
    for truck_ in created_truck_yCycle.keys():
        node_ = created_truck_yCycle[truck_][1]
        Ab_sol[(node_, truck_)] = sol[count]
        count += 1
    
    if verbose > 1:
        print('+++ The x_sol:')
        for key, value in x_sol.items():
            if value == 1:
                print(f'        {key, value}')
        print('+++ The y_sol:')
        for key, value in y_sol.items():
            if value == 1:
                print(f'        {key, value}')

    # cargo cost: proportional to the total size of cargo carried by the only truck
    cost_cargo_size_value = 0
    for truck_ in created_truck_all.keys():
        for cargo_ in selected_cargo.keys():
            cost_cargo_size_value += \
            y_sol[truck_, cargo_] * selected_cargo[cargo_][0] * \
            constant['truck_running_cost'] * 1
    if verbose >1:
        print('+++ [cost_cargo_size_value] ', cost_cargo_size_value)
    
    # cargo number cost: proportional to the number of cargo carried by the only truck
    cost_cargo_number_value = 0
    for truck_ in created_truck_all.keys():
        for cargo_ in selected_cargo.keys():
            cost_cargo_number_value += \
            y_sol[truck_, cargo_] * constant['truck_running_cost'] * 1000
    if verbose >1:
        print('+++ [cost_cargo_number_value] ', cost_cargo_number_value)
    
    # traveling cost: proportional to total travel time
    cost_travel_value = 0
    for truck_ in created_truck_all.keys():
        for node1 in node_list_truck_hubs[truck_]:
            for node2 in node_list_truck_hubs[truck_]:
                if node1 != node2:
                    cost_travel_value += x_sol[(node1, node2, truck_)] * \
                    selected_edge[(node1, node2)] * \
                    constant['truck_running_cost']
    if verbose >1:
        print('+++ [cost_travel_value] ', cost_travel_value)
    
    # deviation cost: proportional to total deviation of cargo carried by the only truck
    cost_deviation_value = 0
    for truck_ in created_truck_all.keys():
        for cargo_ in selected_cargo.keys():
            cost_deviation_value += \
            y_sol[truck_, cargo_] * 1 * \
            single_truck_deviation[(cargo_, truck_)] * \
            constant['truck_running_cost']
    if verbose >1:
        print('+++ [cost_deviation_value] ', cost_deviation_value, '\n')
    
    
         
    del MP
    
    
    return obj_val_MP, runtime_MP, \
           x_sol, {}, y_sol, S_sol, D_sol, A_sol, \
           Sb_sol, Db_sol, Ab_sol, \
           cost_cargo_size_value, cost_cargo_number_value, \
           cost_travel_value, cost_deviation_value

