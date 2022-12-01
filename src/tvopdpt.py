import time
from gurobipy import Model, quicksum, GRB
import numpy as np




    
def tvopdpt_milp_gurobi(constant, 
                        selected_cargo, 
                        selected_truck, 
                        created_truck_yCycle, 
                        created_truck_nCycle,
                        selected_node, 
                        selected_edge, 
                        node_cargo_size_change, 
                        runtime, filename, 
                        verbose = 0,
                        early_termination_timelimit = None):
    
    """
    code a MILP model for two-veihcle orienteering pick-up and delivery problem with transfer (TVOPDPT) in Gurobi
    input:
        constant: dict of parameter values
        selected_cargo: dict of cargos for TVOPDPT
                cargo['nb_cargo'] = ['size', 'lb_time', 'ub_time', 'departure_node', 'arrival_node']
        selected_truck: dict of trucks for TVOPDPT, selected_truck = created_truck_yCycle U created_truck_nCycle
                truck['nb_truck'] = ['departure_node', 'arrival_node', 'max_worktime', 'max_capacity']
        created_truck_yCycle: dict of trucks whose origin and destination nodes are the same
        created_truck_nCycle: dict of trucks whose origin and destination nodes are different
        selected_node: list of nodes for TVOPDPT
        selected_edge: dict of edges for TVOPDPT, selected_edge[(node1, node2)]
        node_cargo_size_change: dict of cargo size change at each node
                node_cargo_size_change[(node_, cargo_)] =  0 if node is not cargo origin or destination
                                                        =  cargo_size if n is cargo origin 
                                                        = -cargo_size if n is destination
    return 
        obj_val: objective value
        model_runtime: runtime
        x_sol, z_sol, S_sol, D_sol, A_sol, \
        Sb_sol, Db_sol, Ab_sol, u_sol, w_sol
        
    """
    def early_termination_callback(model, where):
        if where == GRB.Callback.MIPNODE:
            # Get model objective
            obj = model.cbGet(GRB.Callback.MIPNODE_OBJBST)
            if abs(obj - model._cur_obj) > 1e-8:
                # If so, update incumbent and time
                model._cur_obj = obj
                model._time = time.time()
        # Terminate if objective has not improved in 20s
        if time.time() - model._time > early_termination_timelimit:
            MP._termination_flag = 1
            model.terminate()

    MP = Model("Gurobi MIP for TVOPDPT")    
    
    ###### Decision Variables ######
    ###### six types of decision variables ######
    
    # binary variables x
    # ==1 if the truck_ traverse an edge 
    x = {}
    for truck_ in selected_truck.keys():
        for i in selected_node:
            for j in selected_node:
                if i != j:
                    x[(i, j, truck_)] = \
                    MP.addVar(vtype=GRB.BINARY)


    # Integer variables S
    # current total size of cargos on truck_ at node_
    S = {}
    Sb = {}
    for truck_ in selected_truck.keys():
        for node_ in selected_node:
            S[(node_, truck_)] = MP.addVar(vtype=GRB.INTEGER, lb=0,
                                           ub=selected_truck[truck_][3])
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
    for truck_ in selected_truck.keys():
        for node_ in selected_node:
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
    for truck_ in selected_truck.keys():
        for node_ in selected_node:
            A[(node_, truck_)] = MP.addVar(vtype=GRB.INTEGER, lb=0)
    # if truck_ is a cycle truck and node_ is its destination
    # add a decision variable Db
    for truck_ in created_truck_yCycle.keys():
        node_ = created_truck_yCycle[truck_][1]
        Ab[(node_, truck_)] = MP.addVar(vtype=GRB.INTEGER, lb=0)
    
    u, w, z = {}, {}, {}
    #u[(node_, cargo_)] = 1 if cargo_ is transferred from k1 to k2 at node_
    for node_ in selected_node:
        for cargo_ in selected_cargo.keys():
            u[(node_, cargo_)] = MP.addVar(vtype=GRB.BINARY)
    for node_ in selected_node: 
        for cargo_ in selected_cargo.keys():
            w[(node_, cargo_)] = MP.addVar(vtype=GRB.BINARY)
    for edge_ in selected_edge.keys():
        for truck_ in selected_truck.keys():
            for cargo_ in selected_cargo.keys():
                if edge_[0] != edge_[1]:
                    z[(edge_[0], edge_[1], truck_, cargo_)] = \
                    MP.addVar(vtype=GRB.BINARY)

    ###### Constraints ######
    ###### Distinguish cycle trucks and non-cycle trucks ######
    

    # constraints related to transfer variables

    # sum_i u^r_i  + sum_i w^r_i <=1, each cargo can be tranfers at most once
    for cargo_key in selected_cargo.keys():
        MP.addConstr(
            quicksum(w[(node_, cargo_key)] for node_ in selected_node) + quicksum(u[(node_, cargo_key)] for node_ in selected_node)  <= 1
        )


    # # sum_i u^r_i <=1, sum_i w^r_i <=1
    # for cargo_key in selected_cargo.keys():
    #     MP.addConstr(
    #         quicksum(u[(node_, cargo_key)] for node_ in selected_node) <= 1
    #     )
    #     MP.addConstr(
    #         quicksum(w[(node_, cargo_key)] for node_ in selected_node) <= 1
    #     )
    # # u[(node_, cargo_)] + w[(node_, cargo_)] <=1
    # for cargo_key in selected_cargo.keys():
    #     for node_ in selected_node:
    #         w[(node_, cargo_key)] + u[(node_, cargo_key)] <= 1 

    truck_1, truck_2 = selected_truck.keys()
    for cargo_key, cargo_value in selected_cargo.items():
        cargo_origin, cargo_destination = cargo_value[-2], cargo_value[-1]
        
        MP.addConstr(u[(cargo_origin, cargo_key)] == 0)                # do not transfer at cargo's origin
        MP.addConstr(u[(cargo_destination, cargo_key)] == 0)           # do not transfer at cargo's destination
        MP.addConstr(u[(selected_truck[truck_1][0], cargo_key)] == 0)  # do not tranfer from truck 1 to truck 2 at truck1's origin
        MP.addConstr(u[(selected_truck[truck_2][1], cargo_key)] == 0)  # ddo not tranfer from truck 1 to truck 2 at truck2's dest

        MP.addConstr(w[(cargo_origin, cargo_key)] == 0)                # do not transfer at cargo's origin
        MP.addConstr(w[(cargo_destination, cargo_key)] == 0)           # do not transfer at cargo's destination
        MP.addConstr(w[(selected_truck[truck_2][0], cargo_key)] == 0)  # do not tranfer from truck 2 to truck 1 at truck2's origin
        MP.addConstr(w[(selected_truck[truck_1][1], cargo_key)] == 0)  # do not tranfer from truck 2 to truck 1 at truck1's dest



    
    for cargo_key, cargo_value in selected_cargo.items():
        cargo_origin, cargo_destination = cargo_value[-2], cargo_value[-1]
        for node_ in selected_node:
            if node_ not in [cargo_origin, cargo_destination, selected_truck[truck_1][0], selected_truck[truck_2][1]]:
                # u[node_, cargo_] = sum_j z^{k1r}_{ji} - sum_j z^{k1r}_{ij}
                # except cargo_origin, cargo_destination, truck1_origin, truck2_destination
                MP.addConstr( # update nov. 29th
                    u[(node_, cargo_key)] == quicksum(z[(node_prev, node_, truck_1, cargo_key )]
                                               for node_prev in selected_node if node_prev!= node_)
                                        - quicksum(z[(node_, node_next, truck_1, cargo_key )]
                                               for node_next in selected_node if node_next!= node_)
                )
            if node_ not in [cargo_origin, cargo_destination, selected_truck[truck_2][0], selected_truck[truck_1][1]]:
                
                MP.addConstr(
                    w[(node_, cargo_key)] == quicksum(z[(node_prev, node_, truck_2, cargo_key )]
                                               for node_prev in selected_node if node_prev!= node_)
                                        - quicksum(z[(node_, node_next, truck_2, cargo_key )]
                                               for node_next in selected_node if node_next!= node_)
                )


    for cargo_ in selected_cargo.keys():
    # at most one cargo flow from cargo's origin
        origin_cargo = selected_cargo[cargo_][3]
        MP.addConstr( 
            quicksum(z[(origin_cargo, succ_node, truck_, cargo_)] * 1
                     for succ_node in selected_node 
                     if succ_node != origin_cargo
                     for truck_ in selected_truck.keys())
                     <= 1 
        )
    # no cargo flow to cargo's origin
        MP.addConstr( 
            quicksum(z[(succ_node, origin_cargo, truck_, cargo_)] * 1
                     for succ_node in selected_node 
                     if succ_node != origin_cargo
                     for truck_ in selected_truck.keys())
                     == 0 
        )
    # at most one cargo flow to cargo's dest
        destination_cargo = selected_cargo[cargo_][4]
        MP.addConstr( 
            quicksum(z[(pred_node, destination_cargo, truck_, cargo_)] * 1
                     for pred_node in selected_node 
                     if pred_node != destination_cargo
                     for truck_ in selected_truck.keys())
                     <= 1 
        )
    # no cargo flow from cargo's destination
        MP.addConstr( 
            quicksum(z[(destination_cargo, pred_node, truck_, cargo_)] * 1
                     for pred_node in selected_node 
                     if pred_node != destination_cargo
                     for truck_ in selected_truck.keys())
                     == 0 
        )
    
    # added on Nov. 29th
    for cargo_ in selected_cargo.keys():
        origin_cargo = selected_cargo[cargo_][3]
        destination_cargo = selected_cargo[cargo_][4]
        for node_ in selected_node:
            if node_ != origin_cargo and node_ != destination_cargo:
                MP.addConstr(
                    quicksum(z[(pred_node, node_, truck_, cargo_)] * 1
                             for pred_node in selected_node 
                             if pred_node != node_
                             for truck_ in selected_truck.keys())
                    ==
                    quicksum(z[(node_, succ_node, truck_, cargo_)] * 1
                             for succ_node in selected_node 
                             if succ_node != node_
                             for truck_ in selected_truck.keys())
                )

    # added nov. 29th
    for cargo_key, cargo_value in selected_cargo.items():
        cargo_origin, cargo_dest = cargo_value[-2], cargo_value[-1]
        MP.addConstr( 
              quicksum(z[(cargo_origin, node_next, truck_1, cargo_key )] + z[(cargo_origin, node_next, truck_2, cargo_key )] 
                        for node_next in selected_node if node_next!= cargo_origin)
             == quicksum(z[(node_prev, cargo_dest, truck_1, cargo_key )] + z[(node_prev, cargo_dest, truck_2, cargo_key )] 
                        for node_prev in selected_node if node_prev!= cargo_dest)
        )

    # z^{k1r}_{ij} + z^{k2r}_{ij} <=1

    for node_curr in selected_node:
        for node_next in selected_node:
            if node_curr != node_next:
                for cargo_key in selected_cargo.keys():
                    MP.addConstr(
                        z[node_curr, node_next, truck_1, cargo_key] + z[node_curr, node_next, truck_2, cargo_key]
                        <= 1
                    )
    
    # added nov. 29th
    # cargo can arrive at cargo_dest or leave cargo_origin in only one truck
    for cargo_key, cargo_value in selected_cargo.items():
        cargo_origin, cargo_dest = cargo_value[-2], cargo_value[-1]
        MP.addConstr( quicksum(   z[cargo_origin, node_next, truck_1, cargo_key] 
                                + z[cargo_origin, node_next, truck_2, cargo_key] 
                                for node_next in selected_node if node_next != cargo_origin)
                    <=1
        )
        MP.addConstr( quicksum(   z[node_prev, cargo_dest, truck_1, cargo_key] 
                                + z[node_prev, cargo_dest, truck_2, cargo_key] 
                                for node_prev in selected_node if cargo_dest != node_prev)
                    <=1
        )
        
    # Flow constraints (3.1)
    # the truck must start from its origin
    for truck_ in selected_truck.keys():
        origin_truck = selected_truck[truck_][0]
        if origin_truck in selected_node:
            MP.addConstr( 
                quicksum(x[(origin_truck, succ_node, truck_)] for succ_node in selected_node if succ_node != origin_truck) == 1 
            )
        
    # Flow constraints (3.2)  
    # only applies to non-cycle trucks
    # no flow enters the origin of a non-cycle truck
    for truck_ in created_truck_nCycle.keys():
        origin_truck = created_truck_nCycle[truck_][0]
        if origin_truck in selected_node:
            MP.addConstr( 
                quicksum(x[(succ_node, origin_truck, truck_)] for succ_node in selected_node if succ_node != origin_truck) == 0 
            )

    # Flow constraints (3.3)
    # the truck must end at its destination
    for truck_ in selected_truck.keys():
        destination_truck = selected_truck[truck_][1]
        if destination_truck in selected_node:
            MP.addConstr( 
                quicksum(x[(node_perv, destination_truck, truck_)] for node_perv in selected_node if node_perv != destination_truck) == 1
            )    
        
    # Flow constraints (3.4)
    # only applies to non-cycle trucks
    # no flow departs from the destination of a non-cycle truck
    for truck_ in created_truck_nCycle.keys():
        destination_truck = created_truck_nCycle[truck_][1]
        if destination_truck in selected_node:
            MP.addConstr( 
                quicksum(x[(destination_truck, node_next, truck_)] for node_next in selected_node if node_next != destination_truck) == 0 
            )
    
    
    ### No cycle part below ----------------------------------------------
    
    # Flow constraints (3.5)
    # flow in = flow out
    # Don't consider origin_truck and destination_truck in this constraint
    for truck_ in selected_truck.keys():
        origin_truck = selected_truck[truck_][0]
        destination_truck = selected_truck[truck_][1]
        for node_ in selected_node:
            if node_ != origin_truck and node_ != destination_truck:
                MP.addConstr( quicksum(x[(pred_node, node_, truck_)]  for pred_node in selected_node if pred_node != node_) 
                            == quicksum(x[(node_, succ_node, truck_)]  for succ_node in selected_node if succ_node != node_) 
                )
    
    # An edge is used at most once by a truck (3.6)
    # only apply for non-cycle trucks
    # and non-origin nodes for cycle trucks
    for truck_ in selected_truck.keys():
        if truck_ in created_truck_nCycle.keys():
            for i in selected_node:
                for j in selected_node:
                    if i != j:
                        MP.addConstr(
                            x[(i, j, truck_)] + x[(j, i, truck_)]
                            <= 1
                        )
        elif truck_ in created_truck_yCycle.keys():
            origin_truck = created_truck_yCycle[truck_][0]
            for i in selected_node:
                for j in selected_node:
                    if i != j:
                        if i != origin_truck and j != origin_truck:
                            MP.addConstr(
                                x[(i, j, truck_)] + x[(j, i, truck_)]
                                <= 1
                            )
            
    for node_curr in selected_node:
        for node_next in selected_node:
            if node_curr != node_next:
                for cargo_key in selected_cargo.keys():
                    for truck_key in selected_truck.keys():
                        MP.addConstr(
                            x[(node_curr, node_next, truck_key)] >= z[(node_curr, node_next, truck_key, cargo_key)]
                        )
    
    ### Capacity ----------------------------------------------------
    
    # capacity constraints (3.14)
    for truck_key in selected_truck.keys():
        # if truck_ is a NON-cycle truck and node_ is its destination
        # then the truck capacity when departing its destination is 0
        if truck_key in created_truck_nCycle.keys():
            destination_truck = selected_truck[truck_][1]
            MP.addConstr(
                S[(destination_truck, truck_key)] 
                == 0
            )

    truck_1, truck_2 = selected_truck.keys()
    truck_1_origin, truck_1_dest = selected_truck[truck_1][:2]
    truck_2_origin, truck_2_dest = selected_truck[truck_2][:2]

    bigM_capacity = 30000
    for node_curr in selected_node:
        for node_prev in selected_node:
            if node_curr != node_prev:
                if truck_1 in created_truck_yCycle.keys(): # if truck_1 is a cycle truck
                    if node_curr == truck_1_dest: 
                        # if node_curr is the destination of truck1
                        # then we can only deliver cargo, and transfer cargo to truck 2
                        
                        MP.addConstr(
                            Sb[(node_curr, truck_1)] - S[(node_prev, truck_1)] 
                            >=  quicksum(z[(node_prev, node_curr, truck_1, cargo_key)]* 
                                            node_cargo_size_change[(node_curr, cargo_)]
                                            for cargo_key, cargo_value in selected_cargo.items() 
                                            if cargo_value[4] == node_curr) # deliver cargo if node_curr is cargo's dest
                                # - bigM_capacity *(1 - x[(node_prev, node_curr, truck_1)])
                                - quicksum(u[(node_curr, cargo_key)]*cargo_value[0] # transfer to truck 2 if node_curr is not cargo's origin
                                           for cargo_key, cargo_value in selected_cargo.items() if node_curr != cargo_value[3]) 
                        )
                    elif node_curr != truck_1_origin and node_curr != truck_1_dest:
                         # if node_curr is not the destination or the origin of truck1
                          # then truck1 can deliver cargo, pickup cargo, transfer to truck2 and receive from truck 2
                        MP.addConstr(
                            S[(node_curr, truck_1)] - S[(node_prev, truck_1)] 
                            >=  quicksum(z[(node_prev, node_curr, truck_1, cargo_key)]* 
                                            node_cargo_size_change[(node_curr, cargo_)]
                                            for cargo_key, cargo_value in selected_cargo.items() 
                                            if cargo_value[4] == node_curr) # deliver cargo
                                + quicksum( z[(node_curr, node_next, truck_1, cargo_key)]* 
                                            node_cargo_size_change[(node_curr, cargo_)]
                                            for node_next in selected_node 
                                            for cargo_key, cargo_value in selected_cargo.items()
                                            if cargo_value[3] == node_curr and node_curr != node_next) # pickup cargo
                                # - bigM_capacity *(1 - x[(node_prev, node_curr, truck_1)])
                                - quicksum(u[(node_curr, cargo_key)]*cargo_value[0] # transfer to truck 2 if node_curr is not cargo's origin
                                           for cargo_key, cargo_value in selected_cargo.items() if node_curr != cargo_value[3]) 
                                + quicksum(w[(node_curr, cargo_key)]*cargo_value[0] # receive from truck 2 if node_curr is not cargo's dest
                                           for cargo_key, cargo_value in selected_cargo.items() if node_curr != cargo_value[4])
                        )
                else: # if truck 1 is a non-cucle truck
                    if node_curr == truck_1_dest: 
                        #if node_curr is the destination of truck1
                        # then we can only deliver cargo, and transfer cargo to truck 2

                        MP.addConstr(
                            S[(node_curr, truck_1)] - S[(node_prev, truck_1)] 
                            >=  quicksum(z[(node_prev, node_curr, truck_1, cargo_key)]* node_cargo_size_change[(node_curr, cargo_)]
                                            for cargo_key, cargo_value in selected_cargo.items() 
                                            if cargo_value[4] == node_curr) # deliver cargo
                                # - bigM_capacity *(1 - x[(node_prev, node_curr, truck_1)])
                                - quicksum(u[(node_curr, cargo_key)]*cargo_value[0] # transfer to other truck 2 if node_curr is not cargo's origin
                                           for cargo_key, cargo_value in selected_cargo.items() if node_curr != cargo_value[3] )
                        )
                    elif node_curr != truck_1_origin and node_curr != truck_1_dest:
                        # if node_curr is not the destination or the origin of truck1
                        # then truck1 can deliver cargo, pickup cargo, transfer to truck2 and receive from truck 2
                        MP.addConstr(
                            S[(node_curr, truck_1)] - S[(node_prev, truck_1)] 
                            >=  quicksum(z[(node_prev, node_curr, truck_1, cargo_key)]* node_cargo_size_change[(node_curr, cargo_)]
                                            for cargo_key, cargo_value in selected_cargo.items() 
                                            if cargo_value[4] == node_curr) # deliver cargo
                                + quicksum( z[(node_curr, node_next, truck_1, cargo_key)]* node_cargo_size_change[(node_curr, cargo_)]
                                            for node_next in selected_node 
                                            for cargo_key, cargo_value in selected_cargo.items()
                                            if cargo_value[3] == node_curr and node_curr != node_next) # pickup cargo
                                # - bigM_capacity *(1 - x[(node_prev, node_curr, truck_1)])
                                - quicksum(u[(node_curr, cargo_key)]*cargo_value[0] # transfer to truck 2 if node_curr is not cargo's origin
                                           for cargo_key, cargo_value in selected_cargo.items() if node_curr != cargo_value[3] )
                                + quicksum(w[(node_curr, cargo_key)]*cargo_value[0] # receive from truck 2 if node_curr is not cargo's dest
                                           for cargo_key, cargo_value in selected_cargo.items() if node_curr != cargo_value[4] )
                        )
                if truck_2 in created_truck_yCycle.keys(): # if truck_2 is a cycle truck
                    if node_curr == truck_2_dest: 
                        #if node_curr is the destination of truck2
                        # then we can only deliver cargo, and transfer cargo to truck 1
                        MP.addConstr(
                            Sb[(node_curr, truck_)] - S[(node_prev, truck_)] 
                            >=  quicksum(z[(node_prev, node_curr, truck_2, cargo_key)]* node_cargo_size_change[(node_curr, cargo_)]
                                            for cargo_key, cargo_value in selected_cargo.items() 
                                            if cargo_value[4] == node_curr) # delivery cargo
                                # - bigM_capacity *(1 - x[(node_prev, node_curr, truck_2)])
                                - quicksum(w[(node_curr, cargo_key)]*cargo_value[0] # transfer cargo to truck 1 if node_curr is not cargo's destination
                                           for cargo_key, cargo_value in selected_cargo.items() if node_curr != cargo_value[4] )
                        )
                    elif node_curr != truck_2_origin and node_curr != truck_2_dest:
                        # if node_curr is not the destination or the origin of truck1
                        # then truck1 can deliver cargo, pickup cargo, transfer to truck2 and receive from truck 2
                        MP.addConstr(
                            S[(node_curr, truck_)] - S[(node_prev, truck_)] 
                            >=  quicksum(z[(node_prev, node_curr, truck_2, cargo_key)]* 
                                            node_cargo_size_change[(node_curr, cargo_)]
                                            for cargo_key, cargo_value in selected_cargo.items() 
                                            if cargo_value[4] == node_curr) # delivery cargo
                                + quicksum( z[(node_curr, node_next, truck_2, cargo_key)]* node_cargo_size_change[(node_curr, cargo_)]
                                            for node_next in selected_node 
                                            for cargo_key, cargo_value in selected_cargo.items()
                                            if cargo_value[3] == node_curr and node_curr != node_next) # pickup cargo
                                # - bigM_capacity *(1 - x[(node_prev, node_curr, truck_2)])
                                - quicksum(w[(node_curr, cargo_key)]*cargo_value[0] # transfer cargo to truck 1 if node_curr is not cargo's destnation
                                           for cargo_key, cargo_value in selected_cargo.items() if node_curr != cargo_value[3] )
                                + quicksum(u[(node_curr, cargo_key)]*cargo_value[0] # receive cargo from truck 1 if node_curr is not cargo's dest
                                           for cargo_key, cargo_value in selected_cargo.items() if node_curr != cargo_value[4] )
                        )
                else: # if truck 1 is a non-cucle truck
                    if node_curr == truck_2_dest: 
                        #if node_curr is the destination of truck2
                        # then we can only deliver cargo, and transfer cargo to truck 1
                        MP.addConstr(
                            S[(node_curr, truck_)] - S[(node_prev, truck_)] 
                            >=  quicksum(z[(node_prev, node_curr, truck_2, cargo_key)]* node_cargo_size_change[(node_curr, cargo_)]
                                            for cargo_key, cargo_value in selected_cargo.items() 
                                            if cargo_value[4] == node_curr) # delivery cargo
                                # - bigM_capacity *(1 - x[(node_prev, node_curr, truck_2)])
                                - quicksum(w[(node_curr, cargo_key)]*cargo_value[0] # transfer cargo to truck 1 if node_curr is not cargo's destination
                                           for cargo_key, cargo_value in selected_cargo.items() if node_curr != cargo_value[4] )
                        )
                    elif node_curr != truck_2_origin and node_curr != truck_2_dest:
                        # if node_curr is not the destination or the origin of truck1
                        # then truck1 can deliver cargo, pickup cargo, transfer to truck2 and receive from truck 2
                        MP.addConstr(
                            S[(node_curr, truck_)] - S[(node_prev, truck_)] 
                            >=  quicksum(z[(node_prev, node_curr, truck_2, cargo_key)]* 
                                            node_cargo_size_change[(node_curr, cargo_)]
                                            for cargo_key, cargo_value in selected_cargo.items() 
                                            if cargo_value[4] == node_curr) # delivery cargo
                                + quicksum( z[(node_curr, node_next, truck_2, cargo_key)]* node_cargo_size_change[(node_curr, cargo_)]
                                            for node_next in selected_node 
                                            for cargo_key, cargo_value in selected_cargo.items()
                                            if cargo_value[3] == node_curr and node_curr != node_next) # pickup cargo
                                # - bigM_capacity *(1 - x[(node_prev, node_curr, truck_2)])
                                - quicksum(w[(node_curr, cargo_key)]*cargo_value[0] # transfer cargo to truck 1 if node_curr is not cargo's destnation
                                           for cargo_key, cargo_value in selected_cargo.items() if node_curr != cargo_value[3] )
                                + quicksum(u[(node_curr, cargo_key)]*cargo_value[0] # receive cargo from truck 1 if node_curr is not cargo's dest
                                           for cargo_key, cargo_value in selected_cargo.items() if node_curr != cargo_value[4] )
                        )
    
    # Change 20220911 TAN
    # Add total size of cargo <= M * sum_j x^k(i,j)
    for truck_ in selected_truck.keys():
        for node1 in selected_node:
            # if truck_ is a cycle truck 
            # and node2 is its destination
            MP.addConstr(
                S[(node1, truck_)]<= 
                bigM_capacity * quicksum(x[(node1, node2, truck_)]\
                        for node2 in selected_node if node1 != node2))

    # total size of cargos at truck origins (3.16)  
    # Should be an equality constraint
    truck_1, truck_2 = selected_truck.keys()
    truck_1_origin, truck_1_dest = selected_truck[truck_1][:2]
    truck_2_origin, truck_2_dest = selected_truck[truck_2][:2]

    # at truck_1 origin, we can only pickup cargo whose origin is the same as truck1's origin
    # and receive cargo from truck 2 if truck1's origin is not the cargo's dest
    MP.addConstr(
        S[(truck_1_origin, truck_1)] == 
        quicksum(z[(truck_1_origin, node_next, truck_1, cargo_key)] * \
                    node_cargo_size_change[(truck_1_origin, cargo_key)]
                    for node_next in selected_node if node_next != truck_1_origin
                    for cargo_key, cargo_value in selected_cargo.items()
                    if cargo_value[3] == truck_1_origin) # pickup cargo if truck1's origin = cargo's origin
        + quicksum(w[(node_curr, cargo_key)]*cargo_value[0] # receive cargo from truck 2 if truck_1_origin is not cargo's destination
                    for cargo_key, cargo_value in selected_cargo.items() if truck_1_origin != cargo_value[4] )    
        )

    MP.addConstr(
        S[(truck_2_origin, truck_2)] == 
        quicksum(z[(truck_2_origin, node_next, truck_2, cargo_key)] * \
                    node_cargo_size_change[(truck_2_origin, cargo_key)]
                    for node_next in selected_node if node_next != truck_2_origin
                    for cargo_key, cargo_value in selected_cargo.items()
                    if cargo_value[3] == truck_2_origin) # pickup cargo if truck2's origin = cargo's origin
        + quicksum(u[(node_curr, cargo_key)]*cargo_value[0] # transfer cargo to truck 1 if truck_2_origin is not cargo's destination
                    for cargo_key, cargo_value in selected_cargo.items() if truck_2_origin != cargo_value[4] )    
        )

    ### Time --------------------------------------------------------
    # Same as SVOPDP model
    # The arrival time of a truck at any node (even if not visited) 
    # is less than or equal to the departure time of a truck
    for truck_ in selected_truck.keys():
        for node_ in selected_node:
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
    

    for truck_key in selected_truck.keys():
        for node_curr in selected_node:
            if truck_key in created_truck_yCycle.keys(): # if truck_ is a cycle truck
                if node_curr == created_truck_yCycle[truck_key][1]: # if node_ is truck destination
                    #at^k_i + fix + sum_{j in V, r in R: i==r_dest} z^{kr}_{ji} * r_size * unit_load_time
                    #             + sum_{r in R} (u^r_i + w^r_i ) * r_size * unit_load_time
                    # <= dt^k_i
                    MP.addConstr(
                        Ab[(node_curr, truck_key)] + constant['node_fixed_time'] +
                        quicksum(z[(nod_prev, node_curr, truck_key, cargo_key)] * 
                                    int(np.ceil(cargo_value[0] * constant['loading_variation_coefficient']))
                                    for nod_prev in selected_node
                                    for cargo_key, cargo_value in selected_cargo.items()
                                    if node_curr == cargo_value[4]
                                )
                      + quicksum((u[(node_curr, cargo_key)] + w[(node_curr, cargo_key)])* 
                                    int(np.ceil(cargo_value[0] * constant['loading_variation_coefficient']))
                                    for cargo_key, cargo_value in selected_cargo.items()
                                )
                     <= Db[(node_curr, truck_key)]
                    )      
                elif node_curr == created_truck_yCycle[truck_key][0]: # if node_ is truck origin
                    #at^k_i + fix + sum_{j in V, r in R: i==r_origin} z^{kr}_{ij} * r_size * unit_load_time
                    #             + sum_{r in R} (u^r_i + w^r_i ) * r_size * unit_load_time
                    # <= dt^k_i
                    MP.addConstr(
                        A[(node_curr, truck_key)] + constant['node_fixed_time'] +
                        quicksum(z[(node_curr, node_next, truck_key, cargo_key)] * 
                                    int(np.ceil(cargo_value[0] * constant['loading_variation_coefficient']))
                                    for node_next in selected_node
                                    for cargo_key, cargo_value in selected_cargo.items()
                                    if node_curr == cargo_value[3]
                                )
                      + quicksum((u[(node_curr, cargo_key)] + w[(node_curr, cargo_key)])* 
                                    int(np.ceil(cargo_value[0] * constant['loading_variation_coefficient']))
                                    for cargo_key, cargo_value in selected_cargo.items()
                                )
                     <= D[(node_curr, truck_key)]
                    )  
                else: # if node_ not a truck origin or dest
                    #at^k_i + fix + sum_{j in V, r in R: i==r_origin} z^{kr}_{ij} * r_size * unit_load_time
                    #             + sum_{j in V, r in R: i==r_dest} z^{kr}_{ji} * r_size * unit_load_time 
                    #             + sum_{r in R} (u^r_i + w^r_i ) * r_size * unit_load_time
                    # <= dt^k_i
                    MP.addConstr(
                        A[(node_curr, truck_key)] + constant['node_fixed_time']
                      + quicksum(z[(node_curr, node_next, truck_key, cargo_key)] * 
                                    int(np.ceil(cargo_value[0] * constant['loading_variation_coefficient']))
                                    for node_next in selected_node
                                    for cargo_key, cargo_value in selected_cargo.items()
                                    if node_curr == cargo_value[3]
                                )
                      + quicksum(z[(node_prev, node_curr, truck_key, cargo_key)] * 
                                    int(np.ceil(cargo_value[0] * constant['loading_variation_coefficient']))
                                    for node_prev in selected_node
                                    for cargo_key, cargo_value in selected_cargo.items()
                                    if node_curr == cargo_value[4]
                                )
                      + quicksum((u[(node_curr, cargo_key)] + w[(node_curr, cargo_key)])* 
                                    int(np.ceil(cargo_value[0] * constant['loading_variation_coefficient']))
                                    for cargo_key, cargo_value in selected_cargo.items()
                                )
                     <= D[(node_curr, truck_key)]
                    )  
            else: # if truck is a non-cycle truck
                if node_curr == selected_truck[truck_key][1]: # if node_ is truck destination
                    #at^k_i + fix + sum_{j in V, r in R: i==r_dest} z^{kr}_{ji} * r_size * unit_load_time
                    #             + sum_{r in R} (u^r_i + w^r_i ) * r_size * unit_load_time
                    # <= dt^k_i
                    MP.addConstr(
                        A[(node_curr, truck_key)] + constant['node_fixed_time'] +
                        quicksum(z[(node_prev, node_curr, truck_key, cargo_key)] * 
                                    int(np.ceil(cargo_value[0] * constant['loading_variation_coefficient']))
                                    for node_prev in selected_node if node_prev != node_curr
                                    for cargo_key, cargo_value in selected_cargo.items()
                                    if node_curr == cargo_value[4]
                                )
                      + quicksum((u[(node_curr, cargo_key)] + w[(node_curr, cargo_key)])* 
                                    int(np.ceil(cargo_value[0] * constant['loading_variation_coefficient']))
                                    for cargo_key, cargo_value in selected_cargo.items()
                                )
                     <= D[(node_curr, truck_key)]
                    )      
                elif node_curr == selected_truck[truck_key][0]: # if node_ is truck origin
                    #at^k_i + fix + sum_{j in V, r in R: i==r_origin} z^{kr}_{ij} * r_size * unit_load_time
                    #             + sum_{r in R} (u^r_i + w^r_i ) * r_size * unit_load_time
                    # <= dt^k_i
                    MP.addConstr(
                        A[(node_curr, truck_key)] + constant['node_fixed_time'] +
                        quicksum(z[(node_curr, node_next, truck_key, cargo_key)] * 
                                    int(np.ceil(cargo_value[0] * constant['loading_variation_coefficient']))
                                    for node_next in selected_node if node_next != node_curr
                                    for cargo_key, cargo_value in selected_cargo.items()
                                    if node_curr == cargo_value[3]
                                )
                      + quicksum((u[(node_curr, cargo_key)] + w[(node_curr, cargo_key)])* 
                                    int(np.ceil(cargo_value[0] * constant['loading_variation_coefficient']))
                                    for cargo_key, cargo_value in selected_cargo.items()
                                )
                     <= D[(node_curr, truck_key)]
                    )  
                else: # if node_ not a truck origin or dest
                    #at^k_i + fix + sum_{j in V, r in R: i==r_origin} z^{kr}_{ij} * r_size * unit_load_time
                    #             + sum_{j in V, r in R: i==r_dest} z^{kr}_{ji} * r_size * unit_load_time 
                    #             + sum_{r in R} (u^r_i + w^r_i ) * r_size * unit_load_time
                    # <= dt^k_i
                    MP.addConstr(
                        A[(node_curr, truck_key)] + constant['node_fixed_time']
                      + quicksum(z[(node_curr, node_next, truck_key, cargo_key)] * 
                                    int(np.ceil(cargo_value[0] * constant['loading_variation_coefficient']))
                                    for node_next in selected_node if node_next != node_curr
                                    for cargo_key, cargo_value in selected_cargo.items()
                                    if node_curr == cargo_value[3]
                                )
                      + quicksum(z[(node_prev, node_curr, truck_key, cargo_key)] * 
                                    int(np.ceil(cargo_value[0] * constant['loading_variation_coefficient']))
                                    for node_prev in selected_node if node_prev != node_curr
                                    for cargo_key, cargo_value in selected_cargo.items()
                                    if node_curr == cargo_value[4]
                                )
                      + quicksum((u[(node_curr, cargo_key)] + w[(node_curr, cargo_key)])* 
                                    int(np.ceil(cargo_value[0] * constant['loading_variation_coefficient']))
                                    for cargo_key, cargo_value in selected_cargo.items()
                                )
                     <= D[(node_curr, truck_key)]
                    )  


    bigM_time = 2000
    truck_1, truck_2 = selected_truck.keys()
    for node_curr in selected_node:
        for cargo_ in selected_cargo.keys():
            if truck_1 in created_truck_yCycle.keys(): # if truck_1 is a cycle truck
                if truck_2 in created_truck_yCycle.keys(): # if truck_2 is a cycle truck
                    if node_curr == created_truck_yCycle[truck_1][1]: 
                        if node_curr == created_truck_yCycle[truck_2][1]: 
                            # node_curr is the dest of truck 1 and 2
                            MP.addConstr(
                                Db[(node_curr, truck_1)] >= Ab[(node_curr, truck_2)] 
                                + 2* quicksum(w[(node_curr, cargo_key)]* 
                                                int(np.ceil(cargo_value[0] * constant['loading_variation_coefficient']))
                                                for cargo_key, cargo_value in selected_cargo.items())
                                - bigM_time*(1-w[(node_curr, cargo_)])
                            )
                        else:
                            # node_curr is the dest of truck 1 not dest of truck 2
                            MP.addConstr(
                                Db[(node_curr, truck_1)] >= A[(node_curr, truck_2)] 
                                + 2* quicksum(w[(node_curr, cargo_key)]* 
                                                int(np.ceil(cargo_value[0] * constant['loading_variation_coefficient']))
                                                for cargo_key, cargo_value in selected_cargo.items())
                                - bigM_time*(1-w[(node_curr, cargo_)])
                            )
                    else:
                        if node_curr == created_truck_yCycle[truck_2][1]: 
                            # node_curr is the dest of truck 2, not the dest of truck 1
                            MP.addConstr(
                                D[(node_curr, truck_1)] >= Ab[(node_curr, truck_2)] 
                                + 2* quicksum(w[(node_curr, cargo_key)]* 
                                                int(np.ceil(cargo_value[0] * constant['loading_variation_coefficient']))
                                                for cargo_key, cargo_value in selected_cargo.items())
                                - bigM_time*(1-w[(node_curr, cargo_)])
                            )
                        else:
                            # node_curr is not the dest of truck 1 or 2
                            MP.addConstr(
                                D[(node_curr, truck_1)] >= A[(node_curr, truck_2)] 
                                + 2* quicksum(w[(node_curr, cargo_key)]* 
                                                int(np.ceil(cargo_value[0] * constant['loading_variation_coefficient']))
                                                for cargo_key, cargo_value in selected_cargo.items())
                                - bigM_time*(1-w[(node_curr, cargo_)])
                            )
                else: # truck 2 is a non-cycle truck
                    if node_curr == created_truck_yCycle[truck_1][1]: 
                            MP.addConstr(
                                Db[(node_curr, truck_1)] >= A[(node_curr, truck_2)] 
                                + 2* quicksum(w[(node_curr, cargo_key)]* 
                                                int(np.ceil(cargo_value[0] * constant['loading_variation_coefficient']))
                                                for cargo_key, cargo_value in selected_cargo.items())
                                - bigM_time*(1-w[(node_curr, cargo_)])
                            )                    
                    else:
                        # node_curr is not the dest of truck 1 or 2
                        MP.addConstr(
                            D[(node_curr, truck_1)] >= A[(node_curr, truck_2)] 
                            + 2* quicksum(w[(node_curr, cargo_key)]* 
                                            int(np.ceil(cargo_value[0] * constant['loading_variation_coefficient']))
                                            for cargo_key, cargo_value in selected_cargo.items())
                            - bigM_time*(1-w[(node_curr, cargo_)])
                        )
            else: # truck 1 is a non-cycle truck
                if truck_2 in created_truck_yCycle.keys(): # if truck_2 is a cycle truck
                    if node_curr == created_truck_yCycle[truck_2][1]: 
                        MP.addConstr(
                            D[(node_curr, truck_1)] >= Ab[(node_curr, truck_2)] 
                            + 2* quicksum(w[(node_curr, cargo_key)]* 
                                            int(np.ceil(cargo_value[0] * constant['loading_variation_coefficient']))
                                            for cargo_key, cargo_value in selected_cargo.items())
                            - bigM_time*(1-w[(node_curr, cargo_)])
                        )
                    else:
                        MP.addConstr(
                            D[(node_curr, truck_1)] >= A[(node_curr, truck_2)] 
                            + 2* quicksum(w[(node_curr, cargo_key)]* 
                                            int(np.ceil(cargo_value[0] * constant['loading_variation_coefficient']))
                                            for cargo_key, cargo_value in selected_cargo.items())
                            - bigM_time*(1-w[(node_curr, cargo_)])
                        )
                else:
                    MP.addConstr(
                        D[(node_curr, truck_1)] >= A[(node_curr, truck_2)] 
                        + 2* quicksum(w[(node_curr, cargo_key)]* 
                                        int(np.ceil(cargo_value[0] * constant['loading_variation_coefficient']))
                                        for cargo_key, cargo_value in selected_cargo.items())
                        - bigM_time*(1-w[(node_curr, cargo_)])
                    )
    

    # same as SVOPDP model
    # bigM constraints for travel time on edge(i,j) (3.18) 
    # D[prev_node] + edge[(prev_node, curr_node)] <= A[curr_node]
    for truck_ in selected_truck.keys():
        for node1 in selected_node:
            for node2 in selected_node:
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
    for truck_ in selected_truck.keys():
        for cargo_ in selected_cargo.keys():
            origin_cargo = selected_cargo[cargo_][3]
            if origin_cargo in selected_node:
                MP.addConstr(
                    D[(origin_cargo, truck_)]
                    >= 
                    selected_cargo[cargo_][1] * quicksum(z[(origin_cargo, node_next, truck_, cargo_)] 
                                                        for node_next in selected_node if node_next != origin_cargo)
                )
            
    # Latest time window of cargos (3.20)
    for truck_ in selected_truck.keys():
        for cargo_ in selected_cargo.keys():
            destination_cargo = selected_cargo[cargo_][4]
            if destination_cargo in selected_node:
                # if truck_ is a cycle truck and 
                # destination_cargo is its destination
                if truck_ in created_truck_yCycle.keys() and \
                   destination_cargo == created_truck_yCycle[truck_][1]:
                    MP.addConstr(
                        Ab[(destination_cargo, truck_)]
                        <= 
                        selected_cargo[cargo_][2] + 
                        bigM_time * (1 - quicksum(z[(node_prev, destination_cargo, truck_, cargo_)] 
                                                    for node_prev in selected_node if node_prev != destination_cargo)
                                    )
                    )
                else:
                    MP.addConstr(
                        A[(destination_cargo, truck_)]
                        <= 
                        selected_cargo[cargo_][2] + 
                        bigM_time * (1 - quicksum(z[(node_prev, destination_cargo, truck_, cargo_)] 
                                                    for node_prev in selected_node if node_prev != destination_cargo)
                                    )
                    )

    # maximum worktime of trucks (3.21)
    for truck_ in selected_truck.keys():
        origin_truck = selected_truck[truck_][0]
        destination_truck = selected_truck[truck_][1]
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
                selected_truck[truck_][2]  # z[truck_] * 
            )
            MP.addConstr(
                D[(destination_truck, truck_)] - D[(origin_truck, truck_)]
                >= 
                0  # z[truck_] * 
            )

    
    # # first pickup and then delivery (3.22)
    # if cargo is picked-up and delivered by the same truck
    for truck_ in selected_truck.keys():
        for cargo_ in selected_cargo.keys():
            origin_cargo = selected_cargo[cargo_][3]
            destination_cargo = selected_cargo[cargo_][4]
            if destination_cargo in selected_node:
                # if truck_ is a cycle truck and 
                # destination_cargo is its destination
                if truck_ in created_truck_yCycle.keys():
                    if destination_cargo == created_truck_yCycle[truck_][1]:
                        MP.addConstr(
                            Ab[(destination_cargo, truck_)] - D[(origin_cargo, truck_)]
                            >=  selected_edge[(origin_cargo, destination_cargo)] 
                            - bigM_time * (2 - quicksum(z[(node_prev, destination_cargo, truck_, cargo_)] 
                                                            for node_prev in selected_node if node_prev != destination_cargo)
                                            - quicksum(z[(origin_cargo, node_next, truck_, cargo_)] 
                                                            for node_next in selected_node if node_next != origin_cargo)
                                          )
                            # - bigM_time * quicksum(u[(node_, cargo_)] + w[(node_, cargo_)]for node_ in selected_node)
                        )
                    else:                    
                        MP.addConstr(
                        A[(destination_cargo, truck_)] -  D[(origin_cargo, truck_)]
                        >=  selected_edge[(origin_cargo, destination_cargo)] 
                        - bigM_time * (2 - quicksum(z[(node_prev, destination_cargo, truck_, cargo_)] 
                                                        for node_prev in selected_node if node_prev != destination_cargo)
                                            - quicksum(z[(origin_cargo, node_next, truck_, cargo_)] 
                                                            for node_next in selected_node if node_next != origin_cargo)
                                      )
                        # - bigM_time * quicksum(u[(node_, cargo_)] + w[(node_, cargo_)]for node_ in selected_node)
                        )
                else:
                    MP.addConstr(
                        A[(destination_cargo, truck_)] -  D[(origin_cargo, truck_)]
                        >=  selected_edge[(origin_cargo, destination_cargo)] 
                        - bigM_time * (2 - quicksum(z[(node_prev, destination_cargo, truck_, cargo_)] 
                                                        for node_prev in selected_node if node_prev != destination_cargo)
                                            - quicksum(z[(origin_cargo, node_next, truck_, cargo_)] 
                                                            for node_next in selected_node if node_next != origin_cargo)
                                      )
                        # - bigM_time * quicksum(u[(node_, cargo_)] + w[(node_, cargo_)]for node_ in selected_node)
                    )
    
    truck_1, truck_2 = selected_truck.keys()
    for cargo_key, cargo_value in selected_cargo.items():
        origin_cargo, destination_cargo = cargo_value[3], cargo_value[4]
        if truck_1 in created_truck_yCycle.keys():
            if destination_cargo == created_truck_yCycle[truck_1][1]:
                MP.addConstr(
                    Ab[(destination_cargo, truck_1)] - D[(origin_cargo, truck_2)]
                    >=  selected_edge[(origin_cargo, destination_cargo)] 
                    - bigM_time * (2 - quicksum(z[(node_prev, destination_cargo, truck_1, cargo_)] 
                                                    for node_prev in selected_node if node_prev != destination_cargo)
                                    - quicksum(z[(origin_cargo, node_next, truck_2, cargo_)] 
                                                    for node_next in selected_node if node_next != origin_cargo)
                                  )
                )
            else:
                MP.addConstr(
                    A[(destination_cargo, truck_1)] - D[(origin_cargo, truck_2)]
                    >=  selected_edge[(origin_cargo, destination_cargo)] 
                    - bigM_time * (2 - quicksum(z[(node_prev, destination_cargo, truck_1, cargo_)] 
                                                    for node_prev in selected_node if node_prev != destination_cargo)
                                    - quicksum(z[(origin_cargo, node_next, truck_2, cargo_)] 
                                                    for node_next in selected_node if node_next != origin_cargo)
                                  )
                )
        else:
                MP.addConstr(
                    A[(destination_cargo, truck_1)] - D[(origin_cargo, truck_2)]
                    >=  selected_edge[(origin_cargo, destination_cargo)] 
                    - bigM_time * (2 - quicksum(z[(node_prev, destination_cargo, truck_1, cargo_)] 
                                                    for node_prev in selected_node if node_prev != destination_cargo)
                                    - quicksum(z[(origin_cargo, node_next, truck_2, cargo_)] 
                                                    for node_next in selected_node if node_next != origin_cargo)
                                  )
                )

        if truck_2 in created_truck_yCycle.keys():
            if destination_cargo == created_truck_yCycle[truck_2][1]:
                MP.addConstr(
                    Ab[(destination_cargo, truck_2)] - D[(origin_cargo, truck_1)]
                    >=  selected_edge[(origin_cargo, destination_cargo)] 
                    - bigM_time * (2 - quicksum(z[(node_prev, destination_cargo, truck_2, cargo_)] 
                                                    for node_prev in selected_node if node_prev != destination_cargo)
                                    - quicksum(z[(origin_cargo, node_next, truck_1, cargo_)] 
                                                    for node_next in selected_node if node_next != origin_cargo)
                                  )
                )
            else:
                MP.addConstr(
                    A[(destination_cargo, truck_2)] - D[(origin_cargo, truck_1)]
                    >=  selected_edge[(origin_cargo, destination_cargo)] 
                    - bigM_time * (2 - quicksum(z[(node_prev, destination_cargo, truck_2, cargo_)] 
                                                    for node_prev in selected_node if node_prev != destination_cargo)
                                    - quicksum(z[(origin_cargo, node_next, truck_1, cargo_)] 
                                                    for node_next in selected_node if node_next != origin_cargo)
                                  )
                )
        else:
                MP.addConstr(
                    A[(destination_cargo, truck_2)] - D[(origin_cargo, truck_1)]
                    >=  selected_edge[(origin_cargo, destination_cargo)] 
                    - bigM_time * (2 - quicksum(z[(node_prev, destination_cargo, truck_2, cargo_)] 
                                                    for node_prev in selected_node if node_prev != destination_cargo)
                                    - quicksum(z[(origin_cargo, node_next, truck_1, cargo_)] 
                                                    for node_next in selected_node if node_next != origin_cargo)
                                  )
                )          

    # for cargo_key, cargo_value in selected_cargo.items():
    #     origin_cargo, destination_cargo = cargo_value[3], cargo_value[4]
    #     if truck_1 in created_truck_yCycle.keys():
    #         if destination_cargo == created_truck_yCycle[truck_1][1]:
    #             MP.addConstr(
    #                 Ab[(destination_cargo, truck_1)] - D[(origin_cargo, truck_2)]
    #                 >=  selected_edge[(origin_cargo, destination_cargo)] 
    #                 - bigM_time * (1 - quicksum(z[(node_prev, destination_cargo, truck_1, cargo_)] 
    #                                                 for node_prev in selected_node if node_prev != destination_cargo)
    #                               )
    #                 - bigM_time * (1 - quicksum(w[(node_, cargo_)]for node_ in selected_node))
    #                 - bigM_time * quicksum(u[(node_, cargo_)] for node_ in selected_node)
    #             )
    #         else:
    #             MP.addConstr(
    #                 A[(destination_cargo, truck_1)] - D[(origin_cargo, truck_2)]
    #                 >=  selected_edge[(origin_cargo, destination_cargo)] 
    #                 - bigM_time * (1 - quicksum(z[(node_prev, destination_cargo, truck_1, cargo_)] 
    #                                                 for node_prev in selected_node if node_prev != destination_cargo)
    #                               )
    #                 - bigM_time * (1 - quicksum(w[(node_, cargo_)]for node_ in selected_node))
    #                 - bigM_time * quicksum(u[(node_, cargo_)] for node_ in selected_node)
    #             )
    #     else:
    #             MP.addConstr(
    #                 A[(destination_cargo, truck_1)] - D[(origin_cargo, truck_2)]
    #                 >=  selected_edge[(origin_cargo, destination_cargo)] 
    #                 - bigM_time * (1 - quicksum(z[(node_prev, destination_cargo, truck_1, cargo_)] 
    #                                                 for node_prev in selected_node if node_prev != destination_cargo)
    #                               )
    #                 - bigM_time * (1 - quicksum(w[(node_, cargo_)]for node_ in selected_node))
    #                 - bigM_time * quicksum(u[(node_, cargo_)] for node_ in selected_node)
    #             )

    #     if truck_2 in created_truck_yCycle.keys():
    #         if destination_cargo == created_truck_yCycle[truck_2][1]:
    #             MP.addConstr(
    #                 Ab[(destination_cargo, truck_2)] - D[(origin_cargo, truck_1)]
    #                 >=  selected_edge[(origin_cargo, destination_cargo)] 
    #                 - bigM_time * (1 - quicksum(z[(node_prev, destination_cargo, truck_1, cargo_)] 
    #                                                 for node_prev in selected_node if node_prev != destination_cargo)
    #                               )
    #                 - bigM_time * (1 - quicksum(u[(node_, cargo_)]for node_ in selected_node))
    #                 - bigM_time * quicksum(w[(node_, cargo_)] for node_ in selected_node)
    #             )
    #         else:
    #             MP.addConstr(
    #                 A[(destination_cargo, truck_2)] - D[(origin_cargo, truck_1)]
    #                 >=  selected_edge[(origin_cargo, destination_cargo)] 
    #                 - bigM_time * (1 - quicksum(z[(node_prev, destination_cargo, truck_1, cargo_)] 
    #                                                 for node_prev in selected_node if node_prev != destination_cargo)
    #                               )
    #                 - bigM_time * (1 - quicksum(u[(node_, cargo_)]for node_ in selected_node))
    #                 - bigM_time * quicksum(w[(node_, cargo_)] for node_ in selected_node)
    #             )
    #     else:
    #             MP.addConstr(
    #                 A[(destination_cargo, truck_2)] - D[(origin_cargo, truck_1)]
    #                 >=  selected_edge[(origin_cargo, destination_cargo)] 
    #                 - bigM_time * (1 - quicksum(z[(node_prev, destination_cargo, truck_1, cargo_)] 
    #                                                 for node_prev in selected_node if node_prev != destination_cargo)
    #                               )
    #                 - bigM_time * (1 - quicksum(u[(node_, cargo_)]for node_ in selected_node))
    #                 - bigM_time * quicksum(w[(node_, cargo_)] for node_ in selected_node)
    #             )          
    
    ###### Objective ######
    

    # cargo number cost: proportional to the number of cargo carried by the only truck
    cost_cargo_number = quicksum(z[(node_, selected_cargo[cargo_][-1], truck_, cargo_)] 
                                for cargo_ in selected_cargo.keys()
                                for node_ in selected_node if node_ != selected_cargo[cargo_][-1]
                                for truck_ in selected_truck.keys()
                                )
    
    MP.setObjective(cost_cargo_number)
    MP.modelSense = GRB.MAXIMIZE
    # set Params.Heuristics to 0.5 
    # such that it better finds feasible solution
    MP.Params.Heuristics = 0.8
    MP.Params.LogFile = filename
    if early_termination_timelimit is None:
        callback = None
    else:
        callback = early_termination_callback
    
    ###### Integrate the model and optimize ######

    # private parameters to help with callback function
    # MP.Params.LogToConsole  = 0
    MP._cur_obj = float('inf')
    MP._time = time.time()
    MP._no_improve_iter = 0
    MP._termination_flag = 0

    MP.Params.timeLimit = runtime
    MP.Params.OutputFlag = 1
    MP.update()

    if callback is None:
        MP.optimize()
    else:
        if verbose >0:
            print('Use soft-termination through callback, terminate if no better solution in 20 s')
        MP.optimize(callback=callback)

    # if infeasible
    if MP.Status == 3:
        if verbose >0: print('+++ MIP [Infeasible Proved] ')
        return -1, runtime, [], [], [], [], [], [], [], [], [], -1, -1, -1, -1
    
    print(MP.Status)
    runtime_MP = MP.Runtime
    obj_val_MP = MP.objVal

    
    # if no objective value
    if float('inf') == obj_val_MP:
        if verbose >0: print('+++ MIP [Infeasible] ')
        return -1, runtime, [], [], [], [], [], [], [], [], [], -1, -1, -1, -1
        
    
    if verbose > 0:
        print(f'+++ {MP.ModelName} [Feasible] ')
        if MP._termination_flag == 1:
            print('    soft termination: failed to improve best solution for 20s.')
        elif MP._termination_flag == 2:
            print('    soft termination: failed to improve obj for 50 consecutive feasible solutions.') 
        if verbose > 1:
            print("   [Gurobi obj value] is %i" % obj_val_MP)
            print("   [Gurobi runtime] is %f" % runtime_MP)
    
       
    ###### Get solutions ######
    
    # store all values in a list: sol
    sol = []
    for ss in MP.getVars():
        sol.append(int(ss.x))
        
    # retrieve values from the list sol
    count = 0
    # binary variables x
    x_sol = {}
    for truck_ in selected_truck.keys():
        for i in selected_node:
            for j in selected_node:
                if i != j:
                    x_sol[(i, j, truck_)] = sol[count]
                    count += 1
                
    # integer variable S
    S_sol = {}
    Sb_sol = {}
    for truck_ in selected_truck.keys():
        for node_ in selected_node:
            S_sol[(node_, truck_)] = sol[count]
            count += 1
    # if truck_ is a cycle truck and node_ is its destination
    for truck_ in created_truck_yCycle.keys():
        node_ = created_truck_yCycle[truck_][1]
        Sb_sol[(node_, truck_)] = 0
            
    # integer variable D
    D_sol = {}
    Db_sol = {}
    for truck_ in selected_truck.keys():
        for node_ in selected_node:
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
    for truck_ in selected_truck.keys():
        for node_ in selected_node:
            A_sol[(node_, truck_)] = sol[count]
            count += 1
    # if truck_ is a cycle truck and node_ is its destination
    for truck_ in created_truck_yCycle.keys():
        node_ = created_truck_yCycle[truck_][1]
        Ab_sol[(node_, truck_)] = sol[count]
        count += 1
    
    u_sol, w_sol, z_sol = {}, {}, {}
    for node_ in selected_node:
        for cargo_ in selected_cargo.keys():
            u_sol[(node_, cargo_)] = sol[count]
            count += 1
    #u[(node_, cargo_)] = 1 if cargo_ is transferred from k1 to k2 at node_
    for node_ in selected_node: 
        for cargo_ in selected_cargo.keys():
            w_sol[(node_, cargo_)] = sol[count]
            count += 1
    for edge_ in selected_edge.keys():
        for truck_ in selected_truck.keys():
            for cargo_ in selected_cargo.keys():
                if edge_[0] != edge_[1]:
                    z_sol[(edge_[0], edge_[1], truck_, cargo_)] = sol[count]
                    count += 1

   
         
    del MP

    gurobi_res = (x_sol, z_sol, w_sol, u_sol, S_sol, D_sol, A_sol, \
                    Sb_sol, Db_sol, Ab_sol)

    
    return obj_val_MP, runtime_MP, gurobi_res