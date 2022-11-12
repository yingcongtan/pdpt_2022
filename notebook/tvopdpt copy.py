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
                        runtime, filename, verbose = 0):
    
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
        if time.time() - model._time > 20:
            MP._termination_flag = 1
            model.terminate()

    
    MP = Model("Gurobi MIP for PDOTW")
    # else:
    #     MP = Model("Minimize travel cost of PDOTW sol")

    
    
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
    
#     # binary variables z
#     # ==1 if the truck_ is used in the solution
#     z = {}
#     for truck_ in selected_truck.keys():
#         z[truck_] = MP.addVar(vtype=GRB.BINARY)
    
    # binary variables y
    # ==1 if cargo_ is carried by truck_
    # y={}
    # if y_sol_ is None:
    #     for truck_ in selected_truck.keys():
    #         for cargo_ in selected_cargo.keys():
    #             y[(truck_, cargo_)] = MP.addVar(vtype=GRB.BINARY)

    # else:
    #     for truck_ in selected_truck.keys():
    #         for cargo_ in selected_cargo.keys():
    #             y[(truck_, cargo_)] = y_sol_[(truck_, cargo_)]
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
    
        
    ###### Constraints ######
    ###### Distinguish cycle trucks and non-cycle trucks ######
    
    # Flow constraints (3.1)
    # the truck must start from its origin
    for truck_ in selected_truck.keys():
        origin_truck = selected_truck[truck_][0]
        if origin_truck in selected_node:
            MP.addConstr( 
                quicksum(x[(origin_truck, succ_node, truck_)] * 1
                         for succ_node in selected_node
                         if succ_node != origin_truck) == 1 
            )
        
    # Flow constraints (3.2)  
    # only applies to non-cycle trucks
    # no flow enters the origin of a non-cycle truck
    for truck_ in created_truck_nCycle.keys():
        origin_truck = created_truck_nCycle[truck_][0]
        if origin_truck in selected_node:
            MP.addConstr( 
                quicksum(x[(succ_node, origin_truck, truck_)] * 1
                         for succ_node in selected_node
                         if succ_node != origin_truck) == 0 
            )

    # Flow constraints (3.3)
    # the truck must end at its destination
    for truck_ in selected_truck.keys():
        destination_truck = selected_truck[truck_][1]
        if destination_truck in selected_node:
            MP.addConstr( 
                quicksum(x[(pred_node, destination_truck, truck_)] * 1
                         for pred_node in selected_node
                         if pred_node != destination_truck) == 1
            )    
        
    # Flow constraints (3.4)
    # only applies to non-cycle trucks
    # no flow departs from the destination of a non-cycle truck
    for truck_ in created_truck_nCycle.keys():
        destination_truck = created_truck_nCycle[truck_][1]
        if destination_truck in selected_node:
            MP.addConstr( 
                quicksum(x[(destination_truck, pred_node, truck_)] * 1
                         for pred_node in selected_node
                         if pred_node != destination_truck) == 0 
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
                MP.addConstr(
                    quicksum(x[(pred_node, node_, truck_)] * 1
                             for pred_node in selected_node
                             if pred_node != node_) 
                    ==
                    quicksum(x[(node_, succ_node, truck_)] * 1
                             for succ_node in selected_node
                             if succ_node != node_) 
                )
    
    # An edge is used at most once by a truck (3.6)
    # only apply for non-cycle trucks
    # and non-origin nodes for cycle trucks
    for truck_ in created_truck_nCycle.keys():
        for i in selected_node:
            for j in selected_node:
                if i != j:
                    MP.addConstr(
                        x[(i, j, truck_)] +
                        x[(j, i, truck_)]
                        <= 1
                    )
    for truck_ in created_truck_yCycle.keys():
        origin_truck = created_truck_yCycle[truck_][0]
        for i in selected_node:
            for j in selected_node:
                if i != j:
                    if i != origin_truck and j != origin_truck:
                        MP.addConstr(
                            x[(i, j, truck_)] +
                            x[(j, i, truck_)]
                            <= 1
                        )
            
    
    # origin_c is visited by truck_ if y[(truck_, c)] == 1 (3.9)
    for truck_ in selected_truck.keys():
        for cargo_ in selected_cargo.keys():
            origin_cargo = selected_cargo[cargo_][3]
            if origin_cargo in selected_node:
                MP.addConstr(
                    quicksum(x[(origin_cargo, node_, truck_)] * 1
                             for node_ in selected_node
                             if node_ != origin_cargo)
                    >= 
                    y[(truck_, cargo_)]
                )
    
    # destination_c is visited by truck_ if y[(truck_, c)] == 1 (3.10)
    for truck_ in selected_truck.keys():
        for cargo_ in selected_cargo.keys():
            destination_cargo = selected_cargo[cargo_][4]
            if destination_cargo in selected_node:
                MP.addConstr(
                    quicksum(x[(node_, destination_cargo, truck_)] * 1
                             for node_ in selected_node
                             if node_ != destination_cargo)
                    >= 
                    y[(truck_, cargo_)]
                )
    
    ### Capacity ----------------------------------------------------
    
    # capacity constraints (3.14)
    for truck_ in selected_truck.keys():
        # if truck_ is a NON-cycle truck and node_ is its destination
        # then the truck capacity when departing its destination is 0
        if truck_ in created_truck_nCycle.keys():
            destination_truck = selected_truck[truck_][1]
            MP.addConstr(
                S[(destination_truck, truck_)] 
                == 0
            )
            
    # Cumulative total size of a truck at a node (3.15)
    # be aware of whether the node is 
    # a cargo origin or cargo destination, or both
    bigM_capacity = 30000
    for truck_ in selected_truck.keys():
        for node1 in selected_node:
            for node2 in selected_node:
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
    for truck_ in selected_truck.keys():
        origin_truck = selected_truck[truck_][0]
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
    
    # loading and unloading time between arrival and departure (3.17)
    # Don't consider origin_truck in this constraint
    # but for cycle trucks, their origins are also their destinations
    # so we only consider their destination parts
    for truck_ in selected_truck.keys():
        for node_ in selected_node:
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
                if node_ != selected_truck[truck_][0]:
                    if node_ == selected_truck[truck_][1]:
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
                    selected_cargo[cargo_][1] * y[(truck_, cargo_)]
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

    
    # first pickup and then delivery (3.22)
    for truck_ in selected_truck.keys():
        for cargo_ in selected_cargo.keys():
            origin_cargo = selected_cargo[cargo_][3]
            destination_cargo = selected_cargo[cargo_][4]
            if destination_cargo in selected_node:
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
    

    if y_sol_ is None:
        # cargo size cost: proportional to the total size of cargo carried by the only truck
        cost_cargo_size = quicksum(y[truck_, cargo_] * selected_cargo[cargo_][0] * 
                                constant['truck_running_cost'] * 1
                                for truck_ in selected_truck.keys()
                                for cargo_ in selected_cargo.keys())
        
        # cargo number cost: proportional to the number of cargo carried by the only truck
        cost_cargo_number = quicksum(y[truck_, cargo_] * 
                                    constant['truck_running_cost'] * 1000
                                    for truck_ in selected_truck.keys()
                                    for cargo_ in selected_cargo.keys())
        
        MP.setObjective(cost_cargo_size + cost_cargo_number)
        MP.modelSense = GRB.MAXIMIZE
        # set Params.Heuristics to 0.5 
        # such that it better finds feasible solution
        MP.Params.Heuristics = 0.5
        MP.Params.LogFile = filename
        callback=early_termination_callback

    else:
        cost_travel = quicksum(x[(node1, node2, truck_)] * 
                            selected_edge[(node1, node2)] * 
                            constant['truck_running_cost']
                            for truck_ in selected_truck.keys()
                            for node1 in selected_node
                            for node2 in selected_node
                            if node1 != node2)
        MP.setObjective(cost_travel)
        MP.modelSense = GRB.MINIMIZE
        MP.Params.LogFile = filename[:-4]+'_reopt.log'
        callback=None
    
    ###### Integrate the model and optimize ######

    # private parameters to help with callback function
    MP.Params.LogToConsole  = 0
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
                
    # # binary variables y
    # if y_sol_ is None:
    #     y_sol = {}
    #     for truck_ in selected_truck.keys():
    #         for cargo_ in selected_cargo.keys():
    #             y_sol[(truck_, cargo_)] = sol[count]
    #             count += 1
    # else:
    #     y_sol = y_sol_.copy() # for the convinience of the printout below
    
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
    
   
         
    del MP

    
    return obj_val_MP, runtime_MP, \
           x_sol, {}, y_sol, S_sol, D_sol, A_sol, \
           Sb_sol, Db_sol, Ab_sol