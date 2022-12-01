selected_truck = []
selected_cargo = []
selected_node = []

created_truck_yCycle = []
selected_edge = []
Ab = []
z=[]
A = []
D = []
Db=[]
bigM_time = 0
bigM_capacity = 0
node_cargo_size_change=[]
S=[]
x = []
w=[]
u = []

from gurobipy import Model, quicksum, GRB
MP = Model("Gurobi MIP for TVOPDPT")    

constant = []
import numpy as np

truck_1, truck_2 = selected_truck.keys()



bigM_capacity = 30000
for node_curr in selected_node:
    for node_prev in selected_node:
        if node_prev != node_prev:
            if truck_1 in created_truck_yCycle.keys(): # if truck_1 is a cycle truck
                if node_prev == created_truck_yCycle[truck_1][1]: #if node_curr is the destination of truck1
                    # cargo size change from i to j
                    # s[k1, j] - s[k1, i] <= sum_{r if r_dest == j} z[i, j, k1, r]*delta[j, r]
                    #                       - M*(1-x[i, j, k1]) - sum_r u[j, r]*r_size
                    MP.addConstr(
                        Sb[(node_curr, truck_)] - S[(node_prev, truck_)] 
                        >=  quicksum(z[(node_prev, node_curr, truck_1, cargo_key)]* 
                                        node_cargo_size_change[(node_curr, cargo_)]
                                        for cargo_key, cargo_value in selected_cargo.items() 
                                        if cargo_value[4] == node_curr)
                            - bigM_capacity *(1 - x[(node_prev, node_curr, truck_1)])
                            - quicksum(u[(node_curr, cargo_key)]*cargo_value[0]
                                        for cargo_key, cargo_value in selected_cargo.items())
                    )
                else: # if node_curr is not the destination of truck1
                    # cargo size change from i to j
                    # s[k1, j] - s[k1, i] <= sum_{r if r_dest == j} z[i, j, k1, r]*delta[j, r] 
                    #                      + sum_{r if r_origin == j, l in V} z[j, l, k1, r]*delta[j, r] 
                    #                       - M*(1-x[i, j, k1]) - sum_r u[j, r]*r_size +  sum_r w[j, r]*r_size + 
                    MP.addConstr(
                        S[(node_curr, truck_)] - S[(node_prev, truck_)] 
                        >=  quicksum(z[(node_prev, node_curr, truck_1, cargo_key)]* 
                                        node_cargo_size_change[(node_curr, cargo_)]
                                        for cargo_key, cargo_value in selected_cargo.items() 
                                        if cargo_value[4] == node_curr)
                            + quicksum( z[(node_curr, node_next, truck_1, cargo_key)]* 
                                        node_cargo_size_change[(node_curr, cargo_)]
                                        for node_next in selected_node 
                                        for cargo_key, cargo_value in selected_cargo.items()
                                        if cargo_value[3] == node_curr)
                            - bigM_capacity *(1 - x[(node_prev, node_curr, truck_1)])
                            - quicksum(u[(node_curr, cargo_key)]*cargo_value[0]
                                        for cargo_key, cargo_value in selected_cargo.items())
                            + quicksum(w[(node_curr, cargo_key)]*cargo_value[0]
                                        for cargo_key, cargo_value in selected_cargo.items())
                    )
            else: # if truck 1 is a non-cucle truck
                if node_prev == created_truck_yCycle[truck_1][1]: #if node_curr is the destination of truck1
                    MP.addConstr(
                        S[(node_curr, truck_)] - S[(node_prev, truck_)] 
                        >=  quicksum(z[(node_prev, node_curr, truck_1, cargo_key)]* 
                                        node_cargo_size_change[(node_curr, cargo_)]
                                        for cargo_key, cargo_value in selected_cargo.items() 
                                        if cargo_value[4] == node_curr)
                            - bigM_capacity *(1 - x[(node_prev, node_curr, truck_1)])
                            - quicksum(u[(node_curr, cargo_key)]*cargo_value[0]
                                        for cargo_key, cargo_value in selected_cargo.items())
                    )
                else: # if node_curr is not the destination of truck1
                    # cargo size change from i to j
                    # s[k1, j] - s[k1, i] <= sum_{r if r_dest == j} z[i, j, k1, r]*delta[j, r] 
                    #                      + sum_{r if r_origin == j, l in V} z[j, l, k1, r]*delta[j, r] 
                    #                       - M*(1-x[i, j, k1]) - sum_r u[j, r]*r_size +  sum_r w[j, r]*r_size + 
                    MP.addConstr(
                        S[(node_curr, truck_)] - S[(node_prev, truck_)] 
                        >=  quicksum(z[(node_prev, node_curr, truck_1, cargo_key)]* 
                                        node_cargo_size_change[(node_curr, cargo_)]
                                        for cargo_key, cargo_value in selected_cargo.items() 
                                        if cargo_value[4] == node_curr)
                            + quicksum( z[(node_curr, node_next, truck_1, cargo_key)]* 
                                        node_cargo_size_change[(node_curr, cargo_)]
                                        for node_next in selected_node 
                                        for cargo_key, cargo_value in selected_cargo.items()
                                        if cargo_value[3] == node_curr)
                            - bigM_capacity *(1 - x[(node_prev, node_curr, truck_1)])
                            - quicksum(u[(node_curr, cargo_key)]*cargo_value[0]
                                        for cargo_key, cargo_value in selected_cargo.items())
                            + quicksum(w[(node_curr, cargo_key)]*cargo_value[0]
                                        for cargo_key, cargo_value in selected_cargo.items())
                    )
            if truck_2 in created_truck_yCycle.keys(): # if truck_2 is a cycle truck
                if node_prev == created_truck_yCycle[truck_2][1]: #if node_curr is the destination of truck1
                    # cargo size change from i to j
                    # s[k1, j] - s[k1, i] <= sum_{r if r_dest == j} z[i, j, k1, r]*delta[j, r]
                    #                       - M*(1-x[i, j, k1]) - sum_r w[j, r]*r_size
                    MP.addConstr(
                        Sb[(node_curr, truck_)] - S[(node_prev, truck_)] 
                        >=  quicksum(z[(node_prev, node_curr, truck_2, cargo_key)]* 
                                        node_cargo_size_change[(node_curr, cargo_)]
                                        for cargo_key, cargo_value in selected_cargo.items() 
                                        if cargo_value[4] == node_curr)
                            - bigM_capacity *(1 - x[(node_prev, node_curr, truck_2)])
                            - quicksum(w[(node_curr, cargo_key)]*cargo_value[0]
                                        for cargo_key, cargo_value in selected_cargo.items())
                    )
                else: # if node_curr is not the destination of truck1
                    # cargo size change from i to j
                    # s[k1, j] - s[k1, i] <= sum_{r if r_dest == j} z[i, j, k1, r]*delta[j, r] 
                    #                      + sum_{r if r_origin == j, l in V} z[j, l, k1, r]*delta[j, r] 
                    #                       - M*(1-x[i, j, k1]) - sum_r w[j, r]*r_size +  sum_r u[j, r]*r_size + 
                    MP.addConstr(
                        S[(node_curr, truck_)] - S[(node_prev, truck_)] 
                        >=  quicksum(z[(node_prev, node_curr, truck_2, cargo_key)]* 
                                        node_cargo_size_change[(node_curr, cargo_)]
                                        for cargo_key, cargo_value in selected_cargo.items() 
                                        if cargo_value[4] == node_curr)
                            + quicksum( z[(node_curr, node_next, truck_2, cargo_key)]* 
                                        node_cargo_size_change[(node_curr, cargo_)]
                                        for node_next in selected_node 
                                        for cargo_key, cargo_value in selected_cargo.items()
                                        if cargo_value[3] == node_curr)
                            - bigM_capacity *(1 - x[(node_prev, node_curr, truck_2)])
                            - quicksum(w[(node_curr, cargo_key)]*cargo_value[0]
                                        for cargo_key, cargo_value in selected_cargo.items())
                            + quicksum(u[(node_curr, cargo_key)]*cargo_value[0]
                                        for cargo_key, cargo_value in selected_cargo.items())
                    )
            else: # if truck 1 is a non-cucle truck
                if node_prev == created_truck_yCycle[truck_2][1]: #if node_curr is the destination of truck1
                    MP.addConstr(
                        S[(node_curr, truck_)] - S[(node_prev, truck_)] 
                        >=  quicksum(z[(node_prev, node_curr, truck_2, cargo_key)]* 
                                        node_cargo_size_change[(node_curr, cargo_)]
                                        for cargo_key, cargo_value in selected_cargo.items() 
                                        if cargo_value[4] == node_curr)
                            - bigM_capacity *(1 - x[(node_prev, node_curr, truck_2)])
                            - quicksum(w[(node_curr, cargo_key)]*cargo_value[0]
                                        for cargo_key, cargo_value in selected_cargo.items())
                    )
                else: # if node_curr is not the destination of truck1
                    # cargo size change from i to j
                    # s[k1, j] - s[k1, i] <= sum_{r if r_dest == j} z[i, j, k1, r]*delta[j, r] 
                    #                      + sum_{r if r_origin == j, l in V} z[j, l, k1, r]*delta[j, r] 
                    #                       - M*(1-x[i, j, k1]) - sum_r u[j, r]*r_size +  sum_r w[j, r]*r_size + 
                    MP.addConstr(
                        S[(node_curr, truck_)] - S[(node_prev, truck_)] 
                        >=  quicksum(z[(node_prev, node_curr, truck_2, cargo_key)]* 
                                        node_cargo_size_change[(node_curr, cargo_)]
                                        for cargo_key, cargo_value in selected_cargo.items() 
                                        if cargo_value[4] == node_curr)
                            + quicksum( z[(node_curr, node_next, truck_2, cargo_key)]* 
                                        node_cargo_size_change[(node_curr, cargo_)]
                                        for node_next in selected_node 
                                        for cargo_key, cargo_value in selected_cargo.items()
                                        if cargo_value[3] == node_curr)
                            - bigM_capacity *(1 - x[(node_prev, node_curr, truck_2)])
                            - quicksum(w[(node_curr, cargo_key)]*cargo_value[0]
                                        for cargo_key, cargo_value in selected_cargo.items())
                            + quicksum(u[(node_curr, cargo_key)]*cargo_value[0]
                                        for cargo_key, cargo_value in selected_cargo.items())
                    )