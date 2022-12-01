selected_truck = []
selected_cargo = []
selected_node = []

created_truck_yCycle = []
created_truck_nCycle= []
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
Sb=[]

from gurobipy import Model, quicksum, GRB
MP = Model("Gurobi MIP for TVOPDPT")    

constant = []
import numpy as np

# truck_1, truck_2 = selected_truck.keys()
truck_1 = selected_truck.keys()[0]
truck_1_origin, truck_1_dest = selected_truck[truck_1][:2]
# truck_2_origin, truck_2_dest = selected_truck[truck_2][:2]


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
                        >=  quicksum(z[(node_prev, node_curr, truck_1, cargo_key)]* node_cargo_size_change[(node_curr, cargo_key)]
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
                                        node_cargo_size_change[(node_curr, cargo_key)]
                                        for cargo_key, cargo_value in selected_cargo.items() 
                                        if cargo_value[4] == node_curr) # deliver cargo
                            + quicksum( z[(node_curr, node_next, truck_1, cargo_key)]* 
                                        node_cargo_size_change[(node_curr, cargo_key)]
                                        for node_next in selected_node 
                                        for cargo_key, cargo_value in selected_cargo.items()
                                        if cargo_value[3] == node_curr and node_curr != node_next) # pickup cargo
                            # - bigM_capacity *(1 - x[(node_prev, node_curr, truck_1)])
                            - quicksum(u[(node_curr, cargo_key)]*cargo_value[0] # transfer to truck 2 if node_curr is not cargo's origin
                                        for cargo_key, cargo_value in selected_cargo.items() if node_curr != cargo_value[3]) 
                            + quicksum(w[(node_curr, cargo_key)]*cargo_value[0] # receive from truck 2 if node_curr is not cargo's dest
                                        for cargo_key, cargo_value in selected_cargo.items() if node_curr != cargo_value[4])
                    )
            elif truck_1 not in created_truck_yCycle.keys(): # if truck 1 is a non-cucle truck
                if node_curr == truck_1_dest: 
                    #if node_curr is the destination of truck1
                    # then we can only deliver cargo, and transfer cargo to truck 2

                    MP.addConstr(
                        S[(node_curr, truck_1)] - S[(node_prev, truck_1)] 
                        >=  quicksum(z[(node_prev, node_curr, truck_1, cargo_key)]* node_cargo_size_change[(node_curr, cargo_key)]
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
                        >=  quicksum(z[(node_prev, node_curr, truck_1, cargo_key)]* node_cargo_size_change[(node_curr, cargo_key)]
                                        for cargo_key, cargo_value in selected_cargo.items() 
                                        if cargo_value[4] == node_curr) # deliver cargo
                            + quicksum( z[(node_curr, node_next, truck_1, cargo_key)]* node_cargo_size_change[(node_curr, cargo_key)]
                                        for node_next in selected_node 
                                        for cargo_key, cargo_value in selected_cargo.items()
                                        if cargo_value[3] == node_curr and node_curr != node_next) # pickup cargo
                            # - bigM_capacity *(1 - x[(node_prev, node_curr, truck_1)])
                            - quicksum(u[(node_curr, cargo_key)]*cargo_value[0] # transfer to truck 2 if node_curr is not cargo's origin
                                        for cargo_key, cargo_value in selected_cargo.items() if node_curr != cargo_value[3] )
                            + quicksum(w[(node_curr, cargo_key)]*cargo_value[0] # receive from truck 2 if node_curr is not cargo's dest
                                        for cargo_key, cargo_value in selected_cargo.items() if node_curr != cargo_value[4] )
                    )
    