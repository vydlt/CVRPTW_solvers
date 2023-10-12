# -*- coding: utf-8 -*-

from utilities import *

def create_model(d, q, Q, a, b, n, K):
    """Create mathematical model"""
    M = compute_max_cost(d, a, b, n)
    model = gp.Model("ESPModel")
    A = [(i,j,k) for i in range(n+2) for j in range(n+2) for k in range(K)]
    # Decision variables
    x_vars = model.addVars(A, vtype=gp.GRB.BINARY, name="arc")
    s_vars = model.addVars(n+2, K, vtype=gp.GRB.CONTINUOUS, name="service")
    v_vars = model.addVars(K, vtype=gp.GRB.BINARY, name="vehicle")
    # Covering constraints     
    for i in range(1,n+1):
        model.addConstr(gp.quicksum(x_vars[i,j,k] for j in range(1,n+2) for\
                                    k in range(K) if j != i) == 1)
    # Vehicle use constraints
    for k in range(K):
        for j in range(1,n+1):
            model.addConstr(x_vars[0,j,k] <= v_vars[k])
    # Capacity constraint
    for k in range(K) :
        model.addConstr(sum([q[i] * \
                    gp.quicksum(x_vars[i,j,k] for j in range(n+2)) \
                        for i in range(1,n+1)]) <= Q*v_vars[k])
    # Depot start constraint
    for k in range(K):
            model.addConstr(gp.quicksum(x_vars[0,j,k] for j in range(n+2)) <= 1)
    # Depot finish constraint
    for k in range(K):
            model.addConstr(gp.quicksum(x_vars[i,n+1,k] for i in range(n+2)) <= 1)
    # Flow costraints
    for k in range(K):
        for h in range(1,n+1):
            model.addConstr(gp.quicksum(x_vars[i,h,k] for i in range(n+2))\
                            - gp.quicksum(x_vars[h,j,k] for j in range(n+2)) == 0)
    # Time windows contraints
    for k in range(K):
        for i in range(n+2):
            for j in range(n+2):
                if j!=i and i!=0 and j!=n+1:
                    model.addConstr(s_vars[i,k] + d[i,j] - M*(1-x_vars[i,j,k]) <= \
                                    s_vars[j,k]*v_vars[k])
                if j!=i and i==0 and j!=n+1:
                    model.addConstr(a[i] + d[i,j] - M*(1-x_vars[i,j,k]) <= \
                                    s_vars[j,k]*v_vars[k])
                if j!=i and i!=0 and j==n+1:
                    model.addConstr(s_vars[i,k] + d[i,j] - M*(1-x_vars[i,j,k]) <= \
                                    b[j]*v_vars[k])
    # Service time constraints
    for k in range(K):
        model.addConstrs(s_vars[i,k] >= a[i]*v_vars[k] for i in range(1,n+1))
        model.addConstrs(s_vars[i,k] <= b[i]*v_vars[k] for i in range(1,n+1))
        model.addConstr(s_vars[0,k] == a[0]*v_vars[k])
        model.addConstr(s_vars[n+1,k] == b[n+1]*v_vars[k])
    # Goodsense constraints:
    for k in range(K):
        # Must not exist an arc that connects a customer with himself
        model.addConstr(gp.quicksum(x_vars[i,i,k] for i in range(n+2)) == 0)
        # No arc can enter in the first node
        model.addConstr(gp.quicksum(x_vars[i,0,k] for i in range(n+2)) == 0)
        # No arc can exit from the last node
        model.addConstr(gp.quicksum(x_vars[n+1,j,k] for j in range(n+2)) == 0)
    
        
    model.setObjective((gp.quicksum(x_vars[i,j,k]*d[i,j] for i in range(n+2) \
                                       for k in range(K) for j in range(n+2))) + \
                           (gp.quicksum(v_vars[k] for k in range(K))), \
                           gp.GRB.MINIMIZE)
    # model.write("ESPModel.lp")
        
    return model, x_vars, A

def construct_routes(x_vars, A):
    active_arcs = [i for i in A if x_vars[i].x >= 0.5]
    #print("Active arcs: ", active_arcs)
    # Find all unique vehicle numbers
    vehicle_numbers = set(arc[2] for arc in active_arcs)
    # Initialize a dictionary to store the routes per vehicle
    routes = {vehicle_number: [0] for vehicle_number in vehicle_numbers}
    # Extend the routes based on the remaining arcs
    for _ in range(len(active_arcs) - 1):
        updated = False
        for arc in active_arcs:
            node_i, node_j, vehicle_number = arc
            if node_i in routes[vehicle_number] and node_j not in routes[vehicle_number]:
                routes[vehicle_number].append(node_j)
                updated = True
        if not updated:
            break
    # Sort the routes based on the last node
    sorted_routes = {vehicle_number: route + [max(route)] for vehicle_number, route in routes.items()}
    for vehicle_number, route in sorted_routes.items():
        last_node = route[-1]
        if route.count(last_node) > 1:
            route.remove(last_node)
    # Print the routes for each vehicle
    #for vehicle_number, route in sorted_routes.items():
        #print(f"Route for vehicle {vehicle_number}: {route}")

    return active_arcs, sorted_routes

#------------------------------------------------------------------------------
'''Create instance, run, and store results'''
#------------------------------------------------------------------------------

instance_name, file_path, n = select_instance()
K, Q, x, y, q, a, b = read_instance(file_path, n)
d = create_distance_matrix(x, y)
#print("Distance matrix:\n", d, "\n")
ESPmodel, x_vars, A = create_model(d, q, Q, a, b, n, K)

start = process_time()

ESPmodel.optimize()

end = process_time()
sec = int(end-start)
min = int(sec/60)
sec %= 60

obj = ESPmodel.getObjective()
#print("Objective value:", obj.getValue())

active_arcs, sorted_routes = construct_routes(x_vars, A)

# Write results on file in directory "results"
if not os.path.exists(os.path.join("gurobi-results")):
    os.mkdir(os.path.join("gurobi-results"))
filenameOut = os.path.join("gurobi-results", \
                           "results-"+str(instance_name)+"-"+str(n)+".txt")
fout = open(filenameOut, "w")
fout.write("Instance: " + str(instance_name) + "\n")
fout.write("Number of customers: " + str(n) + "\n")
fout.write("MILP (Gurobi) exact cost solution: " + str(obj.getValue()) + "\n")
fout.write("Time elapsed: " + str(min) + "min" + str(sec) + "s" + "\n")
#fout.write("Decision variable solution: " + "\n")
#for v in ESPmodel.getVars():
    #if v.x != 0:
        #fout.write(str('%s %g' % (v.VarName, v.X)) + "\n")
i = 0
for vehicle_number, route in sorted_routes.items():
    fout.write(f"Route for vehicle {i}: {route}" + "\n")
    i += 1
fout.close()

# Plot routes
plt.figure(figsize=(8, 6))
for i,j,k in active_arcs:
    plt.plot([x[i],x[j]],[y[i],y[j]],c='C1',zorder=0)
plt.plot(x[0],y[0],c='r',marker='s')
plt.scatter(x[1:],y[1:],c='b')

# Save plots on file
if not os.path.exists(os.path.join("gurobi-plots")):
    os.mkdir(os.path.join("gurobi-plots"))
plotfileOut = os.path.join("gurobi-plots", \
                           "plots-"+str(instance_name)+"-"+str(n))
plt.savefig(plotfileOut)
plt.close()
