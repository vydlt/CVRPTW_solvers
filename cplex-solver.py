import numpy as np
import matplotlib.pyplot as plt
from sys import exit
import os
from docplex.mp.model import Model


def Input_Instance():
    print("Type instance ([r|c|rc][NNN]):")
    instance_name = None
    try:
        instance_name = input()
    except Exception as e:
        print("Invalid instance. Exit."); exit(1)
    if instance_name.endswith(".txt"):
        instance_name = instance_name[:-4]
    file_path = os.path.join("solomon-instances", instance_name+".txt")
    if not instance_name or not os.path.exists(file_path):
        print("Instance does not exists. Exit."); exit(1)
    print("Select number of costumers (1-100):")
    n = None
    try:
        n = int(input())
    except Exception as e:
        print("Invalid number of costumers. Exit."); exit(1)
    if not n or n < 1 or n > 100:
        print("Invalid number of costumers. Exit."); exit(1)
    
    return instance_name, file_path, n


def Read_Instance(path, n):
    stream = ""
    with open(path, "r") as file:
        stream = file.readlines()
    if stream == "":
        print("Error in reading file")
    else:
        print("Read file", path, "\n")

    K, Q = [int(i) for i in stream[4].split()]
    fields = ("CUST-NO.", "XCOORD.", "YCOORD.", "DEMAND", "READY-TIME", \
                "DUE-DATE", "SERVICE-TIME")
    data = list()
    for i in range(9, len(stream)):
        if stream[i] == "\n":
            continue
        val = stream[i].split()
        if len(val) != len(fields):
            print("Error in reading data")
            continue
        customer = dict(zip(fields, val))
        data.append(customer)
    data = data[0:n+1]
    data.append(data[0]) # The depot is represented by two identical
                         # nodes: 0 and n+1
    data[-1]["CUST-NO."] = "51"
    x = []; y = []; q = []; a = []; b = []; s = []
    for customer in data:
        x.append(int(customer["XCOORD."]))
        y.append(int(customer["YCOORD."]))
        q.append(int(customer["DEMAND"]))
        a.append(int(customer["READY-TIME"]))
        b.append(int(customer["DUE-DATE"]))
        # s.append(int(customer["SERVICE-TIME"]))

    return K, Q, x, y, q, a, b


def Create_Distance_Matrix(x, y):
    n = len(x)
    d = np.zeros((n,n))
    for i in range(n):
        for j in range(i+1,n):
            p1 = np.array([x[i], y[i]])
            p2 = np.array([x[j], y[j]])
            d[i,j] = d[j,i] = int(round(np.linalg.norm(p1-p2))) 
            # compute L2-norm of a vector
    
    return d



def Create_Model(K, Q, n, q, a, b, d,**kwargs):
    
    m = Model(name = 'VRPTW_MILP_Model', **kwargs)
    
    bigM = 100000
        
    # Dec.Var. Arcs traversed by vehicles
    m.xvar_var = m.binary_var_cube(n+2, n+2, K, name = "xvar")
    
    # Dec.Var. Starting time of service of customers
    m.w_var = m.continuous_var_matrix(n+2, K, name = "w")

    # C1: Covering constraints
    m.covering_constrs = []
    for i in range(1,n+1): 
        cust_satisf_constr = m.sum(
            m.xvar_var[i,j,k] for j in range(1,n+2) for k in range(K) if j != i) == 1
        m.add_constraint(cust_satisf_constr)
        m.covering_constrs.append(cust_satisf_constr)
        
    
    # C2: Route structure constraints
    m.flow_constrs = []
    
    # Flow conservation 
    for i in range(1,n+1):
        for k in range(K):
            flow_constr = (m.sum(m.xvar_var[i,j,k] for j in range(1,n+2) if j != i) - \
            m.sum(m.xvar_var[j,i,k] for j in range(n+1) if j != i)) == 0
            m.add_constraint(flow_constr)
            m.flow_constrs.append(flow_constr)
    
    # Origin and destination of routes
    for k in range(K): 
        flow_constr = m.sum(m.xvar_var[0,j,k] for j in range(1,n+1)) <= 1
        m.add_constraint(flow_constr)
        m.flow_constrs.append(flow_constr)
    for k in range(K): 
        flow_constr = m.sum(m.xvar_var[j,n+1,k] for j in range(1,n+1)) == \
            m.sum(m.xvar_var[0,j,k] for j in range(1,n+1))
        m.add_constraint(flow_constr)
        m.flow_constrs.append(flow_constr)


    # C3: Capacity constraints
    m.capacity_constrs = []
    for k in range(K):
        capacity_constr = m.sum(q[i] * m.xvar_var[i,j,k] for i in range(n+2) \
                                for j in range(n+2) if i != j) <= Q
        m.add_constraint(capacity_constr)
        m.capacity_constrs.append(capacity_constr)

    # C4: Time constraints
    m.TW_constrs = []
    for i in range(n+1):
        for j in range(1,n+2): 
            if j != i:
                for k in range(K):
                    TW_constr = m.w_var[i,k] + d[i][j] <= m.w_var[j,k] + \
                    bigM*(1 - m.xvar_var[i,j,k])
                    m.add_constraint(TW_constr)
                    m.TW_constrs.append(TW_constr)
    for i in range(n+1):
        for k in range(K):
            TW_constr_lb = a[i] * m.sum(m.xvar_var[i,j,k] for j in range(1,n+2) \
                                 if j!=i) <= m.w_var[i,k]
            TW_constr_ub = b[i] * m.sum(m.xvar_var[i,j,k] for j in range(1,n+2) \
                                 if j!=i) >= m.w_var[i,k]
            m.add_constraint(TW_constr_lb)
            m.add_constraint(TW_constr_ub)
            m.TW_constrs.append(TW_constr_lb)
            m.TW_constrs.append(TW_constr_ub)

    # Obj.Func. Minimize the total travelling cost
    m.total_cost = m.sum(d[i][j]*m.xvar_var[i,j,k] for i in range(n+2) for j in range(n+2) \
                         for k in range(K) if i != j)
                
    m.set_objective("min", m.total_cost)
    m.solve()
    m.print_information()
    m.print_solution()

    return m



instance_name, file_path, n = Input_Instance()

K, Q, x, y, q, a, b = Read_Instance(file_path, n)

d = Create_Distance_Matrix(x, y)
#print("Distance matrix:\n", d, "\n")

VRPTW_Model = Create_Model(K, Q, n, q, a, b, d)
