# -*- coding: utf-8 -*-

from utilities import *
from docplex.mp.model import Model


def create_model(K, Q, n, q, a, b, d,**kwargs):
    """Create model"""
    m = Model(name = 'VRPTW_MILP_Model', **kwargs)
    bigM = 100000
    # Dec.Var. Arcs traversed by vehicles
    m.xvar_var = m.binary_var_cube(n+2, n+2, K, name = "xvar")
    # Dec.Var. Starting time of service of customers
    m.w_var = m.continuous_var_matrix(n+2, K, name = "w")
    # Covering constraints
    m.covering_constrs = []
    for i in range(1,n+1): 
        cust_satisf_constr = m.sum(
            m.xvar_var[i,j,k] for j in range(1,n+2) for k in range(K) if j != i) == 1
        m.add_constraint(cust_satisf_constr)
        m.covering_constrs.append(cust_satisf_constr)
    # Route structure constraints
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
    # Capacity constraints
    m.capacity_constrs = []
    for k in range(K):
        capacity_constr = m.sum(q[i] * m.xvar_var[i,j,k] for i in range(n+2) \
                                for j in range(n+2) if i != j) <= Q
        m.add_constraint(capacity_constr)
        m.capacity_constrs.append(capacity_constr)
    # Time constraints
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

def main():
    instance_name, file_path, n = select_instance()
    K, Q, x, y, q, a, b = read_instance(file_path, n)
    d = create_distance_matrix(x, y)
    VRPTW_Model = create_model(K, Q, n, q, a, b, d)

main()