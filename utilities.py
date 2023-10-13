import numpy as np
import os
from sys import exit

def select_instance():
    """Enter solomon instance and select a number of customers"""
    print("Type instance ([r|c|rc][NNN]):")
    instance_name = None
    try:
        instance_name = input()
    except Exception:
        print("Invalid instance. Exit."); exit(1)
    if instance_name.endswith(".txt"):
        instance_name = instance_name[:-4]
    file_path = os.path.join("solomon-instances", instance_name+".txt")
    if not instance_name or not os.path.exists(file_path):
        print("Instance does not exists. Exit."); 3
    print("Select number of costumers (1-100):")
    n = None
    try:
        n = int(input())
    except Exception:
        print("Invalid number of costumers. Exit."); exit(1)
    if not n or n < 1 or n > 100:
        print("Invalid number of costumers. Exit."); exit(1)
        
    return instance_name, file_path, n
    
def read_instance(path, n):
    """Read selected instance from file"""
    stream = ""
    with open(path, "r") as file:
        stream = file.readlines()
    if stream == "":
        print("Error in reading file")
    else:
        print("Read file", path, "\n")
    K, Q = [int(i) for i in stream[4].split()]
    fields = ("CUST-NO.", "XCOORD.", "YCOORD.", "DEMAND", "READY-TIME", "DUE-DATE", "SERVICE-TIME")
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
    data.append(data[0]) # The depot is represented by two identical nodes 0 and n+1.
    data[-1]["CUST-NO."] = "51"
    x = []; y = []; q = []; a = []; b = []; 
    for customer in data:
        x.append(int(customer["XCOORD."]))
        y.append(int(customer["YCOORD."]))
        q.append(int(customer["DEMAND"]))
        a.append(int(customer["READY-TIME"]))
        b.append(int(customer["DUE-DATE"]))
        #s.append(int(customer["SERVICE-TIME"]))
    
    return K, Q, x, y, q, a, b
    
def create_distance_matrix(x, y):
    """Compute distance between every two nodes"""
    n = len(x)
    d = np.zeros((n,n))
    for i in range(n):
        for j in range(i+1,n):
            p1 = np.array([x[i], y[i]])
            p2 = np.array([x[j], y[j]])
            d[i,j] = d[j,i] = int(round(np.linalg.norm(p1-p2))) 
    
    return d
    
def compute_max_cost(d, a, b, n): 
    return max([b[i] + d[i,j] - a[j] for i in range(n+2) for j in range(n+2)])