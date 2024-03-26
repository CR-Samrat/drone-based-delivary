import numpy as np
from sympy import *

def gradT(f, x, a): #Evaluate gradient of f(x) at x=a, returns a column vector
    n=len(x)
    gr=np.zeros((1, n)) #row of n elements
    for i in range(n):
        gr[0][i]=diff(f, x[i]). evalf(subs={x[j]:a[j] for j in range(n)})
    return (gr.T) #transpose of gr


def EvHess(f,x,a): #Hessian of f(x) evaluated at x=a
    n=len(x)
    G=np.zeros((n,n))
    for i in range(n):
        for j in range(n):
            G[i,j]=(diff(diff(f,x[i]),x[j])).evalf(subs={x[k]:a[k] for k in range(n)})
    #print (G)
    return (G)

def NewtonMultiDimension(f, x, a):
    a_org = a
    n = len(x)
    g_k = gradT(f,x,a)
    # x_{k+1}=x_k - inverse of Hessian*gradient's transpose
    while (np.dot(g_k.T, g_k) > 0.000001):
        H = EvHess(f, x, a)
        if (np.linalg.det(H) == 0):
            print("No improvement in Drone position is possible")
            return (a_org, 0)
        H_in = np.linalg.inv(H)
        a = a-np.dot(H_in,g_k).T[0,:]
        #f.evalf(subs={x[i]:a[i] for i in range(n)})
        #print (a)
        g_k = gradT(f,x,a)
    return (a, 1)

def Optimize_Drone_Position_Euclidean(F, L, Drone):
    num_of_F = len(F)
    num_of_L = len(L)
    Cluster_FL_points_X = np.zeros(num_of_F + num_of_L)
    Cluster_FL_points_Y = np.zeros(num_of_F + num_of_L)
    for i in range(num_of_F):
        Cluster_FL_points_X[i] = F[i][0] 
        Cluster_FL_points_Y[i] = F[i][1]
    for i in range(num_of_L):
        Cluster_FL_points_X[i + num_of_F] = L[i][0] 
        Cluster_FL_points_Y[i + num_of_F] = L[i][1]

    #print("F: ", F)
    #print("L: ", L)
    #print("Cluster_FL_points_X: ", Cluster_FL_points_X)

    x = symbols('x:2')
    f_dist = ((Cluster_FL_points_X[0] - x[0])**2 + (Cluster_FL_points_Y[0] - x[1])**2)**(0.5)
    for i in range(1, (num_of_F + num_of_L)):
        f_dist = f_dist + ((Cluster_FL_points_X[i] - x[0])**2 + (Cluster_FL_points_Y[i] - x[1])**2)**(0.5)
     
    (Drone, ret) = NewtonMultiDimension(f_dist, x, Drone) #minimize f_dist
    print("New Drone:", Drone)

    return (Drone, ret)