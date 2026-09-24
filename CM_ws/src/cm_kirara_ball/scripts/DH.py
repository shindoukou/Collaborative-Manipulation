import cmath
import  math
import numpy as np
def DH_mat(theta,d,alpha,a):
    R_z_theta = ([np.cos(theta), -np.sin(theta), 0, 0],[np.sin(theta), np.cos(theta), 0, 0],[0, 0, 1, 0],[0, 0, 0, 1])
    T_z_d = ([1, 0, 0, 0],[0, 1, 0, 0], [0, 0, 1, d],[0, 0, 0, 1])
    R_x_alpha = ([1, 0, 0, 0],[0, np.cos(alpha), -1*np.sin(alpha), 0], [0, np.sin(alpha), np.cos(alpha), 0],[0, 0, 0, 1])
    T_x_a = ([1, 0, 0, a],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1])
    A = np.dot(R_z_theta,T_z_d)
    B = np.dot(A,T_x_a)
    C = np.dot(B,R_x_alpha)
    
    return  C
def jacobi(theta,d,alpha,a):
    theta1 = theta[0, 0];theta2 = theta[1, 0];theta3 = theta[2, 0];theta4 = theta[3, 0];theta5 = theta[4, 0];theta6 = theta[5, 0]
    d1 = d[0, 0];d2 = d[1, 0];d3 = d[2, 0];d4 = d[3, 0];d5 = d[4, 0];d6 = d[5, 0]
    alpha1 = alpha[0, 0];alpha2 = alpha[1, 0];alpha3 = alpha[2, 0];alpha4 = alpha[3, 0];alpha5 = alpha[4, 0];alpha6 = alpha[5, 0]
    a1=a[0, 0];a2=a[1, 0];a3=a[2, 0];a4=a[3, 0];a5=a[4, 0];a6=a[5, 0]
    A1 = DH_mat(theta1, d1, alpha1, a1)
    A2 = DH_mat(theta2, d2, alpha2, a2)
    A3 = DH_mat(theta3, d3, alpha3, a3)
    A4 = DH_mat(theta4, d4, alpha4, a4)
    A5 = DH_mat(theta5, d5, alpha5, a5)
    A6 = DH_mat(theta6, d6, alpha6, a6)
    
    D1 = A1[0:3, 0:3]
    D2 = A2[0:3, 0:3]
    D3 = A3[0:3, 0:3]
    D4 = A4[0:3, 0:3]
    D5 = A5[0:3, 0:3]
    D6 = A6[0:3, 0:3]
    
    T1 = A1
    T2 = np.dot(T1, A2)
    T3 = np.dot(T2, A3)
    T4 = np.dot(T3, A4)
    T5 = np.dot(T4, A5)
    T6 = np.dot(T5, A6)
    
    R1 = D1
    R2 = np.dot(R1, D2)
    R3 = np.dot(R2, D3)
    R4 = np.dot(R3, D4)
    R5 = np.dot(R4, D5)
    R6 = np.dot(R5, D6)
    
    z0 = np.array([0, 0, 1])
    z1 = np.dot(R1, z0)
    z2 = np.dot(R2, z0)
    z3 = np.dot(R3, z0)
    z4 = np.dot(R4, z0)
    z5 = np.dot(R5, z0)
    z6 = np.dot(R6, z0)
    
    p1 = T1[0:3, 3]
    p2 = T2[0:3, 3]
    p3 = T3[0:3, 3]
    p4 = T4[0:3, 3]
    p5 = T5[0:3, 3]
    p6 = T6[0:3, 3]
    
    J_1 = np.mat(np.hstack((np.cross(z0, p6), np.array(z0)))).T
    J_2 = np.mat(np.hstack((np.cross(z1, p6 - p1), np.array(z1)))).T
    J_3 = np.mat(np.hstack((np.cross(z2, p6 - p2), np.array(z2)))).T
    J_4 = np.mat(np.hstack((np.cross(z3, p6 - p3), np.array(z3)))).T
    J_5 = np.mat(np.hstack((np.cross(z4, p6 - p4), np.array(z4)))).T
    J_6 = np.mat(np.hstack((np.cross(z5, p6 - p5), np.array(z5)))).T
    J_ = np.hstack((J_1, J_2, J_3, J_4, J_5, J_6))
    return  J_
def tran(theta,d,alpha,a):
    theta1 = theta[0, 0];theta2 = theta[1, 0];theta3 = theta[2, 0];theta4 = theta[3, 0];theta5 = theta[4, 0];theta6 = theta[5, 0]
    d1 = d[0, 0];d2 = d[1, 0];d3 = d[2, 0];d4 = d[3, 0];d5 = d[4, 0];d6 = d[5, 0]
    alpha1 = alpha[0, 0];alpha2 = alpha[1, 0];alpha3 = alpha[2, 0];alpha4 = alpha[3, 0];alpha5 = alpha[4, 0];alpha6 = alpha[5, 0]
    a1=a[0, 0];a2=a[1, 0];a3=a[2, 0];a4=a[3, 0];a5=a[4, 0];a6=a[5, 0]
    A1 = DH_mat(theta1, d1, alpha1, a1)
    A2 = DH_mat(theta2, d2, alpha2, a2)
    A3 = DH_mat(theta3, d3, alpha3, a3)
    A4 = DH_mat(theta4, d4, alpha4, a4)
    A5 = DH_mat(theta5, d5, alpha5, a5)
    A6 = DH_mat(theta6, d6, alpha6, a6)
    T1 = A1
    T2 = np.dot(T1, A2)
    T3 = np.dot(T2, A3)
    T4 = np.dot(T3, A4)
    T5 = np.dot(T4, A5)
    T6 = np.dot(T5, A6)

    return T6