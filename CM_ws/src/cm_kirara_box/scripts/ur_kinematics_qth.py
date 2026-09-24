import numpy as np

def forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6):
    
    d1 = 0.089159
    a2 = -0.425
    a3 = -0.39225
    d4 = 0.10915
    d5 = 0.09465
    d6 = 0.0823

    
    T_base = np.array([[-1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, d1],
                      [0, 0, 0, 1]])

    
    T1 = np.array([[np.cos(theta1), -np.sin(theta1), 0, 0],
                  [np.sin(theta1), np.cos(theta1), 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

    T2 = np.array([[np.cos(theta2), -np.sin(theta2), 0, a2*np.cos(theta2)],
                  [np.sin(theta2), np.cos(theta2), 0, a2*np.sin(theta2)],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

    T3 = np.array([[np.cos(theta3), -np.sin(theta3), 0, a3*np.cos(theta3)],
                  [np.sin(theta3), np.cos(theta3), 0, a3*np.sin(theta3)],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

    T4 = np.array([[np.cos(theta4), -np.sin(theta4), 0, 0],
                  [0, 0, 1, d4],
                  [-np.sin(theta4), -np.cos(theta4), 0, 0],
                  [0, 0, 0, 1]])

    T5 = np.array([[np.cos(theta5), -np.sin(theta5), 0, 0],
                  [0, 0, -1, -d5],
                  [np.sin(theta5), np.cos(theta5), 0, 0],
                  [0, 0, 0, 1]])

    T6 = np.array([[np.cos(theta6), -np.sin(theta6), 0, 0],
                  [0, 0, 1, d6],
                  [-np.sin(theta6), -np.cos(theta6), 0, 0],
                  [0, 0, 0, 1]])

    
    T_end = T_base.dot(T1).dot(T2).dot(T3).dot(T4).dot(T5).dot(T6)

    
    position = T_end[0:3, 3]
    orientation = T_end[0:3, 0:3]

    return position, orientation

def joint(theta1, theta2, theta3, theta4, theta5):
    
    d1 = 0.089159
    a2 = -0.425
    a3 = -0.39225
    d4 = 0.10915
    d5 = 0.09465
    d6 = 0.0823

    
    T_base = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, d1],
                      [0, 0, 0, 1]])

    
    T1 = np.array([[np.cos(theta1), -np.sin(theta1), 0, 0],
                  [np.sin(theta1), np.cos(theta1), 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

    T2 = np.array([[np.cos(theta2), -np.sin(theta2), 0, a2*np.cos(theta2)],
                  [np.sin(theta2), np.cos(theta2), 0, a2*np.sin(theta2)],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

    T3 = np.array([[np.cos(theta3), -np.sin(theta3), 0, a3*np.cos(theta3)],
                  [np.sin(theta3), np.cos(theta3), 0, a3*np.sin(theta3)],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

    T4 = np.array([[np.cos(theta4), -np.sin(theta4), 0, 0],
                  [0, 0, 1, d4],
                  [-np.sin(theta4), -np.cos(theta4), 0, 0],
                  [0, 0, 0, 1]])

    T5 = np.array([[np.cos(theta5), -np.sin(theta5), 0, 0],
                  [0, 0, -1, -d5],
                  [np.sin(theta5), np.cos(theta5), 0, 0],
                  [0, 0, 0, 1]])

    p = []
    T = T_base.dot(T1)
    p.append(T[0:3, 3])
    T = T_base.dot(T1).dot(T2)
    p.append(T[0:3, 3])
    T = T_base.dot(T1).dot(T2).dot(T3)
    p.append(T[0:3, 3])
    T = T_base.dot(T1).dot(T2).dot(T3).dot(T4)
    p.append(T[0:3, 3])
    T = T_base.dot(T1).dot(T2).dot(T3).dot(T4).dot(T5)
    p.append(T[0:3, 3])

    return p


