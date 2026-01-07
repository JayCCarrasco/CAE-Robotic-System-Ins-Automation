import numpy as np
from params import M, m, l, I, g

def dynamics(t, y, F = 0):
    x, x_dot, theta, theta_dot = y

    #LINEAR EQUATIONS
    D = (M + m)*(I + m*l)**2 - (m*l)**2
    x_ddot = (F*(I + m*l**2) - m**2 * g * l**2 * theta) / D
    theta_ddot = ((M + m)*m*g*l*theta - m*l*F) / D


    #NON-LINEAR EQUATIONS
    #denom = I*(M+m) + M*m*l**2
    #theta_ddot = (-(M+m)*g*l*np.sin(theta)
    #              - m*l*theta_dot**2*np.sin(theta)*np.cos(theta)
    #              + F*m*l*np.cos(theta)) / denom
    #x_ddot = (F + m*l*(theta_dot**2*np.sin(theta)
    #          - theta_ddot*np.cos(theta))) / (M + m)


    return [x_dot, x_ddot, theta_dot, theta_ddot]
