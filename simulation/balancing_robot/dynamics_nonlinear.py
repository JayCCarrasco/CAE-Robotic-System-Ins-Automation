import numpy as np
from params import M, m, l, I, g

def dynamics(t, y, F = 0):
    x, x_dot, theta, theta_dot = y
    
    # aceleración horizontal del eje de las ruedas
    x_ddot = (
        F 
        + m * l * (theta_dot**2) * np.sin(theta)
        - m * g * np.sin(theta) * np.cos(theta)
    ) / (M + m * (np.sin(theta)**2))
    
    # aceleración angular del cuerpo
    theta_ddot = (g * np.sin(theta) - x_ddot * np.cos(theta)) / l
    
    return [x_dot, x_ddot, theta_dot, theta_ddot]