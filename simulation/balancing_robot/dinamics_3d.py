#IMPORTANTE!!!!!
#CÓDIGO NO PROBADO, GENERADO POR IA!!!!!
#ESTÁ AQUÍ PARA PROBARLO MÁS ADELANTE CUANDO VALIDEMOS EL ROBOT EN 2D
#RECORDATORIO: NO HACER COPY/PASTE DE LOS CÓDIGOS, LAS IAS SON UNA HERRAMIENTA DE APOYO
#EL INGENIERO TIENE QUE SER CAPAZ DE DESARROLLAR EL PROGRAMA Y HACER USO DE LAS HERRAMIENTAS
#POR SÍ MISMO!!!!!!!!!!!!!!!!!!!!!


import numpy as np
from params import M, m, l, I, g, d, I_psi

def dynamics(t, y, F_L=0.0, F_R=0.0):
    # Estados
    x, y_pos, psi, v, theta, theta_dot, psi_dot = y

    # Fuerzas
    F = F_L + F_R
    tau_psi = (F_R - F_L) * d / 2

    # --- DINÁMICA DE PITCH (LINEALIZADA) ---
    D = (M + m)*(I + m*l**2) - (m*l)**2

    v_dot = (F*(I + m*l**2) - m**2 * g * l**2 * theta) / D
    theta_ddot = ((M + m)*m*g*l*theta - m*l*F) / D

    # --- DINÁMICA DE YAW ---
    psi_ddot = tau_psi / I_psi

    # --- CINEMÁTICA EN EL PLANO ---
    x_dot = v * np.cos(psi)
    y_dot = v * np.sin(psi)

    return [
        x_dot,        # ẋ
        y_dot,        # ẏ
        psi_dot,      # ψ̇
        v_dot,        # v̇
        theta_dot,    # θ̇
        theta_ddot,   # θ̈
        psi_ddot      # ψ̈
    ]
