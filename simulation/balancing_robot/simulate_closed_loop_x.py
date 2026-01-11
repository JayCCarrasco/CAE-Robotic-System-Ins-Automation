import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from dynamics import dynamics
from pid_controller_x import PIDControllerX

# [77.81343703  8.50544776  4.99995538  1.99995165]
#mejores (PID Tuning): Kp_theta = 7418, Ki_theta = 0, Kd_theta = 249, Kp_x = 0.2, Kd_x = 0.067 
#mejores (Mezclando): Kp_theta = 77.81, Ki_theta = 0, Kd_theta = 8.5, Kp_x = 0.2, Kd_x = 0.067
#mejores (GPT_pid_tuning): 

pid = PIDControllerX(Kp_theta = 71.16, Ki_theta = 0, Kd_theta = 5.21, Kp_x = 0.095, Kd_x = 0.053)
y0 = [0.0, 0.0, 0.1, 0.0]

def closed_loop(t, y):
    dt = 0.01
    F = pid.update( y[0], y[1], y[2], y[3], dt)
    return dynamics(t, y, F)

t_span = (0, 10)
t_eval = np.linspace(*t_span, 500)
sol = solve_ivp(closed_loop, t_span, y0, t_eval = t_eval)

plt.figure(figsize=(10, 5))
plt.plot(sol.t, sol.y[0], label = "x [m]")
plt.plot(sol.t, sol.y[2], label = "theta [rad]")
plt.xlabel("Time [s]")
plt.ylabel("State")
plt.legend()
plt.grid(True)
plt.show()