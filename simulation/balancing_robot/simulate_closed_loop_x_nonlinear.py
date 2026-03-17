import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from dynamics_nonlinear import dynamics
from pid_controller_x import PIDControllerX

pid = PIDControllerX(Kp_theta = 100, Ki_theta = 0, Kd_theta = 40.0, Kp_x = 0.1, Ki_x = 0, Kd_x = 0.7)
y0 = [0.0, 0.0, 0.1, 0.0]

#def closed_loop(t, y):
#    dt = 0.01
#    F = pid.update( y[0], y[1], y[2], y[3], dt)
#    return dynamics(t, y, F)

last_t = 0.0

def closed_loop(t, y):
    global last_t
    dt = t - last_t if last_t != 0 else 0.01
    last_t = t
    F = pid.update(y[0], y[1], y[2], y[3], dt)
    return dynamics(t, y, F)

t_span = (0, 20)
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