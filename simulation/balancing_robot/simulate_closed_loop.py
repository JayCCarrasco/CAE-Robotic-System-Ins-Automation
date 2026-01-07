import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from dynamics import dynamics
from pid_controller import PIDController

pid = PIDController(Kp = 100, Ki = 0, Kd = 5)
y0 = [0.0, 0.0, 0.1, 0.0]

def closed_loop(t, y):
    dt = 0.01
    F = pid.update(y[2], y[3], dt)
    return dynamics(t, y, F)

t_span = (0, 2)
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