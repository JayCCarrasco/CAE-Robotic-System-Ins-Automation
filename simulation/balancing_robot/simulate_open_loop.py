import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from dynamics import dynamics

#Initial state
y0 = [0.0, 0.0, 0.1, 0.0]

t_span = (0, 5)
t_eval = np.linspace(*t_span, 500)

sol = solve_ivp(dynamics, t_span, y0, t_eval = t_eval)

plt.figure(figsize=(10, 5))
plt.plot(sol.t, sol.y[0], label = "x [m]")
plt.plot(sol.t, sol.y[2], label = "theta [rad]")
plt.xlabel("Time [s]")
plt.ylabel("State")
plt.legend()
plt.grid(True)
plt.show()
