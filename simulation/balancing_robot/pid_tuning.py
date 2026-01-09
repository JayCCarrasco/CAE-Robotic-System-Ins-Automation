from scipy.optimize import minimize
from pid_controller_x import PIDControllerX
from scipy.integrate import solve_ivp
from dynamics import dynamics
import numpy as np

y0 = [0.0, 0.0, 0.1, 0.0]

def cost(K):
    Kp_t, Kd_t, Kp_x, Kd_x = K
    
    pid = PIDControllerX(
        Kp_theta=Kp_t, Ki_theta=0,
        Kd_theta=Kd_t,
        Kp_x=Kp_x, Kd_x=Kd_x
    )

    def closed_loop(t, y):
        dt = 0.01
        F = pid.update(y[0], y[1], y[2], y[3], dt)
        return dynamics(t, y, F)

    sol = solve_ivp(closed_loop, (0, 3), y0, max_step=0.01)

    x = sol.y[0]
    theta = sol.y[2]

    return np.trapz(theta**2 + 0.1*x**2, sol.t)

K0 = [100, 10, 0.2, 0.05]
res = minimize(cost, K0, method="Nelder-Mead")

print(res.x)