#Code 100% CHAT-GPT MADE

import numpy as np
from scipy.optimize import minimize
from scipy.integrate import solve_ivp

from pid_controller_x import PIDControllerX
from dynamics import dynamics

# -----------------------------
# Parámetros de simulación
# -----------------------------
T_END = 3.0
N_STEPS = 3000
t_eval = np.linspace(0.0, T_END, N_STEPS)

# Condiciones iniciales (puedes añadir más luego)
initial_conditions = [
    [0.0, 0.0, 0.1, 0.0],
]

# -----------------------------
# Función de coste
# -----------------------------
def cost(u):
    """
    u ∈ [0,1]^4  (variables normalizadas)
    """

    # Desnormalización de ganancias
    Kp_t = 50  + 150 * u[0]
    Kd_t = 5   + 45  * u[1]
    Kp_x = 1.0 * u[2]
    Kd_x = 0.1 * u[3]

    total_cost = 0.0

    for y0 in initial_conditions:

        pid = PIDControllerX(
            Kp_theta=Kp_t, Ki_theta=0.0, Kd_theta=Kd_t,
            Kp_x=Kp_x, Kd_x=Kd_x
        )

        last_t = None
        F_log = []

        def closed_loop(t, y):
            nonlocal last_t

            dt = 0.0 if last_t is None else t - last_t
            last_t = t

            F = pid.update(
                x=y[0],
                x_dot=y[1],
                theta=y[2],
                theta_dot=y[3],
                dt=dt
            )

            F_log.append(F)
            return dynamics(t, y, F)

        sol = solve_ivp(
            closed_loop,
            (0.0, T_END),
            y0,
            t_eval=t_eval,
            max_step=0.01,
            rtol=1e-6,
            atol=1e-8
        )

        # Penalización dura si explota
        if not sol.success or np.any(np.isnan(sol.y)):
            return 1e6

        x = sol.y[0]
        theta = sol.y[2]
        F = np.array(F_log)

        # Penalización si se cae
        if np.max(np.abs(theta)) > 1.0:
            return 1e6

        # Ajuste de longitudes por seguridad
        min_len = min(len(theta), len(F))
        theta = theta[:min_len]
        x = x[:min_len]
        F = F[:min_len]
        t = sol.t[:min_len]

        # Coste tipo LQR continuo
        J = np.trapz(
            theta**2 +
            0.1 * x**2 +
            0.001 * F**2,
            t
        )

        total_cost += J

    return total_cost / len(initial_conditions)


# -----------------------------
# Optimización
# -----------------------------
u0 = [0.5, 0.5, 0.2, 0.2]   # punto inicial normalizado
bounds = [(0.0, 1.0)] * 4

res = minimize(
    cost,
    u0,
    method="Powell",
    bounds=bounds,
    options={"maxiter": 200, "disp": True}
)

# -----------------------------
# Resultados finales
# -----------------------------
u_opt = res.x

K_opt = {
    "Kp_theta": 50  + 150 * u_opt[0],
    "Kd_theta": 5   + 45  * u_opt[1],
    "Kp_x":     1.0 * u_opt[2],
    "Kd_x":     0.1 * u_opt[3],
}

print("\nGanancias óptimas:")
for k, v in K_opt.items():
    print(f"{k}: {v:.4f}")
