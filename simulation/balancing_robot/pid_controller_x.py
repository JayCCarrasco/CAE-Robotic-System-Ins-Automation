import numpy as np

class PIDControllerX:
    def __init__(self, Kp_theta, Ki_theta, Kd_theta, Kp_x, Kd_x):
        # PID del ángulo
        self.Kp_theta = Kp_theta
        self.Ki_theta = Ki_theta
        self.Kd_theta = Kd_theta
        self.integral_theta = 0.0
        self.prev_theta_error = 0.0
        
        # PID de la posición
        self.Kp_x = Kp_x
        self.Kd_x = Kd_x

    def update(self, x, x_dot, theta, theta_dot, dt, x_ref=0.0):
        # Limit max force of the robot
        Fmax = 20

        # PID de la posición → setpoint para θ
        theta_ref = self.Kp_x * (x_ref - x) - self.Kd_x * x_dot

        # PID de ángulo
        theta_error = theta - theta_ref
        self.integral_theta += theta_error * dt
        derivative = (theta_error - self.prev_theta_error) / dt
        self.prev_theta_error = theta_error

        F = (self.Kp_theta * theta_error +
             self.Ki_theta * self.integral_theta +
             self.Kd_theta * derivative)
             #self.Kd_theta * theta_dot)
        F = np.clip(F, -Fmax, Fmax)
        return F
