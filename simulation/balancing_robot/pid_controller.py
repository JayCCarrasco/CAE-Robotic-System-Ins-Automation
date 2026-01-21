class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, theta, theta_dot, dt):
        self.integral += theta * dt
        derivative = (theta - self.prev_error) / dt
        self.prev_error = theta
        F = self.Kp * theta + self.Ki * self.integral + self.Kd * theta_dot #self.Kd * theta_dot
        return F 