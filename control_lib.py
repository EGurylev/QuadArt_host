class pid:

    def __init__(self, P, I, D, dt):
        self.P = P
        self.I = I
        self.D = D
        self.dt = dt
        self.integral = 0.0
        self.error_prev = 0.0
        self.setpoint = 0.0

    def evaluate(self, measured):      
        error = self.setpoint - measured
        self.integral += self.I * error * self.dt
        out = self.P * error + self.integral + self.D * (error - self.error_prev) / self.dt
        self.error_prev = error
        return out
