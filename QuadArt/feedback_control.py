import root as r

class pid:

    def __init__(self, p, i, d, alpha, dt, upper=float('Inf'), lower=float('-Inf'), sat_flag=False):
        self.p = p
        self.i = i
        self.d = d
        self.alpha = alpha
        self.dt = dt
        self.integral = 0.0
        self.error_prev = 0.0
        self.setpoint = 0.0
        self.upper = upper
        self.lower = lower
        self.sat_flag = sat_flag

    def evaluate(self, measured):       
        error = (self.setpoint - measured) * self.alpha + self.error_prev * (1 - self.alpha)
        self.integral += self.i * error * self.dt
        out = self.p * error + self.integral + self.d * (error - self.error_prev) / self.dt
        if self.sat_flag:
            if out > self.upper:
                out = self.upper
            if out < self.lower:
                out = self.lower
        self.error_prev = error
        return out

# Controller initialization (simulator)
z_controller = pid(0.5,0,0.5,1,r.dt)
z_controller.setpoint = 0.0
psi_controller = pid(0.5,0,0.5,1,r.dt)
psi_controller.setpoint = 0

x_controller = pid(0.2,0,0.1,1,r.dt)
x_controller.setpoint = -1
theta_controller = pid(0.1,0,0.1,1,r.dt)

y_controller = pid(0.2,0,0.1,1,r.dt)
y_controller.setpoint = 2.6
phi_controller = pid(0.1,0,0.1,1,r.dt)

# Controller initialization (real quad)
z_controller_cf = pid(80,30,90,0.35,1/30.0, upper=7000, lower=-7000, sat_flag=True)
z_controller_cf.setpoint = 0.0

x_controller_cf = pid(0.2,0.04,0.22,0.15,1/30.0, upper=20, lower=-20, sat_flag=True)
x_controller_cf.setpoint = 0.0

y_controller_cf = pid(0.2,0.06,0.22,0.15,1/30.0, upper=20, lower=-20, sat_flag=True)
y_controller_cf.setpoint = 80.0

def control_loop(y):
    # Z control
    force = z_controller.evaluate(y[1][5])
    force += r.mass * r.g
    # Cascade control of X
    theta_controller.setpoint = x_controller.evaluate(y[1][3])# control X
    tau_theta = theta_controller.evaluate(y[1][7])# control roll
    # Cascade control of Y
    phi_controller.setpoint = -y_controller.evaluate(y[1][4])# control Y
    tau_phi = phi_controller.evaluate(y[1][6])# control pitch
    # Yaw control
    tau_psi = psi_controller.evaluate(y[1][8])

    return (force, tau_theta, tau_phi, tau_psi)
