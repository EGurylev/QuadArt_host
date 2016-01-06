import root as r

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

# Controller initialization
ZController = pid(0.5,0,0.5,r.dt)
ZController.setpoint = 0.0
PsiController = pid(0.5,0,0.5,r.dt)
PsiController.setpoint = 0

XController = pid(0.2,0,0.1,r.dt)
XController.setpoint = -1
ThetaController = pid(0.1,0,0.1,r.dt)

YController = pid(0.2,0,0.1,r.dt)
YController.setpoint = 2.6
PhiController = pid(0.1,0,0.1,r.dt)

def control_loop(y):
    # Z control
    F = ZController.evaluate(y[1][5])
    F += r.M * r.g
    # Cascade control of X
    ThetaController.setpoint = XController.evaluate(y[1][3])# control X
    tau_theta = ThetaController.evaluate(y[1][7])# control roll
    # Cascade control of Y
    PhiController.setpoint = -YController.evaluate(y[1][4])# control Y
    tau_phi = PhiController.evaluate(y[1][6])# control pitch
    # Yaw control
    tau_psi = PsiController.evaluate(y[1][8])

    return (F, tau_theta, tau_phi, tau_psi)
