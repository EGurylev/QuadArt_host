import root as r
import control.matlab as ml
import matplotlib.pyplot as plt
import sympy as sp
import numpy as np

# Quadrotor model in sympy
x, y, z = sp.symbols('x, y, z', real=True)
v_x, v_y, v_z = sp.symbols('v_x, v_y, v_z', real=True)
phi, theta, psi = sp.symbols('phi, theta, psi', real=True)
omega_x, omega_y, omega_z = sp.symbols('omega_x, omega_y, omega_z', real=True)

# Body linear velocities    
f1 = omega_z * v_y - omega_y * v_z + r.g * sp.sin(theta)
f2 = omega_x * v_z - omega_z * v_x - r.g * sp.cos(theta) * sp.sin(phi)
f3 = omega_y * v_x - omega_x * v_y - r.g * sp.cos(theta) * sp.cos(phi) + \
    r.force / r.mass
# Global positions
f4 = sp.cos(theta) * sp.cos(psi) * v_x + \
    (sp.sin(phi) * sp.sin(theta) * sp.cos(psi) - sp.cos(phi) * sp.sin(psi)) * v_y + \
    (sp.cos(phi) * sp.sin(theta) * sp.cos(psi) + sp.sin(phi) * sp.sin(psi)) * v_z
f5 = sp.cos(theta) * sp.sin(psi) * v_x + \
    (sp.sin(phi) * sp.sin(theta) * sp.sin(psi) + sp.cos(phi) * sp.cos(psi)) * v_y + \
    (sp.cos(phi) * sp.sin(theta) * sp.sin(psi) - sp.sin(phi) * sp.cos(psi)) * v_z
f6 = -sp.sin(theta) * v_x + sp.sin(phi) * sp.cos(theta) * v_y + \
    sp.cos(phi) * sp.cos(theta) * v_z  
# Global angles
f7 = omega_x + sp.sin(phi) * sp.tan(theta) * omega_y + sp.cos(phi) * \
    sp.tan(theta) * omega_z
f8 = sp.cos(phi) * omega_y - sp.sin(phi) * omega_z
f9 = sp.sin(phi) / sp.cos(theta) * omega_y + sp.cos(phi) \
    / sp.cos(theta) * omega_z
# Body angular velocities
f10 = (r.j_yy - r.j_zz) / r.j_xx * omega_y * omega_z + r.tau_phi / r.j_xx
f11 = (r.j_zz - r.j_xx) / r.j_yy * omega_x * omega_z + r.tau_theta / r.j_yy
f12 = (r.j_xx - r.j_yy) / r.j_zz * omega_x * omega_y + r.tau_psi / r.j_zz

# Linearize quadrotor model
F = sp.Matrix([f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12])
J = F.jacobian([v_x, v_y, v_z, x, y, z, phi, theta, psi, omega_x, omega_y, omega_z])
# Substitute linearization point and obtain matrix A (state-space form)
A = J.subs([(v_x, 0), (v_y, 0), (v_z, 0), (phi, 0), (theta, 0), (psi, 0),
    (omega_x, 0), (omega_y, 0), (omega_z, 0)])
    
