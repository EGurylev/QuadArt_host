import numpy as np
import control
import scipy as sc
import math

def d2c(sys, method='zoh'):
    flag = 0
    if isinstance(sys, control.TransferFunction):
        sys = control.tf2ss(sys)
        flag = 1

    a = sys.A
    b = sys.B
    c = sys.C
    d = sys.D
    Ts = sys.dt
    n = np.shape(a)[0]
    nb = np.shape(b)[1]
    nc = np.shape(c)[0]
    tol=1e-12
    
    if method == 'zoh':
        if n == 1:
            if b[0,0] == 1:
                A = 0
                B = b / sys.dt
                C = c
                D = d
        else:
            tmp1 = np.hstack((a, b))
            tmp2 = np.hstack((np.zeros((nb, n)),np.eye(nb)))
            tmp = np.vstack((tmp1, tmp2))
            s = sc.linalg.logm(tmp)
            s = s / Ts
            s = np.real(s)
            A = s[0:n, 0:n]
            B = s[0:n, n:n + nb]
            C = c
            D = d
    elif method == 'foh':
        a = np.mat(a)
        b = np.mat(b)
        c = np.mat(c)
        d = np.mat(d)
        Id = np.mat(np.eye(n))
        A = sc.linalg.logm(a) / Ts
        A = np.real(np.around(A, 12))
        Amat = np.mat(A)
        B = (a - Id) ** (-2) * Amat ** 2 * b * Ts
        B = np.real(np.around(B, 12))
        Bmat = np.mat(B)
        C = c
        D = d - C * (Amat ** (-2) / Ts * (a - Id) - Amat ** (-1)) * Bmat
        D = np.real(np.around(D, 12))
    elif method == 'bi':
        a = np.mat(a)
        b = np.mat(b)
        c = np.mat(c)
        d = np.mat(d)    
        I = np.mat(np.eye(n, n))
        tk = 2 / math.sqrt(Ts)
        A = (2 / Ts) * (a - I) * sc.linalg.inv(a + I)
        iab = sc.linalg.inv(I + a) * b
        B = tk * iab
        C = tk * (c * sc.linalg.inv(I + a))
        D = d - (c * iab)
    else:
        print "Method not supported"
        return
    
    sysc = control.StateSpace(A, B, C, D)
    if flag == 1:
        sysc = control.ss2tf(sysc)
    return sysc
