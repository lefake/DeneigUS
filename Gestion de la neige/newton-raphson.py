"""
@author: Samuel Faucher

Piste de solution abandonnée pour l'instant
-model_inverse.py- est la solution en focus pour l'instant
"""

import numpy as np
from numpy import sin, cos
from scipy import optimize
from sympy import sin, cos, Matrix
from sympy.abc import theta, phi, t
from time import time


#def pos2cmd(x, y, z, v_air):
    '''
    Compute numericaly the command needed to trow the snow at a certain position
    INPUT:
        v_air: speed (m/s) of the wind in 3D [x,y,z] vector
        x: goal position (m) in the x axis
        y: goal position (m) in the y axis
        z: goal position (m) in the z axis
    OUTPUT:
        v_out: speed (m/s) of the snow when it leaves the machine
        angle_elev: elevation angle (rad) of the chute
        angle_rot: rotation angle (rad) of the chute
        t_total: total time (sec) for the snow to land

    time1 = time()
    g = 9.81  # m/s^2

    # Speed of the snow (motor speed fixed for now)
    v_motor = 4000*0.4  # rpm
    v_out = v_motor/60*2*np.pi*0.2  # m/s
    vflat = v_out * cos(phi)
    vx = vflat * cos(theta)  # m/s
    vy = vflat * sin(theta)  # m/s
    vz = v_out * sin(phi)  # m/s

    # offset between the center of the prototype and the end point of the snow chute
    x0 = 0.675  # m
    y0 = 0.090  # m
    z0 = 0.000  # m

    # Snow attributes
    A = 0.2 * 0.1  # m^2
    ro = 0.1 * 10**2  # kg/m^3
    Cd = 0.5

    # Air drag
    Fd_x = 0.5 * ro * Cd * A * (vx-v_air[0])**2
    Fd_y = 0.5 * ro * Cd * A * (vy-v_air[1])**2
    Fd_z = 0.5 * ro * Cd * A * (vz-v_air[2])**2 + g

    # Direct equations
    F1 = -0.5*Fd_x* t**2 + vx* t*cos(theta) + (x0-x)
    F2 = -0.5*Fd_y* t**2 + vy* t*sin(theta) + (y0-y)
    F3 = -0.5*Fd_z* t**2 + vz* t*sin(phi) + (z0-z)

    # Jacobian
    F = Matrix([F1, F2, F3])
    print(F)
    J = F.jacobian(Matrix([theta, phi, t]))
    print(J)
    Jnum = J.subs([(theta, 0), (phi, 0), (t, 5)])
    print(Jnum)
    Jinv = Jnum.inv()
    print(Jinv)

    time2 = time()
    print('Function took {:.3f} ms'.format((time2 - time1) * 1000.0))

    return Jnum

def nextCMD(inv_Jnum, Xn, F_xn):
    return Xn - inv_Jnum @ F_xn
'''

def F(Xn):
    # Input
    [theta, phi, t] = Xn

    # Goal -> TODO sortir de la fonction
    x = 1
    y = 1
    z = 0

    # For the wind
    g = 9.81  # m/s^2
    v_air = [-0.01, -0.01, 0]

    # Speed of the snow (motor speed fixed for now)
    v_motor = 4000 * 0.4  # rpm
    v_out = v_motor / 60 * 2 * np.pi * 0.2  # m/s
    vflat = v_out * sin(phi)
    vx = vflat * cos(theta)  # m/s
    vy = vflat * sin(theta)  # m/s
    vz = v_out * cos(phi)  # m/s

    # offset between the center of the prototype and the end point of the snow chute
    x0 = 0.675  # m
    y0 = 0.090  # m
    z0 = 0.000  # m

    # Snow attributes
    A = 0.2 * 0.1  # m^2
    ro = 0.1 * 10 ** 2  # kg/m^3
    Cd = 0.5

    # Air drag
    Fd_x = 0.5 * ro * Cd * A * (vx - v_air[0]) ** 2
    Fd_y = 0.5 * ro * Cd * A * (vx - v_air[1]) ** 2
    Fd_z = 0.5 * ro * Cd * A * (vx - v_air[2]) ** 2 + g

    # Direct equations
    F1 = -0.5 * Fd_x * t ** 2 + vx * t * cos(theta) + (x0 - x)
    F2 = -0.5 * Fd_y * t ** 2 + vy * t * sin(theta) + (y0 - y)
    F3 = -0.5 * Fd_z * t ** 2 + vz * t * sin(phi) + (z0 - z)

    # Out
    F = np.array([F1, F2, F3])

    return F

if __name__ == '__main__':
    root = optimize.newton(F, [0.1,0.1,5], maxiter=25)
    print(root)