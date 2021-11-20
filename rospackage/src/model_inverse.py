"""
@author: Samuel Faucher

Functionnal code for the controler
"""

import numpy as np


def cart2pol(xCible, yCible):
    d = np.sqrt(xCible**2 + yCible**2)
    theta = np.arctan2(yCible, xCible)
    return d, theta


def matriceRot(vecteur, angle):
    R = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    return R @ vecteur


def model_inverse(xCible, yCible, v_wind, ro_snow, force_45):
    '''
    v_wind: wind speed in [x, y] format
    ro_snow: snow density in kg/m³ -> light_snow=300, heavy_snow=800
    '''
    # Snow attributes
    r = 0.1  # m
    A = r**2 * np.pi  # m^2
    ro = 1.2  # kg/m^3 -> air
    Cd = 0.47
    m = ro_snow * ((4/3) * np.pi * r**3)  #kg

    # Constantes
    g = 9.81  # m/s^2
    v_motor = 500  # rpm -> nominal désiré
    v_out = v_motor / 60 * 2 * np.pi * 0.2  # m/s

    # Base change
    d, theta_0 = cart2pol(xCible, yCible)
    v_wind_r = matriceRot(v_wind, theta_0)
    dir_wind_x = -1*np.sign(v_out - v_wind_r[0])
    dir_wind_y = -1*np.sign(v_wind_r[1])
    print(f'distance demande: {d}')

    # Max dist before 45deg
    phi_max = np.deg2rad(45)
    tf_max = 2 * v_out * np.sin(phi_max) / g
    delta_rx_max = (1 / (4 * m)) * ro * Cd * A * (v_out - v_wind_r[0])**2 * tf_max**2
    d_max = (v_out**2 * np.sin(2*phi_max) / g) + dir_wind_x*delta_rx_max
    print(f'distance max avec vitesse nom: {d_max}')

    # Blow strategy
    if d < d_max and not force_45:
        phi_0 = 0.5 * np.arcsin((d * g) / (v_out**2))
        tf = 2 * v_out * np.sin(phi_0) / g

        delta_ry = (1 / (4 * m)) * ro * Cd * A * (0 - v_wind_r[1])**2 * tf**2
        delta_rx = (1 / (4 * m)) * ro * Cd * A * (v_out - v_wind_r[0])**2 * tf**2

        phi = 0.5 * np.arcsin(((d - dir_wind_x*delta_rx) * g) / v_out**2)
        theta = theta_0 + (dir_wind_y*np.arctan2(delta_ry, d))
    else:
        phi = phi_max
        v_out_0 = np.sqrt((d*g) / np.sin(2*phi))

        tf = 2 * v_out_0 * np.sin(phi) / g

        delta_ry = (1 / (4 * m)) * ro * Cd * A * (0-v_wind_r[1])**2 * tf**2
        delta_rx = (1 / (4 * m)) * ro * Cd * A * (v_out_0 - v_wind_r[0])**2 * tf**2

        v_out = np.sqrt(((d - dir_wind_x*delta_rx) * g) / np.sin(2 * phi))
        theta = theta_0 + (dir_wind_y*np.arctan2(delta_ry, d))

    # WARNING : rad needed for coppelia
    return theta, phi, v_out


