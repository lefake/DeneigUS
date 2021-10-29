# -*- coding: utf-8 -*-
"""
Created on Sun Jun 21 12:31:59 2020

@author: Samuel Faucher
"""

import numpy as np


def dir_traj(v_out, angle_elev, angle_rot, t_total, nbr_echantillons):
    '''
    Compute numericaly the trajectory of the snow and its ending point
    INPUT:
        v_out: speed (m/s) of the snow when it leaves the machine
        angle_elev: elevation angle (rad) of the chute
        angle_rot: rotation angle (rad) of the chute
        t_total: total time (sec) to compute the trajectory
        nbr_echantillons:
    '''
    
    g = np.array([0,0,-9.81]) #m/s^2
    
    vflat = v_out*np.sin(angle_elev)
    vlast_x = vflat*np.cos(angle_rot) #m/s
    vlast_y = vflat*np.sin(angle_rot) #m/s
    vlast_z = v_out*np.cos(angle_elev) #m/s
    pos_0 = np.array([0, 0, 0.5, 0]) #m -> offset from the physical machine and time zero

    dt = t_total/nbr_echantillons #s
    pos = np.zeros((4, nbr_echantillons)) #m
    
    i = 0
    pos[:,i] = pos_0
    while(pos[2,i] >= 0.0):
        i += 1
        t = i*dt

        v_x = vlast_x + g[0]*dt
        v_y = vlast_y + g[1]*dt
        v_z = vlast_z + g[2]*dt

        pos[0,i] = 0.5*g[0]*t**2 + v_x*t + pos[0,(i-1)]
        pos[1,i] = 0.5*g[1]*t**2 + v_y*t + pos[1,(i-1)]
        pos[2,i] = 0.5*g[2]*t**2 + v_z*t + pos[2,(i-1)]
        pos[3,i] = t

        vlast_x = v_x
        vlast_y = v_y
        vlast_z = v_z
    pos[0,i:(nbr_echantillons)] = pos[0,i]
    pos[1,i:(nbr_echantillons)] = pos[1,i]
    pos[2,i:(nbr_echantillons)] = pos[2,i]
    
    return pos