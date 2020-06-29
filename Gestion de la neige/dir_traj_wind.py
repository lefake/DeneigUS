# -*- coding: utf-8 -*-
"""
Created on Mon Jun 29 10:57:04 2020

@author: Pierre-Olivier
"""


import numpy as np
from air_drag import air_drag


def dir_traj_wind(v_out, v_wind, angle_elev, angle_rot, t_total, nbr_echantillons):
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
    m = 1 #kg
    
    Vflat = v_out*np.sin(angle_elev)
    vlast_x = Vflat*np.cos(angle_rot) #m/s
    vlast_y = Vflat*np.sin(angle_rot) #m/s
    vlast_z = v_out*np.cos(angle_elev) #m/s
    pos_0 = np.array([0, 0, 0.5, 0]) #m -> offset from the physical machine and time zero

    dt = t_total/nbr_echantillons #s
    
    pos = np.zeros((4, nbr_echantillons)) #m
    speed = np.zeros((3, nbr_echantillons))
    Fd = np.zeros((3, nbr_echantillons)) #N
    accel = np.zeros((3, nbr_echantillons))
    
    i = 0
    pos[:,i] = pos_0
    while(pos[2,i] >= 0.0 and i<nbr_echantillons-1):
        i += 1
        t = i*dt
        
        g[0] = -1*np.sign(vlast_x-v_wind[0]) * Fd[0,i-1]/m
        g[1] = -1*np.sign(vlast_y-v_wind[1]) * Fd[1,i-1]/m
        g[2] = -1*np.sign(vlast_z-v_wind[2]) * Fd[2,i-1]/m - 9.81
        
        accel[:,i] = g
        
        v_x = vlast_x + g[0]*dt
        v_y = vlast_y + g[1]*dt
        v_z = vlast_z + g[2]*dt
        
        speed[:,i] = [v_x,v_y,v_z]
        Fd[:,i] = air_drag([v_x,v_y,v_z], v_wind)

        pos[0,i] = v_x*dt + pos[0,(i-1)]
        pos[1,i] = v_y*dt + pos[1,(i-1)]
        pos[2,i] = v_z*dt + pos[2,(i-1)]
        pos[3,i] = t

        vlast_x = v_x
        vlast_y = v_y
        vlast_z = v_z

    pos[0,i:(nbr_echantillons)] = pos[0,i]
    pos[1,i:(nbr_echantillons)] = pos[1,i]
    pos[2,i:(nbr_echantillons)] = pos[2,i]
    
    return pos, speed, accel, Fd