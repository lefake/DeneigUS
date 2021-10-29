# -*- coding: utf-8 -*-
"""
Created on Sun Jun 21 11:07:03 2020

@author: Pierre-Olivier et Samuel Faucher
"""

import numpy as np
from GraphGeneration import GraphGeneration
from dir_traj_wind import dir_traj_wind
from model_inverse import model_inverse

    
if __name__ == '__main__':
    theta, phi, v_out = model_inverse(20, 0, [20,0], 300)
    print(f'theta: {np.rad2deg(theta)}')
    print(f'phi: {np.rad2deg(phi)}')
    print(f'v_out: {v_out}')


    point = 1000
    t_max = 1  # sec
    
    teta_rot = theta #-90/180*np.pi  # rad
    teta_elev = phi #45/180*np.pi  # rad
    v_snow = v_out  # 5 m/s
    v_wind = [20,0,0]  # m/s -> x,y,z
    
    traj_with_time, speed, accel, Fd = dir_traj_wind(v_snow, v_wind, teta_elev, teta_rot, t_max, point)
    print(traj_with_time)

    traj = traj_with_time[0:3,:]
    t = traj_with_time[3,:]
    
    graph_generator = GraphGeneration()
    graph_generator.update_trajectory(traj)
    graph_generator.plot_3dtrajectory()
    '''
    graph_generator.update_externals_forces(Fd)
    graph_generator.plot_externals_forces()
    
    graph_generator.update_snow_speed(speed)
    graph_generator.plot_snow_speed()
    
    graph_generator.update_snow_accel(accel)
    graph_generator.plot_snow_accel()
    '''