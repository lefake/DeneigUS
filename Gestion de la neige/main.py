# -*- coding: utf-8 -*-
"""
Created on Sun Jun 21 11:07:03 2020

@author: Pierre-Olivier et Samuel Faucher
"""

from GraphGeneration import GraphGeneration
from air_drag import air_drag
import numpy as np

from dir_traj_wind import dir_traj_wind


    
if __name__ == '__main__':
    
    
    point = 1000
    t_max = 1
    
    teta_rot = -90/180*np.pi
    teta_elev = 45/180*np.pi
    v_snow = 5
    v_wind = [0,5,0]
    
    traj_with_time, speed, accel, Fd = dir_traj_wind(v_snow, v_wind, teta_elev, teta_rot, t_max, point)
    

    traj = traj_with_time[0:3,:]
    t = traj_with_time[3,:]
    
    graph_generator = GraphGeneration()
    graph_generator.update_trajectory(traj)
    graph_generator.plot_3dtrajectory()
    
    graph_generator.update_externals_forces(Fd)
    graph_generator.plot_externals_forces()
    
    graph_generator.update_snow_speed(speed)
    graph_generator.plot_snow_speed()
    
    graph_generator.update_snow_accel(accel)
    graph_generator.plot_snow_accel()