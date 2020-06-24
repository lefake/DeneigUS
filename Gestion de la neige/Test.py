# -*- coding: utf-8 -*-
"""
Created on Sun Jun 21 12:33:32 2020

@author: Utilisateur
"""


def test_graph():

    time_max = 10
    point = 1000
    dt = time_max/point
    

    z = np.linspace(0,15,1000)
    x = np.cos(z)
    y = np.sin(z)
    t = np.linspace(0,dt,time_max)
    
    graph_genetator  = GraphGeneration()
    graph_genetator.update_trajectory([x,y,z])
    graph_genetator.plot_3dtrajectory()
    
def test_traj_without_wind():
    point = 1000
    t_max = 2
    
    teta_rot = 30/180*np.pi
    teta_elev = 45/180*np.pi
    v_snow = 2
    
    traj_with_time = dir_traj(v_snow, teta_elev, teta_rot, t_max, point)
    

    traj = traj_with_time[0:3,:]
    t = traj_with_time[3,:]
    
    graph_generator = GraphGeneration()
    graph_genetator.update_trajectory(traj)
    graph_generator.plot_3dtrajectory()