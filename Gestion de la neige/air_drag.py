# -*- coding: utf-8 -*-
"""
Created on Wed Jun 24 15:17:09 2020

@author: Pierre-Olivier Lagacé
"""

import numpy as np


def air_drag(v_snow, v_air):
    """
    Fd is the drag force
    ro is the dentity of the fluid
    v_snow is the speed of the object in each direction [x,y,z]
    v_air  is the speed of the fluid (air) in each direction [x,y,z]
    v is the speed of the object relatively to the fluid (air)
    A is the cross sectional area of the snow 
    Cd is the drag coefficient - a dimensionless number (~0.5)
    
    Returns Fd
    """
    
    # Snow attributes
    #A = 0.2*0.1  # m^2
    #ro = 0.1*10**2  # kg/m^3
    #Cd = 0.5

    r = 0.1  # m
    A = r ** 2 * np.pi  # m^2
    ro = 1.2  # kg/m^3 -> air
    Cd = 0.47
    m = 300 * ((4 / 3) * np.pi * r ** 3)  # kg
    
    v = np.subtract(v_snow,v_air)
    
    Fd_x = 0.5*ro*Cd*A*v[0]**2
    Fd_y = 0.5*ro*Cd*A*v[1]**2
    Fd_z = 0.5*ro*Cd*A*v[2]**2
    
    Fd = [Fd_x, Fd_y, Fd_z]
    
    return Fd