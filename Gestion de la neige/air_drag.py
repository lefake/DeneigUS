# -*- coding: utf-8 -*-
"""
Created on Wed Jun 24 15:17:09 2020

@author: Pierre-Olivier Lagacé
"""

import numpy as np

def air_drag(v_snow, v_air):
    """
    Fd is the drag force
    p is the dentity of the fluid
    v_snow is the speed of the object in each direction [x,y,z]
    v_air  is the speed of the fluid (air) in each direction [x,y,z]
    v is the speed of the object relatively to the fluid (air)
    A is the cross sectional area of the snow 
    Cd is the drag coefficient - a dimensionless number (~0.5)
    D is the characteristic diameter
    
    
    Returns Fd
    -------
    Fd : TYPE
        DESCRIPTION.

    """
    
    # Snow attributes
    A = 0.1 #m^2
    p = 0.1*10**(2) #kg/m^3
    Cd = 0.5
    
    
    v = np.subtract(v_snow,v_air)
    
    Fd_x = 0.5*p*Cd*A*v[0]**2
    Fd_y = 0.5*p*Cd*A*v[1]**2
    Fd_z = 0.5*p*Cd*A*v[2]**2
    
    Fd = [Fd_x, Fd_y, Fd_z]
    
    return Fd