# -*- coding: utf-8 -*-
"""
Created on Sun Jun 21 11:07:03 2020

@author: Utilisateur
"""

from GraphGeneration import GraphGeneration
import numpy as np

from dir_traj import dir_traj


    
if __name__ == '__main__':
    graph_generator = GraphGeneration()
    graph_generator.plot_externals_forces()
    graph_generator.plot_snow_speed()