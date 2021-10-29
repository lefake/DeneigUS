# -*- coding: utf-8 -*-
"""
Created on Sun Jun 21 11:08:26 2020

@author: Pierre-Olivier Lagacé
"""
import matplotlib.pyplot as plt


class GraphGeneration:
    """
        This class contains all functions to plot graph to simulate trajectory,
        forces, speed on the snow during simulation.
    """
    
    def __init__(self):
        print("GraphGeneration")
               
    def update_trajectory(self, trajectory):
        self.trajectory = trajectory
        
    def update_externals_forces(self, externals_forces):
        self.externals_forces = externals_forces

    def update_snow_speed(self, snow_speed):
        self.snow_speed = snow_speed
        
    def update_snow_accel(self, snow_accel):
        self.snow_accel = snow_accel
    
    def plot_3dtrajectory(self):
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.set_title('3dtrajectory');
        ax.set_xlabel('x')
        ax.set_xlim((-4,4))
        ax.set_ylabel('y')
        ax.set_ylim((-4,4))
        ax.set_zlabel('z')
        ax.plot3D(self.trajectory[0],self.trajectory[1],self.trajectory[2], 'gray')
        plt.show()
        
    def plot_externals_forces(self):
        fig, ax = plt.subplots(3, 1, constrained_layout=True)
        fig.suptitle('Externals forces on the snow', fontsize=16)
        
        ax[0].set_title('in x');
        ax[0].set_ylabel('forces')
        ax[0].set_xlabel('distance')
        ax[0].plot(self.externals_forces[0])
        
        ax[1].set_title('in y');
        ax[1].set_ylabel('forces')
        ax[1].set_xlabel('distance')
        ax[1].plot(self.externals_forces[1])
        
        ax[2].set_title('in z');
        ax[2].set_ylabel('forces')
        ax[2].set_xlabel('distance')
        ax[2].plot(self.externals_forces[2])
        

        plt.show()
        
    def plot_snow_speed(self):
        fig, ax = plt.subplots(3, 1, constrained_layout=True)
        fig.suptitle('Snow speed', fontsize=16)
        
        ax[0].set_title('in x');
        ax[0].set_ylabel('speed')
        ax[0].set_xlabel('distance')
        ax[0].plot(self.snow_speed[0])
        
        ax[1].set_title('in y');
        ax[1].set_ylabel('speed')
        ax[1].set_xlabel('distance')
        ax[1].plot(self.snow_speed[1])
        
        ax[2].set_title('in z');
        ax[2].set_ylabel('speed')
        ax[2].set_xlabel('distance')
        ax[2].plot(self.snow_speed[2])
        
        plt.show()
        
    def plot_snow_accel(self):
        fig, ax = plt.subplots(3, 1, constrained_layout=True)
        fig.suptitle('Snow acceleration', fontsize=16)
        
        ax[0].set_title('in x');
        ax[0].set_ylabel('acceleration')
        ax[0].set_xlabel('distance')
        ax[0].plot(self.snow_accel[0])
        
        ax[1].set_title('in y');
        ax[1].set_ylabel('acceleration')
        ax[1].set_xlabel('distance')
        ax[1].plot(self.snow_accel[1])
        
        ax[2].set_title('in z');
        ax[2].set_ylabel('acceleration')
        ax[2].set_xlabel('distance')
        ax[2].plot(self.snow_accel[2])
        
        plt.show()