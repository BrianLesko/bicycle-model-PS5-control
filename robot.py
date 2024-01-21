# Brian Lesko 
# 12/3/2023
# This code supports the animation of jointed robots

import numpy as np 
import modern_robotics as mr
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator, AutoMinorLocator

class CyclicVariable:
    def __init__(self, range_vector):
        self.range_vector = range_vector
        self.index = 0

    def get_value(self):
        return self.range_vector[self.index]

    def increment(self):
        self.index = (self.index + 1) % len(self.range_vector)

    def decrement(self):
        self.index = (self.index - 1) % len(self.range_vector)

class two2_robot:

    def __init__(self, L1=1, L2=1):
        self.L1 = L1
        self.L2 = L2
        self.th1 = 0
        self.th2 = 0
        self.thetas = [self.th1,self.th2]
        self.th = CyclicVariable(self.thetas)

    @staticmethod
    def get_colored_plt(hex = '#FFFFFF', hex2 = '#D6D6D6', hex3 = '#D6D6D6',figsize=(6, 6)):
            # hex1 is the background color   # hex2 is the text color    # hex3 is the secondary color
            fig, ax = plt.subplots(figsize=figsize)
            ax.set_facecolor(hex)  
            fig.set_facecolor(hex) 
            for item in ax.get_xticklabels() + ax.get_yticklabels() + ax.get_xgridlines() + ax.get_ygridlines():
                item.set_color(hex2)
            for item in list(ax.spines.values()):
                item.set_color(hex3)
            ax.tick_params(colors=hex3, axis='both', which='major', labelsize=8)
            for spine in ax.spines.values():
                spine.set_linewidth(2)  # Change '2' to your desired linewidth
            return fig, ax
    
    @staticmethod
    def set_axes(ax, xlim=[-2,4], ylim=[-2,2], xlabel='X', ylabel='Y', aspect='equal'):
        ax.set(xlim=xlim, ylim=ylim, xlabel=xlabel, ylabel=ylabel, aspect=aspect)

    @staticmethod
    def get_twin_ax(ax):
        # Set the visibility of the spines to False
        for spine in ax.spines.values():
            spine.set_visible(False)
        # Remove the ticks
        ax.tick_params(axis='both', which='both', length=0)
        ax.set_yticklabels([])
        ax.set_ylim([-2*np.pi,2*np.pi])
        ax.xaxis.label.set_color('#D6D6D6')
        ax.yaxis.label.set_color('#D6D6D6')
        ax2 = ax.twinx()
        return ax2

    def getT_list(self):
        # The link lengths are defined as L1 and L2, or L = [L1 L2]'
        th = [self.th1, self.th2]
        th = np.reshape(th, (2,))

        # The home configuration of a 2R planar robot
        p = np.array([[self.L1+self.L2], [0], [0]]) # The end effector position in the home configuration
        M02 = np.block([[np.eye(3), p], [0, 0, 0, 1]]) # The end effector frame in the home configuration
        p = np.array([[self.L1], [0], [0]]) # The first joint position in the home configuration
        M01 = np.block([[np.eye(3), p], [0, 0, 0, 1]]) # The first joint frame in the home configuration
        p = np.array([[0], [0], [0]]) # The base frame in the home configuration
        M00 = np.block([[np.eye(3), p], [0, 0, 0, 1]]) # The base frame in the home configuration

        # Screw Axis
        # A screw axis is a line about which a rigid body move and rotate it is defined by a unit vector s and a point q that lies on the line 
        s1 = np.array([[0], [0], [1], [0], [0], [0]]) 
        s2 = np.array([[0], [0], [1], [0], [-self.L1], [0]])
        Slist = np.hstack((s1, s2))

        # Forward Kinematics
        T02 = mr.FKinSpace(M02, Slist, th)
        T01 = mr.FKinSpace(M01, s1, [th[0]])
        T00 = mr.FKinSpace(M00, s1, [th[0]])

        T_list = [T00,T01, T02]
        return T_list
    
    def plot_robot(self, ax, T_list, alpha = 1.0):
        # Plot the links 
        for i in range(len(T_list)):
            if i == 0:
                ax.plot([0, T_list[i][0,3]], [0, T_list[i][1,3]], 'r-',alpha = alpha)
            if i > 0:
                ax.plot([T_list[i-1][0,3], T_list[i][0,3]], [T_list[i-1][1,3], T_list[i][1,3]], 'r-',alpha = alpha)
        # Plot the joints
        ax.plot(0, 0, 0, 'ro',alpha = alpha)
        for i in range(len(T_list)-1):
            # extract the origin of each frame
            T = T_list[i]
            print(T)
            [x, y, z] = T[0:3, 3]
            # plot the origin
            ax.plot(x, y, 'ro',alpha = alpha)

    def get_robot_figure(self, th1, th2, Axes = False):
        self.th1 = th1
        self.th2 = th2
        T_list = self.getT_list()
        fig, ax = self.get_colored_plt('#F6F6F3','#335095','#D6D6D6')
        self.set_axes(ax)
        self.plot_robot(ax, T_list)
        if Axes == True: self.draw_axes(ax, T_list, length=1, alpha=.333)
        return fig
    
    def set_c_space_ax(self, ax): 
        pi = np.pi
        self.set_axes(ax,xlim=[-2*pi,2*pi], ylim=[-2*pi,2*pi])
        ax.set_xlabel('$\Theta_1$', color='#D6D6D6')  # Set color of x label
        ax.set_ylabel('$\Theta_2$', color='#D6D6D6')  # Set color of y label
        ax.set_xticks([-2*pi, 0, 2*pi])
        ax.set_yticks([-2*pi, 0, 2*pi])
        ax.set_xticklabels(['-2π', '0', '2π'], color = '#D6D6D6')
        ax.set_yticklabels(['-2π', '0', '2π'], color = '#D6D6D6')
        ax.tick_params(width=0)  # Set the thickness of the tickmarks

    def draw_axes(self, ax, T_list, length=4, alpha=1.0):
        colors = ['#ff0000', '#00ff00', '#0000ff']
        origin = np.array([0, 0, 0, 1])  # Homogeneous coordinate for the origin

        for T in T_list:
            for i, color in enumerate(colors):
                end_point = np.array([0, 0, 0, 1])
                end_point[i] = length
                transformed_origin, transformed_end = np.dot(T, origin), np.dot(T, end_point)
                ax.quiver(*transformed_origin[:2], *(transformed_end - transformed_origin)[:2], color=color, linewidth=2, alpha=alpha)
                ax.scatter(*transformed_end[:2], s=1, color=color, alpha=.01)

class TwoD_objects():
    
    def circle(self, x=1, y=1, r=1, color = 'r'):
        theta = np.linspace(0, 2*np.pi, 100)
        x = r*np.cos(theta) + x
        y = r*np.sin(theta) + y
        #ax.plot(x, y, color = color)
        return x, y

    def rectangle(self, x=1, y=1, w=1, h=1, color = 'r'):
        x = [x,x+w,x+w,x,x]
        y = [y,y,y+h,y+h,y]
        #ax.plot(x, y, color = color)
        return x, y

    def ellipse(self, x=1, y=1, a=1, b=1, color = 'r'):
        theta = np.linspace(0, 2*np.pi, 100)
        x = a*np.cos(theta) + x
        y = b*np.sin(theta) + y
        #ax.plot(x, y, color = color)
        return x, y

    def polygon(self, x=1, y=1, n=3, r=1, color = 'r'):
        theta = np.linspace(0, 2*np.pi, n+1)
        x = r*np.cos(theta) + x
        y = r*np.sin(theta) + y
        #ax.plot(x, y, color = color)
        return x, y
    
    
