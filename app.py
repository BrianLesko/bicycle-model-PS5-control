##################################################################
# Brian Lesko 
# 12/2/2023
# Robotics Studies, Visualize Obstacles in Configuration Space

import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator, AutoMinorLocator
import time
import math
import plot 
my_plot = plot.plotting()
import dualsense # DualSense controller communication
import customize_gui # streamlit GUI modifications
DualSense = dualsense.DualSense
gui = customize_gui.gui()

def transformBodies(dx,dy,da,bodies):
    T_bodies = []
    for body in bodies:
        T = np.array([[np.cos(da), -np.sin(da), dx], [np.sin(da), np.cos(da), dy], [0, 0, 1]])
        body_homogeneous = np.vstack((body.T, np.ones((1, body.shape[0]))))
        body_homogeneous = np.matmul(T, body_homogeneous)
        body = body_homogeneous[:2].T
        T_bodies.append(body)
    return T_bodies

def main():
    # Set up the app UI
    gui.clean_format(wide=True)
    gui.about(text = "This code Shows the bicycle steering model and its Center of rotation, and velocity vector, assumes no wheel slip")
    Title, subTitle, Sidebar, image_spot = st.empty(), st.empty(), st.sidebar.empty(), st.columns([1,5,1])[1].empty()
    title = "<span style='font-size:30px;'>Kinematic Bicycle model:</span>"
    with Title: st.markdown(f" {title} &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; ", unsafe_allow_html=True)
    
    # Setting up the dualsense controller connection
    vendorID, productID = int("0x054C", 16), int("0x0CE6", 16)
    ds = DualSense(vendorID, productID)
    try: ds.connect()
    except Exception as e: st.error("Error occurred while connecting to Dualsense controller. Make sure the controller is wired up and the vendor and product ID's are correctly set in the python script.")
    
    # Initialize loop variables
    fig, ax = my_plot.get_colored_plt('#FFFFFF','#335095','#D6D6D6')
    my_plot.set_c_space_ax(ax)

    # Setup the plotted objects
    fw, = ax.plot([],[],'k',linewidth=1)
    icr, = ax.plot([],[],'ro')
    vel, = ax.plot([],[])

    # car body 
    L = 1.5 # Wheel base
    Lr = .75 # Rear wheel to vehicle slip center
    front_wheel = np.array([[-.5,1],[.5,1],[.5,-1],[-.5,-1],[-.5,1]])*.25
    back_wheel = np.array([[-.5,1],[.5,1],[.5,-1],[-.5,-1],[-.5,1]])*.25 + np.array([0,-L/2])
    body = np.array([[0,L],[0,-L]])*.5
    #ax.plot(front_wheel[:,0],front_wheel[:,1],'k',linewidth=1)
    ax.plot(back_wheel[:,0],back_wheel[:,1],'k',linewidth=1)
    ax.plot(body[:,0],body[:,1],'k',linewidth=2.5,solid_capstyle='round')

    # Control Loop
    prev_theta = 0
    while True:
        ds.receive()
        ds.updateThumbsticks()
        theta = ds.Ltheta
        d_theta =  prev_theta - theta

        # Rotate the front wheel by d_theta
        R = np.array([[np.cos(d_theta), -np.sin(d_theta)], [np.sin(d_theta), np.cos(d_theta)]])
        front_wheel = np.matmul(front_wheel, R)

        # Update the previous value of theta
        prev_theta = theta

        fw.set_data(front_wheel[:,0],front_wheel[:,1]+L/2)

        # Determine the ICR, Center of rotation
        R = L / np.tan(theta)
        # R connects the back wheel center to the ICR
        if np.abs(theta) > 0: 
            icr.set_data(-R,-Lr)

        # beta is the angle of the velocity vector, its origin the the center of mass
        beta = np.arctan(Lr/L * np.tan(theta))
        with subTitle: st.write(beta)
        vel.set_data([0, -np.sin(beta)], [0, 1])
        
        with image_spot: st.pyplot(fig)

main()