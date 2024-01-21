##################################################################
# Brian Lesko 
# 12/2/2023
# Robotics Studies, Visualize Obstacles in Configuration Space

import streamlit as st
import numpy as np
import modern_robotics as mr
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator, AutoMinorLocator
import time
import math

import dualsense # DualSense controller communication
import customize_gui # streamlit GUI modifications
import robot
DualSense = dualsense.DualSense
gui = customize_gui.gui()
my_robot = robot.two2_robot()
Obstacles = robot.TwoD_objects()

def main():
    # Set up the app UI
    gui.clean_format(wide=True)
    gui.about(text = "This code implements the configuration space of a 2R robot as a 2D plot.")
    Title, subTitle, Sidebar, image_spot = st.empty(), st.empty(), st.sidebar.empty(), st.columns([1,5,1])[1].empty()
    
    # Setting up the dualsense controller connection
    vendorID, productID = int("0x054C", 16), int("0x0CE6", 16)
    ds = DualSense(vendorID, productID)
    try: ds.connect()
    except Exception as e: st.error("Error occurred while connecting to Dualsense controller. Make sure the controller is wired up and the vendor and product ID's are correctly set in the python script.")
    
    # Initialize loop variables
    thetas = [0,0]
    fig, ax = my_robot.get_colored_plt('#FFFFFF','#335095','#D6D6D6')
    my_robot.set_c_space_ax(ax)

    # Create a static axis for the obstacles
    x0, y0 = Obstacles.rectangle(x=-3,y=3,w=2,h=2)
    x1, y1 = Obstacles.circle(x=2,y=2,r=1)
    x2, y2 = Obstacles.polygon(x=-2,y=-2,n=5,r=2)
    x3, y3 = Obstacles.polygon(x=4,y=-4,n=3,r=2)
    x = np.concatenate((x0,x1,x2,x3))
    y = np.concatenate((y0,y1,y2,y3))

    start = [0,0]
    ax.plot(start[0],start[1],'ko',markersize=5)
    goal = [5,5]
    ax.plot(goal[0],goal[1],'rx',markersize=5)

    dynamic_content, = ax.plot([],[],'ro',linewidth=1) # Coding note, the comma unpacks the list of line objects that plot returns
    static_content, = ax.plot(x,y, 'ro', linewidth=.1)

    # Control Loop
    while True:
        ds.receive()
        ds.updateThumbsticks()

        # Thumbstick control
        ds.updateThumbsticks()
        if abs(ds.LX) > 4:
            thetas[0] = thetas[0] + (ds.LX)/200
        if abs(ds.LY) > 4:
            thetas[1] = thetas[1] - (ds.LY)/200

        # Determine which joint is selected
        joints_label = "<span style='font-size:30px;'>Joints:</span>"
        j1 = "<span style='font-size:30px;'>J1</span>" if abs(ds.LX) > 30 else "<span style='font-size:20px;'>J1</span>"
        j2 = "<span style='font-size:30px;'>J2</span>" if abs(ds.LY) > 30 else "<span style='font-size:20px;'>J2</span>"
        with Title: st.markdown(f" {joints_label} &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;{j1} &nbsp; | &nbsp; {j2} &nbsp; ", unsafe_allow_html=True)
            
        # make sure th1 and th2 are between -2pi and 2pi 
        thetas[0] = (thetas[0] + 2*np.pi) % (4*np.pi) - 2*np.pi
        thetas[1] = (thetas[1] + 2*np.pi) % (4*np.pi) - 2*np.pi
            
        # Show the C-Space map 
        dynamic_content.set_data(thetas[0], thetas[1])
        fig.canvas.draw_idle()
        with image_spot: st.pyplot(fig)

main()