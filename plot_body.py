#!/usr/bin/env python3
import random
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
from math import pi
from spot_micro_kinematics_python.spot_micro_stick_figure import SpotMicroStickFigure
from spot_micro_kinematics_python.utilities import spot_micro_kinematics as smk

from pyPS4Controller.controller import Controller

import time

from threading import Timer

class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False

plt.ion() 


d2r = pi/180
r2d = 180/pi

# Attaching 3D axis to the figure
fig = plt.figure()


# Set azimtuth and elevation of plot
# ax.view_init(elev=135,azim=0)

# Instantiate spot micro stick figure obeject
sm = SpotMicroStickFigure(x=0,y=0.18,z=0, theta=00*d2r)

# Define absolute position for the legs
l = sm.body_length
w = sm.body_width
l1 = sm.hip_length
l2 = sm.upper_leg_length
l3 = sm.lower_leg_length

rrx = -l/2
rfx =  l/2
lfx =  l/2
lrx = -l/2

rry = 0
rfy = 0
lfy = 0
lry = 0

rrz = w/2 + l1
rfz = w/2 + l1
lfz = -w/2 - l1
lrz = -w/2 - l1

theta= 10*d2r
psi= 0*d2r
phi= 0*d2r

arr= np.array([ [rrx,   rry,  rrz],
                [rfx ,  rfy,  rfz],
                [lfx ,  lfy,  lfz],
                [lrx ,  lry,  lrz] ])

def plotme(msg):
    global theta, psi, phi
    print ("==========================")
    print ("in plotme:", msg)
    print ("theta: ", theta,"psi: ", psi, "phi: ",phi)
    print ("==========================")
    ax = p3.Axes3D(fig)
    ax.cla()

    ax.set_xlabel('X')
    ax.set_ylabel('Z')
    ax.set_zlabel('Y')

    ax.set_xlim3d([-0.2, 0.2])
    ax.set_zlim3d([0, 0.4])
    ax.set_ylim3d([0.2,-0.2])
    desired_p4_points = arr

    sm.set_absolute_foot_coordinates(desired_p4_points)

    # Set a pitch angle
    sm.set_body_angles(theta=theta,psi=psi,phi=phi)

    # Get leg coordinates
    coords = sm.get_leg_coordinates()

    # Initialize empty list top hold line objects
    lines = []

    # Construct the body of 4 lines from the first point of each leg (the four corners of the body)
    for i in range(4):
        # For last leg, connect back to first leg point
        if i == 3:
            ind = -1
        else:
            ind = i

        # Due to mplot3d rotation and view limitations, swap y and z to make the stick figure
        # appear oriented better
        x_vals = [coords[ind][0][0], coords[ind+1][0][0]]
        y_vals = [coords[ind][0][1], coords[ind+1][0][1]]
        z_vals = [coords[ind][0][2], coords[ind+1][0][2]]
        lines.append(ax.plot(x_vals,z_vals,y_vals,color='k')[0])


    # Plot color order for leg links: (hip, upper leg, lower leg)
    plt_colors = ['r','c','b']
    for leg in coords:
        for i in range(3):
            
            # Due to mplot3d rotation and view limitations, swap y and z to make the stick figure
            # appear oriented better
            x_vals = [leg[i][0], leg[i+1][0]]
            y_vals = [leg[i][1], leg[i+1][1]]
            z_vals = [leg[i][2], leg[i+1][2]]
            lines.append(ax.plot(x_vals,z_vals,y_vals,color=plt_colors[i])[0])
    #plt.ioff()
    
    plt.show()
    plt.pause(0.0001) 


def reset():
    global arr
    global rrx, rry, rrz
    global rfx, rfy, rfz
    global lfx, lfy, lfz
    global lrx, lry, lrz
    
    rrx = -l/2
    rfx =  l/2
    lfx =  l/2
    lrx = -l/2

    rry = 0
    rfy = 0
    lfy = 0
    lry = 0

    rrz = w/2 + l1
    rfz = w/2 + l1
    lfz = -w/2 - l1
    lrz = -w/2 - l1

    theta= 10*d2r
    psi= 0*d2r
    phi= 0*d2r

    arr= np.array([ [rrx,   rry,  rrz],
		     [rfx ,  rfy,  rfz],
		     [lfx ,  lfy,  lfz],
		     [lrx ,  lry,  lrz] ])


def stand():
    # Stand
                 
    # Get leg angles
    leg_angs = sm.get_leg_angles()
    print('Leg angles: ', leg_angs[0][0]*r2d, leg_angs[0][1]*r2d, leg_angs[0][2]*r2d)
    print('Leg angles: ', leg_angs[1][0]*r2d, leg_angs[1][1]*r2d, leg_angs[1][2]*r2d)
    print('Leg angles: ', leg_angs[2][0]*r2d, leg_angs[2][1]*r2d, leg_angs[2][2]*r2d)
    print('Leg angles: ', leg_angs[3][0]*r2d, leg_angs[3][1]*r2d, leg_angs[3][2]*r2d)
    #print('q1: %2.1f deg, q2: %2.1f deg, q3: %2.1f deg'%(q1*r2d,q2*r2d,q3*r2d))
        
                
rt = RepeatedTimer(3, plotme, "hey")

def left_arrow_press(self):
    global arr
    global rrx, rry, rrz
    global rfx, rfy, rfz
    global lfx, lfy, lfz
    global lrx, lry, lrz
    
    zmax = 0.02 #meter
    delta = 0.004
    rry = rry-delta
    rfy = rfy-delta
    lfy = lfy+delta
    lry = lry+delta

    arr= np.array([ [rrx,   rry,  rrz],
                    [rfx ,  rfy,  rfz],
                    [lfx ,  lfy,  lfz],
                    [lrx ,  lry,  lrz] ])

def right_arrow_press(self):
    global arr
    global rrx, rry, rrz
    global rfx, rfy, rfz
    global lfx, lfy, lfz
    global lrx, lry, lrz
    
    zmax = 0.02 #meter
    delta = 0.004
    rry = rry+delta
    rfy = rfy+delta
    lfy = lfy-delta
    lry = lry-delta

    arr= np.array([ [rrx,   rry,  rrz],
                    [rfx ,  rfy,  rfz],
                    [lfx ,  lfy,  lfz],
                    [lrx ,  lry,  lrz] ])

def up_arrow_press(self):
    global arr
    global rrx, rry, rrz
    global rfx, rfy, rfz
    global lfx, lfy, lfz
    global lrx, lry, lrz
    
    zmax = 0.02 #meter
    delta = 0.004
    rry = rry-delta
    rfy = rfy-delta
    lfy = lfy-delta
    lry = lry-delta

    arr= np.array([ [rrx,   rry,  rrz],
                    [rfx ,  rfy,  rfz],
                    [lfx ,  lfy,  lfz],
                    [lrx ,  lry,  lrz] ])

def down_arrow_press(self):
    global arr
    global rrx, rry, rrz
    global rfx, rfy, rfz
    global lfx, lfy, lfz
    global lrx, lry, lrz
    
    zmax = 0.02 #meter
    delta = -0.004
    rry = rry-delta
    rfy = rfy-delta
    lfy = lfy-delta
    lry = lry-delta

    arr= np.array([ [rrx,   rry,  rrz],
                    [rfx ,  rfy,  rfz],
                    [lfx ,  lfy,  lfz],
                    [lrx ,  lry,  lrz] ])
                    
def leftjoyleft(self, value):
    global phi
    phi_max=20
    phi_min=0
    # 0 - 32767
    para = int( 100*value/32767)  # in %
    phi=(para/100)*phi_max*d2r

def leftjoyright(self, value):
    global phi
    phi_max=20
    phi_min=0
    # 0 - 32767
    para = int( 100*value/32767)  # in %
    phi=(para/100)*phi_max*d2r

def leftjoyup(self, value):
    global theta
    theta_max=20
    theta_min=0
    # 0 - 32767
    para = int( 100*value/32767)  # in %
    theta=(para/100)*theta_max*d2r

def leftjoydown(self, value):
    global theta
    theta_max=20
    theta_min=0
    # 0 - 32767
    para = int( 100*value/32767)  # in %
    theta=(para/100)*theta_max*d2r

def rightjoyleft(self, value):
    global psi
    psi_max=20
    psi_min=0
    # 0 - 32767
    para = int( 100*value/32767)  # in %
    psi=(para/100)*psi_max*d2r

def rightjoyright(self, value):
    global psi
    psi_max=20
    psi_min=0
    # 0 - 32767
    para = int( 100*value/32767)  # in %
    psi=(para/100)*psi_max*d2r

### run PS4Controller   
class MyController(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
    
    def on_share_press(self):
    	reset()
    
    def on_left_arrow_press(self):
        left_arrow_press("left_arrow_press")
        
    def on_right_arrow_press(self):
        right_arrow_press("right_arrow_press")
    
    def on_up_arrow_press(self):
        up_arrow_press("up_arrow_press")
        
    def on_down_arrow_press(self):
        down_arrow_press("down_arrow_press")
        
    def on_L3_left(self, value):
        leftjoyleft("L3_left", value)
        
    def on_L3_right(self, value):
        leftjoyright("L3_right", value)

    def on_L3_up(self, value):
        leftjoyup("L3_up", value)
        
    def on_L3_down(self, value):
        leftjoydown("L3_down", value)

    def on_R3_left(self, value):
        rightjoyleft("R3_left", value)
        
    def on_R3_right(self, value):
        rightjoyright("RL3_right", value)
    '''
    def on_x_press(self):
        print("on_x_Press")
        test()
        
    def on_triangle_press(self):
        print("on_triangle_press")
        initialize()
    '''

controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
controller.listen(timeout=60)
