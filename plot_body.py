#!/usr/bin/env python3
import io
import os
import random
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import configparser
from math import pi
from spot_micro_kinematics_python.spot_micro_stick_figure import SpotMicroStickFigure
from spot_micro_kinematics_python.utilities import spot_micro_kinematics as smk

from pyPS4Controller.controller import Controller
from Classes.TheThreads import MyThread

import time
from threading import Timer

class RepeatTimer(Timer):
    def run(self):
        while not self.finished.wait(self.interval):
            self.function(*self.args, **self.kwargs)


configfile_name = "./config.ini"

# Load the configuration file
with open(configfile_name) as f:
    sample_config = f.read()
    config = configparser.RawConfigParser(allow_no_value=True)
    config.readfp(io.StringIO(sample_config))
    #parser.read_file(io.StringIO(sample_config))
    
    '''
    # List all contents
    print("List all contents")
    for section in config.sections():
        print("Section: %s" % section)
        for options in config.options(section):
            print("x %s:::%s:::%s" % (options,
                                      config.get(section, options),
                                      str(type(options))))
    '''

    Coxa_LF_center_pwm= int(config.get("LF", "Coxa-center")) +int(config.get("LF", "Coxa-center-dif"))
    Coxa_LF_length = float(config.get("LF", "Coxa-length"))
    Coxa_LF_maxangle = int(config.get("LF", "Coxa-maxangle"))
    Coxa_LF_minangle = int(config.get("LF", "Coxa-minangle"))
    
    Femur_LF_center_pwm=int(config.get("LF", "Femur-center"))+int(config.get("LF", "Femur-center-dif"))
    Femur_LF_length = float(config.get("LF", "Femur-length"))
    Femur_LF_maxangle = int(config.get("LF", "Femur-maxangle"))
    Femur_LF_minangle = int(config.get("LF", "Femur-minangle"))
    
    Tibia_LF_center_pwm= int(config.get("LF", "Tibia-center"))+int(config.get("LF", "Tibia-center-dif"))
    Tibia_LF_length = float(config.get("LF", "Tibia-length"))
    Tibia_LF_maxangle = int(config.get("LF", "Tibia-maxangle"))
    Tibia_LF_minangle = int(config.get("LF", "Tibia-minangle"))
    
    Coxa_RF_center_pwm= int(config.get("RF", "Coxa-center")) +int(config.get("RF", "Coxa-center-dif"))
    Coxa_RF_length = float(config.get("RF", "Coxa-length"))
    Coxa_RF_maxangle = int(config.get("RF", "Coxa-maxangle"))
    Coxa_RF_minangle = int(config.get("RF", "Coxa-minangle"))

    ''' later... set in SpotMicroStickFigure
    Conf_speed = float(config.get("GENERAL", "speed"))
    Conf_length = float(config.get("GENERAL", "length"))
    Conf_width = float(config.get("GENERAL", "width"))
    '''

d2r = pi/180
r2d = 180/pi

# Attaching 3D axis to the figure
fig = plt.figure()

# Set azimtuth and elevation of plot
# ax.view_init(elev=135,azim=0)

# Instantiate spot micro stick figure obeject
#sm = SpotMicroStickFigure(x=0,y=0.18,z=0, theta=00*d2r)
Conf_maxheight = float(config.get("GENERAL", "maxheight"))
sm = SpotMicroStickFigure(x=0,y=Conf_maxheight,z=0, theta=00*d2r)

# Define absolute position for the legs
'''
        self.hip_length = 0.03
        self.upper_leg_length = 0.042
        self.lower_leg_length = 0.055
        self.body_width = 0.08
        self.body_length = 0.118
'''
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
                
theta_tmp= 999
psi_tmp= psi
phi_tmp= phi

arr_tmp= arr.copy()

ax = p3.Axes3D(fig)

ax.set_xlabel('X')
ax.set_ylabel('Z')
ax.set_zlabel('Y')
'''
ax.set_xlim3d([-0.2, 0.2])
ax.set_zlim3d([0, 0.4])
ax.set_ylim3d([0.2,-0.2])
'''
def haschangedparams():
    if (theta_tmp !=  theta):
        return True
    if (psi_tmp !=  psi):
        return True
    if (phi_tmp !=  phi):
        return True
    if((arr_tmp!=arr).all()):
        return True
    
    return False

def plotme(msg):
    global theta, psi, phi, ax, arr
    global theta_tmp, psi_tmp, phi_tmp, arr_tmp
    print ("==========================")
    print ("in plotme:", msg)
    print ("theta: ", theta,"psi: ", psi, "phi: ",phi)
    print ("==========================")
    
    if ( haschangedparams() == False ):
        print (haschangedparams(), "nothing changed()")
        return
        
    theta_tmp= theta
    psi_tmp= psi
    phi_tmp= phi
    arr_tmp= arr.copy
    
    
    #theta = random.randint(0,20)*d2r
    ax.cla()
    ax.set_xlim3d([-0.1, 0.1])
    ax.set_zlim3d([0, 0.1])
    ax.set_ylim3d([0.1,-0.1])
    
    plt.ion() 
    plt.show()
    
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

    plt.pause(0.0000001)     
    

def reset():

    global arr
    global rrx, rry, rrz
    global rfx, rfy, rfz
    global lfx, lfy, lfz
    global lrx, lry, lrz
    
    global l, l1, w
    
    global theta
    global psi
    global phi
    
    rrx = -l/2
    rfx =  l/2
    lfx =  l/2
    lrx = -l/2

    rry = 0
    rfy = 0
    lfy = 0
    lry = 0

    rrz =  w/2 + l1
    rfz =  w/2 + l1
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
  
  
def walkthread(self, servopin, angle, transition):
    print (servopin, angle, transition)

def walk():

    
    print ("in walk")
    global arr, theta, psi, phi
    n = 0
    while n < 1:
        print("n: ",n)
        n += 1
        '''
        # Try leg to a desired position
        x4 = 0
        y4 = -0.18
        z4 = 00.05
        '''
        for i in range(0, 40, 8):
            theta = 10*d2r  #pitch
            psi   = 0*d2r   #yaw
            phi   = 0*d2r   #roll

            arr= np.array([ [rrx+i/1000,   rry+0.03,  rrz],
                            [rfx ,  rfy,  rfz],
                            [lfx ,  lfy,  lfz],
                            [lrx ,  lry,  lrz] ])

            (q1,q2,q3) = smk.ikine(rrx+i/1000,rry+0.03,rrz,l1,l2,l3,legs12 = True)

            print('Leg angles')
            print('q1: %2.1f deg, q2: %2.1f deg, q3: %2.1f deg'%(q1*r2d,q2*r2d,q3*r2d))

            pyrothred_COXA_REAR_RIGHT = MyThread(target=walkthread, args=(0, 0 , "COXA_REAR_RIGHT", 0 ))
            pyrothred_COXA_REAR_RIGHT.start() 
            plotme("")
            

        for i in range(40, -40, -8):
            theta = 10*d2r  #pitch
            psi   = 0*d2r   #yaw
            phi   = 0*d2r   #roll

            arr= np.array([ [rrx+i/1000,   rry,  rrz],
                            [rfx ,  rfy,  rfz],
                            [lfx ,  lfy,  lfz],
                            [lrx ,  lry,  lrz] ])

            plotme("")

        for i in range(-40, 0, 8):
            theta = 10*d2r  #pitch
            psi   = 0*d2r   #yaw
            phi   = 0*d2r   #roll

            arr= np.array([ [rrx+i/1000,   rry+0.03,  rrz],
                            [rfx ,  rfy,  rfz],
                            [lfx ,  lfy,  lfz],
                            [lrx ,  lry,  lrz] ])

            plotme("")      
                
#rt = RepeatedTimer(3, plotme, "hey")

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
    phi_max=10
    # 0 - 32767 
    print(value)
    para = int( 100*value/32767)  # in %
    phi=(para/100)*phi_max*d2r

def leftjoyright(self, value):
    global phi
    phi_max=10
    # 0 - 32767
    para = int( 100*value/32767)  # in %
    phi=(para/100)*phi_max*d2r
    print("+++++++++++++ ",phi*r2d)

def leftjoyup(self, value):
    global theta
    theta_max=20
    # 0 - 32767
    para = int( 100*value/32767)  # in %
    theta=(para/100)*theta_max*d2r
    #plotme("") 

def leftjoydown(self, value):
    global theta
    theta_max=20
    # 0 - 32767
    para = int( 100*value/32767)  # in %
    theta=(para/100)*theta_max*d2r

def rightjoyleft(self, value):
    global psi
    psi_max=20
    # 0 - 32767
    para = int( 100*value/32767)  # in %
    psi=(para/100)*psi_max*d2r

def rightjoyright(self, value):
    global psi
    psi_max=20
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

#plotme("hi")
'''
for value in range(0,32767,1000):
    ##time.sleep(0.1)
    print ("value: ",value)
    leftjoyup("L3_up", value)
    plotme("ho")
'''  
reset()
#plotme("")
time.sleep(1)

#### walk()


reset()  
plotme("rr")

timer = RepeatTimer(0.3, plotme, args=("",))
timer.start()
#time.sleep(5)
#timer.cancel()


#time.sleep(3)
#time.sleep(1000)
controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
controller.listen(timeout=60)
