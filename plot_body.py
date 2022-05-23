#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
from math import pi
from spot_micro_kinematics_python.spot_micro_stick_figure import SpotMicroStickFigure
from spot_micro_kinematics_python.utilities import spot_micro_kinematics as smk

import time

plt.ion() 


d2r = pi/180
r2d = 180/pi

# Attaching 3D axis to the figure
fig = plt.figure()


# Set azimtuth and elevation of plot
# ax.view_init(elev=135,azim=0)

# Instantiate spot micro stick figure obeject
sm = SpotMicroStickFigure(x=0,y=0.18,z=0, theta=00*d2r)

# # Try setting each leg to a desired y position
# x4 = -.055
# y4 = -.18
# z4 = 0

# Define absolute position for the legs
l = sm.body_length
w = sm.body_width
l1 = sm.hip_length
l2 = sm.upper_leg_length
l3 = sm.lower_leg_length

#def plotme():

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




def plotme(theta, psi, phi, arr):
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


theta = 0*d2r  #pitch
psi   = 0*d2r   #yaw
phi   = 0*d2r  #roll
#plotme(theta, psi, phi, arr) 
#time.sleep(0.5)
n = 0
while n < 1:
    print("n: ",n)
    n += 1

    # Try leg to a desired position
    x4 = 0
    y4 = -0.18
    z4 = 00.05
        
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
        
        plotme(theta, psi, phi, arr)
        #time.sleep(0.2)
        print(i)


    for i in range(40, -40, -8):
        theta = 10*d2r  #pitch
        psi   = 0*d2r   #yaw
        phi   = 0*d2r   #roll
        
        arr= np.array([ [rrx+i/1000,   rry,  rrz],
                        [rfx ,  rfy,  rfz],
                        [lfx ,  lfy,  lfz],
                        [lrx ,  lry,  lrz] ])
        
        plotme(theta, psi, phi, arr)
        #time.sleep(0.2)
        print(i)

    for i in range(-40, 0, 8):
        theta = 10*d2r  #pitch
        psi   = 0*d2r   #yaw
        phi   = 0*d2r   #roll
        
        arr= np.array([ [rrx+i/1000,   rry+0.03,  rrz],
                        [rfx ,  rfy,  rfz],
                        [lfx ,  lfy,  lfz],
                        [lrx ,  lry,  lrz] ])
        
        plotme(theta, psi, phi, arr)
        #time.sleep(0.2)
        print(i)

#plt.show()
time.sleep(2)

theta = 10*d2r  #pitch
psi   = 0*d2r   #yaw
phi   = 0*d2r   #roll
        
arr= np.array([ [rrx,   rry,  rrz],
                        [rfx ,  rfy,  rfz],
                        [lfx ,  lfy,  lfz],
                        [lrx ,  lry,  lrz] ])
        
(q1,q2,q3) = smk.ikine(rrx+i/1000,rry+0.03,rrz,l1,l2,l3,legs12 = True)

print('Leg angles: ')
print('q1: %2.1f deg, q2: %2.1f deg, q3: %2.1f deg'%(q1*r2d,q2*r2d,q3*r2d))
        
plotme(theta, psi, phi, arr)

time.sleep(5)

'''    
for i in range(10, 0, -1):
    theta = i*d2r  #pitch
    psi   = 0*d2r   #yaw
    phi   = 0*d2r  #roll
    plotme(theta, psi, phi, arr)
    #time.sleep(0.1)
    print("i ",i)
'''    
'''
theta = 10*d2r  #pitch
psi   = 10*d2r   #yaw
phi   = 10*d2r  #roll
plotme(theta, psi, phi, arr)
time.sleep(0.5)
theta = 10*d2r  #pitch
psi   = -10*d2r   #yaw
phi   = 10*d2r  #roll
plotme(theta, psi, phi, arr)
time.sleep(0.5)
theta = 10*d2r  #pitch
psi   = 0*d2r   #yaw
phi   = 10*d2r  #roll
plotme(theta, psi, phi, arr)
time.sleep(5)
'''
