#!/usr/bin/env python3
import os
import time
import busio
import threading
import io
from adafruit_servokit import ServoKit
from adafruit_pca9685 import PCA9685
import board
import numpy as np
from board import SCL, SDA

from spot_micro_kinematics_python.spot_micro_stick_figure import SpotMicroStickFigure
from spot_micro_kinematics_python.utilities import spot_micro_kinematics as smk

i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)
kit = ServoKit(channels=16)
i2c= busio.I2C(board.SCL, board.SDA)

def init():
    print ("init")

    kit.servo[0].pin = 0
    kit.servo[1].pin = 8
    kit.servo[2].pin = 4
    
    kit.servo[3].pin = 12
    kit.servo[4].pin = 1
    kit.servo[5].pin = 9
    
    kit.servo[6].pin = 5
    kit.servo[7].pin = 13  
    kit.servo[8].pin = 2
    
    kit.servo[9].pin = 10
    kit.servo[10].pin = 6
    kit.servo[11].pin = 14
    
    kit.servo[0].maxrange =  20
    kit.servo[0].minrange = -20
    kit.servo[1].maxrange =  20
    kit.servo[1].minrange = -20
    kit.servo[2].maxrange =  20
    kit.servo[2].minrange = -20
    kit.servo[3].maxrange =  20
    kit.servo[3].minrange = -20
    
    kit.servo[4].maxrange =  80
    kit.servo[4].minrange = -45
    kit.servo[5].maxrange =  80
    kit.servo[5].minrange = -45
    kit.servo[6].maxrange =  80
    kit.servo[6].minrange = -45
    kit.servo[7].maxrange =  80
    kit.servo[7].minrange = -45
    
    kit.servo[8].maxrange =  55
    kit.servo[8].minrange = -55
    kit.servo[9].maxrange =  55
    kit.servo[9].minrange = -55
    kit.servo[10].maxrange = 55
    kit.servo[10].minrange = -55
    kit.servo[11].maxrange =  55
    kit.servo[11].minrange = -55
        
    kit.servo[0].rotationdirection  =  1  # LF C
    kit.servo[1].rotationdirection  = -1  # RF C
    kit.servo[2].rotationdirection  =  1  # RR C
    kit.servo[3].rotationdirection  = -1  # LR C
        
    kit.servo[4].rotationdirection  =  1  # LF F
    kit.servo[5].rotationdirection  = -1  # RF F
    kit.servo[6].rotationdirection  = -1  # RR F
    kit.servo[7].rotationdirection  =  1  # LR F
        
    kit.servo[8].rotationdirection   =  1  # LF T
    kit.servo[9].rotationdirection   = -1  # RF T
    kit.servo[10].rotationdirection  = -1  # RR T
    kit.servo[11].rotationdirection  =  1  # LR T
        
    kit.servo[0].correction = -10
    kit.servo[1].correction = -7
    kit.servo[2].correction = -5
    kit.servo[3].correction = -10
        
    kit.servo[4].correction = 0-45
    kit.servo[5].correction = -4+45
    kit.servo[6].correction = +45
    kit.servo[7].correction = -2-45
        
    kit.servo[8].correction = 0
    kit.servo[9].correction = -5
    kit.servo[10].correction = 10
    kit.servo[11].correction = 0
    
    kit.gyro_roll = 0
    kit.gyro_pitch = 0
    kit.gyro_roll_tmp = 0
    kit.gyro_pitch_tmp = 0
    kit.ismoving = False
    
    #taken from the amazing https://github.com/PetoiCamp/OpenCat/blob/main/src/InstinctBittle.h

    #include Gaits and skills
    import instincts
    from instincts import instincts
    kit.init    = instincts["ini"] 
    kit.calib     = instincts["calib"]
    kit.sit     = instincts["sit"]
    kit.rest    = instincts["rest"]
    kit.balance = instincts["balance"]
    kit.walkF = instincts["walkF"]
    kit.walkL = instincts["walkL"]
    kit.walkR = instincts["walkR"]
    kit.crF = instincts["crF"]
    kit.hi =  instincts["hi"]
    threads = []
    initialize()

def startthreads(threads,caller):
    for i in range(len(threads)):
        if caller == "initialize":
            time.sleep(1)
        threads[i].start()
  
def checkrange(servopin, angle):  
   
    if angle>kit.servo[servopin].maxrange:
        print ("Range exeeded! ",servopin," ", angle," > ",kit.servo[servopin].maxrange)
        release()
        os._exit(1)
    if angle<kit.servo[servopin].minrange:
        print ("Below range! ",servopin," ", angle," > ",kit.servo[servopin].minrange)
        release()
        os._exit(1)
            
    return True
  
def initialize():
    threads = []
    #kit.status = "init"
    for i in range(0, 12):
        kit.servo[kit.servo[i].pin].set_pulse_width_range(500, 2500) # 500 - 2500 max
        print ("+++++init array: ",i, " angle: ", kit.init[i], "° "," Servopin: ",kit.servo[i].pin)
      
        pin = kit.servo[i].pin
        ang = 90 + kit.init[i] * kit.servo[i].rotationdirection + kit.servo[i].correction
        print ("-----init array: ",i, " angle: ", ang, "° "," Servopin: ",kit.servo[i].pin)
        t = threading.Thread(target=move, args=(pin, ang, 0, 0))
        
        threads.append(t)
    startthreads(threads,"initialize")
    
''' 
-------------------------
''' 
def ik_move(x, h, z, theta ,psi, phi):
    
    #sm = SpotMicroStickFigure(x=0, y=0, z=0, theta=0 ,psi=0, phi)
    sm = SpotMicroStickFigure(x=0 ,y=0.18, z=0, theta=theta, psi=0, phi=0)
    # Define absolute position for the legs
    y=h
    l = sm.body_length
    w = sm.body_width
    l1 = sm.hip_length
    l2 = sm.upper_leg_length
    l3 = sm.lower_leg_length
    '''
    self.hip_length = 0.06
    self.upper_leg_length = 0.105
    self.lower_leg_length = 0.14
    self.body_width = 0.19
    self.body_length = 0.245
    '''
    desired_p4_points = np.array([  [-l/2,   y,  w/2 + l1],
                                    [ l/2 ,  y,  w/2 + l1],
                                    [ l/2 ,  y, -w/2 - l1],
                                    [-l/2 ,  y, -w/2 - l1] ])
    
    #print("-l/2,y,w,l1,w/2+l1", -l/2,y,w,l1,w/2+l1)
    
    sm.set_absolute_foot_coordinates(desired_p4_points)
    leg_angs = sm.get_leg_angles()
    
    leg_angs20= leg_angs[2][0]/np.math.pi*180   #lf upper leg in deg
    leg_angs21= leg_angs[2][1]/np.math.pi*180   #lf leg in deg
    leg_angs22= 90+leg_angs[2][2]/np.math.pi*180   #lf foot in deg
    
    leg_angs10= leg_angs[1][0]/np.math.pi*180   #rf upper leg
    leg_angs11= -leg_angs[1][1]/np.math.pi*180   #rf leg
    leg_angs12= 90-leg_angs[1][2]/np.math.pi*180   #rf foot
    
    leg_angs00= leg_angs[0][0]/np.math.pi*180   #rr upper leg
    leg_angs01= -leg_angs[0][1]/np.math.pi*180   #rr leg
    leg_angs02= 90-leg_angs[0][2]/np.math.pi*180   #rr foot
    
    leg_angs30= leg_angs[0][0]/np.math.pi*180   #lr upper leg
    leg_angs31= leg_angs[3][1]/np.math.pi*180   #lr leg
    leg_angs32= 90+leg_angs[3][2]/np.math.pi*180   #lr leg
    print(y, "0LF: leg_angs20, leg_angs21, leg_angs22 ###> ", leg_angs20,"°", leg_angs21,"°", leg_angs22,"°")
    print(y, "0RF: leg_angs10, leg_angs11, leg_angs12 ###> ", leg_angs10,"°", leg_angs11,"°", leg_angs12,"°")
    print(y, "0RR: leg_angs00, leg_angs01, leg_angs02 ###> ", leg_angs00,"°", leg_angs01,"°", leg_angs02,"°")
    print(y, "0RL: leg_angs30, leg_angs31, leg_angs32 ###> ", leg_angs30,"°", leg_angs31,"°", leg_angs32,"°")
    print(" -------------- ")
 
    threads = []
    
    ang20 = 90 + (leg_angs20 + kit.servo[0].correction)*kit.servo[0].rotationdirection
    ang21 = 90 + (leg_angs21 + kit.servo[4].correction)*kit.servo[4].rotationdirection
    ang22 = 90 + (leg_angs22 + kit.servo[8].correction)*kit.servo[8].rotationdirection
    
    ang10 = 90 + (leg_angs10 - kit.servo[1].correction)*kit.servo[1].rotationdirection
    ang11 = 90 + (leg_angs11 - kit.servo[5].correction)*kit.servo[5].rotationdirection
    ang12 = 90 + (leg_angs12 - kit.servo[9].correction)*kit.servo[9].rotationdirection
    
    ang00 = 90 + (leg_angs00 + kit.servo[2].correction)*kit.servo[2].rotationdirection
    ang01 = 90 + (leg_angs01 - kit.servo[6].correction)*kit.servo[6].rotationdirection
    ang02 = 90 + (leg_angs02 - kit.servo[10].correction)*kit.servo[10].rotationdirection
    
    ang30 = 90 + (leg_angs30 - kit.servo[3].correction)*kit.servo[3].rotationdirection
    ang31 = 90 + (leg_angs31 + kit.servo[7].correction)*kit.servo[7].rotationdirection
    ang32 = 90 + (leg_angs32 + kit.servo[11].correction)*kit.servo[11].rotationdirection
    
    print("-###############################-")
    print ("balance: [  0,   0,   0,   0,  40,  40,  40,  40,  20,  20,  20,  20 ]")
    print("-###############################-")
    print(y,"->LF: leg_angs20, leg_angs21, leg_angs22 ###> ", ang20,"°", ang21,"°", ang22,"°")
    print(y,"->RF: leg_angs10, leg_angs11, leg_angs12 ###> ", ang10,"°", ang11,"°", ang12,"°")
    print(y,"->RR: leg_angs00, leg_angs01, leg_angs02 ###> ", ang00,"°", ang01,"°", ang02,"°")
    print(y,"->RL: leg_angs30, leg_angs31, leg_angs32 ###> ", ang30,"°", ang31,"°", ang32,"°")
    
    #ang = 90 + kit.init[i] * kit.servo[i].rotationdirection + kit.servo[i].correction
   
    delay=0.03
    pin = kit.servo[0].pin
    t0 = threading.Thread(target=move, args=(pin, ang20, 1, delay))  
    threads.append(t0)
    
    pin = kit.servo[4].pin
    t1 = threading.Thread(target=move, args=(pin, ang21, 1, delay ))
    threads.append(t1)
    
    pin = kit.servo[8].pin
    t2 = threading.Thread(target=move, args=(pin, ang22, 1, delay ))
    threads.append(t2)
    #---
    pin = kit.servo[1].pin
    t3 = threading.Thread(target=move, args=(pin, ang10, 1, delay))  
    threads.append(t3)
    
    pin = kit.servo[5].pin
    t4 = threading.Thread(target=move, args=(pin, ang11, 1, delay ))
    threads.append(t4)
    
    pin = kit.servo[9].pin
    t5 = threading.Thread(target=move, args=(pin, ang12, 1, delay ))
    threads.append(t5)
    #---
    pin = kit.servo[2].pin
    t6 = threading.Thread(target=move, args=(pin, ang00, 1, delay))  
    threads.append(t6)
    
    pin = kit.servo[6].pin
    t7 = threading.Thread(target=move, args=(pin, ang01, 1, delay ))
    threads.append(t7)
    
    pin = kit.servo[10].pin
    t8 = threading.Thread(target=move, args=(pin, ang02, 1, delay ))
    threads.append(t8)
    #---
    pin = kit.servo[3].pin
    t9 = threading.Thread(target=move, args=(pin, ang30, 1, delay))  
    threads.append(t9)
    
    pin = kit.servo[7].pin
    t10 = threading.Thread(target=move, args=(pin, ang31, 1, delay ))
    threads.append(t10)
    
    pin = kit.servo[11].pin
    t11 = threading.Thread(target=move, args=(pin, ang32, 1, delay ))
    threads.append(t11)
    
    startthreads(threads,"ik_move")
    
    
def test():
    x= 0
    y= -0.065 # ref  more minus = stand higher
    y= 0
    z=0
    theta=0
    psi=0
    phi=0
    ik_move(x, y, z, theta ,psi, phi)
   
    time.sleep(3)
    y= -0.045
    ik_move(x, y, z, theta ,psi, phi)
    
    time.sleep(3)
    y= +0.085
    ik_move(x, y, z, theta ,psi, phi)
    
    time.sleep(3)
    
    theta=-20*np.pi/180.0 # +-20°
    psi=0
    phi=0
    y= 0
    ik_move(x, y, z, theta ,psi, phi)
    

def rest():
    threads = []
    #kit.status = "rest"
    for i in range(0, 12):
        print ("rest array: ",i, " angle: ", kit.rest[i], "° "," Servopin: ",kit.servo[i].pin)
      
        pin = kit.servo[i].pin
        checkrange(i, kit.rest[i])
        ang = 90 + kit.rest[i] * kit.servo[i].rotationdirection + kit.servo[i].correction
        t = threading.Thread(target=move, args=(pin, ang, 1, 0.05))
        
        threads.append(t)
    startthreads(threads, "rest")

def sit():
    threads = []
    #kit.status = "rest"
    for i in range(0, 12):
        print ("rest array: ",i, " angle: ", kit.sit[i], "° "," Servopin: ",kit.servo[i].pin)
      
        pin = kit.servo[i].pin
        checkrange(i, kit.sit[i])
        ang = 90 + kit.sit[i] * kit.servo[i].rotationdirection + kit.servo[i].correction
        t = threading.Thread(target=move, args=(pin, ang, 1, 0.05))
        
        threads.append(t)
    startthreads(threads, "sit")
    
    
def calib():
    threads = []
    #kit.status = "calib"
    for i in range(0, 12):
        print ("calib array: ",i, " angle: ", kit.calib[i], "° "," Servopin: ",kit.servo[i].pin)
      
        pin = kit.servo[i].pin
        checkrange(i, kit.calib[i])
        ang = 90 + kit.calib[i] * kit.servo[i].rotationdirection + kit.servo[i].correction
        t = threading.Thread(target=move, args=(pin, ang, 1, 0.05))
        
        threads.append(t)
    startthreads(threads, "calib")

def balance():
    threads = []
    #kit.status = "balance"
    for i in range(0, 12):
        print ("calib array: ",i, " angle: ", kit.balance[i], "° "," Servopin: ",kit.servo[i].pin)
      
        pin = kit.servo[i].pin
        checkrange(i, kit.balance[i])
        ang = 90 + kit.balance[i] * kit.servo[i].rotationdirection + kit.servo[i].correction
        t = threading.Thread(target=move, args=(pin, ang, 1, 0.05))
        
        threads.append(t)
    startthreads(threads, "balance")
    
def release():
    for i in range(15):
        kit.servo[i].angle = None

# from https://github.com/kevinmcaleer/PicoCat
def ease_in_out_sine(current_time, start_value, change_in_value, duration):
    """ sinusoidal easing in/out - accelerating until halfway, then decelerating """
    calc = 1-change_in_value/2 * (np.cos(np.pi*current_time/duration) - 1) + start_value 
    #print("current_time: ",current_time, " start_value: ", start_value, " change_in_value: ",change_in_value," duration: ",duration, " calc: ",calc)
    return int(calc)
       
def move(servopin, angle, transition, delay):
    
    #print (servopin ," ############ ", angle, transition, delay)
    
    transspeed = 1
    if transition == 0:
        kit.servo[servopin].angle = angle
    else:
       
        sourcepos = kit.servo[servopin].angle

        if sourcepos is None:
            return
        start_time=time.time()

        changeinVal = angle-sourcepos

        while (time.time()-start_time) <= transspeed:
            angle = ease_in_out_sine(current_time=time.time()-start_time,
                                  start_value=sourcepos,
                                  change_in_value=changeinVal,
                                  duration=transspeed)

            #print ("-> Angle: ",angle)
            time.sleep(delay)
            kit.servo[servopin].angle = angle
    


if __name__ == '__main__':
    init()
    '''
    time.sleep(3)
    calib()
    time.sleep(3)
    sit()
    time.sleep(3)
    balance()
    time.sleep(3)
    rest()
    time.sleep(3)
    '''
    time.sleep(3)
    test()
    #time.sleep(3)
    #test2()
    time.sleep(3)
    release()
