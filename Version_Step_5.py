#!/usr/bin/env python3
import sys
import os
import time
import busio
import threading
import io
from adafruit_servokit import ServoKit
from adafruit_pca9685 import PCA9685
import numpy as np
import configparser
import board
from board import SCL, SDA
from pyPS4Controller.controller import Controller
from spot_micro_kinematics_python.spot_micro_stick_figure import SpotMicroStickFigure
from spot_micro_kinematics_python.utilities import spot_micro_kinematics as smk
import adafruit_mpu6050
from Classes.AngleMeterAlpha import AngleMeterAlpha

configfile_name = "/home/pi/pyro_reloaded/config.ini"

i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)
kit = ServoKit(channels=16)
i2c= busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)
angleMeter = AngleMeterAlpha()
sm = SpotMicroStickFigure(x=0, y=0, z=0, theta=0 ,psi=0, phi=0)
def init():
    print ("init")
    
    # Load the configuration file
    with open(configfile_name) as f:
        my_config = f.read()
        config = configparser.RawConfigParser(allow_no_value=True)
   
        config.read_file(io.StringIO(my_config))
        
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
     
        kit.Conf_speed = float(config.get("GENERAL", "speed")) # unused yet
        
        kit.Conf_maxheight   = float(config.get("GENERAL", "maxheight"))
        kit.Conf_minheight   = float(config.get("GENERAL", "minheight")) 
        
        kit.Coxa_LF_nr   = int(config.get("LF", "coxa-nr")) 
        kit.Coxa_LF_controller_pin   = int(config.get("LF", "coxa-controller-pin"))
        kit.Coxa_LF_maxangle = int(config.get("LF", "coxa-maxangle"))
        kit.Coxa_LF_minangle = int(config.get("LF", "coxa-minangle"))
        kit.Coxa_LF_rotationdirection = int(config.get("LF", "coxa-rotationdirection"))
        kit.Coxa_LF_centercorrection = int(config.get("LF", "coxa-centercorrection"))
        
        kit.Femur_LF_nr   = int(config.get("LF", "femur-nr")) 
        kit.Femur_LF_controller_pin   = int(config.get("LF", "femur-controller-pin"))
        kit.Femur_LF_maxangle = int(config.get("LF", "femur-maxangle"))
        kit.Femur_LF_minangle = int(config.get("LF", "femur-minangle"))
        kit.Femur_LF_rotationdirection = int(config.get("LF", "femur-rotationdirection"))
        kit.Femur_LF_centercorrection = int(config.get("LF", "femur-centercorrection"))
        
        kit.Tibia_LF_nr   = int(config.get("LF", "tibia-nr")) 
        kit.Tibia_LF_controller_pin   = int(config.get("LF", "tibia-controller-pin"))
        kit.Tibia_LF_maxangle = int(config.get("LF", "tibia-maxangle"))
        kit.Tibia_LF_minangle = int(config.get("LF", "tibia-minangle"))
        kit.Tibia_LF_rotationdirection = int(config.get("LF", "tibia-rotationdirection"))
        kit.Tibia_LF_centercorrection = int(config.get("LF", "tibia-centercorrection"))
        # ----------------------
        kit.Coxa_RF_nr   = int(config.get("RF", "coxa-nr")) 
        kit.Coxa_RF_controller_pin   = int(config.get("RF", "coxa-controller-pin"))
        kit.Coxa_RF_maxangle = int(config.get("RF", "coxa-maxangle"))
        kit.Coxa_RF_minangle = int(config.get("RF", "coxa-minangle"))
        kit.Coxa_RF_rotationdirection = int(config.get("RF", "coxa-rotationdirection"))
        kit.Coxa_RF_centercorrection = int(config.get("RF", "coxa-centercorrection"))
        
        kit.Femur_RF_nr   = int(config.get("RF", "femur-nr")) 
        kit.Femur_RF_controller_pin   = int(config.get("RF", "femur-controller-pin"))
        kit.Femur_RF_maxangle = int(config.get("RF", "femur-maxangle"))
        kit.Femur_RF_minangle = int(config.get("RF", "femur-minangle"))
        kit.Femur_RF_rotationdirection = int(config.get("RF", "femur-rotationdirection"))
        kit.Femur_RF_centercorrection = int(config.get("RF", "femur-centercorrection"))
        
        kit.Tibia_RF_nr   = int(config.get("RF", "tibia-nr")) 
        kit.Tibia_RF_controller_pin   = int(config.get("RF", "tibia-controller-pin"))
        kit.Tibia_RF_maxangle = int(config.get("RF", "tibia-maxangle"))
        kit.Tibia_RF_minangle = int(config.get("RF", "tibia-minangle"))
        kit.Tibia_RF_rotationdirection = int(config.get("RF", "tibia-rotationdirection"))
        kit.Tibia_RF_centercorrection = int(config.get("RF", "tibia-centercorrection"))
        # ----------------------
        kit.Coxa_RR_nr   = int(config.get("RR", "coxa-nr")) 
        kit.Coxa_RR_controller_pin   = int(config.get("RR", "coxa-controller-pin"))
        kit.Coxa_RR_maxangle = int(config.get("RR", "coxa-maxangle"))
        kit.Coxa_RR_minangle = int(config.get("RR", "coxa-minangle"))
        kit.Coxa_RR_rotationdirection = int(config.get("RR", "coxa-rotationdirection"))
        kit.Coxa_RR_centercorrection = int(config.get("RR", "coxa-centercorrection"))
        
        kit.Femur_RR_nr   = int(config.get("RR", "femur-nr")) 
        kit.Femur_RR_controller_pin   = int(config.get("RR", "femur-controller-pin"))
        kit.Femur_RR_maxangle = int(config.get("RR", "femur-maxangle"))
        kit.Femur_RR_minangle = int(config.get("RR", "femur-minangle"))
        kit.Femur_RR_rotationdirection = int(config.get("RR", "femur-rotationdirection"))
        kit.Femur_RR_centercorrection = int(config.get("RR", "femur-centercorrection"))
        
        kit.Tibia_RR_nr   = int(config.get("RR", "tibia-nr")) 
        kit.Tibia_RR_controller_pin   = int(config.get("RR", "tibia-controller-pin"))
        kit.Tibia_RR_maxangle = int(config.get("RR", "tibia-maxangle"))
        kit.Tibia_RR_minangle = int(config.get("RR", "tibia-minangle"))
        kit.Tibia_RR_rotationdirection = int(config.get("RR", "tibia-rotationdirection"))
        kit.Tibia_RR_centercorrection = int(config.get("RR", "tibia-centercorrection"))
        # ----------------------
        kit.Coxa_RL_nr   = int(config.get("RL", "coxa-nr")) 
        kit.Coxa_RL_controller_pin   = int(config.get("RL", "coxa-controller-pin"))
        kit.Coxa_RL_maxangle = int(config.get("RL", "coxa-maxangle"))
        kit.Coxa_RL_minangle = int(config.get("RL", "coxa-minangle"))
        kit.Coxa_RL_rotationdirection = int(config.get("RL", "coxa-rotationdirection"))
        kit.Coxa_RL_centercorrection = int(config.get("RL", "coxa-centercorrection"))
        
        kit.Femur_RL_nr   = int(config.get("RL", "femur-nr")) 
        kit.Femur_RL_controller_pin   = int(config.get("RL", "femur-controller-pin"))
        kit.Femur_RL_maxangle = int(config.get("RL", "femur-maxangle"))
        kit.Femur_RL_minangle = int(config.get("RL", "femur-minangle"))
        kit.Femur_RL_rotationdirection = int(config.get("RL", "femur-rotationdirection"))
        kit.Femur_RL_centercorrection = int(config.get("RL", "femur-centercorrection"))
        
        kit.Tibia_RL_nr   = int(config.get("RL", "tibia-nr")) 
        kit.Tibia_RL_controller_pin   = int(config.get("RL", "tibia-controller-pin"))
        kit.Tibia_RL_maxangle = int(config.get("RL", "tibia-maxangle"))
        kit.Tibia_RL_minangle = int(config.get("RL", "tibia-minangle"))
        kit.Tibia_RL_rotationdirection = int(config.get("RL", "tibia-rotationdirection"))
        kit.Tibia_RL_centercorrection = int(config.get("RL", "tibia-centercorrection"))
    kit.status="init"

    kit.servo[kit.Coxa_LF_nr].pin = kit.Coxa_LF_controller_pin
    kit.servo[kit.Coxa_RF_nr].pin = kit.Coxa_RF_controller_pin
    kit.servo[kit.Coxa_RR_nr].pin = kit.Coxa_RR_controller_pin
    kit.servo[kit.Coxa_RL_nr].pin = kit.Coxa_RL_controller_pin
    
    kit.servo[kit.Femur_LF_nr].pin = kit.Femur_LF_controller_pin
    kit.servo[kit.Femur_RF_nr].pin = kit.Femur_RF_controller_pin
    kit.servo[kit.Femur_RR_nr].pin = kit.Femur_RR_controller_pin
    kit.servo[kit.Femur_RL_nr].pin = kit.Femur_RL_controller_pin
    
    kit.servo[kit.Tibia_LF_nr].pin = kit.Tibia_LF_controller_pin
    kit.servo[kit.Tibia_RF_nr].pin = kit.Tibia_RF_controller_pin
    kit.servo[kit.Tibia_RR_nr].pin = kit.Tibia_RR_controller_pin
    kit.servo[kit.Tibia_RL_nr].pin = kit.Tibia_RL_controller_pin
    
    kit.servo[kit.Coxa_LF_nr].maxrange = kit.Coxa_LF_maxangle
    kit.servo[kit.Coxa_LF_nr].minrange = kit.Coxa_LF_minangle
    kit.servo[kit.Coxa_RF_nr].maxrange = kit.Coxa_RF_maxangle
    kit.servo[kit.Coxa_RF_nr].minrange = kit.Coxa_RF_minangle
    kit.servo[kit.Coxa_RR_nr].maxrange = kit.Coxa_RR_maxangle
    kit.servo[kit.Coxa_RR_nr].minrange = kit.Coxa_RR_minangle
    kit.servo[kit.Coxa_RL_nr].maxrange = kit.Coxa_RL_maxangle
    kit.servo[kit.Coxa_RL_nr].minrange = kit.Coxa_RL_minangle
    
    kit.servo[kit.Femur_LF_nr].maxrange = kit.Femur_LF_maxangle
    kit.servo[kit.Femur_LF_nr].minrange = kit.Femur_LF_minangle
    kit.servo[kit.Femur_RF_nr].maxrange = kit.Femur_RF_maxangle
    kit.servo[kit.Femur_RF_nr].minrange = kit.Femur_RF_minangle
    kit.servo[kit.Femur_RR_nr].maxrange = kit.Femur_RR_maxangle
    kit.servo[kit.Femur_RR_nr].minrange = kit.Femur_RR_minangle
    kit.servo[kit.Femur_RL_nr].maxrange = kit.Femur_RL_maxangle
    kit.servo[kit.Femur_RL_nr].minrange = kit.Femur_RL_minangle
    
    kit.servo[kit.Tibia_LF_nr].maxrange = kit.Tibia_LF_maxangle
    kit.servo[kit.Tibia_LF_nr].minrange = kit.Tibia_LF_minangle
    kit.servo[kit.Tibia_RF_nr].maxrange = kit.Tibia_RF_maxangle
    kit.servo[kit.Tibia_RF_nr].minrange = kit.Tibia_RF_minangle
    kit.servo[kit.Tibia_RR_nr].minrange = kit.Tibia_RR_minangle
    kit.servo[kit.Tibia_RR_nr].maxrange = kit.Tibia_RR_maxangle
    kit.servo[kit.Tibia_RL_nr].minrange = kit.Tibia_RL_minangle
    kit.servo[kit.Tibia_RL_nr].maxrange = kit.Tibia_RL_maxangle
        
    kit.servo[kit.Coxa_LF_nr].rotationdirection  = kit.Coxa_LF_rotationdirection  # LF C
    kit.servo[kit.Coxa_RF_nr].rotationdirection  = kit.Coxa_RF_rotationdirection  # RF C
    kit.servo[kit.Coxa_RR_nr].rotationdirection  = kit.Coxa_RR_rotationdirection  # RR C
    kit.servo[kit.Coxa_RL_nr].rotationdirection  = kit.Coxa_RL_rotationdirection  # LR C
        
    kit.servo[kit.Femur_LF_nr].rotationdirection  =  kit.Femur_LF_rotationdirection  # LF F
    kit.servo[kit.Femur_RF_nr].rotationdirection  =  kit.Femur_RF_rotationdirection  # RF F
    kit.servo[kit.Femur_RR_nr].rotationdirection  =  kit.Femur_RR_rotationdirection  # RR F
    kit.servo[kit.Femur_RL_nr].rotationdirection  =  kit.Femur_RL_rotationdirection  # LR F
        
    kit.servo[kit.Tibia_LF_nr].rotationdirection  =  kit.Tibia_LF_rotationdirection  # LF T
    kit.servo[kit.Tibia_RF_nr].rotationdirection  =  kit.Tibia_RF_rotationdirection  # RF T
    kit.servo[kit.Tibia_RR_nr].rotationdirection  =  kit.Tibia_RR_rotationdirection  # RR T
    kit.servo[kit.Tibia_RL_nr].rotationdirection  =  kit.Tibia_RL_rotationdirection  # LR T
        
    kit.servo[kit.Coxa_LF_nr].correction = kit.Coxa_LF_centercorrection
    kit.servo[kit.Coxa_RF_nr].correction = kit.Coxa_RF_centercorrection
    kit.servo[kit.Coxa_RR_nr].correction = kit.Coxa_RR_centercorrection
    kit.servo[kit.Coxa_RL_nr].correction = kit.Coxa_RL_centercorrection
        
    kit.servo[kit.Femur_LF_nr].correction = kit.Femur_LF_centercorrection
    kit.servo[kit.Femur_RF_nr].correction = kit.Femur_RF_centercorrection
    kit.servo[kit.Femur_RR_nr].correction = kit.Femur_RR_centercorrection
    kit.servo[kit.Femur_RL_nr].correction = kit.Femur_RL_centercorrection
        
    kit.servo[kit.Tibia_LF_nr].correction = kit.Tibia_LF_centercorrection
    kit.servo[kit.Tibia_RF_nr].correction = kit.Tibia_RF_centercorrection
    kit.servo[kit.Tibia_RR_nr].correction = kit.Tibia_RR_centercorrection
    kit.servo[kit.Tibia_RL_nr].correction = kit.Tibia_RL_centercorrection
    

    # mpu6050
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
    
    for i in range(0, 12):
        kit.servo[kit.servo[i].pin].set_pulse_width_range(500, 2500) # 500 - 2500 max
    initialize()


def correction_roll(delta_roll):
    #print ( "delta_roll",delta_roll )
    
    #sm.set_body_angles(theta=0.5*delta_roll*math.pi/180 ,psi=0,phi=0.5*delta_pitch*math.pi/180)
    '''
        phi: roll angle in radians of body
        theta: pitch angle in radians of body  
        psi: yaw angle in radians of body
    '''
    fak=2
    ik_move(x=0, h=0, z=0, theta=fak*kit.gyro_pitch*np.pi/180 ,psi=0, phi=fak*kit.gyro_roll*np.pi/180, transition=0, delay=0)
    
    #time.sleep(0.1)
   
def correction_pitch(delta_pitch):
    print ( "delta_pitch",delta_pitch )
    
    #sm.set_body_angles(theta=0.5*delta_roll*math.pi/180 ,psi=0,phi=0.5*delta_pitch*math.pi/180)
    '''
        phi: roll angle in radians of body
        theta: pitch angle in radians of body  
        psi: yaw angle in radians of body
    '''
    fak=2
    ik_move(x=0, h=0, z=0, theta=fak*kit.gyro_pitch*np.pi/180 ,psi=0, phi=fak*kit.gyro_roll*np.pi/180, transition=0, delay=0)
    #time.sleep(0.1)
    
def gyro (arg):
    
    #acc = mpu.acceleration
    angleMeter.measure()
         
    t = kit.gyrothread #threading.current_thread()
    kit.gyro_roll_init = angleMeter.get_int_roll()
    kit.gyro_pitch_init = angleMeter.get_int_pitch()
    
    while getattr(t, "do_run", True):
        #print ("working on %s" % arg)
        
        #print("Roll: ",angleMeter.get_int_roll(), "Pitch: ",angleMeter.get_int_pitch())
        #print(angleMeter.get_kalman_roll(),",", angleMeter.get_complementary_roll(), ",",angleMeter.get_kalman_pitch(),",", angleMeter.get_complementary_pitch(),".")
        
        kit.gyro_roll = angleMeter.get_int_roll()
        kit.gyro_pitch = angleMeter.get_int_pitch()
           
        if kit.gyro_roll != kit.gyro_roll_init or kit.gyro_pitch != kit.gyro_pitch_init:
            if kit.gyro_roll != kit.gyro_roll_tmp:
                kit.gyro_roll_tmp =  int(kit.gyro_roll)
               
                delta_roll =  kit.gyro_roll  - kit.gyro_roll_init
                
                print("Roll: ",kit.gyro_roll)
             
                correction_roll(int(delta_roll))
                
        if kit.gyro_pitch != kit.gyro_pitch_tmp:
                kit.gyro_pitch_tmp = int(kit.gyro_pitch)

                delta_pitch = kit.gyro_pitch - kit.gyro_pitch_init 
                
                print("delta_Pitch: ",delta_pitch)
                correction_pitch(int(-delta_pitch))
    print("Stopping gyro.")

kit.gyrothread = threading.Thread(target=gyro, args=("gyro",))
kit.gyrothread.do_run = False
#kit.gyrothread.start()
    
def switchgyro():
    if (kit.gyrothread.do_run == False):
        
        kit.gyrothread = threading.Thread(target=gyro, args=("gyro",))
        kit.gyrothread.do_run = True
        kit.gyrothread.start()
    else:
        kit.gyrothread.do_run = False


def startthreads(threads,caller):
    for i in range(len(threads)):
        if caller == "initialize":
            time.sleep(0.3)
        
        threads[i].start()
        
    #time.sleep(0.1) #wait until movements are finished
    

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
    
    for i in range(0, 12):
        ##kit.servo[kit.servo[i].pin].set_pulse_width_range(500, 2500) # 500 - 2500 max
        #print ("+++++init array: ",i, " angle: ", kit.init[i], "° "," Servopin: ",kit.servo[i].pin)
      
        pin = kit.servo[i].pin
        ang = 90 + kit.init[i] * kit.servo[i].rotationdirection + kit.servo[i].correction
        #print ("-----init array: ",i, " angle: ", ang, "° "," Servopin: ",kit.servo[i].pin)
        t = threading.Thread(target=move, args=(pin, ang, 0, 0))
        
        threads.append(t)
    startthreads(threads,"initialize")

    
def ik_move(x, h, z, theta ,psi, phi, transition, delay):
    
    #sm = SpotMicroStickFigure(x=0, y=0, z=0, theta=0 ,psi=0, phi)
    sm = SpotMicroStickFigure(x=0 ,y=0.18, z=0, theta=theta, psi=0, phi=phi)
    # Define absolute position for the legs
    y=h
    
    # see SpotMicroStickFigure.py
    l = sm.body_length
    w = sm.body_width
    l1 = sm.hip_length
    l2 = sm.upper_leg_length
    l3 = sm.lower_leg_length
    
    desired_p4_points = np.array([  [-l/2,   y,  w/2 + l1],
                                    [ l/2 ,  y,  w/2 + l1],
                                    [ l/2 ,  y, -w/2 - l1],
                                    [-l/2 ,  y, -w/2 - l1] ])

    sm.set_absolute_foot_coordinates(desired_p4_points)
    leg_angs = sm.get_leg_angles()
    
    leg_angs20= leg_angs[2][0]/np.math.pi*180    #lf upper leg in deg
    leg_angs21= leg_angs[2][1]/np.math.pi*180    #lf leg in deg
    leg_angs22= 90+leg_angs[2][2]/np.math.pi*180 #lf foot in deg
    
    leg_angs10= leg_angs[1][0]/np.math.pi*180    #rf upper leg
    leg_angs11= -leg_angs[1][1]/np.math.pi*180   #rf leg
    leg_angs12= 90-leg_angs[1][2]/np.math.pi*180 #rf foot
    
    leg_angs00= leg_angs[0][0]/np.math.pi*180    #rr upper leg
    leg_angs01= -leg_angs[0][1]/np.math.pi*180   #rr leg
    leg_angs02= 90-leg_angs[0][2]/np.math.pi*180 #rr foot
    
    leg_angs30= -leg_angs[0][0]/np.math.pi*180    #lr upper leg
    leg_angs31= leg_angs[3][1]/np.math.pi*180    #lr leg
    leg_angs32= 90+leg_angs[3][2]/np.math.pi*180 #lr leg
 
    threads = []
    ### hier
    ang20 = 90 + (leg_angs20 + kit.servo[0].correction)*kit.servo[0].rotationdirection
    ang21 = 90 + (leg_angs21 + kit.servo[4].correction)*kit.servo[4].rotationdirection
    ang22 = 90 + (leg_angs22 + kit.servo[8].correction)*kit.servo[8].rotationdirection
    
    ang10 = 90 + (leg_angs10 + kit.servo[1].correction)*kit.servo[1].rotationdirection
    ang11 = 90 + (leg_angs11 - kit.servo[5].correction)*kit.servo[5].rotationdirection
    ang12 = 90 + (leg_angs12 - kit.servo[9].correction)*kit.servo[9].rotationdirection
    
    ang00 = 90 + (leg_angs00 + kit.servo[2].correction)*kit.servo[2].rotationdirection
    ang01 = 90 + (leg_angs01 - kit.servo[6].correction)*kit.servo[6].rotationdirection
    ang02 = 90 + (leg_angs02 - kit.servo[10].correction)*kit.servo[10].rotationdirection
    
    ang30 = 90 + (leg_angs30 - kit.servo[3].correction)*kit.servo[3].rotationdirection
    ang31 = 90 + (leg_angs31 + kit.servo[7].correction)*kit.servo[7].rotationdirection
    ang32 = 90 + (leg_angs32 + kit.servo[11].correction)*kit.servo[11].rotationdirection
    
    print(y,"->LF: leg_angs20, leg_angs21, leg_angs22 ###> ", ang20,"°", ang21,"°", ang22,"°")
    print(y,"->RF: leg_angs10, leg_angs11, leg_angs12 ###> ", ang10,"°", ang11,"°", ang12,"°")
    print(y,"->RR: leg_angs00, leg_angs01, leg_angs02 ###> ", ang00,"°", ang01,"°", ang02,"°")
    print(y,"->RL: leg_angs30, leg_angs31, leg_angs32 ###> ", ang30,"°", ang31,"°", ang32,"°")
     
    #delay=0.03
    pin = kit.servo[0].pin
    t0 = threading.Thread(target=move, args=(pin, ang20, transition, delay))  
    threads.append(t0)
    
    pin = kit.servo[4].pin
    t1 = threading.Thread(target=move, args=(pin, ang21, transition, delay ))
    threads.append(t1)
    
    pin = kit.servo[8].pin
    t2 = threading.Thread(target=move, args=(pin, ang22, transition, delay ))
    threads.append(t2)
    #---
    pin = kit.servo[1].pin
    t3 = threading.Thread(target=move, args=(pin, ang10, transition, delay))  
    threads.append(t3)
    
    pin = kit.servo[5].pin
    t4 = threading.Thread(target=move, args=(pin, ang11, transition, delay ))
    threads.append(t4)
    
    pin = kit.servo[9].pin
    t5 = threading.Thread(target=move, args=(pin, ang12, transition, delay ))
    threads.append(t5)
    #---
    pin = kit.servo[2].pin
    t6 = threading.Thread(target=move, args=(pin, ang00, transition, delay))  
    threads.append(t6)
    
    pin = kit.servo[6].pin
    t7 = threading.Thread(target=move, args=(pin, ang01, transition, delay ))
    threads.append(t7)
    
    pin = kit.servo[10].pin
    t8 = threading.Thread(target=move, args=(pin, ang02, transition, delay ))
    threads.append(t8)
    #---
    pin = kit.servo[3].pin
    t9 = threading.Thread(target=move, args=(pin, ang30, transition, delay))  
    threads.append(t9)
    
    pin = kit.servo[7].pin
    t10 = threading.Thread(target=move, args=(pin, ang31, transition, delay ))
    threads.append(t10)
    
    pin = kit.servo[11].pin
    t11 = threading.Thread(target=move, args=(pin, ang32, transition, delay ))
    threads.append(t11)
    
    startthreads(threads,"ik_move")
    

def rest():
    threads = []
    kit.status = "rest"
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
    kit.status = "sit"
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
    kit.status = "calib"
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
    
    kit.status = "balance"
 
    for i in range(0, 12):
        print ("balance array: ",i, " angle: ", kit.balance[i], "° "," Servopin: ",kit.servo[i].pin)
      
        pin = kit.servo[i].pin
        checkrange(i, kit.balance[i])
        ang = 90 + kit.balance[i] * kit.servo[i].rotationdirection + kit.servo[i].correction
        t = threading.Thread(target=move, args=(pin, ang, 1, 0.04))
        
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
    
    
def exitpyro():
    
    print ("Exit Pyro")
    kit.gyrothread.do_run = False
    time.sleep(1)
    rest()
    time.sleep(1)
    release()
    os._exit(1)


if __name__ == '__main__':
   
    init()
    
        ### run PS4Controller
    class MyController(Controller):
    
        def __init__(self, **kwargs):
            Controller.__init__(self, **kwargs)
     
        def on_x_press(self):
            print("on_x_Press")
            exitpyro()
        
        def on_square_press(self):
            if (kit.status == "rest"): return
            kit.gyrothread.do_run = False
            rest()
            
        def on_circle_press(self):
            if (kit.status == "sit"): return
            sit()
        
        def on_options_press(self):
            if (kit.status == "calib"): return
            kit.gyrothread.do_run = False
            calib()   
            
        def on_triangle_press(self):
            if (kit.status == "balance"): return
            #balance()
            '''
            maxheight = +0.085
            minheight = -0.045
            '''
            x=0
            y=  -0.00 #= max ref + stand higher
            z=0
            theta=-15*np.pi/180 #10*np.pi/180
            psi=0
            phi=0 #10*np.pi/180
            transition=1
            delay=0.02
            ik_move(x, y, z, theta ,psi, phi, transition, delay)
            kit.status = "balance"
            
        def on_share_press(self):
            if (kit.status != "balance"): return
            switchgyro()

    
    if len(sys.argv) == 2:
        arg=sys.argv[1]
        
        if arg=="c":
            calib()
            time.sleep(2)
            exitpyro()
            
        if arg=="r":
            rest()
            time.sleep(2)
            exitpyro()
            
        if arg=="b":
            balance()
            time.sleep(2)
            #exitpyro()
         
        if arg=="i":
              
            x= 0
            y= -0.065 # ref  more minus = stand higher
            y= 0
            z=0
            theta=0*np.pi/180
            psi=0
            phi=0*np.pi/180
            transition=1
            delay=0.03
            ik_move(x, y, z, theta ,psi, phi, transition, delay)
            
            time.sleep(6)
            theta=0*np.pi/180
            psi=0
            phi=15*np.pi/180
            transition=1
            delay=0.03
            ik_move(x, y, z, theta ,psi, phi, transition, delay)
            
            time.sleep(6)
            theta=0*np.pi/180
            psi=0
            phi=-15*np.pi/180
            transition=1
            delay=0.03
            ik_move(x, y, z, theta ,psi, phi, transition, delay)
            
            '''
            y= -0.045
            ik_move(x, y, z, theta ,psi, phi, transition, delay)
            
            time.sleep(3)
            y= +0.085
            ik_move(x, y, z, theta ,psi, phi, transition, delay)
            
            time.sleep(3)
    
            theta=-20*np.pi/180.0 # +-20°
            psi=0
            phi=0
            y= 0
            ik_move(x, y, z, theta ,psi, phi, transition, delay)
            '''
            time.sleep(2)
            exitpyro()

        if arg=="s":
            sit()
            time.sleep(2)
            exitpyro()
        
    else:
            controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
            controller.listen(timeout=60)
  
   
