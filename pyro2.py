#!/usr/bin/env python3
import os
import sys
import threading
from pyPS4Controller.controller import Controller
from Classes.AngleMeterAlpha import AngleMeterAlpha
from numpy import *
#import numpy
import time
from adafruit_servokit import ServoKit
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
import adafruit_mpu6050
import board

i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)
kit = ServoKit(channels=16)
i2c= busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)

angleMeter = AngleMeterAlpha()
angleMeter.measure()
    
    
def gyro (arg):
   
    #acc = mpu.acceleration
         
    #t = threading.current_thread()
    while getattr(t, "do_run", True):
        ##print ("working on %s" % arg)
        
        print("Roll: ",angleMeter.get_int_roll(), "Pitch: ",angleMeter.get_int_pitch())
        #print(angleMeter.get_kalman_roll(),",", angleMeter.get_complementary_roll(), ",",angleMeter.get_kalman_pitch(),",", angleMeter.get_complementary_pitch(),".")
        time.sleep(0.5)
    print("Stopping as you wish.")
    
   
t = threading.Thread(target=gyro, args=("gyro",))

#t.daemon = True

def release():
    for i in range(12):
        kit.servo[i].angle = None
    kit.status = "release"


def rest():

    threads = []
 
    for i in range(12):
        ang = 90 + kit.rest[i] * kit.servo[i].rotationdirection + kit.servo[i].correction
        t = threading.Thread(target=move, args=(i, ang, 1))
        
        threads.append(t)
        
    startthreads(threads)
        
    kit.status = "rest"


def sit():

    threads = []
 
    for i in range(12):
        ang = 90 + kit.sit[i] * kit.servo[i].rotationdirection + kit.servo[i].correction
        t = threading.Thread(target=move, args=(i, ang, 1))
        
        threads.append(t)
        
    startthreads(threads)
        
    kit.status = "sit"
    

def balance():
    
    threads = []
    
    for i in range(12):

        ang = 90 + kit.balance[i] * kit.servo[i].rotationdirection + kit.servo[i].correction
        t = threading.Thread(target=move, args=(i, ang, 1))
        threads.append(t)
        
    startthreads(threads)
    
    
def startthreads(threads):
    threads[0].start()
    threads[6].start()
    threads[3].start()
    threads[9].start()
    
    threads[1].start()
    threads[7].start()
    threads[4].start()
    threads[10].start()
    
    threads[2].start()
    threads[8].start()
    threads[5].start()
    threads[11].start()
    
    kit.status = "balance"
    
def exitpyro():
    t.do_run = False
    #t.join()
    print ("Release Servos")
    rest()
    time.sleep(0.2)
    release()
    os._exit(1)


# from https://github.com/kevinmcaleer/PicoCat
def ease_in_out_sine(current_time, start_value, change_in_value, duration):
    """ sinusoidal easing in/out - accelerating until halfway, then decelerating """
    calc = 1-change_in_value/2 * (cos(pi*current_time/duration) - 1) + start_value 
    #print("current_time: ",current_time, " start_value: ", start_value, " change_in_value: ",change_in_value," duration: ",duration, " calc: ",calc)
    return int(calc)
    
def move(servopin, angle, transition):
    #print (servopin ," ############ ", angle)
    transspeed = 1 
    if transition == 0:
        kit.servo[servopin].angle = angle
    else:      
    	
        sourcepos = kit.servo[servopin].angle
        if sourcepos is None:
            return
        start_time=time.time()
        print (angle, " - ",sourcepos)
        changeinVal = angle-sourcepos
              
        while (time.time()-start_time) <= transspeed:
            angle = ease_in_out_sine(current_time=time.time()-start_time, 
                                  start_value=sourcepos, 
                                  change_in_value=changeinVal,
                                  duration=transspeed)
                
            #print ("-> Angle: ",angle)
            time.sleep(0.05)

            kit.servo[servopin].angle = angle

def init():
    print ("init")
    
    kit.servo[0].rotationdirection = 1  # LF C
    kit.servo[1].rotationdirection = 1  # LF F 
    kit.servo[2].rotationdirection = 1  # LF T 
    kit.servo[3].rotationdirection = 1  # RF C 
    kit.servo[4].rotationdirection = -1 # RF F
    kit.servo[5].rotationdirection = 1  # RF T
    kit.servo[6].rotationdirection = -1 # RR C 
    kit.servo[7].rotationdirection = -1 # RR F
    kit.servo[8].rotationdirection = 1  # RR T
    kit.servo[9].rotationdirection = 1  # RL C
    kit.servo[10].rotationdirection = -1# RL F
    kit.servo[11].rotationdirection = -1# RL T
    
    kit.servo[0].correction = 5
    kit.servo[1].correction = -15
    kit.servo[2].correction = 1
    kit.servo[3].correction = 5
    kit.servo[4].correction = 0
    kit.servo[5].correction = -5
    kit.servo[6].correction = 5
    kit.servo[7].correction = 5
    kit.servo[8].correction = -15
    kit.servo[9].correction = 5
    kit.servo[10].correction = 15
    kit.servo[11].correction = 0
    
    #taken from the amazing https://github.com/PetoiCamp/OpenCat/blob/main/src/InstinctBittle.h

    kit.init    = [  0,  0 ,  0 ,  0 ,  0 ,  0,  0 ,  0 ,  0 , 0 ,  0 ,  0  ] # = calib
    sitbuff     = [  -5, -5,  20,   20,   45,   45,  105,  105,  45,  45, -45, -45  ]
    kit.sit     = [sitbuff[0], sitbuff[4], sitbuff[8],  sitbuff[1],   sitbuff[5],  sitbuff[9], sitbuff[2], sitbuff[6], sitbuff[10], sitbuff[3], sitbuff[7], sitbuff[11]]
    kit.rest    = [  -3, 75 , -55 ,  -3 , 75 , -55,  3 , 75 , -55 , 3 , 75 , -55  ]
    kit.balance = [  0,  30 ,  30 ,  0 ,  30 ,  30,  0 ,  30 ,  30 ,  0  ,  30 ,  30  ]
    
    #threads = []
    for i in range(12):
        kit.servo[i].set_pulse_width_range(600, 2420)
        kit.servo[i].angle = 90 + kit.init[i] * kit.servo[i].rotationdirection + kit.servo[i].correction 
        
        #print (i, (90 + kit.init[i] * kit.servo[i].rotationdirection + kit.servo[i].correction) )
    
    kit.status = "init"
    # set up MPU  
    t.start()


    ### run PS4Controller
    
    

    class MyController(Controller):
    
        def __init__(self, **kwargs):
            Controller.__init__(self, **kwargs)
     
        def on_x_press(self):
            print("on_x_Press")
            exitpyro()
        
        def on_square_press(self):
            if kit.status == "balance" or  kit.status == "init":
                rest()
            else:
                balance()
            
        def on_circle_press(self):
            if kit.status == "balance" or  kit.status == "init":
                sit()
            else:
                balance()
    
    controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller.listen(timeout=60)
    ### PS4Controller running

       
def main():     

    '''
    pca.frequency = 50
    
    #kit.servo[0].angle = 45
    #time.sleep(3)
    '''
    '''
    kit.servo[1].angle = 120
    time.sleep(2)
    kit.servo[1].angle = 70
    time.sleep(2)
    kit.servo[1].angle = 140
    '''
    '''
    time.sleep(7)
    #kit.servo[0].angle = 135
    t.do_run = False
    #t.join()
    #t.join()
    print ("Release Servos")
    release()
    #print ("Released Servos, exiting")
    #sys.exit()
    '''
    
    
'''
kit.servo[2].angle = 0
time.sleep(3)
kit.servo[2].angle = 90
time.sleep(3)
kit.servo[2].angle = 180
time.sleep(3)
kit.servo[2].angle = 90

kit.continuous_servo[2].throttle = 0.3
time.sleep(3)
kit.continuous_servo[2].throttle = -0.3
time.sleep(3)
#kit.servo[2].angle = 0
#time.sleep(3)
kit.continuous_servo[2].throttle = 0.5

'''
if __name__ == '__main__':
    init()
    main()
    


