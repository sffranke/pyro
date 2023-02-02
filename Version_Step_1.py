#!/usr/bin/env python3
import os
import time
import busio
import threading
import io
from adafruit_servokit import ServoKit
from adafruit_pca9685 import PCA9685
import board
from numpy import * #cos
from board import SCL, SDA

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
    #threads = []
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
        print ("init array: ",i, " angle: ", kit.init[i], "° "," Servopin: ",kit.servo[i].pin)
      
        pin = kit.servo[i].pin
        ang = 90 + kit.init[i] * kit.servo[i].rotationdirection + kit.servo[i].correction
        t = threading.Thread(target=move, args=(pin, ang, 0, 0))
        
        threads.append(t)

    startthreads(threads,"initialize")

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
    calc = 1-change_in_value/2 * (cos(pi*current_time/duration) - 1) + start_value 
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
    
    time.sleep(3)
    calib()
    time.sleep(3)
    sit()
    time.sleep(3)
    balance()
    time.sleep(3)
    rest()
    time.sleep(3)
    release()
