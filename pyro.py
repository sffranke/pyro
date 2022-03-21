#!/usr/bin/python3
import sys
import Adafruit_PCA9685
import time
from Classes.TheThreads import MyThread
from math import sin, cos, pi, sqrt

class Status():
    def __init__(self):
        self.__servostati = [0,0,0,0, 0,0,0,0, 0,0,0,0]
        #self.__status = "init"  #balance,stand
    
    def GetStatus(self):
        return self.__servostati

    def SetStatus(self, i, newStatus):
        self.__servostati[i] = newStatus
        


class Servo:
    def __init__(self, name, servopin, minangle, maxangle, center_pwm, transition, transitionspeed, servorotation):
        self.__name = name
        self.__servopin = servopin
        self.__minangle = minangle
        self.__maxangle = maxangle
        self.__center_pwm = center_pwm
        self.__transition = transition
        self.__transitionspeed = transitionspeed
        self.__servorotation = servorotation
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)
        
    
    def ease_in_out_sine(self, current_time, start_value, change_in_value, duration):
        calc = 1-change_in_value/2 * (cos(pi*current_time/duration) - 1) + start_value -1
        print("current_time: ",current_time, " start_value: ", start_value, " change_in_value: ",change_in_value," duration: ",duration, " calc: ",calc)
        return int(calc)
                
                
    def move(self, servopin, angle, transition):
        print ("angle: ",angle, " pin: ", servopin, self.__servorotation, " transition:", transition)
        '''
        duty = self.deg2pwm(angle, self.__servorotation)
        print ("duty: ",duty, " pin: ", servopin, self.__servorotation)
        self.pwm.set_pwm(servopin, 1, duty)
        #print (Status.servostati[servopin],  " <------")
        
        s=Status()
        s.SetStatus(servopin, angle)
        print ("Status: ", s.GetStatus() )
        '''
        
        s=Status()
        sourcepos = s.GetStatus()[servopin]
        
        if(sourcepos == angle):
            return
            
        start_time=time.time()
            
        changeinVal = angle-sourcepos
        while (time.time()-start_time) <= self.__transitionspeed:
            angle = self.ease_in_out_sine(current_time=time.time()-start_time, 
                                          start_value=sourcepos, 
                                          change_in_value=changeinVal,
                                          duration=self.__transitionspeed)
            duty = self.deg2pwm(angle, self.__servorotation)
            self.pwm.set_pwm(servopin, 1, duty)
            
        s.SetStatus(servopin, angle)       
        
        #print ("   Transitionangle: ",angle, " pwm: ",self.deg2pwm(angle, self.__servorotation))
        
    
    def printdata(self):
        print(self.__name, self.__minangle, self.__maxangle, self.__transition, self.__transitionspeed)
        
    def deg2pwm(self, angle, rotation):
        if rotation == 1:
            r= angle*(self.__maxangle-self.__minangle)/180 + self.__center_pwm
        else:
            r=-angle*(self.__maxangle-self.__minangle)/180 + self.__center_pwm
        
        return(int(r))
        
    def angle(self):
        return self.__angle
        
servos = [
    Servo("Coxa_LF", servopin=0, minangle=-90, maxangle=90, center_pwm=306, transition=0,  transitionspeed=1, servorotation= 1),
    Servo("Coxa_LR", servopin=4, minangle=-90, maxangle=90, center_pwm=306, transition=0,  transitionspeed=1, servorotation= -1),
    ]

for servo in servos:  
    servo.printdata()
    # every Servo has its own thread!


def initialize():
    servopin=0
    angle=0
    transition=1
    pyrothred_FEMUR_FRONT_LEFT = MyThread(target=servo.move, args=(servopin, angle, servos[0], transition ))
    pyrothred_FEMUR_FRONT_LEFT.start()
    
    servopin=3
    angle=0
    pyrothred_FEMUR_FRONT_RIGHT = MyThread(target=servo.move, args=(servopin, angle, servos[1], transition ))
    pyrothred_FEMUR_FRONT_RIGHT.start()
    
    time.sleep(1)
    
    servopin=0
    angle=-75
    pyrothred_FEMUR_FRONT_LEFT = MyThread(target=servo.move, args=(servopin, angle, servos[0], transition ))
    pyrothred_FEMUR_FRONT_LEFT.start()
    
    servopin=3
    angle=-75
    pyrothred_FEMUR_FRONT_RIGHT = MyThread(target=servo.move, args=(servopin, angle, servos[1], transition ))
    pyrothred_FEMUR_FRONT_RIGHT.start()
    
    time.sleep(1)

    servopin=0
    angle=75
    pyrothred_FEMUR_FRONT_LEFT = MyThread(target=servo.move, args=(servopin, angle, servos[0], transition ))
    pyrothred_FEMUR_FRONT_LEFT.start()

    servopin=3
    angle=75
    pyrothred_FEMUR_FRONT_RIGHT = MyThread(target=servo.move, args=(servopin, angle, servos[1], transition ))
    pyrothred_FEMUR_FRONT_RIGHT.start()
    

arg = sys.argv[1]
if arg=="i":
    initialize()





