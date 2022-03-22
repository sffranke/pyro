#!/usr/bin/python3
import sys
import Adafruit_PCA9685
import time
from Classes.TheThreads import MyThread
from math import sin, cos, pi, sqrt
from pyPS4Controller.controller import Controller

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
        self.__servostatus = 0

    def GetStatus(self):
        return self.__servostatus

    def SetStatus(self, i, newStatus):
        self.__servostatus = newStatus
        
    def GetServopin(self):
        return self.__servopin
        
    def ease_in_out_sine(self, current_time, start_value, change_in_value, duration):
        calc = 1-change_in_value/2 * (cos(pi*current_time/duration) - 1) + start_value -1
        #print("current_time: ",current_time, " start_value: ", start_value, " change_in_value: ",change_in_value," duration: ",duration, " calc: ",calc)
        return int(calc)
                
                
    def move(self, servopin, angle, transition):
        if transition == 0:
            print ("angle: ",angle, " pin: ", servopin, self.__servorotation, " transition:", transition)
        
            duty = self.deg2pwm(angle, self.__servorotation)
            #print ("duty: ",duty, " pin: ", servopin, self.__servorotation)
            self.pwm.set_pwm(servopin, 1, duty)
            
            self.SetStatus(servopin, angle)
            #print ("Status: ", s.GetStatus() )
        
        else:
        
            sourcepos = self.GetStatus()
        
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
            
        self.SetStatus(servopin, angle)       
        
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
    Servo(name="Coxa_LF", servopin=0, minangle=-90, maxangle=90, center_pwm=306, transition=0,  transitionspeed=1, servorotation= 1),
    Servo(name="Coxa_LR", servopin=3, minangle=-90, maxangle=90, center_pwm=306, transition=0,  transitionspeed=1, servorotation= -1),
    ]
    
Coxa_LF = servos[0] 
Coxa_RF = servos[1] 

for servo in servos:  
    servo.GetStatus()
    # every Servo has its own thread!


def initialize():
    
    transition=0
    servopin=Coxa_LF.GetServopin()
    angle=0
    
    pyrothred_FEMUR_FRONT_LEFT = MyThread(target=servo.move, args=(servopin, angle, Coxa_LF, transition ))
    pyrothred_FEMUR_FRONT_LEFT.start()
    
    servopin==Coxa_RF.GetServopin()
    angle=0
    pyrothred_FEMUR_FRONT_RIGHT = MyThread(target=servo.move, args=(servopin, angle, Coxa_RF, transition ))
    pyrothred_FEMUR_FRONT_RIGHT.start()
    
    
def test():
    print ("test")
    
    
    
def release():
    for i in range(2):  #16 
        servos[i].pwm.set_pwm(i, 0, 0)

arg = sys.argv[1]
if arg=="i":
    initialize()

if arg=="r":
    release()

if arg=="t":
    initialize()
    test()

def joyleft(self, value):
	
	# 0 - 32767
    para = int( value*45/32767)
   
    angle = servos[0].GetStatus()
    pyrothred_FEMUR_FRONT_LEFT = MyThread(target=servo.move, args=(servos[0].GetServopin(), para, Coxa_LF, 0 ))
    pyrothred_FEMUR_FRONT_LEFT.start()
    angle = servos[1].GetStatus()
    pyrothred_FEMUR_FRONT_RIGHT = MyThread(target=servo.move, args=(servos[1].GetServopin(), para, Coxa_RF, 0 ))
    pyrothred_FEMUR_FRONT_RIGHT.start()
    
def joyright(self, value):
	
	# 0 - 32767
    para = int( value*45/32767)
    
    angle = Coxa_LF.GetStatus()
    pyrothred_FEMUR_FRONT_LEFT = MyThread(target=servo.move, args=(Coxa_LF.GetServopin(), -para, Coxa_LF, 0 ))
    pyrothred_FEMUR_FRONT_LEFT.start()
    angle = Coxa_RF.GetStatus()
    pyrothred_FEMUR_FRONT_RIGHT = MyThread(target=servo.move, args=(Coxa_RF.GetServopin(), -para, Coxa_RF, 0 ))
    pyrothred_FEMUR_FRONT_RIGHT.start()


### run PS4Controller
class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        
    def on_L3_left(self, value):
        joyleft("L3_left", value)

    def on_L3_right(self, value):
        joyleft("L3_right", value)

    '''
    def on_x_press(self):
        print("on_x_Press")
        robot.rest()

    def on_x_release(self):
        print("on_x_release")
        robot.release()

    def on_triangle_press(self):
        print("on_triangle_press")
        
    def on_triangle_release(self):
        print("on_triangle_release")
        Servo.abort_thread = True

    def on_circle_press(self):
        robot.release()
        robot.stand()
        
    def on_square_press(self):
        robot.calibrate()

    def on_share_press(self):
        robot.sit()

    def on_options_press(self):
        robot.hi()

    def on_L3_up(self, value):
        print ("L3up")
        #robot.joyleft("L3up", value)

    def on_L3_down(self, value):
        print ("L3down")
        #robot.joyleft("L3down",value)

    def on_L3_left(self, value):
        print ("Status.robotstatus: ",Status.robotstatus)
        if Status.robotstatus == "L3left" or Status.robotstatus == "L3right":
            robot.joyleft("L3left", value)
        else:
            robot.stand()
            time.sleep(1)
            Status.robotstatus = "L3left"

    def on_L3_right(self, value):
        if Status.robotstatus == "L3left" or Status.robotstatus == "L3right":
            robot.joyleft("L3right", value)
        else:
            robot.stand()
            time.sleep(1)
            Status.robotstatus = "L3right"
                
    
    def on_down_arrow_press(self):
        if Status.robotstatus == "move_up" or Status.robotstatus == "move_down":
            robot.move_down(1)
        else:
            robot.stand()
            time.sleep(1)
            Status.robotstatus = "move_down"

    def on_up_arrow_press(self):
        if Status.robotstatus == "move_up" or Status.robotstatus == "move_down":
            robot.move_up(1)
        else:
            robot.stand()
            time.sleep(1)
            Status.robotstatus = "move_up"
    '''
    
controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
controller.listen(timeout=60)
