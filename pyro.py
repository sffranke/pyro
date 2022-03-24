#!/usr/bin/python3
import sys
import Adafruit_PCA9685
import time
from Classes.TheThreads import MyThread
from math import sin, cos, pi, sqrt
from pyPS4Controller.controller import Controller

import configparser
import io
import os
configfile_name = "config.ini"

class Servo:
    def __init__(self, name, servopin, minangle, maxangle, center_pwm, center_degrees, transition, transitionspeed, servorotation, area):
        self.__name = name
        self.__servopin = servopin
        self.__minangle = minangle
        self.__maxangle = maxangle
        self.__center_pwm = center_pwm
        self.__center_degrees = center_degrees
        self.__transition = transition
        self.__transitionspeed = transitionspeed
        self.__servorotation = servorotation
        self.__area = area
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)
        self.__servostatus = 0

    def SetCenterPWM(self, i, newStatus):
        self.__center_pwm = newStatus
        
    def GetStatus(self):
        return self.__servostatus

    def SetStatus(self, i, newStatus):
        self.__servostatus = newStatus
        
    def GetServopin(self):
        return self.__servopin
        
    def GetArea(self):
        return self.__area
        
    def GetCenterDegrees(self):
        return self.__center_degrees
        
    def GetMaxAngle(self):
        return self.__minangle
        
    def GetMinAngle(self):
        return self.__maxangle
        
    def ease_in_out_sine(self, current_time, start_value, change_in_value, duration):
        calc = 1-change_in_value/2 * (cos(pi*current_time/duration) - 1) + start_value -1
        #print("current_time: ",current_time, " start_value: ", start_value, " change_in_value: ",change_in_value," duration: ",duration, " calc: ",calc)
        return int(calc)
                
                
    def move(self, servopin, angle_pwm, transition):
        angle_degrees = self.deg2pwm(angle_pwm, self.__servorotation)
        if transition == 0:
            print ("angle: ",angle_pwm, " pin: ", servopin, self.__servorotation, " transition:", transition)
        
            
            self.pwm.set_pwm(servopin, 1, angle_degrees)
            
            #self.SetStatus(servopin, angle_degrees)
            #print ("Status: ", s.GetStatus() )
        
        else:
        
            sourcepos = self.GetStatus()
        
            if(sourcepos == angle_degrees):
                return
            
            start_time=time.time()
            
            changeinVal = angle_degrees-sourcepos
            while (time.time()-start_time) <= self.__transitionspeed:
                angle = self.ease_in_out_sine(current_time=time.time()-start_time, 
                                      start_value=sourcepos, 
                                      change_in_value=changeinVal,
                                      duration=self.__transitionspeed)
                angle_degrees = self.deg2pwm(angle_pwm, self.__servorotation)
                self.pwm.set_pwm(servopin, 1, angle_degrees)
            
        self.SetStatus(servopin, angle_degrees)       
        
        #print ("   Transitionangle: ",angle, " pwm: ",self.deg2pwm(angle, self.__servorotation))
        
    
    def printdata(self):
        print(self.__name, self.__minangle, self.__maxangle, self.__transition, self.__transitionspeed)
        
    def deg2pwm(self, angle, rotation):
        if rotation == 1:
            r= angle*(self.__maxangle-self.__minangle)/self.GetMaxAngle() + self.__center_pwm
        else:
            r=-angle*(self.__maxangle-self.__minangle)/self.GetMaxAngle() + self.__center_pwm
        
        return(int(r))
        
    def angle(self):
        return self.__angle

maxangle= 80
minangle=-80
center_pwm=306
servos = [
    Servo(name="Coxa_LF",  servopin=0, minangle=minangle, maxangle=maxangle, center_pwm=center_pwm, center_degrees=0, transition=0,  transitionspeed=1, servorotation=  1, area=45),
    Servo(name="Femur_LF", servopin=1, minangle=minangle, maxangle=maxangle, center_pwm=center_pwm, center_degrees=0, transition=0,  transitionspeed=1, servorotation=  1, area=45),
    Servo(name="Tibia_LF", servopin=2, minangle=minangle, maxangle=maxangle, center_pwm=center_pwm, center_degrees=0, transition=0,  transitionspeed=1, servorotation=  1, area=45),
    
    Servo(name="Coxa_RF",  servopin=3, minangle=minangle, maxangle=maxangle, center_pwm=center_pwm, center_degrees=0, transition=0,  transitionspeed=1, servorotation= -1, area=45),
    ]
    
Coxa_LF  = servos[0] 
Femur_LF = servos[1] 
Tibia_LF = servos[2] 

Coxa_RF  = servos[3] 


for servo in servos:  
    servo.GetStatus()
    # every Servo has its own thread!


def initialize():
    
    transition=0
    pyrothred_COXA_FRONT_LEFT = MyThread(target=servo.move, args=(Coxa_LF.GetServopin(), Coxa_LF.GetCenterDegrees() , Coxa_LF, transition ))
    pyrothred_COXA_FRONT_LEFT.start()   
    pyrothred_FEMUR_FRONT_LEFT = MyThread(target=servo.move, args=(Femur_LF.GetServopin(), Femur_LF.GetCenterDegrees() , Femur_LF, transition ))
    pyrothred_FEMUR_FRONT_LEFT.start()
    pyrothred_TIBIA_FRONT_LEFT = MyThread(target=servo.move, args=(Tibia_LF.GetServopin(), Tibia_LF.GetCenterDegrees() , Tibia_LF, transition ))
    pyrothred_TIBIA_FRONT_LEFT.start()
     
    pyrothred_FEMUR_FRONT_RIGHT = MyThread(target=servo.move, args=(Coxa_RF.GetServopin(), Coxa_RF.GetCenterDegrees(), Coxa_RF, transition ))
    pyrothred_FEMUR_FRONT_RIGHT.start()
    
    
def test():
    print ("test")
    transition=1
    pyrothred_COXA_FRONT_LEFT = MyThread(target=servo.move, args=(Coxa_LF.GetServopin(), Coxa_LF.GetCenterDegrees() , Coxa_LF, transition ))
    pyrothred_COXA_FRONT_LEFT.start()   
    pyrothred_FEMUR_FRONT_LEFT = MyThread(target=servo.move, args=(Femur_LF.GetServopin(), 45 , Femur_LF, transition ))
    pyrothred_FEMUR_FRONT_LEFT.start()
    pyrothred_TIBIA_FRONT_LEFT = MyThread(target=servo.move, args=(Tibia_LF.GetServopin(), -45 , Tibia_LF, transition ))
    pyrothred_TIBIA_FRONT_LEFT.start()
    
    time.sleep(2)
    
    pyrothred_COXA_FRONT_LEFT = MyThread(target=servo.move, args=(Coxa_LF.GetServopin(), Coxa_LF.GetCenterDegrees() , Coxa_LF, transition ))
    pyrothred_COXA_FRONT_LEFT.start()   
    pyrothred_FEMUR_FRONT_LEFT = MyThread(target=servo.move, args=(Femur_LF.GetServopin(), -45 , Femur_LF, transition ))
    pyrothred_FEMUR_FRONT_LEFT.start()
    pyrothred_TIBIA_FRONT_LEFT = MyThread(target=servo.move, args=(Tibia_LF.GetServopin(), +45 , Tibia_LF, transition ))
    pyrothred_TIBIA_FRONT_LEFT.start()
    
def release():
    for i in range(len(servos)):
        servos[i].pwm.set_pwm(i, 0, 0)
        
def calibrate():
    for i in range(len(servos)): 
        servos[i].pwm.set_pwm(i, 0, 0)
        print (    "Coxa_LF -> 0,  Femur_LF-> 4,  Tibia_LF-> 8")
        print (    "Coxa_RF -> 1,  Femur_RF-> 5,  Tibia_RF-> 9")
        print (    "Coxa_RR -> 2,  Femur_RR-> 6,  Tibia_RR-> 10")
        print (    "Coxa_LR -> 3,  Femur_LR-> 7,  Tibia_LR-> 11")
        
        print("Status: " ,servos[i].GetStatus())
        # Check if there is already a configurtion file
        if not os.path.isfile(configfile_name):
            # Create the configuration file as it doesn't exist yet
            cfgfile = open(configfile_name, 'w')

            # Add content to the file
            Config = configparser.ConfigParser()
            Config.add_section("LF")
            Config.set("LF", "Coxa-center", "306")
            Config.set("LF", "Femur-center", "306")
            Config.set("LF", "Tibia-center", "306")
            Config.add_section("RF")
            Config.set("RF", "Coxa-center", "306")
            Config.set("RF", "Femur-center", "306")
            Config.set("RF", "Tibia-center", "306")
            Config.add_section("RR")
            Config.set("RR", "Coxa-center", "306")
            Config.set("RR", "Femur-center", "306")
            Config.set("RR", "Tibia-center", "306")
            Config.add_section("RL")
            Config.set("RL", "Coxa-center", "306")
            Config.set("RL", "Femur-center", "306")
            Config.set("RL", "Tibia-center", "306")        
           
            Config.write(cfgfile)
            cfgfile.close()
            
    # Load the configuration file
    with open("config.ini") as f:
        sample_config = f.read()
    config = configparser.RawConfigParser(allow_no_value=True)
    config.readfp(io.StringIO(sample_config))
	
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
    # Print some contents
    print("\nPrint some contents")
    print(config.get("LF", "Coxa-center"))
    servos[1].SetCenterPWM(1, int(config.get("LF", "Coxa-center")))


arg=sys.argv[1]

'''
if len(sys.argv) >2:
    limb = sys.argv[2]
'''

if arg=="i":
    initialize()

if arg=="r":
    release()

if arg=="t":
    initialize()
    test()
    
if arg=="c":
    initialize()
    calibrate()

def joyleft(self, value):
    
    # 0 - 32767
    para = int( value*Coxa_LF.GetArea()/32767)
    pyrothred_FEMUR_FRONT_LEFT = MyThread(target=servo.move, args=(Coxa_LF.GetServopin(), para, Coxa_LF, 0 ))
    pyrothred_FEMUR_FRONT_LEFT.start()
    pyrothred_FEMUR_FRONT_RIGHT = MyThread(target=servo.move, args=(Coxa_RF.GetServopin(), para, Coxa_RF, 0 ))
    pyrothred_FEMUR_FRONT_RIGHT.start()
    
def joyright(self, value):
    
    # 0 - 32767
    para = int( value*Coxa_RF.GetArea()/32767)
    pyrothred_FEMUR_FRONT_LEFT = MyThread(target=servo.move, args=(Coxa_LF.GetServopin(), para, Coxa_LF, 0 ))
    pyrothred_FEMUR_FRONT_LEFT.start()
    pyrothred_FEMUR_FRONT_RIGHT = MyThread(target=servo.move, args=(Coxa_RF.GetServopin(), para, Coxa_RF, 0 ))
    pyrothred_FEMUR_FRONT_RIGHT.start()


### run PS4Controller
class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        
    def on_L3_left(self, value):
        joyleft("L3_left", value)

    def on_L3_right(self, value):
        joyleft("L3_right", value)

    
    def on_x_press(self):
        print("on_x_Press")
        test()
        
    def on_triangle_press(self):
        print("on_triangle_press")
        initialize()
        
'''
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
