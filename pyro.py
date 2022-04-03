#!/usr/bin/python3
import sys
import Adafruit_PCA9685
import time
from Classes.TheThreads import MyThread
#from math import sin, cos, pi, sqrt
from numpy import *
from pyPS4Controller.controller import Controller

import configparser
import io
import os
configfile_name = "/home/ubuntu/pyro/config.ini"

class Servo:
    def __init__(self, name, servopin, minangle, maxangle, center_pwm, transition, transitionspeed, servorotation, area):
        self.__name = name
        self.__servopin = servopin
        self.__minangle = minangle
        self.__maxangle = maxangle
        self.__center_pwm = center_pwm
        self.__center_degrees = 0
        self.__transition = transition
        self.__transitionspeed = transitionspeed
        self.__servorotation = servorotation
        self.__area = area
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)
        self.__servostatus = 0

    def SetCenterPWM(self, newStatus):
        self.__center_pwm = newStatus
        
    def GetStatus(self):
        return self.__servostatus

    def SetStatus(self, newStatus):
        self.__servostatus = newStatus
        
    def GetServopin(self):
        return self.__servopin
    
    def GetCenterDegrees(self):
        return self.__center_degrees
        
    def GetArea(self):
        return self.__area
        
    def GetMaxAngle(self):
        return self.__minangle
        
    def GetMinAngle(self):
        return self.__maxangle
        
    def ease_in_out_sine(self, current_time, start_value, change_in_value, duration):
        """ sinusoidal easing in/out - accelerating until halfway, then decelerating """
        calc = 1-change_in_value/2 * (cos(pi*current_time/duration) - 1) + start_value 
        #print("current_time: ",current_time, " start_value: ", start_value, " change_in_value: ",change_in_value," duration: ",duration, " calc: ",calc)
        return int(calc)
    
    def ease_in_sine(self, current_time, start_value, change_in_value, duration):
        """ sinusoidal easing in - accelerating from zero velocity """
        calc = 1-change_in_value * cos(current_time / duration * (pi/2)) + change_in_value + start_value
        return int(calc)
    
    def ease_out_sine(self, current_time, start_value, change_in_value, duration):
        """ sinusoidal easing out - decelerating to zero velocity """
        calc = change_in_value * sin(current_time / duration * (pi/2)) + start_value
        return int(calc)
                
    def ease_in_expo(self, current_time, start_value, change_in_value, duration):
        """ exponential easing in - accelerating from zero velocity """
        calc = current_time * pow(2, 10 * (current_time / duration - 1)) + start_value
        return int(calc)

    def ease_out_expo(self, current_time, start_value, change_in_value, duration):
        """ exponential easing out - decelerating to zero velocity """
        calc = change_in_value * (pow (2, -10 * current_time / duration) + 1) + start_value
        return int(calc)

    def ease_in_out_expo(self, current_time, start_value, change_in_value, duration):
        """ exponential easing in/out - accelerating until halfway, then decelerating """
        current_time /= duration/2
        if(current_time < 1):
            calc = change_in_value/2 * pow(2, 10 * (current_time -1)) + start_value
            return int(calc)
        current_time -= 1
        calc = change_in_value/2 * (pow(2, -10 * current_time) + 2) + start_value
        return int(calc)

    def ease_in_circ(self, current_time, start_value, change_in_value, duration):
        """ circular easing in - accelerating from zero velocity """
        current_time /= duration
        calc = 1-change_in_value * (sqrt(1 - current_time*current_time) - 1) + start_value
        return int(calc)

    def ease_out_circ(self, current_time, start_value, change_in_value, duration):
        """ circular easing out - decelerating to zero velocity """
        current_time /= duration
        current_time -= 1
        calc = change_in_value * sqrt(1 - current_time*current_time) + start_value
        return int(calc)

    def ease_in_out_circ(self, current_time, start_value, change_in_value, duration):
        """ circular easing in/out - acceleration until halfway, then deceleration """
        current_time /= duration/2
        if (current_time < 1):
            calc = 1-change_in_value/2 * (sqrt(1 - current_time-current_time) -1) + start_value
            return int(calc)
        current_time -= 2
        calc = change_in_value / 2 * (sqrt(1 - current_time*current_time) + 1) + start_value
        return int(calc)
    
    def move(self, servopin, angle, transition):
        angle_pwm = self.deg2pwm(angle, self.__servorotation)
        if transition == 0:
            #print ("angle: ",angle_pwm, " pin: ", servopin, self.__servorotation, " transition:", transition)
            
            self.pwm.set_pwm(servopin, 1, angle_pwm)
            
            #self.SetStatus(servopin, angle_degrees)
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
                
                angle_pwm = self.deg2pwm(angle, self.__servorotation)
                print ("-> Angle: ",angle)
                self.pwm.set_pwm(servopin, 1, angle_pwm)
            
        self.SetStatus(angle)       
        
        #print ("   Transitionangle: ",angle, " pwm: ",self.deg2pwm(angle, self.__servorotation))
        
    
    def printdata(self):
        print(self.__name, self.__minangle, self.__maxangle, self.__transition, self.__transitionspeed)
        
    def deg2pwm(self, angle, rotation):
        if rotation == 1:
            pwm= angle*(self.__maxangle-self.__minangle)/self.GetMaxAngle() + self.__center_pwm
        else:
            pwm=-angle*(self.__maxangle-self.__minangle)/self.GetMaxAngle() + self.__center_pwm
        
        return(int(pwm))
        
    def angle(self):
        return self.__angle

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
    Coxa_LF_length = int(config.get("LF", "Coxa-length"))
    Coxa_LF_maxangle = int(config.get("LF", "Coxa-maxangle"))
    Coxa_LF_minangle = int(config.get("LF", "Coxa-minangle"))
    
    Femur_LF_center_pwm=int(config.get("LF", "Femur-center"))+int(config.get("LF", "Femur-center-dif"))
    Femur_LF_length = int(config.get("LF", "Femur-length"))
    Femur_LF_maxangle = int(config.get("LF", "Femur-maxangle"))
    Femur_LF_minangle = int(config.get("LF", "Femur-minangle"))
    
    Tibia_LF_center_pwm= int(config.get("LF", "Tibia-center"))+int(config.get("LF", "Tibia-center-dif"))
    Tibia_LF_length = int(config.get("LF", "Tibia-length"))
    Tibia_LF_maxangle = int(config.get("LF", "Tibia-maxangle"))
    Tibia_LF_minangle = int(config.get("LF", "Tibia-minangle"))
    
    Coxa_RF_center_pwm= int(config.get("RF", "Coxa-center")) +int(config.get("RF", "Coxa-center-dif"))
    Coxa_RF_length = int(config.get("RF", "Coxa-length"))
    Coxa_RF_maxangle = int(config.get("RF", "Coxa-maxangle"))
    Coxa_RF_minangle = int(config.get("RF", "Coxa-minangle"))
 
    Conf_speed = float(config.get("GENERAL", "speed"))
#maxangle= 80
#minangle=-80
speed = Conf_speed

servos = [
    Servo(name="Coxa_LF",  servopin=0, minangle=Coxa_LF_minangle, maxangle=Coxa_LF_maxangle, center_pwm=Coxa_LF_center_pwm, transition=0,  transitionspeed=speed, servorotation=  1, area=45),
    Servo(name="Femur_LF", servopin=1, minangle=Femur_LF_minangle, maxangle=Femur_LF_maxangle, center_pwm=Femur_LF_center_pwm, transition=0,  transitionspeed=speed, servorotation=  1, area=45),
    Servo(name="Tibia_LF", servopin=2, minangle=Tibia_LF_minangle, maxangle=Tibia_LF_maxangle, center_pwm=Tibia_LF_center_pwm, transition=0,  transitionspeed=speed, servorotation=  1, area=45),
    
    Servo(name="Coxa_RF",  servopin=3, minangle=Coxa_RF_minangle, maxangle=Coxa_LF_maxangle, center_pwm=Coxa_RF_center_pwm, transition=0,  transitionspeed=speed, servorotation= -1, area=45),
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
    pyrothred_FEMUR_FRONT_LEFT = MyThread(target=servo.move, args=(Femur_LF.GetServopin(), 0 , Femur_LF, transition ))
    pyrothred_FEMUR_FRONT_LEFT.start()
    pyrothred_TIBIA_FRONT_LEFT = MyThread(target=servo.move, args=(Tibia_LF.GetServopin(), 0 , Tibia_LF, transition ))
    pyrothred_TIBIA_FRONT_LEFT.start()
    
    time.sleep(2)
    
    pyrothred_COXA_FRONT_LEFT = MyThread(target=servo.move, args=(Coxa_LF.GetServopin(), Coxa_LF.GetCenterDegrees() , Coxa_LF, transition ))
    pyrothred_COXA_FRONT_LEFT.start()   
    pyrothred_FEMUR_FRONT_LEFT = MyThread(target=servo.move, args=(Femur_LF.GetServopin(), 45 , Femur_LF, transition ))
    pyrothred_FEMUR_FRONT_LEFT.start()
    pyrothred_TIBIA_FRONT_LEFT = MyThread(target=servo.move, args=(Tibia_LF.GetServopin(), 45 , Tibia_LF, transition ))
    pyrothred_TIBIA_FRONT_LEFT.start()
    
    time.sleep(2)
    
    pyrothred_COXA_FRONT_LEFT = MyThread(target=servo.move, args=(Coxa_LF.GetServopin(), Coxa_LF.GetCenterDegrees() , Coxa_LF, transition ))
    pyrothred_COXA_FRONT_LEFT.start()   
    pyrothred_FEMUR_FRONT_LEFT = MyThread(target=servo.move, args=(Femur_LF.GetServopin(), -45 , Femur_LF, transition ))
    pyrothred_FEMUR_FRONT_LEFT.start()
    pyrothred_TIBIA_FRONT_LEFT = MyThread(target=servo.move, args=(Tibia_LF.GetServopin(), -45 , Tibia_LF, transition ))
    pyrothred_TIBIA_FRONT_LEFT.start()

def get_IK_angles(drop, slew):

        l_femur = Femur_LF_length*10 # mm
        l_tibia = Tibia_LF_length*10 # mm

        hypotenuse = sqrt(drop**2+slew**2)
        phi_1 = arccos((hypotenuse**2 + l_femur**2-l_tibia**2)/(2*hypotenuse*l_femur))
        phi_2 = arctan(slew/drop)
        hip = rad2deg(phi_2-phi_1)
        knee = rad2deg(arccos((l_femur**2+l_tibia**2-hypotenuse**2)/(2*l_femur*l_tibia)))
        
        return [0, int(hip-90), int(knee-90)]

"""
INVERSE KINEMATICS by https://github.com/miguelasd688/Quadruped-dog-like-robot/blob/master/src/quadpod.py
"""
def checkdomain(D):
    if D > 1 or D < -1:
        print("____OUT OF DOMAIN____")
        if D > 1: 
            D = 0.99
            return D
        elif D < -1:
            D = -0.99
            return D
    else:
        return D
        

def solveIK_R(coord , coxa , femur , tibia): 
"""where:
   coord = np.array([x4,y4,z4])
   coxa = L1 
   femur = L2 
   tibia = L3
"""

    D = (coord[1]**2+(-coord[2])**2-coxa**2+(-coord[0])**2-femur**2-tibia**2)/(2*tibia*femur)  
    D = checkdomain(D) #This is just to deal with NaN solutions
    gamma = numpy.arctan2(-numpy.sqrt(1-D**2),D)
    tetta = -numpy.arctan2(coord[2],coord[1])-numpy.arctan2(numpy.sqrt(coord[1]**2+(-coord[2])**2-coxa**2),-coxa)
    alpha = numpy.arctan2(-coord[0],numpy.sqrt(coord[1]**2+(-coord[2])**2-coxa**2))-numpy.arctan2(tibia*numpy.sin(gamma),femur+tibia*numpy.cos(gamma))
    angles = numpy.array([-tetta, alpha, gamma])
    return angles

def ik(height, xpos):

        #arr = get_IK_angles(height,xpos)
        arr = solveIK_L([x4,y4,z4],5, 5, 1)
        print ("arr: ", arr)
        #return
    
        COXA_FRONT_LEFT_pos_angle  = arr[0]
        FEMUR_FRONT_LEFT_pos_angle = arr[1]
        TIBIA_FRONT_LEFT_pos_angle = arr[2]
        
        transition=0
        pyrothred_COXA_FRONT_LEFT = MyThread(target=servo.move, args=(Coxa_LF.GetServopin(), COXA_FRONT_LEFT_pos_angle, Coxa_LF, transition ))
        pyrothred_COXA_FRONT_LEFT.start()   
        pyrothred_FEMUR_FRONT_LEFT = MyThread(target=servo.move, args=(Femur_LF.GetServopin(), FEMUR_FRONT_LEFT_pos_angle, Femur_LF, transition ))
        pyrothred_FEMUR_FRONT_LEFT.start()
        pyrothred_TIBIA_FRONT_LEFT = MyThread(target=servo.move, args=(Tibia_LF.GetServopin(), TIBIA_FRONT_LEFT_pos_angle, Tibia_LF, transition ))
        pyrothred_TIBIA_FRONT_LEFT.start()
        
    
def release():
    for i in range(len(servos)):
        servos[i].pwm.set_pwm(i, 0, 0)
        
def calibrate():
    transition=0
    for i in range(len(servos)): 
        print (    "Coxa_LF -> 0,  Femur_LF-> 4,  Tibia_LF-> 8")
        print (    "Coxa_RF -> 1,  Femur_RF-> 5,  Tibia_RF-> 9")
        print (    "Coxa_RR -> 2,  Femur_RR-> 6,  Tibia_RR-> 10")
        print (    "Coxa_LR -> 3,  Femur_LR-> 7,  Tibia_LR-> 11")
        
        print("Status: " ,servos[i].GetStatus())
        pyrothred_COXA_FRONT_LEFT = MyThread(target=servo.move, args=(Coxa_LF.GetServopin(), Coxa_LF.GetCenterDegrees() , Coxa_LF, transition ))
        pyrothred_COXA_FRONT_LEFT.start()   
        pyrothred_FEMUR_FRONT_LEFT = MyThread(target=servo.move, args=(Femur_LF.GetServopin(), Femur_LF.GetCenterDegrees() , Femur_LF, transition ))
        pyrothred_FEMUR_FRONT_LEFT.start()
        pyrothred_TIBIA_FRONT_LEFT = MyThread(target=servo.move, args=(Tibia_LF.GetServopin(), Tibia_LF.GetCenterDegrees() , Tibia_LF, transition ))
        pyrothred_TIBIA_FRONT_LEFT.start()
    

#arg=sys.argv[1]
arg = "i"
'''
if len(sys.argv) >2:
    limb = sys.argv[2]
'''

if arg=="i":
    initialize()
    time.sleep(1)
    ik(80,0)  # mm

if arg=="r":
    release()

if arg=="t":
    #initialize()
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
