#!/usr/bin/python3

class Servo:
    def __init__(self, name, mingrad, maxgrad, grad, transition, transitionspeed):
        self.__name = name
        self.__mingrad = mingrad
        self.__maxgrad = maxgrad
        self.__grad = grad
        self.__transition = transition
        self.__transitionspeed = transitionspeed
        
    def printdata(self):
        print(self.__name, self.__mingrad, self.__maxgrad, self.__grad, self.__transition, self.__transitionspeed)
        
servos = [
    Servo("Coxa_LF", -45, 45, 0, 1, 0.1),
    #Servo("Coxa_RF", 45, -45, 0, 1, 0.1)
    ]

for servo in servos:  
    servo.printdata()


