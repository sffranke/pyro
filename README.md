**PYRO**

I have replicated some four-legged robot projects and even with good documentation it is often not possible to follow the installation. There are extreme dependencies on the OS and many software packages, especially when using ROS, which even with a successful build can make long-term operation questionable.  Therefore my attempt to build a robot based on a Pi 4, python and other elementary packages. 

I was able to rebuild these two projects very well and I learned a lot.  
https://github.com/PetoiCamp/OpenCat  
https://github.com/mike4192/spotMicro  

Furthermore, I decided to work with hobby mini servos like MG92B for the small prototype. They are affordable, sufficient as proof of concept, and if everything goes well you can still realize a big dog.
The big version uses PDI-HV5523MG 23KG servos.  

*setup raspian 2022-04-04-raspios-bullseye-armhf.img.xz*  
```
sudo apt-get update; sudo apt-get upgrade  
sudo apt install pi-bluetooth  

sudo bluetoothctl    
pairable on   
scan on  
scan off  
pair A4:AE:11:DE:4E:BB  
trust A4:AE:11:DE:4E:BB 

wget http://repo.continuum.io/miniconda/Miniconda3-latest-Linux-armv7l.sh  

sudo /bin/bash Miniconda3-latest-Linux-armv7l.sh  
conda create -n env106 python==3.10.6  
conda acivate enc3106  

/home/pi/.conda/envs/env3106/bin/pip install python3-pip  
/home/pi/.conda/envs/env3106/bin/pip install python3-numpy  
/home/pi/.conda/envs/env3106/bin/pip install adafruit-pca9685  
/home/pi/.conda/envs/env3106/bin/pip install adafruit-circuitpython-servokit  
/home/pi/.conda/envs/env3106/bin/pip install pyPS4Controller  
/home/pi/.conda/envs/env3106/bin/pip install urdfpy  
sudo apt install i2c-tools  
sudo raspi-config -> enable interfacing, i2c  
sudo chown :i2c /dev/i2c-1   
sudo chmod g+rw /dev/i2c-1 
  
install configparser  
sudo apt install git  
```

Servos:  
PDI-HV5523MG 23KG High Precision Metal Gear Digital HV  
Operating Speed (8.4V): 0.16 sec/60° 
Maximum pulse width: 500-2500us  

To move the motors to a defined starting position at the beginning (init), I control the motors one after the other, because I am still working with a laboratory power supply, which I cannot load highly. Later, the power supply will be taken over by a Lipo, which can deliver the necessary high currents, then the motors will be controlled simultaneously.  

*Version_Step_1.py:*  
Basic movement, static, no inverse kinematic yet 
Implemented rest, sit, calibate and stand/balance for testing the servo control and motion.  
Done.

*Version_Step_2.py:*
Inverse Kinematics     
Moves using the great package from Mike (https://github.com/mike4192/spotMicro):  
git clone git@github.com:sffranke/spot_micro_kinematics_python.git  
Done.  

*Version_Step_3.py:*  
Add PS4 Controller for posing.  
pyro.py c -> calibration pose  
pyro.py r -> rest or init pose  
pyro.py b -> stand pose  
pyro.py i -> sequence of inverse kinematiks poses for testing  

pyro.py without argument runs the PS4 Controller:   
Press X-Button: rest and exit programm  
Press Square-Button: rest or stand pose  
Press Circle-Button: sit or stand pose  
press Options-Button: calibration pose (LL)  
Done.

*Version_Step_4.py:*  
Added conig file confg.ini for properties and servo parameters.  
Done.  

*Version_Step_5.py:*  
Add MPU6050* for balancing  
PS4 Buttonon_share sitches MPU Messurement and correction on and off for balancing in pitch axis and roll-axis.  
In progress  

*Walking*   
coming not so soon :-)

