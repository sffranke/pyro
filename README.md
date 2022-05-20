**PYRO**

I have replicated some four-legged robot projects and even with good documentation it is often not possible to follow the installation. There are extreme dependencies on the OS and many software packages, especially when using ROS, which even with a successful build can make long-term operation questionable.  Therefore my attempt to build a robot based on Python and other elementary packages. 

I was able to rebuild these two projects very well and I learned a lot.  
https://github.com/PetoiCamp/OpenCat  
https://github.com/mike4192/spotMicro  

Furthermore, I decided to work with hobby mini servos like MG92B. They are affordable, sufficient as proof of concept, and if everything goes well you can still realize a big dog.

*setup raspian 2022-04-04-raspios-bullseye-armhf.img.xz*  
apt-get update; apt-get upgrade  

sudo apt install libgl1-mesa-glx  
sudo apt-get install -y libdbus-glib-1-2

#sudo apt install pi-bluetooth  

sudo bluetoothctl    
pairable on   
scan on  
scan off  
pair A4:AE:11:DE:4E:BB  
trust A4:AE:11:DE:4E:BB  

#sudo apt install python3-pip  
#sudo apt install python3-numpy
sudo pip3 install adafruit-pca9685  
sudo pip3 install pyPS4Controller  
sudo apt install i2c-tools  

sudo raspi-config -> enable interfacing, i2c  
sudo chown :i2c /dev/i2c-1   
sudo chmod g+rw /dev/i2c-1 

sudo pip3 install configparser  

#sudo apt install git  

sudo pip3 install matplotlib

