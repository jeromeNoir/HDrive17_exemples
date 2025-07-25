# Library to run the motor Hdrive17 from henschel-robotics
# ''
# written by j.Noir 31/03/2023

# webpage: https://henschel-robotics.ch/
# email Chris Henschel: info@henschel-robotics.ch

# 1. For oscillation: exemple
# - mydrive=hdrive() #create an instance motor
# - mydrive.amplitude=15 #oscillation amplitude in deg +/- 15 deg here
# - mydrive.frequency=0.1 #oscillation frequency in Hz
# - mydrive.gearRatio=n #If using a gear head enter the gear ratio, by default it is set to 1
# - mydrive.connect_drive() #connect to the drive
# - mydrive.mode=135 #select the drive control mode (134,135 or 136) by default use 136
# - mydrive.home() #set the position to 0deg, the position of the drive when you turned it on the last time
# - time.sleep(1) #pause for 1 second, hold the position
# - mydrive.start() #start the oscillation, it will run continuously in an independent thread
# - time.sleep(5) #let it run for 5s, if you want to run for an unknown duration skip this line and the next one. To stop the drive use a new cell and type excetute mydrive.stop()
# - mydrive.stop()# stop the motor, home, reset

# 2. To move to a position:
# - mydrive=hdrive() #create an instance motor if not done already
# - mydrive.gearRatio=n #If using a gear head enter the gear ratio, by default it is set to 1
# - mydrive.connect_drive() #connect to the drive if not done already, ok to do it again
# - mydrive.move(10) # move to 10deg, the 0 is the position when you last powered the drive

# 3. For constant rotation
# - mydrive=hdrive() #create an instance motor if not done already
# - mydrive.gearRatio=n #If using a gear head enter the gear ratio, by default it is set to 1
# - mydrive.connect_drive() #connect to the drive if not done already, ok to do it again
# - mydrive.rotate(30) # rotate continously at 30rpm
# - time.sleep(5) #let it run for 5s, if you want to run for an unknown duration skip this line and the next one. To stop the drive use a new cell and type excetute mydrive.stop()
# - mydrive.stop()# stop the motor, home, reset
# ''

########## ADDITIONAL BUILD-IN FUNCTIONS

# self.restart(): restart the motor
# self.setzero(): The sensor zero position to the current position !will not be retained after a restart
# self.home(): move to the current zero position
# self.disconnect(): disconnect the drive tcp port and unpower the drive
# self.servoOff(): unpower the servo, the axis can move freely, the drive tcp port is still connected
# self.calibration(): runs the calibration procedure. Best to do from the web interface.


#IMPORT

import socket
import time
import struct
import threading
import numpy as np
from matplotlib.pyplot import *


#CLASS HDRIVE:

class hdrive:
    def __init__(self):
        self.udp_port=1001
        self.driveIP="192.168.1.102"
        self.mode=134
        self.gearRatio=1
        self.offset=0
        self.phase=0
        self.amplitude=10
        self.frequency=0.5
        self.pos=0
        self.torque=400
        self.speed=1000
        self.acc=10000
        self.decc=10000
        self.stop_event=threading.Event()  # create an event object
        self.s_send = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.s_receive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
    def get_drive_output(self):

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.bind(('', self.udp_port))
            data, addr = s.recvfrom(82)
            position=int(data[18:28])
            speed=int(data[37:45])
            torque=int(data[55:63])
            t=int(data[71:81])
            return t,position,speed,torque
        
    
    def start(self):
        if self.mode==134:
            print('start running in mode 134')
            #
            self.run = threading.Thread(target=self.run134)
            self.run.start()

        elif self.mode==136:
            print('start running in mode 136')
            self.run = threading.Thread(target=self.run136)
            self.run.start()

        elif self.mode==135:
            print('start running in mode 135')
            self.run = threading.Thread(target=self.run135)
            self.run.start()
            
    def connect_drive(self):
        self.s_send.connect(('192.168.1.102', 1000))
        

    def disconnect(self):
        self.s_send.shutdown(1)
        
            
    def stop(self):
        self.stop_event.set()
        

    def home(self):
        # str_command=b'"<control pos=\"0\" speed=\"400\" torque=\"600\" mode=\"134\" acc=\"10000\" decc=\"10000\" />"'
        #str_command=b'"<control pos=\"0\" speed=\"%f\" torque=\"%f\" mode=\"134\" acc=\"%f\" decc=\"%f\" />"'%(self.speed,self.torque,self.acc,self.decc)
        str_command=b'"<control mode="101" />"'
        self.s_send.sendall(str_command) 

    def servoOff(self):
        str_command=b'"<control pos=\"0\" speed=\"400\" torque=\"600\" mode=\"0\" acc=\"10000\" decc=\"10000\" />"'
        self.s_send.sendall(str_command)

    def move(self,new_pos):
        new_pos*=self.gearRatio
        new_pos*=10
        print(new_pos)
        str_command=b'"<control pos=\"%f\" speed=\"%f\" torque=\"%f\" mode=\"134\" acc=\"%f\" decc=\"%f\" />"'%(new_pos,self.speed,self.torque,self.acc,self.decc)
        self.s_send.sendall(str_command) 

    def calibration(self):
        str_command=b'"<control pos=\"0\" speed=\"100\" torque=\"600\" mode=\"9\" acc=\"1000\" decc=\"1000\" />"'
        self.s_send.sendall(str_command)

    def setzero(self):
        str_command=b'"<system mode="2" a="1" b="2" c="3" />"'
        self.s_send.sendall(str_command)

    def rotate(self,rpm):
        rpm*=self.gearRatio
        str_command=b'"<control pos=\"0\" speed=\"%f\" torque=\"%f\" mode=\"130\" acc=\"%f\" decc=\"%f\" />"'%(rpm,self.torque,self.acc,self.decc)
        self.s_send.sendall(str_command)

    def restart(self):
        str_command=b'"<system mode="6" />"'
        self.s_send.sendall(str_command)
        
    def run134(self):
    
    
        epsilon=(self.gearRatio*self.amplitude)
        f=(self.frequency)
        
        self.target_pos=[]
        self.act_pos=[]
        self.target_time=[]
        self.act_time=[]
        self.target_speed=[]
        self.act_speed=[]
        self.target_acc=[]



        t_start=time.time_ns()
        t0_drive,position_drive,speed_drive,torque_drive=self.get_drive_output()

        while not self.stop_event.is_set():
            current_time=time.time_ns()
            
            t_drive,position_drive,speed_drive,torque_drive=self.get_drive_output()
            t_drive-=t0_drive
            t_drive/=1e6
            
        
            self.act_time.append(t_drive)
            self.act_pos.append(position_drive/10)
            self.act_speed.append(speed_drive)

            t=(current_time-t_start)/1e9
            
            pos=epsilon*np.sin(2*np.pi*f*t)
            speed=epsilon*2*np.pi*f*np.cos(2*np.pi*f*t)
            acc=epsilon*(2*np.pi*f)**2*np.cos(2*np.pi*f*t)
            
            pos_motor=pos*10
           

            self.target_pos.append(pos)
            self.target_speed.append(speed)
            self.target_acc.append(acc)
            self.target_time.append(t)
            str_command=b'"<control pos=\"%f\" speed=\"%f\" torque=\"%f\" mode=\"134\" acc=\"%f\" decc=\"%f\" />"'%(pos_motor,self.speed,self.torque,self.acc,self.decc)
            self.s_send.sendall(str_command) 
            
        

        self.target_time=np.array(self.target_time)
        self.target_pos=np.array(self.target_pos)
        self.target_speed=np.array(self.target_speed)
        self.target_acc=np.array(self.target_acc)
        self.act_time=np.array(self.act_time)
        self.act_pos=np.array(self.act_pos)
        self.act_speed=np.array(self.act_speed)
        
    def run135(self):
        
        
        #get the oscillation parameters and rescale for the mode 136
        _amplitude=self.gearRatio*self.amplitude*10
        _frequency=self.frequency*100
        _offset=self.gearRatio*self.offset*10
        _phase=self.phase
        
        # current_time=time.time_ns()
        st = '"<control pos=\"' + str(_amplitude) + '\" frequency=\"' + str(_frequency) + '\" torque=\"%f\" mode=\"135\" offset=\"'%self.torque + str(_offset) + '\" phase=\"' + str(_phase) + '\" />"';
        self.s_send.sendall(st.encode('ascii')) # send XML command
        
        self.target_pos=[]
        self.act_pos=[]
        self.target_time=[]
        self.act_time=[]
        self.target_speed=[]
        self.act_speed=[]
        self.target_acc=[]
        
        # t_start=time.time_ns()
        t0_drive,position_drive,speed_drive,torque_drive=self.get_drive_output()
        
        while not self.stop_event.is_set():
            
            
            t_drive,position_drive,speed_drive,torque_drive=self.get_drive_output()
            t_drive-=t0_drive
            t_drive/=1e6
            f=self.frequency
            epsilon=self.gearRatio*self.amplitude
            

         

            self.act_time.append(t_drive)
            self.act_pos.append(position_drive/10)
            self.act_speed.append(speed_drive)

            
            pos=epsilon*np.sin(2*np.pi*f*t_drive)
            speed=epsilon*2*np.pi*f*np.cos(2*np.pi*f*t_drive)
            acc=epsilon*(2*np.pi*f)**2*np.cos(2*np.pi*f*t_drive)

         

            self.target_pos.append(pos)
            self.target_speed.append(speed)
            self.target_acc.append(acc)
            self.target_time.append(t_drive)
            
        self.target_time=np.array(self.target_time)
        self.target_pos=np.array(self.target_pos)
        self.target_speed=np.array(self.target_speed)
        self.target_acc=np.array(self.target_acc)
        self.act_time=np.array(self.act_time)
        self.act_pos=np.array(self.act_pos)
        self.act_speed=np.array(self.act_speed)

    def run136(self):
        
        
        #get the oscillation parameters and rescale for the mode 136
        _amplitude=self.gearRatio*self.amplitude*10
        _frequency=self.frequency*100
        _offset=self.gearRatio*self.offset*10
        _phase=self.phase
        
        current_time=time.time_ns()
        st = '"<control pos=\"' + str(_amplitude) + '\" frequency=\"' + str(_frequency) + '\" torque=\"%f\" mode=\"136\" offset=\"'%self.torque + str(_offset) + '\" phase=\"' + str(_phase) + '\" />"';
        self.s_send.sendall(st.encode('ascii')) # send XML command
        
        self.target_pos=[]
        self.act_pos=[]
        self.target_time=[]
        self.act_time=[]
        self.target_speed=[]
        self.act_speed=[]
        self.target_acc=[]
        
        t_start=time.time_ns()
        t0_drive,position_drive,speed_drive,torque_drive=self.get_drive_output()
        
        while not self.stop_event.is_set():
            
            
            t_drive,position_drive,speed_drive,torque_drive=self.get_drive_output()
            t_drive-=t0_drive
            t_drive/=1e6
            f=self.frequency
            epsilon=self.gearRatio*self.amplitude
            

         

            self.act_time.append(t_drive)
            self.act_pos.append(position_drive/10)
            self.act_speed.append(speed_drive)

            
            pos=epsilon*np.sin(2*np.pi*f*t_drive)
            speed=epsilon*2*np.pi*f*np.cos(2*np.pi*f*t_drive)
            acc=epsilon*(2*np.pi*f)**2*np.cos(2*np.pi*f*t_drive)


            self.target_pos.append(pos)
            self.target_speed.append(speed)
            self.target_acc.append(acc)
            self.target_time.append(t_drive)
            
        self.target_time=np.array(self.target_time)
        self.target_pos=np.array(self.target_pos)
        self.target_speed=np.array(self.target_speed)
        self.target_acc=np.array(self.target_acc)
        self.act_time=np.array(self.act_time)
        self.act_pos=np.array(self.act_pos)
        self.act_speed=np.array(self.act_speed)

