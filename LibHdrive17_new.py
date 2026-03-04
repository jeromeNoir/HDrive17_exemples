# Library to run the motor Hdrive17 from henschel-robotics
# ''
# written by j.Noir 04/03/2026

# webpage: https://henschel-robotics.ch/
# email Chris Henschel: info@henschel-robotics.ch

# INITIALISATION

# - mydrive=hdrive() #create an instance motor
# - mydrive.init() #connect to the drive
# - mydrive.gearRatio=n #If using a gear head enter the gear ratio, by default it is set to 1
# - mydrive.mode='osc' for oscillation, 'rot' for rotation, 'pos' for position control

# HOME POSITION
# During the initialisation the current position of the drive is set to home, to change the home position, move the drive to the desired position using mydrive.move(position in deg) and execute mydrive.setzero().
# To come back to the home position execute mydrive.home(). You can move the motor by hand and set the zero position to the current position by executing mydrive.setzero().

# STARTING AND STOPPING THE DRIVE
# In both modes the motor will run indefinitely after you call mydrive.start(), to stop it you call mydrive.stop(). 
# The start function will create a thread, you can run the stop function in a different cell
# To fixe the duration of the oscillation or rotation use mydrive.start(),time.sleep(duration in seconds) and then call mydrive.stop() all in one cell.
# e.g. to run an oscillation for 5s:
# - mydrive.mode='osc'
# - mydrive.start()
# - time.sleep(5)
# - mydrive.stop()

# to run untill you stop
# cell 1:
# - mydrive.mode='osc'
# - mydrive.start() 
# cell 2:
# - mydrive.stop()

#RESTARTING THE DRIVE
#You can restart the drive with mydrive.restart(), this will reset the drive, you will need to run the initialisation steps again after a restart.



# OSCILLATION:
## FOR SAFETY REASONS, THE DRIVE SET THE ZERO POSITION AS IT IS AT THE BEGINING OF THE OSCILLATION.

## set the oscillation parameters
# - mydrive.amplitude=15 #oscillation amplitude in deg +/- 15 deg here
# - mydrive.frequency=0.1 #oscillation frequency in Hz
# - mydrive.start() #start the oscillation, it will run continuously in an independent thread
# - time.sleep(duration in seconds) or let it run until you call mydrive.stop() to stop the oscillation
# # - mydrive.stop()# stop the motor, home, reset

# MOVE TO POSITION:
# - mydrive.move(10) # move to 10deg, the 0 is the position when you last powered the drive

# ROTATION: (NOT VERY STABLE BELOW 100rpm)
## set the rotation parameters
# - mydrive.rpm=30 #rotation speed in rpm

# rotate for a fixed duration:
# - mydrive.mode='rot'
# - mydrive.start()
# - time.sleep(3)
# - mydrive.stop()

# or for undefined duration:

# cell 1:
# - mydrive.mode='rot'  
# # - mydrive.start()    
# cell 2:
# - mydrive.stop()


########## ADDITIONAL BUILD-IN FUNCTIONS

# mydrive.disconnect(): disconnect the drive tcp port and unpower the drive
# mydrive.servoOff(): unpower the servo, the axis can move freely, the drive tcp port is still connected
# mydrive.speed=30 #set the speed for the position control mode in rpm
# mydrive.acc=30000 #set the acceleration for the position control mode in rpm/s
# mydrive.decc=30000 #set the decceleration for the position control mode in rpm/s
# mydrive.torque=200 #set the torque for the position control mode in mNm, the maximum is 600mNm
# mydrive.get_drive_output() #get the current time, position, speed and torque of the drive, the position is in deg, the speed in rpm and the torque in mNm


#IMPORT

import socket
import time
# import struct
import threading
import numpy as np
from matplotlib.pyplot import *


#CLASS HDRIVE:

class hdrive:
    def __init__(self):
        self.udp_port=1001
        self.driveIP="192.168.1.102"
        self.gearRatio=1
        self.rpm=30
        self.offset=0
        self.phase=0
        self.amplitude=10
        self.frequency=0.5
        self.pos=0
        self.torque=200
        self.speed=100
        self.acc=3000
        self.decc=3000
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
        
    def init(self):
        self.connect_drive()
        time.sleep(0.1)
        self.setzero()
        time.sleep(0.1)
        
    
    def start(self):

        if self.mode=='osc':
            print('start the oscillation in mode 135')
            self.run = threading.Thread(target=self.run135)
            self.run.start()


        elif self.mode=='rot':
            print('start running in rotation')
            self.run = threading.Thread(target=self.rotate)
            self.run.start()

        else:
            print('mode not recognized, chose between "osc", "rot"')

            
    def connect_drive(self):
        self.s_send.connect(('192.168.1.102', 1000))
        

    def disconnect(self):
        self.s_send.shutdown(1)
        
            
    def stop(self):
        self.stop_event.set()
        

    def home(self):
        self.move(0) 


    def servoOff(self):
        str_command=b'"<control pos=\"0\" speed=\"400\" torque=\"600\" mode=\"0\" acc=\"10000\" decc=\"10000\" />"'
        self.s_send.sendall(str_command)

    def move(self,new_pos):
        _acc=self.acc
        _decc=self.decc
        self.acc=1000
        self.decc=500

        new_pos*=self.gearRatio
        new_pos*=10
        t,position,speed,torque=self.get_drive_output() #get the current position to compare with the target position
        print('Current position',position/10)
        print('Moving to position',new_pos/10)

        while position!=new_pos: 
            str_command=b'"<control pos=\"%f\" speed=\"%f\" torque=\"%f\" mode=\"134\" acc=\"%f\" decc=\"%f\" />"'%(new_pos,self.speed,self.torque,self.acc,self.decc)
            self.s_send.sendall(str_command) 
            t,position,speed,torque=self.get_drive_output()
        time.sleep(0.1)
        self.servoOff()
        self.acc=_acc
        self.decc=_decc

    def calibration(self):
        str_command=b'"<control pos=\"0\" speed=\"100\" torque=\"600\" mode=\"9\" acc=\"1000\" decc=\"1000\" />"'
        self.s_send.sendall(str_command)
        self.servoOff()

    def setzero(self):
        str_command=b'"<system mode="2" a="1" b="2" c="3" />"'
        self.s_send.sendall(str_command)
        self.servoOff()

    def rotate(self):
        print('start rotating at rpm until')
        rpm=self.rpm
        rpm*=self.gearRatio
        # str_command=b'"<control pos=\"0\" speed=\"%f\" torque=\"%f\" mode=\"130\" acc=\"%f\" decc=\"%f\" />"'%(rpm,self.torque,self.acc,self.decc)
        # self.s_send.sendall(str_command)
        while not self.stop_event.is_set():
            str_command=b'"<control pos=\"0\" speed=\"%f\" torque=\"%f\" mode=\"130\" acc=\"%f\" decc=\"%f\" />"'%(rpm,self.torque,self.acc,self.decc)
            self.s_send.sendall(str_command)
        
        self.stop_event.clear()
        self.servoOff()    

    def ocillate(self):
        print('start oscillating at frequency and amplitude until')
        while not self.stop_event.is_set():
            st = b'"<control pos=\"%f\" frequency=\"%f\" torque=\"200\" mode=\"135\" offset=\"%f\" phase=\"%f\" />"'%(self.gearRatio*self.amplitude*10,self.frequency*100,self.gearRatio*self.offset*10,self.phase)
            self.s_send.sendall(st) # send XML command 

        self.stop_event.clear()            
        self.servoOff()       
        
            

    def restart(self):
        str_command=b'"<system mode="6" />"'
        self.s_send.sendall(str_command)
        
    
        
    def run135(self):
        
        self.target_pos=[]
        self.act_pos=[]
        self.target_time=[]
        self.act_time=[]
        self.target_speed=[]
        self.act_speed=[]
        self.target_acc=[]
        
        # t_start=time.time_ns()
        t0_drive,position_drive,speed_drive,torque_drive=self.get_drive_output()
        self.setzero() # set the current position to zero, so the oscillation will be around the current position, you can comment this line if you want the oscillation to be around the home position
        
        while not self.stop_event.is_set():

            st = b'"<control pos=\"%f\" frequency=\"%f\" torque=\"200\" mode=\"135\" offset=\"%f\" phase=\"%f\" />"'%(self.gearRatio*self.amplitude*10,self.frequency*100,self.gearRatio*self.offset*10,self.phase)
            self.s_send.sendall(st) # send XML command 
            
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

        self.stop_event.clear()
        self.servoOff()     
        self.target_time=np.array(self.target_time)
        self.target_pos=np.array(self.target_pos)
        self.target_speed=np.array(self.target_speed)
        self.target_acc=np.array(self.target_acc)
        self.act_time=np.array(self.act_time)
        self.act_pos=np.array(self.act_pos)
        self.act_speed=np.array(self.act_speed)





################################## UNUSED FUNCTIONS, FOR DEV AND TESTING PURPOSES, NOT CALLED IN THE MAIN CODE
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

        self.stop_event.clear()        
        self.servoOff()     
        self.target_time=np.array(self.target_time)
        self.target_pos=np.array(self.target_pos)
        self.target_speed=np.array(self.target_speed)
        self.target_acc=np.array(self.target_acc)
        self.act_time=np.array(self.act_time)
        self.act_pos=np.array(self.act_pos)
        self.act_speed=np.array(self.act_speed)

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
            print("Running in mode 134")
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

        self.stop_event.clear()            
        self.servoOff() 

        self.target_time=np.array(self.target_time)
        self.target_pos=np.array(self.target_pos)
        self.target_speed=np.array(self.target_speed)
        self.target_acc=np.array(self.target_acc)
        self.act_time=np.array(self.act_time)
        self.act_pos=np.array(self.act_pos)
        self.act_speed=np.array(self.act_speed)