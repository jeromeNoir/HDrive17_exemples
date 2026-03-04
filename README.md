# Library to run the motor Hdrive17 from henschel-robotics
''
written by j.Noir 04/03/2026

[webpage:] (https://henschel-robotics.ch/)
email Chris Henschel: info@henschel-robotics.ch

# NOTES
- YOU NEED PYTHON 3.8.XX AND NUMPY 1.XX
- the IP of the drive is 192.168.1.102
- the IP of the host PC with ethernet connection must be set manually to 192.168.1.222

# INITIALISATION

- mydrive=hdrive() #create an instance motor
- mydrive.init() #connect to the drive
- mydrive.gearRatio=n #If using a gear head enter the gear ratio, by default it is set to 1
- mydrive.mode='osc' for oscillation, 'rot' for rotation, 'pos' for position control

# HOME POSITION
During the initialisation the current position of the drive is set to home, to change the home position, move the drive to the desired position using mydrive.move(position in deg) and execute mydrive.setzero().
To come back to the home position execute mydrive.home(). You can move the motor by hand and set the zero position to the current position by executing mydrive.setzero().

# STARTING AND STOPPING THE DRIVE
In both modes the motor will run indefinitely after you call mydrive.start(), to stop it you call mydrive.stop(). 
The start function will create a thread, you can run the stop function in a different cell
To fixe the duration of the oscillation or rotation use mydrive.start(),time.sleep(duration in seconds) and then call mydrive.stop() all in one cell.
e.g. to run an oscillation for 5s:
- mydrive.mode='osc'
- mydrive.start()
- time.sleep(5)
- mydrive.stop()

to run untill you stop
cell 1:
- mydrive.mode='osc'
- mydrive.start() 
cell 2:
- mydrive.stop()

# RESTARTING THE DRIVE
You can restart the drive with mydrive.restart(), this will reset the drive, you will need to run the initialisation steps again after a restart.



# OSCILLATION:
FOR SAFETY REASONS, THE DRIVE SET THE ZERO POSITION AS IT IS AT THE BEGINING OF THE OSCILLATION.

## set the oscillation parameters
- mydrive.amplitude=15 #oscillation amplitude in deg +/- 15 deg here
- mydrive.frequency=0.1 #oscillation frequency in Hz
- mydrive.start() #start the oscillation, it will run continuously in an independent thread
- time.sleep(duration in seconds) or let it run until you call mydrive.stop() to stop the oscillation


# MOVE TO POSITION:
- mydrive.move(10) # move to 10deg, the 0 is the position when you last powered the drive

# ROTATION: (NOT VERY STABLE BELOW 100rpm)
## set the rotation parameters
- mydrive.rpm=30 #rotation speed in rpm

## rotate for a fixed duration:
- mydrive.mode='rot'
- mydrive.start()
- time.sleep(3)
- mydrive.stop()

## or for undefined duration:

cell 1:
- mydrive.mode='rot'  
- mydrive.start()    
cell 2:
- mydrive.stop()


######### ADDITIONAL BUILD-IN FUNCTIONS

- mydrive.disconnect(): disconnect the drive tcp port and unpower the drive
- mydrive.servoOff(): unpower the servo, the axis can move freely, the drive tcp port is still connected
- mydrive.speed=30 #set the speed for the position control mode in rpm
- mydrive.acc=30000 #set the acceleration for the position control mode in rpm/s
- mydrive.decc=30000 #set the decceleration for the position control mode in rpm/s
- mydrive.torque=200 #set the torque for the position control mode in mNm, the maximum is 600mNm
- mydrive.get_drive_output() #get the current time, position, speed and torque of the drive, the position is in deg, the speed in rpm and the torque in mNm
