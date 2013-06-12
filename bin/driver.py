#!/usr/bin/env python
import roslib; 
from numpy.ma.core import ceil
roslib.load_manifest("crazyflie_ros")
import rospy
from std_msgs.msg import UInt16
import tf
import tf.msg as TFMSG
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from crazyflie_ros.cfg import driverConfig as driverCFG




import time, sys
from optparse import OptionParser
from threading import Thread
from math import pi as PI
from math import sqrt, sin, cos, degrees, radians, atan2, atan
import numpy as np
from sensor_msgs.msg import Joy

from crazyflie_ros.msg import mag as magMSG
from crazyflie_ros.msg import gyro as gyroMSG
from crazyflie_ros.msg import acc as accMSG
from crazyflie_ros.msg import motor as motorMSG
from crazyflie_ros.msg import rpyt as rpytMSG
from crazyflie_ros.msg import bat as batMSG
from crazyflie_ros.msg import baro as baroMSG
from crazyflie_ros.msg import plot as plotMSG
from crazyflie_ros.msg import attitude as attitudeMSG
from crazyflie_ros.msg import hover as hoverMSG
 
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie import State as CFState
from cflib.crazyflie.log import Log
from cfclient.utils.logconfigreader import LogVariable, LogConfig







def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    reverse = dict((value, key) for key, value in enums.iteritems())
    enums['to_str'] = reverse
    return type('Enum', (), enums)

Button = enum(L1=10,R1=11,Select=0,Start=3,L2=8,R2=9,Up=4,Right=5,Down=6,Left=7,Square=15,Cross=14,Triangle=12,Circle=13)
Axes = enum(SLL=0,SLU=1,SRL=2,SRU=3,Up=4,Right=5,Down=6,Left=7,L2=8,R2=9,L1=10,R1=11,Triangle=12,Circle=13,Cross=14,Square=15,AccL=16,AccF=17,AccU=18,GyroY=19)



MAX_THRUST = 65365.0


# TODO:shouldnt min thrust be subtracted?
def thrustToPercentage( thrust):
    return ((float(thrust)/MAX_THRUST)*100.0)

def percentageToThrust( percentage):
    return int(MAX_THRUST*(percentage/100.0))

def deadband(value, threshold):
    if abs(value)<threshold:
        return 0.0
    elif value>0:
        return value-threshold
    else:
        return value+threshold
   
        

class JoyController:
    def __init__(self):     
        # Use this flag when the human is flying the quad.
        self.human_flight = True
           
        self.joy_scale = [-1,1,-1,1,1] #RPYT
        self.trim_roll = 0
        self.trim_pitch = 0
        self.max_angle = 30
        self.max_yawangle = 200
        
        
        self.max_thrust = 80.
        self.min_thrust = 25.
        self.max_thrust_raw = percentageToThrust(self.max_thrust)
        self.min_thrust_raw = percentageToThrust(self.min_thrust)       
        self.old_thurst_raw = 0
        
        self.slew_limit = 45
        self.slew_rate = 30
        self.slew_limit_raw = percentageToThrust(self.slew_limit)            
        self.slew_rate_raw = percentageToThrust(self.slew_rate)   
        
        self.dynserver = None
        self.prev_cmd = None
        self.curr_cmd = None
        self.hover = False
        
        
    def released(self, id):
        return self.prev_cmd.buttons[id] and not self.curr_cmd.buttons[id]
    
    def pressed(self, id):
        return not self.prev_cmd.buttons[id] and self.curr_cmd.buttons[id]
    
    def held(self, id):
        return self.prev_cmd.buttons[id] and self.curr_cmd.buttons[id]    
        
    #Needed so the class can change the dynserver values with button presses    
    def set_dynserver(self, server):
        self.dynserver = server
        
        
    def thurstToRaw(self, joy_thrust):           
        
        # Deadman button or invalid thrust
        if not self.curr_cmd.buttons[Button.L1] or joy_thrust>1:
            return 0
        
        raw_thrust = 0
        if joy_thrust > 0.01:
            raw_thrust = self.min_thrust_raw + joy_thrust*(self.max_thrust_raw-self.min_thrust_raw)
              
        
        if self.slew_rate_raw>0 and self.slew_limit_raw > raw_thrust:
            if self.old_thurst_raw > self.slew_limit_raw:
                self.old_thurst_raw = self.slew_limit_raw
            if raw_thrust < (self.old_thurst_raw - (self.slew_rate_raw/100)):
                raw_thrust = self.old_thurst_raw - self.slew_rate_raw/100
            if joy_thrust < 0 or raw_thrust < self.min_thrust_raw:
                raw_thrust = 0
        self.old_thurst_raw = raw_thrust
        
        return raw_thrust        
    
    def get_control(self, joymsg):
        #Should run at 100hz. Use launch files roslaunch crazyflie_row joy.launch
        if self.prev_cmd == None:
            self.prev_cmd = joymsg
            return (0,0,0,0,False,False,0)
        
        self.curr_cmd = joymsg
        hover = False
        
        
        x = 0
        y = 0
        z = 0
        r = 0
        r2 = 0
        # Get stick positions [-1 1]
        if self.curr_cmd.buttons[Button.L1]:         
            x = self.joy_scale[0] * self.curr_cmd.axes[Axes.SLL] # Roll
            y = self.joy_scale[1] * self.curr_cmd.axes[Axes.SLU] # Pitch
            r = self.joy_scale[2] * self.curr_cmd.axes[Axes.SRL] # Yaw
            z = self.joy_scale[3] * self.curr_cmd.axes[Axes.SRU] # Thrust            
            r2 = self.joy_scale[4] * (self.curr_cmd.axes[Axes.L2] - self.curr_cmd.axes[Axes.R2])
            hover = self.curr_cmd.axes[Axes.L1]<-0.75
        
        roll = x * self.max_angle
        pitch = y * self.max_angle        
        yaw = 0
        
        if r2!=0:
            yaw = r2 * self.max_yawangle
        else:
            # Deadzone     
            if r < -0.2 or r > 0.2:
                if r < 0:
                    yaw = (r + 0.2) * self.max_yawangle * 1.25
                else:
                    yaw = (r - 0.2) * self.max_yawangle * 1.25
                
        thrust = self.thurstToRaw(z)   
        trimmed_roll = roll + self.trim_roll
        trimmed_pitch = pitch + self.trim_pitch
        
            
        # Control trim manually        
        new_settings = {}       
        if self.curr_cmd.buttons[Button.Left]:
            new_settings["trim_roll"] = max(self.trim_roll + self.curr_cmd.axes[Axes.Left]/10.0, -10)                         
        if self.curr_cmd.buttons[Button.Right]:
            new_settings["trim_roll"] =  min(self.trim_roll - self.curr_cmd.axes[Axes.Right]/10.0, 10)  
        if self.curr_cmd.buttons[Button.Down]:
            new_settings["trim_pitch"] = max(self.trim_pitch + self.curr_cmd.axes[Axes.Down]/10.0, -10)                
        if self.curr_cmd.buttons[Button.Up]:
            new_settings["trim_pitch"] = min(self.trim_pitch - self.curr_cmd.axes[Axes.Up]/10.0, 10)
        
        # Set trim to current input
        if self.released(Button.R1):
            new_settings["trim_roll"] = min(10, max(trimmed_roll, -10))
            new_settings["trim_pitch"] = min(10, max(trimmed_pitch, -10))   
            rospy.loginfo("Trim updated Roll/Pitch: %r/%r", round(new_settings["trim_roll"],2), round(new_settings["trim_roll"],2))
        
        # Reset Trim
        if self.released(Button.Square):
            new_settings["trim_roll"] = 0
            new_settings["trim_pitch"] = 0       
            rospy.loginfo("Pitch reset to 0/0")        
            
        if new_settings != {} and self.dynserver!=None:
            self.dynserver.update_configuration(new_settings)            
            
        hover_set = hover and not self.hover       
        self.hover = hover
        
        quad_cmd = (trimmed_roll,trimmed_pitch,yaw,thrust,hover,hover_set, z)
        
        # Cache prev joy command
        self.prev_cmd = self.curr_cmd
        
   
        return quad_cmd
    
    def reconfigure(self, config, level):
        self.trim_roll = config["trim_roll"]
        self.trim_pitch = config["trim_pitch"]
        self.max_angle = config["max_angle"]
        self.max_yawangle = config["max_yawangle"]
        self.max_thrust = config["max_thrust"]
        self.min_thrust = config["min_thrust"]
        self.slew_limit = config["slew_limit"]
        self.slew_rate = config["slew_rate"]       

        self.max_thrust_yaw = percentageToThrust(self.max_thrust)
        self.min_thrust_yaw = percentageToThrust(self.min_thrust)
        self.slew_limit_yaw = percentageToThrust(self.slew_limit)
        self.slew_rate_yaw = percentageToThrust(self.slew_rate)
             
        return config 


    
from collections import deque
from scipy.signal import filtfilt, butter

class Avg3:
    def __init__(self, leng=30):
        self.samples = deque(maxlen=leng)
        self.current = 0
        self.offset = None
        
    def zero_calib(self):
        if self.offset==None:
            self.offset = self.get_mean()
        else:
            self.offset += self.get_mean()
        rospy.loginfo("calibrated: " + str(self.offset))
        
    
    def set_len(self, size_q):
        temp = list(self.samples)
        self.samples = deque(maxlen=size_q)
        self.samples.extend(temp)

    def add(self, sample):
        if self.offset == None:
            self.samples.append(sample)
        else:            
            self.samples.append(sample-self.offset)

    def get_median(self):  
        return np.median(list(self.samples),0)
    
    def add_get_median(self, sample):
        self.samples.append(sample) 
        return self.get_median() 
   
    def get_mean(self,frac=1):
        # frac between 0 and 1. 0 means take last element, 1 means take all, 5 means take half
        
        if frac<=0:
            return list(self.samples)[-1]
        if frac>=1:          
            return np.mean(list(self.samples),0)
        #else
        nr = int(max(1,ceil(len(self.samples)/frac)))
        return np.mean(list(self.samples)[-nr:],0)
    
    def add_get_mean(self, sample):
        self.samples.append(sample) 
        return self.get_mean()   


class LPF:
    def __init__(self, alpha = 0.98, initial = None):
        self.alpha = alpha
        self.value = initial
        
    def reset(self):
        self.value = None
    def update(self, value):
        if self.value == None:
            self.value = value
        else:
            self.value = self.value*(1.-self.alpha) + value*self.alpha
        return self.value 
    def set_alpha(self, alpha):
        self.alpha = alpha
    def get_value(self):
        return self.value


    


class Driver:
    def __init__(self, options):
        self.options = options        
        
        self.crazyflie = Crazyflie()
        cflib.crtp.init_drivers()
        
        # Joystick controller. Listens to /joy and converts those messages to CF commands
        self.joy_controller = JoyController() 
                        
        # Some shortcuts
        self.HZ100 = 10
        self.HZ10 = 100
        self.HZ1 = 1000        
        self.HZ500 = 2
        self.HZ250 = 4
        self.HZ125 = 8
        self.HZ50 = 20

        # Callbacks the CF driver calls      
        self.crazyflie.connectSetupFinished.add_callback(self.connectSetupFinishedCB)
        self.crazyflie.connectionFailed.add_callback(self.connectionFailedCB)
        self.crazyflie.connectionInitiated.add_callback(self.connectionInitiatedCB)
        self.crazyflie.disconnected.add_callback(self.disconnectedCB)
        self.crazyflie.connectionLost.add_callback(self.connectionLostCB)
        self.crazyflie.linkQuality.add_callback(self.linkQualityCB)   
        
        # Advertise publishers 
        self.pub_acc   = rospy.Publisher("/cf/acc", accMSG)
        self.pub_mag   = rospy.Publisher("/cf/mag", magMSG)
        self.pub_gyro  = rospy.Publisher("/cf/gyro", gyroMSG)
        self.pub_baro  = rospy.Publisher("/cf/baro", baroMSG)
        self.pub_motor = rospy.Publisher("/cf/motor", motorMSG)
        self.pub_hover = rospy.Publisher("/cf/hover", hoverMSG)
        self.pub_attitude = rospy.Publisher("/cf/attitude", attitudeMSG)
        self.pub_bat   = rospy.Publisher("/cf/bat", batMSG)
        self.pub_tf    = tf.TransformBroadcaster()            
        
        # Subscribers           
        self.sub_tf    = tf.TransformListener()         
        self.sub_joy   = rospy.Subscriber("/joy", Joy, self.new_joydata)
        
        
        # Keep track of link quality to send out with battery status
        self.link = 0
        
        # Loggers 
        self.logMotor = None
        self.logBaro = None
        self.logMag = None
        self.logGyro = None
        self.logAttitude = None
        self.logAcc = None
        self.logBat = None
        self.logHover = None             
        self.logGravOffset = None #only here till it works
        
        # keep tracks if loggers are running
        self.acc_monitor = False
        self.gyro_monitor = False
        self.baro_monitor = False
        self.mag_monitor = False
        self.bat_monitor = False
        self.motor_monitor = False    
        self.attitude_monitor = False   
        self.hover_monitor = False 
        
        # CF params we wanna change
        self.cf_params = {"hover.baro_alpha":0.92, "hover.kp":0.05, "hover.ki":0.00, "hover.kd":0.00}
        
        # Dynserver                
        self.dynserver = DynamicReconfigureServer(driverCFG, self.reconfigure)
        self.joy_controller.set_dynserver(self.dynserver)
                
                   
        self.connect(self.options)
        
    def connect(self, options):     
        """Look for crazyflie every 2s and connect to the specified one if found"""                          
        rospy.loginfo("Waiting for crazyflie...")
        while not rospy.is_shutdown():
            interfaces = cflib.crtp.scan_interfaces()
            if len(interfaces)>1:
                radio = None
                rospy.loginfo("Found: ")  
                for i in interfaces:
                    rospy.loginfo("--> %s [%s]", i[0],i[1])
                    if i[0].startswith(options.uri):                                
                        radio = i[0]
                if radio!=None:
                    self.crazyflie.open_link(radio)
                    break
            else:
                rospy.sleep(2)                                     
 
 
    def new_joydata(self, joymsg):
        """Joydata arrived. Should happen at 100hz."""
        
        # Parse joydata and extract commands
        roll, pitch, yawrate, thrust, hover, set_hover, hover_change = self.joy_controller.get_control(joymsg)   
        
        # Send to flie              
        self.send_control((roll, pitch, yawrate, thrust, hover, set_hover, hover_change))
        

    def reconfigure(self, config, level):
        """ Fill in local variables with values received from dynamic reconfigure clients (typically the GUI)."""
        
        # Allow the joystick class to use the callback and update config
        config = self.joy_controller.reconfigure(config, level)  
        
        # On / off logging
        if self.gyro_monitor != config["read_gyro"]:
            self.gyro_monitor = config["read_gyro"]           
            if self.logGyro != None:
                if self.gyro_monitor:               
                    self.logGyro.start()
                    rospy.loginfo("Gyro Logging started")
                else:   
                    self.logGyro.stop()
                    rospy.loginfo("Gyro Logging stopped")

        if self.hover_monitor != config["read_hover"]:
            self.hover_monitor = config["read_hover"]           
            if self.logHover != None:
                if self.hover_monitor:               
                    self.logHover.start()
                    rospy.loginfo("Hover Logging started")
                else:   
                    self.logHover.stop()
                    rospy.loginfo("Hover Logging stopped")                    
                    
        if self.acc_monitor != config["read_acc"]:
            self.acc_monitor = config["read_acc"]           
            if self.logGyro != None:
                if self.acc_monitor:               
                    self.logAcc.start()
                    rospy.loginfo("Acc Logging started")
                else:   
                    self.logAcc.stop()
                    rospy.loginfo("Acc Logging stopped")
                    
        if self.baro_monitor != config["read_baro"]:
            self.baro_monitor = config["read_baro"]           
            if self.logBaro != None:
                if self.baro_monitor:               
                    self.logBaro.start()
                    rospy.loginfo("Baro Logging started")
                else:   
                    self.logBaro.stop()
                    rospy.loginfo("Baro Logging stopped")
                    
        if self.mag_monitor != config["read_mag"]:
            self.mag_monitor = config["read_mag"]           
            if self.logMag != None:
                if self.mag_monitor:               
                    self.logMag.start()
                    rospy.loginfo("Mag Logging started")
                else:   
                    self.logMag.stop()
                    rospy.loginfo("Mag Logging stopped")
                    
        if self.bat_monitor != config["read_bat"]:
            self.bat_monitor = config["read_bat"]           
            if self.logBat != None:
                if self.bat_monitor:               
                    self.logBat.start()
                    rospy.loginfo("Battery/Link Logging started")
                else:   
                    self.logBat.stop()
                    rospy.loginfo("Battery/Link Logging stopped")
                    
        if self.attitude_monitor != config["read_attitude"]:
            self.attitude_monitor = config["read_attitude"]           
            if self.logAttitude != None:
                if self.attitude_monitor:               
                    self.logAttitude.start()
                    rospy.loginfo("Attitude Logging started")
                else:   
                    self.logAttitude.stop()
                    rospy.loginfo("Attitude Logging stopped")
                                                                                                                        
        if self.motor_monitor != config["read_motor"]:
            self.motor_monitor = config["read_motor"]           
            if self.logMotor != None:
                if self.motor_monitor:               
                    self.logMotor.start()
                    rospy.loginfo("Motor Logging started")
                else:   
                    self.logMotor.stop()
                    rospy.loginfo("Motor Logging stopped")                                                                                                                        

            
        # SET CRAZYFLIE PARAMS
        # Could be done nicer, eg looping through key,value pairs and renaming the cfg variables
        if self.cf_params["hover.baro_alpha"] != config["pressure_smooth"]:
             self.cf_params["hover.baro_alpha"] = config["pressure_smooth"]
             self.send_param("hover.baro_alpha", config["pressure_smooth"])
        
        
        if self.cf_params["hover.kp"] != config["p"]:
             self.cf_params["hover.kp"] = config["p"]
             self.send_param("hover.kp", config["p"]) 
       
        if self.cf_params["hover.ki"] != config["i"]:
             self.cf_params["hover.ki"] = config["i"]
             self.send_param("hover.ki", config["i"]) 
       
        if self.cf_params["hover.kd"] != config["d"]:
             self.cf_params["hover.kd"] = config["d"]
             self.send_param("hover.kd", config["d"]) 
                           
             
            
        return config
    
    def send_param(self, key, value):
        self.crazyflie.param.set_value(key, str(value))
         
    def restart(self):
        rospy.loginfo("Restarting Driver")
        rospy.sleep(1)
        self.connect(self.options)         
        
    def logErrorCB(self, errmsg):
        rospy.logerr("Log error: %s", errmsg)
        
    def send_control(self,cmd):
        """ Roll, pitch in deg, yaw in deg/s, thrust in 10000-60000, hover bool, set_hover bool, hover chance float -1 to +1 """
        roll, pitch, yawrate, thrust, hover, set_hover, hover_change = cmd    
        if self.crazyflie.state == CFState.CONNECTED:    
            self.crazyflie.commander.send_setpoint(roll, pitch, yawrate, thrust, hover, set_hover, hover_change)
        
        
    def setup_log(self):
        
        """ Console callbacks """
        self.crazyflie.console.receivedChar.add_callback(self.consoleCB)
 
        rospy.sleep(0.25)
        
        """ ATTITUDE LOGGING @ 100hz """
        logconf = LogConfig("attitude", self.HZ100) #ms
        logconf.addVariable(LogVariable("attitude.q0", "float"))
        logconf.addVariable(LogVariable("attitude.q1", "float"))
        logconf.addVariable(LogVariable("attitude.q2", "float"))
        logconf.addVariable(LogVariable("attitude.q3", "float"))             
        self.logAttitude = self.crazyflie.log.create_log_packet(logconf)  
        if (self.logAttitude is not None):
            self.logAttitude.dataReceived.add_callback(self.logCallbackAttitude)
            self.logAttitude.error.add_callback(self.logErrorCB)
            if self.attitude_monitor:
                self.logAttitude.start()
                rospy.loginfo("Attitude Logging started")     
        else:
            rospy.logwarn("Could not setup Attitude logging!")   
            
        rospy.sleep(0.25)
        
        
        """ HOVER LOGGING @ 100hz """
        logconf = LogConfig("hover", self.HZ100) #ms
        logconf.addVariable(LogVariable("hover.p", "float"))
        logconf.addVariable(LogVariable("hover.i", "float"))
        logconf.addVariable(LogVariable("hover.d", "float"))
        logconf.addVariable(LogVariable("hover.err", "float"))
        logconf.addVariable(LogVariable("hover.target", "float"))
        logconf.addVariable(LogVariable("hover.pid", "float"))     
        self.logHover = self.crazyflie.log.create_log_packet(logconf)  
        if (self.logHover is not None):
            self.logHover.dataReceived.add_callback(self.logCallbackHover)
            self.logHover.error.add_callback(self.logErrorCB)
            if self.hover_monitor:
                self.logHover.start()
                rospy.loginfo("Hover Logging started")     
        else:
            rospy.logwarn("Could not setup Hover logging!")               
             
        rospy.sleep(0.25)  
        
       
       
        # ONLY HERE UNTIL IT WORKS 
        # TODO REMOVE ME  
        """ GRAV OFFSET LOGGING @ 100hz """
        logconf = LogConfig("gravoffset", self.HZ100) #ms
        logconf.addVariable(LogVariable("gravoffset.x", "float"))
        logconf.addVariable(LogVariable("gravoffset.y", "float"))
        logconf.addVariable(LogVariable("gravoffset.z", "float")) 
     
        self.logGravOffset = self.crazyflie.log.create_log_packet(logconf)  
        if (self.logGravOffset is not None):
            self.logGravOffset.dataReceived.add_callback(self.logCallbackGravOffset)
            self.logGravOffset.error.add_callback(self.logErrorCB)
            if True:
                self.logGravOffset.start()
                rospy.loginfo("GravOffset Logging started")     
        else:
            rospy.logwarn("Could not setup GravOffset logging!")               
             
        rospy.sleep(0.25)        
         
        
        
        
        """ GYROMETER LOGGING @ 100hz """
        logconf = LogConfig("LoggingGyro", self.HZ100) #ms
        logconf.addVariable(LogVariable("gyro.x", "float"))
        logconf.addVariable(LogVariable("gyro.y", "float"))
        logconf.addVariable(LogVariable("gyro.z", "float"))         
        self.logGyro = self.crazyflie.log.create_log_packet(logconf)  
        if (self.logGyro is not None):
            self.logGyro.dataReceived.add_callback(self.logCallbackGyro)
            self.logGyro.error.add_callback(self.logErrorCB)
            if self.gyro_monitor:
                self.logGyro.start()
                rospy.loginfo("Gyro Logging started")     
        else:
            rospy.logwarn("Could not setup Gyro logging!")  
            
        rospy.sleep(0.25)          
             
        """ ACCELEROMETER LOGGING @ 100hz """
        logconf = LogConfig("LoggingAcc", self.HZ100) #ms
        logconf.addVariable(LogVariable("acc.x", "float"))
        logconf.addVariable(LogVariable("acc.y", "float"))
        logconf.addVariable(LogVariable("acc.z", "float"))
        logconf.addVariable(LogVariable("acc.xw", "float"))
        logconf.addVariable(LogVariable("acc.yw", "float"))
        logconf.addVariable(LogVariable("acc.zw", "float"))   
        self.logAcc = self.crazyflie.log.create_log_packet(logconf)  
        if (self.logAcc is not None):
            self.logAcc.dataReceived.add_callback(self.logCallbackAcc)
            self.logAcc.error.add_callback(self.logErrorCB)
            if self.acc_monitor:
                self.logAcc.start()
                rospy.loginfo("Acc Logging started")     
        else:
            rospy.logwarn("Could not setup Acc logging!")     
            
        rospy.sleep(0.25)       

        """ MAGNETOMETER LOGGING @ 100hz """
        logconf = LogConfig("LoggingMag", self.HZ100) #ms          
        logconf.addVariable(LogVariable("mag.x", "float"))        
        logconf.addVariable(LogVariable("mag.y", "float"))
        logconf.addVariable(LogVariable("mag.z", "float"))        
        self.logMag = self.crazyflie.log.create_log_packet(logconf)  
        if (self.logMag is not None):
            self.logMag.dataReceived.add_callback(self.logCallbackMag)
            self.logMag.error.add_callback(self.logErrorCB)
            if self.mag_monitor:
                self.logMag.start()
                rospy.loginfo("Mag Logging started")     
        else:
            rospy.logwarn("Could not setup Mag logging!")            


        rospy.sleep(0.25)
        
        """ BAROMETER LOGGING @ 100hz """
        logconf = LogConfig("LoggingBaro", self.HZ100) #ms
        logconf.addVariable(LogVariable("baro.asl_raw", "float"))
        logconf.addVariable(LogVariable("baro.asl", "float"))
        logconf.addVariable(LogVariable("baro.temp", "float"))
        logconf.addVariable(LogVariable("baro.pressure", "float"))              
        self.logBaro = self.crazyflie.log.create_log_packet(logconf)  
        if (self.logBaro is not None):
            self.logBaro.dataReceived.add_callback(self.logCallbackBaro)
            self.logBaro.error.add_callback(self.logErrorCB)
            if self.baro_monitor:
                self.logBaro.start()
                rospy.loginfo("Baro Logging started")     
        else:
            rospy.logwarn("Could not setup Baro logging!")                                                 
            
        rospy.sleep(0.25)    
            
        """ BATTERY/LINK LOGGING @ 10hz """
        logconf = LogConfig("LoggingBat", self.HZ10) #ms
        logconf.addVariable(LogVariable("pm.vbat", "float"))
        logconf.addVariable(LogVariable("pm.state", "uint8_t"))
        logconf.addVariable(LogVariable("pm.state_charge", "uint8_t"))
        self.logBat = self.crazyflie.log.create_log_packet(logconf)  
        if (self.logBat is not None):
            self.logBat.dataReceived.add_callback(self.logCallbackBat)
            self.logBat.error.add_callback(self.logErrorCB)
            if self.bat_monitor:
                self.logBat.start()
                rospy.loginfo("Bat Logging started")     
        else:
            rospy.logwarn("Could not setup Battery/Link logging!")
            
        rospy.sleep(0.25)    
      
            
        #TODO motor logging not working
        """ MOTOR LOGGING @ 100hz"""
        logconf = LogConfig("LoggingMotor", self.HZ100) #ms
        logconf.addVariable(LogVariable("motor.m1", "uint32_t")) 
        logconf.addVariable(LogVariable("motor.m2", "uint32_t"))
        logconf.addVariable(LogVariable("motor.m3", "uint32_t"))
        logconf.addVariable(LogVariable("motor.m4", "uint32_t"))  
        logconf.addVariable(LogVariable("motor.thrust", "uint16_t"))  
        self.logMotor = self.crazyflie.log.create_log_packet(logconf)  
        if (self.logMotor is not None):
            self.logMotor.dataReceived.add_callback(self.logCallbackMotor)
            self.logMotor.error.add_callback(self.logErrorCB)
            if self.motor_monitor:
                self.logMotor.start()
                rospy.loginfo("Motor Logging started")     
        else:
            rospy.logwarn("Could not setup Motor logging!")                                    
        
         
 
    def logCallbackAcc(self, data):
        """Accelerometer axis data in mG --> G"""
        msg = accMSG()
        msg.header.stamp = rospy.Time.now()
        msg.acc[0] = data["acc.x"]
        msg.acc[1] = data["acc.y"]
        msg.acc[2] = data["acc.z"]
        msg.acc_w[0] = data["acc.xw"]
        msg.acc_w[1] = data["acc.yw"]
        msg.acc_w[2] = data["acc.zw"]        
        self.pub_acc.publish(msg)
        
        self.pub_tf.sendTransform(msg.acc_w,(0,0,0,1),
             rospy.Time.now(), 
             "/acc_w","/cf_q")
        
        self.pub_tf.sendTransform(msg.acc,(0,0,0,1),
             rospy.Time.now(), 
             "/acc", "/cf_q")
        
    def logCallbackMag(self, data):
        """TODO UNITS"""
        msg = magMSG()
        msg.header.stamp = rospy.Time.now()
        msg.mag[0] = data["mag.x"]
        msg.mag[1] = data["mag.y"]
        msg.mag[2] = data["mag.z"]      
        self.pub_mag.publish(msg)       
        
        self.pub_tf.sendTransform(msg.mag/np.linalg.norm(msg.mag),(0,0,0,1),
             rospy.Time.now(), 
             "/mag",
             "/cf_q")
                
    def logCallbackGyro(self, data):    
        """Gyro axis data in deg/s -> rad/s"""       
        msg = gyroMSG()   
        msg.header.stamp = rospy.Time.now()
        msg.gyro[0] = radians(data["gyro.x"])
        msg.gyro[1] = radians(data["gyro.y"])
        msg.gyro[2] = radians(data["gyro.z"])
        self.pub_gyro.publish(msg)
        
    def logCallbackHover(self, data):    
        """Output data regarding hovering""" 
        msg = hoverMSG()   
        msg.header.stamp = rospy.Time.now()
        msg.err = data["hover.err"]
        msg.pid = data["hover.pid"]
        msg.p = data["hover.p"]
        msg.i = data["hover.i"]
        msg.d = data["hover.d"]
        msg.asl_target = data["hover.target"]
        self.pub_hover.publish(msg)        

    def logCallbackAttitude(self, data):
        msg = attitudeMSG()   
        msg.header.stamp = rospy.Time.now()
        q0 = data["attitude.q0"]
        q1 = data["attitude.q1"]
        q2 = data["attitude.q2"]
        q3 = data["attitude.q3"]
        msg.q = np.array((q0,q1,q2,q3))
        self.pub_attitude.publish(msg)        
        self.pub_tf.sendTransform((0, 0, 0),(q1,q2,q3,q0),
             rospy.Time.now(), 
             "/cf_q",
             "/world") 
        
    def logCallbackGravOffset(self, data):
        pass
        #TODO_ cannot subtract the offsets :/
        #print data
        
    def logCallbackBaro(self, data):
        msg = baroMSG()
        msg.header.stamp = rospy.Time.now()
        msg.temperature = data["baro.temp"]
        msg.pressure = data["baro.pressure"]
        msg.asl = data["baro.asl"]
        msg.asl_raw = data["baro.asl_raw"]
        self.pub_baro.publish(msg)     
             
                
    def logCallbackBat(self, data):
        msg = batMSG()
        msg.header.stamp = rospy.Time.now()
        msg.link = self.link
        msg.bat_v = data["pm.vbat"]
        msg.bat_p = (data["pm.vbat"]-3.0)/1.15*100.
        msg.charge_state = data["pm.state_charge"]
        msg.state = data["pm.state"]
        self.pub_bat.publish(msg)

        
    def logCallbackMotor(self, data):
        msg = motorMSG()
        msg.header.stamp = rospy.Time.now()
        msg.pwm[0] = thrustToPercentage(data["motor.m1"])
        msg.pwm[1] = thrustToPercentage(data["motor.m2"])
        msg.pwm[2] = thrustToPercentage(data["motor.m3"])
        msg.pwm[3] = thrustToPercentage(data["motor.m4"])
        msg.thrust = thrustToPercentage(data["motor.thrust"])
        self.pub_motor.publish(msg)                                                  
        
        
    def connectSetupFinishedCB(self, uri=None):
        rospy.loginfo("...Connected")      
        self.setup_log()
        
    def connectionFailedCB(self, msg=None, errmsg=None):
        rospy.logerr("Connection failed: %s", errmsg)
        exit()
        #self.restart()
        
    def connectionInitiatedCB(self, msg=None):
        rospy.loginfo("Connecting to: %s...", msg)
        
    def disconnectedCB(self, msg=None):
        if msg!=None:
            rospy.loginfo("Disconnected from: %s", msg)
        
    def connectionLostCB(self, msg=None, errmsg=None):
        rospy.logerr("Connection lost: %s", errmsg)        
        self.restart()
        
    def linkQualityCB(self, msg=None):
        self.link = msg
        
    def consoleCB(self, msg):
        rospy.loginfo(msg)        

    def shutdown(self):
        self.crazyflie.close_link()
        

def run(args=None):    
    parser = OptionParser(description='Crazyflie ROS Driver')
    parser.add_option('--uri', '-u',
                        dest='uri',
                        default='radio://0/4',
                        help='URI to connect to')    
    (options, leftargs) = parser.parse_args(args=args)
        
    #Load some DB depending on options 
    rospy.init_node('CrazyFlieDriver')   
    driver = Driver(options)

    #START NODE
    try:
        rospy.spin()
    except KeyboardInterrupt:    
        rospy.loginfo('...exiting due to keyboard interrupt')
    driver.shutdown()


if __name__ == "__main__":   
    run(sys.argv)
