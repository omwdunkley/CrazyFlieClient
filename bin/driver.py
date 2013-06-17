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



import csv
import time, sys, datetime
from optparse import OptionParser
from threading import Thread
from math import pi as PI
from math import sqrt, sin, cos, degrees, radians, atan2, atan
import numpy as np
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
from crazyflie_ros.msg import CFJoy as joyMSG
from crazyflie_ros.msg import mag_calib as magCalibMSG
from crazyflie_ros.msg import zpid as zpidMSG
 
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie import State as CFState
from cflib.crazyflie.log import Log
from cfclient.utils.logconfigreader import LogVariable, LogConfig



MAX_THRUST = 65365.0

class zSpeed:
    def __init__(self):
        self.speed = 0.0
        self.bias = 0.0
        self.pub_speed   = rospy.Publisher("/cf/speed", accMSG)
        self.bias_alpha = 0.99
        
    def add(self, sample):
        self.speed += sample * 0.01
        self.bias = self.bias * self.bias_alpha + self.speed * (1.0-self.bias_alpha) 
        
        msg = accMSG()
        msg.header.stamp = rospy.Time.now()
        msg.acc[0] = self.speed
        #msg.acc[1] = data["acc.y"]
        #msg.acc[2] = data["acc.z"]
        msg.acc_w[0] = self.bias
        msg.acc_w[1] = self.speed  - self.bias
        #msg.acc_w[2] = data["acc.zw"]        
        self.pub_speed.publish(msg)
        
    def reconfigure(self, config):
        #self.bias_alpha = config["zbias"]
        return config

class ChargeState:
    def __init__(self):
        self.mA100 = 0
        self.mA500 = 1
        self.mAMAX = 2
        self.states = {0:"100mA", 1:"500mA", 2:"max"}
        
    def get(self,nr):
        return self.states[nr]
        
    
class BatteryStatus:
    def __init__(self):
        self.BATTERY = 0
        self.CHARHING = 1
        self.CHARGED = 2
        self.LOW = 3
        self.SHUTDOWN = 4    
        self.states = {0:"BATTERY",1:"CHARGING",2:"CHARGED",3:"LOW",4:"SHUTDOWN"}       
    def get(self,nr):
        return self.states[nr]
    
    
# TODO:shouldnt min thrust be subtracted?
def thrustToPercentage( thrust):
    return ((float(thrust)/MAX_THRUST)*100.0)
    
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


    
    
class BatteryMonitor:
    def __init__(self):
        self.state = -1
        self.charge_state = -1
        self.voltage = Avg3(100)
        self.C_STATE = ChargeState()
        self.B_STATE = BatteryStatus()
        self.vmax = 4.2 # when charging
        self.vmin = 3.7 # when charging
        self.last_time = None
        self.t = None
        
        self.time_volt = []
        self.start_charge = None
        
        self.temp = -1
        
        
    def done(self):
        for v,t in self.time_volt:
            print str(v)+","+str(t)                
            self.time_volt = []
            
        
    def update(self, state, charge_state, voltage):   
        
        self.voltage.add(voltage)        
          
        # Battery state changed  
        if self.state != state:
            self.state = state
            rospy.loginfo("Battery State changed to " + self.B_STATE.get(state))
            
            # Battery become low
            if state == self.B_STATE.LOW:
                rospy.logwarn("Battery low!")
              
            # Finished charging  
            if state == self.B_STATE.CHARGED:
                rospy.loginfo("Charged in " + str( round((self.start_charge - rospy.Time.now()).to_sec()/60),2))
                self.done()
             
            # Started charging   
            if state == self.B_STATE.CHARHING:
                self.start_charge = rospy.Time.now()
                
            # Switched to battery    
            if state == self.B_STATE.BATTERY:
                self.done()
                
        # Charging State changed            
        if self.charge_state != charge_state:
            self.charge_state = charge_state
            rospy.loginfo("Chrage State changed to " + self.C_STATE.get(charge_state))
            
        if self.state == self.B_STATE.CHARHING:
            if voltage != self.temp:                
                self.temp = round(voltage,5)
                #rospy.loginfo("Time to charge:"+str(datetime.timedelta(minutes= round(self.predict_time(self.voltage.get_mean()),2))))
                
                t = round( (rospy.Time.now() - self.start_charge).to_sec(), 5)
                v = self.temp
                self.time_volt.append((t, self.temp ))
            
            
#     def predict_time(self, voltage):
#         # 4.2v @ 18 min
#         # 3.7v @ 1min
#         # = .0v = 17 min
#         return 17 - ((voltage - self.vmin) / (self.vmax-self.vmin) * 17.)
#              


class Driver:
    def __init__(self, options):
        self.options = options        
        
        self.crazyflie = Crazyflie()
        self.battery_monitor = BatteryMonitor()
        cflib.crtp.init_drivers()
        
        self.zspeed = zSpeed()
                
        
        #self.csvwriter = csv.writer(open('baro.csv', 'wb'), delimiter=',',quotechar='"', quoting=csv.QUOTE_MINIMAL)
        #self.baro_start_time = None
                 
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
        self.pub_zpid   = rospy.Publisher("/cf/zpid", zpidMSG)
        self.pub_tf    = tf.TransformBroadcaster()            
        
        # Subscribers           
        self.sub_tf    = tf.TransformListener()         
        self.sub_joy   = rospy.Subscriber("/cfjoy", joyMSG, self.new_joydata)
        self.sub_magCalib   = rospy.Subscriber("/magCalib", magCalibMSG, self.new_magCalib)
        
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
        self.logZPid = None                   
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
        self.zpid_monitor = False 
        
        # CF params we wanna be able to change
        self.cf_param_groups = ["hover", "sensorfusion6", "magCalib"]
        self.cf_params_cache = {}
       
        
        # Dynserver                
        self.dynserver = DynamicReconfigureServer(driverCFG, self.reconfigure)
        
        # Finished set up. Connect to flie            
        self.connect(self.options)
        
    def new_magCalib(self, msg):  
        """ Receive magnetometer calibration results. Send to flie via dyn conf"""      
        new_settings = {}
        #TODO        
        new_settings["magCalib_off_x"] = msg.offset[0]
        new_settings["magCalib_off_y"] = msg.offset[1]
        new_settings["magCalib_off_z"] = msg.offset[2]
        new_settings["magCalib_thrust_x"] = msg.thrust_comp[0]
        new_settings["magCalib_thrust_y"] = msg.thrust_comp[1]
        new_settings["magCalib_thrust_z"] = msg.thrust_comp[2]
        new_settings["magCalib_scale_x"] = msg.scale[0]
        new_settings["magCalib_scale_y"] = msg.scale[1]
        new_settings["magCalib_scale_z"] = msg.scale[2]        
        self.dynserver.update_configuration(new_settings)  
        rospy.loginfo("Updated magnetometer calibration data") 
    
    def connect(self, options):     
        """ Look for crazyflie every 2s and connect to the specified one if found"""                          
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
        """ Joydata arrived. Should happen at 100hz."""
        
        # Parse joydata and extract commands
        roll = joymsg.roll
        pitch = joymsg.pitch
        yawrate = joymsg.yaw
        thrust = joymsg.thrust
        hover = joymsg.hover
        set_hover = joymsg.hover_set
        hover_change = joymsg.hover_change
        
        # Send to flie              
        self.send_control((roll, pitch, yawrate, thrust, hover, set_hover, hover_change))
        

    def reconfigure(self, config, level):
        """ Fill in local variables with values received from dynamic reconfigure clients (typically the GUI)."""
        config = self.zspeed.reconfigure(config)
        


        # On / off logging
        if self.zpid_monitor != config["read_zpid"]:
            self.zpid_monitor = config["read_zpid"]           
            if self.logZPid != None:
                if self.zpid_monitor:               
                    self.logZPid.start()
                    rospy.loginfo("zPID Logging started")
                else:   
                    self.logZPid.stop()
                    rospy.loginfo("zPID Logging stopped")

        
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
        
        for key, value in config.iteritems():
            # Skip this key (whats its purpose??)            
            if key == "groups":
                continue
                    
            # Continue if variable exists and is correct
            if self.cf_params_cache.has_key(key):
                if config[key] == self.cf_params_cache[key]:
                    # Nothing changed
                    continue
            
            # Doesnt exist or has changed
            
            # Split into group and name
            s = key.split("_",1)
            
            # Make sure it is a group name pair
            if len(s) == 2:
                # make sure group is valid
                if s[0] in self.cf_param_groups:
                    # update cache, send variable to flie
                    self.cf_params_cache[key] = value                    
                    self.send_param(s[0]+"."+s[1], value)    
                    print "updated: ",s[0]+"."+s[1],"->",value   
                    rospy.sleep(0.1)
                                                    
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
        
        """ ZPID LOGGING @ 100hz """
        logconf = LogConfig("zpid", self.HZ100) #ms
        logconf.addVariable(LogVariable("zpid.p", "float"))
        logconf.addVariable(LogVariable("zpid.i", "float"))   
        logconf.addVariable(LogVariable("zpid.d", "float"))   
        logconf.addVariable(LogVariable("zpid.pid", "float"))              
        self.logZPid = self.crazyflie.log.create_log_packet(logconf)  
        if (self.logZPid is not None):
            self.logZPid.dataReceived.add_callback(self.logCallbackZPid)
            self.logZPid.error.add_callback(self.logErrorCB)
            if self.zpid_monitor:
                self.logZPid.start()
                rospy.loginfo("ZPID Logging started")     
        else:
            rospy.logwarn("Could not setup ZPID logging!")               
        rospy.sleep(0.25)
        
        
        """ HOVER LOGGING @ 100hz """
        logconf = LogConfig("hover", self.HZ100) #ms
        #logconf.addVariable(LogVariable("hover.p", "float"))
        #logconf.addVariable(LogVariable("hover.i", "float"))
        #logconf.addVariable(LogVariable("hover.d", "float"))
        #logconf.addVariable(LogVariable("hover.pid", "float"))
        logconf.addVariable(LogVariable("hover.err", "float"))
        logconf.addVariable(LogVariable("hover.target", "float"))        
        logconf.addVariable(LogVariable("hover.zSpeed", "float"))
        logconf.addVariable(LogVariable("hover.zBias", "float"))    
        logconf.addVariable(LogVariable("hover.acc_vspeed", "float"))
        logconf.addVariable(LogVariable("hover.asl_vspeed", "float"))
        
       
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
        logconf.addVariable(LogVariable("mag.x_raw", "float"))
        logconf.addVariable(LogVariable("mag.y_raw", "float"))
        logconf.addVariable(LogVariable("mag.z_raw", "float"))
               
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
        logconf.addVariable(LogVariable("baro.aslRaw", "float"))
        logconf.addVariable(LogVariable("baro.asl", "float"))
        logconf.addVariable(LogVariable("baro.temp", "float"))
        logconf.addVariable(LogVariable("baro.pressure", "float"))
        logconf.addVariable(LogVariable("baro.aslLong", "float"))                 
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
        logconf.addVariable(LogVariable("motor.vLong", "float")) #TODO move
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
        
        self.zspeed.add(msg.acc_w[2])  
        
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
        msg.magRaw[0] = data["mag.x_raw"]
        msg.magRaw[1] = data["mag.y_raw"]
        msg.magRaw[2] = data["mag.z_raw"] 
        self.pub_mag.publish(msg)       
        
        if np.linalg.norm(msg.magRaw)>1:
            self.pub_tf.sendTransform(msg.mag/np.linalg.norm(msg.magRaw),(0,0,0,1),
                 rospy.Time.now(), 
                 "/mag_raw",
                 "/cf_q")
        if np.linalg.norm(msg.mag)>1:
            self.pub_tf.sendTransform(msg.mag/np.linalg.norm(msg.mag),(0,0,0,1),
                 rospy.Time.now(), 
                 "/mag_calib",
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
        msg.zSpeed = data["hover.zSpeed"]
        msg.acc_vspeed = data["hover.acc_vspeed"]
        msg.asl_vspeed = data["hover.asl_vspeed"]
        msg.target = data["hover.target"]
        msg.zBias = data["hover.zBias"]
        print msg
        
        self.pub_hover.publish(msg)        


    def logCallbackZPid(self, data):
        msg = zpidMSG()   
        msg.header.stamp = rospy.Time.now()
        msg.p = data["zpid.p"]
        msg.i = data["zpid.i"]
        msg.d = data["zpid.d"]
        msg.pid = data["zpid.pid"]
        
        self.pub_zpid.publish(msg) 
          
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
        
    def logCallbackBaro(self, data):
        msg = baroMSG()
        msg.header.stamp = rospy.Time.now()
        msg.temp = data["baro.temp"]
        msg.pressure = data["baro.pressure"]
        msg.asl = data["baro.asl"]
        msg.aslRaw = data["baro.aslRaw"]
        msg.aslLong = data["baro.aslLong"]
        self.pub_baro.publish(msg)           
        
        #if self.baro_start_time == None:
        #    self.baro_start_time = rospy.Time.now() 
        #self.csvwriter.writerow([(msg.header.stamp-self.baro_start_time).to_sec(),msg.asl, msg.asl_raw,msg.temperature,msg.pressure])
      
                
    def logCallbackBat(self, data):
        msg = batMSG()
        msg.header.stamp = rospy.Time.now()
        msg.link = self.link
        msg.bat_v = data["pm.vbat"]
        msg.bat_p = (data["pm.vbat"]-3.0)/1.15*100.
        msg.charge_state = data["pm.state_charge"]
        msg.state = data["pm.state"]        
        self.pub_bat.publish(msg)
        self.battery_monitor.update(msg.state, msg.charge_state, msg.bat_v)

        
    def logCallbackMotor(self, data):
        msg = motorMSG()
        msg.header.stamp = rospy.Time.now()
        msg.pwm[0] = thrustToPercentage(data["motor.m1"])
        msg.pwm[1] = thrustToPercentage(data["motor.m2"])
        msg.pwm[2] = thrustToPercentage(data["motor.m3"])
        msg.pwm[3] = thrustToPercentage(data["motor.m4"])
        msg.thrust = thrustToPercentage(data["motor.thrust"])/100.
        msg.thrust_raw = data["motor.thrust"]
        msg.vLong = data["motor.vLong"]
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
        self.battery_monitor.done()
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
