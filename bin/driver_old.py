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
from crazyflie_ros.msg import ollie as ollieMSG
from crazyflie_ros.msg import attitude as attitudeeMSG
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


class PID:
    """PID Controller"""
    def __init__(self, P=0.5,I=0.0,D=0.1):
        self.P = P
        self.I = I
        self.D = D
        
        # Useful for I part
        self.error_sum = 0.0;
        
        # Useful for D part
        self.last_time = rospy.Time.now()
        self.last_error = sys.float_info.max
        return
    
    def reset(self):
        self.error_sum = 0.0;
        self.last_time = rospy.Time.now()
        self.last_error = 0.0 #not max?
        
    def get_command(self, error, time):
        dt = (time-self.last_time).to_sec()
        if not dt>0:            
            rospy.logwarn("Negative time in PID controller: "+str(dt)+"s")
            return 0; #todo check
        
        P = self.P * (error);
        I = self.I * self.error_sum       
        D = self.D * ((self.last_error-error) / dt);        

        self.last_time  = time;
        self.last_error = error;
        self.error_sum  = self.error_sum + error * dt;        
        return P-I+D;
    
    def set_gains(self, P,I,D):
        self.P = P
        self.I = I
        self.D = D
        return



def thrustToPercentage( thrust):
    return ((thrust/MAX_THRUST)*100.0)

def percentageToThrust( percentage):
    return int(MAX_THRUST*(percentage/100.0))

def getRPYRad(q):
    q0,q1,q2,q3 = q

    gx = 2 * (q1 * q3 - q0 * q2);
    gy = 2 * (q0 * q1 + q2 * q3);
    gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    yaw = atan2(2 * q1 * q2 - 2 * q0 * q3, 2 * q0 * q0 + 2 * q1 * q1 - 1)
    pitch = atan(gx / sqrt(gy * gy + gz * gz))
    roll = atan(gy / sqrt(gx * gx + gz * gz))
    return (roll, pitch, yaw)

def invSqrt(x):
    return x**-1/2


def deadband(value, threshold):
    if abs(value)<threshold:
        return 0.0
    elif value>0:
        return value-threshold
    else:
        return value+threshold


class Mahony:
    def __init__(self):
        self.twoKp = 2. * 0.5 #proportional gain
        self.twoKi = 2. * 0.0 #integral gain
        self.freq = 100. #hz
        self.q = (1.0,0.0,0.0,0.0) #quat
        self.integralFB = [0.0,0.0,0.0] # integral error terms scaled by ki

    
      
    def mahonyAHRSupdate(self, g,a,m):
        gx,gy,gz = g
        ax,ay,az = a
        mx,my,mz = m
        
        recipNorm = 0.0;
        q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3 = 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0
        hx, hy, bx, bz = 0.0, 0.0, 0.0, 0.0
        halfvx, halfvy, halfvz, halfwx, halfwy, halfwz = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        halfex, halfey, halfez = 0.0,0.0,0.0
        qa, qb, qc = 0.0,0.0,0.0
        q0,q1,q2,q3 = self.q
        
        if ax==0.0 and ay==0.0 and az==0.0:
            pass
        else:
            # Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;     
    
            # Normalise magnetometer measurement
            recipNorm = invSqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm
            my *= recipNorm
            mz *= recipNorm;
    
            # Auxiliary variables to avoid repeated arithmetic
            q0q0 = q0 * q0
            q0q1 = q0 * q1
            q0q2 = q0 * q2
            q0q3 = q0 * q3
            q1q1 = q1 * q1
            q1q2 = q1 * q2
            q1q3 = q1 * q3
            q2q2 = q2 * q2
            q2q3 = q2 * q3
            q3q3 = q3 * q3   
    
            # Reference direction of Earth's magnetic field
            hx = 2.0 * (mx * (0.5 - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2))
            hy = 2.0 * (mx * (q1q2 + q0q3) + my * (0.5 - q1q1 - q3q3) + mz * (q2q3 - q0q1))
            bx = sqrt(hx * hx + hy * hy);
            bz = 2.0 * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5 - q1q1 - q2q2))
    
            # Estimated direction of gravity and magnetic field
            halfvx = q1q3 - q0q2
            halfvy = q0q1 + q2q3
            halfvz = q0q0 - 0.5 + q3q3
            halfwx = bx * (0.5 - q2q2 - q3q3) + bz * (q1q3 - q0q2)
            halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3)
            halfwz = bx * (q0q2 + q1q3) + bz * (0.5 - q1q1 - q2q2)  
        
            # Error is sum of cross product between estimated direction and measured direction of field vectors
            halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy)
            halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz)
            halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx)
    
            # Compute and apply integral feedback if enabled
            if self.twoKi > 0.0:
                self.integralFB[0] += self.twoKi * halfex * (1.0 / self.freq)    # integral error scaled by Ki
                self.integralFB[1] += self.twoKi * halfey * (1.0 / self.freq)
                self.integralFB[2] += self.twoKi * halfez * (1.0 / self.freq)
                gx += self.integralFB[0]    # apply integral feedback
                gy += self.integralFB[1]
                gz += self.integralFB[2]
            
            else:
                self.integralFB[0] = 0.0    # prevent integral windup
                self.integralFB[1] = 0.0
                self.integralFB[2] = 0.0
            
    
            # Apply proportional feedback
            gx += self.twoKp * halfex;
            gy += self.twoKp * halfey;
            gz += self.twoKp * halfez;
        
        # Integrate rate of change of quaternion
        gx *= (0.5 * (1.0 / self.freq))         # pre-multiply common factors
        gy *= (0.5 * (1.0 / self.freq))
        gz *= (0.5 * (1.0 / self.freq))
        qa = q0
        qb = q1
        qc = q2
        q0 += (-qb * gx - qc * gy - q3 * gz)
        q1 += (qa * gx + qc * gz - q3 * gy)
        q2 += (qa * gy - qb * gz + q3 * gx)
        q3 += (qa * gz + qb * gy - qc * gx) 
        
        # Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        q0 *= recipNorm
        q1 *= recipNorm
        q2 *= recipNorm
        q3 *= recipNorm
        
        self.q = (q0,q1,q2,q3)
        return self.q
            
                
        
        

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
        
        
    def released(self, id):
        return self.prev_cmd.buttons[id] and not self.curr_cmd.buttons[id]
    def pressed(self, id):
        return not self.prev_cmd.buttons[id] and self.curr_cmd.buttons[id]
    def held(self, id):
        return self.prev_cmd.buttons[id] and self.curr_cmd.buttons[id]    
        
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
            return (0,0,0,0,False,0)
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
            hover = self.curr_cmd.axes[Axes.L1]<-0.6
        
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
            
        # Cache prev joy command
        self.prev_cmd = self.curr_cmd
        
        quad_cmd = (trimmed_roll,trimmed_pitch,yaw,thrust,hover, z)
   
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


class kalman1d:
    def __init__(self,q=0.125,r=10,p=1.056,k=0.105):
        self.q = q # process noise covariance
        self.r = r # measurement noise covariance
        self.x = None # value
        self.p = p # estimation error covariance
        self.k = k # kalman gain
    def update(self, measurement, prediction=None):
        if self.x == None:
            self.x = measurement    
        else:
            # prediction update
            # omit x = x
            if prediction!=None:
                self.x = self.x+prediction
            self.p +=self.q;
                    
            # measurement update
            self.k = self.p / (self.p + self.r);
            self.x = self.x + self.k * (measurement - self.x);
            self.p = (1 - self.k) * self.p;
        return self.x
    
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



class VCon:
    def __init__(self):
        
        self.alt_avg = Avg3(5)
        self.alt_avg = Avg3(5)
        self.acc_avg = Avg3(10)
        self.mag_avg = Avg3(10)
        self.gyro_avg = Avg3(10)
        self.q = (1.0,0.0,0.0,0.0)
        self.acc_offset = LPF(0.98, np.array([0.0,0.0,0.0]))
        #self.speed_z_baro_1 = Avg3(50)#LPF(0.95, 0.0) #short term
        #self.speed_z_baro_2 = Avg3(100)#LPF(0.75, 0.0) #long term
        self.speed_z_baro_1 = LPF(0.04, 0.0) #short term
        self.speed_z_baro_2 = LPF(0.09, 0.0) #long term
        
        self.mahony = Mahony()
        
        self.pub_plot   = rospy.Publisher("/cf/plot", plotMSG)
        self.pub_accg   = rospy.Publisher("/accg", accMSG)   
        self.pub_acc   = rospy.Publisher("/acc", accMSG)      
        self.pub_tf    = tf.TransformBroadcaster()  
        
        self.speed_z = 0    
        self.z = 0
        self.z2 = 0
        
        self.alpha = 0.98
        
        self.p = 1.0
        self.i = 1.0
        self.d = 1.0
        self.world_acc_offset = np.array((0.0,0.0,0.0))
        
        self.altitude_kf = kalman1d()
        
        self.est_alt = 0    
        self.vz_est = 0
        self.h_est = 0
        self.k_h_est = -0.0001
        self.k_vz = -0.008
        self.agl_r_pre = 0.0
        self.rate = 0
#         self.inited    = 0.
#         self.AltErrorI = 0.
#         self.EstVelocity = 0.
#         self.EstAlt = 0
#         self.kp1 = 0.55 
#         self.kp2 = 1.00
        
    def update_acc_off(self, g_raw, a_raw, gv, a_comp):
        a_ok = True #abs(np.linalg.norm(a_raw) - 1.0)<0.08
        g_ok = max(abs(g_raw)) < 0.015
        gv_ok = True # gv[2] >0.996
        if a_ok and g_ok and gv_ok:
            self.acc_offset.update(a_comp)
        return self.acc_offset.get_value()
        
        
    def reconfigure(self, config, level):  
        self.alt_avg.set_len(config["alt_win"])
        self.alpha = config["pressure_smooth"]  
        self.k_h_est = config["k_h_est"]  
        self.k_vz = config["k_vz"]    
        
        self.altitude_kf.k = config["kalman_gain"] 
        self.altitude_kf.p = config["estimation_noise_cov"]
        self.altitude_kf.q = config["process_noise_cov"]
        self.altitude_kf.r = config["measurement_noise_cov"]
        
        #self.speed_z_baro_1.set_alpha(config["P"])
        #self.speed_z_baro_2.set_alpha(config["I"])
        self.p = config["P"]
        self.i = config["I"]
        self.d = config["D"]
        
              
        if config["reset_baro"]: 
            self.vz_est = 0.0
            self.h_est = 0.0
            self.altitude_kf.x=0.0
            self.agl_r_pre = 0.0
            self.z = 0.
            self.z2 = 0.
            self.rate = 0
            self.speed_z_baro_1.reset()
            self.speed_z_baro_2.reset()
            self.alt_avg.set_len(config["alt_win"])
            #self.acc_avg.zero_calib()
            #self.gyro_avg.zero_calib()
        return config
    
    def update_gyro(self, gyro):
        self.gyro_avg.add(gyro)  
        
    def update_mag(self, mag):
        self.mag_avg.add(mag)      
                    
    def update_alt(self, alt):        
        self.alt_avg.add(alt)            
        
    def update_acc(self, acc):        
        self.acc_avg.add(acc)  
        
    def update_q(self,q):
        self.q = q
        
        
    def get_grav_vec(self, q):
         g0 = 2. * (q[1] * q[3] - q[0] * q[2]);
         g1 = 2. * (q[0] * q[1] + q[2] * q[3]);
         g2 = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
         return np.array((g0,g1,g2))        
     
    def q_mult(self,q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
        return w, x, y, z
    
     
    def q_conjugate(self, q):
        #q = normalize(q)
        w, x, y, z = q
        return (w, -x, -y, -z)
     
    def qv_mult(self, q1, v1):
        #v1 = v1/np.linalg.norm(v1)
        x,y,z = v1
        q2 = (0.0, x,y,z)
        return np.array(self.q_mult(self.q_mult(q1, q2), self.q_conjugate(q1))[1:]) 
    
    def deadzone(self,value,thresh):
        if abs(value)<thresh:
            return 0
        else:
            return value
        
    
                  
        
    #Returns a throttle scaler between min and max
    def get_q(self):
        #print len(self.gyro_avg.samples), len(self.acc_avg.samples), len(self.mag_avg.samples)  
        #if len(self.gyro_avg.samples)>3 and len(self.acc_avg.samples)>3 and len(self.mag_avg.samples)>3 and len(self.alt_avg.samples)>3:
       
        if len(self.acc_avg.samples)>=3 and len(self.alt_avg.samples)>=3 and len(self.gyro_avg.samples)>3:
            # Filtered
            a_f = self.acc_avg.get_mean()
            g_f = self.gyro_avg.get_mean()
            #m_f = self.mag_avg.get_mean()
            agl_f = self.alt_avg.get_mean()-0.6
            # Raw
            a_r = self.acc_avg.get_mean(0)
            g_r = self.gyro_avg.get_mean(0)
            #m_r = self.mag_avg.get_mean(0)
            agl_r = self.alt_avg.get_mean(0) -0.6
            
            self.speed_z_baro_1.update(agl_r)
            self.speed_z_baro_2.update(agl_r)
            #self.speed_z_baro_1.add(agl_r)
            #self.speed_z_baro_2.add(agl_r)
            

            
            
            #9d update            
#             q0,q1,q2,q3 = self.mahony.mahonyAHRSupdate(g_f, a_f, m_f) 
#             q_tf = (q1,q2,q3,q0)
#             q = (q0,q1,q2,q3)
#             self.pub_tf.sendTransform((0.5, 0, 0),q_tf, rospy.Time.now(), "/cf_mahony","/world",)   

            # Get gravity compensated acceleration
            gv = self.get_grav_vec(self.q)
            
            #gv = np.array((0,0,1.0))
            #gv /= np.linalg.norm(gv)
            #gv += np.array((self.p, self.i, self.d))-np.array([1.0,1.0,1.0])
            
            a_comp = a_r-gv
            # update offset if still on flat surface
            # Ideadly we would use the fact that there are no motor commands here either
            #offset = self.update_acc_off(g_f, a_f, gv, a_comp)
            #print offset            
            #a_comp -= offset 
            
            
            
           
            #self.pub_tf.sendTransform(a_comp, (0,0,0,1), rospy.Time.now(), "/acc2", "/cf_q")

            
                
            dt = 0.01
            world_acc = self.qv_mult(self.q, a_comp)
            
            if max(np.absolute(g_r)) < 0.015:
                self.world_acc_offset = self.world_acc_offset * 0.98 + world_acc * 0.02
            
            world_acc -= self.world_acc_offset
                        
            acc_z = world_acc[2]
            #acc_z = self.deadzone(acc_z, 0.003)            
            self.speed_z += acc_z*dt
            #self.speed_z *= 0.9

            
            #nr = max(0.2,1-max(0,min(1,(abs(acc_z)-0.03) / 0.3)))            
 
 

             
#             self.est_alt = (1-self.alpha)*(self.est_alt + self.speed_z) + (self.alpha)*(agl_f)
    
                        
            #self.vz_est = self.vz_est + acc_z        * dt * 0.99
            #self.h_est  = self.h_est  + self.vz_est  * dt
            #self.vz_est = self.vz_est + self.k_vz    * (self.h_est - agl_r)
            #self.h_est  = self.h_est  + self.k_h_est * (self.h_est - agl_r)
            

            #self.altitude_kf.update(agl_r,self.speed_z*dt)
            #agl_kf = self.altitude_kf.x  

#             if self.inited!=1:
#                 self.EstAlt = agl_f
# 
#             # Estimation Error
#             AltError = agl_f - self.EstAlt;
#             self.EstAlt = agl_f
#             self.AltErrorI += AltError;
#             self.AltErrorI = max(-25000,min(25000,self.AltErrorI))
#             # Gravity vector correction and projection to the local Z
#             InstAcc = acc_z +  self.AltErrorI / 10.;
#             
#             # Integrators            
#             Delta = InstAcc * dt + (self.kp1 * dt) * AltError;
#             self.EstAlt += (self.EstVelocity/5. + Delta) * (dt / 2.) + (self.kp2 * dt) * AltError;
#             self.EstVelocity += Delta*10.;
# 

            #self.z += self.speed_z*dt         
            
            #self.z2 += self.speed_z*dt
            #self.z2 = self.z2*self.alpha + agl_r*(1.-self.alpha)
            
            #alpha = 1.-max(0.000001,min(0.999999,abs(self.speed_z)/self.alpha*2.5))
            alpha = abs(self.speed_z*self.alpha)
        
            #self.z += self.speed_z*dt
            #self.z = self.z2*alpha + agl_f*(1.-alpha)
            #self.z2 = self.z2*alpha + self.speed_z_baro_1.get_value()*(1.-alpha)
            
            bdiff = self.speed_z_baro_2.get_value()-self.speed_z_baro_1.get_value()
            
            
            #print alpha, bdiff, alpha/bdiff, bdiff/alpha
            
            
            #self.z = self.z*alpha + (self.z+bdiff)*(1.-alpha)
            #if abs(self.speed_z)>0.005:
            self.z = agl_f#(self.z*0.7) + ((self.z + ((bdiff*self.alpha) + (self.speed_z*(1-self.alpha)))) * 0.3)
           # else:
            #    self.z = self.z*0.99 + (self.z+bdiff)*(0.01)
                
            #self.speed_z *= 0.95
                
          #  else:
          #      self.z = self.z
            
            #self.z = self.z*alpha + agl_r*(1.-alpha) 
           
            #print "SUM:", sum(world_acc)
            #print "ABS SUM", sum(np.abs(world_acc))
            
            
            msg = plotMSG()
            msg.stuff[0] = world_acc[0]
            msg.stuff[1] = world_acc[1]
            msg.stuff[2] = world_acc[2]
#             #msg.stuff[1] = agl_f            
#             msg.stuff[2] = agl_r
#             #msg.stuff[3] = self.vz_est
#             #msg.stuff[4] = self.h_est
#              
#             msg.stuff[3] = self.speed_z_baro_1.get_value()
#             msg.stuff[4] = self.speed_z_baro_2.get_value()
#             #msg.stuff[3] = self.speed_z_baro_1.get_mean()#self.speed_z_baro_1.get_value()
#             #msg.stuff[4] = self.speed_z_baro_2.get_mean()#self.speed_z_baro_2.get_value()            
#             msg.stuff[5] = bdiff
#              
#             #msg.stuff[3] = self.z
#             #msg.stuff[6] = self.z2
#             msg.stuff[7] = self.z
#             #msg.stuff[5] = self.rate
#             #msg.cmd = 1-alpha
#             #msg.stuff[3] = agl_kf
#              
#             #msg.stuff[3] = self.h_est
#             #msg.stuff[4] = self.vz_est
            self.pub_plot.publish(msg)
        return self.z
        
              
            

         
    
def get_heading(y,x):
    heading = atan2(y,x);
    declinationAngle = 42.76 / 1000. #mradians / 1000 = radians
    heading += declinationAngle
    
    if heading < 0:
        heading += 2*PI;
    
    # Check for wrap due to addition of declination.
    if heading > 2*PI:
        heading -= 2*PI;
    
    # Convert radians to degrees for readability.
    headingDegrees = degrees(heading)
    return round( headingDegrees)



class Driver:

    def __init__(self, options):
        self.options = options        
        self.crazyflie = Crazyflie()
        cflib.crtp.init_drivers()
        
        self.joy_controller = JoyController() 
        
        self.HZ100 = 10
        self.HZ10 = 100
        self.HZ1 = 1000
        
        self.HZ500 = 2
        self.HZ250 = 4
        self.HZ125 = 8
        self.HZ50 = 20

        self.vertical_controller = VCon()
                
        self.crazyflie.connectSetupFinished.add_callback(self.connectSetupFinishedCB)
        self.crazyflie.connectionFailed.add_callback(self.connectionFailedCB)
        self.crazyflie.connectionInitiated.add_callback(self.connectionInitiatedCB)
        self.crazyflie.disconnected.add_callback(self.disconnectedCB)
        self.crazyflie.connectionLost.add_callback(self.connectionLostCB)
        self.crazyflie.linkQuality.add_callback(self.linkQualityCB)   
        
        self.pub_acc   = rospy.Publisher("/cf/acc", accMSG)
        self.pub_mag   = rospy.Publisher("/cf/mag", magMSG)
        self.pub_gyro  = rospy.Publisher("/cf/gyro", gyroMSG)
        self.pub_baro  = rospy.Publisher("/cf/baro", baroMSG)
        self.pub_motor = rospy.Publisher("/cf/motor", motorMSG)
        #self.pub_RPYT  = rospy.Publisher("/cf/rpyt", rpytMSG)
        self.pub_bat   = rospy.Publisher("/cf/bat", batMSG)
        #self.pub_ollie   = rospy.Publisher("/cf/ollie", ollieMSG)
        self.pub_pid   = rospy.Publisher("/cf/pid", plotMSG)                
        
        self.pub_tf    = tf.TransformBroadcaster()   
        self.sub_tf    = tf.TransformListener()         
        self.sub_joy   = rospy.Subscriber("/joy", Joy, self.new_joydata)
        
        
        # Keep track of link quality to send out with battery status
        self.link = 0
        
        self.logMotor = None
        self.logBaro = None
        self.logMag = None
        self.logGyro = None
        self.logOllie = None
        self.logAcc = None
        self.logBat = None
        self.logRPYT = None  
        self.logGravOffset = None
        
        self.acc_monitor = False
        self.gyro_monitor = False
        self.baro_monitor = False
        self.mag_monitor = False
        self.bat_monitor = False
        self.rpyt_monitor = False
        self.motor_monitor = False
        
        self.hover_on = False
        self.hover_target = 0
        
        self.use_9d = False
        
        self.baro_calibration = 0
        self.baro_accum = []
        self.pid = PID(1.0, 0, 0.0)
        self.fac = Avg3(5)
        self.err = 0.0
        
        self.dynserver = DynamicReconfigureServer(driverCFG, self.reconfigure)
        self.joy_controller.set_dynserver(self.dynserver)
           
        
        self.connect(self.options)
        
    def connect(self, options):                               
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
        
        roll, pitch, yawrate, thrust, hover, hover_change  =self.joy_controller.get_control(joymsg)   
        
        alt = self.vertical_controller.get_q()
        
        set_hover = False
        if not self.hover_on and hover:
            self.hover_on = True
            self.hover_target = alt
            set_hover = True
            
        
        if hover:            
             msg = plotMSG()
             old_thrust = thrust
             msg.stuff[0] = thrustToPercentage(thrust)/100.
             msg.stuff[1] = alt
             msg.stuff[2] = self.hover_target
             
             self.err =  (self.hover_target-alt)*0.3 + self.err * 0.7            
             cmd =  self.pid.get_command(self.err, rospy.Time.now())
             
             
             
             #cmd = self.pid.get_command(self.err, rospy.Time.now())
             
             
             #msg.stuff[5] = self.err
             self.fac.add(cmd)
             fac = self.fac.get_mean()
             msg.stuff[3] = fac
             fac +=1.
             #print "Thrust multi:",round(fac*100)
             
             
             if thrust>15000:
                 thrust = max(thrust-5500, min(thrust * fac, min(60000,thrust+3500)))
             else:
                 thrust = min(thrust * fac, min(60000,thrust+3500))
             #print "-->", thrust
             
             
             #print "ALT:",alt, "TARGET", self.hover_target,"ERR:", round(self.err,3),"FAC:",fac-1,"THR:", round(thrustToPercentage(old_thrust),1),"->", round(thrustToPercentage(thrust),1), " = ", round(old_thrust) ,"->", round(thrust)
             
             
             
             msg.stuff[4] = thrustToPercentage(thrust)/100.
             self.pub_pid.publish(msg)
            
        else:
            self.hover_on = False
            self.pid.reset()
            self.fac.samples.clear()
            self.fac.add(0.)
            self.err = 0.
            
        #thrust = 0
        self.send_control((roll, pitch, yawrate, thrust, self.hover_on, set_hover, hover_change))
        

    def reconfigure(self, config, level):
        # Fill in local variables with values received from dynamic reconfigure clients (typically the GUI).
        
        if self.gyro_monitor != config["read_gyro"]:
            self.gyro_monitor = config["read_gyro"]           
            if self.logGyro != None:
                if self.gyro_monitor:               
                    self.logGyro.start()
                    rospy.loginfo("Gyro Logging started")
                else:   
                    self.logGyro.stop()
                    rospy.loginfo("Gyro Logging stopped")
                    
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
                    
        if self.rpyt_monitor != config["read_rpyt"]:
            self.rpyt_monitor = config["read_rpyt"]           
            if self.logRPYT != None:
                if self.rpyt_monitor:               
                    self.logRPYT.start()
                    rospy.loginfo("Role/Pitch/Yaw/Thrust Logging started")
                else:   
                    self.logRPYT.stop()
                    rospy.loginfo("Role/Pitch/Yaw/Thrust Logging stopped")
                                                                                                                        
        if self.motor_monitor != config["read_motor"]:
            self.motor_monitor = config["read_motor"]           
            if self.logMotor != None:
                if self.motor_monitor:               
                    self.logMotor.start()
                    rospy.loginfo("Motor Logging started")
                else:   
                    self.logMotor.stop()
                    rospy.loginfo("Motor Logging stopped")                                                                                                                        

        config = self.vertical_controller.reconfigure(config, level)
        config = self.joy_controller.reconfigure(config, level)      
        
        if self.use_9d != config["use_9d"]:
            self.crazyflie.param.setParamValue("use_9d.use_9d", config["use_9d"])   
            self.use_9d = config["use_9d"];  
            
                
        if config["reset_baro"]:
            config["reset_baro"] = False
            self.baro_accum = []
            self.baro_calibration = 0
        
        self.pid.set_gains(config["P"], config["I"], config["D"])
             
            
        return config
         
    def restart(self):
        rospy.loginfo("Restarting Driver")
        rospy.sleep(1)
        self.connect(self.options)         
        
    def logErrorCB(self, errmsg):
        rospy.logerr("Log error: %s", errmsg)
        
    def send_control(self,cmd):
        """ Roll, pitch in deg, yaw in deg/s, thrust in 10000-60000 """
        roll, pitch, yawrate, thrust, hover, set_hover, hover_change = cmd    
        if self.crazyflie.state == CFState.CONNECTED:    
            self.crazyflie.commander.send_setpoint(roll, pitch, yawrate, thrust, hover, set_hover, hover_change)
        
        
    def setup_log(self):
        
        """ Console callbacks """
        self.crazyflie.console.receivedChar.add_callback(self.consoleCB)
 
        rospy.sleep(0.25)
        
        """ OLLIE LOGGING @ 100hz """
        logconf = LogConfig("stabilizer_q", self.HZ100) #ms
        logconf.addVariable(LogVariable("stabilizer_q.q0", "float"))
        logconf.addVariable(LogVariable("stabilizer_q.q1", "float"))
        logconf.addVariable(LogVariable("stabilizer_q.q2", "float"))
        logconf.addVariable(LogVariable("stabilizer_q.q3", "float"))   
           
    
        self.logOllie = self.crazyflie.log.create_log_packet(logconf)  
        if (self.logOllie is not None):
            self.logOllie.dataReceived.add_callback(self.logCallbackOllie)
            self.logOllie.error.add_callback(self.logErrorCB)
            if True:
                self.logOllie.start()
                rospy.loginfo("Ollie Logging started")     
        else:
            rospy.logwarn("Could not setup Ollie logging!")   
            
        rospy.sleep(0.25)
        
       
        
         
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
        logconf.addVariable(LogVariable("baro.target", "float"))                  
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
          
        """ ROLL/PITCH/YAW/THRUST LOGGING @ 100hz"""  
        logconf = LogConfig("LoggingRPYT", self.HZ100) #ms
        logconf.addVariable(LogVariable("stabilizer.thrust", "int16_t"))
        logconf.addVariable(LogVariable("stabilizer.yaw", "float"))
        logconf.addVariable(LogVariable("stabilizer.roll", "float"))
        logconf.addVariable(LogVariable("stabilizer.pitch", "float"))
        self.logRPYT = self.crazyflie.log.create_log_packet(logconf)  
        if (self.logRPYT is not None):
            self.logRPYT.dataReceived.add_callback(self.logCallbackRPYT)
            self.logRPYT.error.add_callback(self.logErrorCB)
            if self.rpyt_monitor:
                self.logRPYT.start()
                rospy.loginfo("RPYT Logging started")     
        else:
            rospy.logwarn("Could not setup RPYT logging!")
            
        rospy.sleep(0.25)
            
        """ MOTOR LOGGING @ 100hz"""
        logconf = LogConfig("LoggingMotor", self.HZ100) #ms
        logconf.addVariable(LogVariable("motor.m1", "uint32_t")) 
        logconf.addVariable(LogVariable("motor.m2", "uint32_t"))
        logconf.addVariable(LogVariable("motor.m3", "uint32_t"))
        logconf.addVariable(LogVariable("motor.m4", "uint32_t"))  
        logconf.addVariable(LogVariable("motor.thrust", "int16_t"))  
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
        self.vertical_controller.update_acc(msg.acc)
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
        self.vertical_controller.update_mag(msg.mag)      
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
        self.vertical_controller.update_gyro(msg.gyro)
        self.pub_gyro.publish(msg)
        
    def logCallbackHover(self, data):    
        """Gyro axis data in deg/s -> rad/s"""       
        msg = gyroMSG()   
        msg.header.stamp = rospy.Time.now()
        msg.gyro[0] = radians(data["gyro.x"])
        msg.gyro[1] = radians(data["gyro.y"])
        msg.gyro[2] = radians(data["gyro.z"])
        self.vertical_controller.update_gyro(msg.gyro)
        self.pub_gyro.publish(msg)        

    def logCallbackOllie(self, data):
        msg = ollieMSG()   
        msg.header.stamp = rospy.Time.now()
        q0 = data["stabilizer_q.q0"]
        q1 = data["stabilizer_q.q1"]
        q2 = data["stabilizer_q.q2"]
        q3 = data["stabilizer_q.q3"]
        self.vertical_controller.update_q((q0,q1,q2,q3))
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
        msg.temp = data["baro.temp"]
        msg.pressure = data["baro.pressure"]
        msg.agl = data["baro.asl"]# ((pow((1015.7 / msg.pressure), 1/5.257) - 1.0) * (25. + 273.15)) / 0.0065#data["baro.asl"]*2.
        msg.asl_target = data["baro.target"]
        msg.asl_raw = data["baro.asl_raw"]
        self.vertical_controller.update_alt(msg.agl)
        self.pub_baro.publish(msg)     
        
#         if msg.pressure > 0:            
#             if self.baro_calibration>0:     
#                 msg.agl -= self.baro_calibration 
#                 self.pub_baro.publish(msg)             
#                 self.vertical_controller.update_alt(msg.agl) 
#             else:
#                 self.baro_accum.append(msg.agl)
#                 if len(self.baro_accum)> 150:
#                     self.baro_calibration = np.mean(self.baro_accum[-100:])
#                     rospy.loginfo("Barometer calibrated")
                    

                
                
    def logCallbackBat(self, data):
        msg = batMSG()
        msg.header.stamp = rospy.Time.now()
        msg.link = self.link
        msg.bat_v = data["pm.vbat"]
        msg.bat_p = (data["pm.vbat"]-3.0)/1.15*100.
        msg.charge_state = data["pm.state_charge"]
        msg.state = data["pm.state"]
        self.pub_bat.publish(msg)
                 
    def logCallbackRPYT(self, data):
        msg = rpytMSG()
        msg.header.stamp = rospy.Time.now()
        msg.rpy[0] = radians(data["stabilizer.roll"])
        msg.rpy[1] = radians(data["stabilizer.pitch"])
        msg.rpy[2] = radians(data["stabilizer.yaw"])
        msg.thrust = thrustToPercentage(data["stabilizer.thrust"])
        self.pub_RPYT.publish(msg)
      
        
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
