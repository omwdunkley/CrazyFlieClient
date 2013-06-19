#!/usr/bin/env python
import os.path
import numpy
import rospy
import roslib
import sys, random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PlotWindow import PlotWindow
roslib.load_manifest("crazyflie_ros")

from std_msgs.msg import Int8

from crazyflie_ros.msg import CFJoy as joyMSG

from crazyflie_ros.msg import mag as magMSG
from crazyflie_ros.msg import motor as motorMSG
from crazyflie_ros.msg import bat as batMSG
from crazyflie_ros.msg import mag_calib as magCalibMSG

class MagCalibrator(PlotWindow):
    def __init__(self):
        PlotWindow.__init__(self)
    
        self.window_size=20
        self.values=numpy.zeros((self.window_size))
        self.index=0

        self.axes2D.set_autoscaley_on(False)
        self.axes2D.set_xlim((-1, 1))
        self.axes2D.set_ylim((-1, 1))
        
        self.axes3D.set_autoscaley_on(True)

        self.current_step = 0
        self.current_data = "flat"
        self.new_calibration = False
                        
        self.data_read = { "mag" : False, "pwm" : False, "bat" : False }
        self.latest = { "mag" : numpy.zeros(3), "pwm" : numpy.zeros(1), "bat" : numpy.zeros(1) }
        self.thrust = 0
        self.all_data = { "flat" :   { "mag" : { "x": [], "y": [], "z": []}, 
                                       "thrust" : [],
                                       "bat" : [] },
                          "sphere" : { "mag" : { "x": [], "y": [], "z": []}, 
                                       "thrust" : [],
                                       "bat" : [] }, 
                          "motor"  : { "mag" : { "x": [], "y": [], "z": []}, 
                                       "thrust" : [],
                                       "bat" : [] }}
        
        ## ROS
        rospy.init_node('magCalibrator', anonymous=True)
        self.subscriber = rospy.Subscriber("/cf/mag", magMSG, self.readMag, queue_size = 1 )
        self.subscriber = rospy.Subscriber("/cf/motor", motorMSG, self.readMotor, queue_size = 1 )
        self.subscriber = rospy.Subscriber("/cf/bat", batMSG, self.readBat, queue_size = 1 )
        self.sub_joy   = rospy.Subscriber("/cfjoy", joyMSG, self.new_joydata)
        self.pub_cal   = rospy.Publisher("/magCalib", magCalibMSG)

        self._readCalibrationFromFile()

    def readMag(self, data):
        self.data_read["mag"] = True
        self.latest["mag"] = data.magRaw
        
        self.readData()

        mag = [ (data.magRaw[0] - self.cal_offset[0]) / self.cal_scale[0] - self.thrust * self.cal_comp[0],
                (data.magRaw[1] - self.cal_offset[1]) / self.cal_scale[1] - self.thrust * self.cal_comp[1],
                (data.magRaw[2] - self.cal_offset[2]) / self.cal_scale[2] - self.thrust * self.cal_comp[2] ]
        magData = [ [ 0, mag[0] ], [ 0, mag[1] ], [ 0, mag[2] ] ]
        magData2 = [ [ 0.001, data.mag[0] ], [ 0.001, data.mag[1] ], [ 0.001, data.mag[2] ] ]
        
        self.axes2D.clear();
        self.axes2D.set_autoscaley_on(False)
        self.axes2D.set_xlim((-1, 1))
        self.axes2D.set_ylim((-1, 1))
        self.axes2D.plot(magData[0], magData[1], 'b');
        self.axes2D.plot(magData2[0], magData2[1], 'r');
        
        if self.new_calibration:
            msg = magCalibMSG()
            msg.header.stamp = rospy.Time.now()
            msg.offset[0:3] = self.cal_offset[0:3]
            msg.scale[0:3] = self.cal_scale[0:3]
            msg.thrust_comp[0:3] = self.cal_comp[0:3]
            self.pub_cal.publish(msg)
            
            self.new_calibration = False
            

    def readMotor(self, data):
        self.data_read["pwm"] = True
        self.latest["pwm"] = data.thrust_raw
        self.thrust = data.thrust_raw
        
    def readBat(self, data):
        self.data_read["bat"] = True
        self.latest["bat"] = data.bat_v

    def readData(self):
        # datatype should be "mag" "pwm" or "bat"
        if self.data_read["mag"] == self.data_read["bat"] == self.data_read["pwm"] == True:
            if self.current_step > 1:
                self.all_data[self.current_data]["mag"]["x"].append(self.latest["mag"][0])
                self.all_data[self.current_data]["mag"]["y"].append(self.latest["mag"][1])
                self.all_data[self.current_data]["mag"]["z"].append(self.latest["mag"][2])
                self.all_data[self.current_data]["pwm"].append(self.latest["pwm"])
                self.all_data[self.current_data]["bat"].append(self.latest["bat"])
                
                self.data_read["mag"] = self.data_read["bat"] = self.data_read["pwm"] = False
                
                self.plotResults()

            self.canvas.draw()


    # 3D plot of calibration data
    def plotResults(self):
        self.axes3D.clear()        

        if self.index == self.window_size - 1:
            self.index = 0
        else:
            self.index = self.index + 1

        self.axes3D.plot3D(self.all_data["flat"]["mag"]["x"], 
                           self.all_data["flat"]["mag"]["y"], 
                           self.all_data["flat"]["mag"]["z"], color='b')

        self.axes3D.plot3D(self.all_data["sphere"]["mag"]["x"], 
                           self.all_data["sphere"]["mag"]["y"], 
                           self.all_data["sphere"]["mag"]["z"], color='r')
        
        self.axes3D.plot3D(self.all_data["motor"]["mag"]["x"], 
                           self.all_data["motor"]["mag"]["y"], 
                           self.all_data["motor"]["mag"]["z"], color='g')


    def new_joydata(self, joymsg):
        """Joydata arrived. Should happen at 100hz."""
        if joymsg.calib:
            self.step()

    def step(self):
        if self.current_step == 0:
            self._step_0()
        elif self.current_step == 1:
            self._step_1()
        elif self.current_step == 2:
            self._step_2()
        elif self.current_step == 3:
            self._step_3()
         
    def _step_3(self):
        # step 3: testing
        rospy.loginfo("**************************************************************")
        rospy.loginfo("Calibration Complete")
        self.current_step = 0;
                
        # output for matlab
#         f = open('calibration_data.txt', 'w')
#  
#         self._writeDataToFile(f, "flat")
#         self._writeDataToFile(f, "sphere")
#         self._writeDataToFile(f, "motor")
#          
#         f.close();
        
        if self._gaussNewton() and self._motorCompensation():
            self._writeCalibrationToFile()
            self.new_calibration = True
 
    def _writeDataToFile(self, f, data_name):
        
        data = self.all_data[data_name]
        
        f.write(data_name + "_data = struct('mag', [], 'bat', [], 'pwm', []); \n")

        f.write(data_name + "_data.bat = [ ...\n")
        f.write("  ")
        for idx, el in enumerate(data["bat"]):
            f.write( "  " + str(data["bat"][idx]) + "; ...\n" )

        f.write("  ];\n")
                                 
        f.write(data_name + "_data.mag = [ ...\n")
        f.write("  ")
        for idx, el in enumerate(data["mag"]["x"]):
            f.write( "  " + str(data["mag"]["x"][idx]) + 
                     ", " + str(data["mag"]["y"][idx]) +
                     ", " + str(data["mag"]["z"][idx]) + "; ...\n" )

        f.write("  ];\n")

        f.write(data_name + "_data.pwm = [ ...\n")
        f.write("  ")
        for idx, el in enumerate(data["pwm"]):
            f.write( "  " + str(data["pwm"][idx]) + "; ...\n" )

        f.write("  ];\n")

    def _step_2(self):
        # step 2: throttle
        rospy.loginfo("**************************************************************")
        rospy.loginfo("Hold the CrazyFlie still while varying throttle from 0 to full")
        rospy.loginfo("Rotate it and repeat 3+ times")
        rospy.loginfo("Press [] when done")
         
        self.current_step += 1;
        self.current_data = "motor"
 
    def _step_1(self):
        # step 1: free motion
        rospy.loginfo("**************************************************************")
        rospy.loginfo("Rotate CrazyFlie freely in the air")
        rospy.loginfo("Do not use throttle")
        rospy.loginfo("Try to make a complete ellipse")
        rospy.loginfo("Press [] when done")
        self.current_step += 1;
        self.current_data = "sphere"
                   
    def _step_0(self):
        # step 0: confirmation
        self.current_step += 1
         
        rospy.loginfo("**************************************************************")
        rospy.loginfo("Magnetometer Calibration")
        rospy.loginfo("")
        rospy.loginfo("Place CrazyFlie away from magnetic objects and electronics on a flat surface")
        rospy.loginfo("Press [] again to begin")
        rospy.loginfo("")

        self.all_data = { "flat" :   { "mag" : { "x": [], "y": [], "z": []}, 
                                       "pwm" : [],
                                       "bat" : [] },
                          "sphere" : { "mag" : { "x": [], "y": [], "z": []}, 
                                       "pwm" : [],
                                       "bat" : [] }, 
                          "motor"  : { "mag" : { "x": [], "y": [], "z": []}, 
                                       "pwm" : [],
                                       "bat" : [] }}

    def _writeCalibrationToFile(self):
        numpy.save('cal_offset', self.cal_offset)
        numpy.save('cal_scale', self.cal_scale)
        numpy.save('cal_comp', self.cal_comp)

    def _readCalibrationFromFile(self):
        self.new_calibration = True
        if os.path.isfile('cal_offset.npy'):
            self.cal_offset = numpy.load('cal_offset.npy')
        else:
            self.cal_offset = numpy.zeros(3)
            self.new_calibration = False
        if os.path.isfile('cal_scale.npy'):
            self.cal_scale = numpy.load('cal_scale.npy')
        else:
            self.cal_scale = numpy.zeros(3)
            self.new_calibration = False
        if os.path.isfile('cal_comp.npy'):
            self.cal_comp = numpy.load('cal_comp.npy')
        else:
            self.cal_comp = numpy.zeros(3)
            self.new_calibration = False
                
        if self.new_calibration:
            print "Calibration read from file: " 
            print "> Offset: ", self.cal_offset
            print "> Scale: ", self.cal_scale
            print "> Compensation:", self.cal_comp
            
    ### learning of parameters
    def _gaussNewton(self):
        min_error = 0.01
    
        mag_data = self.all_data["sphere"]["mag"]
        
        # number of points
        N = len(mag_data["x"])
        # magnetometer data points
        P = numpy.zeros((N, 3))
        P[0:N, 0] = mag_data["x"]
        P[0:N, 1] = mag_data["y"]
        P[0:N, 2] = mag_data["z"]
        
        # scale
        S = numpy.std(P, 0);
        # offset
        O = numpy.mean(P, 0);
        
        A = numpy.divide(P - numpy.tile(O, [N, 1]), numpy.tile(S, [N, 1]))
        # residual
        res = numpy.ones(N) - numpy.sum(numpy.power(A, 2), 1)
            
        # error
        err = sum(numpy.power(res, 2))
        
        print "err: ", err

        iter = 0
        delta = 1
        max_iterations = 100
        
        while numpy.sum(numpy.power(delta, 2)) > min_error and iter < max_iterations:
            iter += 1
            # compute Jacobian
            J = numpy.zeros((N, 6))
            J[0:N, 0:3] = 2 * numpy.divide(P - numpy.tile(O, [N, 1]), numpy.tile(numpy.power(S, 2), [N, 1]))
            J[0:N, 3:6] = 2 * numpy.divide(numpy.power(P - numpy.tile(O, [N, 1]), 2), numpy.tile(numpy.power(S, 3), [N, 1]))
            
            delta = numpy.dot(numpy.dot(numpy.linalg.inv(numpy.dot(numpy.transpose(J), J)), numpy.transpose(J)), res)
            O = O - delta[0:3]
            S = S - delta[3:6]
            
            A = numpy.divide(P - numpy.tile(O, [N, 1]), numpy.tile(S, [N, 1]))
            # residual
            res = numpy.ones(N) - numpy.sum(numpy.power(A, 2), 1)

            # error
            err = sum(numpy.power(res, 2))

        if iter == max_iterations:
            print "Calibration failed: max iterations reached"
            return False
        else:
            print "Calibration successful"
            self.cal_offset = O
            self.cal_scale = S
            return True

    def _motorCompensation(self):
        # find first use of thrust for at least a few steps
        mag_data = self.all_data["motor"]["mag"]
        
        # number of points
        N = len(mag_data["x"])
        # magnetometer data points
        P = numpy.zeros((N, 3))
        P[0:N, 0] = mag_data["x"]
        P[0:N, 1] = mag_data["y"]
        P[0:N, 2] = mag_data["z"]
        
        T = numpy.array(self.all_data["motor"]["pwm"])
        
        a = 0
        b = 0

        # thrust is something like this:
        # 0 .... 0 10 20 25 25 20 20 20 20 10 0 0 ... 0
        #        a (zero before nonzeros)   b (last non zero)

        while a < numpy.size(T) - 2:
            if T[a] == 0 and T[a + 1] > 0 and T[a + 2] > 0:
                break
            a += 1
            b = a + 1
        
        while b < numpy.size(T):
            if T[b] == 0:
                break
            b += 1
        
        if a == 0 or b == 0:
            rospy.logerr('Error: no thrust detected.')
            return False
        
        # calibrated magnetometer readings during interval [a b]        
        N = b-a
        offset = numpy.tile(self.cal_offset, [N, 1])
        scale  = numpy.tile(self.cal_scale,  [N, 1])
        Q = numpy.divide((P[a:b, 0:3] - offset), scale)

        comp = numpy.divide(Q[1:N, 0:3] - numpy.tile(Q[0, 0:3], [N - 1, 1]), numpy.transpose(numpy.tile(T[a+1:b], [3, 1])))

        self.cal_comp = numpy.mean(comp, 0)

        return True

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MagCalibrator()
    window.show()
    app.exec_()

