#!/usr/bin/env python
import sys
from PyQt4 import QtGui, QtCore
import rospy
import roslib
from math import atan2, degrees, radians

from PyQt4.QtCore import QObject, pyqtSignal

roslib.load_manifest("crazyflie_ros")
import tf
import numpy

from crazyflie_ros.msg import mag as magMSG
from crazyflie_ros.msg import attitude as attitudeMSG

from cfclient.ui.widgets.ai import AttitudeIndicator


class ArtificialHorizon(QtGui.QWidget):

    yawSignal   = pyqtSignal(float)
    rollPitchYawSignal = pyqtSignal(float, float, float)
    rollPitchSignal = pyqtSignal(float, float)

    useMag = False

    def __init__(self):
        super(ArtificialHorizon, self).__init__()

        self.initUI()
        
        rospy.init_node('artificialHorizon', anonymous=True)
        self.subscriber = rospy.Subscriber("/cf/mag", magMSG, self.readMag, queue_size = 1 )
        self.subscriber = rospy.Subscriber("/cf/attitude", attitudeMSG, self.readAttitude, queue_size = 1 )
        
        self.yawSignal.connect(self.wid.setYaw)
        self.rollPitchYawSignal.connect(self.wid.setRollPitchYaw)
        self.rollPitchSignal.connect(self.wid.setRollPitch)
        
    def readMag(self, data):
        if data.mag[0] == 0 and data.mag[1] == 0 and data.mag[2] == 0:
            self.useMag = False
            return
        
        self.useMag = True
        angle = degrees(atan2(data.mag[0], data.mag[1]))
        self.yawSignal.emit(angle)

    def readAttitude(self, data):
        x,y,z,w = data.q
        angles = tf.transformations.euler_from_quaternion((w,x,y,z))
        
        
        roll = degrees(angles[0]) + 180.
        pitch = degrees(-angles[1])
        
        if self.useMag == False:
            yaw = degrees(angles[2])
            if yaw > 180:
                yaw = yaw - 360
            elif yaw < -180:
                yaw = yaw + 360
            self.rollPitchYawSignal.emit(roll, pitch, yaw)
        else:
            self.rollPitchSignal.emit(roll, pitch)
           

    def initUI(self):
        self.wid = AttitudeIndicator()
        hbox = QtGui.QHBoxLayout()
        hbox.addWidget(self.wid)        
        self.setLayout(hbox)
        self.setGeometry(300, 300, 210, 210)
        self.setWindowTitle('Artificial Horizon')
        self.show()



def main():

    app = QtGui.QApplication(sys.argv)
    ex = ArtificialHorizon()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
