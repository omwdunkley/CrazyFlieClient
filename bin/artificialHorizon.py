#!/usr/bin/env python
import sys
from PyQt4 import QtGui, QtCore
import rospy
import roslib
from math import atan2

from PyQt4.QtCore import QObject, pyqtSignal

roslib.load_manifest("crazyflie_ros")
import tf
import numpy

from crazyflie_ros.msg import mag as magMSG
from crazyflie_ros.msg import attitude as attitudeMSG

from cfclient.ui.widgets.ai import AttitudeIndicator

if __name__=="__main__":
    class ArtificialHorizon(QtGui.QWidget):

        rollSignal  = pyqtSignal(float)
        pitchSignal = pyqtSignal(float)
        yawSignal   = pyqtSignal(float)

        useMag = False

        def __init__(self):
            super(ArtificialHorizon, self).__init__()

            self.initUI()
            
            rospy.init_node('artificialHorizon', anonymous=True)
            self.subscriber = rospy.Subscriber("/cf/mag", magMSG, self.readMag, queue_size = 1 )
            self.subscriber = rospy.Subscriber("/cf/attitude", attitudeMSG, self.readAttitude, queue_size = 1 )
            
            self.rollSignal.connect(self.updateRoll)
            self.pitchSignal.connect(self.updatePitch)
            self.yawSignal.connect(self.updateYaw)
            
        def readMag(self, data):
            if data.mag[0] == 0 and data.mag[1] == 0 and data.mag[2] == 0:
                self.useMag = False
                return
            
            self.useMag = True
            r2d = 180.0 / 3.1415926535
            angle = atan2(data.mag[0], data.mag[1]) * r2d
            #self.updateYaw(angle * 180.0 / 3.1415926535)
            self.yawSignal.emit(angle)

        def readAttitude(self, data):
            q = numpy.zeros(4)
            q[0] = data.q[3]
            q[1] = data.q[0]
            q[2] = data.q[1]
            q[3] = data.q[2]
            angles = tf.transformations.euler_from_quaternion(q)
            
            r2d = 180.0 / 3.1415926535
            roll = angles[0] * r2d + 180
            self.rollSignal.emit(roll)
            self.pitchSignal.emit(-angles[1] * r2d)
            
            if self.useMag == False:
                yaw = angles[2] * r2d
                if yaw > 180:
                    yaw = yaw - 360
                elif yaw < -180:
                    yaw = yaw + 360
                self.yawSignal.emit(yaw)

        def updatePitch(self, pitch):
            self.wid.setPitch(pitch, True)

        def updateRoll(self, roll):
            self.wid.setRoll(roll, True)
            
        def updateYaw(self, yaw):
            self.wid.setYaw(yaw, True)

        def initUI(self):

            # 
            
            self.wid = AttitudeIndicator()
            hbox = QtGui.QHBoxLayout()
            hbox.addWidget(self.wid)

            # old code with sliders

#             vbox = QtGui.QVBoxLayout()
# 
#             sld = QtGui.QSlider(QtCore.Qt.Horizontal, self)
#             sld.setFocusPolicy(QtCore.Qt.NoFocus)
#             sld.setRange(0, 3600)
#             sld.setValue(1800)
#             
#             vbox.addWidget(sld)
# 
#             self.wid = AttitudeIndicator()
# 
#             sld.valueChanged[int].connect(self.updateRoll)
#             vbox.addWidget(self.wid)
# 
#             hbox = QtGui.QHBoxLayout()
#             hbox.addLayout(vbox)
#             
#             sldPitch = QtGui.QSlider(QtCore.Qt.Vertical, self)
#             sldPitch.setFocusPolicy(QtCore.Qt.NoFocus)
#             sldPitch.setRange(0, 180)
#             sldPitch.setValue(90)
#             
#             sldPitch.valueChanged[int].connect(self.updatePitch)
#             
#             hbox.addWidget(sldPitch)
#             hboxYaw = QtGui.QHBoxLayout()
#             
#             sldYaw = QtGui.QSlider(QtCore.Qt.Horizontal, self)
#             sldYaw.setFocusPolicy(QtCore.Qt.NoFocus)
#             sldYaw.setRange(0, 2*3600)
#             sldYaw.setValue(3600)
#             
#             sldYaw.valueChanged[int].connect(self.updateYaw)
#             
#             hboxYaw.addWidget(sldYaw)
#             vbox.addLayout(hboxYaw)
            
            ####
            
            self.setLayout(hbox)

            self.setGeometry(300, 300, 210, 210)
            self.setWindowTitle('Artificial Horizon')
            self.show()

        def changeValue(self, value):

            self.c.updateBW.emit(value)
            self.wid.repaint()

    def main():

        app = QtGui.QApplication(sys.argv)
        ex = ArtificialHorizon()
        sys.exit(app.exec_())


    if __name__ == '__main__':
        main()
