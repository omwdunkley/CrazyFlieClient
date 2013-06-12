import roslib; roslib.load_manifest("crazyflie_ros")
import rospy



import time
from threading import Thread
 
import cflib.crtp
from cflib.crazyflie import Crazyflie




 
class Driver:
 
    # Initial values, you can use these to set trim etc.
    roll = 0.0
    pitch = 0.0
    yawrate = 0
    thrust = 10001
 
    def __init__(self, options=None):
        self.crazyflie = Crazyflie()
        cflib.crtp.init_drivers()
        
        rospy.loginfo("Waiting for crazyflie...")        
        while 1:
            interfaces = cflib.crtp.scan_interfaces()
            if len(interfaces)>1:
                radio = interfaces[0][0]
                rospy.loginfo("Found: " + radio)  
                self.crazyflie.open_link(radio)
                break
            else:
                rospy.sleep(1)
              
 
 
        self.crazyflie.connectSetupFinished.add_callback(self.connectSetupFinished)
 
    def connectSetupFinished(self, linkURI):
        rospy.loginfo("Connected to " + linkURI)
#        
#        # Keep the commands alive so the firmware kill-switch doesn't kick in.
#        Thread(target=self.pulse_command).start()
# 
#        while 1:
#            self.thrust = int(raw_input("Set thrust (10001-60000):"))
# 
#            if self.thrust == 0:
#                self.crazyflie.close_link()
#                break
#            elif self.thrust <= 10000:
#                self.thrust = 10001
#            elif self.thrust > 60000:
#                self.thrust = 60000
# 
#    def pulse_command(self):
#        self.crazyflie.commander.send_setpoint(self.roll, self.pitch, self.yawrate, self.thrust)
#        time.sleep(0.01)
# 
#        # ..and go again!
#        self.pulse_command()
 

def run(args=None):    
    parser = OptionParser(description='Crazyflie ROS Driver')
    parser.add_option('--ipython', '-i',
                        action="store_true",
                        dest='ipython',
                        default=False,
                        help='If true will drop into the ipython interpreter')
    (options, leftargs) = parser.parse_args(args=args)
        
    #Load some DB depending on options 
    rospy.init_node('CrazyFlieDriver')   
    driver = Driver(options)

    #START NODE
    rospy.loginfo('Spinning '+__file__+' thread')
    try:
       rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('...exiting due to keyboard interrupt')


if __name__ == "__main__":   
    run(sys.argv)
