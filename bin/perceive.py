import message_filters
from sensor_msgs.msg import Image, CameraInfo

def callback(i, z, info):
  rospy.loginfo("-")




sub_image_i    = message_filters.Subscriber('/camera/rgb/image_rect', Image)
sub_image_z    = message_filters.Subscriber('/camera/depth_registered/image_rect', Image)
sub_image_info = message_filters.Subscriber('camera_info', CameraInfo)

ts = message_filters.TimeSynchronizer([sub_image_i, sub_image_z, sub_image_info], 10)
ts.registerCallback(callback)
rospy.spin()
