# object detection using PointCloud data
#PointCloud data is filtered and downsampled using voxel grid

#!/usr/bin/env python
import numpy as np
import sys
import rospy
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Int32
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError

class obst_detector:
  def __init__(self):
    self.bridge = CvBridge()
    self.obstacle_pub = rospy.Publisher("/obstacle/warning",Int32, queue_size=1)
    self.image_sub = rospy.Subscriber("/voxel_grid/output", PointCloud2,self.obstacle_callback, queue_size=1)

  def obstacle_callback(self,data):
    gen = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z"))
    for P in gen:
      #print (P)
      x = P[0]
      print (x)
      switch=0.85;
      if x > switch:
       if len (data.data) > 10:
         self.obstacle_pub.publish(Int32(switch))
         print (switch)


def main(args):
  rospy.init_node('obst_detector', anonymous=True)
  ic = obst_detector()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
