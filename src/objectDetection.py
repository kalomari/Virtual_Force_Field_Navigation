# An easy way to detect obstacle in depth image - window

#!/usr/bin/env python
import numpy as np
import sys
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class obst_detector:
  def __init__(self):
    self.bridge = CvBridge()
    self.obstacle_pub = rospy.Publisher("/obstacle/warning",Int32, queue_size=1)
    self.image_sub = rospy.Subscriber("/sensors/camera/depth/image_rect_raw", Image,self.obstacle_callback, queue_size=1)

  def obstacle_callback(self,data):
    #print("hello")
    try:
      depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
    except CvBridgeError as e:
      print(e)
    depth_array = np.array(depth_image, dtype=np.uint16)
    height, width = depth_array.shape[0:2]
    
    curr_lane = 1
    switch = 950
    #depth_mean=0
    #pix_num=0
    
    #print (depth_array[height//2,width//2])
    for y in xrange(height//3, 2*height//3,1):
      for x in xrange(width//3,2*width//3,1):
        print (depth_array[height//2,width//2], switch)
        #depth_mean=depth_mean+ depth_array[y,x]
        #pix_num=pix_num+1
       
    #depth_mean=depth_mean/pix_num
    #if depth_mean >= 2000:
      #print(depth_mean)
    #mounted objects have closer range 
      #switch = min(depth_array[height//2,width//2],switch)
      switch = min(depth_array[y,x],switch)
    try:
        #0 would be inside the camera -> flag for nothing detected
        self.obstacle_pub.publish(Int32(switch))
    except CvBridgeError as e:
        print(e)


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
