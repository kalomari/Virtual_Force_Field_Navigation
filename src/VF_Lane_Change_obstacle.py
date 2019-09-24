# Navigating using Virtual Force Field Approach and switch lane when detecting obstacle on the cuurent lane

#!/usr/bin/env python
import numpy as np
import sys
import rospy
import rospkg
import math
import tf

from std_msgs.msg import Int32, UInt8, Float64, Int16, Float64MultiArray, Bool, Float32
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2

from nav_msgs.msg import Odometry
from autominy_msgs.msg import NormalizedSteeringCommand, NormalizedSpeedCommand

class obst_driver:
    def __init__(self):
        
        self.lane=1
        self.speed_value= 0.1
        self.publishing=False

        # imageimport
        self.bridge = CvBridge()
        
        # load forcefield
        rospack = rospkg.RosPack()
        self.file_path=rospack.get_path('fub_navigation')+'/src/'
        if (self.lane==0):
            self.matrix = np.load(self.file_path+'matrix50cm_lane1.npy')
        else:
            self.matrix = np.load(self.file_path+'matrix50cm_lane2.npy')

        rospy.on_shutdown(self.shutdown)
        self.shutdown_ = False

        # dynamic mapsize for some reason matrix is not y * x (rows by columns)
        self.resolution = 1  # cm
        self.map_size_x, self.map_size_y = self.matrix.shape[:2]
        self.map_size_y *= self.resolution
        self.map_size_x *= self.resolution

        #publisher
        self.obstacle_pub = rospy.Publisher("/obstacle/warning", Int32, queue_size=1)
        self.vel_pub = rospy.Publisher("/actuators/speed_normalized",NormalizedSpeedCommand, queue_size=100)
        self.str_pub = rospy.Publisher("/actuators/steering_normalized",NormalizedSteeringCommand, queue_size=1)
        self.lane_change = rospy.Publisher("/Lane_change", Bool, queue_size=1)

        #subscribers
        self.image_sub = rospy.Subscriber("/voxel_grid/output", PointCloud2,self.obstacle_callback, queue_size=1)
        self.obstacle_sub = rospy.Subscriber("/obstacle/warning", Int32, self.callback, queue_size=1)
        self.odom_sub = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, self.PID_call_back)
        self.lane_change_sub = rospy.Subscriber("/Lane_change", Bool, self.Lane_cahange)
        
        # PID controller intern variables
        self.aq_error = 0.0
        self.last_error = 0.0
        self.last_time = rospy.Time.now()
        #self.str_val = 0.0
        
        #object avoiding
        self.time_acc = rospy.Time.now()



    def obstacle_callback(self, data):
        gen = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z"))
        for P in gen:
            print (P)
            x = P[0]
            #print (x)
            switch=0.95;
            if x > switch:
                if len (data.data) > 10:
                    self.obstacle_pub.publish(Int32(switch))
                    #print (x, switch)

        #print("switch")

    def Lane_cahange(self, raw_msgs):
        # swap out loaded force field
        #print("Hi")
        self.lane = (self.lane + 1) % 2
        if (self.lane == 0):
            self.matrix = np.load(self.file_path + 'matrix50cm_lane1.npy')
            print("Lane_1")
        else:
            self.matrix = np.load(self.file_path + 'matrix50cm_lane2.npy')
            print("Lane_2")

    def PID_call_back(self, raw_msgs):

        #retrieving values from message
        x = raw_msgs.pose.pose.position.x
        y = raw_msgs.pose.pose.position.y
        orientation= raw_msgs.pose.pose.orientation
        orientation_array = [orientation.x, orientation.y, orientation.z, orientation.w]
        
        #change the read-out data from Euler to Rad
        #(roll, pitch, yaw) = (orientation_array)
        orientation_in_Rad= tf.transformations.euler_from_quaternion(orientation_array) 
        self.yaw =orientation_in_Rad[2]
        x_ind = np.int(x*(100.0/self.resolution))
        y_ind = np.int(y*(100.0/self.resolution))
        
        #if abroad, car virtually set on the map
        if x_ind < 0:
            x_ind = 0
        if x_ind > self.map_size_x/self.resolution -1:
            x_ind = self.map_size_x/self.resolution -1
        
        if y_ind < 0:
            y_ind = 0
        if y_ind > self.map_size_y/self.resolution -1:
            y_ind = self.map_size_y/self.resolution -1
        
        x_map, y_map = self.matrix[x_ind, y_ind]
    
        x_car = np.cos(self.yaw)*x_map + np.sin(self.yaw)*y_map
        y_car = -np.sin(self.yaw)*x_map + np.cos(self.yaw)*y_map
        
        #hyperparameters for PID
        Kp= 3.0
        Ki= 0.0
        Kd= 0.0

        #so the error is the steepness of the correction anlge
        error= np.arctan2(y_car, x_car)
        #print(x_ind, y_ind, x_car, y_car)
        
        #pseudo integral memory in borders
        self.aq_error = self.aq_error + error
        if self.aq_error > 10:
            self.aq_error=10
        elif self.aq_error < -10:
            self.aq_error = -10
        
        #time between measurements for delta t
        current_time = rospy.Time.now()
        dif_time = (current_time - self.last_time).to_sec()
        #if dif_time == 0.0:
        #   return

        #error manipulation with PID
        PID= Kp * error  + Ki * self.aq_error * dif_time + Kd * (error - self.last_error) / dif_time
        #print(PID)
        
        self.last_time = current_time
        self.last_error = error

        
        #detect min dist direction
        vel=NormalizedSpeedCommand()
        if (x_car<0):
            vel.value = self.speed_value
        else:
            vel.value = -self.speed_value
        if x_car > 0:
        
            #reduce speed based on steering
            vel.value = max(self.speed_value, vel.value * ((np.pi/3.0)/(abs(PID)+1.0)))
        
            #valid steering angle -100<=alpha<=80 
        
        if PID > 1:
            PID= 1
        elif PID < -1:
            PID = -1

        str_val = NormalizedSteeringCommand()
        str_val.value = PID
        
        #dont start acceleration after shutdown
        if not self.publishing:
            self.str_pub.publish(str_val)
            self.vel_pub.publish(vel)

    def callback(self,raw_msg):
        #flag something detected
        if raw_msg.data != 0.95:
            if (rospy.Time.now() - self.time_acc).to_sec() > 3 :
                self.lane_change.publish(Bool(True))
                self.time_acc = rospy.Time.now()

    def shutdown(self):
        #set speed to 0
        print("shutdown!")
        self.publishing=True
        msgs=NormalizedSpeedCommand()
        self.vel_pub.publish(msgs)
        rospy.sleep(1)

def main(args):
    rospy.init_node('obst_driver', anonymous=True)
    driv = obst_driver()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

