#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#############################################
#                                           #
# Node to control the Tello Drone.          #
# This node receives image point and sends  #
# control signal to the cmd_vel topic.      #
#                                           #
# Author: Adalberto Oliveira                #
# Modified By: Ramon Alves                  #
# Autonomous Vehicle - Infnet	             #
# Version: 1.3                              #
# Date: 20 mar 2021                         #
#                                           #
#############################################


# importing libraries
import rospy, time, sys, math, control_lib, tf
import numpy as np
from geometry_msgs.msg import Pose2D, Twist
from turtlesim.msg import Pose
from std_msgs.msg import Bool, Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
import image_geometry

############ WORK FUNCTIONS ##################

def callback_camera_info(msg):

    global model
    global camera_matrix


    model.fromCameraInfo(msg)
    
    K = np.array(msg.K).reshape([3, 3])
    f = K[0][0]
    u0 = K[0][2]
    v0 = K[1][2]

    camera_matrix[0] = f
    camera_matrix[1] = u0
    camera_matrix[2] = v0

def callback_img_point(msg):
    """
    This function receives the goal and saves it 
    in a globa variable goal
    """

    global camera_height
    global image_point
    global mask_is_true

    # recovering point
    u = msg.x
    v = msg.y
    base_point = [u, v]
    mask_is_true = msg.theta
    distance = 0

    try:
        # finding distance to the point 
        pixel_rectified = model.rectifyPoint(base_point)
        line = model.projectPixelTo3dRay(pixel_rectified)
        th = math.atan2(line[2],line[1])
        distance = math.tan(th) * camera_height

        image_point.x = u
        image_point.y = v
        image_point.theta = distance

    except:
        pass

def control_robot():
    """
    This function is called from the main conde and calls 
    all work methods of fucntions into the codde.
    """

    # Global variables
    global img_goal
    global image_point
    global robot_pose
    global gains_cart
    global max_lin
    global max_ang
    global goal
    global camera_matrix
    global mask_is_true

    # Initializing ros node
    rospy.init_node('tello_control', anonymous=True)    # node name
    
    # Subscribers
    rospy.Subscriber('img_point',Pose2D, callback_img_point)   # receives the goal coordinates
    rospy.Subscriber('camera_info',CameraInfo, callback_camera_info)   # receives the goal coordinates

    # Publishers
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)  # send control signals

    # control rate
    rate = rospy.Rate(2)

    # main loop
    while not rospy.is_shutdown():

        # Computing the control signal
        control_signal = Twist()
        
        #Foi utilizado o método IBVS para realização do controle do drone

        try:
            if mask_is_true:
                control_signal = control_lib.ibvs(img_goal, image_point, camera_matrix, gains_cart,vel_lim)
                
            else:
                control_signal = Twist()
                control_signal.linear.x = 0.
                control_signal.angular.z = 0.5
        except:
            pass

        #print control_signal
        cmd_vel.publish(control_signal)

        print('\rDistance to the target:',image_point.theta, end='\r')

        rate.sleep()
    



############ MAIN CODE #######################
# initializing Global variables
# Readin from launch
K_eu = float(sys.argv[1])   # Ganho de velocidade linear
K_ev = float(sys.argv[2])   # Ganho de velocidade angular
X_goal = float(sys.argv[3])
Y_goal = float(sys.argv[4])
max_lin = float(sys.argv[5]) #Velocidade máxima linear
max_ang = float(sys.argv[6]) #Velocidade máxima angular
camera_height = float(sys.argv[7])

# Inner values
image_point = Pose2D()
gains_cart = [K_eu, K_ev]
img_goal = Pose2D()
img_goal.x = X_goal
img_goal.y = Y_goal
camera_matrix = np.zeros((3,1))
vel_lim = [max_lin, max_ang]
mask_is_true = False

# creating a camera model
model = image_geometry.PinholeCameraModel()


print('#############################################',
      '\n#                                           #',
      '\n# Node to control the Tello Drone.          #',
      '\n# This node receives image point and sends  #',
      '\n# control signal to the cmd_vel topic.      #',
      '\n#                                           #',
      '\n# Author: Adalberto Oliveira                #',
      '\n# Modified By: Ramon Alves                  #',
      '\n# Autonomous Vehicle - Infnet               #',
      '\n# Version: 1.3                              #',
      '\n# Date: 20 mar 2021                         #',
      '\n#                                           #',
      '\n#############################################')




if __name__ == '__main__':
    control_robot()
