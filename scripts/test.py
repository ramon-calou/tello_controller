#!/usr/bin/env python3
import cv2, rospy
from cv_bridge import CvBridge
import image_lib_v2 as img
from sensor_msgs.msg import Image

# Load the cascade
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

# To capture video from webcam. 
#cap = cv2.VideoCapture(0)
# To use a video file as input 
# cap = cv2.VideoCapture('filename.mp4')

def callback_img (msg):
  
  global img
  global gray
  
  bridge = CvBridge()

  img = bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")
  gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
  #cv2.imshow("HSV", cv_image)
  #cv2.waitKey(3) 

img = 0
gray = 0

rospy.init_node('extract_image_info_node', anonymous=True)

# Publishers
pub_goal_centroid = rospy.Publisher('goal_centroid', Pose2D, queue_size=10)  # send control signals
pub_goal_base = rospy.Publisher('goal_base', Pose2D, queue_size=10)  # send control signals

rospy.Subscriber('/tello/camera/image_raw', Image, callback_img)  
rate = rospy.Rate(30) 
rospy.sleep(2)

while True:
    # Read the frame
    #_, img = cap.read()
    # Convert to grayscale
    #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Detect the faces
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=4, minSize=(30, 30))
    # Draw the rectangle around each face
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
    # Display
    cv2.imshow('img', img)
    # Stop if escape key is pressed
    k = cv2.waitKey(30) & 0xff
    if k==27:
        # closing window
        cv2.destroyAllWindows()
        break