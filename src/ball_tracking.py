#!/usr/bin/env python3

## @package ball_tracking
#
# uses cv2 libraries to track the ball in the map and follow it (when in the play behaviour) 

# Python libs
import sys
import time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage, JointState 
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String, Float64
from random import randint
import math 

VERBOSE = False
pubHeadPos = None


## class ball_tracking
#
# implements a ball tracking algorithm using cv2
class ball_tracking:
    ## method __init__
    #
    # initialization of the ball_tracking class
    def __init__(self):
        # variables initialization
        self.ball_detected = False
        self.near_ball = False
        self.center = None
        self.radius = None
        self.behaviour = None
        self.ball_stopped = False

        # publishers
        self.vel_pub = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=1)
        self.pubBall = rospy.Publisher("/ball_detected", Bool, queue_size=1)

        # subscriber to camera 
        self.cam_sub = rospy.Subscriber("/robot/camera1/image_raw/compressed", CompressedImage, self.callback,  queue_size=1)

        # subscriber to current behaviour
        rospy.Subscriber("/behaviour", String, self.get_behaviour)
    


    ## method get_behaviour
    #
    # subscriber callback to the behaviour topic
    def get_behaviour(self, state):
        self.behaviour = state.data

    ## method follow_ball
    #
    # publish velocities to follow the ball
    def follow_ball(self):
        if self.ball_stopped:
            # if the ball stops, stop completely the robot
            twist_msg = Twist()
            twist_msg.angular.z = 0.0
            twist_msg.linear.x = 0.0
            self.vel_pub.publish(twist_msg) 
        else:
            # if the ball is detected go towards it and start following it
            if self.ball_detected:
                if self.near_ball: 
                    # if near enough to the ball start following it
                    twist_msg = Twist()
                    twist_msg.angular.z = 0.003*(self.center[0] - 400)
                    twist_msg.linear.x = -0.01*(self.radius - 100)
                    self.vel_pub.publish(twist_msg) 
                else:
                    # if not near enough go towards the ball
                    twist_msg = Twist()
                    twist_msg.linear.x = 0.8
                    self.vel_pub.publish(twist_msg)
            # if the ball is not detected search it
            elif not self.ball_detected:
                twist_msg = Twist()
                twist_msg.angular.z = 0.9
                #if self.behaviour == "play":
                self.vel_pub.publish(twist_msg)

    ## method callback
    #
    # Callback function of subscribed topic. 
    # Here images get converted and features detected
    def callback(self, ros_data):
        if not self.ball_stopped:
            if VERBOSE:
                print ('received image of type: "%s"' % ros_data.format)

            angular_z = None
            linear_x = None
            #### direct conversion to CV2 ####
            np_arr = np.fromstring(ros_data.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

            greenLower = (50, 50, 20)
            greenUpper = (70, 255, 255)

            blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, greenLower, greenUpper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            #cv2.imshow('mask', mask)
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            self.center = None
            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                self.radius = radius
                M = cv2.moments(c)
                self.center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # set ball detected to True
                self.ball_detected = True

                # only proceed if the radius meets a minimum size
                if radius > 10:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius),
                            (0, 255, 255), 2)
                    cv2.circle(image_np, self.center, 5, (0, 0, 255), -1)

                    self.near_ball = True
                else:
                    self.near_ball = False

            else:
                self.ball_detected = False
            
            # publish if the ball has been detected
            self.pubBall.publish(self.ball_detected)

            # update the points queue
            # pts.appendleft(center)
            cv2.imshow('window', image_np)
            cv2.waitKey(2)

            # if behaviour is play, follow the ball
            if self.behaviour == "play":
                if self.ball_detected and self.near_ball:
                    angular_z = 0.003*(self.center[0] - 400)
                    linear_x = -0.01*(self.radius - 100)
                    # if the ball is still, move the head
                    if abs(angular_z) < 0.04 and abs(linear_x) < 0.04 :
                        # signal that the ball has stopped
                        self.ball_stopped = True
                # follow the ball
                self.follow_ball()

## function move_head
#
# move the head of the robot when the ball stops
def move_head():
        rospy.loginfo("The ball has stopped!")

        angle = math.pi/4

        # move the head in one direction 45 degrees
        head_angle = Float64()
        head_angle.data = angle
        pubHeadPos.publish(head_angle)
        rospy.sleep(randint(3,6))

        # move the head in the other direction 45 degrees
        head_angle = Float64()
        head_angle.data = -angle
        pubHeadPos.publish(head_angle)
        rospy.sleep(randint(3,6))

        # move the head in the the default direction
        head_angle = Float64()
        head_angle.data = 0
        pubHeadPos.publish(head_angle)

## function main
#
# init node and ball tracking class, and checks when the robot head should be moved
def main(args):
    ## initialize ball tracking node
    rospy.init_node('ball_tracking', anonymous=True)

    global pubHeadPos

    rate = rospy.Rate(100)

    pubHeadPos = rospy.Publisher("/robot/joint_position_controller/command", Float64, queue_size=1)

    # Initializes class 
    bt = ball_tracking()

    while not rospy.is_shutdown():
        # if the ball stops, move the head of the robot
        if bt.ball_stopped:
            move_head()
            rospy.sleep(2)
            bt.ball_stopped = False
            rospy.loginfo("Return to ball tracking...")

        rate.sleep()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
