#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

import copy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import tf
import math

from rviz_textured_quads.msg import TexturedQuad, TexturedQuadArray


def pub_image():

    rospy.init_node('rviz_display_image_test', anonymous=True)
    image_pub = rospy.Publisher("/targets", TexturedQuadArray, queue_size=10)

    img1 = cv2.imread('./textures/bebop_drone.jpg',cv2.IMREAD_COLOR)
    img_msg1 = CvBridge().cv2_to_imgmsg(img1, "bgr8")

    img2 = cv2.imread('./textures/Decal.png',cv2.IMREAD_COLOR)
    img_msg2 = CvBridge().cv2_to_imgmsg(img2, "bgr8")

    cap = cv2.VideoCapture('/home/mohitshridhar/Downloads/ICRA_2010.mov')

    display_image = TexturedQuad()

    pose = Pose()

    pose.position.x =  -1.2
    pose.position.y =  -0.5
    pose.position.z =  2.0

    # pose.orientation.x = 0.70710678
    # pose.orientation.y = 0.0
    # pose.orientation.z = 0.0
    # pose.orientation.w = 0.70710678

    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0

    scale = 0.5

    display_image.image = img_msg1
    display_image.pose = pose
    display_image.width = scale
    display_image.height = (scale * img_msg1.height)/img_msg1.width
    display_image.border_color = [1., 0., 0., 0.5]
    display_image.border_size = 0.05
    display_image.caption = 'ICRA Video'


    second_image = copy.deepcopy(display_image)
    second_image.image = img_msg2
    second_image.width = 1.0
    second_image.height = 1.0
    second_image.pose.position.x = 2.2
    second_image.pose.position.y = -0.3
    second_image.pose.position.z = 2.0

    second_image.pose.orientation.x = 0.0
    second_image.pose.orientation.y = 0.70710678
    second_image.pose.orientation.z = 0.0
    second_image.pose.orientation.w = 0.70710678
    second_image.border_color = [0.5, 1.0, 0.0, 0.5]
    second_image.border_size = 0.1
    second_image.caption = 'Decal'

    display_images = TexturedQuadArray()
    display_images = np.array([display_image, second_image])

    rate = rospy.Rate(30) # 10hz
    deg_increment = 0.005
    angle = 0
    count = 0

    while not rospy.is_shutdown():
        if (cap.isOpened()):
            ret, frame = cap.read()

            display_image.image = CvBridge().cv2_to_imgmsg(frame, "bgr8")

        angle += deg_increment

        second_image.pose.position.x = 2.0 * math.sin(angle)
        second_image.pose.position.y = 2.0 * math.cos(angle)


        q = tf.transformations.quaternion_from_euler(angle + deg_increment, 0., 0.);
        second_image.pose.orientation.x = q[0]
        second_image.pose.orientation.y = q[1]
        second_image.pose.orientation.z = q[2]
        second_image.pose.orientation.w = q[3]

        # q = tf.transformations.quaternion_from_euler(0., angle + deg_increment, 0.);
        # display_image.pose.orientation.x = q[0]
        # display_image.pose.orientation.y = q[1]
        # display_image.pose.orientation.z = q[2]
        # display_image.pose.orientation.w = q[3]


        count = count + 1

        image_pub.publish(display_images)
        rate.sleep()

    cap.release()

if __name__ == '__main__':

    try:
        pub_image()
    except rospy.ROSInterruptException:
        pass
