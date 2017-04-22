#!/usr/bin/env python
# -*- coding: utf-8 -*-
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


def callback(data):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    img_msg1 = bridge.cv2_to_imgmsg(cv_image, "bgr8")

    image_pub = rospy.Publisher("/semantic_targets", TexturedQuadArray, queue_size=10)

    display_image = TexturedQuad()
    pose = Pose()
    pose.position.x =  0.0
    pose.position.y =  0.0
    pose.position.z =  0.0


    pose.orientation.x = 1.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 0.0

    display_image.image = img_msg1
    display_image.pose = pose
    display_image.width = 25
    display_image.height = 25
    display_image.border_color = [1., 0., 0., 0.5]
    display_image.border_size = 0.0
    display_image.caption = 'Directional Field'
    display_images = TexturedQuadArray()
    display_images = np.array([display_image])

    image_pub.publish(display_images)
    rate = rospy.Rate(30) # 10hz
    rate.sleep()

def listener():

    rospy.init_node('listener', anonymous=True)
    image_sub = rospy.Subscriber("/tf_image",Image,callback)
    rospy.spin()


if __name__ == '__main__':

    try:
        listener()
    except rospy.ROSInterruptException:
        pass
