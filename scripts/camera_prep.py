#!/usr/bin/env python
# coding=utf-8
'''
Author: HoveXb
Date: 2021-07-06 
Desciption: 发布摄像头msg
Version: 1.0
Email: hovexb428@hnu.edu.cn
'''


import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import argparse


def read_camera(id=3, rate_hz=15):
    """
    读取摄像头，并按照频率发布
    """
    image_pub = rospy.Publisher('msg_camera_prep_%s'%id, Image, queue_size=1)
    bridge = CvBridge()
    
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        raise Exception('Camera %d is not available!'%id) 
    
    rate = rospy.Rate(rate_hz)
    msg = Image()
    rospy.loginfo('Publishing camera %s frame: %d FPS' % (id, rate_hz))
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.loginfo('Failed to read image from camera %s!'%id)
        else:
            # if arg.view_img:
                # cv2.imshow("1", frame)
                # cv2.waitKey(1)
            msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            #rospy.loginfo('Publishing camera %s frame: %d FPS' % (id, rate_hz))
        image_pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    id= 3
    rate = 30
    rospy.init_node('n_camera_prep', anonymous=True)
    read_camera(id, rate)

