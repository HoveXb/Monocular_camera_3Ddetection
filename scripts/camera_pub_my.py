#! /usr/bin/env python3
# coding=utf-8

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np 
from std_msgs.msg import Header

def camera_capture():
    rospy.init_node("read_camera",anonymous=True)
    cam_pub = rospy.Publisher("msg_camera_prep_3",Image,queue_size=10)

    capture = cv2.VideoCapture(0)
    ret, frame = capture.read()
    size_h,size_w = frame.shape[0],frame.shape[1]
    while not rospy.is_shutdown():
        
            ret, frame = capture.read()
            #frame = cv2.flip(frame,0)   #镜像操作
            #frame = cv2.flip(frame,1)   #镜像操作   
       
            ros_frame = Image()
            header = Header(stamp = rospy.Time.now())
            # header.frame_id = "Camera3"
            ros_frame.header=header
            ros_frame.width = size_w
            ros_frame.height = size_h
            ros_frame.encoding = "bgr8"
            ros_frame.step = 1920
            ros_frame.data = np.array(frame).tostring() #图片格式转换
            cam_pub.publish(ros_frame) #发布消息


if __name__=="__main__":
    camera_capture()
