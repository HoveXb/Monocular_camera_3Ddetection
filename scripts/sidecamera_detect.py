#!/usr/bin/env python3
# coding=utf-8
'''
Author: HoveXb
Date: 2021-6-26 
LastEditTime: 2021-6-26 
Desciption:侧视摄像头目标检测与位置估计
Version: 
Email: hovexb428@hnu.edu.cn
'''

from numpy.matrixlib.defmatrix import mat
import rospy
from sensor_msgs.msg import Image
from camera_detect.msg import BoundingBox
from camera_detect.msg import BoundingBoxes
from std_msgs.msg import Header
import torch
import numpy as np
from models.experimental import attempt_load
from utils.datasets import letterbox
from utils.general import check_img_size,  non_max_suppression,scale_coords
from utils.plots import plot_one_box
from utils.torch_utils import select_device

class Sidecam_detect(object):
    def __init__(self,hyps):
        super(Sidecam_detect).__init__()
        #load hyperparameters
        self.device = hyps['device']
        self.weights = hyps['weights']
        self.img_size = hyps['img_size']
        self.conf_thres = hyps['conf_thres']
        self.iou_thres = hyps['iou_thres']
        self.draw_result=hyps['draw_result']
        self.view_img = hyps['view_img']
        self.save_img = hyps['save_img']
        self.save_path = hyps['save_path']
        self.classes = hyps['classes']
        self.agnostic_nms = hyps['agnostic_nms']
        self.augment = hyps['augment']
        self.id = hyps['camera_id']
        self.half = hyps['half']# 1660s不支持float16
        self.class_name = hyps['class_name']

        #PNP parameters
        self.camera_matrix = np.load(hyps['camera_matrix'])
        self.dist_coefs = np.load(hyps['dist_coefs'])
        self.rotM = np.load(hyps['rotM'])
        self.tvec = np.load(hyps['tvec'])

        self.devices = select_device(self.device)

        if self.half:
            rospy.loginfo("Warning:1660s cann't float16!Change to float32 by default.\n")
            self.half = False
        self.model = attempt_load(self.weights, map_location=self.devices)  # load FP32 model

        #check image size
        stride = int(self.model.stride.max())  # model stride
        imgsz = check_img_size(self.img_size, s=stride)  # check img_size

        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[np.random.randint(0, 255) for _ in range(3)] for _ in self.names]

        # run once, warm up
        if self.devices.type != 'cpu':
            self.model(torch.zeros(1, 3, imgsz, imgsz).to(self.devices).type_as(next(self.model.parameters())))

        self.image_sub = rospy.Subscriber(f'msg_camera_prep_{self.id}', Image, self.detect,queue_size=1)
        self.obj_pub = rospy.Publisher(f'msg_camera_obj_{self.id}', BoundingBoxes, queue_size=1)
        if self.view_img:
            self.image_pub = rospy.Publisher(f'msg_camera_result_{self.id}',Image,queue_size=1)
            self.ros_frame = Image()

    @torch.no_grad()
    def detect(self,data):
        pass
        boxes = BoundingBoxes()
        box = BoundingBox()
        if len(data.data) == 0:
            boxes.camera_status = 0
        else:
            boxes.camera_status = 1

            img0 = np.ndarray((data.height, data.width, 3), dtype=np.uint8, buffer=data.data)

            #备份原始图像，供后续绘制结果使用
            im0s = img0
            # 去噪
            # img0 = cv2.fastNlMeansDenoisingColored(img0, None, 10, 10, 7, 21)

            img = letterbox(img0, new_shape=self.img_size)[0]
            # Convert
            img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
            img = np.ascontiguousarray(img)

            img = torch.from_numpy(img).to(self.devices)
            img = img.half() if self.half else img.float()  # uint8 to fp16/32
            img /= 255.0  # 0 - 255 to 0.0 - 1.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)

            # Inference
            pred = self.model(img, augment=self.augment)[0]

            # Apply NMS
            pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, classes=self.classes, agnostic=self.agnostic_nms)

            # Process detections
            for i, det in enumerate(pred):  # detections per image
                im0 = im0s
                if len(det):
                    # Rescale boxes from img_size to im0 size
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                    boxes.count = det.shape[0]
                    for j in range(det.shape[0]):
                        # box.camera_obj_class = self.class_name[int(det[j, -1])]
                        box.camera_obj_class = int(det[j, -1])
                        box.camera_obj_prob = float(det[j, -2])
                        box.camera_obj_xmin = float(det[j, 0])
                        box.camera_obj_ymin = float(det[j, 1])
                        box.camera_obj_xmax = float(det[j, 2])
                        box.camera_obj_ymax = float(det[j, 3])
                        # box.camera_obj_id = j

                        #PNP to get 3D location
                        twoD_matric = [int(det[j, 2]), int(det[j, 3]),1]
                        temmat = mat(self.rotM).I * mat(self.camera_matrix).I * mat(twoD_matric).T
                        temmat2 = mat(self.rotM).I * mat(self.tvec)
                        s = temmat2[2]
                        s = s / temmat[2]
                        wcpoint = mat(self.rotM).I * (s[0, 0] * mat(self.camera_matrix).I * mat(twoD_matric).T - mat(self.tvec))

                        box.camera_obj_x = float(wcpoint[0])
                        box.camera_obj_y = float(wcpoint[1])
                        box.camera_obj_z = 0

                        boxes.bounding_boxes.append(box)
                    if self.draw_result:
                        #draw the result
                        for *xyxy, conf, cls in reversed(det):
                            label = f'{self.names[int(cls)]} {conf:.2f}'
                            plot_one_box(xyxy, im0, label=label, color=self.colors[int(cls)], line_thickness=2,
                            camera_matrix=self.camera_matrix,rotM=self.rotM,tvec=self.tvec)

                        #publish the detection result img
                        if self.view_img:
                            # cv2.imshow("result", im0)
                            # cv2.waitKey(1)
                            header = Header(stamp=rospy.Time.now())
                            # header.frame_id = "Camera3"
                            self.ros_frame.header = header
                            self.ros_frame.width = im0.shape[:2][1]
                            self.ros_frame.height = im0.shape[:2][0]
                            self.ros_frame.encoding = "bgr8"
                            #self.ros_frame.step = im0.shape[:2][1]
                            self.ros_frame.data = np.array(im0).tostring()  # 图片格式转换
                            self.image_pub.publish(self.ros_frame)  # 发布消息

                        if self.save_img:
                            import cv2
                            import time
                            import os
                            cv2.imwrite(os.path.join(self.save_path,str(time.strftime("%Y-%m-%d-%H:%M:%S", time.localtime()))+".png"), im0)

        self.obj_pub.publish(boxes)
if __name__ == '__main__':

    #initialize ros_node
    rospy.init_node('n_camera_obj_3')
    #load hyperparameters
    import yaml
    config_path = rospy.get_param('n_camera_obj_config_3')
    with open(config_path) as f:
        hyps = yaml.load(f, Loader=yaml.SafeLoader)  # load hyps
    
    sideCamDetector = Sidecam_detect(hyps)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('KeyboardIterrupt,see you again!')
