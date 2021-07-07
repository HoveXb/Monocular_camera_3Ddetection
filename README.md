# Side_Camera_detection
>* 说明：本仓库是大轿车智能化项目侧视盲区摄像头检测的ros实现代码
>* 大轿车共搭载5颗usb摄像头，各摄像头的id按前后左右车内分别设为0，1，2，3，4;
>* 其中，左右摄像头用于车辆盲区检测，其利用yolov5作为2D目标检测，并利用PNP算法将2D目标逆投影得到目标3D位置信息
>* ros中的各话题名称具体如下：

>|话题名称|含义|
>|---|----|
>|`msg_camera_prep_id`|第id个摄像头的图像数据|
>|`msg_camera_obj_id`|第id个摄像头的检测到的目标信息|
>|`msg_camera_result_id`|第id个摄像头的图像检测结果|
>各个摄像头检测算法的配置信息在`n_camera_obj_config_id.yaml`
***
## 1.环境需求
环境安装：
```python
pip install requirement.txt
```
> PS:pytorch 版本不高于1.18（本仓库中的有些接口在1.19中已被弃用）
*** 

## 2.如何运行
1. 下载本仓库：`git clone git@github.com:HoveXb/Side_Camera_detection.git`
2. 将本仓库移至你的ros工作空间中的src目录
3. 修改本仓库launch文件下`sidecam_launch.launch`中`n_camera_obj_config_id`的`value`,改值为`n_camera_obj_config_id.yaml`文件的绝对路径
4. 依据需求修改本仓库中的`n_camera_obj_config.yaml`中的各项属性
5. 回到你的ros工作空间，编译`catkin_make`
6. 添加路径，`export ./devel/setup.sh`
7. 运行lanch文件，`roslaunch sidecamera_detect sidecam_launch.launch`
***
## 3.n_camera_obj_config.yaml的各项属性说明
| 属性          | 注释                                                                     |
| ------------- | ------------------------------------------------------------------------ |
| device        | 指定GPU设备                                                              |
| weights       | 权重文件路径                                                             |
| img_size      | 输入至网络的图片大小                                                     |
| conf_thres    | 检测结果的置信度阈值                                                     |
| iou_thres     | 进行Nms时，IOU的阈值                                                     |
| draw_result   | 是否绘制检测结果                                                         |
| view_img      | 是否发布检测后的图像（可使用rviz查看），若draw_result=False,则该项无效   |
| save_img      | 是否保存检测结果，若draw_result=False,则该项无效                         |
| save_path     | 检测结果保存路径，若draw_result=False,则该项无效                         |
| classes       | 选择要检测的类别,若全检测，则不设置值；若指定类别，可设 0 或者 [0,1,3,4] |
| agnostic_nms  | 是否使用agnostic_nsm                                                     |
| augment       | 是否对图片进行增强                                                       |
| camera_id     | 摄像头id，按前后左右驾驶室分为0，1，2，3，4                              |
| half          | 推理时是否使用半精度（1660s不支持）                                      |
| class_name    | 检测类别名称                                                             |
| camera_matrix | 相机内参矩阵绝对路径                                                     |
| dist_coefs    | 相机畸变系数绝对路径                                                     |
| rotM          | PNP中外参旋转矩阵绝对路径                                                |
| tvec          | PNP中外参平移矩阵绝对路径                                                |
*** 

## 4.相机内参和PNP转换矩阵的获得
参考另一个仓库
