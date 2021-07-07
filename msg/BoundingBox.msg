float32 camera_obj_prob # 障碍物概率
float32 camera_obj_xmin # 相对原点像素偏移
float32 camera_obj_ymin
float32 camera_obj_xmax
float32 camera_obj_ymax
float32 camera_obj_x # 车辆坐标系x坐标(m)
float32 camera_obj_y # 车辆坐标系y坐标(m)
float32 camera_obj_z # 车辆坐标系z坐标(m)
float32 camera_obj_length # 障碍物长度(m)
float32 camera_obj_width # 障碍物宽度(m)
float32 camera_obj_height # 障碍物高度(m)
float32 camera_obj_orientation # 障碍物形状方位角，单位度
int16 camera_obj_id # 在当前帧目标id
int16 camera_obj_class # 目标属于种类序号 0:未知；1：汽车；2：卡车；3：行人；4：摩托车；5自行车；6：大障碍物；7：小障碍物。