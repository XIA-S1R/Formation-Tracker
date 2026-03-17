Header header
int32 drone_id

# 三个搜索目标点
geometry_msgs/Point forward
geometry_msgs/Point left_45
geometry_msgs/Point right_45

# 反推后的均值位置和速度
geometry_msgs/Point mean_pos
geometry_msgs/Vector3 mean_vel

# 消失帧数
int32 num_frames_back
