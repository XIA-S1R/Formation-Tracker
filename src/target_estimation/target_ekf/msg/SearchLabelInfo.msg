Header header
int32 source_drone_id          # 发布此消息的无人机ID（掌管者）

# 标签信息
int32 label                    # 标签类型: 0=STRAIGHT, 1=LEFT_TURN, 2=RIGHT_TURN

# 粒子群均值位置
geometry_msgs/Point mean_pos

# 粒子群均值速度方向（用于计算偏航角）
geometry_msgs/Vector3 mean_vel

# 分配到该标签的无人机ID列表
int32[] assigned_drone_ids

# 粒子数量（用于调试）
int32 num_particles
