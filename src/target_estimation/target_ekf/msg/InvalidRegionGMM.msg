std_msgs/Header header
int32 drone_id
float64 origin_x     # 栅格原点 x (m)
float64 origin_y     # 栅格原点 y (m)
float64 resolution   # 栅格分辨率 (m/cell)
int32 nx             # x方向格数
int32 ny             # y方向格数
# 游程编码 (RLE): 每段 (value, count, frame)
# value: 0=未知, 1=负观测无效, 2=障碍
int32[]  rle_values
int32[]  rle_counts
uint16[] rle_frames  # 每段负观测格的最后标记帧号（障碍段忽略）
