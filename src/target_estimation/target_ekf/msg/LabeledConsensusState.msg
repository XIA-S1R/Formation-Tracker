Header header
int32 drone_id                           # 无人机ID
int32 label                    # 标签类型 (0=STRAIGHT, 1=LEFT_TURN, 2=RIGHT_TURN)
int32 num_components          # GMM分量数
int32 state_dim               # 状态维度 (固定为9)
float64[] alpha               # 混合权重数组
float64[] a                   # 一阶矩数组 (大小: num_components * state_dim)
float64[] b                   # 二阶矩数组 (大小: num_components * state_dim * state_dim)
bool has_obs                  # 是否有观测