std_msgs/Header header
int32 drone_id
int32 num_components
int32 state_dim    # 新增：GMM拟合的状态维度=9，对应论文的nx
int32 obs_dim      # 保留：观测维度=6，仅用于观测更新
bool has_observation

# 本地统计量u (论文公式21)，9维状态空间
float64[] alpha_local
float64[] a_local    # C*state_dim 维
float64[] b_local    # C*state_dim*state_dim 维

# 共识状态ζ (论文公式26)，9维状态空间
float64[] zeta_alpha
float64[] zeta_a     # C*state_dim 维
float64[] zeta_b     # C*state_dim*state_dim 维

