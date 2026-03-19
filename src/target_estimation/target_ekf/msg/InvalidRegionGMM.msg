std_msgs/Header header
int32 drone_id
int32 type          # 0=negative_observation, 1=obstacle
int32 num_components
float64[] weights   # [C]
float64[] means     # [C*3] 3D position means
float64[] covs      # [C*9] 3x3 covariance matrices (row-major)
