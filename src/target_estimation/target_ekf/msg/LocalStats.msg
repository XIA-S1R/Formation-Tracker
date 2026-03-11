# Local summary quantities for Distributed Particle Filter (Gu 2007, Eq.21)
# Each drone publishes this after computing local statistics from weighted particles.

Header    header
int32     drone_id            # sender drone ID (m)
int32     num_components      # C: number of GMM components
int32     obs_dim             # n_z: dimension of predicted observations

# Local summary quantities for each GMM component c (Eq.21):
#   alpha_local[c] = k * alpha_{m,c}  (scalar per component)
#   a_local[c]     = a_{m,c}          (obs_dim vector per component, flattened)
#   b_local[c]     = b_{m,c}          (obs_dim x obs_dim matrix per component, flattened)

float64[] alpha_local         # [C]                        
float64[] a_local             # [C * obs_dim]              
float64[] b_local             # [C * obs_dim * obs_dim]    

bool      has_observation     # whether this drone had a valid observation this step
