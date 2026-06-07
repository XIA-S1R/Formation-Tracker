# real_uav_bridge

UDP bridge for the real-tracker deployment.

In this mode the ground PC runs mapping, target simulation, DPF, planning, formation logic, and the ground bridge. Each onboard computer runs MAVROS, odometry, `px4ctrl`, and the onboard bridge. The bridge sends only fresh `quadrotor_msgs/PositionCommand` packets from the ground PC to each onboard computer, and sends `nav_msgs/Odometry` plus `/traj_start_trigger` back to the ground PC.

## Ground PC

```bash
roslaunch simulation tracking_real_triangle.launch \
  drone0_ip:=192.168.1.101 \
  drone1_ip:=192.168.1.102 \
  drone2_ip:=192.168.1.103
```

Use a measured static map instead of `mockamap`:

```bash
roslaunch simulation tracking_real_triangle.launch \
  use_mock_map:=false \
  static_map_file:=/path/to/site_map.pcd
```

The map must be in the same ENU/world frame as `/drone0/odom`, `/drone1/odom`, and `/drone2/odom`.

## Onboard Computer

Run one instance per real tracker:

```bash
roslaunch real_uav_bridge onboard_tracker_px4.launch \
  drone_id:=0 \
  ground_ip:=192.168.1.10 \
  cmd_listen_port:=18000 \
  ground_port:=18100 \
  odom_topic:=/vins_estimator/imu_propagate \
  start_mavros:=true \
  fcu_url:=/dev/ttyTHS1:921600
```

Port pairing must match the ground launch:

| drone | onboard `cmd_listen_port` | ground `ground_port` |
| --- | ---: | ---: |
| drone0 | 18000 | 18100 |
| drone1 | 18001 | 18101 |
| drone2 | 18002 | 18102 |

`px4ctrl` publishes `/traj_start_trigger` after it is ready for command control. The onboard bridge returns that trigger to the ground PC, and the ground bridge publishes `/triger` only after all trackers report ready.

