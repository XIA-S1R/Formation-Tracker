# real_uav_bridge

UDP bridge for the real-field tracker deployment.

The ground PC keeps the successful `tracking_real_field_sim_triangle.launch` planning chain: static real map, local mapping, DPF target estimation, tracker planning, and virtual target simulation. Only the tracker simulator/controller is replaced by real mocap odometry input and UDP `quadrotor_msgs/PositionCommand` output.

## Ground PC

Start the real-field real-tracker launch:

```bash
source devel/setup.bash
roslaunch simulation tracking_real_field_real_triangle.launch \
  drone0_ip:=192.168.1.101 \
  drone1_ip:=192.168.1.102 \
  drone2_ip:=192.168.1.103
```

This launch starts:

- `/global_map` from the measured static map.
- `/drone0/odom`, `/drone1/odom`, `/drone2/odom` from UDP JSON mocap receivers.
- `/droneX/planning`, `/droneX/mapping`, `/droneX/target_dpf_sim_node`, and `/droneX/traj_server`.
- `/droneX/position_cmd` UDP senders, gated by `/triger`.
- `trigger_all_ready_node`, which publishes `/triger` only after all three `/droneX/traj_start_trigger` messages arrive.
- simulated target dynamics and `real_field_evasion.py`, which waits for `/triger` and then delays before target evasion.

Mocap relay ports default to:

| drone | PC mocap UDP port | ROS output |
| --- | ---: | --- |
| drone0 | 5005 | `/drone0/odom` |
| drone1 | 5006 | `/drone1/odom` |
| drone2 | 5007 | `/drone2/odom` |

Position command ports default to:

| drone | onboard command port | PC trigger listen port |
| --- | ---: | ---: |
| drone0 | 18000 | 18100 |
| drone1 | 18001 | 18101 |
| drone2 | 18002 | 18102 |

By default the PC command sender drops `/droneX/position_cmd` until `/triger` is published. Keep `send_position_cmd_before_trigger:=false` for real flight.

## Onboard Computer

Run one instance per real tracker:

```bash
source devel/setup.bash
roslaunch real_uav_bridge onboard_position_cmd_receiver_px4.launch \
  drone_id:=0 \
  ground_ip:=192.168.1.10 \
  cmd_listen_port:=18000 \
  ground_port:=18100 \
  odom_topic:=/vins_estimator/imu_propagate \
  start_mavros:=true \
  fcu_url:=/dev/ttyTHS1:921600
```

The onboard bridge receives UDP `PositionCommand` packets and republishes them to `/tracker_cmd/position_cmd`; `px4ctrl` remaps private `~cmd` to that topic. It also forwards px4ctrl `/traj_start_trigger` back to the PC.

For this deployment, PC-side planner odometry comes from mocap UDP JSON, not from onboard odometry. The onboard bridge may still send odom packets to the PC bridge, but the PC launch maps them to `unused_udp_odom`.

## Safe Handover

1. Start the PC launch. It should not publish `/triger` yet because no tracker is ready.
2. Start each onboard launch with MAVROS, odometry, px4ctrl, and UDP receiver.
3. Manually take off with RC and hover at the safe test height.
4. Confirm mocap odometry is stable on the PC and in the same `world` frame as the map.
5. Toggle RC channel 5 into `AUTO_HOVER`; confirm px4ctrl holds position.
6. Toggle RC channel 6 to allow command control. px4ctrl publishes `/traj_start_trigger`.
7. After the PC receives all three ready triggers, it publishes `/triger`; tracker planning starts and the command UDP gate opens.
8. For the first real test, run with `start_target_evasion:=false`; after hover/tracking handover is safe, enable target evasion.

Emergency stop hierarchy:

- Preferred abort: toggle channel 6 out of command mode. px4ctrl should leave `CMD_CTRL` and return to `AUTO_HOVER`.
- Stronger abort: toggle channel 5 out of hover/offboard. px4ctrl returns to manual/PX4 control.
- Last resort: use RC/PX4 kill/disarm only for immediate danger. It is feasible, but it is a hard motor stop and can drop the vehicle, so test the switch props-off and keep it as the final emergency action.
