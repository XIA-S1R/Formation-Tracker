#!/bin/bash
# 向所有跟踪无人机发布降落触发信号
# planning.launch 已将 ~land_triger remap 到全局 /land_triger，所有无人机共享此话题
# 持续发布（不加 -1），确保节点启动后一定能收到，Ctrl-C 停止
# 用法：bash land_triger.sh

echo "[land_triger] Publishing /land_triger ... (Ctrl-C to stop)"

rostopic pub /land_triger geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
