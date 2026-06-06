# Tracking-in-Formation
这是一个融合跟踪能力和编队优化的 ROS 项目，使无人机群体能够协作跟踪目标。

## 安装
1. 克隆项目到 catkin 工作空间。
2. 编译项目：
   ```bash
   catkin_make
   ```

## 快速启动指南

### 前置条件
- 已安装 ROS（Robot Operating System）。
- 已安装 RViz（用于可视化）。
- 确保所有依赖项已正确安装。

### 启动步骤

1. **配置工作空间**
   打开终端，进入工作空间并加载环境：
   ```bash
   source devel/setup.bash
   ```

2. **启动仿真**
   运行以下命令启动仿真：
   ```bash
   roslaunch simulation tracking_sim_triangle.launch
   ```

3. **启动 RViz**
   打开 RViz 并加载 `tracking_sim.rviz` 配置文件：
   ```bash
   rviz -d $(rospack find simulation)/config/tracking_sim.rviz
   ```

4. **启动控制**
   在启动控制前，需要手动运行触发脚本：
   ```bash
   ./sh_utils/pub_triger.sh
   ```

5. **控制目标无人机**
   默认使用离线机动脚本，让目标无人机直接执行一段固定参考轨迹。
   脚本会直接向 `/target/position_cmd` 发布离线轨迹采样点：
   ```bash
   python3 src/planning/planning/scripts/full_evasion.py
   ```
6. **监控无人机群**
   观察 3 架追踪无人机形成三角形编队并跟随目标无人机，在目标不可观的时候自动搜寻目标。

### 注意事项
- 无人机的初始位置在 `tracking_sim_triangle.launch` 文件中定义。
- 确保所有参数在启动文件和配置文件中正确设置。
- 当前主链路中，target 由 `full_evasion.py` 直接向 `/target/position_cmd` 下发离线参考轨迹。

更多详情请参考项目文档。


7. **批量实验**
运行批量实验，在指定目录生成bag
```bash
python3 src/planning/planning/scripts/run_full_evasion_batch.py \
  --runs 10 \
  --run-duration 70 \
  --reacq-timeout-sec 10 \
  --output-dir experiment_results/evasion_batch_10runs \
  --with-rviz
```
可选参数还有
```bash
--runs，默认 5
--run-duration，默认 120.0（秒）
--warmup，默认 8.0（秒）
--cooldown，默认 2.0（秒）
--drone-count，默认 3
--launch-cmd，默认 roslaunch simulation tracking_sim_triangle.launch
--trigger-cmd，默认 ./sh_utils/pub_triger.sh
--trigger-max-seconds，默认 1.5
--evasion-cmd，默认 python3 src/planning/planning/scripts/full_evasion.py
--rviz-cmd，默认 rviz -d $(rospack find simulation)/config/tracking_sim.rviz
--with-rviz（开关，默认关闭）
--formation-side-length，默认 2.0
--collision-distance，默认 0.0（机间碰撞）
--collision-release-distance，默认 0.0（机间碰撞释放）
--obstacle-collision-distance，默认 0.0（障碍碰撞）
--obstacle-collision-release-distance，默认 0.0（障碍碰撞释放）
--reacq-timeout-sec，默认 10.0
--output-dir，默认空（自动生成到 experiment_results/full_evasion_batch_时间戳）
```

python3 src/planning/planning/scripts/run_full_evasion_batch.py \
--runs 10 \
--run-duration 30 \
--warmup 3.0 \
--reacq-timeout-sec 10 \
--evasion-cmd "python3 src/planning/planning/scripts/free_run.py" \
--with-rviz
