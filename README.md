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
   在 RViz 中使用 "2D Nav Goal" 工具设置目标无人机的目的地，目标无人机将移动到指定位置。

6. **监控无人机群**
   观察 7 架追踪无人机形成六边形编队并跟随目标无人机。

### 注意事项
- 无人机的初始位置在 `tracking_sim.launch` 文件中定义。
- 确保所有参数在启动文件和配置文件中正确设置。

更多详情请参考项目文档。
