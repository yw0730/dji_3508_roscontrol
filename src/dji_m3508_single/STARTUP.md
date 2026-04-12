# DJI M3508 单电机启动说明（仿真 + CAN）

## 1. 功能概览

本包仅保留两类运行方式：

- 纯 ROS 仿真：`mock_components/GenericSystem`
- Gazebo 仿真：`gazebo_ros2_control/GazeboSystem`
- 真实硬件（CAN）：`dji_m3508_ros2/DjiM3508Hardware`

控制接口统一为速度控制：

- 指令话题：`/velocity_controller/commands`
- 指令类型：`std_msgs/msg/Float64MultiArray`
- 数据格式：`{data: [target_velocity_rad_s]}`

## 2. 项目结构

```text
dji_m3508_single/
├── CMakeLists.txt
├── package.xml
├── dji_m3508_hardware.xml
├── include/dji_m3508_single/
│   ├── dji_m3508_hardware.hpp
│   └── can_utils.hpp
├── src/
│   ├── dji_m3508_hardware.cpp
│   └── can_utils.cpp
├── config/
│   ├── controllers.yaml
│   └── robot.urdf.xacro
└── launch/
    └── m3508.launch.py
```

## 3. 环境依赖

- ROS 2 Humble
- `ros2_control`、`ros2_controllers`、`xacro`
- Gazebo 仿真：`gazebo_ros`、`gazebo_ros2_control`
- CAN 实机：Linux SocketCAN + `can-utils`

安装 CAN 工具：

```bash
sudo apt update
sudo apt install -y can-utils iproute2
```

## 4. 编译

```bash
cd /home/wy/dji_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dji_m3508_single
source install/setup.bash
```

## 5. 仿真启动

### 5.1 纯 ROS 仿真

```bash
cd /home/wy/dji_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dji_m3508_single m3508.launch.py use_fake_hardware:=true use_gazebo:=false
```

### 5.2 Gazebo 仿真

```bash
ros2 launch dji_m3508_single m3508.launch.py use_fake_hardware:=true use_gazebo:=true
```

检查控制器：

```bash
ros2 control list_controllers --controller-manager /controller_manager
```

期望：

- `joint_state_broadcaster` 为 `active`
- `velocity_controller` 为 `active`

## 6. CAN 实机启动

### 6.1 配置 CAN 口（1 Mbps）

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

### 6.2 启动实机控制

```bash
cd /home/wy/dji_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dji_m3508_single m3508.launch.py use_fake_hardware:=false use_gazebo:=false
```

## 7. 当前默认 CAN 参数

参数位置：`config/robot.urdf.xacro`

- `can_interface: can0`
- `motor_id: 1`
- `kp: 80.0`
- `ki: 0.0`
- `kd: 0.0`
- `min_effective_current: 350`
- `max_current: 16384`
- `error_deadband: 1.0`
- `startup_error_threshold: 8.0`
- `current_slew_rate_per_sec: 4000.0`
- `velocity_lpf_alpha: 0.2`

## 8. 发命令测试

先小速度：

```bash
ros2 topic pub -r 50 /velocity_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [20.0]}"
```

再提高速度：

```bash
ros2 topic pub -r 100 /velocity_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [80.0]}"
```

## 9. 联调观察

查看反馈状态：

```bash
ros2 topic echo /joint_states
```

抓 CAN 控制与反馈帧：

```bash
candump can0,200:7FF,201:7FF
```

判据：

- `0x200` 对应槽位电流非零且变化平滑
- `0x201` 的 rpm 非零并随命令变化

## 10. 使用 rqt_plot 可视化曲线

### 10.1 安装 rqt_plot（只需一次）

```bash
sudo apt update
sudo apt install -y ros-humble-rqt-plot
```

### 10.2 新终端加载环境

```bash
source /opt/ros/humble/setup.bash
source /home/wy/dji_ws/install/setup.bash
```

### 10.3 启动 rqt_plot

```bash
ros2 run rqt_plot rqt_plot
```

### 10.4 添加曲线

在输入框中填入以下任意一项并回车：

- 速度：`/joint_states/velocity[0]`
- 位置：`/joint_states/position[0]`

### 10.5 联合测试方法

一边发送命令：

```bash
ros2 topic pub -r 100 /velocity_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [80.0]}"
```

一边观察 `rqt_plot`：

- 速度曲线会快速上升
- 稳态应在目标值附近（如 80 rad/s 附近）

## 11. 常见问题

1. 电机不动：确认不是 `use_fake_hardware:=true`。
2. can0 不存在：USB-CAN 未直通或驱动未加载。
3. 抖动明显：优先降低 `kp` 或 `current_slew_rate_per_sec`。
