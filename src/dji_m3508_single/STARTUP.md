# DJI M3508 单电机启动说明

## 1. 项目介绍

本包实现了一个面向 **单个 DJI M3508 电机**（CAN ID `0x201`）的完整 `ros2_control` 控制系统，支持以下三种运行方式：

- 纯 ROS 仿真：`mock_components/GenericSystem`
- Gazebo 仿真：`gazebo_ros2_control/GazeboSystem`
- 真实硬件：`dji_m3508_ros2/DjiM3508Hardware`
- 真实硬件（UART 协议）：`dji_m3508_ros2/DjiM3508UartHardware`

控制目标：

- 输入目标转速（单位 `rad/s`）

设计目标：

- 先在仿真验证控制链路，再切换实机；除硬件插件选择外，控制器与话题接口保持一致。

## 2. 工程结构

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

## 3. 运行模式

### 3.1 纯 ROS 仿真（建议先跑）

- 参数：`use_fake_hardware:=true use_gazebo:=false`
- 硬件插件：`mock_components/GenericSystem`
- 不需要 CAN 设备
- command 会回显到 state，便于快速验证控制逻辑

### 3.2 Gazebo 仿真

- 参数：`use_fake_hardware:=true use_gazebo:=true`
- 硬件插件：`gazebo_ros2_control/GazeboSystem`
- 会自动启动 Gazebo，并把机器人模型注入世界

### 3.3 实机模式

- 参数：`use_fake_hardware:=false use_gazebo:=false`
- 硬件插件：`dji_m3508_ros2/DjiM3508Hardware`
- 需要 Linux SocketCAN 与 C620 电调链路

### 3.4 实机 UART 模式

- 参数：`use_fake_hardware:=false use_gazebo:=false transport:=uart`
- 硬件插件：`dji_m3508_ros2/DjiM3508UartHardware`
- 串口参数：`serial_port`（默认 `/dev/ttyUSB0`）、`baudrate`（默认 `921600`）
- 命令超时参数：`cmd_timeout_ms`（默认 `100`）

## 4. 环境依赖

- ROS 2 Humble
- `ros2_control`、`ros2_controllers`、`xacro`
- Gazebo 仿真额外需要：`gazebo_ros`、`gazebo_ros2_control`
- 实机模式额外需要：
- Linux CAN 支持（`socketcan`）
- CAN 网口（默认 `can0`）
- 配置网口权限（通常使用 `sudo`）

## 5. 编译

在工作区根目录执行：

```bash
cd /home/wy/dji_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dji_m3508_single
source install/setup.bash
```

## 6. 启动仿真模式

### 6.1 纯 ROS 仿真

```bash
cd /home/wy/dji_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dji_m3508_single m3508.launch.py use_fake_hardware:=true use_gazebo:=false
```

### 6.2 Gazebo 仿真

```bash
ros2 launch dji_m3508_single m3508.launch.py use_fake_hardware:=true use_gazebo:=true
```

启动后检查控制器状态：

```bash
ros2 control list_controllers --controller-manager /controller_manager
```

期望看到处于 `active` 的控制器：

- `joint_state_broadcaster`
- `velocity_controller`

## 7. 一键速度指令测试

发布 `1.0 rad/s` 目标速度：

```bash
ros2 topic pub /velocity_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "data: [1.0]"
```

观察关节状态：

```bash
ros2 topic echo /joint_states
```

## 8. 让 Gazebo 里的电机明显转起来

在 Gazebo 启动后，持续发布较大速度指令（示例 `8.0 rad/s`）：

```bash
ros2 topic pub -r 50 /velocity_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [8.0]}"
```

另开终端验证速度反馈：

```bash
ros2 topic echo /joint_states
```

期望现象：

- `joint1.velocity` 接近 `8.0`
- `joint1.position` 持续增大
- Gazebo 中红色转子标记持续旋转

## 9. 启动实机模式

### 8.1 配置 CAN 口（1 Mbps）

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

### 8.2 启动实机控制

```bash
cd /home/wy/dji_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dji_m3508_single m3508.launch.py use_fake_hardware:=false
```

建议显式加上 `use_gazebo:=false`：

```bash
ros2 launch dji_m3508_single m3508.launch.py use_fake_hardware:=false use_gazebo:=false
```

## 10. 可调参数

在 `config/robot.urdf.xacro` 的实机分支中可修改：

- `can_interface`：默认 `can0`
- `motor_id`：默认 `1`（对应反馈帧 `0x201`）
- PID 参数：
- `kp`：默认 `10.0`
- `ki`：默认 `0.1`
- `kd`：默认 `0.01`

UART 模式参数（`transport:=uart`）：

- `serial_port`：默认 `/dev/ttyUSB0`
- `baudrate`：默认 `921600`
- `cmd_timeout_ms`：默认 `100`
- `motor_id`：默认 `1`

串口权限与设备检查：

```bash
ls -l /dev/ttyUSB*
groups
```

如果当前用户不在 `dialout` 组，可执行：

```bash
sudo usermod -aG dialout $USER
```

重新登录后生效。

UART 启动示例：

```bash
ros2 launch dji_m3508_single m3508.launch.py \
  use_fake_hardware:=false use_gazebo:=false \
  transport:=uart serial_port:=/dev/ttyUSB0 baudrate:=921600 cmd_timeout_ms:=100
```

ACK 结果码说明（与 MCU 协议一致）：

- `ACK=0`：OK，命令执行成功
- `ACK=1`：WARN，命令被限幅或降级执行（允许继续运行）
- `ACK=2`：ERROR，命令执行失败（应按错误处理）

## 11. 话题与接口

- 指令话题：
- `/velocity_controller/commands`
- 消息类型：`std_msgs/msg/Float64MultiArray`
- 数据格式：`[target_velocity_rad_s]`

- 状态话题：
- `/joint_states`
- 包含 `joint1` 的 position 与 velocity

## 12. 常见问题排查

- 控制器未激活：
- 运行 `ros2 control list_controllers` 查看状态
- 检查 launch 日志中两个 spawner 是否启动成功

- 实机无反馈：
- 检查 CAN 线束与供电
- 确认 CAN 波特率为 `1000000`
- 确认反馈 ID 为 `0x201`（`motor_id=1`）

- UART 模式启动失败或无状态回传：
- 检查 `transport:=uart`、`serial_port`、`baudrate` 参数是否匹配 MCU 配置
- 检查串口权限与占用：`ls -l /dev/ttyUSB*`、`lsof /dev/ttyUSB0`
- 查看日志中 ACK 结果：`ACK WARN` 表示降级执行，`ACK ERROR` 表示命令失败

- CAN 权限问题：
- 使用 `sudo` 配置 CAN 网口

- Gazebo 启动报 `bind: Address already in use`：
- 说明已有旧的 `gzserver` 占用了端口，先执行 `pkill -f gzserver` 后再启动

- Gazebo 窗口打开但看起来不转：
- 先确认没有暂停仿真（播放按钮处于运行状态）
- 用 `ros2 topic echo /joint_states` 确认 `joint1.velocity` 是否非零
- 增大指令到 `8.0` 或 `12.0 rad/s` 便于肉眼观察

## 13. 推荐验证流程

1. 编译工程
2. 以纯 ROS 仿真启动并确认 `/joint_states` 跟随指令变化
3. 以 Gazebo 仿真启动并确认模型在场景内旋转
4. 切换 `use_fake_hardware:=false use_gazebo:=false` 进入实机模式
5. 保持同一套控制器与话题接口进行联调
