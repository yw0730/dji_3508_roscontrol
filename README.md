# DJI M3508 ROS 2 Control

基于 **ROS 2 Humble** + **ros2_control** 框架的 DJI M3508 无刷电机（配合 C620 电调）单轴速度控制驱动包。支持纯 ROS 仿真、Gazebo 仿真和 Linux SocketCAN 实机三种运行模式，无需切换代码，仅通过 launch 参数即可切换。

---

## 功能特性

- **三种运行模式**：mock 仿真 / Gazebo 仿真 / CAN 实机，统一接口无缝切换
- **内置速度 PID 控制器**：kp/ki/kd 可通过 URDF 参数在线配置
- **多圈位置累计**：基于编码器跨零点跳变自动计圈
- **电流斜率限制（Slew Rate）**：抑制扭矩突变，降低机械冲击
- **起转补偿（Startup Boost）**：克服静摩擦，保证小目标速度可靠起转
- **一阶低通滤波（LPF）**：平滑速度反馈，降低噪声对 PID 的影响
- **速度误差死区**：减少目标值附近的持续抖动
- **安全保护**：激活时自动清零历史误差，退出时下发零电流

---

## 目录结构

```
dji_3508_roscontrol/
├── README.md
├── MCU_DIAGNOSTICS.md          # CAN 诊断清单与参数调优指南
└── src/
    └── dji_m3508_single/
        ├── CMakeLists.txt
        ├── package.xml
        ├── dji_m3508_hardware.xml  # pluginlib 插件描述
        ├── STARTUP.md              # 启动说明文档
        ├── include/dji_m3508_single/
        │   ├── dji_m3508_hardware.hpp  # 硬件接口类声明
        │   └── can_utils.hpp           # SocketCAN 工具函数声明
        ├── src/
        │   ├── dji_m3508_hardware.cpp  # 硬件接口实现（PID、CAN 读写）
        │   └── can_utils.cpp           # SocketCAN 工具函数实现
        ├── config/
        │   ├── controllers.yaml        # ros2_control 控制器配置
        │   └── robot.urdf.xacro        # 机器人描述 & 硬件参数
        ├── launch/
        │   └── m3508.launch.py         # 统一启动文件
        └── worlds/
            └── m3508_empty.world       # Gazebo 空世界文件
```

---

## 依赖

| 依赖 | 版本 |
|------|------|
| ROS 2 | Humble |
| ros2_control / ros2_controllers | Humble |
| xacro | Humble |
| gazebo_ros / gazebo_ros2_control | Humble（Gazebo 仿真时需要） |
| Linux SocketCAN + can-utils | 实机运行时需要 |

安装 CAN 工具：

```bash
sudo apt update
sudo apt install -y can-utils iproute2
```

---

## 编译

```bash
cd ~/dji_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dji_m3508_single
source install/setup.bash
```

---

## 启动

### launch 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `use_fake_hardware` | bool | `true` | `true` 使用 mock 仿真，`false` 使用 CAN 实机 |
| `use_gazebo` | bool | `false` | `true` 启动 Gazebo 仿真（需同时 `use_fake_hardware:=true`） |

### 模式一：纯 ROS 仿真（mock）

```bash
ros2 launch dji_m3508_single m3508.launch.py use_fake_hardware:=true use_gazebo:=false
```

### 模式二：Gazebo 仿真

```bash
ros2 launch dji_m3508_single m3508.launch.py use_fake_hardware:=true use_gazebo:=true
```

### 模式三：CAN 实机

**第一步**：配置 CAN 接口（波特率 1 Mbps）

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

**第二步**：启动节点

```bash
ros2 launch dji_m3508_single m3508.launch.py use_fake_hardware:=false use_gazebo:=false
```

### 验证控制器状态

```bash
ros2 control list_controllers --controller-manager /controller_manager
```

期望输出：`joint_state_broadcaster` 和 `velocity_controller` 均为 `active`。

---

## 话题接口

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/velocity_controller/commands` | `std_msgs/msg/Float64MultiArray` | 输入 | 目标速度指令（rad/s） |
| `/joint_states` | `sensor_msgs/msg/JointState` | 输出 | 关节位置（rad）与速度（rad/s）反馈 |

### 发送速度指令示例

```bash
# 低速测试（20 rad/s）
ros2 topic pub -r 50 /velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [20.0]}"

# 提高速度（80 rad/s）
ros2 topic pub -r 100 /velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [80.0]}"

# 停止
ros2 topic pub -r 100 /velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0]}"
```

### 查看反馈

```bash
ros2 topic echo /joint_states
```

---

## 硬件参数配置

所有参数在 `config/robot.urdf.xacro` 中配置，无需重新编译，修改后重启节点即可生效。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `can_interface` | `can0` | Linux SocketCAN 接口名 |
| `motor_id` | `1` | C620 电调 ID（1~4） |
| `kp` | `80.0` | 速度误差比例增益 |
| `ki` | `0.0` | 积分增益（建议初始保持 0） |
| `kd` | `0.0` | 微分增益（建议初始保持 0） |
| `min_effective_current` | `350` | 起转补偿最小电流（原始值） |
| `max_current` | `16384` | 输出电流硬上限（C620 最大值） |
| `error_deadband` | `1.0` | 速度误差死区（rad/s） |
| `startup_error_threshold` | `8.0` | 触发起转补偿的误差阈值（rad/s） |
| `current_slew_rate_per_sec` | `4000.0` | 电流斜率限制（原始值/s） |
| `velocity_lpf_alpha` | `0.2` | 速度 LPF 系数（0~1，越小越平滑） |

---

## CAN 协议说明

驱动基于 DJI M3508 + C620 标准 CAN 协议：

- **控制帧 ID**：`0x200`，帧长 8 字节，按 `motor_id` 写入对应槽位（`(motor_id-1)*2` 字节偏移，高字节在前）
- **反馈帧 ID**：`0x200 + motor_id`（如 ID=1 时为 `0x201`），解析前 6 字节：
  - Byte 0-1：编码器角度（0~8191）
  - Byte 2-3：转速（rpm，有符号）
  - Byte 4-5：实际扭矩电流

---

## 快速诊断

实机不转时，可直接通过 CAN 发送电流帧排查硬件链路：

```bash
cansend can0 200#1000000000000000
```

- **能转**：硬件链路正常，检查 ROS 参数或控制器状态
- **不能转**：检查供电、线序、终端电阻或电调状态

监听 CAN 总线：

```bash
candump can0,200:7FF,201:7FF
```

详细诊断步骤与调参指南见 [`MCU_DIAGNOSTICS.md`](MCU_DIAGNOSTICS.md)。

---

## 可视化（rqt_plot）

```bash
# 安装（仅需一次）
sudo apt install -y ros-humble-rqt-plot

# 启动
source /opt/ros/humble/setup.bash
ros2 run rqt_plot rqt_plot
```

在输入框中添加曲线：
- 速度：`/joint_states/velocity[0]`
- 位置：`/joint_states/position[0]`

---

## 常见问题

| 现象 | 排查方向 |
|------|----------|
| 电机不动 | 确认 `use_fake_hardware:=false`；检查 CAN 接口是否 up |
| `can0` 不存在 | USB-CAN 未直通或驱动未加载（`lsusb` / `ip -brief link`） |
| 速度上不去 | 增大 `kp` 或 `min_effective_current` |
| 运行抖动 | 降低 `kp` 或 `current_slew_rate_per_sec` |
| 低速附近来回抖 | 增大 `error_deadband`，减小 `velocity_lpf_alpha` |
| 起步顿挫明显 | 减小 `min_effective_current` 或增大 `startup_error_threshold` |
| 控制器未 active | 检查 URDF 硬件参数格式；查看 `ros2 control list_controllers` |

---

## License

Apache-2.0
