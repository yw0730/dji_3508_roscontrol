# CAN 诊断清单（M3508 + C620）

## 1. 接入检查

```bash
lsusb
ip -brief link
```

期望：

- 能看到 USB-CAN 设备
- 存在 `can0`

## 2. CAN 口配置

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

## 3. ROS 启动检查

```bash
cd /home/wy/dji_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dji_m3508_single m3508.launch.py use_fake_hardware:=false use_gazebo:=false
```

另开终端：

```bash
ros2 control list_controllers --controller-manager /controller_manager
```

期望：

- `joint_state_broadcaster` 和 `velocity_controller` 都是 `active`

## 4. 命令链路检查

持续发送速度命令：

```bash
ros2 topic pub -r 100 /velocity_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [80.0]}"
```

抓控制与反馈：

```bash
candump can0,200:7FF,201:7FF
```

判据：

- `0x200`：第 1 电机槽位（前两字节）应非零
- `0x201`：rpm 字段非零，并随命令变化

## 5. 快速隔离法

直发电流（绕过 ROS）：

```bash
cansend can0 200#1000000000000000
```

- 能转：硬件链路 OK，优先调 ROS 参数
- 不能转：查供电/线序/终端电阻/电调状态

## 6. 参数调优建议

参数在 `config/robot.urdf.xacro`：

- 抖动大：降低 `kp` 或 `current_slew_rate_per_sec`
- 起转困难：增大 `min_effective_current`
- 低速抖动：增大 `error_deadband` 或减小 `velocity_lpf_alpha`

## 7. 参数含义（CAN 模式）

以下参数位于 `dji_m3508_ros2/DjiM3508Hardware` 分支：

- `kp`：速度误差到电流输出的比例增益。越大响应越快，但更容易抖动和过冲。
- `ki`：积分增益。用于消除稳态误差；过大易积累导致低频摆动。当前建议先保持 `0.0`。
- `kd`：微分增益。抑制快速变化；在噪声较大的速度反馈上容易放大抖动。当前建议先保持 `0.0`。
- `min_effective_current`：起转补偿下限电流。误差较大且低速时，至少给到该电流避免卡在静摩擦区。
- `max_current`：输出电流硬限幅，防止指令超过 C620 允许范围。
- `error_deadband`：速度误差死区。误差在该范围内按 0 处理，可减少围绕目标值抖动。
- `startup_error_threshold`：触发起转补偿的误差阈值。只有误差足够大才启用 `min_effective_current`。
- `current_slew_rate_per_sec`：电流斜率限制，约束每秒电流变化量，降低扭矩突变。
- `velocity_lpf_alpha`：速度反馈一阶低通滤波系数（0~1）。越小越平滑但延迟更大；越大响应更快但更噪。

## 8. 调参技巧（实操顺序）

先固定基础条件：

- 波特率固定 `1Mbps`，总线稳定。
- 命令发布频率固定（建议 `50~100Hz`）。
- 每次只改 1 个参数，记录改动与现象。

推荐顺序：

1. 先把 `ki=0`、`kd=0`，只调 `kp`。
2. 调 `kp` 到"刚好够快但不明显抖"。
3. 低速发抖时先调 `error_deadband`（小步增加）。
4. 起转困难再调 `min_effective_current`（每次加 50~100）。
5. 扭矩变化突兀再降 `current_slew_rate_per_sec`。
6. 反馈噪声大就减小 `velocity_lpf_alpha`（如 `0.3 -> 0.2 -> 0.15`）。

现象 -> 优先参数：

- 速度上不去：`kp` 增大，必要时增大 `min_effective_current`。
- 能转但抖：`kp` 降低，或降低 `current_slew_rate_per_sec`。
- 低速附近来回抖：增大 `error_deadband`，减小 `velocity_lpf_alpha`。
- 起步有明显顿挫：减小 `min_effective_current` 或增大 `startup_error_threshold`。
- 命令突变时冲击大：降低 `current_slew_rate_per_sec`。

## 9. 推荐微调步长

- `kp`：每次 `5~10`
- `min_effective_current`：每次 `50~100`
- `error_deadband`：每次 `0.1~0.2`
- `startup_error_threshold`：每次 `1~2`
- `current_slew_rate_per_sec`：每次 `500~1000`
- `velocity_lpf_alpha`：每次 `0.05`

调参结束前务必验证：

- 连续运行 2~3 分钟无明显温升异常。
- 停止命令时可稳定回到低电流状态。
- `candump can0,200:7FF,201:7FF` 中 `0x200` 变化平滑、`0x201` rpm 波动可接受。
