# MCU 诊断清单

## 步骤1：物理连接检查
```bash
# 确保USB线已连接，然后运行
lsusb | grep -i "usb"
lsusb | grep -i "1a86"  # 查找CH340芯片
```
**预期**：看到 `1a86:7523` (CH340/CH341)

---

## 步骤2：驱动/设备节点检查
```bash
# 检查设备节点是否存在
ls -la /dev/ttyUSB*
ls -la /dev/serial/by-id/

# 检查驱动是否加载
lsmod | grep ch341
lsmod | grep usbserial
```
**预期**：
- `/dev/ttyUSB0` 存在，权限 `crw-rw---- root:dialout`
- `ch341` 和 `usbserial` 模块已加载

---

## 步骤3：MCU活跃性检查（关键！）
**打开新终端，运行这个命令监听串口数据：**
```bash
# 使用strace/dtrace追踪串口读取
sudo strace -e openat,read,write -f -p $(pgrep ros2_control_node) 2>&1 | grep -i ttyUSB

# 或使用tio监听（需要安装：sudo apt install tio）
tio /dev/ttyUSB0
```

**或者用Python快速测试MCU是否在发送数据：**
```bash
python3 << 'PY'
import serial
import time

try:
    ser = serial.Serial('/dev/ttyUSB0', 921600, timeout=0.5)
    ser.write(b'\xAA\x55')  # 发送SOF字节
    time.sleep(0.1)
    
    data = ser.read(100)
    if data:
        print(f"✓ MCU响应！收到 {len(data)} 字节: {data.hex()}")
    else:
        print("✗ MCU无响应（超时）- MCU可能未运行或未连接")
    ser.close()
except Exception as e:
    print(f"✗ 错误: {e}")
PY
```

---

## 步骤4：查看SET_MODE帧是否真的被发送

**在ROS2启动时捕获串口数据：**
```bash
# 先在一个终端启动监听（使用hexdump）
sudo cat /dev/ttyUSB0 | xxd

# 然后在另一个终端运行ROS2启动
newgrp dialout
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dji_m3508_single m3508.launch.py \
  use_fake_hardware:=false use_gazebo:=false \
  transport:=uart
```

**或使用tcpdump/strace查看底层write调用：**
```bash
sudo strace -e write -s 200 -f -p $(pgrep ros2_control_node) 2>&1 | grep -A2 "ttyUSB\|AA55"
```

---

## 步骤5：增加ROS2日志级别看详细信息

```bash
newgrp dialout
source /opt/ros/humble/setup.bash
source install/setup.bash

# 设置高详度日志
export ROS_LOG_DIR=/tmp/ros_debug_$(date +%s)
mkdir -p $ROS_LOG_DIR

# 运行launch（会输出更多调试信息）
ros2 launch dji_m3508_single m3508.launch.py \
  use_fake_hardware:=false use_gazebo:=false \
  transport:=uart \
  --log-level DEBUG

# 查看日志
tail -f $ROS_LOG_DIR/ros2_control_node-?/*.log
```

---

## 快速诊断决策树

```
MCU连接？
├─ 否 → 检查USB线、驱动（步骤1-2）
└─ 是 → MCU运行中？
   ├─ 否 → MCU代码未烧写或MCU已死机，需重新启动/烧写
   └─ 是 → SET_MODE帧发送了吗？
      ├─ 否 → ROS插件问题（检查cpp代码）
      └─ 是 → MCU未响应ACK
         ├─ 检查MCU端口监听代码
         ├─ 检查MCU波特率设置(需要921600)
         └─ 检查序列号匹配（SET_MODE seq=0 → MCU回复ACK seq=0？）
```

---

## 诊断输出示例

**表现1：MCU存在且运行**
```
$ python3 test.py
✓ MCU响应！收到 14 字节: aa55010120000000000100...
```
→ 问题在ROS插件或协议解析

**表现2：MCU超时无响应**
```
$ python3 test.py
✗ MCU无响应（超时）- MCU可能未运行或未连接
```
→ 检查MCU硬件/驱动、或增加超时重试

**表现3：SET_MODE帧未发送**
```
$ sudo cat /dev/ttyUSB0 | xxd
(无数据输出)
```
→ ROS2控制节点可能在earlier阶段就失败了，检查日志

---

## 我的建议

1. **立即运行步骤3的Python脚本** ← 这个最快确定MCU状态
2. 如果MCU无响应 → 检查你的下位机代码是否已烧写到MCU
3. 如果MCU有响应 → 增加SET_MODE超时时间（100ms→500ms）
