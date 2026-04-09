# M3508 下位机接口文档（ROS2 对接）

## 1. 文档目的

本文档用于约定“上位机 ROS2 控制系统”与“嵌入式下位机（MCU）”之间的接口，保证以下目标：

- 上位机发布目标速度（`rad/s`）
- 下位机完成速度闭环并输出 C620 电流指令（`-16384 ~ 16384`）
- 下位机回传电机状态（位置/速度/电流/温度）
- 与现有 `dji_m3508_single` 包字段保持一致，便于无缝切换

## 2. 系统角色与边界

- 上位机（ROS2）：
  - 输入：`/velocity_controller/commands`
  - 输出观察：`/joint_states`
  - 通过串口/以太网向下位机发送速度目标

- 下位机（MCU）：
  - 接收上位机目标速度
  - 通过 CAN 与 C620 通信
  - 解析 `0x201` 反馈并回传上位机
  - 执行超时保护、限幅、急停策略

- 电调/电机：
  - 控制帧：`0x200`
  - 反馈帧：`0x201`（对应 motor_id=1）

## 3. 物理与链路层

### 3.1 MCU <-> C620（CAN）

- 协议：CAN2.0A（11-bit ID）
- 波特率：`1 Mbps`
- 控制帧 ID：`0x200`
- 反馈帧 ID：`0x201`

### 3.2 上位机 <-> MCU（建议）

可选实现：

- UART（推荐首版）：`921600 8N1`
- 或 UDP（以太网场景）

本文定义“应用层二进制协议”，可承载于 UART 或 UDP。

## 4. 应用层协议（上位机 <-> MCU）

### 4.1 帧格式（小端）

| 字段 | 长度(byte) | 说明 |
|---|---:|---|
| SOF | 2 | 固定 `0xAA 0x55` |
| VER | 1 | 协议版本，当前 `0x01` |
| TYPE | 1 | 消息类型 |
| SEQ | 2 | 包序号，循环计数 |
| TS_MS | 4 | 发送时间戳（ms） |
| LEN | 2 | Payload 长度 |
| PAYLOAD | N | 数据 |
| CRC16 | 2 | CRC-16/IBM（不含 SOF） |

### 4.2 消息类型

| TYPE | 方向 | 名称 | 说明 |
|---|---|---|---|
| `0x01` | 双向 | HEARTBEAT | 心跳保活 |
| `0x10` | 上位机->MCU | SET_VELOCITY | 设置目标速度 |
| `0x11` | 上位机->MCU | SET_MODE | 设置控制模式 |
| `0x20` | MCU->上位机 | MOTOR_STATE | 电机状态回传 |
| `0x21` | MCU->上位机 | DIAG_STATE | 诊断状态回传 |
| `0x30` | 上位机->MCU | PARAM_SET | 参数写入（PID/限幅） |
| `0x31` | 上位机->MCU | PARAM_GET | 参数读取 |
| `0x7F` | MCU->上位机 | ACK/NACK | 命令响应 |

## 5. 载荷定义

### 5.1 `SET_VELOCITY`（`0x10`）

| 字段 | 类型 | 单位 | 说明 |
|---|---|---|---|
| motor_id | `uint8` | - | `1` 对应 `0x201` |
| target_vel | `float32` | rad/s | 目标转速 |
| timeout_ms | `uint16` | ms | 命令超时时间，建议 `100` |

约束：

- 推荐范围：`-80 ~ +80 rad/s`（可按机构修订）
- 超范围输入由下位机钳位并返回 `ACK` 告警位

### 5.2 `SET_MODE`（`0x11`）

| 字段 | 类型 | 说明 |
|---|---|---|
| mode | `uint8` | `0=DISABLE, 1=VELOCITY, 2=ESTOP` |

状态机建议：

- `DISABLE`：下发零电流
- `VELOCITY`：按目标速度闭环
- `ESTOP`：立即零电流并锁定，需显式恢复

### 5.3 `MOTOR_STATE`（`0x20`）

| 字段 | 类型 | 单位 | 说明 |
|---|---|---|---|
| motor_id | `uint8` | - | 电机编号 |
| position | `float32` | rad | 多圈累计角度 |
| velocity | `float32` | rad/s | 当前速度 |
| current | `int16` | dq axis raw | 反馈电流原始值 |
| temperature | `uint8` | degC | 电机温度 |
| status_bits | `uint16` | bitfield | 状态位 |

`status_bits` 建议：

- bit0：命令超时
- bit1：过温
- bit2：CAN 反馈丢失
- bit3：速度饱和
- bit4：急停中

### 5.4 `DIAG_STATE`（`0x21`）

| 字段 | 类型 | 说明 |
|---|---|---|
| can_rx_hz | `uint16` | 反馈帧频率 |
| can_tx_hz | `uint16` | 控制帧频率 |
| last_err | `uint16` | 最近错误码 |
| bus_off_cnt | `uint16` | CAN bus-off 次数 |

### 5.5 `ACK/NACK`（`0x7F`）

| 字段 | 类型 | 说明 |
|---|---|---|
| ack_seq | `uint16` | 被响应命令的 SEQ |
| result | `uint8` | `0=OK, 1=WARN, 2=ERROR` |
| err_code | `uint16` | 错误码 |

## 6. CAN 帧定义（MCU <-> C620）

### 6.1 控制帧 `0x200`（MCU -> C620）

- DLC = 8
- 数据布局（4 电机共帧，big-endian int16）：

| Byte | 内容 |
|---:|---|
| 0-1 | 电机1电流指令 |
| 2-3 | 电机2电流指令 |
| 4-5 | 电机3电流指令 |
| 6-7 | 电机4电流指令 |

当前项目仅使用电机1：

- `byte0 = current_cmd >> 8`
- `byte1 = current_cmd & 0xFF`
- 其余字节置零

### 6.2 反馈帧 `0x201`（C620 -> MCU）

- DLC = 8
- 数据布局：

| Byte | 内容 | 类型 |
|---:|---|---|
| 0-1 | 编码器角度 | `uint16` |
| 2-3 | 转速 | `int16`（rpm） |
| 4-5 | 实际电流 | `int16` |
| 6 | 温度 | `uint8` |
| 7 | 保留 | `uint8` |

转换关系（与上位机代码一致）：

- `velocity_rad_s = rpm * 2*pi / 60`
- `position_rad = ticks / 8191 * 2*pi`（多圈累计）

## 7. 时序要求

- MCU 控制循环：`1 kHz`（推荐）
- 控制帧发送频率：`500~1000 Hz`
- 状态上报频率：`100 Hz`（上位机可视化足够）
- 心跳周期：`20 ms`
- 命令超时：`100 ms`（超时后自动零电流）

## 8. 下位机控制与保护策略

- 输入限幅：
  - 速度目标限幅
  - 电流输出限幅 `[-16384, 16384]`

- 失联保护：
  - 超过 `timeout_ms` 未收到 `SET_VELOCITY`，立即零电流

- 急停优先级最高：
  - 一旦进入 `ESTOP`，忽略速度指令

- 温度保护（建议）：
  - 温度阈值 85 degC 告警，90 degC 强制降额/停机

## 9. 参数项（建议固件持久化）

| 参数名 | 类型 | 默认值 | 说明 |
|---|---|---:|---|
| kp | `float32` | 10.0 | 速度环比例 |
| ki | `float32` | 0.1 | 速度环积分 |
| kd | `float32` | 0.01 | 速度环微分 |
| vel_limit | `float32` | 80.0 | 速度上限（rad/s） |
| cmd_timeout_ms | `uint16` | 100 | 命令超时 |

## 10. 错误码建议

| 错误码 | 含义 |
|---:|---|
| `0x0000` | 无错误 |
| `0x0001` | CRC 错误 |
| `0x0002` | 未知消息类型 |
| `0x0003` | 载荷长度错误 |
| `0x0101` | 速度超范围（已钳位） |
| `0x0201` | CAN 发送失败 |
| `0x0202` | CAN 反馈超时 |
| `0x0301` | 急停状态 |

## 11. 联调步骤（建议）

1. 下位机上电，CAN 口 `1Mbps`，周期发送 `DIAG_STATE`
2. 上位机建立通信并发送 `HEARTBEAT`
3. 上位机发送 `SET_MODE=VELOCITY`
4. 周期发送 `SET_VELOCITY`（例如 `1.0 rad/s`）
5. 监控 `MOTOR_STATE` 的速度、位置与状态位
6. 停止测试时发送 `SET_MODE=DISABLE`，验证零电流

## 12. 与 ROS2 包字段映射

| ROS2 字段 | 下位机字段 | 备注 |
|---|---|---|
| `joint1 velocity command` | `SET_VELOCITY.target_vel` | 单位同为 `rad/s` |
| `joint1 state velocity` | `MOTOR_STATE.velocity` | 单位同为 `rad/s` |
| `joint1 state position` | `MOTOR_STATE.position` | 单位同为 `rad` |
| 硬件参数 `motor_id` | `motor_id` | 默认值 `1` |

## 13. 版本管理建议

- 协议版本放在 `VER` 字段，新增字段时仅追加，不破坏旧字段顺序
- 建议保留至少 2 个版本兼容窗口
- 每次变更同步更新本文档与固件变更记录
