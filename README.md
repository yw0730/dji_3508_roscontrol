dji_3508_roscontrol

DJI M3508 电机的 ROS2 ros2_control 硬件接口标准示例工程

本项目展示了如何按照 ros2_control 官方规范，为 DJI M3508 电机编写 SystemInterface 硬件插件，并实现：
	•	✅ Fake Hardware（纯软件调试）
	•	✅ Gazebo 仿真
	•	✅ 真实 SocketCAN 硬件控制

三种模式共用同一套 URDF + Controller + Hardware 架构，只需修改 launch 参数即可切换。

这是一个非常标准、可扩展到多电机底盘、云台、机械臂的 ros2_control 硬件架构

📁 目录结构

src/dji_m3508_single/
├── include/dji_m3508_single/
│   ├── dji_m3508_hardware.hpp      # ros2_control 硬件接口定义
│   └── can_utils.hpp               # SocketCAN 封装
├── src/
│   ├── dji_m3508_hardware.cpp      # read / write 核心逻辑
│   └── can_utils.cpp
├── config/
│   ├── robot.urdf.xacro            # ros2_control + URDF 绑定
│   └── controllers.yaml            # 控制器配置
├── launch/
│   └── m3508.launch.py             # 三模式统一启动入口
├── worlds/
│   └── m3508_empty.world           # Gazebo 世界
└── dji_m3508_hardware.xml         # pluginlib 导出


⸻

🧠 整体架构关系

JointGroupVelocityController
            │
            ▼
   ControllerManager (ros2_control)
            │
            ▼
   DjiM3508Hardware (SystemInterface)
            │
            ├── Fake 数据
            ├── GazeboSystem
            └── SocketCAN → 电调 → M3508


⸻

🧩 依赖环境
	•	Ubuntu 22.04
	•	ROS2 Humble
	•	Gazebo
	•	SocketCAN
	•	xacro

安装依赖：

sudo apt install ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-gazebo-ros2-control \
                 can-utils


⸻

🚀 编译

colcon build
source install/setup.bash


⸻

▶️ 三种运行模式

1️⃣ Fake Hardware（默认）

不接电机，验证控制链路：

ros2 launch dji_m3508_single m3508.launch.py

发送速度：

ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [10.0]}"


⸻

2️⃣ Gazebo 仿真模式

ros2 launch dji_m3508_single m3508.launch.py use_gazebo:=true use_fake_hardware:=false

此时硬件接口变为：

gazebo_ros2_control/GazeboSystem


⸻

3️⃣ 真实 DJI M3508 + SocketCAN

开启 CAN：

sudo ip link set can0 up type can bitrate 1000000

启动真实电机：

ros2 launch dji_m3508_single m3508.launch.py use_fake_hardware:=false

控制链路：

Controller → DjiM3508Hardware → SocketCAN → 电调 → M3508


⸻

⚙️ URDF 中的关键参数

robot.urdf.xacro：

<xacro:arg name="use_fake_hardware" default="true"/>
<xacro:arg name="use_gazebo" default="false"/>

通过这两个参数决定加载：
	•	Fake
	•	GazeboSystem
	•	真实 CAN 硬件插件

⸻

🎮 控制器

controllers.yaml 使用：
	•	joint_state_broadcaster
	•	JointGroupVelocityController

控制话题：

/velocity_controller/commands

接口类型：velocity

⸻

🔌 硬件接口实现说明（核心）

DjiM3508Hardware 实现标准接口：
	•	on_init
	•	export_state_interfaces
	•	export_command_interfaces
	•	read()：从 CAN 读取角度、转速
	•	write()：向 0x200 帧发送电流控制

这是一个标准 ros2_control 硬件插件写法模板。

⸻

🧪 本工程解决的典型问题

很多人困惑：
	•	“3508 怎么用 ROS2 控制？”
	•	“Gazebo 和真机怎么共用代码？”
	•	“ros2_control 硬件插件到底怎么写？”

本仓库完整展示了标准实现方式。


⸻

📄 License

Apache-2.0

⸻

👤 Author

yw0730
