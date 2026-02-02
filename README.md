# Master-Slaver Controller

基于ROS的主从控制系统，用于机器人双臂、腰部和头部的运动控制。

## 系统架构

### Master-Slaver模式
- **Master Controller**: 读取电机状态并发布到ROS话题
- **Slaver Controller**: 订阅话题并控制电机跟随

### 跨机器通信 (Rosbridge)

支持通过rosbridge在不同机器间传输数据：

```
机器A (Master)                    机器B (Slaver)
┌─────────────┐                  ┌─────────────┐
│   Master    │                  │   Slaver    │
│ Controller  │                  │ Controller  │
└──────┬──────┘                  └──────▲──────┘
       │                                │
       ▼                                │
┌─────────────┐                  ┌─────────────┐
│  Rosbridge  │ ─────网络──────>  │   Relay     │
│   Server    │   (WebSocket)    │    Node     │
└─────────────┘                  └─────────────┘
```

**配置要求**：每台机器需独立配置ROS环境
```bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
```

---

## 🆕 统一双臂系统（推荐）

### 系统特点

- ✅ **统一配置**：使用单个配置文件 `pi_plus_arm_cfg.yaml` 控制左右手臂（12个电机）
- ✅ **统一话题**：主机发布单个话题 `/dual_arms_joint_states` 包含双臂状态
- ✅ **统一重力补偿**：一次性计算左右手臂的重力补偿力矩
- ✅ **同步控制**：从机同时控制左右手臂，完全同步

### 文件说明

#### 主机文件 (192.168.100.189)

**源代码：**
- `src/dual_arms_motor_controller.cpp` - 双臂电机控制器
  - 同时读取12个电机状态（左臂6个 + 右臂6个）
  - 发布统一话题：`/dual_arms_joint_states`
  - 订阅重力补偿：`/dual_arms_gravity_compensation_torques`
  - 应用重力补偿到12个电机

- `src/dual_arms_gravity_compensation.cpp` - 双臂重力补偿计算器
  - 使用Pinocchio库计算双臂重力补偿
  - 订阅：`/dual_arms_joint_states`
  - 发布：`/dual_arms_gravity_compensation_torques`

**Launch文件：**
- `launch/rosbridge_master.launch` - ROS Bridge服务器（端口9090）
- `launch/dual_arms_gravity_comp.launch` - 双臂重力补偿系统

**配置文件：**
- `motor_cfg/pi_plus_arm_cfg.yaml` - 统一的双臂电机配置
  - CANport_1 (serial_id: 3) - 左手臂6个电机
  - CANport_2 (serial_id: 4) - 右手臂6个电机

#### 从机文件 (192.168.100.109)

**源代码：**
- `src/dual_arms_slaver_impedance.cpp` - 双臂阻抗控制器
  - 订阅主机话题：`/dual_arms_joint_states`
  - 同时控制12个电机的阻抗
  - 发布从机状态：`/slave_dual_arms_joint_states`
  - 实现柔顺跟随控制

**Launch文件：**
- `launch/dual_arms_slaver_impedance.launch` - 双臂阻抗控制系统
  - 包含ROS Bridge中继节点
  - 包含双臂阻抗控制器

**配置文件：**
- `motor_cfg/pi_plus_arm_cfg.yaml` - 统一的双臂电机配置（与主机相同）

### 使用方法

#### 主机 (192.168.100.189)

```bash
# 终端1: 启动ROS Bridge服务器
roslaunch master_slaver_controller rosbridge_master.launch

# 终端2: 启动双臂重力补偿系统
roslaunch master_slaver_controller dual_arms_gravity_comp.launch
```

#### 从机 (192.168.100.109)

```bash
# 只需一个命令！启动双臂阻抗跟随控制
roslaunch master_slaver_controller dual_arms_slaver_impedance.launch
```

### 参数配置

#### 重力补偿参数 (`dual_arms_gravity_comp.launch`)

```xml
<arg name=compensation_scale default=0.8/>   <!-- 重力补偿缩放系数 -->
<arg name=enable_compensation default=true/> <!-- 是否启用重力补偿 -->
<arg name=torque_scale default=1.0/>         <!-- 力矩缩放系数 -->
<arg name=control_rate default=400/>         <!-- 控制频率 (Hz) -->
```

#### 阻抗控制参数 (`dual_arms_slaver_impedance.launch`)

```xml
<arg name=stiffness_kp default=1.0/>         <!-- 位置刚度 -->
<arg name=damping_kd default=0.2/>           <!-- 速度阻尼 -->
<arg name=max_torque default=10.0/>          <!-- 最大力矩限制 (Nm) -->
<arg name=control_rate default=400/>         <!-- 控制频率 (Hz) -->
<arg name=enable_feedforward default=false/> <!-- 前馈控制（当前未实现）-->
```

**参数调节建议：**
- `stiffness_kp`: 增大使跟随更紧，但可能变硬；减小使更柔顺
- `damping_kd`: 增大可减少震荡，但可能变慢；减小响应更快但可能不稳定
- `compensation_scale`: 调节重力补偿强度，范围 0.0-1.0

### 运动学映射

#### 左手臂 (motors 0-5)
- motor0 ↔ l_shoulder_pitch (直接映射)
- motor1, motor2 ↔ l_shoulder_roll, l_shoulder_yaw (锥齿轮)
- motor3, motor4 ↔ l_elbow_pitch, l_elbow_yaw (锥齿轮)
- motor5 ↔ l_wrist (直接映射，符号取反)

#### 右手臂 (motors 6-11)
- motor6 ↔ r_shoulder_pitch (直接映射，符号取反)
- motor7, motor8 ↔ r_shoulder_roll, r_shoulder_yaw (锥齿轮)
- motor9, motor10 ↔ r_elbow_pitch, r_elbow_yaw (锥齿轮)
- motor11 ↔ r_wrist (直接映射，符号取反)

### 话题说明

| 话题名称 | 类型 | 发布者 | 订阅者 | 说明 |
|---------|------|--------|--------|------|
| `/dual_arms_joint_states` | sensor_msgs/JointState | dual_arms_motor_controller | dual_arms_gravity_compensation, dual_arms_slaver_impedance | 双臂关节状态（12个关节） |
| `/dual_arms_gravity_compensation_torques` | sensor_msgs/JointState | dual_arms_gravity_compensation | dual_arms_motor_controller | 双臂重力补偿力矩 |
| `/slave_dual_arms_joint_states` | sensor_msgs/JointState | dual_arms_slaver_impedance | - | 从机双臂状态（用于监控） |

---
