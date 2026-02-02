#!/usr/bin/env python3
"""
从远程Master机器通过rosbridge接收关节状态,并在本地重新发布
"""

import rospy
import roslibpy
from sensor_msgs.msg import JointState
import sys

# Fix for Twisted logging compatibility with Python 3.8+
import logging
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

# Disable Twisted logging to Python logging bridge
import twisted.python.log
twisted.python.log.PythonLoggingObserver._oldFindCaller = logging.Logger.findCaller
def _findCaller(self, stack_info=False, stacklevel=1):
    return twisted.python.log.PythonLoggingObserver._oldFindCaller(self, stack_info)
logging.Logger.findCaller = _findCaller


class RosbridgeSlaverRelay:
    def __init__(self, master_ip, master_port=9090, topic_name='dual_arms_joint_states'):  #原来是master_controller_manipulator_state
        self.master_ip = master_ip
        self.master_port = master_port
        self.topic_name = topic_name

        # 确保话题名称格式正确
        self.remote_topic = f'/{topic_name}' if not topic_name.startswith('/') else topic_name
        self.local_topic = topic_name.lstrip('/')

        # 初始化ROS节点
        rospy.init_node('rosbridge_slaver_relay', anonymous=False)

        # 创建本地发布器
        self.local_pub = rospy.Publisher(
            self.local_topic,
            JointState,
            queue_size=10
        )

        # 连接到远程rosbridge
        self.client = None
        self.listener = None
        self.connect_to_master()

        rospy.loginfo("\033[1;32mRosbridge Slaver Relay Started\033[0m")
        rospy.loginfo(f"Connected to Master: ws://{master_ip}:{master_port}")
        rospy.loginfo(f"Subscribing to remote: {self.remote_topic}")
        rospy.loginfo(f"Publishing to local: {self.local_topic}")

    def connect_to_master(self):
        #连接到Master机器的rosbridge服务器
        try:
            # 创建rosbridge客户端
            self.client = roslibpy.Ros(
                host=self.master_ip,
                port=self.master_port
            )

            # 连接
            self.client.run()
            rospy.loginfo(f"Successfully connected to rosbridge at {self.master_ip}:{self.master_port}")

            # 订阅远程话题
            self.listener = roslibpy.Topic(
                self.client,
                self.remote_topic,
                'sensor_msgs/JointState'
            )

            self.listener.subscribe(self.joint_state_callback)
            rospy.loginfo(f"Subscribed to remote topic: {self.remote_topic}")

        except Exception as e:
            rospy.logerr(f"Failed to connect to rosbridge: {e}")
            sys.exit(1)

    def joint_state_callback(self, message):
        #接收远程关节状态并在本地重新发布
        try:
            # 转换rosbridge消息为ROS消息
            joint_state = JointState()

            # 设置时间戳
            joint_state.header.stamp = rospy.Time.now()
            joint_state.header.frame_id = message.get('header', {}).get('frame_id', '')

            # 设置关节数据
            joint_state.name = message.get('name', [])
            joint_state.position = message.get('position', [])
            joint_state.velocity = message.get('velocity', [])
            joint_state.effort = message.get('effort', [])

            # 在本地发布
            self.local_pub.publish(joint_state)

            # 可选：打印调试信息
            if rospy.get_param('~debug', False):
                rospy.loginfo(f"Relayed joint state with {len(joint_state.name)} joints from {self.topic_name}")

        except Exception as e:
            rospy.logerr(f"Error in callback: {e}")

    def run(self):
        """保持节点运行"""
        rospy.loginfo("Relay node is running...")
        # 使用rospy.spin()保持节点运行，接收rosbridge消息
        rospy.spin()

        # 清理资源
        if self.listener:
            self.listener.unsubscribe()
        if self.client:
            self.client.terminate()

def main():
    if len(sys.argv) < 2:
        rospy.logerr("Usage: rosbridge_slaver_relay.py <master_ip> [port] [topic_name]")
        rospy.logerr("Example: rosbridge_slaver_relay.py 192.168.1.100 9090 master_controller_manipulator_state")
        rospy.logerr("")
        rospy.logerr("Available topics:")
        rospy.logerr("  - master_controller_manipulator_state (双臂)")
        rospy.logerr("  - master_controller_manipulator_left_state (左臂)")
        rospy.logerr("  - master_controller_manipulator_right_state (右臂)")
        rospy.logerr("  - master_controller_head5_state (头部)")
        rospy.logerr("  - master_controller_waist_state (腰部)")
        sys.exit(1)

    master_ip = sys.argv[1]
    master_port = int(sys.argv[2]) if len(sys.argv) > 2 else 9090
    topic_name = sys.argv[3] if len(sys.argv) > 3 else 'master_controller_manipulator_state'

    try:
        relay = RosbridgeSlaverRelay(master_ip, master_port, topic_name)
        relay.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
