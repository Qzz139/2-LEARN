#!/usr/bin/env python3
"""Normalize Foxy gazebo_ros2_control's `_mimic` hardware-state suffix.

The raw topic remains available for diagnosis. Values and timestamps are actual
Gazebo feedback, not synthesized commands.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# 将硬件反馈中的 mimic 后缀统一为 URDF 关节名。
class JointStates(Node):
    def __init__(self):
        super().__init__('ep_joint_state_adapter')
        self.pub = self.create_publisher(JointState, '/ep_joint_states', 10)
        self.create_subscription(JointState, '/joint_states', self.receive, 10)

    # 仅改关节名称，保留原始位置、速度、力矩和时间戳。
    def receive(self, msg):
        msg.name = [name[:-6] if name.endswith('_mimic') else name for name in msg.name]
        self.pub.publish(msg)


# 运行 ROS 回调循环，并在中断后释放节点。
def main():
    rclpy.init(); node = JointStates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
