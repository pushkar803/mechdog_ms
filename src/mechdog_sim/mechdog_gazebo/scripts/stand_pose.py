#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# Joint order matches controllers.yaml
# [fl_hip, fl_knee, fr_hip, fr_knee, rl_hip, rl_knee, rr_hip, rr_knee]
STAND = [0.20, -1.05,  0.20, -1.05,  -0.10, -1.10,  -0.10, -1.10]

class StandPose(Node):
    def __init__(self):
        super().__init__("mechdog_stand_pose")
        self.pub = self.create_publisher(Float64MultiArray, "/joint_group_position_controller/commands", 10)
        self.timer = self.create_timer(0.2, self.tick)
        self.count = 0

    def tick(self):
        msg = Float64MultiArray()
        msg.data = STAND
        self.pub.publish(msg)
        self.count += 1
        # publish a few times then stop
        if self.count >= 10:
            self.get_logger().info("Stand pose sent.")
            rclpy.shutdown()

def main():
    rclpy.init()
    node = StandPose()
    rclpy.spin(node)

if __name__ == "__main__":
    main()