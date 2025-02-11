#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn

class PX4Interface:
    def __init__(self):
        rospy.init_node('px4_interface')

        # Publisher to send RC override commands to Pixhawk
        self.rc_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

        # Subscriber to receive `cmd_vel` from laptop
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # Channel mappings
        self.surge_channel = 2   # Forward/Backward
        self.sway_channel = 3    # Left/Right
        self.heave_channel = 4   # Up/Down
        self.yaw_channel = 5     # Rotation

        self.rc_min = 1100  # Minimum RC signal
        self.rc_mid = 1500  # Neutral RC signal
        self.rc_max = 1900  # Maximum RC signal

    def cmd_vel_callback(self, twist_msg):
        rc_msg = OverrideRCIn()

        # Convert `cmd_vel` to RC PWM values
        surge = int(self.rc_mid + (twist_msg.linear.x * (self.rc_max - self.rc_mid)))
        sway = int(self.rc_mid + (twist_msg.linear.y * (self.rc_max - self.rc_mid)))
        heave = int(self.rc_mid + (twist_msg.linear.z * (self.rc_max - self.rc_mid)))
        yaw = int(self.rc_mid + (twist_msg.angular.z * (self.rc_max - self.rc_mid)))

        # Ensure RC values are within bounds
        surge = max(self.rc_min, min(self.rc_max, surge))
        sway = max(self.rc_min, min(self.rc_max, sway))
        heave = max(self.rc_min, min(self.rc_max, heave))
        yaw = max(self.rc_min, min(self.rc_max, yaw))

        # Assign values to the correct RC channels
        rc_msg.channels = [1500] * 8  # Default to neutral for all channels
        rc_msg.channels[self.surge_channel] = surge
        rc_msg.channels[self.sway_channel] = sway
        rc_msg.channels[self.heave_channel] = heave
        rc_msg.channels[self.yaw_channel] = yaw

        # Publish to Pixhawk
        self.rc_pub.publish(rc_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = PX4Interface()
    node.run()
