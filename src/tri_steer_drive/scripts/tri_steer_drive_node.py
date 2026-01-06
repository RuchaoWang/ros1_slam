#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

def wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class TriSteerDrive:
    """
    三舵轮底盘驱动（Twist -> 3个舵向角 + 3个驱动轮角速度）
    轮位布局：1号 0deg，2号 120deg，3号 240deg（顺时针/逆时针只要与你模型一致）
    """
    def __init__(self):
        # ====== 参数 ======
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.05)  # m
        self.R = rospy.get_param("~R", 0.20)                        # 轮子到中心半径 m
        self.cmd_timeout = rospy.get_param("~cmd_timeout", 0.25)    # s
        self.v_limit = rospy.get_param("~v_limit", 0.6)             # m/s
        self.w_limit = rospy.get_param("~w_limit", 1.5)             # rad/s
        self.rate_hz = rospy.get_param("~rate", 50.0)

        self.invert_y  = rospy.get_param("~invert_y", False)
        self.invert_wz = rospy.get_param("~invert_wz", False)

        # 轮角（rad）
        # 默认：1=0deg, 2=120deg, 3=240deg
        self.theta1 = math.radians(rospy.get_param("~wheel1_deg", 0.0))
        self.theta2 = math.radians(rospy.get_param("~wheel2_deg", 120.0))
        self.theta3 = math.radians(rospy.get_param("~wheel3_deg", 240.0))

        # ====== 关节话题（可按你的controller名字改）======
        j11 = rospy.get_param("~joint_steer_1", "/joint11_position_controller/command")
        j21 = rospy.get_param("~joint_steer_2", "/joint21_position_controller/command")
        j31 = rospy.get_param("~joint_steer_3", "/joint31_position_controller/command")

        j12 = rospy.get_param("~joint_drive_1", "/joint12_velocity_controller/command")
        j22 = rospy.get_param("~joint_drive_2", "/joint22_velocity_controller/command")
        j32 = rospy.get_param("~joint_drive_3", "/joint32_velocity_controller/command")

        self.pub_steer1 = rospy.Publisher(j11, Float64, queue_size=1)
        self.pub_steer2 = rospy.Publisher(j21, Float64, queue_size=1)
        self.pub_steer3 = rospy.Publisher(j31, Float64, queue_size=1)

        self.pub_drive1 = rospy.Publisher(j12, Float64, queue_size=1)
        self.pub_drive2 = rospy.Publisher(j22, Float64, queue_size=1)
        self.pub_drive3 = rospy.Publisher(j32, Float64, queue_size=1)

        # ====== 输入速度 ======
        self.last_cmd_time = rospy.Time(0)
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        in_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
        rospy.Subscriber(in_topic, Twist, self.cb_cmd, queue_size=1)

        rospy.loginfo("tri_steer_drive started. cmd_topic=%s", in_topic)

    def cb_cmd(self, msg: Twist):

        vx = clamp(msg.linear.x, -self.v_limit, self.v_limit)
        vy = clamp(msg.linear.y, -self.v_limit, self.v_limit)
        wz = clamp(msg.angular.z, -self.w_limit, self.w_limit)

        # 只修左右和旋转（不动前后）
        if self.invert_y:
            vy = -vy
        if self.invert_wz:
            wz = -wz

        self.vx = vx
        self.vy = vy
        self.wz = wz

        self.last_cmd_time = rospy.Time.now()

    def wheel_cmd(self, theta: float):
        # 轮子位置 r=[Rcos, Rsin]
        x = self.R * math.cos(theta)
        y = self.R * math.sin(theta)

        # v_i = [vx, vy] + wz x r = [vx - wz*y, vy + wz*x]
        vix = self.vx - self.wz * y
        viy = self.vy + self.wz * x

        steer = math.atan2(viy, vix)  # 舵角
        speed = math.sqrt(vix*vix + viy*viy)  # 线速度
        wheel_w = speed / max(1e-6, self.wheel_radius)  # 角速度

        return wrap_to_pi(steer), wheel_w

    def publish_zero(self):
        self.pub_steer1.publish(Float64(0.0))
        self.pub_steer2.publish(Float64(0.0))
        self.pub_steer3.publish(Float64(0.0))
        self.pub_drive1.publish(Float64(0.0))
        self.pub_drive2.publish(Float64(0.0))
        self.pub_drive3.publish(Float64(0.0))

    def spin(self):
        r = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if (rospy.Time.now() - self.last_cmd_time).to_sec() > self.cmd_timeout:
                # 超时自动刹停
                self.vx = self.vy = self.wz = 0.0

            s1, w1 = self.wheel_cmd(self.theta1)
            s2, w2 = self.wheel_cmd(self.theta2)
            s3, w3 = self.wheel_cmd(self.theta3)

            self.pub_steer1.publish(Float64(s1))
            self.pub_steer2.publish(Float64(s2))
            self.pub_steer3.publish(Float64(s3))
            self.pub_drive1.publish(Float64(w1))
            self.pub_drive2.publish(Float64(w2))
            self.pub_drive3.publish(Float64(w3))

            r.sleep()

if __name__ == "__main__":
    rospy.init_node("tri_steer_drive_node", anonymous=False)
    node = TriSteerDrive()
    node.spin()
