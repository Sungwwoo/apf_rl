import rclpy
from apf_interfaces.srv import SetWeights, GetForces, GetWeights
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


# TODO isaac sim api


class RobotEnv(Node):
    def __init__(self):
        super().__init__("rl_wrapper")

        self.robot_pose = [0.0] * 3
        self.robot_twist = [0.0] * 2
        self.laser_scan = [0.0] * 811
        self.weights = [0.0] * 2
        self.forces = [0.0] * 9

        self.observations = (
            self.robot_pose + self.robot_twist + self.laser_scan + self.weights + self.forces
        )

        self.actions = [0.0] * 2

        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.cb_scan)
        self.sub_odom = self.create_subscription(Odometry, "/odometry/filtered", self.cb_odom)

    def reset(self):
        return

    def step(self, action):
        state = self.get_observation()
        reward, done = self.reward(state, action)
        # reteurn state, reward, done, progs
        return state, reward, done

    def get_observation(self):
        return

    def send_action(self, action):
        return

    def reward(self, state, action):
        # reward function
        reward = 0
        done = False
        return reward, done

    def cb_scan(self, scan):
        self.laser_scan = scan.ranges[:]

    def cb_odom(self, odom):
        lin = odom.twist.twist.linear.x
        ang = odom.twist.twist.angular.z
        self.robot_twist = [lin, ang]


if __name__ == "__main__":
    rclpy.init()
