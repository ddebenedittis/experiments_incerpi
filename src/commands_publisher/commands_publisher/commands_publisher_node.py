import numpy as np
import pandas as pd
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class HumbleNode(Node):
    def __init__(self, csv_file_path):
        super().__init__('humble_node')

        self.pub_torques = self.create_publisher(Float64MultiArray, 'TopicName1', 1)
        self.pub_joint_pos = self.create_publisher(Float64MultiArray, 'TopicName2', 1)
        self.pub_joint_vel = self.create_publisher(Float64MultiArray, 'TopicName3', 1)

        self.csv_file_path = csv_file_path  # Initialize csv_file_path

        self.tab = pd.read_csv(self.csv_file_path)
        self.n_timesteps = self.tab.shape[0]  # Sample number
        self.n_joints = 3

        self.torques = np.zeros((self.n_timesteps, self.n_joints))
        self.joint_pos = np.zeros((self.n_timesteps, self.n_joints))
        self.joint_vel = np.zeros((self.n_timesteps, self.n_joints))

        timer_period = 1.0 / 500  # seconds
        self.timer = self.create_timer(timer_period, self.publish_csv_vector)

    def csv_vector(self):
        for i in range(self.n_timesteps):
            self.torques[i, :] = self.tab.values[i, 0:3]
            self.joint_pos[i, :] = self.tab.values[i, 3:6]
            self.joint_vel[i, :] = self.tab.values[i, 6:9]

    def publish_csv_vector(self):
        self.csv_vector()  # Call the csv_vector method to populate the arrays
        for i in range(self.n_timesteps):
            torques_msg = Float64MultiArray()
            joint_pos_msg = Float64MultiArray()
            joint_vel_msg = Float64MultiArray()

            torques_msg.data = self.torques[i, :].tolist()
            joint_pos_msg.data = self.joint_pos[i, :].tolist()
            joint_vel_msg.data = self.joint_vel[i, :].tolist()

            self.pub_torques.publish(torques_msg)
            self.pub_joint_pos.publish(joint_pos_msg)
            self.pub_joint_vel.publish(joint_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HumbleNode('m.csv')  # '/path/to/your/csv/file.csv'
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
