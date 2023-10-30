import numpy as np
import pandas as pd
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Float64MultiArray
from sensor_msgs.msg import JointState

class HumbleNode(Node):
    def __init__(self, csv_file_path):
        super().__init__('humble_node')

        self.pub_torques = self.create_publisher(Float64MultiArray, 'TopicName1', 1)
        self.pub_joint_pos = self.create_publisher(Float64MultiArray, 'TopicName2', 1)
        self.pub_joint_vel = self.create_publisher(Float64MultiArray, 'TopicName3', 1)
        
        self.pub_rviz_joint_pos = self.create_publisher(JointState, '/joint_states', 1)

        self.csv_file_path = csv_file_path  # Initialize csv_file_path

        self.tab = pd.read_csv(self.csv_file_path)
        self.n_timesteps = self.tab.shape[0]  # Sample number
        self.n_joints = 3

        self.torques = np.zeros((self.n_timesteps, self.n_joints))
        self.joint_pos = np.zeros((self.n_timesteps, self.n_joints))
        self.joint_vel = np.zeros((self.n_timesteps, self.n_joints))
        
        self.csv_vector()  # Call the csv_vector method to populate the arrays
        
        self.counter = 0

        timer_period = 1.0 / 500  # seconds
        self.timer = self.create_timer(timer_period, self.publish_csv_vector)

    def csv_vector(self):
        for i in range(self.n_timesteps):
            self.torques[i, :] = self.tab.values[i, 0:3]
            self.joint_pos[i, :] = self.tab.values[i, 3:6]
            self.joint_vel[i, :] = self.tab.values[i, 6:9]

    def publish_csv_vector(self):
        i = self.counter
        self.counter += 1
        
        if self.counter >= np.shape(self.torques)[0]:
            return
        
        torques_msg = Float64MultiArray()
        joint_pos_msg = Float64MultiArray()
        joint_vel_msg = Float64MultiArray()

        torques_msg.data = self.torques[i, :].tolist()
        joint_pos_msg.data = self.joint_pos[i, :].tolist()
        joint_vel_msg.data = self.joint_vel[i, :].tolist()

        self.pub_torques.publish(torques_msg)
        self.pub_joint_pos.publish(joint_pos_msg)
        self.pub_joint_vel.publish(joint_vel_msg)
        
        rviz_joint_msg = JointState()
        rviz_joint_msg.header = Header()
        rviz_joint_msg.header.stamp = self.get_clock().now().to_msg()
        
        rviz_joint_msg.name = ['JOINT_1', 'JOINT_2', 'JOINT_3']
        rviz_joint_msg.position = self.joint_pos[i, :].tolist()
        self.pub_rviz_joint_pos.publish(rviz_joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HumbleNode('m.csv')  # '/path/to/your/csv/file.csv'
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
