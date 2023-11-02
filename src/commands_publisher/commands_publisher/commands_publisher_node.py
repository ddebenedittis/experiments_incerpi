import numpy as np
import pandas as pd
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from pi3hat_moteus_int_msgs.msg import JointsCommand

class TestNode(Node):
    def __init__(self, csv_file_path):
        super().__init__('test_node')
        
        self.declare_parameter('publish_joint_states', False)
        self.publish_joint_states = self.get_parameter('publish_joint_states').value

        self.pub = self.create_publisher(JointsCommand, 'joint_controller/command', 1)
       
        self.pub_rviz_joint_pos = self.create_publisher(JointState, '/joint_states', 1)

        self.csv_file_path = csv_file_path  # Initialize csv_file_path

        self.tab = pd.read_csv(self.csv_file_path)
        self.n_timesteps = self.tab.shape[0]  # Sample number
        self.n_joints = 3

        self.torques = np.zeros((self.n_timesteps, self.n_joints))
        self.joint_pos = np.zeros((self.n_timesteps, self.n_joints))
        self.joint_vel = np.zeros((self.n_timesteps, self.n_joints))
        
        self.csv_to_vectors()  # Call the csv_vector method to populate the arrays
        
        self.counter = 0

        timer_period = 1.0 / 500  # seconds
        self.timer = self.create_timer(timer_period, self.publish_vectors)

    def csv_to_vectors(self):
        for i in range(self.n_timesteps):
            self.joint_pos[i, :] = self.tab.values[i, 0:3]
            self.joint_vel[i, :] = self.tab.values[i, 3:6]
            self.torques[i, :] = self.tab.values[i, 6:9]
 
    def publish_vectors(self):
        i = self.counter
        self.counter += 1
        
        if self.counter >= np.shape(self.torques)[0]:
            raise SystemExit
                
        joints_command_msg = JointsCommand()
       
        # Joint names in the pi3hat controller interfece. 
        joints_command_msg.name = ['joint1', 'joint2', 'joint3']
        joints_command_msg.position = self.joint_pos[i, :].tolist()
        joints_command_msg.velocity = self.joint_vel[i, :].tolist()
        joints_command_msg.effort = self.torques[i, :].tolist()
        joints_command_msg.kp_scale = [3., 3., 3.]
        joints_command_msg.kd_scale =  [0.05, 0.05, 0.05]

        self.pub.publish(joints_command_msg)
       
        if self.publish_joint_states:
            rviz_joint_msg = JointState()
            rviz_joint_msg.header = Header()
            rviz_joint_msg.header.stamp = self.get_clock().now().to_msg()
            
            rviz_joint_msg.name = ['JOINT_1', 'JOINT_2', 'JOINT_3']
            rviz_joint_msg.position = self.joint_pos[i, :].tolist()
            self.pub_rviz_joint_pos.publish(rviz_joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestNode('m.csv')  # '/path/to/your/csv/file.csv'
    
    rclpy.logging.get_logger("Publishing node").info('Trajectory started.')
    
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("Publishing node").info('Trajectory finished.')
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
