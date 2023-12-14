import csv
import datetime
import numpy as np
import os
import pandas as pd
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from pi3hat_moteus_int_msgs.msg import JointsCommand,JointsStates

class TestNode(Node):
    def __init__(self, csv_file_path):
        super().__init__('test_node')
        
        # Set to True when we want to use RViz.
        self.declare_parameter('publish_joint_states', True)
        self.publish_joint_states = self.get_parameter('publish_joint_states').value

        self.pub = self.create_publisher(JointsCommand, 'joint_controller/command', 1)

        
        self.pub_rviz_joint_pos = self.create_publisher(JointState, '/joint_states', 1)

        self.csv_file_path = csv_file_path  # Initialize csv_file_path

        self.tab = pd.read_csv(self.csv_file_path)
        self.n_timesteps = self.tab.shape[0]  # Sample number
        self.n_joints = 3

        self.joint_pos = np.zeros((self.n_timesteps, self.n_joints))
        self.joint_vel = np.zeros((self.n_timesteps, self.n_joints))
        self.torques = np.zeros((self.n_timesteps, self.n_joints))


        self.csv_to_vectors()  # Call the csv_vector method to populate the arrays

        self.sec_0 = -1
        self.nsec_0 = 0

        self.data_matrix = np.zeros((0,13))   # [time, pos, vel, temp, torque]
        self.headers = ["time","q1","q2","q3","q1dot","q2dot","q3dot","T1","T2","T3","tau1","tau2","tau3"]

        self.subscriber = self.create_subscription(
            JointsStates,
            'state_broadcaster/joints_state',
            self.topic_callback,
            10,
        )           

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
       
        joints_command_msg.name = ['joint1', 'joint2', 'joint3']    # joint names in the interface
        joints_command_msg.position = self.joint_pos[i, :].tolist()
        joints_command_msg.velocity = self.joint_vel[i, :].tolist()
        joints_command_msg.effort = self.torques[i, :].tolist()
        joints_command_msg.kp_scale = [3., 3., 3.]
        joints_command_msg.kd_scale =  [0.05, 0.05, 0.05]
        self.pub.publish(joints_command_msg)
       
        # Used for the visualization in RViz.
        if self.publish_joint_states:
            rviz_joint_msg = JointState()
            rviz_joint_msg.header = Header()
            rviz_joint_msg.header.stamp = self.get_clock().now().to_msg()
            
            rviz_joint_msg.name = ['JOINT_1', 'JOINT_2', 'JOINT_3'] # joint names in the urdf
            rviz_joint_msg.position = self.joint_pos[i, :].tolist()
            self.pub_rviz_joint_pos.publish(rviz_joint_msg)

    def topic_callback(self, msg: JointsStates): #function that will be executed when a message is received on the subscribed topic

        tsec = msg.header.stamp.sec
        tnsec = msg.header.stamp.nanosec

        if self.sec_0 < 0:
            self.sec_0 = tsec
            self.nsec_0 = tnsec

        time = (tsec - self.sec_0) + (tnsec - self.nsec_0)/10.**9
        pos = msg.position
        vel = msg.velocity
        temp = msg.temperature
        torque = msg.effort

        # Add the data to the matrix as rows
        data_array = np.concatenate((np.array([time]), pos, vel, temp, torque))
        self.data_matrix = np.append(self.data_matrix, [data_array], axis=0)


    def export_to_csv(self):
        
        # Specify the CSV file path
        
        path = "csv/" + f"{datetime.datetime.now():%Y-%m-%d-%H:%M:%S}" + "/"

        try:
            os.makedirs(path)
        except OSError:
            print("Creation of the directories %s failed" % path)
        else:
            print("Successfully created the directories %s" % path)
        

        # Write matrix data to CSV file
       
        with open(path +'data.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(self.headers)
            writer.writerows(self.data_matrix)

def main(args=None):
    rclpy.init(args=args)
    node = TestNode('m.csv')  # '/path/to/your/csv/file.csv'
    
    rclpy.logging.get_logger("Publishing node").info('Trajectory started.')
    
    try:
        rclpy.spin(node)
    except SystemExit:
        node.export_to_csv()
        rclpy.logging.get_logger("Publishing node").info('Trajectory finished.')
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
