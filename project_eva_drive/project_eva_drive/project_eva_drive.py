import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, MultiArrayDimension  # Import MultiArrayDimension
import time

# Constants
TRANSFORMATION_MATRIX = [[0,  1, 0.255],
                         [-1, 0, 0.255],
                         [0, -1, 0.255],
                         [1,  0, 0.255]]

LINEAR_SPEED_MULTIPLIER = 4  # Adjust this value to control speed
ANGULAR_SPEED_MULTIPLIER = 15

class ProjectEvaNode(Node):
    def __init__(self):
        super().__init__('project_eva_node')

        # Initialize velocities
        self.joint_velocities = [0.0] * 4

        # Create publisher for velocity commands (adjust topic and message type as needed)
        self.velocity_publisher = self.create_publisher(Float64MultiArray, '/drive_controller/commands', 10)  # Example

        # Subscribe to /cmd_vel topic
        qos = QoSProfile(depth=10)
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, qos)

        # Transformation matrix
        self.transformation_matrix = TRANSFORMATION_MATRIX

    def cmd_vel_callback(self, msg):
        wheel_velocities = self.calculate_wheel_velocities(
            msg.linear.x * LINEAR_SPEED_MULTIPLIER,
            msg.linear.y * LINEAR_SPEED_MULTIPLIER,
            msg.angular.z * ANGULAR_SPEED_MULTIPLIER
        )        
    
        self.joint_velocities = wheel_velocities

        # Prepare and publish velocity message
        velocity_msg = Float64MultiArray()
        velocity_msg.layout.dim.append(MultiArrayDimension())  # Add a dimension
        velocity_msg.layout.dim[0].label = ''
        velocity_msg.layout.dim[0].size = 4  # Number of joints
        velocity_msg.layout.dim[0].stride = 4
        velocity_msg.layout.data_offset = 0
        velocity_msg.data = self.joint_velocities
        self.velocity_publisher.publish(velocity_msg)


    def calculate_wheel_velocities(self, linear_x, linear_y, angular_z):
        # Your wheel velocity calculation logic remains the same
        velocities = [linear_x, linear_y, angular_z]
        wheel_velocities = [0.0] * len(self.transformation_matrix)

        for i in range(len(self.transformation_matrix)):
            for j in range(len(self.transformation_matrix[i])):
                wheel_velocities[i] += self.transformation_matrix[i][j] * velocities[j]
        # # print(wheel_velocities)
        # print(wheel_velocities)
        # return wheel_velocities
        
        # Adjust the order of wheel velocities
        adjusted_wheel_velocities = [wheel_velocities[0], 
                                     wheel_velocities[1],
                                     wheel_velocities[2],
                                     wheel_velocities[3]]

        print(adjusted_wheel_velocities)
        return adjusted_wheel_velocities

def main():
    rclpy.init()

    # Create the node
    project_eva_node = ProjectEvaNode()
    rclpy.spin(project_eva_node)
    project_eva_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



