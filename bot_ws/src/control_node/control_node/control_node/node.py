import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped
import math

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        self.integral += error
        derivative = error - self.prev_error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.path_sub = self.create_subscription(Path, 'planned_path', self.path_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.path = []
        self.lookahead_distance = 1.0  # Example parameter
        self.max_speed = 0.5  # Maximum speed in m/s
        self.steering_pid = PID(1.0, 0.0, 0.1)  # Example PID coefficients

    def path_callback(self, msg):
        self.path = msg.poses
        self.control_loop()

    def control_loop(self):
        current_pose = self.get_current_pose()
        if not self.path:
            return

        lookahead_point = self.get_lookahead_point(current_pose)
        if lookahead_point:
            velocity, steering_angle = self.compute_control_commands(current_pose, lookahead_point)
            self.publish_commands(velocity, steering_angle)

    def get_current_pose(self):
        # Placeholder for current pose; replace with actual robot localization data
        return PoseStamped()

    def get_lookahead_point(self, current_pose):
        # Implement lookahead logic to find the point on the path a certain distance ahead of the current position
        # This is a placeholder function and needs to be implemented according to your path and robot kinematics
        if not self.path:
            return None
        for pose in self.path:
            # Calculate distance to each point in the path
            dx = pose.pose.position.x - current_pose.pose.position.x
            dy = pose.pose.position.y - current_pose.pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)
            if distance > self.lookahead_distance:
                return pose
        return None

    def compute_control_commands(self, current_pose, lookahead_point):
        # Calculate heading to the lookahead point
        dx = lookahead_point.pose.position.x - current_pose.pose.position.x
        dy = lookahead_point.pose.position.y - current_pose.pose.position.y
        heading = math.atan2(dy, dx)

        # Calculate the heading error
        current_heading = self.get_heading_from_pose(current_pose)
        heading_error = heading - current_heading

        # Use PID controller to calculate steering angle
        steering_angle = self.steering_pid.compute(0, heading_error)

        # Set velocity to a constant value or based on some criteria
        velocity = self.max_speed

        return velocity, steering_angle

    def get_heading_from_pose(self, pose):
        # Assuming pose.orientation is a quaternion, convert to yaw angle
        # This conversion depends on the quaternion representation
        # Placeholder, replace with actual conversion
        return 0.0

    def publish_commands(self, velocity, steering_angle):
        cmd = Twist()
        cmd.linear.x = velocity
        cmd.angular.z = steering_angle
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
