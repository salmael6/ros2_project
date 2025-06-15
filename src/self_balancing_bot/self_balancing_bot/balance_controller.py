import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import tf_transformations
import time

class BalanceController(Node):

    def teleop_callback(self, msg: Twist):
            self.teleop_cmd = msg
    def __init__(self):
        super().__init__('balance_controller')

        # Parameters (tune these)
        self.kp = 5.0
        self.ki = 0.1
        self.kd = 0.05

        self.alpha = 0.98  # Complementary filter coefficient
        self.dt = 0.01     # Initial delta time

        # State variables for PID
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()

        # Estimated pitch angle (radians)
        self.pitch_est = 0.0

        # Subscriber to IMU data
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',  
            self.imu_callback,
            10)
        
        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Publisher for velocity commands
        self.teleop_sub = self.create_subscription(
            Twist,
            '/teleop_cmd_vel',  
            self.teleop_callback,
            10)
        self.teleop_cmd = Twist() 
        



    def imu_callback(self, msg: Imu):
        # Current time and delta time
        current_time = self.get_clock().now()
        self.dt = (current_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = current_time
        if self.dt <= 0 or self.dt > 0.1:
            self.dt = 0.01  # Prevent division by zero

        # Extract orientation quaternion
        q = msg.orientation
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        _, pitch_imu, _ = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])
            
        if not hasattr(self, 'initialized'):
            self.pitch_est = pitch_imu
            self.initialized = True

        # Angular velocity around Y axis (pitch rate)
        gyro_y = msg.angular_velocity.y

        # Complementary filter to estimate pitch angle
        pitch_gyro = self.pitch_est + gyro_y * self.dt
        self.pitch_est = self.alpha * pitch_gyro + (1 - self.alpha) * pitch_imu

        # PID controller on pitch angle
        error = self.pitch_est  # Desired pitch = 0 (upright)
        deadband = math.radians(1)
        if abs(error) < deadband:
            error = 0.0
        self.integral += error * self.dt
        integral_limit = 10.0
        self.integral = max(min(self.integral, integral_limit), -integral_limit)
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Seuil pour considérer que la commande téléop est active
        teleop_active = abs(self.teleop_cmd.linear.x) > 0.01 or abs(self.teleop_cmd.angular.z) > 0.01

        cmd = Twist()
        if teleop_active:
            # Priorité à la téléop : on publie directement la commande téléop
            cmd.linear.x = self.teleop_cmd.linear.x
            cmd.angular.z = self.teleop_cmd.angular.z
        else:
            # Pas de commande téléop : on applique la correction d’équilibre
            cmd.linear.x = -output
            cmd.angular.z = 0.0

        # Limitation des vitesses
        max_speed = 0.5
        cmd.linear.x = max(min(cmd.linear.x, max_speed), -max_speed)
        cmd.angular.z = max(min(cmd.angular.z, max_speed), -max_speed)

        self.cmd_pub.publish(cmd)

        # Debug info
        self.get_logger().info(
            f'Pitch_est: {math.degrees(self.pitch_est):.2f} deg, '
            f'Cmd_vel linear.x: {cmd.linear.x:.2f} m/s, '
            f'Cmd_vel angular.z: {cmd.angular.z:.2f} rad/s')

def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

