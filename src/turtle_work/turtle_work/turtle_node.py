import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import random
import math
from turtlesim.msg import Pose as TurtlePose
from functools import partial
from turtlesim.srv import SetPen
import subprocess
import time


class TurtleHunter(Node):
    def __init__(self):
        super().__init__('turtle_hunter')
        
        # Master turtle and target information
        self.master_pose = None
        self.targets = []
        self.turtle_poses = {}

        # Master turtle control and position monitoring
        self.master_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(TurtlePose, '/turtle1/pose', self.master_pose_callback, 10)
        
        # Turtle spawn client
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.turtle_counter = 2
        self.spawn_timer = self.create_timer(3.0, self.spawn_turtle)

        # Following logic
        self.current_target = None
        self.ANGULAR_SPEED = 0.5
        self.LINEAR_SPEED = 0.5
        self.follow_chain = []
        self.move_timer = self.create_timer(0.1, self.move_to_target)
        self.chain_pub = self.create_publisher(String, '/follow_chain', 10)
        self.follower_publishers = {}
        
        # Following parameters
        self.FOLLOW_DISTANCE = 0.3  # Reduced following distance
        self.ANGLE_THRESHOLD = 0.05  # Increased angle threshold
        self.SPEED_FACTOR = 2.0  # Speed factor
        self.last_commands = {}  # Store previous commands

        # Logging logic (Task3)
        self.master_logs = []  # Store master turtle's position and direction
        self.log_limit = 100
        self.log_timer = self.create_timer(0.1, self.log_master_direction)
        self.follower_timer = self.create_timer(0.1, self.update_followers)


    def spawn_turtle(self):
        if not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for spawn service...')
            return
        
        request = Spawn.Request()
        request.x = random.uniform(1.0, 10.0)
        request.y = random.uniform(1.0, 10.0)
        request.theta = random.uniform(0.0, 2*math.pi)
        request.name = f'turtle{self.turtle_counter}'
        
        self.get_logger().info(f'Spawning new turtle: {request.name}')
        future = self.spawn_client.call_async(request)
        future.add_done_callback(partial(self.spawn_callback, name=request.name))
        self.turtle_counter += 1

    def spawn_callback(self, future, name):
        try:
            response = future.result()
            self.targets.append(response.name)
            self.get_logger().info(f'New turtle spawned: {response.name}')
            self.create_subscription(
                TurtlePose,
                f"/{response.name}/pose",
                lambda msg, name=name: self.turtle_pose_callback(msg, name),
                10)
        except Exception as e:
            self.get_logger().error(f'Spawn failed: {e}')

    def master_pose_callback(self, msg):
        self.master_pose = msg

    def turtle_pose_callback(self, msg, turtle_name):
        self.turtle_poses[turtle_name] = msg

    def calculate_distance(self, pose1, pose2):
        return math.hypot(pose2.x - pose1.x, pose2.y - pose1.y)

    def calculate_angle_diff(self, current_angle, target_angle):
        diff = target_angle - current_angle
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def find_closest_turtle(self):
        if not self.master_pose or not self.turtle_poses:
            return None

        closest_name = None
        min_distance = float('inf')
        closest_pose = None
        
        for name, pose in self.turtle_poses.items():
            if name in self.follow_chain:
                continue
            dx = pose.x - self.master_pose.x
            dy = pose.y - self.master_pose.y
            distance = math.hypot(dx, dy)
            if distance < min_distance:
                min_distance = distance
                closest_name = name
                closest_pose = pose

        if closest_name:
            return {'name': closest_name, 'pose': closest_pose}
        return None

    def move_to_target(self):
        # Ensure master turtle position is initialized
        if not self.master_pose:
            return

        # If no current target, select the closest turtle
        if not self.current_target:
            self.current_target = self.find_closest_turtle()
            if not self.current_target:
                return

        target = self.current_target['pose']
        dx = target.x - self.master_pose.x
        dy = target.y - self.master_pose.y
        distance = math.hypot(dx, dy)

        target_angle = math.atan2(dy, dx)
        angle_diff = self.calculate_angle_diff(self.master_pose.theta, target_angle)

        cmd = Twist()
        if abs(angle_diff) > 0.1:
            cmd.angular.z = self.ANGULAR_SPEED if angle_diff > 0 else -self.ANGULAR_SPEED
        else:
            if distance > 0.1:
                cmd.linear.x = self.LINEAR_SPEED
            else:
                self.handle_arrival()
                return

        self.master_pub.publish(cmd)

    def handle_arrival(self):
        captured = self.current_target['name']
        self.follow_chain.append(captured)
        self.get_logger().info(f"Capture successful! Current queue: {self.follow_chain}")
        self.follower_publishers[captured] = self.create_publisher(Twist, f'/{captured}/cmd_vel', 10)
        msg = String()
        msg.data = ",".join(self.follow_chain)
        self.chain_pub.publish(msg)
        self.current_target = None

        # Disable pen
        pen_client = self.create_client(SetPen, f'/{captured}/set_pen')
        while not pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{captured} set_pen service not available, waiting...')

        pen_req = SetPen.Request()
        pen_req.r = 0
        pen_req.g = 0
        pen_req.b = 0
        pen_req.width = 0
        pen_req.off = 1  # Turn off pen

        future = pen_client.call_async(pen_req)

    def log_master_direction(self):
        if not self.master_pose:
            return

        timestamp = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        log_entry = {
            'time': round(timestamp, 3),
            'x': round(self.master_pose.x, 2),
            'y': round(self.master_pose.y, 2),
            'theta': round(self.master_pose.theta, 3),
        }
        self.master_logs.append(log_entry)

    
        if len(self.master_logs) > self.log_limit:
            self.master_logs.pop(0)

    def print_logs(self):
        self.get_logger().info("=== Master Turtle Movement Logs (First 5 entries) ===")
        for entry in self.master_logs[:5]:
            self.get_logger().info(str(entry))
            

    def update_followers(self):
        if len(self.follow_chain) < 1:
            return

        for idx, name in enumerate(self.follow_chain):
            if name not in self.turtle_poses:
                continue
            if name not in self.follower_publishers:
                continue

            # Current follower's position
            pose = self.turtle_poses[name]

            # Get target to follow (previous turtle's position)
            target_pose = self.master_pose if idx == 0 else self.turtle_poses.get(self.follow_chain[idx - 1])
            if not target_pose:
                continue

            # Calculate target position (behind the previous turtle)
            dx = target_pose.x - pose.x
            dy = target_pose.y - pose.y
            distance = math.hypot(dx, dy)
            target_angle = math.atan2(dy, dx)
            angle_diff = self.calculate_angle_diff(pose.theta, target_angle)

            cmd = Twist()
            
            # Get previous command
            last_cmd = self.last_commands.get(name, Twist())
            
            # Angle control
            if abs(angle_diff) > self.ANGLE_THRESHOLD:
                # Smooth angle adjustment
                cmd.angular.z = 2.0 * angle_diff
                cmd.linear.x = 0.0  # Slow down while turning
            else:
                # Distance control
                if distance > self.FOLLOW_DISTANCE:
                    # Dynamically adjust speed based on distance
                    speed_factor = min(self.SPEED_FACTOR, 1.0 + (distance - self.FOLLOW_DISTANCE))
                    cmd.linear.x = self.LINEAR_SPEED * speed_factor
                    # Maintain small angle adjustments
                    cmd.angular.z = angle_diff * 1.0
                else:
                    # When distance is appropriate, maintain same speed as previous turtle
                    cmd.linear.x = target_pose.linear_velocity
                    cmd.angular.z = target_pose.angular_velocity

            # Smooth speed changes
            if name in self.last_commands:
                cmd.linear.x = 0.8 * cmd.linear.x + 0.2 * last_cmd.linear.x
                cmd.angular.z = 0.8 * cmd.angular.z + 0.2 * last_cmd.angular.z

            # Save current command
            self.last_commands[name] = cmd
            
            # Publish command
            self.follower_publishers[name].publish(cmd)
        

def main(args=None):
    rclpy.init(args=args)
    turtlesim_process = subprocess.Popen(['ros2', 'run', 'turtlesim', 'turtlesim_node'])
    time.sleep(1)
    hunter = TurtleHunter()

    try:
        rclpy.spin(hunter)
    except KeyboardInterrupt:
        hunter.print_logs()  
    finally:
        hunter.destroy_node()
        rclpy.shutdown()
        turtlesim_process.terminate()

if __name__ == '__main__':
    main()
