#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from autoware_auto_planning_msgs.msg import Trajectory 


class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.subscription = self.create_subscription(
            autoware_auto_planning_msgs/msg/Trajectory,
            '/planing/racing_planer/trajectory',
            self.trajectory_callback,
            10)
        self.publisher_ = self.create_publisher(autoware_auto_planning_msgs/msg/Trajectory, '/planing/racing_planer/avoidance/trajectory', 10)
        self.subscription_laser = self.create_subscription(
            sensor_msgs/msg/LaserScan,
            '/sensing/lidar/scan',
            self.lidar_callback,
            10)
        self.subscription  # prevent unused variable warning

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        obstacle_positions = self.ranges_to_positions(ranges, msg.angle_min, msg.angle_increment)

        grid_size = (30, 30)
        goal = (25, 25)
        obstacle_charge = 100
        goal_charge = -200

        # pole potencjalne
        potential_field = self.compute_potential_field(grid_size, goal, obstacle_positions, obstacle_charge, goal_charge)


    def trajectory_callback(self, msg):

        # republikowanie obecnej sciezki 
        msg = autoware_auto_planning_msgs/msg/Trajectory()
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

    def ranges_to_positions(self, ranges, angle_min, angle_increment):
        # zamieniamy zakresy skanera na pozycje przeszkód
        positions = []
        angle = angle_min
        for range in ranges:
            if range < 30:  # przyjmujemy, że maksymalny zasięg to 30
                x = range * np.cos(angle)
                y = range * np.sin(angle)
                positions.append((int(x), int(y)))
            angle += angle_increment
        return positions

    def compute_potential_field(self, grid_size, goal, obstacles, obstacle_charge, goal_charge):
        x_range = np.arange(grid_size[0])
        y_range = np.arange(grid_size[1])
        potential_field = np.zeros((grid_size[1], grid_size[0]))

        # przyciąganie celu
        for x in x_range:
            for y in y_range:
                potential_field[y, x] -= goal_charge / ((x - goal[0])**2 + (y - goal[1])**2 + 1)

        # odpychanie od przeszkód
        for obs in obstacles:
            for x in x_range:
                for y in y_range:
                    potential_field[y, x] += obstacle_charge / ((x - obs[0])**2 + (y - obs[1])**2 + 1)

        return potential_field

    def find_best_direction(self, position, field):
        x, y = position
        min_potential = float('inf')
        best_direction = (0, 0)

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < field.shape[1] and 0 <= ny < field.shape[0]:
                    if field[ny, nx] < min_potential:
                        min_potential = field[ny, nx]
                        best_direction = (dx, dy)
        return best_direction

    def create_trajectory_message(self, position, direction, timestamp):
        x, y = position
        dx, dy = direction

        # Tworzenie trajektorii jako listy punktów
        trajectory = Trajectory()
        trajectory.header.stamp = timestamp
        trajectory.header.frame_id = "map"

        for step in range(10):  # generujemy 10 punktów do przodu
            x += dx
            y += dy
            point = TrajectoryPoint()
            point.pose.position.x = x
            point.pose.position.y = y
            point.pose.position.z = 0
            trajectory.points.append(point)

        return trajectory


def main(args=None):
    rclpy.init(args=args)

    obstacle_avoidance = ObstacleAvoidance()

    rclpy.spin(obstacle_avoidance)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    obstacle_avoidance.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()