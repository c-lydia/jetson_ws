import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import cv2
import numpy as np
import time
import os

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.publisher = self.create_publisher(OccupancyGrid, 'map', 10)
        self.subscriber = self.create_subscription(Bool, 'request_map', self.request_callback, 1)

        time.sleep(3)

        self.request_callback(Bool)
    def request_callback(self, msg):
        img_path = '~/map/Gamefield.jpg'
        img_path = os.path.expanduser(img_path)
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)

        if img is None:
            self.get_logger().error(f"Failed to load image at {img_path}")
            return

        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "odom"
        map_msg.info.resolution = 0.05  # meters/pixel
        map_msg.info.width = img.shape[1]  # image width in pixels
        map_msg.info.height = img.shape[0]  # image height in pixels
        map_msg.info.origin = Pose()  # Default to (0,0,0)
        map_msg.info.origin.position.y = -12.0 
        
        # Convert grayscale to occupancy values: 255 (white) to 0 (free), 0 (black) to 100 (occupied)
        occupancy_grid = np.zeros_like(img, dtype=np.int8)
        occupancy_grid[img < 50] = 100  # Assuming black pixels represent obstacles
        occupancy_grid[img > 205] = 0   # Assuming white pixels represent free space
        # Intermediate values can be considered as unknown (-1)
        occupancy_grid[(img >= 50) & (img <= 205)] = -1

        map_msg.data = occupancy_grid.flatten().tolist()
        self.publisher.publish(map_msg)
        self.get_logger().info('Map Published')

    time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    map_publisher = MapPublisher()
    rclpy.spin(map_publisher)
    map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

