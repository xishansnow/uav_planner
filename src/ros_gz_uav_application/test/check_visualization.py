#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
import sys

class VisualizationChecker(Node):
    def __init__(self):
        super().__init__('visualization_checker')
        self.subscription = self.create_subscription(
            MarkerArray,
            '/planning_visualization',
            self.marker_callback,
            10
        )
        self.marker_count = 0
        
    def marker_callback(self, msg):
        self.marker_count += 1
        self.get_logger().info(f'Received MarkerArray #{self.marker_count}: {len(msg.markers)} markers')
        
        for i, marker in enumerate(msg.markers):
            self.get_logger().info(f'  Marker {i}: ns={marker.ns}, id={marker.id}, type={marker.type}')
            if hasattr(marker, 'pose') and hasattr(marker.pose, 'position'):
                pos = marker.pose.position
                self.get_logger().info(f'    Position: ({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f})')

def main():
    rclpy.init()
    checker = VisualizationChecker()
    
    try:
        rclpy.spin(checker)
    except KeyboardInterrupt:
        pass
    finally:
        checker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 