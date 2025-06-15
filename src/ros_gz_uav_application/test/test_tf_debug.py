#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import time

class TFDebugNode(Node):
    def __init__(self):
        super().__init__('tf_debug_node')
        
        # Create TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create timer to check TF periodically
        self.timer = self.create_timer(1.0, self.check_tf)
        
        self.get_logger().info("TF Debug Node started")
    
    def check_tf(self):
        try:
            # Try to get transform from world to map
            transform = self.tf_buffer.lookup_transform(
                'world', 'map', rclpy.time.Time())
            
            self.get_logger().info(f"TF found: world -> map")
            self.get_logger().info(f"  Translation: ({transform.transform.translation.x}, "
                                 f"{transform.transform.translation.y}, "
                                 f"{transform.transform.translation.z})")
            self.get_logger().info(f"  Rotation: ({transform.transform.rotation.x}, "
                                 f"{transform.transform.rotation.y}, "
                                 f"{transform.transform.rotation.z}, "
                                 f"{transform.transform.rotation.w})")
            
        except Exception as e:
            self.get_logger().warn(f"TF not found: {e}")

def main():
    rclpy.init()
    node = TFDebugNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 