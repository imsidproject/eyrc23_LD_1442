import rclpy

def timer_callback():
    node.get_logger().info( str(node.get_clock().now().to_msg()))
    print("lalalalala")

if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('time')
    timer = node.create_timer(0.333, timer_callback)
    try:
        rclpy.spin(node)
    finally:

        print("bababababa")
    rclpy.shutdown()