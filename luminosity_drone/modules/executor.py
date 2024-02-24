import threading
import rclpy
import alien_center
import alien_detection_node

rclpy.init()
node = rclpy.create_node('alien_detection')
node2 = rclpy.create_node('waypoint_controller')
controller = alien_center.DroneController(node)
alien_detection = alien_detection_node.AlienDetection(node)
while not controller.filt.filter_ready:
    rclpy.spin_once(node)
print("Starting")
controller.start()
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(node)
executor.add_node(node2)
try:
    executor_thread = threading.Thread(target= executor.spin)
    executor_thread.start()
    executor_thread.join()
finally:
    controller.disarm()
    node.destroy_node()
    node2.destroy_node()
    rclpy.shutdown()    