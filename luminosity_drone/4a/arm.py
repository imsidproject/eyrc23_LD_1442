import rclpy
import time
from swift_msgs.msg import RCMessage
from rclpy.node import Node
from swift_msgs.srv import CommandBool

commandbool = CommandBool.Request()
service_endpoint = "/swift/cmd/arming"
rc=RCMessage()
rc.rc_throttle=1050
rc.rc_roll=1500
rc.rc_pitch=1550
rc.rc_yaw=1500

rclpy.init(args=None)
node = rclpy.create_node('controller')
rc_pub = node.create_publisher(RCMessage, "/swift/rc_command",1)
arming_service_client = node.create_client(CommandBool,service_endpoint)
while not arming_service_client.wait_for_service(timeout_sec=1.0):
	node.get_logger().info("Service not available")
commandbool.value = True
future = arming_service_client.call_async(commandbool)
rclpy.spin_until_future_complete(node,future)
node.get_logger().info(str(future.result().success))

rc_pub.publish(rc)


time.sleep(2)
commandbool.value = False
future = arming_service_client.call_async(commandbool)
rclpy.spin_until_future_complete(node,future)
node.get_logger().info(str(future.result().success))
