# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String

# class HelloPublisher(Node):
#     def __init__(self):
#         super().__init__('hello_publisher')
#         self.publisher_ = self.create_publisher(String, 'hello_topic', 10)
#         timer_period = 1.0  # seconds
#         self.timer = self.create_timer(timer_period, self.publish_message)
#         self.get_logger().info('HelloPublisher node has been started.')

#     def publish_message(self):
#         msg = String()
#         msg.data = 'Hello, world!'
#         self.publisher_.publish(msg)
#         self.get_logger().info(f'Published: "{msg.data}"')

# def main(args=None):
#     rclpy.init(args=args)
#     node = HelloPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt  # New import

class HelloPublisher(Node):
    def __init__(self):
        super().__init__('hello_publisher')

        # ROS 2 Publisher
        self.publisher_ = self.create_publisher(String, 'hello_topic', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_message)

        # MQTT Client Setup
        self.mqtt_client = mqtt.Client()
        try:
            self.mqtt_client.connect("10.88.0.1", 1883, 60)
            self.get_logger().info('Connected to MQTT broker at 10.88.0.1')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MQTT broker: {e}')

        self.get_logger().info('HelloPublisher node has been started.')

    def publish_message(self):
        msg = String()
        msg.data = 'Hello, world!'

        # Publish to ROS 2 topic
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published to ROS 2: "{msg.data}"')

        # Publish to MQTT
        try:
            self.mqtt_client.publish("hello/mqtt", msg.data)
            self.get_logger().info(f'Published to MQTT: "{msg.data}"')
        except Exception as e:
            self.get_logger().error(f'MQTT publish failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = HelloPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt_client.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

