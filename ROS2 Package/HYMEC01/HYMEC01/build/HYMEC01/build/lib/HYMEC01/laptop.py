import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyLaptopDataNode(Node):
    def __init__(self):
        super().__init__('laptop')
        self.subscription1 = self.create_subscription(String, 'debug1_topic', self.listener_callback, 10)
        self.subscription2 = self.create_subscription(String, 'debug2_topic', self.listener_callback, 10)
        self.subscription3 = self.create_subscription(String, 'debug3_topic', self.listener_callback, 10)
        self.subscription4 = self.create_subscription(String, 'debug4_topic', self.listener_callback, 10)
        self.publisher = self.create_publisher(String, 'command', 10)

    def listener_callback(self, msg):
        data_received = msg.data
        topic_name = self.subscription1.topic_name if msg._get_topic_name() == 'debug1_topic' else \
                     self.subscription2.topic_name if msg._get_topic_name() == 'debug2_topic' else \
                     self.subscription3.topic_name if msg._get_topic_name() == 'debug3_topic' else \
                     self.subscription4.topic_name if msg._get_topic_name() == 'debug4_topic' else None
        if topic_name is not None:
            self.get_logger().info('Data from %s : %s' % (topic_name, data_received))

    def run(self):
        while True:
            data_to_send = input("Type a message: ")
            msg = String()
            msg.data = data_to_send
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyLaptopDataNode()
    node.run()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

