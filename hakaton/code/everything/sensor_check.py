import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


class SensorCheckNode(Node):
    """
    A simple diagnostic tool to listen to all known distance sensor topics
    and print their live readings to the terminal.
    """
    def __init__(self):
        super().__init__("sensor_check_node")

        # Define all topics we found in the 'ros2 topic list'
        self.topics = [
            "/io/distance/front_left",
            "/io/distance/front_right",
            "/io/distance/rear_left",
            "/io/distance/rear_right"
        ]

        # Store the latest reading for each topic
        self.readings = {topic: "Waiting..." for topic in self.topics}

        # Create subscribers dynamically for every topic
        self.subscribers = []
        for topic in self.topics:
            sub = self.create_subscription(
                Range,
                topic,
                self._make_callback(topic),
                10
            )
            self.subscribers.append(sub)

        # Create a timer to print the status every 0.5 seconds
        self.timer = self.create_timer(0.5, self._print_status)
        self.get_logger().info("Sensor Diagnostic Tool Started. Waiting for data...")

    def _make_callback(self, topic_name):
        """Creates a specific callback function for a given topic."""
        def callback(msg: Range):
            # Store the range value (rounded to 2 decimal places if it's a number)
            if msg.range == float('inf'):
                self.readings[topic_name] = "Out of Range (inf)"
            else:
                self.readings[topic_name] = f"{msg.range:.2f} m"
        return callback

    def _print_status(self):
        """Prints the current state of all sensors."""
        # Clear the terminal screen slightly for readability (ANSI escape code)
        print("\033c", end="")
        
        print("=== LIVE SENSOR READINGS ===")
        print("Wave your hand in front of the robot's sensors to see which ones update.\n")
        
        for topic, reading in self.readings.items():
            print(f"{topic.ljust(30)} : {reading}")
            
        print("\nPress Ctrl+C to stop.")


def main(args=None):
    rclpy.init(args=args)
    node = SensorCheckNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
