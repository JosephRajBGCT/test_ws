import rclpy
from rclpy.node import Node
from .single_ultrasonic_sensor import UltrasonicSensor

class MultiSensorNode(Node):
    def __init__(self):
        super().__init__('multi_sensor_node')
        
        # Initialize multiple sensors
        self.sensors = [
            UltrasonicSensor(23, 24, 1),
            UltrasonicSensor(17, 27, 2),
            UltrasonicSensor(5, 6, 3)
        ]

        self.get_logger().info(f"Initialized {len(self.sensors)} ultrasonic sensors.")

    def destroy_node(self):
        self.get_logger().info("Shutting down sensors...")
        for sensor in self.sensors:
            sensor.stop()  # Gracefully stop each sensor thread
        super().destroy_node()

    def stop_all_sensors(self):
        for sensor in self.sensors:
            sensor.stop()


def main(args=None):
    rclpy.init(args=args)
    node = MultiSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down...")
    finally:
        node.stop_all_sensors()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

