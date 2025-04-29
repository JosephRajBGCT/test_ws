import rclpy
from rclpy.node import Node
import os
from .single_ultrasonic_withfilter import UltrasonicSensor  # Assuming your refactored Python class is in this file

class MultiSensorNode(Node):
    def __init__(self):
        super().__init__('multi_sensor_node')
        self.get_logger().info(f"Main Process PID: {os.getpid()}")
        
        # Initialize multiple sensors
        self.sensors = [
            UltrasonicSensor(23, 24, 1),
            UltrasonicSensor(12, 13, 2)
        ]

        self.get_logger().info(f"Initialized {len(self.sensors)} ultrasonic sensors.")

        # Publisher for each sensor
        self.publishers = [
            self.create_publisher(Range, f'ultrasonic_distance_{sensor.sensor_id}', 15) for sensor in self.sensors
        ]

        # Start the sensors
        self.sensor_threads = []
        for sensor in self.sensors:
            thread = threading.Thread(target=self.run_sensor, args=(sensor,))
            thread.start()
            self.sensor_threads.append(thread)

    def run_sensor(self, sensor):
        while rclpy.ok():
            filtered_distance = sensor.get_filtered_distance()
            self.publish_distance(filtered_distance, sensor)
            time.sleep(0.02)

    def publish_distance(self, distance, sensor):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"ultrasonic_sensor_{sensor.sensor_id}"
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.52
        msg.min_range = 3.0
        msg.max_range = 100.0

        if distance < msg.min_range or distance > msg.max_range or distance == -1:
            msg.range = float('inf')
        else:
            msg.range = float(distance)

        self.publishers[sensor.sensor_id - 1].publish(msg)
        self.get_logger().info(f"Sensor {sensor.sensor_id} Distance: {msg.range:.2f} cm")

    def destroy_node(self):
        if rclpy.ok():
            self.get_logger().info("Shutting down sensors...")
        else:
            print("[STDOUT] Shutting down sensors...")

        # Stop all sensors
        for sensor in self.sensors:
            sensor.stop()

        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MultiSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("[STDOUT] KeyboardInterrupt received. Shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

