import time
import threading
import lgpio
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

import os
class UltrasonicSensor(Node):
    def __init__(self, trig_pin, echo_pin, sensor_id=None):
        super().__init__(f'ultrasonic_sensor_{sensor_id}' if sensor_id else 'ultrasonic_sensor')
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.sensor_id = sensor_id or 'default'
        self.running = True

        # Set up GPIO
        self.chip = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.chip, self.trig_pin, 0)
        lgpio.gpio_claim_input(self.chip, self.echo_pin)

        # ROS2 Publisher
        self.publisher = self.create_publisher(Range, f'ultrasonic_distance_{self.sensor_id}', 20)

        # Distance filtering
        self.filtered_distance = 0.0
        self.alpha = 0.3
        self.outlier_threshold = 15

        # Thread management

        print(f"[STDOUT] Sensor {self.sensor_id} - Process PID: {os.getpid()}")  # Direct stdout print
        # Print process and thread IDs
        #self.get_logger().info(f"Sensor {self.sensor_id} - Process PID: {os.getpid()}")
        self.thread = threading.Thread(target=self.run_sensor, daemon=True)
        self.thread.start()
        print(f"[STDOUT] Sensor {self.sensor_id} - Thread ID: {self.thread.ident}")
        self.get_logger().info(f"Sensor {self.sensor_id} - Thread ID: {self.thread.ident}")
    def measure_distance(self):
        lgpio.gpio_write(self.chip, self.trig_pin, 0)
        time.sleep(0.002)
        lgpio.gpio_write(self.chip, self.trig_pin, 1)
        time.sleep(0.00001)
        lgpio.gpio_write(self.chip, self.trig_pin, 0)

        timeout = time.time() + 0.02
        while lgpio.gpio_read(self.chip, self.echo_pin) == 0:
            if time.time() > timeout:
                return -1

        pulse_start = time.time()

        while lgpio.gpio_read(self.chip, self.echo_pin) == 1:
            if time.time() > timeout:
                return -1

        pulse_end = time.time()
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        return distance

    def reject_outliers(self, new_value):
        if abs(new_value - self.filtered_distance) > self.outlier_threshold:
            return self.filtered_distance
        return new_value

    def low_pass_filter(self, new_value):
        return self.alpha * new_value + (1 - self.alpha) * self.filtered_distance

    def run_sensor(self):
        while self.running and rclpy.ok():
            raw_distance = self.measure_distance()
            valid_distance = self.reject_outliers(raw_distance)
            self.filtered_distance = self.low_pass_filter(valid_distance)
            self.publish_distance()
            time.sleep(0.02)

    def publish_distance(self):
        if rclpy.ok() and self.running:
            msg = Range()
            msg.header.stamp =        self.get_clock().now().to_msg()
            msg.header.frame_id = f"ultrasonic_sensor_{self.sensor_id}"
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.52  # Example FOV in radians, adjust as needed
            msg.min_range = 3.0      # 3 cm minimum
            msg.max_range = 400.0       # 4 m maximum
            msg.range = float(max(self.filtered_distance, msg.min_range))
            self.publisher.publish(msg)
            self.get_logger().info(f"Sensor {self.sensor_id} Distance: {msg.range:.2f} cm")

    def stop(self):
        
        self.running = False
        if self.thread.is_alive():
            self.thread.join()
        if rclpy.ok():
            self.get_logger().info(f"Stopping Sensor")
        else:
            print(f"[STDOUT] Stopping Sensor {self.sensor_id}")
        super().destroy_node()

    def __del__(self):
        if hasattr(self, 'chip'):
            lgpio.gpiochip_close(self.chip)
        if rclpy.ok():
            self.get_logger().info(f"Sensor {self.sensor_id} resources cleaned up.")
        else:
            print(f"[STDOUT] Sensor {self.sensor_id} resources cleaned up.")


