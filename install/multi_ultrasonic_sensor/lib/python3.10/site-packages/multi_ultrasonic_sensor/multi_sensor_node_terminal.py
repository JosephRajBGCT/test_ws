import rclpy
from rclpy.node import Node
import argparse
from .single_ultrasonic_sensor import UltrasonicSensor

class MultiSensorNode(Node):
    def __init__(self, sensor_configs):
        super().__init__('multi_sensor_node')
        
        # Initialize multiple sensors with their respective trigger and echo pins
        self.sensors = [
            UltrasonicSensor(trig_pin, echo_pin, sensor_id=sensor_id)
            for sensor_id, trig_pin, echo_pin in sensor_configs
        ]
        
        self.get_logger().info(f"Initialized {len(self.sensors)} ultrasonic sensors.")

def parse_arguments():
    parser = argparse.ArgumentParser(description="Multi Ultrasonic Sensor Node")
    
    parser.add_argument(
        '--sensors',
        nargs='+',
        metavar='SENSOR_CONFIG',  # Fixed metavar
        type=int,
        help='Provide sensor configurations as: SENSOR_ID TRIG_PIN ECHO_PIN ...',
        required=True
    )
    
    args = parser.parse_args()

    # Validate input length
    if len(args.sensors) % 3 != 0:
        raise ValueError("Provide sensor configurations in multiples of 3 (SENSOR_ID TRIG_PIN ECHO_PIN)")

    # Group into tuples (sensor_id, trig_pin, echo_pin)
    sensor_configs = [
        (args.sensors[i], args.sensors[i+1], args.sensors[i+2])
        for i in range(0, len(args.sensors), 3)
    ]
    
    return sensor_configs

def main(args=None):
    rclpy.init(args=args)
    sensor_configs = parse_arguments()
    node = MultiSensorNode(sensor_configs)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

