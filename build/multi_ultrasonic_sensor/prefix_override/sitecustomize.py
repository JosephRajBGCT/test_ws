import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sabbi/test_ws/install/multi_ultrasonic_sensor'
