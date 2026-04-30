import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ammar/Documents/Pendulum/pendulum_pid_ws/install/linear_inverted_pendulum_pid_sim'
