import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ammar/Documents/Pendulum/pendulu_real_ws/install/linear_inverted_pendulum_real_sim'
