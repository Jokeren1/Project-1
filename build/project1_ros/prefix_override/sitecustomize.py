import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jokeren/Documents/ItIR/Project-1/install/project1_ros'
