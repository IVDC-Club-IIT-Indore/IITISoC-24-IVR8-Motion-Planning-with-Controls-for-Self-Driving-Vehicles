import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/manan/IITISOC/IITISoC-24-IVR8-Motion-Planning-with-Controls-for-Self-Driving-Vehicles/bot_ws/src/install/path_planner'
