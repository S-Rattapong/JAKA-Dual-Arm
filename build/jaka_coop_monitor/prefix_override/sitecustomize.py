import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/newtonrattapong/jaka_ws/install/jaka_coop_monitor'
