import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vivit/Projects/smr_tutorial/install/smr_tutorial'
