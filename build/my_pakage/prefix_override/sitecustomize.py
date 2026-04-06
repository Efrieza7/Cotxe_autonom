import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/enric-linux/Cotxe_autonom/install/my_pakage'
