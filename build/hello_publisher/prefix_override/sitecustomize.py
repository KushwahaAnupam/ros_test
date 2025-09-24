import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tar-tt069-anupam/Pictures/hello_publisher/install/hello_publisher'
