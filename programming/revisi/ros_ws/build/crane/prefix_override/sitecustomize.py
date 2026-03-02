import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/putra-adnyana/kuliah/magang-bayucaraka/FP-Bayucaraka/programming/revisi/ros_ws/install/crane'
