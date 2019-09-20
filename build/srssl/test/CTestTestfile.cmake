# CMake generated Testfile for 
# Source directory: /home/FE/pilz/working/sidelink/srssl/test
# Build directory: /home/FE/pilz/working/sidelink/build/srssl/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(metrics_test_sl "metrics_test_sl" "-o" "/home/FE/pilz/working/sidelink/build/srssl/test/ue_metrics.csv")
add_test(mac_test_sl "mac_test_sl")
add_test(rest_test "rest_test")
