# CMake generated Testfile for 
# Source directory: /home/FE/pilz/working/srsLTE/srsue/test
# Build directory: /home/FE/pilz/working/srsLTE/debug/srsue/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(metrics_test "metrics_test" "-o" "/home/FE/pilz/working/srsLTE/debug/srsue/test/ue_metrics.csv")
subdirs(phy)
subdirs(mac)
subdirs(upper)
