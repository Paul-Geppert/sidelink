# CMake generated Testfile for 
# Source directory: /home/FE/pilz/working/sidelink/lib/src/phy/ch_estimation/test
# Build directory: /home/FE/pilz/working/sidelink/build/lib/src/phy/ch_estimation/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(chest_test_dl_cellid0 "chest_test_dl" "-c" "0")
add_test(chest_test_dl_cellid1 "chest_test_dl" "-c" "1")
add_test(chest_test_dl_cellid2 "chest_test_dl" "-c" "2")
add_test(chest_test_dl_cellid0_50prb "chest_test_dl" "-c" "0" "-r" "50")
add_test(chest_test_dl_cellid1_50prb "chest_test_dl" "-c" "1" "-r" "50")
add_test(chest_test_dl_cellid2_50prb "chest_test_dl" "-c" "2" "-r" "50")
add_test(chest_test_ul_cellid0 "chest_test_ul" "-c" "0" "-r" "50")
add_test(chest_test_ul_cellid1 "chest_test_ul" "-c" "2" "-r" "50")
add_test(refsignal_sl_test "refsignal_sl_test" "-r" "50")
