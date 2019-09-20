# CMake generated Testfile for 
# Source directory: /home/FE/pilz/working/sidelink/srssl/src
# Build directory: /home/FE/pilz/working/sidelink/build/srssl/src
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(srssl_version "srssl" "--version")
add_test(srssl_help "srssl" "--help")
subdirs(phy)
subdirs(stack)
