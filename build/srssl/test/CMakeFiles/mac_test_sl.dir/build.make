# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/FE/pilz/working/sidelink

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/FE/pilz/working/sidelink/build

# Include any dependencies generated for this target.
include srssl/test/CMakeFiles/mac_test_sl.dir/depend.make

# Include the progress variables for this target.
include srssl/test/CMakeFiles/mac_test_sl.dir/progress.make

# Include the compile flags for this target's objects.
include srssl/test/CMakeFiles/mac_test_sl.dir/flags.make

srssl/test/CMakeFiles/mac_test_sl.dir/mac_test.cc.o: srssl/test/CMakeFiles/mac_test_sl.dir/flags.make
srssl/test/CMakeFiles/mac_test_sl.dir/mac_test.cc.o: ../srssl/test/mac_test.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/FE/pilz/working/sidelink/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object srssl/test/CMakeFiles/mac_test_sl.dir/mac_test.cc.o"
	cd /home/FE/pilz/working/sidelink/build/srssl/test && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mac_test_sl.dir/mac_test.cc.o -c /home/FE/pilz/working/sidelink/srssl/test/mac_test.cc

srssl/test/CMakeFiles/mac_test_sl.dir/mac_test.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mac_test_sl.dir/mac_test.cc.i"
	cd /home/FE/pilz/working/sidelink/build/srssl/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/FE/pilz/working/sidelink/srssl/test/mac_test.cc > CMakeFiles/mac_test_sl.dir/mac_test.cc.i

srssl/test/CMakeFiles/mac_test_sl.dir/mac_test.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mac_test_sl.dir/mac_test.cc.s"
	cd /home/FE/pilz/working/sidelink/build/srssl/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/FE/pilz/working/sidelink/srssl/test/mac_test.cc -o CMakeFiles/mac_test_sl.dir/mac_test.cc.s

srssl/test/CMakeFiles/mac_test_sl.dir/mac_test.cc.o.requires:

.PHONY : srssl/test/CMakeFiles/mac_test_sl.dir/mac_test.cc.o.requires

srssl/test/CMakeFiles/mac_test_sl.dir/mac_test.cc.o.provides: srssl/test/CMakeFiles/mac_test_sl.dir/mac_test.cc.o.requires
	$(MAKE) -f srssl/test/CMakeFiles/mac_test_sl.dir/build.make srssl/test/CMakeFiles/mac_test_sl.dir/mac_test.cc.o.provides.build
.PHONY : srssl/test/CMakeFiles/mac_test_sl.dir/mac_test.cc.o.provides

srssl/test/CMakeFiles/mac_test_sl.dir/mac_test.cc.o.provides.build: srssl/test/CMakeFiles/mac_test_sl.dir/mac_test.cc.o


# Object files for target mac_test_sl
mac_test_sl_OBJECTS = \
"CMakeFiles/mac_test_sl.dir/mac_test.cc.o"

# External object files for target mac_test_sl
mac_test_sl_EXTERNAL_OBJECTS =

srssl/test/mac_test_sl: srssl/test/CMakeFiles/mac_test_sl.dir/mac_test.cc.o
srssl/test/mac_test_sl: srssl/test/CMakeFiles/mac_test_sl.dir/build.make
srssl/test/mac_test_sl: srssl/src/stack/mac/libsrssl_mac.a
srssl/test/mac_test_sl: srssl/src/phy/libsrssl_phy.a
srssl/test/mac_test_sl: lib/src/common/libsrslte_common.a
srssl/test/mac_test_sl: lib/src/phy/libsrslte_phy.a
srssl/test/mac_test_sl: lib/src/radio/libsrslte_radio.a
srssl/test/mac_test_sl: lib/src/asn1/libsrslte_asn1.a
srssl/test/mac_test_sl: lib/src/asn1/librrc_asn1.a
srssl/test/mac_test_sl: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
srssl/test/mac_test_sl: /usr/lib/x86_64-linux-gnu/libmbedcrypto.so
srssl/test/mac_test_sl: lib/src/phy/rf/libsrslte_rf.so
srssl/test/mac_test_sl: lib/src/phy/rf/libsrslte_rf_utils.a
srssl/test/mac_test_sl: lib/src/phy/libsrslte_phy.a
srssl/test/mac_test_sl: /usr/lib/x86_64-linux-gnu/libfftw3f.so
srssl/test/mac_test_sl: /usr/local/lib/libuhd.so
srssl/test/mac_test_sl: srssl/test/CMakeFiles/mac_test_sl.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/FE/pilz/working/sidelink/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable mac_test_sl"
	cd /home/FE/pilz/working/sidelink/build/srssl/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mac_test_sl.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
srssl/test/CMakeFiles/mac_test_sl.dir/build: srssl/test/mac_test_sl

.PHONY : srssl/test/CMakeFiles/mac_test_sl.dir/build

srssl/test/CMakeFiles/mac_test_sl.dir/requires: srssl/test/CMakeFiles/mac_test_sl.dir/mac_test.cc.o.requires

.PHONY : srssl/test/CMakeFiles/mac_test_sl.dir/requires

srssl/test/CMakeFiles/mac_test_sl.dir/clean:
	cd /home/FE/pilz/working/sidelink/build/srssl/test && $(CMAKE_COMMAND) -P CMakeFiles/mac_test_sl.dir/cmake_clean.cmake
.PHONY : srssl/test/CMakeFiles/mac_test_sl.dir/clean

srssl/test/CMakeFiles/mac_test_sl.dir/depend:
	cd /home/FE/pilz/working/sidelink/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/FE/pilz/working/sidelink /home/FE/pilz/working/sidelink/srssl/test /home/FE/pilz/working/sidelink/build /home/FE/pilz/working/sidelink/build/srssl/test /home/FE/pilz/working/sidelink/build/srssl/test/CMakeFiles/mac_test_sl.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : srssl/test/CMakeFiles/mac_test_sl.dir/depend

