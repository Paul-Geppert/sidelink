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
CMAKE_SOURCE_DIR = /home/FE/pilz/working/srsLTE

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/FE/pilz/working/srsLTE/debug

# Include any dependencies generated for this target.
include srsue/src/CMakeFiles/srsue.dir/depend.make

# Include the progress variables for this target.
include srsue/src/CMakeFiles/srsue.dir/progress.make

# Include the compile flags for this target's objects.
include srsue/src/CMakeFiles/srsue.dir/flags.make

srsue/src/CMakeFiles/srsue.dir/main.cc.o: srsue/src/CMakeFiles/srsue.dir/flags.make
srsue/src/CMakeFiles/srsue.dir/main.cc.o: ../srsue/src/main.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/FE/pilz/working/srsLTE/debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object srsue/src/CMakeFiles/srsue.dir/main.cc.o"
	cd /home/FE/pilz/working/srsLTE/debug/srsue/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/srsue.dir/main.cc.o -c /home/FE/pilz/working/srsLTE/srsue/src/main.cc

srsue/src/CMakeFiles/srsue.dir/main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/srsue.dir/main.cc.i"
	cd /home/FE/pilz/working/srsLTE/debug/srsue/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/FE/pilz/working/srsLTE/srsue/src/main.cc > CMakeFiles/srsue.dir/main.cc.i

srsue/src/CMakeFiles/srsue.dir/main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/srsue.dir/main.cc.s"
	cd /home/FE/pilz/working/srsLTE/debug/srsue/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/FE/pilz/working/srsLTE/srsue/src/main.cc -o CMakeFiles/srsue.dir/main.cc.s

srsue/src/CMakeFiles/srsue.dir/main.cc.o.requires:

.PHONY : srsue/src/CMakeFiles/srsue.dir/main.cc.o.requires

srsue/src/CMakeFiles/srsue.dir/main.cc.o.provides: srsue/src/CMakeFiles/srsue.dir/main.cc.o.requires
	$(MAKE) -f srsue/src/CMakeFiles/srsue.dir/build.make srsue/src/CMakeFiles/srsue.dir/main.cc.o.provides.build
.PHONY : srsue/src/CMakeFiles/srsue.dir/main.cc.o.provides

srsue/src/CMakeFiles/srsue.dir/main.cc.o.provides.build: srsue/src/CMakeFiles/srsue.dir/main.cc.o


srsue/src/CMakeFiles/srsue.dir/ue_base.cc.o: srsue/src/CMakeFiles/srsue.dir/flags.make
srsue/src/CMakeFiles/srsue.dir/ue_base.cc.o: ../srsue/src/ue_base.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/FE/pilz/working/srsLTE/debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object srsue/src/CMakeFiles/srsue.dir/ue_base.cc.o"
	cd /home/FE/pilz/working/srsLTE/debug/srsue/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/srsue.dir/ue_base.cc.o -c /home/FE/pilz/working/srsLTE/srsue/src/ue_base.cc

srsue/src/CMakeFiles/srsue.dir/ue_base.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/srsue.dir/ue_base.cc.i"
	cd /home/FE/pilz/working/srsLTE/debug/srsue/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/FE/pilz/working/srsLTE/srsue/src/ue_base.cc > CMakeFiles/srsue.dir/ue_base.cc.i

srsue/src/CMakeFiles/srsue.dir/ue_base.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/srsue.dir/ue_base.cc.s"
	cd /home/FE/pilz/working/srsLTE/debug/srsue/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/FE/pilz/working/srsLTE/srsue/src/ue_base.cc -o CMakeFiles/srsue.dir/ue_base.cc.s

srsue/src/CMakeFiles/srsue.dir/ue_base.cc.o.requires:

.PHONY : srsue/src/CMakeFiles/srsue.dir/ue_base.cc.o.requires

srsue/src/CMakeFiles/srsue.dir/ue_base.cc.o.provides: srsue/src/CMakeFiles/srsue.dir/ue_base.cc.o.requires
	$(MAKE) -f srsue/src/CMakeFiles/srsue.dir/build.make srsue/src/CMakeFiles/srsue.dir/ue_base.cc.o.provides.build
.PHONY : srsue/src/CMakeFiles/srsue.dir/ue_base.cc.o.provides

srsue/src/CMakeFiles/srsue.dir/ue_base.cc.o.provides.build: srsue/src/CMakeFiles/srsue.dir/ue_base.cc.o


srsue/src/CMakeFiles/srsue.dir/ue.cc.o: srsue/src/CMakeFiles/srsue.dir/flags.make
srsue/src/CMakeFiles/srsue.dir/ue.cc.o: ../srsue/src/ue.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/FE/pilz/working/srsLTE/debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object srsue/src/CMakeFiles/srsue.dir/ue.cc.o"
	cd /home/FE/pilz/working/srsLTE/debug/srsue/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/srsue.dir/ue.cc.o -c /home/FE/pilz/working/srsLTE/srsue/src/ue.cc

srsue/src/CMakeFiles/srsue.dir/ue.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/srsue.dir/ue.cc.i"
	cd /home/FE/pilz/working/srsLTE/debug/srsue/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/FE/pilz/working/srsLTE/srsue/src/ue.cc > CMakeFiles/srsue.dir/ue.cc.i

srsue/src/CMakeFiles/srsue.dir/ue.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/srsue.dir/ue.cc.s"
	cd /home/FE/pilz/working/srsLTE/debug/srsue/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/FE/pilz/working/srsLTE/srsue/src/ue.cc -o CMakeFiles/srsue.dir/ue.cc.s

srsue/src/CMakeFiles/srsue.dir/ue.cc.o.requires:

.PHONY : srsue/src/CMakeFiles/srsue.dir/ue.cc.o.requires

srsue/src/CMakeFiles/srsue.dir/ue.cc.o.provides: srsue/src/CMakeFiles/srsue.dir/ue.cc.o.requires
	$(MAKE) -f srsue/src/CMakeFiles/srsue.dir/build.make srsue/src/CMakeFiles/srsue.dir/ue.cc.o.provides.build
.PHONY : srsue/src/CMakeFiles/srsue.dir/ue.cc.o.provides

srsue/src/CMakeFiles/srsue.dir/ue.cc.o.provides.build: srsue/src/CMakeFiles/srsue.dir/ue.cc.o


srsue/src/CMakeFiles/srsue.dir/metrics_stdout.cc.o: srsue/src/CMakeFiles/srsue.dir/flags.make
srsue/src/CMakeFiles/srsue.dir/metrics_stdout.cc.o: ../srsue/src/metrics_stdout.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/FE/pilz/working/srsLTE/debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object srsue/src/CMakeFiles/srsue.dir/metrics_stdout.cc.o"
	cd /home/FE/pilz/working/srsLTE/debug/srsue/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/srsue.dir/metrics_stdout.cc.o -c /home/FE/pilz/working/srsLTE/srsue/src/metrics_stdout.cc

srsue/src/CMakeFiles/srsue.dir/metrics_stdout.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/srsue.dir/metrics_stdout.cc.i"
	cd /home/FE/pilz/working/srsLTE/debug/srsue/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/FE/pilz/working/srsLTE/srsue/src/metrics_stdout.cc > CMakeFiles/srsue.dir/metrics_stdout.cc.i

srsue/src/CMakeFiles/srsue.dir/metrics_stdout.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/srsue.dir/metrics_stdout.cc.s"
	cd /home/FE/pilz/working/srsLTE/debug/srsue/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/FE/pilz/working/srsLTE/srsue/src/metrics_stdout.cc -o CMakeFiles/srsue.dir/metrics_stdout.cc.s

srsue/src/CMakeFiles/srsue.dir/metrics_stdout.cc.o.requires:

.PHONY : srsue/src/CMakeFiles/srsue.dir/metrics_stdout.cc.o.requires

srsue/src/CMakeFiles/srsue.dir/metrics_stdout.cc.o.provides: srsue/src/CMakeFiles/srsue.dir/metrics_stdout.cc.o.requires
	$(MAKE) -f srsue/src/CMakeFiles/srsue.dir/build.make srsue/src/CMakeFiles/srsue.dir/metrics_stdout.cc.o.provides.build
.PHONY : srsue/src/CMakeFiles/srsue.dir/metrics_stdout.cc.o.provides

srsue/src/CMakeFiles/srsue.dir/metrics_stdout.cc.o.provides.build: srsue/src/CMakeFiles/srsue.dir/metrics_stdout.cc.o


srsue/src/CMakeFiles/srsue.dir/metrics_csv.cc.o: srsue/src/CMakeFiles/srsue.dir/flags.make
srsue/src/CMakeFiles/srsue.dir/metrics_csv.cc.o: ../srsue/src/metrics_csv.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/FE/pilz/working/srsLTE/debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object srsue/src/CMakeFiles/srsue.dir/metrics_csv.cc.o"
	cd /home/FE/pilz/working/srsLTE/debug/srsue/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/srsue.dir/metrics_csv.cc.o -c /home/FE/pilz/working/srsLTE/srsue/src/metrics_csv.cc

srsue/src/CMakeFiles/srsue.dir/metrics_csv.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/srsue.dir/metrics_csv.cc.i"
	cd /home/FE/pilz/working/srsLTE/debug/srsue/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/FE/pilz/working/srsLTE/srsue/src/metrics_csv.cc > CMakeFiles/srsue.dir/metrics_csv.cc.i

srsue/src/CMakeFiles/srsue.dir/metrics_csv.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/srsue.dir/metrics_csv.cc.s"
	cd /home/FE/pilz/working/srsLTE/debug/srsue/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/FE/pilz/working/srsLTE/srsue/src/metrics_csv.cc -o CMakeFiles/srsue.dir/metrics_csv.cc.s

srsue/src/CMakeFiles/srsue.dir/metrics_csv.cc.o.requires:

.PHONY : srsue/src/CMakeFiles/srsue.dir/metrics_csv.cc.o.requires

srsue/src/CMakeFiles/srsue.dir/metrics_csv.cc.o.provides: srsue/src/CMakeFiles/srsue.dir/metrics_csv.cc.o.requires
	$(MAKE) -f srsue/src/CMakeFiles/srsue.dir/build.make srsue/src/CMakeFiles/srsue.dir/metrics_csv.cc.o.provides.build
.PHONY : srsue/src/CMakeFiles/srsue.dir/metrics_csv.cc.o.provides

srsue/src/CMakeFiles/srsue.dir/metrics_csv.cc.o.provides.build: srsue/src/CMakeFiles/srsue.dir/metrics_csv.cc.o


# Object files for target srsue
srsue_OBJECTS = \
"CMakeFiles/srsue.dir/main.cc.o" \
"CMakeFiles/srsue.dir/ue_base.cc.o" \
"CMakeFiles/srsue.dir/ue.cc.o" \
"CMakeFiles/srsue.dir/metrics_stdout.cc.o" \
"CMakeFiles/srsue.dir/metrics_csv.cc.o"

# External object files for target srsue
srsue_EXTERNAL_OBJECTS =

srsue/src/srsue: srsue/src/CMakeFiles/srsue.dir/main.cc.o
srsue/src/srsue: srsue/src/CMakeFiles/srsue.dir/ue_base.cc.o
srsue/src/srsue: srsue/src/CMakeFiles/srsue.dir/ue.cc.o
srsue/src/srsue: srsue/src/CMakeFiles/srsue.dir/metrics_stdout.cc.o
srsue/src/srsue: srsue/src/CMakeFiles/srsue.dir/metrics_csv.cc.o
srsue/src/srsue: srsue/src/CMakeFiles/srsue.dir/build.make
srsue/src/srsue: srsue/src/mac/libsrsue_mac.a
srsue/src/srsue: srsue/src/phy/libsrsue_phy.a
srsue/src/srsue: srsue/src/upper/libsrsue_upper.a
srsue/src/srsue: lib/src/common/libsrslte_common.a
srsue/src/srsue: lib/src/phy/libsrslte_phy.a
srsue/src/srsue: lib/src/upper/libsrslte_upper.a
srsue/src/srsue: lib/src/radio/libsrslte_radio.a
srsue/src/srsue: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
srsue/src/srsue: lib/src/common/libsrslte_common.a
srsue/src/srsue: /usr/lib/x86_64-linux-gnu/libmbedcrypto.so
srsue/src/srsue: lib/src/asn1/libsrslte_asn1.a
srsue/src/srsue: lib/src/phy/rf/libsrslte_rf.so
srsue/src/srsue: lib/src/phy/rf/libsrslte_rf_utils.a
srsue/src/srsue: lib/src/phy/libsrslte_phy.a
srsue/src/srsue: /usr/lib/x86_64-linux-gnu/libfftw3f.so
srsue/src/srsue: /usr/local/lib/libuhd.so
srsue/src/srsue: srsue/src/CMakeFiles/srsue.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/FE/pilz/working/srsLTE/debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable srsue"
	cd /home/FE/pilz/working/srsLTE/debug/srsue/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/srsue.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
srsue/src/CMakeFiles/srsue.dir/build: srsue/src/srsue

.PHONY : srsue/src/CMakeFiles/srsue.dir/build

# Object files for target srsue
srsue_OBJECTS = \
"CMakeFiles/srsue.dir/main.cc.o" \
"CMakeFiles/srsue.dir/ue_base.cc.o" \
"CMakeFiles/srsue.dir/ue.cc.o" \
"CMakeFiles/srsue.dir/metrics_stdout.cc.o" \
"CMakeFiles/srsue.dir/metrics_csv.cc.o"

# External object files for target srsue
srsue_EXTERNAL_OBJECTS =

srsue/src/CMakeFiles/CMakeRelink.dir/srsue: srsue/src/CMakeFiles/srsue.dir/main.cc.o
srsue/src/CMakeFiles/CMakeRelink.dir/srsue: srsue/src/CMakeFiles/srsue.dir/ue_base.cc.o
srsue/src/CMakeFiles/CMakeRelink.dir/srsue: srsue/src/CMakeFiles/srsue.dir/ue.cc.o
srsue/src/CMakeFiles/CMakeRelink.dir/srsue: srsue/src/CMakeFiles/srsue.dir/metrics_stdout.cc.o
srsue/src/CMakeFiles/CMakeRelink.dir/srsue: srsue/src/CMakeFiles/srsue.dir/metrics_csv.cc.o
srsue/src/CMakeFiles/CMakeRelink.dir/srsue: srsue/src/CMakeFiles/srsue.dir/build.make
srsue/src/CMakeFiles/CMakeRelink.dir/srsue: srsue/src/mac/libsrsue_mac.a
srsue/src/CMakeFiles/CMakeRelink.dir/srsue: srsue/src/phy/libsrsue_phy.a
srsue/src/CMakeFiles/CMakeRelink.dir/srsue: srsue/src/upper/libsrsue_upper.a
srsue/src/CMakeFiles/CMakeRelink.dir/srsue: lib/src/common/libsrslte_common.a
srsue/src/CMakeFiles/CMakeRelink.dir/srsue: lib/src/phy/libsrslte_phy.a
srsue/src/CMakeFiles/CMakeRelink.dir/srsue: lib/src/upper/libsrslte_upper.a
srsue/src/CMakeFiles/CMakeRelink.dir/srsue: lib/src/radio/libsrslte_radio.a
srsue/src/CMakeFiles/CMakeRelink.dir/srsue: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
srsue/src/CMakeFiles/CMakeRelink.dir/srsue: lib/src/common/libsrslte_common.a
srsue/src/CMakeFiles/CMakeRelink.dir/srsue: /usr/lib/x86_64-linux-gnu/libmbedcrypto.so
srsue/src/CMakeFiles/CMakeRelink.dir/srsue: lib/src/asn1/libsrslte_asn1.a
srsue/src/CMakeFiles/CMakeRelink.dir/srsue: lib/src/phy/rf/libsrslte_rf.so
srsue/src/CMakeFiles/CMakeRelink.dir/srsue: lib/src/phy/rf/libsrslte_rf_utils.a
srsue/src/CMakeFiles/CMakeRelink.dir/srsue: lib/src/phy/libsrslte_phy.a
srsue/src/CMakeFiles/CMakeRelink.dir/srsue: /usr/lib/x86_64-linux-gnu/libfftw3f.so
srsue/src/CMakeFiles/CMakeRelink.dir/srsue: /usr/local/lib/libuhd.so
srsue/src/CMakeFiles/CMakeRelink.dir/srsue: srsue/src/CMakeFiles/srsue.dir/relink.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/FE/pilz/working/srsLTE/debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable CMakeFiles/CMakeRelink.dir/srsue"
	cd /home/FE/pilz/working/srsLTE/debug/srsue/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/srsue.dir/relink.txt --verbose=$(VERBOSE)

# Rule to relink during preinstall.
srsue/src/CMakeFiles/srsue.dir/preinstall: srsue/src/CMakeFiles/CMakeRelink.dir/srsue

.PHONY : srsue/src/CMakeFiles/srsue.dir/preinstall

srsue/src/CMakeFiles/srsue.dir/requires: srsue/src/CMakeFiles/srsue.dir/main.cc.o.requires
srsue/src/CMakeFiles/srsue.dir/requires: srsue/src/CMakeFiles/srsue.dir/ue_base.cc.o.requires
srsue/src/CMakeFiles/srsue.dir/requires: srsue/src/CMakeFiles/srsue.dir/ue.cc.o.requires
srsue/src/CMakeFiles/srsue.dir/requires: srsue/src/CMakeFiles/srsue.dir/metrics_stdout.cc.o.requires
srsue/src/CMakeFiles/srsue.dir/requires: srsue/src/CMakeFiles/srsue.dir/metrics_csv.cc.o.requires

.PHONY : srsue/src/CMakeFiles/srsue.dir/requires

srsue/src/CMakeFiles/srsue.dir/clean:
	cd /home/FE/pilz/working/srsLTE/debug/srsue/src && $(CMAKE_COMMAND) -P CMakeFiles/srsue.dir/cmake_clean.cmake
.PHONY : srsue/src/CMakeFiles/srsue.dir/clean

srsue/src/CMakeFiles/srsue.dir/depend:
	cd /home/FE/pilz/working/srsLTE/debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/FE/pilz/working/srsLTE /home/FE/pilz/working/srsLTE/srsue/src /home/FE/pilz/working/srsLTE/debug /home/FE/pilz/working/srsLTE/debug/srsue/src /home/FE/pilz/working/srsLTE/debug/srsue/src/CMakeFiles/srsue.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : srsue/src/CMakeFiles/srsue.dir/depend

