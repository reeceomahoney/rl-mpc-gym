# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/romahoney/4yp/raisim_mpc

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/romahoney/4yp/raisim_mpc/build

# Include any dependencies generated for this target.
include CMakeFiles/whole_body_controller.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/whole_body_controller.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/whole_body_controller.dir/flags.make

CMakeFiles/whole_body_controller.dir/main.cpp.o: CMakeFiles/whole_body_controller.dir/flags.make
CMakeFiles/whole_body_controller.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/romahoney/4yp/raisim_mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/whole_body_controller.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/whole_body_controller.dir/main.cpp.o -c /home/romahoney/4yp/raisim_mpc/main.cpp

CMakeFiles/whole_body_controller.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/whole_body_controller.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/romahoney/4yp/raisim_mpc/main.cpp > CMakeFiles/whole_body_controller.dir/main.cpp.i

CMakeFiles/whole_body_controller.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/whole_body_controller.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/romahoney/4yp/raisim_mpc/main.cpp -o CMakeFiles/whole_body_controller.dir/main.cpp.s

CMakeFiles/whole_body_controller.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/whole_body_controller.dir/main.cpp.o.requires

CMakeFiles/whole_body_controller.dir/main.cpp.o.provides: CMakeFiles/whole_body_controller.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/whole_body_controller.dir/build.make CMakeFiles/whole_body_controller.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/whole_body_controller.dir/main.cpp.o.provides

CMakeFiles/whole_body_controller.dir/main.cpp.o.provides.build: CMakeFiles/whole_body_controller.dir/main.cpp.o


CMakeFiles/whole_body_controller.dir/src/a1.cpp.o: CMakeFiles/whole_body_controller.dir/flags.make
CMakeFiles/whole_body_controller.dir/src/a1.cpp.o: ../src/a1.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/romahoney/4yp/raisim_mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/whole_body_controller.dir/src/a1.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/whole_body_controller.dir/src/a1.cpp.o -c /home/romahoney/4yp/raisim_mpc/src/a1.cpp

CMakeFiles/whole_body_controller.dir/src/a1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/whole_body_controller.dir/src/a1.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/romahoney/4yp/raisim_mpc/src/a1.cpp > CMakeFiles/whole_body_controller.dir/src/a1.cpp.i

CMakeFiles/whole_body_controller.dir/src/a1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/whole_body_controller.dir/src/a1.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/romahoney/4yp/raisim_mpc/src/a1.cpp -o CMakeFiles/whole_body_controller.dir/src/a1.cpp.s

CMakeFiles/whole_body_controller.dir/src/a1.cpp.o.requires:

.PHONY : CMakeFiles/whole_body_controller.dir/src/a1.cpp.o.requires

CMakeFiles/whole_body_controller.dir/src/a1.cpp.o.provides: CMakeFiles/whole_body_controller.dir/src/a1.cpp.o.requires
	$(MAKE) -f CMakeFiles/whole_body_controller.dir/build.make CMakeFiles/whole_body_controller.dir/src/a1.cpp.o.provides.build
.PHONY : CMakeFiles/whole_body_controller.dir/src/a1.cpp.o.provides

CMakeFiles/whole_body_controller.dir/src/a1.cpp.o.provides.build: CMakeFiles/whole_body_controller.dir/src/a1.cpp.o


# Object files for target whole_body_controller
whole_body_controller_OBJECTS = \
"CMakeFiles/whole_body_controller.dir/main.cpp.o" \
"CMakeFiles/whole_body_controller.dir/src/a1.cpp.o"

# External object files for target whole_body_controller
whole_body_controller_EXTERNAL_OBJECTS =

../bin/whole_body_controller: CMakeFiles/whole_body_controller.dir/main.cpp.o
../bin/whole_body_controller: CMakeFiles/whole_body_controller.dir/src/a1.cpp.o
../bin/whole_body_controller: CMakeFiles/whole_body_controller.dir/build.make
../bin/whole_body_controller: ../libs/raisim/lib/libraisim.so
../bin/whole_body_controller: ../libs/raisim/lib/libraisimPng.so
../bin/whole_body_controller: ../libs/raisim/lib/libraisimZ.so
../bin/whole_body_controller: ../libs/raisim/lib/libraisimODE.so
../bin/whole_body_controller: ../libs/raisim/lib/libraisimMine.so
../bin/whole_body_controller: CMakeFiles/whole_body_controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/romahoney/4yp/raisim_mpc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../bin/whole_body_controller"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/whole_body_controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/whole_body_controller.dir/build: ../bin/whole_body_controller

.PHONY : CMakeFiles/whole_body_controller.dir/build

CMakeFiles/whole_body_controller.dir/requires: CMakeFiles/whole_body_controller.dir/main.cpp.o.requires
CMakeFiles/whole_body_controller.dir/requires: CMakeFiles/whole_body_controller.dir/src/a1.cpp.o.requires

.PHONY : CMakeFiles/whole_body_controller.dir/requires

CMakeFiles/whole_body_controller.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/whole_body_controller.dir/cmake_clean.cmake
.PHONY : CMakeFiles/whole_body_controller.dir/clean

CMakeFiles/whole_body_controller.dir/depend:
	cd /home/romahoney/4yp/raisim_mpc/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/romahoney/4yp/raisim_mpc /home/romahoney/4yp/raisim_mpc /home/romahoney/4yp/raisim_mpc/build /home/romahoney/4yp/raisim_mpc/build /home/romahoney/4yp/raisim_mpc/build/CMakeFiles/whole_body_controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/whole_body_controller.dir/depend
