# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_SOURCE_DIR = /home/pi/sicktim/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/sicktim/build

# Utility rule file for run_tests_sick_tim.

# Include the progress variables for this target.
include sick_tim/CMakeFiles/run_tests_sick_tim.dir/progress.make

run_tests_sick_tim: sick_tim/CMakeFiles/run_tests_sick_tim.dir/build.make

.PHONY : run_tests_sick_tim

# Rule to build all files generated by this target.
sick_tim/CMakeFiles/run_tests_sick_tim.dir/build: run_tests_sick_tim

.PHONY : sick_tim/CMakeFiles/run_tests_sick_tim.dir/build

sick_tim/CMakeFiles/run_tests_sick_tim.dir/clean:
	cd /home/pi/sicktim/build/sick_tim && $(CMAKE_COMMAND) -P CMakeFiles/run_tests_sick_tim.dir/cmake_clean.cmake
.PHONY : sick_tim/CMakeFiles/run_tests_sick_tim.dir/clean

sick_tim/CMakeFiles/run_tests_sick_tim.dir/depend:
	cd /home/pi/sicktim/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/sicktim/src /home/pi/sicktim/src/sick_tim /home/pi/sicktim/build /home/pi/sicktim/build/sick_tim /home/pi/sicktim/build/sick_tim/CMakeFiles/run_tests_sick_tim.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : sick_tim/CMakeFiles/run_tests_sick_tim.dir/depend

