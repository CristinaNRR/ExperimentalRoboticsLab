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
CMAKE_SOURCE_DIR = /home/cristina/new_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cristina/new_ws/build

# Utility rule file for clean_test_results_gmapping.

# Include the progress variables for this target.
include SLAM_packages/slam_gmapping/gmapping/CMakeFiles/clean_test_results_gmapping.dir/progress.make

SLAM_packages/slam_gmapping/gmapping/CMakeFiles/clean_test_results_gmapping:
	cd /home/cristina/new_ws/build/SLAM_packages/slam_gmapping/gmapping && /usr/bin/python2 /opt/ros/kinetic/share/catkin/cmake/test/remove_test_results.py /home/cristina/new_ws/build/test_results/gmapping

clean_test_results_gmapping: SLAM_packages/slam_gmapping/gmapping/CMakeFiles/clean_test_results_gmapping
clean_test_results_gmapping: SLAM_packages/slam_gmapping/gmapping/CMakeFiles/clean_test_results_gmapping.dir/build.make

.PHONY : clean_test_results_gmapping

# Rule to build all files generated by this target.
SLAM_packages/slam_gmapping/gmapping/CMakeFiles/clean_test_results_gmapping.dir/build: clean_test_results_gmapping

.PHONY : SLAM_packages/slam_gmapping/gmapping/CMakeFiles/clean_test_results_gmapping.dir/build

SLAM_packages/slam_gmapping/gmapping/CMakeFiles/clean_test_results_gmapping.dir/clean:
	cd /home/cristina/new_ws/build/SLAM_packages/slam_gmapping/gmapping && $(CMAKE_COMMAND) -P CMakeFiles/clean_test_results_gmapping.dir/cmake_clean.cmake
.PHONY : SLAM_packages/slam_gmapping/gmapping/CMakeFiles/clean_test_results_gmapping.dir/clean

SLAM_packages/slam_gmapping/gmapping/CMakeFiles/clean_test_results_gmapping.dir/depend:
	cd /home/cristina/new_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cristina/new_ws/src /home/cristina/new_ws/src/SLAM_packages/slam_gmapping/gmapping /home/cristina/new_ws/build /home/cristina/new_ws/build/SLAM_packages/slam_gmapping/gmapping /home/cristina/new_ws/build/SLAM_packages/slam_gmapping/gmapping/CMakeFiles/clean_test_results_gmapping.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : SLAM_packages/slam_gmapping/gmapping/CMakeFiles/clean_test_results_gmapping.dir/depend
