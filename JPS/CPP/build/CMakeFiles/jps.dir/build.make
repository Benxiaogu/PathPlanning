# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/dawn/workspace/PathPlanning/JPS/CPP

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dawn/workspace/PathPlanning/JPS/CPP/build

# Include any dependencies generated for this target.
include CMakeFiles/jps.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/jps.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/jps.dir/flags.make

CMakeFiles/jps.dir/src/main.cpp.o: CMakeFiles/jps.dir/flags.make
CMakeFiles/jps.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dawn/workspace/PathPlanning/JPS/CPP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/jps.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/jps.dir/src/main.cpp.o -c /home/dawn/workspace/PathPlanning/JPS/CPP/src/main.cpp

CMakeFiles/jps.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/jps.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dawn/workspace/PathPlanning/JPS/CPP/src/main.cpp > CMakeFiles/jps.dir/src/main.cpp.i

CMakeFiles/jps.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/jps.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dawn/workspace/PathPlanning/JPS/CPP/src/main.cpp -o CMakeFiles/jps.dir/src/main.cpp.s

CMakeFiles/jps.dir/src/jps.cpp.o: CMakeFiles/jps.dir/flags.make
CMakeFiles/jps.dir/src/jps.cpp.o: ../src/jps.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dawn/workspace/PathPlanning/JPS/CPP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/jps.dir/src/jps.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/jps.dir/src/jps.cpp.o -c /home/dawn/workspace/PathPlanning/JPS/CPP/src/jps.cpp

CMakeFiles/jps.dir/src/jps.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/jps.dir/src/jps.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dawn/workspace/PathPlanning/JPS/CPP/src/jps.cpp > CMakeFiles/jps.dir/src/jps.cpp.i

CMakeFiles/jps.dir/src/jps.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/jps.dir/src/jps.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dawn/workspace/PathPlanning/JPS/CPP/src/jps.cpp -o CMakeFiles/jps.dir/src/jps.cpp.s

CMakeFiles/jps.dir/src/map.cpp.o: CMakeFiles/jps.dir/flags.make
CMakeFiles/jps.dir/src/map.cpp.o: ../src/map.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dawn/workspace/PathPlanning/JPS/CPP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/jps.dir/src/map.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/jps.dir/src/map.cpp.o -c /home/dawn/workspace/PathPlanning/JPS/CPP/src/map.cpp

CMakeFiles/jps.dir/src/map.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/jps.dir/src/map.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dawn/workspace/PathPlanning/JPS/CPP/src/map.cpp > CMakeFiles/jps.dir/src/map.cpp.i

CMakeFiles/jps.dir/src/map.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/jps.dir/src/map.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dawn/workspace/PathPlanning/JPS/CPP/src/map.cpp -o CMakeFiles/jps.dir/src/map.cpp.s

# Object files for target jps
jps_OBJECTS = \
"CMakeFiles/jps.dir/src/main.cpp.o" \
"CMakeFiles/jps.dir/src/jps.cpp.o" \
"CMakeFiles/jps.dir/src/map.cpp.o"

# External object files for target jps
jps_EXTERNAL_OBJECTS =

jps: CMakeFiles/jps.dir/src/main.cpp.o
jps: CMakeFiles/jps.dir/src/jps.cpp.o
jps: CMakeFiles/jps.dir/src/map.cpp.o
jps: CMakeFiles/jps.dir/build.make
jps: /usr/local/lib/libopencv_dnn.so.3.4.16
jps: /usr/local/lib/libopencv_highgui.so.3.4.16
jps: /usr/local/lib/libopencv_ml.so.3.4.16
jps: /usr/local/lib/libopencv_objdetect.so.3.4.16
jps: /usr/local/lib/libopencv_shape.so.3.4.16
jps: /usr/local/lib/libopencv_stitching.so.3.4.16
jps: /usr/local/lib/libopencv_superres.so.3.4.16
jps: /usr/local/lib/libopencv_videostab.so.3.4.16
jps: /usr/local/lib/libopencv_viz.so.3.4.16
jps: /usr/local/lib/libopencv_calib3d.so.3.4.16
jps: /usr/local/lib/libopencv_features2d.so.3.4.16
jps: /usr/local/lib/libopencv_flann.so.3.4.16
jps: /usr/local/lib/libopencv_photo.so.3.4.16
jps: /usr/local/lib/libopencv_video.so.3.4.16
jps: /usr/local/lib/libopencv_videoio.so.3.4.16
jps: /usr/local/lib/libopencv_imgcodecs.so.3.4.16
jps: /usr/local/lib/libopencv_imgproc.so.3.4.16
jps: /usr/local/lib/libopencv_core.so.3.4.16
jps: CMakeFiles/jps.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dawn/workspace/PathPlanning/JPS/CPP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable jps"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/jps.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/jps.dir/build: jps

.PHONY : CMakeFiles/jps.dir/build

CMakeFiles/jps.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/jps.dir/cmake_clean.cmake
.PHONY : CMakeFiles/jps.dir/clean

CMakeFiles/jps.dir/depend:
	cd /home/dawn/workspace/PathPlanning/JPS/CPP/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dawn/workspace/PathPlanning/JPS/CPP /home/dawn/workspace/PathPlanning/JPS/CPP /home/dawn/workspace/PathPlanning/JPS/CPP/build /home/dawn/workspace/PathPlanning/JPS/CPP/build /home/dawn/workspace/PathPlanning/JPS/CPP/build/CMakeFiles/jps.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/jps.dir/depend
