# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/lmy/download/clion-2021.2.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/lmy/download/clion-2021.2.4/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lmy/project/yolov5-deepsort-tensorrt

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lmy/project/yolov5-deepsort-tensorrt/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/yolosort.dir/depend.make
# Include the progress variables for this target.
include CMakeFiles/yolosort.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/yolosort.dir/flags.make

CMakeFiles/yolosort.dir/src/main.cpp.o: CMakeFiles/yolosort.dir/flags.make
CMakeFiles/yolosort.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lmy/project/yolov5-deepsort-tensorrt/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/yolosort.dir/src/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/yolosort.dir/src/main.cpp.o -c /home/lmy/project/yolov5-deepsort-tensorrt/src/main.cpp

CMakeFiles/yolosort.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/yolosort.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lmy/project/yolov5-deepsort-tensorrt/src/main.cpp > CMakeFiles/yolosort.dir/src/main.cpp.i

CMakeFiles/yolosort.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/yolosort.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lmy/project/yolov5-deepsort-tensorrt/src/main.cpp -o CMakeFiles/yolosort.dir/src/main.cpp.s

CMakeFiles/yolosort.dir/src/manager.cpp.o: CMakeFiles/yolosort.dir/flags.make
CMakeFiles/yolosort.dir/src/manager.cpp.o: ../src/manager.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lmy/project/yolov5-deepsort-tensorrt/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/yolosort.dir/src/manager.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/yolosort.dir/src/manager.cpp.o -c /home/lmy/project/yolov5-deepsort-tensorrt/src/manager.cpp

CMakeFiles/yolosort.dir/src/manager.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/yolosort.dir/src/manager.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lmy/project/yolov5-deepsort-tensorrt/src/manager.cpp > CMakeFiles/yolosort.dir/src/manager.cpp.i

CMakeFiles/yolosort.dir/src/manager.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/yolosort.dir/src/manager.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lmy/project/yolov5-deepsort-tensorrt/src/manager.cpp -o CMakeFiles/yolosort.dir/src/manager.cpp.s

CMakeFiles/yolosort.dir/src/objectProcess.cpp.o: CMakeFiles/yolosort.dir/flags.make
CMakeFiles/yolosort.dir/src/objectProcess.cpp.o: ../src/objectProcess.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lmy/project/yolov5-deepsort-tensorrt/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/yolosort.dir/src/objectProcess.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/yolosort.dir/src/objectProcess.cpp.o -c /home/lmy/project/yolov5-deepsort-tensorrt/src/objectProcess.cpp

CMakeFiles/yolosort.dir/src/objectProcess.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/yolosort.dir/src/objectProcess.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lmy/project/yolov5-deepsort-tensorrt/src/objectProcess.cpp > CMakeFiles/yolosort.dir/src/objectProcess.cpp.i

CMakeFiles/yolosort.dir/src/objectProcess.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/yolosort.dir/src/objectProcess.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lmy/project/yolov5-deepsort-tensorrt/src/objectProcess.cpp -o CMakeFiles/yolosort.dir/src/objectProcess.cpp.s

CMakeFiles/yolosort.dir/src/realsense_config.cpp.o: CMakeFiles/yolosort.dir/flags.make
CMakeFiles/yolosort.dir/src/realsense_config.cpp.o: ../src/realsense_config.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lmy/project/yolov5-deepsort-tensorrt/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/yolosort.dir/src/realsense_config.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/yolosort.dir/src/realsense_config.cpp.o -c /home/lmy/project/yolov5-deepsort-tensorrt/src/realsense_config.cpp

CMakeFiles/yolosort.dir/src/realsense_config.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/yolosort.dir/src/realsense_config.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lmy/project/yolov5-deepsort-tensorrt/src/realsense_config.cpp > CMakeFiles/yolosort.dir/src/realsense_config.cpp.i

CMakeFiles/yolosort.dir/src/realsense_config.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/yolosort.dir/src/realsense_config.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lmy/project/yolov5-deepsort-tensorrt/src/realsense_config.cpp -o CMakeFiles/yolosort.dir/src/realsense_config.cpp.s

CMakeFiles/yolosort.dir/src/thread.cpp.o: CMakeFiles/yolosort.dir/flags.make
CMakeFiles/yolosort.dir/src/thread.cpp.o: ../src/thread.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lmy/project/yolov5-deepsort-tensorrt/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/yolosort.dir/src/thread.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/yolosort.dir/src/thread.cpp.o -c /home/lmy/project/yolov5-deepsort-tensorrt/src/thread.cpp

CMakeFiles/yolosort.dir/src/thread.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/yolosort.dir/src/thread.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lmy/project/yolov5-deepsort-tensorrt/src/thread.cpp > CMakeFiles/yolosort.dir/src/thread.cpp.i

CMakeFiles/yolosort.dir/src/thread.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/yolosort.dir/src/thread.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lmy/project/yolov5-deepsort-tensorrt/src/thread.cpp -o CMakeFiles/yolosort.dir/src/thread.cpp.s

# Object files for target yolosort
yolosort_OBJECTS = \
"CMakeFiles/yolosort.dir/src/main.cpp.o" \
"CMakeFiles/yolosort.dir/src/manager.cpp.o" \
"CMakeFiles/yolosort.dir/src/objectProcess.cpp.o" \
"CMakeFiles/yolosort.dir/src/realsense_config.cpp.o" \
"CMakeFiles/yolosort.dir/src/thread.cpp.o"

# External object files for target yolosort
yolosort_EXTERNAL_OBJECTS =

yolosort: CMakeFiles/yolosort.dir/src/main.cpp.o
yolosort: CMakeFiles/yolosort.dir/src/manager.cpp.o
yolosort: CMakeFiles/yolosort.dir/src/objectProcess.cpp.o
yolosort: CMakeFiles/yolosort.dir/src/realsense_config.cpp.o
yolosort: CMakeFiles/yolosort.dir/src/thread.cpp.o
yolosort: CMakeFiles/yolosort.dir/build.make
yolosort: libyolov5_trt.so
yolosort: /usr/lib/x86_64-linux-gnu/librealsense2.so.2.50.0
yolosort: /usr/local/cuda/lib64/libcudart.so
yolosort: libdeepsort.so
yolosort: /usr/local/lib/libopencv_highgui.so.4.5.3
yolosort: /usr/local/lib/libopencv_ml.so.4.5.3
yolosort: /usr/local/lib/libopencv_objdetect.so.4.5.3
yolosort: /usr/local/lib/libopencv_photo.so.4.5.3
yolosort: /usr/local/lib/libopencv_stitching.so.4.5.3
yolosort: /usr/local/lib/libopencv_video.so.4.5.3
yolosort: /usr/local/lib/libopencv_calib3d.so.4.5.3
yolosort: /usr/local/lib/libopencv_dnn.so.4.5.3
yolosort: /usr/local/lib/libopencv_features2d.so.4.5.3
yolosort: /usr/local/lib/libopencv_flann.so.4.5.3
yolosort: /usr/local/lib/libopencv_videoio.so.4.5.3
yolosort: /usr/local/lib/libopencv_imgcodecs.so.4.5.3
yolosort: /usr/local/lib/libopencv_imgproc.so.4.5.3
yolosort: /usr/local/lib/libopencv_core.so.4.5.3
yolosort: CMakeFiles/yolosort.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lmy/project/yolov5-deepsort-tensorrt/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable yolosort"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/yolosort.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/yolosort.dir/build: yolosort
.PHONY : CMakeFiles/yolosort.dir/build

CMakeFiles/yolosort.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/yolosort.dir/cmake_clean.cmake
.PHONY : CMakeFiles/yolosort.dir/clean

CMakeFiles/yolosort.dir/depend:
	cd /home/lmy/project/yolov5-deepsort-tensorrt/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lmy/project/yolov5-deepsort-tensorrt /home/lmy/project/yolov5-deepsort-tensorrt /home/lmy/project/yolov5-deepsort-tensorrt/cmake-build-debug /home/lmy/project/yolov5-deepsort-tensorrt/cmake-build-debug /home/lmy/project/yolov5-deepsort-tensorrt/cmake-build-debug/CMakeFiles/yolosort.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/yolosort.dir/depend

