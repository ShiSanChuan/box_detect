# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ssc/C++work/box_detect

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ssc/C++work/box_detect/build

# Include any dependencies generated for this target.
include src/CMakeFiles/savecap.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/savecap.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/savecap.dir/flags.make

src/CMakeFiles/savecap.dir/savepicfromcap.cpp.o: src/CMakeFiles/savecap.dir/flags.make
src/CMakeFiles/savecap.dir/savepicfromcap.cpp.o: ../src/savepicfromcap.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ssc/C++work/box_detect/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/savecap.dir/savepicfromcap.cpp.o"
	cd /home/ssc/C++work/box_detect/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/savecap.dir/savepicfromcap.cpp.o -c /home/ssc/C++work/box_detect/src/savepicfromcap.cpp

src/CMakeFiles/savecap.dir/savepicfromcap.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/savecap.dir/savepicfromcap.cpp.i"
	cd /home/ssc/C++work/box_detect/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ssc/C++work/box_detect/src/savepicfromcap.cpp > CMakeFiles/savecap.dir/savepicfromcap.cpp.i

src/CMakeFiles/savecap.dir/savepicfromcap.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/savecap.dir/savepicfromcap.cpp.s"
	cd /home/ssc/C++work/box_detect/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ssc/C++work/box_detect/src/savepicfromcap.cpp -o CMakeFiles/savecap.dir/savepicfromcap.cpp.s

# Object files for target savecap
savecap_OBJECTS = \
"CMakeFiles/savecap.dir/savepicfromcap.cpp.o"

# External object files for target savecap
savecap_EXTERNAL_OBJECTS =

src/savecap: src/CMakeFiles/savecap.dir/savepicfromcap.cpp.o
src/savecap: src/CMakeFiles/savecap.dir/build.make
src/savecap: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.9
src/savecap: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.9
src/savecap: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.9
src/savecap: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.9
src/savecap: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.9
src/savecap: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.9
src/savecap: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.9
src/savecap: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.9
src/savecap: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.9
src/savecap: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.9
src/savecap: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.9
src/savecap: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.9
src/savecap: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.9
src/savecap: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.9
src/savecap: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.9
src/savecap: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.9
src/savecap: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.9
src/savecap: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.9
src/savecap: src/CMakeFiles/savecap.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ssc/C++work/box_detect/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable savecap"
	cd /home/ssc/C++work/box_detect/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/savecap.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/savecap.dir/build: src/savecap

.PHONY : src/CMakeFiles/savecap.dir/build

src/CMakeFiles/savecap.dir/clean:
	cd /home/ssc/C++work/box_detect/build/src && $(CMAKE_COMMAND) -P CMakeFiles/savecap.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/savecap.dir/clean

src/CMakeFiles/savecap.dir/depend:
	cd /home/ssc/C++work/box_detect/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ssc/C++work/box_detect /home/ssc/C++work/box_detect/src /home/ssc/C++work/box_detect/build /home/ssc/C++work/box_detect/build/src /home/ssc/C++work/box_detect/build/src/CMakeFiles/savecap.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/savecap.dir/depend

