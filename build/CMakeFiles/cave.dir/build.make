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
CMAKE_SOURCE_DIR = /home/fernando/SonarSLAM

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/fernando/SonarSLAM/build

# Include any dependencies generated for this target.
include CMakeFiles/cave.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cave.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cave.dir/flags.make

CMakeFiles/cave.dir/examples/cave.cpp.o: CMakeFiles/cave.dir/flags.make
CMakeFiles/cave.dir/examples/cave.cpp.o: ../examples/cave.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/fernando/SonarSLAM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/cave.dir/examples/cave.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cave.dir/examples/cave.cpp.o -c /home/fernando/SonarSLAM/examples/cave.cpp

CMakeFiles/cave.dir/examples/cave.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cave.dir/examples/cave.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/fernando/SonarSLAM/examples/cave.cpp > CMakeFiles/cave.dir/examples/cave.cpp.i

CMakeFiles/cave.dir/examples/cave.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cave.dir/examples/cave.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/fernando/SonarSLAM/examples/cave.cpp -o CMakeFiles/cave.dir/examples/cave.cpp.s

# Object files for target cave
cave_OBJECTS = \
"CMakeFiles/cave.dir/examples/cave.cpp.o"

# External object files for target cave
cave_EXTERNAL_OBJECTS =

../examples/cave/cave: CMakeFiles/cave.dir/examples/cave.cpp.o
../examples/cave/cave: CMakeFiles/cave.dir/build.make
../examples/cave/cave: ../lib/libCS_SLAM.so
../examples/cave/cave: /usr/local/lib/libopencv_dnn.so.4.5.0
../examples/cave/cave: /usr/local/lib/libopencv_highgui.so.4.5.0
../examples/cave/cave: /usr/local/lib/libopencv_ml.so.4.5.0
../examples/cave/cave: /usr/local/lib/libopencv_objdetect.so.4.5.0
../examples/cave/cave: /usr/local/lib/libopencv_photo.so.4.5.0
../examples/cave/cave: /usr/local/lib/libopencv_stitching.so.4.5.0
../examples/cave/cave: /usr/local/lib/libopencv_video.so.4.5.0
../examples/cave/cave: /usr/local/lib/libopencv_calib3d.so.4.5.0
../examples/cave/cave: /usr/local/lib/libopencv_features2d.so.4.5.0
../examples/cave/cave: /usr/local/lib/libopencv_flann.so.4.5.0
../examples/cave/cave: /usr/local/lib/libopencv_videoio.so.4.5.0
../examples/cave/cave: /usr/local/lib/libopencv_imgcodecs.so.4.5.0
../examples/cave/cave: /usr/local/lib/libopencv_imgproc.so.4.5.0
../examples/cave/cave: /usr/local/lib/libopencv_core.so.4.5.0
../examples/cave/cave: /usr/local/lib/libpangolin.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libOpenGL.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libGLX.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libGLU.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libGLEW.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libEGL.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libSM.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libICE.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libX11.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libXext.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libOpenGL.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libGLX.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libGLU.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libGLEW.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libEGL.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libSM.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libICE.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libX11.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libXext.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libdc1394.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libavcodec.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libavformat.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libavutil.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libswscale.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libavdevice.so
../examples/cave/cave: /usr/lib/libOpenNI.so
../examples/cave/cave: /usr/lib/libOpenNI2.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libpng.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libz.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libjpeg.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libtiff.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libIlmImf.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libzstd.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/liblz4.so
../examples/cave/cave: /usr/local/lib/libceres.a
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libglog.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libgflags.so.2.2.2
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libspqr.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libtbb.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libtbbmalloc.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libcholmod.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libccolamd.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libcamd.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libcolamd.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libamd.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/liblapack.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libf77blas.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libatlas.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/librt.so
../examples/cave/cave: /usr/local/lib/libmetis.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libcxsparse.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/liblapack.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libf77blas.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libatlas.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/librt.so
../examples/cave/cave: /usr/local/lib/libmetis.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libcxsparse.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libyaml-cpp.so.0.6.2
../examples/cave/cave: /usr/local/lib/libgtest.a
../examples/cave/cave: /usr/local/lib/libgtest_main.a
../examples/cave/cave: CMakeFiles/cave.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/fernando/SonarSLAM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../examples/cave/cave"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cave.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cave.dir/build: ../examples/cave/cave

.PHONY : CMakeFiles/cave.dir/build

CMakeFiles/cave.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cave.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cave.dir/clean

CMakeFiles/cave.dir/depend:
	cd /home/fernando/SonarSLAM/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/fernando/SonarSLAM /home/fernando/SonarSLAM /home/fernando/SonarSLAM/build /home/fernando/SonarSLAM/build /home/fernando/SonarSLAM/build/CMakeFiles/cave.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cave.dir/depend

