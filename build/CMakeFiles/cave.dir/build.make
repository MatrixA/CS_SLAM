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
CMAKE_SOURCE_DIR = /home/fernando/Code/CS_SLAM

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/fernando/Code/CS_SLAM/build

# Include any dependencies generated for this target.
include CMakeFiles/cave.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cave.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cave.dir/flags.make

CMakeFiles/cave.dir/examples/cave.cpp.o: CMakeFiles/cave.dir/flags.make
CMakeFiles/cave.dir/examples/cave.cpp.o: ../examples/cave.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/fernando/Code/CS_SLAM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/cave.dir/examples/cave.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cave.dir/examples/cave.cpp.o -c /home/fernando/Code/CS_SLAM/examples/cave.cpp

CMakeFiles/cave.dir/examples/cave.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cave.dir/examples/cave.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/fernando/Code/CS_SLAM/examples/cave.cpp > CMakeFiles/cave.dir/examples/cave.cpp.i

CMakeFiles/cave.dir/examples/cave.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cave.dir/examples/cave.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/fernando/Code/CS_SLAM/examples/cave.cpp -o CMakeFiles/cave.dir/examples/cave.cpp.s

# Object files for target cave
cave_OBJECTS = \
"CMakeFiles/cave.dir/examples/cave.cpp.o"

# External object files for target cave
cave_EXTERNAL_OBJECTS =

../examples/cave/cave: CMakeFiles/cave.dir/examples/cave.cpp.o
../examples/cave/cave: CMakeFiles/cave.dir/build.make
../examples/cave/cave: ../lib/libCS_SLAM.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
../examples/cave/cave: /usr/local/lib/libpango_glgeometry.so
../examples/cave/cave: /usr/local/lib/libpango_geometry.so
../examples/cave/cave: /usr/local/lib/libpango_plot.so
../examples/cave/cave: /usr/local/lib/libpango_python.so
../examples/cave/cave: /usr/local/lib/libpango_scene.so
../examples/cave/cave: /usr/local/lib/libpango_tools.so
../examples/cave/cave: /usr/local/lib/libpango_display.so
../examples/cave/cave: /usr/local/lib/libpango_vars.so
../examples/cave/cave: /usr/local/lib/libpango_video.so
../examples/cave/cave: /usr/local/lib/libpango_packetstream.so
../examples/cave/cave: /usr/local/lib/libpango_windowing.so
../examples/cave/cave: /usr/local/lib/libpango_opengl.so
../examples/cave/cave: /usr/local/lib/libpango_image.so
../examples/cave/cave: /usr/local/lib/libpango_core.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libGLEW.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libOpenGL.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libGLX.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libGLU.so
../examples/cave/cave: /usr/local/lib/libtinyobj.so
../examples/cave/cave: /usr/local/lib/libceres.a
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libglog.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libgflags.so.2.2.2
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libspqr.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libtbb.so
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
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libcxsparse.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/liblapack.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libf77blas.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libatlas.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/librt.so
../examples/cave/cave: /usr/lib/x86_64-linux-gnu/libcxsparse.so
../examples/cave/cave: /usr/local/lib/libgtest.a
../examples/cave/cave: /usr/local/lib/libgtest_main.a
../examples/cave/cave: CMakeFiles/cave.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/fernando/Code/CS_SLAM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../examples/cave/cave"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cave.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cave.dir/build: ../examples/cave/cave

.PHONY : CMakeFiles/cave.dir/build

CMakeFiles/cave.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cave.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cave.dir/clean

CMakeFiles/cave.dir/depend:
	cd /home/fernando/Code/CS_SLAM/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/fernando/Code/CS_SLAM /home/fernando/Code/CS_SLAM /home/fernando/Code/CS_SLAM/build /home/fernando/Code/CS_SLAM/build /home/fernando/Code/CS_SLAM/build/CMakeFiles/cave.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cave.dir/depend

