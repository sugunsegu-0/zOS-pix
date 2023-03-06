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
CMAKE_SOURCE_DIR = /home/mz/Documents/zOS/src/SAL/gps

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mz/Documents/zOS/src/SAL/gps/build

# Include any dependencies generated for this target.
include CMakeFiles/gnss.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/gnss.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/gnss.dir/flags.make

CMakeFiles/gnss.dir/src/gnss.cpp.o: CMakeFiles/gnss.dir/flags.make
CMakeFiles/gnss.dir/src/gnss.cpp.o: ../src/gnss.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mz/Documents/zOS/src/SAL/gps/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/gnss.dir/src/gnss.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gnss.dir/src/gnss.cpp.o -c /home/mz/Documents/zOS/src/SAL/gps/src/gnss.cpp

CMakeFiles/gnss.dir/src/gnss.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gnss.dir/src/gnss.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mz/Documents/zOS/src/SAL/gps/src/gnss.cpp > CMakeFiles/gnss.dir/src/gnss.cpp.i

CMakeFiles/gnss.dir/src/gnss.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gnss.dir/src/gnss.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mz/Documents/zOS/src/SAL/gps/src/gnss.cpp -o CMakeFiles/gnss.dir/src/gnss.cpp.s

CMakeFiles/gnss.dir/src/libgpsmm.cpp.o: CMakeFiles/gnss.dir/flags.make
CMakeFiles/gnss.dir/src/libgpsmm.cpp.o: ../src/libgpsmm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mz/Documents/zOS/src/SAL/gps/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/gnss.dir/src/libgpsmm.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gnss.dir/src/libgpsmm.cpp.o -c /home/mz/Documents/zOS/src/SAL/gps/src/libgpsmm.cpp

CMakeFiles/gnss.dir/src/libgpsmm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gnss.dir/src/libgpsmm.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mz/Documents/zOS/src/SAL/gps/src/libgpsmm.cpp > CMakeFiles/gnss.dir/src/libgpsmm.cpp.i

CMakeFiles/gnss.dir/src/libgpsmm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gnss.dir/src/libgpsmm.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mz/Documents/zOS/src/SAL/gps/src/libgpsmm.cpp -o CMakeFiles/gnss.dir/src/libgpsmm.cpp.s

# Object files for target gnss
gnss_OBJECTS = \
"CMakeFiles/gnss.dir/src/gnss.cpp.o" \
"CMakeFiles/gnss.dir/src/libgpsmm.cpp.o"

# External object files for target gnss
gnss_EXTERNAL_OBJECTS =

gnss: CMakeFiles/gnss.dir/src/gnss.cpp.o
gnss: CMakeFiles/gnss.dir/src/libgpsmm.cpp.o
gnss: CMakeFiles/gnss.dir/build.make
gnss: /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.71.0
gnss: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0
gnss: /usr/lib/x86_64-linux-gnu/libecal_core.so.5.11.3
gnss: /home/mz/zOS/src/commons/build/libcommons.so
gnss: /usr/local/lib/libgps.so
gnss: /usr/lib/x86_64-linux-gnu/libecal_proto.a
gnss: /usr/lib/x86_64-linux-gnu/libprotobuf.so
gnss: CMakeFiles/gnss.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mz/Documents/zOS/src/SAL/gps/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable gnss"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gnss.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/gnss.dir/build: gnss

.PHONY : CMakeFiles/gnss.dir/build

CMakeFiles/gnss.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/gnss.dir/cmake_clean.cmake
.PHONY : CMakeFiles/gnss.dir/clean

CMakeFiles/gnss.dir/depend:
	cd /home/mz/Documents/zOS/src/SAL/gps/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mz/Documents/zOS/src/SAL/gps /home/mz/Documents/zOS/src/SAL/gps /home/mz/Documents/zOS/src/SAL/gps/build /home/mz/Documents/zOS/src/SAL/gps/build /home/mz/Documents/zOS/src/SAL/gps/build/CMakeFiles/gnss.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/gnss.dir/depend

