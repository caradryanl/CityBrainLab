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
CMAKE_SOURCE_DIR = /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build

# Include any dependencies generated for this target.
include src/CMakeFiles/src.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/src.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/src.dir/flags.make

src/CMakeFiles/src.dir/checker.cpp.o: src/CMakeFiles/src.dir/flags.make
src/CMakeFiles/src.dir/checker.cpp.o: ../src/checker.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/src.dir/checker.cpp.o"
	cd /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/checker.cpp.o -c /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/src/checker.cpp

src/CMakeFiles/src.dir/checker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/checker.cpp.i"
	cd /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/src/checker.cpp > CMakeFiles/src.dir/checker.cpp.i

src/CMakeFiles/src.dir/checker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/checker.cpp.s"
	cd /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/src/checker.cpp -o CMakeFiles/src.dir/checker.cpp.s

src/CMakeFiles/src.dir/filter.cpp.o: src/CMakeFiles/src.dir/flags.make
src/CMakeFiles/src.dir/filter.cpp.o: ../src/filter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/src.dir/filter.cpp.o"
	cd /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/filter.cpp.o -c /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/src/filter.cpp

src/CMakeFiles/src.dir/filter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/filter.cpp.i"
	cd /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/src/filter.cpp > CMakeFiles/src.dir/filter.cpp.i

src/CMakeFiles/src.dir/filter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/filter.cpp.s"
	cd /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/src/filter.cpp -o CMakeFiles/src.dir/filter.cpp.s

src/CMakeFiles/src.dir/graph.cpp.o: src/CMakeFiles/src.dir/flags.make
src/CMakeFiles/src.dir/graph.cpp.o: ../src/graph.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/CMakeFiles/src.dir/graph.cpp.o"
	cd /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/graph.cpp.o -c /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/src/graph.cpp

src/CMakeFiles/src.dir/graph.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/graph.cpp.i"
	cd /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/src/graph.cpp > CMakeFiles/src.dir/graph.cpp.i

src/CMakeFiles/src.dir/graph.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/graph.cpp.s"
	cd /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/src/graph.cpp -o CMakeFiles/src.dir/graph.cpp.s

src/CMakeFiles/src.dir/kdtree.cpp.o: src/CMakeFiles/src.dir/flags.make
src/CMakeFiles/src.dir/kdtree.cpp.o: ../src/kdtree.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/CMakeFiles/src.dir/kdtree.cpp.o"
	cd /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/kdtree.cpp.o -c /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/src/kdtree.cpp

src/CMakeFiles/src.dir/kdtree.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/kdtree.cpp.i"
	cd /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/src/kdtree.cpp > CMakeFiles/src.dir/kdtree.cpp.i

src/CMakeFiles/src.dir/kdtree.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/kdtree.cpp.s"
	cd /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/src/kdtree.cpp -o CMakeFiles/src.dir/kdtree.cpp.s

src/CMakeFiles/src.dir/log.cpp.o: src/CMakeFiles/src.dir/flags.make
src/CMakeFiles/src.dir/log.cpp.o: ../src/log.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/CMakeFiles/src.dir/log.cpp.o"
	cd /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/log.cpp.o -c /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/src/log.cpp

src/CMakeFiles/src.dir/log.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/log.cpp.i"
	cd /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/src/log.cpp > CMakeFiles/src.dir/log.cpp.i

src/CMakeFiles/src.dir/log.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/log.cpp.s"
	cd /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/src/log.cpp -o CMakeFiles/src.dir/log.cpp.s

src/CMakeFiles/src.dir/matcher.cpp.o: src/CMakeFiles/src.dir/flags.make
src/CMakeFiles/src.dir/matcher.cpp.o: ../src/matcher.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/CMakeFiles/src.dir/matcher.cpp.o"
	cd /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/matcher.cpp.o -c /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/src/matcher.cpp

src/CMakeFiles/src.dir/matcher.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/matcher.cpp.i"
	cd /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/src/matcher.cpp > CMakeFiles/src.dir/matcher.cpp.i

src/CMakeFiles/src.dir/matcher.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/matcher.cpp.s"
	cd /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/src/matcher.cpp -o CMakeFiles/src.dir/matcher.cpp.s

src/CMakeFiles/src.dir/utils.cpp.o: src/CMakeFiles/src.dir/flags.make
src/CMakeFiles/src.dir/utils.cpp.o: ../src/utils.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object src/CMakeFiles/src.dir/utils.cpp.o"
	cd /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/src.dir/utils.cpp.o -c /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/src/utils.cpp

src/CMakeFiles/src.dir/utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/src.dir/utils.cpp.i"
	cd /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/src/utils.cpp > CMakeFiles/src.dir/utils.cpp.i

src/CMakeFiles/src.dir/utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/src.dir/utils.cpp.s"
	cd /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/src/utils.cpp -o CMakeFiles/src.dir/utils.cpp.s

# Object files for target src
src_OBJECTS = \
"CMakeFiles/src.dir/checker.cpp.o" \
"CMakeFiles/src.dir/filter.cpp.o" \
"CMakeFiles/src.dir/graph.cpp.o" \
"CMakeFiles/src.dir/kdtree.cpp.o" \
"CMakeFiles/src.dir/log.cpp.o" \
"CMakeFiles/src.dir/matcher.cpp.o" \
"CMakeFiles/src.dir/utils.cpp.o"

# External object files for target src
src_EXTERNAL_OBJECTS =

src/libsrc.a: src/CMakeFiles/src.dir/checker.cpp.o
src/libsrc.a: src/CMakeFiles/src.dir/filter.cpp.o
src/libsrc.a: src/CMakeFiles/src.dir/graph.cpp.o
src/libsrc.a: src/CMakeFiles/src.dir/kdtree.cpp.o
src/libsrc.a: src/CMakeFiles/src.dir/log.cpp.o
src/libsrc.a: src/CMakeFiles/src.dir/matcher.cpp.o
src/libsrc.a: src/CMakeFiles/src.dir/utils.cpp.o
src/libsrc.a: src/CMakeFiles/src.dir/build.make
src/libsrc.a: src/CMakeFiles/src.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX static library libsrc.a"
	cd /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src && $(CMAKE_COMMAND) -P CMakeFiles/src.dir/cmake_clean_target.cmake
	cd /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/src.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/src.dir/build: src/libsrc.a

.PHONY : src/CMakeFiles/src.dir/build

src/CMakeFiles/src.dir/clean:
	cd /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src && $(CMAKE_COMMAND) -P CMakeFiles/src.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/src.dir/clean

src/CMakeFiles/src.dir/depend:
	cd /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/src /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src /mnt/nas/home/cilab/Caradryan/CBLab/CBData/learning_from_data/routing/roadnet-match/build/src/CMakeFiles/src.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/src.dir/depend
