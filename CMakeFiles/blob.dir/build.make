# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/matteom/fuerte_workspace/tutorialROSOpenCV

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/matteom/fuerte_workspace/tutorialROSOpenCV

# Include any dependencies generated for this target.
include CMakeFiles/blob.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/blob.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/blob.dir/flags.make

CMakeFiles/blob.dir/src/cvblobs8.3_linux/blob.o: CMakeFiles/blob.dir/flags.make
CMakeFiles/blob.dir/src/cvblobs8.3_linux/blob.o: src/cvblobs8.3_linux/blob.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/matteom/fuerte_workspace/tutorialROSOpenCV/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/blob.dir/src/cvblobs8.3_linux/blob.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/blob.dir/src/cvblobs8.3_linux/blob.o -c /home/matteom/fuerte_workspace/tutorialROSOpenCV/src/cvblobs8.3_linux/blob.cpp

CMakeFiles/blob.dir/src/cvblobs8.3_linux/blob.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/blob.dir/src/cvblobs8.3_linux/blob.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/matteom/fuerte_workspace/tutorialROSOpenCV/src/cvblobs8.3_linux/blob.cpp > CMakeFiles/blob.dir/src/cvblobs8.3_linux/blob.i

CMakeFiles/blob.dir/src/cvblobs8.3_linux/blob.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/blob.dir/src/cvblobs8.3_linux/blob.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/matteom/fuerte_workspace/tutorialROSOpenCV/src/cvblobs8.3_linux/blob.cpp -o CMakeFiles/blob.dir/src/cvblobs8.3_linux/blob.s

CMakeFiles/blob.dir/src/cvblobs8.3_linux/blob.o.requires:
.PHONY : CMakeFiles/blob.dir/src/cvblobs8.3_linux/blob.o.requires

CMakeFiles/blob.dir/src/cvblobs8.3_linux/blob.o.provides: CMakeFiles/blob.dir/src/cvblobs8.3_linux/blob.o.requires
	$(MAKE) -f CMakeFiles/blob.dir/build.make CMakeFiles/blob.dir/src/cvblobs8.3_linux/blob.o.provides.build
.PHONY : CMakeFiles/blob.dir/src/cvblobs8.3_linux/blob.o.provides

CMakeFiles/blob.dir/src/cvblobs8.3_linux/blob.o.provides.build: CMakeFiles/blob.dir/src/cvblobs8.3_linux/blob.o
.PHONY : CMakeFiles/blob.dir/src/cvblobs8.3_linux/blob.o.provides.build

CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobContour.o: CMakeFiles/blob.dir/flags.make
CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobContour.o: src/cvblobs8.3_linux/BlobContour.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/matteom/fuerte_workspace/tutorialROSOpenCV/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobContour.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobContour.o -c /home/matteom/fuerte_workspace/tutorialROSOpenCV/src/cvblobs8.3_linux/BlobContour.cpp

CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobContour.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobContour.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/matteom/fuerte_workspace/tutorialROSOpenCV/src/cvblobs8.3_linux/BlobContour.cpp > CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobContour.i

CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobContour.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobContour.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/matteom/fuerte_workspace/tutorialROSOpenCV/src/cvblobs8.3_linux/BlobContour.cpp -o CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobContour.s

CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobContour.o.requires:
.PHONY : CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobContour.o.requires

CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobContour.o.provides: CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobContour.o.requires
	$(MAKE) -f CMakeFiles/blob.dir/build.make CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobContour.o.provides.build
.PHONY : CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobContour.o.provides

CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobContour.o.provides.build: CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobContour.o
.PHONY : CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobContour.o.provides.build

CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobOperators.o: CMakeFiles/blob.dir/flags.make
CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobOperators.o: src/cvblobs8.3_linux/BlobOperators.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/matteom/fuerte_workspace/tutorialROSOpenCV/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobOperators.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobOperators.o -c /home/matteom/fuerte_workspace/tutorialROSOpenCV/src/cvblobs8.3_linux/BlobOperators.cpp

CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobOperators.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobOperators.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/matteom/fuerte_workspace/tutorialROSOpenCV/src/cvblobs8.3_linux/BlobOperators.cpp > CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobOperators.i

CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobOperators.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobOperators.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/matteom/fuerte_workspace/tutorialROSOpenCV/src/cvblobs8.3_linux/BlobOperators.cpp -o CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobOperators.s

CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobOperators.o.requires:
.PHONY : CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobOperators.o.requires

CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobOperators.o.provides: CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobOperators.o.requires
	$(MAKE) -f CMakeFiles/blob.dir/build.make CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobOperators.o.provides.build
.PHONY : CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobOperators.o.provides

CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobOperators.o.provides.build: CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobOperators.o
.PHONY : CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobOperators.o.provides.build

CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobProperties.o: CMakeFiles/blob.dir/flags.make
CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobProperties.o: src/cvblobs8.3_linux/BlobProperties.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/matteom/fuerte_workspace/tutorialROSOpenCV/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobProperties.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobProperties.o -c /home/matteom/fuerte_workspace/tutorialROSOpenCV/src/cvblobs8.3_linux/BlobProperties.cpp

CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobProperties.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobProperties.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/matteom/fuerte_workspace/tutorialROSOpenCV/src/cvblobs8.3_linux/BlobProperties.cpp > CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobProperties.i

CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobProperties.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobProperties.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/matteom/fuerte_workspace/tutorialROSOpenCV/src/cvblobs8.3_linux/BlobProperties.cpp -o CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobProperties.s

CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobProperties.o.requires:
.PHONY : CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobProperties.o.requires

CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobProperties.o.provides: CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobProperties.o.requires
	$(MAKE) -f CMakeFiles/blob.dir/build.make CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobProperties.o.provides.build
.PHONY : CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobProperties.o.provides

CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobProperties.o.provides.build: CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobProperties.o
.PHONY : CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobProperties.o.provides.build

CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobResult.o: CMakeFiles/blob.dir/flags.make
CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobResult.o: src/cvblobs8.3_linux/BlobResult.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/matteom/fuerte_workspace/tutorialROSOpenCV/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobResult.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobResult.o -c /home/matteom/fuerte_workspace/tutorialROSOpenCV/src/cvblobs8.3_linux/BlobResult.cpp

CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobResult.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobResult.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/matteom/fuerte_workspace/tutorialROSOpenCV/src/cvblobs8.3_linux/BlobResult.cpp > CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobResult.i

CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobResult.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobResult.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/matteom/fuerte_workspace/tutorialROSOpenCV/src/cvblobs8.3_linux/BlobResult.cpp -o CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobResult.s

CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobResult.o.requires:
.PHONY : CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobResult.o.requires

CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobResult.o.provides: CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobResult.o.requires
	$(MAKE) -f CMakeFiles/blob.dir/build.make CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobResult.o.provides.build
.PHONY : CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobResult.o.provides

CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobResult.o.provides.build: CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobResult.o
.PHONY : CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobResult.o.provides.build

CMakeFiles/blob.dir/src/cvblobs8.3_linux/ComponentLabeling.o: CMakeFiles/blob.dir/flags.make
CMakeFiles/blob.dir/src/cvblobs8.3_linux/ComponentLabeling.o: src/cvblobs8.3_linux/ComponentLabeling.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/matteom/fuerte_workspace/tutorialROSOpenCV/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/blob.dir/src/cvblobs8.3_linux/ComponentLabeling.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/blob.dir/src/cvblobs8.3_linux/ComponentLabeling.o -c /home/matteom/fuerte_workspace/tutorialROSOpenCV/src/cvblobs8.3_linux/ComponentLabeling.cpp

CMakeFiles/blob.dir/src/cvblobs8.3_linux/ComponentLabeling.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/blob.dir/src/cvblobs8.3_linux/ComponentLabeling.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/matteom/fuerte_workspace/tutorialROSOpenCV/src/cvblobs8.3_linux/ComponentLabeling.cpp > CMakeFiles/blob.dir/src/cvblobs8.3_linux/ComponentLabeling.i

CMakeFiles/blob.dir/src/cvblobs8.3_linux/ComponentLabeling.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/blob.dir/src/cvblobs8.3_linux/ComponentLabeling.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/matteom/fuerte_workspace/tutorialROSOpenCV/src/cvblobs8.3_linux/ComponentLabeling.cpp -o CMakeFiles/blob.dir/src/cvblobs8.3_linux/ComponentLabeling.s

CMakeFiles/blob.dir/src/cvblobs8.3_linux/ComponentLabeling.o.requires:
.PHONY : CMakeFiles/blob.dir/src/cvblobs8.3_linux/ComponentLabeling.o.requires

CMakeFiles/blob.dir/src/cvblobs8.3_linux/ComponentLabeling.o.provides: CMakeFiles/blob.dir/src/cvblobs8.3_linux/ComponentLabeling.o.requires
	$(MAKE) -f CMakeFiles/blob.dir/build.make CMakeFiles/blob.dir/src/cvblobs8.3_linux/ComponentLabeling.o.provides.build
.PHONY : CMakeFiles/blob.dir/src/cvblobs8.3_linux/ComponentLabeling.o.provides

CMakeFiles/blob.dir/src/cvblobs8.3_linux/ComponentLabeling.o.provides.build: CMakeFiles/blob.dir/src/cvblobs8.3_linux/ComponentLabeling.o
.PHONY : CMakeFiles/blob.dir/src/cvblobs8.3_linux/ComponentLabeling.o.provides.build

# Object files for target blob
blob_OBJECTS = \
"CMakeFiles/blob.dir/src/cvblobs8.3_linux/blob.o" \
"CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobContour.o" \
"CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobOperators.o" \
"CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobProperties.o" \
"CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobResult.o" \
"CMakeFiles/blob.dir/src/cvblobs8.3_linux/ComponentLabeling.o"

# External object files for target blob
blob_EXTERNAL_OBJECTS =

lib/libblob.so: CMakeFiles/blob.dir/src/cvblobs8.3_linux/blob.o
lib/libblob.so: CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobContour.o
lib/libblob.so: CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobOperators.o
lib/libblob.so: CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobProperties.o
lib/libblob.so: CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobResult.o
lib/libblob.so: CMakeFiles/blob.dir/src/cvblobs8.3_linux/ComponentLabeling.o
lib/libblob.so: CMakeFiles/blob.dir/build.make
lib/libblob.so: CMakeFiles/blob.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library lib/libblob.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/blob.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/blob.dir/build: lib/libblob.so
.PHONY : CMakeFiles/blob.dir/build

CMakeFiles/blob.dir/requires: CMakeFiles/blob.dir/src/cvblobs8.3_linux/blob.o.requires
CMakeFiles/blob.dir/requires: CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobContour.o.requires
CMakeFiles/blob.dir/requires: CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobOperators.o.requires
CMakeFiles/blob.dir/requires: CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobProperties.o.requires
CMakeFiles/blob.dir/requires: CMakeFiles/blob.dir/src/cvblobs8.3_linux/BlobResult.o.requires
CMakeFiles/blob.dir/requires: CMakeFiles/blob.dir/src/cvblobs8.3_linux/ComponentLabeling.o.requires
.PHONY : CMakeFiles/blob.dir/requires

CMakeFiles/blob.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/blob.dir/cmake_clean.cmake
.PHONY : CMakeFiles/blob.dir/clean

CMakeFiles/blob.dir/depend:
	cd /home/matteom/fuerte_workspace/tutorialROSOpenCV && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/matteom/fuerte_workspace/tutorialROSOpenCV /home/matteom/fuerte_workspace/tutorialROSOpenCV /home/matteom/fuerte_workspace/tutorialROSOpenCV /home/matteom/fuerte_workspace/tutorialROSOpenCV /home/matteom/fuerte_workspace/tutorialROSOpenCV/CMakeFiles/blob.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/blob.dir/depend
