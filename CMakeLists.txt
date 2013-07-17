cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
#pkg_check_modules(OpenCV REQUIRED opencv)
#pkg_check_modules(gtk+ REQUIRED gtk+-2.0)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


set(PATH_SRC ${PROJECT_SOURCE_DIR}/src)
set(PATH_CVBLOBS ${PATH_SRC}/cvblobs8.3_linux)
set(PATH_INC ${PROJECT_SOURCE_DIR}/include)
set(INCLUDE_PATHS ${PATH_INC} ${PATH_CVBLOBS})
link_directories(${LIBRARY_OUTPUT_PATH})

#set filename holders
set(SRC_CVBLOBS ${PATH_CVBLOBS}/blob.cpp
		${PATH_CVBLOBS}/BlobContour.cpp
		${PATH_CVBLOBS}/BlobOperators.cpp
		${PATH_CVBLOBS}/BlobProperties.cpp
		${PATH_CVBLOBS}/BlobResult.cpp
		${PATH_CVBLOBS}/ComponentLabeling.cpp)

include_directories (include)
include_directories (${INCLUDE_PATHS})
#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(blob ${SRC_CVBLOBS})
#rosbuild_add_executable(main src/main.cpp)
#rosbuild_add_executable(main src/map.cpp)
#rosbuild_add_executable(pub  src/image_pub.cpp)

#rosbuild_add_executable(pub_labrob14  src/image_pub_labrob14.cpp)
#rosbuild_add_executable(pub_labrob14 src/map.cpp)



rosbuild_add_executable(cam1_labrob14_pub  src/v2nodes/cam1_labrob14_pub.cpp)
rosbuild_add_executable(cam1_labrob14_pub src/v2nodes/map_1280_1024.cpp)

rosbuild_add_executable(cam2_labrob14_pub  src/v2nodes/cam2_labrob14_pub.cpp)
rosbuild_add_executable(cam2_labrob14_pub src/v2nodes/map_640_480.cpp)


#rosbuild_add_executable(map src/v2nodes/map_640_480.cpp)

rosbuild_add_executable(list_fusioner src/list_fusioner.cpp)
#target_link_libraries(list_fusioner  ncurses)

#target_link_libraries(main blob)
target_link_libraries(cam1_labrob14_pub  blob)
target_link_libraries(cam2_labrob14_pub  blob)



#target_link_libraries(pub_sciame  blob)


#rosbuild_add_library(${PROJECT_NAME} src/compressed_publisher.h)
