find_package(OpenCV  REQUIRED)

INCLUDE_DIRECTORIES(./includes)

aux_source_directory(. DIR_SRCS)
add_executable(box ${DIR_SRCS})

target_link_libraries(box  ${OpenCV_LIBS} pthread librealsense2.so)

