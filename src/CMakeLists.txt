find_package(OpenCV  REQUIRED)


add_executable(hello main.cpp)
add_executable(adjust adjustfindhsv.cpp)
add_executable(savecap savepicfromcap.cpp)
add_executable(Shi_Tomasi Shi_Tomasi.cpp )

target_link_libraries(adjust ${OpenCV_LIBS})
target_link_libraries(hello  ${OpenCV_LIBS})
target_link_libraries(savecap  ${OpenCV_LIBS})
target_link_libraries(Shi_Tomasi ${OpenCV_LIBS})
