find_package(SDL2 REQUIRED)
find_package(Eigen3 REQUIRED)
add_executable(JoyInput joyInput.cpp)
target_link_libraries(JoyInput ${SDL2_LIBRARIES} Eigen3::Eigen3)
