find_package(SDL2 REQUIRED)
add_executable(JoyInput joyInput.cpp)
target_link_libraries(JoyInput ${SDL2_LIBRARIES})
