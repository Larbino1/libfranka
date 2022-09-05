find_package(SDL2 REQUIRED)
find_package(Eigen3 REQUIRED)

#add_executable(JoyInput joyInput.cpp) # Demo with main
add_library(JoyInputLib joyInput.cpp)

#target_link_libraries(JoyInput ${SDL2_LIBRARIES} Eigen3::Eigen3)
target_link_libraries(JoyInputLib ${SDL2_LIBRARIES} Eigen3::Eigen3)

target_include_directories(JoyInputLib
          INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
          )
