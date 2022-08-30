#find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)
add_library(MyLib myLib.cpp)
target_link_libraries(MyLib PUBLIC Franka::Franka Eigen3::Eigen3)
target_include_directories(MyLib
          INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
          )
