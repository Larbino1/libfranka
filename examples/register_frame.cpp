#include <array>
#include <cmath>

#include <functional>
#include <future>
#include <iostream>
#include <string>
#include <sstream>
#include <thread>
#include <regex>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "examples_common.h"
#include "myLib.h"
#include "myJson.h"

int main(int argc, char** argv) {
  // Check whether the required arguments were passed
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
  }
 
  RegistrationPoints registration = parse_registration_points(open_json("data/femoralhead.json"));
  int NPoints = registration.points.cols();

  Eigen::MatrixXd trajectory = parse_trajectory(open_json("data/trajectory.json"));

  std::cout << "Loaded trajectory with " << trajectory.cols() << " points, starting at:" << std::endl << trajectory.col(1) << std::endl;  

  // pointsA << 0.136884, 0.164006, 0.106359, 0.098661, 0.144228,
  //            0.07999,  0.10039,  0.086017, 0.129846, 0.108101,
  //            0.072654, 0.140645, 0.134369, 0.119706, 0.158389;
  // pointsB << 0.482148,  0.46106,  0.470435,     0.424454,  0.453253,
  //            0.0261249, 0.066902, 1.95468e-05, -0.0133286, 0.0352025,
  //            0.0850531, 0.148871, 0.145727,     0.137125,  0.174718;

  // Geometric parameters
  const Eigen::Vector3d ee_offset({0.377, 0.0, 0.042});
  const auto frame = franka::Frame::kEndEffector;

  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();
    
    Eigen::MatrixXd registered_points(3, NPoints);  
    std::cout << "About to iterate through " << NPoints << " points" << std::endl;
    for (int i = 0; i < NPoints; i++) {
      std::cout << "Please register point " << registration.names[i] << " with coordinates: " << std::endl << registration.points.col(i) << std::endl;
      // Register point
      Eigen::Vector3d p_B = register_point_prompt(robot, ee_offset);
      registered_points.col(i) << p_B;
    }

    std::cout << "registration points, from json:" << std::endl << registration.points << std::endl;
    std::cout << "registered points, from robot:" << std::endl << registered_points << std::endl;

    PointMatchResult pointMatchResult = matchPointSets(registration.points, registered_points);
    std::cout << "Success?:" << pointMatchResult.success << std::endl;
    std::cout << "Rotation:" << std::endl << pointMatchResult.rotation << std::endl;
    std::cout << "Translation:" << std::endl << pointMatchResult.translation << std::endl;
    std::cout << "Mean distance:" << pointMatchResult.avg_error * 1000 << "mm." << std::endl;
    std::cout << "Max distance:" << pointMatchResult.max_error * 1000 << "mm." << std::endl;

    // Ask user to move to start point of trajectory

    // Press enter to start following trajectory

    // Repeat


    std::cout << "Done." << std::endl;
  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  return 0;
}
