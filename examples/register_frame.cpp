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
 
  json jsondata = open_points_json();
  json points = jsondata["points"];
  int NPoints = points.size();

  Eigen::MatrixXd pointsA(3, NPoints), pointsB(3, NPoints);

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
    
    std::cout << "About to iterate through " << NPoints << " points with json data: " << points.dump() << std::endl;
    int i = 0;
    for (auto it = points.begin(); it != points.end(); it++) {
      json p = it.value();
      Eigen::Vector3d p_A(p["x"], p["y"], p["z"]);
      pointsA.col(i) << p_A;

      std::cout << "Please register point " << p["name"] << " with coordinates: " << std::endl << p_A << std::endl;

      // Register point
      Eigen::Vector3d p_B = register_point_prompt(robot, ee_offset);
      pointsB.col(i) << p_B;
      i += 1;
    }

    std::cout << "pointset A, from json:" << std::endl << pointsA << std::endl;
    std::cout << "pointset B, from robot:" << std::endl << pointsB << std::endl;

    PointMatchResult pointMatchResult = matchPointSets(pointsA, pointsB);
    std::cout << "Success?:" << pointMatchResult.success << std::endl;
    std::cout << "Rotation:" << std::endl << pointMatchResult.rotation << std::endl;
    std::cout << "Translation:" << std::endl << pointMatchResult.translation << std::endl;
    std::cout << "Mean distance:" << pointMatchResult.avg_error * 1000 << "mm." << std::endl;
    std::cout << "Max distance:" << pointMatchResult.max_error * 1000 << "mm." << std::endl;
    std::cout << "Done." << std::endl;
  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  return 0;
}
