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



int main(int argc, char** argv) {
  DRef ref;
  while (true) { 
    std::chrono::milliseconds timeout(1);
    std::future<DRef> future = std::async(read_reference_input);
    if (future.wait_for(timeout) == std::future_status::ready) {
      ref = future.get();
      std::cout << ref.r << std::endl << ref.dr << std::endl;
    }
  }
  /*// Check whether the required arguments were passed
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  // Geometric parameters
  const Eigen::Vector3d ee_offset({0.422, 0.0, 0.042});
  const auto frame = franka::Frame::kEndEffector;

  // Compliance parameters
  const double translational_stiffness{500.0};
  Eigen::MatrixXd stiffness(3, 3), damping(3, 3);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                     Eigen::MatrixXd::Identity(3, 3);

  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    franka::RobotState initial_state = robot.readOnce();

    // equilibrium point is the initial position
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_d(initial_transform * ee_offset);
  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }
  */
  return 0;
}
