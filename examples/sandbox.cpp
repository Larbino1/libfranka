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
 
  /* Piecewise spring example
  PiecewiseSpring<2, 1> spring({0.0, 1.0}, {0.0, 1.0}, 2.0, 2.0);

  ImpedanceCoordResult<1, 1> ir;
  ir.z(0) = 0.0;
  ir.dz(0) = 0.0;
  ir.J(0,0) = 1.0;

  ir.z(0) = -1;
  std::cout << ir.z << " -> " << spring.F(ir) << std::endl;

  ir.z(0) = 0.0;
  std::cout << ir.z << " -> " << spring.F(ir) << std::endl;

  ir.z(0) = .5;
  std::cout << ir.z << " -> " << spring.F(ir) << std::endl;

  ir.z(0) = 1.0;
  std::cout << ir.z << " -> " << spring.F(ir) << std::endl;

  ir.z(0) = 2.0;
  std::cout << ir.z << " -> " << spring.F(ir) << std::endl;
  */

  /*
  // Geometric parameters
  const Eigen::Vector3d ee_offset({0.377, 0.0, 0.042});
  const auto frame = franka::Frame::kEndEffector;

  // Compliance parameters
  const double translational_stiffness{500.0};
  Eigen::MatrixXd stiffness(3, 3), damping(3, 3);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                     Eigen::MatrixXd::Identity(3, 3);
  */


  /*
  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();
    // Register point
    Eigen::Vector3d position_d = register_point(robot, ee_offset);


    // define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState& robot_state,
                                         franka::Duration duration) -> franka::Torques {

      // compute control
      Eigen::VectorXd tau_d(7);
      tau_d.fill(0);

      // convert to double array
      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
      return tau_d_array;
    };

    // start real-time control loop
    std::cout << "WARNING: Collision thresholds are set to high values. "
              << "Make sure you have the user stop at hand!" << std::endl
              << "After starting try to push the robot and see how it reacts." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(impedance_control_callback);
  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }
  */
  return 0;
}
