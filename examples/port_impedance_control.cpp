#include <array>
#include <cmath>
#include <functional>
#include <iostream>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "examples_common.h"

int main(int argc, char** argv) {
  // Check whether the required arguments were passed
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  // Geometric parameters
  const auto frame = franka::Frame::kEndEffector;
  const Eigen::Vector3d rcm_offset({0.422, 0.0, 0.042});
  const Eigen::Vector3d u1{0.0, 1.0, 0.0};
  const Eigen::Vector3d u2{0.0, 0.0, 1.0};

  // Compliance parameters
  const double translational_stiffness{600.0};
  Eigen::MatrixXd stiffness(2, 2), damping(2, 2);
  stiffness.setZero();
  stiffness.topLeftCorner(2, 2) << translational_stiffness * Eigen::MatrixXd::Identity(2, 2);
  damping.setZero();
  damping.topLeftCorner(2, 2) << 2.0 * sqrt(translational_stiffness) *
                                     Eigen::MatrixXd::Identity(2, 2);

  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    franka::RobotState initial_state = robot.readOnce();

    // equilibrium point is the initial position
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d rcm(initial_transform * rcm_offset);
    PortCoord port;
    port.rcm = rcm;
    port.u1 = u1;
    port.u2 = u2;
    port.offset = rcm_offset;

    // set collision behavior
/*    robot.setCollisionBehavior({{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                               {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                               {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                               {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});*/
    robot.setCollisionBehavior({{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                               {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                               {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                               {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}});

    // define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState& robot_state,
                                         franka::Duration /*duration*/) -> franka::Torques {
      // get state variables
      Eigen::Affine3d current_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> geometric_jacobian(model.zeroJacobian(frame, robot_state).data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

      auto coord_result = computePortCoord(current_transform, geometric_jacobian, port);
      auto error = coord_result.error;
      auto jacobian = coord_result.jacobian;

/*
      std::cout << current_transform.affine() << std::endl << std::endl;
      std::cout << geometric_jacobian << std::endl << std::endl;
      std::cout << port.rcm << std::endl << std::endl;
      std::cout << port.u1 << std::endl << std::endl;
      std::cout << port.u2 << std::endl << std::endl;
      std::cout << port.offset << std::endl << std::endl;
      std::cout << "result" <<std::endl;
     */ 
     // std::cout << error << std::endl << std::endl;
      std::cout << 1000*jacobian * dq << std::endl << std::endl;
/*      std::cout << jacobian << std::endl << std::endl;
*/
      // convert to Eigen
      // Check error not too large
      if (error.norm() > 0.05) {
        throw std::runtime_error("Aborting; too far away from starting pose!");
      }

      // compute control
      Eigen::VectorXd tau_d(7);
      tau_d << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
      //tau_d.setZero();

      // convert to double array
      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d; 
      return tau_d_array;
    };

    /* TESTING
    port.rcm << 1, 2, 3;
    port.offset = rcm_offset;
    Eigen::Matrix4d T; T << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    Eigen::Affine3d current_transform(T);
    Eigen::Matrix<double, 6, 7> geometric_jacobian;
    geometric_jacobian.setOnes();
    auto coord_result = computePortCoord(current_transform, geometric_jacobian, port);
    auto error = coord_result.error;
    auto jacobian = coord_result.jacobian;


    std::cout << current_transform.affine() << std::endl << std::endl;
    std::cout << geometric_jacobian << std::endl << std::endl;
    std::cout << port.rcm << std::endl << std::endl;
    std::cout << port.u1 << std::endl << std::endl;
    std::cout << port.u2 << std::endl << std::endl;
    std::cout << "result" <<std::endl;
    std::cout << error << std::endl << std::endl;
    std::cout << jacobian << std::endl << std::endl;
    */

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

  return 0;
}
