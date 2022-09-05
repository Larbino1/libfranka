#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "joyInput.h"
#include "myLib.h"

#include "examples_common.h"

int main(int argc, char** argv) {
  std::cout << "Instrument impedance control demo" << std::endl;
  // Check whether the required arguments were passed
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
    DRef ref;
    std::future<DRef> future = std::async(read_reference_input);
    std::chrono::microseconds timeout(100);

    // set collision behavior
    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}});

  with_controller([&] (SDL_Joystick* controller) {
      
      // define callback for the torque control loop
      std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
          impedance_control_callback = [&](const franka::RobotState& robot_state,
                                           franka::Duration /*duration*/) -> franka::Torques {
        Eigen::Affine3d current_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> geometric_jacobian(
            model.zeroJacobian(frame, robot_state).data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

        // compute control coordinate error and jacobian
        Eigen::Vector3d position(current_transform * ee_offset);
        Eigen::Matrix<double, 3, 1> error(position - position_d);
        auto jacobian = offset_jacobian(current_transform, geometric_jacobian, ee_offset);

        // Check error not too large
        if (error.norm() > 0.05) {
          throw std::runtime_error("Aborting; too far away from starting pose!");
        }

        // compute control
        Eigen::VectorXd tau_d(7);
        tau_d << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));

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
  }
  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  return 0;
}
