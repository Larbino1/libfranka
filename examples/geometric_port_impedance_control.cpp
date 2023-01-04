#include <array>
#include <cmath>
#include <functional>
#include <iostream>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "myLib.h"

#include "examples_common.h"

int main(int argc, char** argv) {
  // Check whether the required arguments were passed
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  // Geometric parameters
  const auto frame = franka::Frame::kEndEffector;
  const Eigen::Vector3d rcm_offset({0.377, 0.0, 0.042});
  const Eigen::Vector3d u1{0.0, 1.0, 0.0};
  const Eigen::Vector3d u2{0.0, 0.0, 1.0};

  // Compliance parameters
  const double stiffness{1000.0};
  const double damping{20.0};
  DiagonalSpringDamper<2,7> port_impedance{Eigen::Array2d::Constant(stiffness),
                                           Eigen::Array2d::Constant(damping)};

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
    robot.setCollisionBehavior({{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                               {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                               {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                               {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}});

    // define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState& robot_state,
                                         franka::Duration /*duration*/) -> franka::Torques {
      // get state variables
      ImpedanceCoordArgs iargs(robot_state, model, frame);
      // Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());

      auto port_coord = computePortCoord(iargs, port);

      // convert to Eigen
      // Check error not too large
      if (port_coord.z.norm() > 0.05) {
        throw std::runtime_error("Aborting; too far away from starting pose!");
      }

      // Log
      logTorques(model, robot_state);
      logPortError(port_coord);

      // compute control
      Eigen::VectorXd tau_d(7);
      tau_d << port_impedance.tau(port_coord);

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
    robot.control(impedance_control_callback, true, 1000);

  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  return 0;
}
