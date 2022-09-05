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

const double MAX_ERR = 0.04; // in m
const double MAX_DQ = 1.0; // in rad/s
const double VMAX = 0.05;  // In m/s

int main(int argc, char** argv) {
  std::cout << "Instrument impedance control demo" << std::endl;
  // Check whether the required arguments were passed
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  // Geometric parameters
  const double z_offset = 0.042;
  const Eigen::Vector3d ee_offset({0.422, 0.0, z_offset});
  const auto frame = franka::Frame::kEndEffector;
  const Eigen::Vector3d rcm_offset({0.2, 0.0, z_offset});
  const Eigen::Vector3d u1{0.0, 1.0, 0.0};
  const Eigen::Vector3d u2{0.0, 0.0, 1.0};

  // Compliance parameters
  const double stiffness{2000.0};
  const double damping{20.0};
  Eigen::MatrixXd ee_stiffness(3, 3), ee_damping(3, 3), port_stiffness(2, 2), port_damping(2, 2);
  ee_stiffness.setZero();
  ee_stiffness.topLeftCorner(3, 3) << stiffness * Eigen::MatrixXd::Identity(3, 3);
  ee_damping.setZero();
  ee_damping.topLeftCorner(3, 3) << damping * Eigen::MatrixXd::Identity(3, 3);
  port_stiffness.setZero();
  port_stiffness.topLeftCorner(2, 2) << stiffness * Eigen::MatrixXd::Identity(2, 2);
  port_damping.setZero();
  port_damping.topLeftCorner(2, 2) << damping * Eigen::MatrixXd::Identity(2, 2);

  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    franka::RobotState initial_state = robot.readOnce();

    // equilibrium point is the initial position
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    RefState ref(initial_transform * ee_offset);
    int loopCounter = 0;

    // setup port
    Eigen::Vector3d rcm(initial_transform * rcm_offset);
    PortCoord port;
    port.rcm = rcm;
    port.u1 = u1;
    port.u2 = u2;
    port.offset = rcm_offset;

    // set collision behavior
    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}});

    with_controller([&](SDL_Joystick* controller) {
      // Define input functions
      StickReader right_stick(controller, 4, 3, AX_MAX, VMAX);
      TwoTriggerReader triggers(controller, 2, 5, AX_MAX, VMAX);

      // define callback for the torque control loop
      std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
          impedance_control_callback = [&](const franka::RobotState& robot_state,
                                           franka::Duration duration) -> franka::Torques {
        Eigen::Affine3d current_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> geometric_jacobian(
            model.zeroJacobian(frame, robot_state).data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

        loopCounter = (loopCounter + 1) % 20;
        if (loopCounter == 0) {
          poll_SDL_events();
          ref.vel.topRows(2) << right_stick.read();
          double d = triggers.read();
          ref.vel(2) = d;  // third row
        }
        ref.update(duration.toSec());

        // compute control coordinate error and jacobian
        Eigen::Vector3d position(current_transform * ee_offset);
        Eigen::Matrix<double, 3, 1> ee_error(position - ref.pos);
        auto ee_jacobian = offset_jacobian(current_transform, geometric_jacobian, ee_offset);

        auto coord_result = computePortCoord(current_transform, geometric_jacobian, port);
        auto port_error = coord_result.error;
        auto port_jacobian = coord_result.jacobian;

        // Check error not too large
        if (ee_error.norm() > MAX_ERR || port_error.norm() > MAX_ERR) {
          throw std::runtime_error("Aborting; too far away from starting pose!");
        }
        if (dq.cwiseAbs().maxCoeff() > MAX_DQ) {
          throw std::runtime_error("Aborting; Joint velocity too high");
        }

        // compute control
        Eigen::VectorXd tau_d(7);
        tau_d << ee_jacobian.transpose() *
                         (-ee_stiffness * ee_error - ee_damping * (ee_jacobian * dq)) +
                     port_jacobian.transpose() *
                         (-port_stiffness * port_error - port_damping * (port_jacobian * dq));

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
    });
  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  return 0;
}
