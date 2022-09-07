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
  const Eigen::Vector3d ee_offset({0.377, 0.0, z_offset});
  const auto frame = franka::Frame::kEndEffector;
  const Eigen::Vector3d rcm_offset({0.2, 0.0, z_offset});
  const Eigen::Vector3d u1{0.0, 1.0, 0.0};
  const Eigen::Vector3d u2{0.0, 0.0, 1.0};

  // Compliance parameters
  const double stiffness{1200.0};
  const double damping{10.0};
  DiagonalSpringDamper<3,7> ee_impedance{Eigen::Array3d::Constant(stiffness),
                                         Eigen::Array3d::Constant(damping)};
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
    RefState ref(initial_transform * ee_offset);
    int loopCounter = 0;

    // setup ee
    WorldCoord ee;
    ee.offset = ee_offset;
    ee.ref = ref.pos;

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
        ImpedanceCoordArgs iargs(robot_state, model, frame);

        loopCounter = (loopCounter + 1) % 20;
        if (loopCounter == 0) {
          poll_SDL_events();
          ref.vel.topRows(2) << right_stick.read();
          double d = triggers.read();
          ref.vel(2) = d;  // third row
        }
        ref.update(duration.toSec());
        ee.ref = ref.pos;

        // compute control coordinate error and jacobian
        auto ee_coord = computeWorldCoord(iargs, ee);
        auto port_coord = computePortCoord(iargs, port);

        // Check error not too large
        if (ee_coord.z.norm() > 0.05 || port_coord.z.norm() > 0.05) {
          throw std::runtime_error("Aborting; too far away from starting pose!");
        }

        // compute control
        Eigen::VectorXd tau_d(7);
        tau_d << ee_impedance.F(ee_coord) + port_impedance.F(port_coord);

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
