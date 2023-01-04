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
  const Eigen::Vector3d ee_offset({0.377, 0.0, 0.042});
  const auto frame = franka::Frame::kEndEffector;

  // Compliance parameters
  const double stiffness{2000.0};
  const double damping{20.0};
  DiagonalSpringDamper<3,7> ee_impedance{Eigen::Array3d::Constant(stiffness),
                                         Eigen::Array3d::Constant(damping)};

  const double joint_stiffness{0.0};
  const double joint_damping{0.5};
  DiagonalSpringDamper<7,7> joint_impedance{Eigen::Array<double,7,1>::Constant(joint_stiffness),
                                            Eigen::Array<double,7,1>::Constant(joint_damping)};
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

    const double max_f{50.0};
    const double max_t{30.0};
    // set collision behavior
    robot.setCollisionBehavior(
        {{max_t, max_t, max_t, max_t, max_t, max_t, max_t}}, {{max_t, max_t, max_t, max_t, max_t, max_t, max_t}},
        {{max_f, max_f, max_f, max_f, max_f, max_f}}, {{max_f, max_f, max_f, max_f, max_f, max_f}});

    with_controller([&](SDL_Joystick* controller) {
      // Define input functions
      StickReader right_stick(controller, 4, 3, AX_MAX, VMAX);
      TwoTriggerReader triggers(controller, 2, 5, AX_MAX, VMAX);

      // define callback for the torque control loop
      std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
          impedance_control_callback = [&](const franka::RobotState& robot_state,
                                           franka::Duration duration) -> franka::Torques {
        // Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
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
	      auto joint_coord = computeJointCoord(iargs);
        // Check error not too large
        if (ee_coord.z.norm() > 0.05) {
          throw std::runtime_error("Aborting; too far away from starting pose!");
        }

        // compute control
        Eigen::VectorXd tau_d(7);
        tau_d << ee_impedance.tau(ee_coord) + joint_impedance.tau(joint_coord); 

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
    });
  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  return 0;
}
