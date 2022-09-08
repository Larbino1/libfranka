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

  // Dynamic extension
  Eigen::AngleAxisd pitchangle(3.1415926535/2, Eigen::Vector3d::UnitX());
  Eigen::Matrix4d slider_A;
  slider_A.setIdentity();
  slider_A.block<3,3>(0,0) = Eigen::Quaternion<double>(pitchangle).matrix();
  slider_A.block<3,1>(0,3) = rcm_offset;
  VirtualPrismaticJoint slider(frame, Eigen::Affine3d(slider_A), 1.0, 5.0);

  // Compliance parameters
  const double stiffness{1000.0};
  const double damping{20.0};
  DiagonalSpringDamper<3,7> port_impedance{Eigen::Array2d::Constant(stiffness),
                                           Eigen::Array2d::Constant(damping)};
                                           // TODO shouldnt need separate impedances for port and slider
  DiagonalSpringDamper<3, 1> slider_impedance{Eigen::Array2d::Constant(stiffness),
                                              Eigen::Array2d::Constant(damping)};
  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    franka::RobotState initial_state = robot.readOnce();

    // // equilibrium point is the initial position
    // Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    ImpedanceCoordArgs initial_iargs(initial_state, model, frame);
    Eigen::Vector3d rcm(slider.computeCoord(initial_iargs).z);
    // PortCoord port;
    // port.rcm = rcm;
    // port.u1 = u1;
    // port.u2 = u2;
    // port.offset = rcm_offset;


    // set collision behavior
    robot.setCollisionBehavior({{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                               {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                               {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                               {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}});

    // define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState& robot_state,
                                         franka::Duration duration) -> franka::Torques {
      // get state variables
      ImpedanceCoordArgs iargs(robot_state, model, frame);
      // Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());

      auto port_coord_ext = slider.computeCoord(iargs);
      port_coord_ext.z -= rcm;

      ImpedanceCoordResult<3, 7> port_coord;
      port_coord.z = port_coord_ext.z;
      port_coord.dz = port_coord_ext.dz;
      port_coord.J = port_coord_ext.J.leftCols(7);

      ImpedanceCoordResult<3, 1> slider_coord;
      slider_coord.z = port_coord_ext.z;
      slider_coord.dz = port_coord_ext.dz;
      slider_coord.J = port_coord_ext.J.rightCols(1);

      // Check error not too large
      if (port_coord.z.norm() > 0.05) {
        throw std::runtime_error("Aborting; too far away from starting pose!");
      }

      // Update extension
      slider.update(slider_impedance.F(slider_coord)(0, 0), duration.toSec());

      // compute control
      Eigen::VectorXd tau_d(7);
      tau_d << port_impedance.F(port_coord);
      tau_d.setZero();

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

  return 0;
}
