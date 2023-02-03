#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <string>

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
  const Eigen::Vector3d ee_offset({0.377, 0.0, 0.042});

  // ee (for debug)
  WorldCoord ee;
  ee.offset = ee_offset;
  ee.ref = Eigen::Vector3d::Zero();

  // Dynamic extension
  Eigen::AngleAxisd pitchangle(3.1415926535/2, Eigen::Vector3d::UnitY());
  Eigen::Matrix4d slider_A;
  slider_A.setIdentity();
  slider_A.block<3,3>(0,0) = Eigen::Quaternion<double>(pitchangle).matrix();
  slider_A.block<3,1>(0,3) = ee_offset;
  VirtualPrismaticJoint slider(frame, Eigen::Affine3d(slider_A), 0.5, 0.0);
  
  ImpedanceCoordResult<1, 1> slider_extension;
  slider_extension.J(0, 0) = 1;

  // Compliance parameters
  const double stiffness{2000.0};
  const double damping{20.0};
  Eigen::Array3d stiffness_vec(stiffness, 0.0, stiffness);
  DiagonalSpringDamper<3,8> impedance{stiffness_vec, Eigen::Array3d::Constant(damping)};
  PiecewiseSpring<2> insertion_buffer({-0.280, 0.0}, {0.0, 0.0}, 300., 300.);

  // Constrain slider in Y direction between 0.1 and -0.1
  const Eigen::Vector3d Y(0.0, 1.0, 0.0);
  PiecewiseSpring<2> slot_buffer({-0.1, 0.1}, {0.0, 0.0}, stiffness, stiffness);

  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();
    ImpedanceCoordArgs initial_iargs(initial_state, model, frame);

    Eigen::Vector3d rcm(slider.computeCoord(initial_iargs).z);

    // DEBUG
    //auto initial_port_coord_ext = slider.computeCoord(initial_iargs);
    //auto ee_coord = computeWorldCoord(initial_iargs, ee);

    // set collision behavior
    const double max_f{30.0};
    const double max_t{30.0};
    robot.setCollisionBehavior(
        {{max_t, max_t, max_t, max_t, max_t, max_t, max_t}}, {{max_t, max_t, max_t, max_t, max_t, max_t, max_t}},
        {{max_f, max_f, max_f, max_f, max_f, max_f}}, {{max_f, max_f, max_f, max_f, max_f, max_f}});

    // define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState& robot_state,
                                         franka::Duration duration) -> franka::Torques {
      // get state variables
      ImpedanceCoordArgs iargs(robot_state, model, frame);

      // Compute error between slider position and rcm
      auto port_coord_ext = slider.computeCoord(iargs);
      port_coord_ext.z = port_coord_ext.z - rcm;
      // Check error not too large
      if (port_coord_ext.z.norm() > 0.15) {
        throw std::runtime_error("Aborting; too far away from starting pose!");
      }

      // Log
      //logTorques(model, robot_state); 
      //logPortError(port_coord);

      // Update extension
      slider_extension.z(0) = slider.q;
      slider_extension.dz(0) = slider.qdot;
      
      Eigen::Vector3d F = impedance.F(port_coord_ext) + Y*slot_buffer.F(port_coord_ext.z[1]);
      Eigen::Matrix<double, 8, 1> tau = port_coord_ext.J.transpose() * F;
      double tau_slider = tau.tail(1)(0);
      slider.update(tau_slider, duration.toSec()); // + insertion_buffer.F(slider_extension);

      // Check force not too large
      if (F.norm() > 30) {
        throw std::runtime_error("Aborting; demanded force too large!");
      }

      // compute control
      Eigen::VectorXd tau_d(7);
      tau_d << tau.head(7);
      // tau_d.setZero();

      // convert to double array
      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d; 
      return tau_d_array;
    };

    // start real-time control loop
    std::cout << "WARNING: Make sure you have the user stop at hand! Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(impedance_control_callback, true, 1000);

  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  return 0;
}
