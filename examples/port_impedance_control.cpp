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

auto& all = Eigen::all;

Eigen::Matrix3d skew(Eigen::Vector3d vec) {
  Eigen::Matrix3d ret; ret << 0,     -vec[2], vec(1),
                              vec[2], 0,     -vec(0),
                             -vec(1), vec(0), 0;
  return ret;
}

Eigen::Matrix<double, 3, 7> unit_vec_jacobian(Eigen::Matrix3d R, Eigen::Matrix<double, 3, 7> Jw, Eigen::Vector3d u) {
  auto J_R1 = - skew(R(all, 0)) * Jw;
  auto J_R2 = - skew(R(all, 1)) * Jw;
  auto J_R3 = - skew(R(all, 2)) * Jw;
  return J_R1*u + J_R2*u + J_R3*u;
}

int main(int argc, char** argv) {
  // Check whether the required arguments were passed
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  // Geometric parameters
  const Eigen::Vector3d rcm_offset{0.0, 0.0, 0.2};
  const Eigen::Vector3d u1{1.0, 0.0, 0.0};
  const Eigen::Vector3d u2{0.0, 1.0, 0.0};

  // Compliance parameters
  const double translational_stiffness{150.0};
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
    Eigen::Vector3d position_d(initial_transform.translation() + initial_transform * rcm_offset);

    // set collision behavior
    robot.setCollisionBehavior({{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                               {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                               {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                               {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});

    // define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState& robot_state,
                                         franka::Duration /*duration*/) -> franka::Torques {
      // get state variables
      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

      // get rotation transform
      Eigen::Affine3d current_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
      Eigen::Matrix3d rotation_transform(initial_transform.rotation());

      // convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> geometricJacobian(jacobian_array.data());
      auto translational_jacobian = geometricJacobian.topRows<3>();
      auto rotational_jacobian = geometricJacobian.bottomRows<3>();
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.linear());

      // compute error to desired equilibrium pose
      // position error
      Eigen::Vector3d r; r << position - position_d;
      Eigen::Vector3d v1; v1 << rotation_transform * u1;
      Eigen::Vector3d v2; v2 << rotation_transform * u2;
      Eigen::Vector2d error; error << v1.transpose() * r,
                                      v2.transpose() * r;
      auto J_u1 = unit_vec_jacobian(rotation_transform, rotational_jacobian, u1);
      auto J_u2 = unit_vec_jacobian(rotation_transform, rotational_jacobian, u2);
      
      Eigen::Matrix<double, 2, 7> jacobian; 
      jacobian << (v1.transpose() * translational_jacobian) + (r.transpose() * J_u1),
                  (v2.transpose() * translational_jacobian) + (r.transpose() * J_u2);

      // Check error not too large
      if (r.norm() > 0.01) {
        throw std::runtime_error("Aborting; too far away from starting pose!");
      }

      // compute control
      Eigen::VectorXd tau_task(7), tau_d(7);

      // Spring damper system with damping ratio=1
      tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
      tau_d << tau_task + coriolis;

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
