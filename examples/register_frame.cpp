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

const Eigen::Vector3d ZERO(0.0, 0.0, 0.0);

int main(int argc, char** argv) {
  // Check whether the required arguments were passed
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
  }
 
  const char *homedir;
  homedir = getenv("HOME");
  fs::path home(homedir);
  // fs::path registration_points_filepath = home / "data/femoralhead.json";
  fs::path registration_points_filepath = home / "data/calibrationTriangle.json";
  fs::path trajectory_filepath = home / "data/trajectory.json";
  fs::path transform_cache = home / "data/register_frame_demo_transform_cache.json";

  PointsOrTransform p_or_tf = load_points_or_cached_transform(registration_points_filepath, transform_cache);
  Eigen::MatrixXd trajectory = parse_trajectory(open_json(trajectory_filepath));
  std::cout << "Loaded trajectory with " << trajectory.cols() << " points, starting at:" << std::endl << trajectory.col(1) << std::endl;

  // Geometric parameters
  std::cout << "USING POINTY TOOL OFFSET" << std::endl;
  Eigen::Vector3d ee_offset({-0.0005, -0.00014, 0.0322});
  // Eigen::Vector3d ee_offset({0.377, 0.0, 0.042});
  const auto frame = franka::Frame::kEndEffector;

  // Compliance parameters
  const double stiffness{2000.0};
  const double damping{20.0};
  DiagonalSpringDamper<3,7> ee_impedance{Eigen::Array3d::Constant(stiffness),
                                         Eigen::Array3d::Constant(damping)};

  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();

    // ee_offset = register_ee_offset(robot);
    
    Eigen::Affine3d mesh_transform;
    if (p_or_tf.got_transform) {
      mesh_transform = p_or_tf.transform;
    }
    else {
      RegistrationPoints registration = p_or_tf.points;
      int NPoints = registration.points.cols();    
      Eigen::MatrixXd registered_points(3, NPoints);  
      std::cout << "About to iterate through " << NPoints << " points" << std::endl;
      for (int i = 0; i < NPoints; i++) {
        std::cout << "Please register point " << registration.names[i] << std::endl;
        // Register point
        Eigen::Vector3d p_B = register_point_prompt(robot, ee_offset);
        registered_points.col(i) << p_B;
      }
      std::cout << "registration points, from json:" << std::endl << registration.points << std::endl;
      std::cout << "registered points, from robot:" << std::endl << registered_points << std::endl << std::endl;
      PointMatchResult pointMatchResult = matchPointSets(registration.points, registered_points);
      std::cout << "Success?:" << pointMatchResult.success << std::endl;
      std::cout << "Rotation:" << std::endl << pointMatchResult.rotation << std::endl;
      std::cout << "Translation:" << std::endl << pointMatchResult.translation << std::endl;
      std::cout << "Mean distance:" << pointMatchResult.avg_error * 1000 << "mm." << std::endl;
      std::cout << "Max distance:" << pointMatchResult.max_error * 1000 << "mm." << std::endl;
      Eigen::Matrix4d transform; // Your Transformation Matrix
      transform.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
      transform.block<3,3>(0,0) = pointMatchResult.rotation;
      transform.block<3,1>(0,3) = pointMatchResult.translation;    
      if (YesNoPrompt("Save transform to cache? [y/n]")) {
        save_frame_transform(transform_cache, Eigen::Affine3d(transform));
      }
      mesh_transform.matrix() = transform;
    }

    // Ask user to move to start point of trajectory
    std::cout << "Please move to within 1cm of the trajectory start. Current ee error:" << std::endl;
    franka::RobotState state = robot.readOnce();
    Eigen::Vector3d pos, traj_start, err;
    traj_start = mesh_transform * Eigen::Vector3d(trajectory.col(0));
    
    while (((pos - traj_start).norm() > 1e-2) || state.robot_mode == franka::RobotMode::kGuiding) {
      state = robot.readOnce();
      // std::cout << state.robot_mode;
      Eigen::Affine3d ee_transform(Eigen::Map<Eigen::Matrix4d>(state.O_T_EE.data()));
      pos = ee_transform * ee_offset;
      err = pos - traj_start;
      std::cout << "\33[2K\r [" << err[0] << "\r\t, " << err[1] << "\r\t\t, " << err[2] <<"]"; flush(std::cout);
    }
    std::cout << std::endl;

    // Now follow trajectory...

    int loopCounter = 0;
    // setup ee
    WorldCoord ee;
    ee.offset = ee_offset;
    ee.ref = traj_start;

    const double max_f{10.0};
    const double max_t{10.0};
    // set collision behavior
    robot.setCollisionBehavior(
        {{max_t, max_t, max_t, max_t, max_t, max_t, max_t}}, {{max_t, max_t, max_t, max_t, max_t, max_t, max_t}},
        {{max_f, max_f, max_f, max_f, max_f, max_f}}, {{max_f, max_f, max_f, max_f, max_f, max_f}});

    // define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState& robot_state,
                                          franka::Duration duration) -> franka::Torques {
      // Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      ImpedanceCoordArgs iargs(robot_state, model, frame);

      loopCounter++;
      ee.ref = mesh_transform * Eigen::Vector3d(trajectory.col(loopCounter));

      // compute control coordinate error and jacobian
      auto ee_coord = computeWorldCoord(iargs, ee);
      auto joint_coord = computeJointCoord(iargs);
      // Check error not too large
      if (ee_coord.z.norm() > 0.05) {
        throw std::runtime_error("Aborting; too far away!");
      }

      // compute control
      Eigen::VectorXd tau_d(7);
      tau_d << ee_impedance.tau(ee_coord); 
      
      // convert to double array
      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
      if (loopCounter >= trajectory.cols()) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(franka::Torques(tau_d_array));
      }
      return tau_d_array;
    };

    // start real-time control loop
    std::cout << "WARNING: Collision thresholds are set to high values. "
              << "Make sure you have the user stop at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    myGetChar();
    robot.control(impedance_control_callback, true, 1000);
    
  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  return 0;
}