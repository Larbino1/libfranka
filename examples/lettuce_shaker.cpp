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

struct TrajectoryConfig{
  double bias;
  double amplitude;
  double frequency;
  double duration;
};

void logLettuceMotion(ImpedanceCoordResult<3, 7> coord) {
  double z = coord.z.norm();
  double dz = coord.dz.norm();
  std::string msg("");
  msg += std::to_string(z) + ", " + std::to_string(dz);
  myLog("ee_z_dz_", msg);
}

int main(int argc, char** argv) {
  // Check whether the required arguments were passed
  if (argc != 6) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname> <bias_force> <force_sin_amplitude> <freq> <duration>" << std::endl;
    return -1;
  }

  TrajectoryConfig cfg = {
    std::stod(argv[2]), // bias_force (N)
    std::stod(argv[3]), // amplitude (N)
    std::stod(argv[4]), // frequency (Hz)
    std::stod(argv[5]) // duration (s)
  };
  int j7idx = 6;

  std::cout << "Bias " << cfg.bias << ", Amplitude: " << cfg.amplitude << ", Freq: " << cfg.frequency << ", Duration: " << cfg.duration << std::endl;

  // Geometric parameters
  const Eigen::Vector3d ee_offset({0.377, 0.0, 0.042});
  const auto frame = franka::Frame::kEndEffector;
    
  // setup ee
  WorldCoord ee;
  ee.offset = ee_offset;
  // ee.ref = ref.pos;


  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    while (!YesNoPrompt("Move the robot to the correct initial position. Done? [y/n]")){}

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q0; 
    auto initial_state = robot.readOnce();
    q0 = initial_state.q;

    Eigen::Array<double, 7, 1> q0_arr;
    q0_arr << q0[0], q0[1], q0[2], q0[3], q0[4], q0[5], q0[6];

    const double max_f{20.0};
    const double max_t{20.0};
    // set collision behavior
    robot.setCollisionBehavior(
        {{max_t, max_t, max_t, max_t, max_t, max_t, max_t}}, {{max_t, max_t, max_t, max_t, max_t, max_t, max_t}},
        {{max_f, max_f, max_f, max_f, max_f, max_f}}, {{max_f, max_f, max_f, max_f, max_f, max_f}});

    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    // set initial ee ref
    ImpedanceCoordArgs iargs(initial_state, model, frame);
    auto ee_coord_initial = computeWorldCoord(iargs, ee);
    ee.ref = ee_coord_initial.z;
    Eigen::Vector3d force_dir = ee_coord_initial.J.rightCols(1).normalized();

    std::cout << "Force dir:" << std::endl << force_dir << std::endl;

    // Define control parameters
    Eigen::Array<double, 7, 1> joint_stiffness;
    joint_stiffness << 3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 10.0;
    Eigen::Array<double, 7, 1> joint_damping;
    joint_damping << 40.0, 40.0, 25.0, 20.0, 15.0, 10.0, 0.0;

    double motion_amplitude = cfg.amplitude / joint_stiffness[6] * ee_offset[0];
    std::cout << "With this force amplitude and stiffness, a motion of ";
    std::cout << 100*motion_amplitude << "cm is required." << std::endl;

    double time = 0.0;

    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState& robot_state,
                                          franka::Duration duration) -> franka::Torques {
      time += duration.toSec();

      Eigen::Map<const Eigen::Array<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Array<double, 7, 1>> dq(robot_state.dq.data());

      ImpedanceCoordArgs iargs(robot_state, model, frame);

      // compute control coordinate error and jacobian
      auto ee_coord = computeWorldCoord(iargs, ee);

      // Check error not too large
      if (ee_coord.z.norm() > 0.1) {
        throw std::runtime_error("Aborting; too far away from starting pose!");
      }

      Eigen::Matrix<double, 7, 1> tau_fb, tau_ff, tau_d;
      // Joint 1-7 PD control
      double sin_2_pi_f_t = std::sin(2 * M_PI * cfg.frequency * time);
      double cos_2_pi_f_t = std::cos(2 * M_PI * cfg.frequency * time);

      double delta_angle = motion_amplitude * sin_2_pi_f_t;
      Eigen::Array<double, 7, 1> q_des = q0_arr;
      // q_des[j7idx] = q0_arr[j7idx] + delta_angle;
      
      tau_fb = (joint_stiffness * (q_des - q) - joint_damping * dq);
      // Feedforward the desired force 
      // 2 second windowing function
      double F = (cfg.bias - cfg.amplitude * cos_2_pi_f_t);
      tau_ff = ee_coord.J.transpose() * force_dir * F;

      tau_d = tau_fb + tau_ff;
      // convert to double array
      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
      franka::Torques ret(tau_d_array);

      logTorques(model, robot_state);
      logState(robot_state);
      logLettuceMotion(ee_coord);

      if (time >= cfg.duration) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(ret);
      }
      return ret;
    };

    // start real-time control loop
    std::cout << "WARNING: Collision thresholds are set to high values. "
              << "Make sure you have the user stop at hand!" << std::endl
              << "After starting try to push the robot and see how it reacts." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(impedance_control_callback, true, 1000);

    while (!YesNoPrompt("Ready to move back to centre position? [y/n]")){}
    MotionGenerator motion_generator_3(0.1, q0);
    robot.control(motion_generator_3);
    std::cout << "Finished moving." << std::endl;

  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }
  
  return 0;
}
