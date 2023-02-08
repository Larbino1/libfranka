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
  int joint_id;
  double amplitude;
  double frequency;
  double duration;
};

int main(int argc, char** argv) {
  // Check whether the required arguments were passed
  if (argc != 6) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname> <jointid,1-7> <amplitude> <freq> <duration>" << std::endl;
    return -1;
  }

  //TODO input parsing
  TrajectoryConfig cfg = {
    std::stoi(argv[2]) - 1, // joint (idx)
    std::stod(argv[3]), // amplitude (rad)
    std::stod(argv[4]), // frequency (Hz)
    std::stod(argv[5]) // duration (s)
  };
    
  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    while (!YesNoPrompt("Move the robot to the correct initial position. Done? [y/n]")){}

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q0, qleft, qright;
    q0 = robot.readOnce().q;
    qleft = q0;
    qright = q0;
    qleft[6] += cfg.amplitude;
    qright[6] -= cfg.amplitude;

    const double max_f{20.0};
    const double max_t{10.0};
    // set collision behavior
    robot.setCollisionBehavior(
        {{max_t, max_t, max_t, max_t, max_t, max_t, max_t}}, {{max_t, max_t, max_t, max_t, max_t, max_t, max_t}},
        {{max_f, max_f, max_f, max_f, max_f, max_f}}, {{max_f, max_f, max_f, max_f, max_f, max_f}});

    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    while (!YesNoPrompt("Ready to move to leftmost position? [y/n]")){}
    MotionGenerator motion_generator_1(0.1, qleft);
    robot.control(motion_generator_1);
    std::cout << "Finished moving." << std::endl;

    while (!YesNoPrompt("Ready to move to rightmost position? [y/n]")){}
    MotionGenerator motion_generator_2(0.1, qright);
    robot.control(motion_generator_2);
    std::cout << "Finished moving." << std::endl;

    std::array<double, 7> initial_position;
    double time = 0.0;
    std::function<franka::JointPositions(const franka::RobotState&, franka::Duration)>
      joint_control_callback = [&q0, &time, &cfg, &model](
        const franka::RobotState& robot_state, franka::Duration period
        ) -> franka::JointPositions {
          time += period.toSec();
          
          double delta_angle = -cfg.amplitude * std::cos(2 * M_PI * cfg.frequency * time);
          std::array<double, 7> desired_position = q0;
          desired_position[cfg.joint_id] += delta_angle;
          franka::JointPositions output = {{desired_position[0], desired_position[1],
                                            desired_position[2], desired_position[3],
                                            desired_position[4], desired_position[5],
                                            desired_position[6]}};
          logTorques(model, robot_state);
          logState(robot_state);
          if (time >= cfg.duration) {
            std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
            return franka::MotionFinished(output);
          }
          return output;
    };

    // start real-time control loop
    std::cout << "WARNING: Collision thresholds are set to high values. "
              << "Make sure you have the user stop at hand!" << std::endl
              << "After starting try to push the robot and see how it reacts." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(joint_control_callback);

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
