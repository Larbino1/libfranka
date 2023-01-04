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
    std::stoi(argv[2]) - 1, // joint
    std::stod(argv[3]), // amplitude
    std::stod(argv[4]), // frequency
    std::stod(argv[5]) // duration
  };
    
  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();
    // franka::RobotState initial_state = robot.readOnce();

    std::array<double, 7> initial_position;
    double time = 0.0;
    std::function<franka::JointPositions(const franka::RobotState&, franka::Duration)>
      joint_control_callback = [&initial_position, &time, &cfg, &model](
        const franka::RobotState& robot_state, franka::Duration period
        ) -> franka::JointPositions {
          time += period.toSec();
          if (time == 0.0) {
            initial_position = robot_state.q_d;
          }
          
          double delta_angle = cfg.amplitude * (1 - std::cos(2 * M_PI * cfg.frequency * time));
          std::array<double, 7> desired_position = initial_position;
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
  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }
  
  return 0;
}
