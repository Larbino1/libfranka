#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>

#include <Eigen/Dense>

#include <SDL2/SDL.h>

const double AX_MAX = pow(2, 15);
const double VMAX = 0.01;  // In m/s
const double AX_SCALE = VMAX / AX_MAX;
const double DEADZONE = 0.02 * VMAX;

double deadzone(double val) {
  if (abs(val) < DEADZONE) {
    return 0.0;
  } else {
    return val;
  }
}

SDL_Joystick* controllerInit() {
  std::cout << "Starting controller input" << std::endl;
  if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
    fprintf(stderr, "Couldn't initialize SDL: %s\n", SDL_GetError());
    exit(1);
  }
  // Open the joystick for reading and store its handle in the joy variable
  SDL_Joystick* controller = SDL_JoystickOpen(0);
  if (controller != NULL) {
    // Get information about the joystick
    const char* name = SDL_JoystickName(controller);
    const int num_axes = SDL_JoystickNumAxes(controller);
    const int num_buttons = SDL_JoystickNumButtons(controller);
    const int num_hats = SDL_JoystickNumHats(controller);
    if (!(num_axes == 6 && num_buttons == 11 && num_hats == 1)) {
      printf(
          "Error, joystick '%s' with:\n"
          "%d axes\n"
          "%d buttons\n"
          "%d hats\n\n",
          name, num_axes, num_buttons, num_hats);
      printf("Expected 6, 11 and 1.");
      exit(1);
    } 
    return controller;
  }
  else {
    fprintf(stderr, "Couldn't open the joystick (SDL_Joystick*==NULL)\n");
    exit(1);
  }
}

class RefState {
 public:
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  RefState(Eigen::Vector3d pos0) {
    pos = pos0;
    vel.setZero();
  }
  void update(double dt) {  // Method/function defined inside the class
    pos += vel * dt;
  }
};

template <class F>
void with_controller(F&& f) {
  SDL_Joystick* controller = controllerInit();
  f(controller);
  SDL_JoystickClose(controller);
  SDL_Quit();
}

int main() {
  RefState ref(Eigen::Vector3d::Zero());

  with_controller([&](SDL_Joystick* controller) {
    // Keep reading the state of the joystick in a loop
    using namespace std::chrono;
    const auto dt = 10ms;
    const double dtd = 0.01;  // in seconds
    auto next = steady_clock::now() + dt;
    auto prev = next - dt;
    int quit = 0;
    while (quit == 0) {
      if (prev > next) {
        std::cerr << "Error. Controller input loop too slow..." << std::endl;
        quit = 1;
      }
      prev = steady_clock::now();
      next += dt;

      // Read inputs
      double dx = deadzone(SDL_JoystickGetAxis(controller, 3) * AX_SCALE);
      double dy = -deadzone(SDL_JoystickGetAxis(controller, 4) * AX_SCALE);

      ref.vel[0] = dx;
      ref.vel[1] = dy; 
      ref.update(dtd);

      std::cout << ref.pos << std::endl;
      std::cout << ref.vel << std::endl;
      
      // delay until time to iterate again
      std::this_thread::sleep_until(next);
    }
  });
  return 0;
}
