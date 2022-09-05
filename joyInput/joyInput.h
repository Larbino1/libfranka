#include <Eigen/Dense>

#include "SDL2/SDL.h"


double deadzone(double val);

SDL_Joystick* controllerInit();

class RefState {
 public:
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  RefState(Eigen::Vector3d pos0) {
    pos = pos0;
    vel.setZero();
  }
  void update(double dt);
};

class StickReader {
 private:
  SDL_Joystick* joy;
  int ax_x_ID;
  int ax_y_ID;
  double ax_max;
  double out_max;

 public:
  StickReader(SDL_Joystick* joy_, int ax_x_ID_, int ax_y_ID_, double ax_max_, double out_max_) {
    joy = joy_;
    ax_x_ID = ax_x_ID_;
    ax_y_ID = ax_y_ID_;
    ax_max = ax_max_;
    out_max = out_max_;
  }
  Eigen::Vector2d read();
};

template <class F>
void with_controller(F&& f) {
  SDL_Joystick* controller = controllerInit();
  f(controller);
  SDL_JoystickClose(controller);
  SDL_Quit();
}