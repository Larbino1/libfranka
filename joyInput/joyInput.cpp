#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>

#include <SDL2/SDL.h>

const double AX_MAX = pow(2, 15);
const double VMAX = 0.01; // In m/s
const double AX_SCALE = VMAX / AX_MAX;
const double DEADZONE = 0.02*VMAX;

double deadzone(double val) {
  if (abs(val) < DEADZONE) {
    return 0.0;
  } else {
    return val;
  }
}


int main() {
  std::cout << "Starting joystick input" << std::endl;
  if (SDL_Init( SDL_INIT_JOYSTICK ) < 0)
    {
    //  fprintf(stderr, "Couldn't initialize SDL: %s\n", SDL_GetError());
      exit(1);
    }
    // Open the joystick for reading and store its handle in the joy variable
    SDL_Joystick *joy = SDL_JoystickOpen(0);

    double x, y, z, dx, dy, dz;
    x = 0;
    y = 0;
    z = 0;

    // If the joy variable is NULL, there was an error opening it.
    if (joy != NULL) {
      int quit = 0;
      // Get information about the joystick
      const char *name = SDL_JoystickName(joy);
      const int num_axes = SDL_JoystickNumAxes(joy);
      const int num_buttons = SDL_JoystickNumButtons(joy);
      const int num_hats = SDL_JoystickNumHats(joy);
      if (!(num_axes == 6 && num_buttons == 11 && num_hats == 1)) {
        printf("Error, joystick '%s' with:\n"
               "%d axes\n"
               "%d buttons\n"
               "%d hats\n\n",
               name,
               num_axes,
               num_buttons,
               num_hats);
	printf("Expected 6, 11 and 1.");
      	quit = 1;
      }
      // Keep reading the state of the joystick in a loop
      using namespace std::chrono;
      const auto dt = 10ms;
      const double dtd = 0.01; // in seconds
      auto next = steady_clock::now() + dt;
      auto prev = next - dt;
      while (quit == 0) {
        if (SDL_QuitRequested()) {
            quit = 1;
        }
	if (prev > next) {
	  std::cerr << "Error. Controller input loop too slow..." << std::endl;
	  quit = 1;
	}
        prev = steady_clock::now();
        next += dt;

        // Read inputs
	
	dx =  deadzone(SDL_JoystickGetAxis(joy, 3) * AX_SCALE);
	dy = -deadzone(SDL_JoystickGetAxis(joy, 4) * AX_SCALE);

	x = x + dx * dtd;
	y = y + dy * dtd;
	z = z + dz * dtd;

	std::cout << "x" << x;
        std::cout << " y" << y;
        std::cout << " z" << z;
        std::cout << " dx" << dx;
        std::cout << " dy" << dy;
        std::cout << " dz" << dz;
	std::cout << std::endl;
        // delay until time to iterate again
        std::this_thread::sleep_until(next);
      }

      SDL_JoystickClose(joy);
    } else {
        printf("Couldn't open the joystick. Quitting now...\n");
    }

    SDL_Quit();
    return 0;
}
