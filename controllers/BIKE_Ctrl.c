#include <webots/robot.h>
#include <webots/motor.h>


// time in [ms] of a simulation step
#define TIME_STEP 64

#define MAX_SPEED 6.28




float Roll,dRoll;




// entry point of the controller
int main(int argc, char **argv) {
  // initialize the Webots API
  wb_robot_init();

  WbDeviceTag direction_motor = wb_robot_get_device("direction motor");
  WbDeviceTag turn_motor = wb_robot_get_device("turn motor");
  WbDeviceTag dong_motor = wb_robot_get_device("dong motor");

  wb_motor_set_position(direction_motor, INFINITY);
  wb_motor_set_velocity(direction_motor, 0.0);

  wb_motor_set_position(turn_motor, 0);
  //wb_motor_set_velocity(turn_motor, 1.0);
  
  wb_motor_set_position(dong_motor, INFINITY);

  // feedback loop: step simulation until an exit event is received
  while (wb_robot_step(TIME_STEP) != -1) {

    // write actuators inputs
    wb_motor_set_velocity(direction_motor, 0);
    wb_motor_set_position(turn_motor, 0);
    wb_motor_set_velocity(dong_motor, 10);
  }

  // cleanup the Webots API
  wb_robot_cleanup();
  return 0; //EXIT_SUCCESS
}
