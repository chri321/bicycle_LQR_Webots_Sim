#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/inertial_unit.h>
#include <webots/gyro.h>
#include <webots/position_sensor.h>

// time in [ms] of a simulation step
#define TIME_STEP 64

#define MAX_SPEED 6.28

#define rad2deg 180/3.1415


float Roll,dRoll;


	float LQR_K1=76.1107;//偏转角度   
	float LQR_K2=12.0762; //偏转角速度  
	float LQR_K3=-1.0101;//动量轮角速度  
	float LQR_K4=-0.0316;//动量轮角速度 
	  
	float Momentum_Wheel_Balance(float angle,float gyro,float motor_speed,float motor_pos)  
	{  
  	    float Momentum_PID_Out=0;  
	    float Bias;  
	    Bias=angle;    
	    Momentum_PID_Out = LQR_K1 * Bias +LQR_K2*gyro+motor_speed*LQR_K3+motor_pos*LQR_K4;     
	    return  Momentum_PID_Out;     
	}  


// entry point of the controller
int main(int argc, char **argv) {
  // initialize the Webots API
  wb_robot_init();
  
  double previous_position,position,dposition;
  
  //惯性计
  
  
  
  
  int step = wb_robot_get_basic_time_step();
  
  WbDeviceTag position_sensor = wb_robot_get_device("position sensor");
  wb_position_sensor_enable(position_sensor, step);

  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, step);
  
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, step);
  

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
  
     position = wb_position_sensor_get_value(position_sensor);
     dposition=position-previous_position;
     previous_position=position;
  
      // Retrieve robot position using the sensors.
    const double roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0] -1.5708+ M_PI / 2.0;
    const double pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    const double roll_acceleration = wb_gyro_get_values(gyro)[0];
    const double pitch_acceleration = wb_gyro_get_values(gyro)[1];
    
   
    
    const double speedout=100*Momentum_Wheel_Balance(roll,roll_acceleration,dposition,0);
    
   // printf("roll:%2f    pitch:%2f    speedout:%2f     dposition:%2f\n",rad2deg*roll,rad2deg*pitch,speedout,rad2deg*dposition);
    printf("%2f\n",rad2deg*roll);
    // write actuators inputs
    wb_motor_set_velocity(direction_motor,5);
    wb_motor_set_position(turn_motor, 0);
    wb_motor_set_velocity(dong_motor, speedout);
  }

  // cleanup the Webots API
  wb_robot_cleanup();
  return 0; //EXIT_SUCCESS
}
