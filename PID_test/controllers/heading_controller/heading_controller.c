/*
 * File:          epuc_distance_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h> 
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/compass.h>

#include <stdio.h>
#include <math.h>

#include "controller.h"

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define MAX_SPEED 6.28

/* Length of each heading stage */
#define STAGE_LENGTH 5.0


double get_bearing_in_degrees(WbDeviceTag tag) {
  const double *north = wb_compass_get_values(tag);
  double rad = atan2(north[1], north[0]);
  double bearing = (rad / M_PI) * 180.0;
  if (bearing < 0.0)
    bearing = bearing + 360.0;
  return bearing;
}

double get_bearing_in_rads(WbDeviceTag tag) {
  const double *north = wb_compass_get_values(tag);
  double rad = atan2(north[1], north[0]);
  if (rad < 0.0)
    rad = rad + 2*M_PI;
  return rad;
}

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();


  
  /*
   * Create the motors, and set to velocity control.
   */
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.5*MAX_SPEED);
  wb_motor_set_velocity(right_motor, 0.5*MAX_SPEED); 
  
  /*
   * Enable the compass
   */
  WbDeviceTag compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, TIME_STEP);
  
  /* Open the file for storing data */
  char filename[] = "data.csv";
  FILE *fp = fopen(filename,"w");
  if(!fp){
    printf("Error opening %s!\n",filename); 
    return -1; 
  }

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {

    /*
     * Read the sensors
     */
     
     const float base_angle = M_PI/8; 
     const float heading_sp_k[] = {+1.0, +3.0, 5.0, 7.0, 5.0,3.0, +1.0, +3.0};
                               
     unsigned int stage = ((int) (wb_robot_get_time()/STAGE_LENGTH))%8;

    float heading_sp = heading_sp_k[stage] * base_angle; 

    float m = controller(activeController,heading_sp, get_bearing_in_rads(compass)); 

    /* apply corrections to motors */
    
    float left_speed = 0.5*MAX_SPEED + m; 
    float right_speed = 0.5*MAX_SPEED - m; 
        
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed); 

    bool left_speed_exceeded = (left_speed > MAX_SPEED);
    bool right_speed_exceeded = (right_speed > MAX_SPEED);
    

    printf("%6.3f\t",wb_robot_get_time()); 
    printf("%d\t",stage);
    printf("%6.3f\t",heading_sp);
    // printf("%5.2f\t",base_angle); 
    printf("%5.2f\n",get_bearing_in_rads(compass));


    fprintf(fp,"%6.3f\t",wb_robot_get_time()); 
    fprintf(fp,"%d\t",stage);
    fprintf(fp,"%d\t",left_speed_exceeded || right_speed_exceeded);
    fprintf(fp,"%6.3f\t",heading_sp);
    // fprintf(fp,"%5.2f\t",base_angle); 
    fprintf(fp,"%5.2f\n",get_bearing_in_rads(compass));
    /* force write */
    fflush(fp);

    
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
