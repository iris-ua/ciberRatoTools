/*
 * pioneer_wall_follower.c
 * First-approach right-wall follower for the Webots Pioneer 3-DX.
 *
 * Uses sonars so7 (front-right side) and so8 (rear-right side), both
 * pointing straight right, to estimate distance and angle to the wall.
 * Uses the front sonars so3/so4 to avoid obstacles ahead.
 *
 * Check SONAR_SPACING and the sonar lookupTable against Pioneer3dx.proto.
 */

#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/supervisor.h>
#include <stdio.h>
#include <math.h>
#include "controller.h"


#define N_SONARS 16
#define MAX_SPEED 12.3 /* rad/s, wheel limit */
#define BASE_SPEED 4.0 /* rad/s, cruising speed (~0.4 m/s) */

#define TARGET_Y 0.5

/* The CONTROL_TYPE macro will be passed to the controller() function and 
    defines the type of control (see controller.h) */
#define CONTROL_TYPE BANG2

static double clamp(double v, double lo, double hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

double get_robot_y(void)
{
  static WbNodeRef self = NULL;
  if (self == NULL)
    self = wb_supervisor_node_get_self();
  if (self == NULL)
    return NAN; /* not a supervisor */
  return wb_supervisor_node_get_position(self)[1];
}

int main(void)
{
  wb_robot_init();
  int time_step = (int)wb_robot_get_basic_time_step();

  WbDeviceTag left_motor = wb_robot_get_device("left wheel");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  FILE *log = fopen("robot_log.csv", "w");
  if (log == NULL)
  {
    fprintf(stderr, "Cannot open robot_log.csv\n");
    return 1;
  }
  fprintf(log, "time,target,y,error,steer\n");

  int step = 0;
  double target = TARGET_Y;
  while (wb_robot_step(time_step) != -1)
  {

    double left_speed, right_speed;

    double y = get_robot_y();
    double error = target - y;

    double steer = controller(CONTROL_TYPE,target,y);

    left_speed = BASE_SPEED - steer;
    right_speed = BASE_SPEED + steer;

    if (step % 2 == 0)
    {
      printf("T=%.2f y=%.2f e:%.2f s:%.2f L:%.2f R:%.2f \n",
             target, y, error, steer, left_speed, right_speed);
    }

    fprintf(log, "%.3f,%.4f,%.4f,%.4f,%.4f\n",
            wb_robot_get_time(), target, y, error, steer);
    fflush(log);

    wb_motor_set_velocity(left_motor, clamp(left_speed, -MAX_SPEED, MAX_SPEED));
    wb_motor_set_velocity(right_motor, clamp(right_speed, -MAX_SPEED, MAX_SPEED));

    step++;

    if(wb_robot_get_time() > 60.0){
      break; 
    }

    if(wb_robot_get_time() > 30.0){
      target = 0.8*TARGET_Y;
    }
  }

  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  fclose(log);
  wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);
  wb_robot_step(time_step); /* sends the pause request to Webots */
  wb_robot_cleanup();
  return 0;
}
