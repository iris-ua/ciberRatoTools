/*
 * File:          pioneer3dx_wall_follower.c
 * Date:
 * Description:
 * Author:
 * Modifications:
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
#define BASE_SPEED 1.0 /* rad/s, cruising speed (~0.1 m/s) */

#define SONAR_RANGE   5.0    /* m, lookupTable: 1024 at 0 m -> 0 at 5 m */
#define SONAR_MAXVAL  1024.0
#define SONAR_SPACING 0.216   /* m, distance between so7 and so8 along x */

#define TARGET_Y 0.5

#define TARGET_DIST   0.50   /* m, desired sonar-to-wall distance */
#define FRONT_LIMIT   0.70   /* m, obstacle ahead -> turn left */
#define LOST_DIST     2.00   /* m, beyond this the wall is considered lost */

#define TIME1   120.0
#define TIME2   240.0
#define TARGET_RATIO 1.1


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

static double sonar_to_meters(double value) {
  double d = SONAR_RANGE * (1.0 - value / SONAR_MAXVAL);
  if (d < 0.0) d = 0.0;
  if (d > SONAR_RANGE) d = SONAR_RANGE;
  return d;
}

int main(void)
{
  wb_robot_init();
  int time_step = (int)wb_robot_get_basic_time_step();
  
  /* Set up the sonar sensors */
  WbDeviceTag so[N_SONARS];
  char name[5];
  for (int i = 0; i < N_SONARS; i++) {
    sprintf(name, "so%d", i);
    so[i] = wb_robot_get_device(name);
    wb_distance_sensor_enable(so[i], time_step);
  }

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
  double target = TARGET_DIST;
  
  while (wb_robot_step(time_step) != -1)
  {
  
    double d[N_SONARS];
    for (int i = 0; i < N_SONARS; i++)
      d[i] = sonar_to_meters(wb_distance_sensor_get_value(so[i]));

    double d_front = fmin(d[3], d[4]);   /* sonars at about +/-10 deg */
    double d7 = d[7];                     /* front-right side */
    double d8 = d[8];                     /* rear-right side */


    double left_speed, right_speed;
    
    double dist, angle;
    const char* mode; 

    if (d_front < FRONT_LIMIT) {
      /* Obstacle ahead: spin left, away from the wall */
      left_speed = -0.5 * BASE_SPEED;
      right_speed = 0.5 * BASE_SPEED;
      mode = "AVOID";
    } else if (d7 > LOST_DIST && d8 > LOST_DIST) {
      /* No wall on the right: arc right to find it */
      left_speed = BASE_SPEED;
      right_speed = 0.5 * BASE_SPEED;
      mode = "SEARCH";
    } else {
      /* Follow the wall */
      double dist, angle;
      if (d7 > LOST_DIST) {          /* only rear sonar sees the wall */
        dist = d8;
        angle = 0.0;
      } else if (d8 > LOST_DIST) {   /* only front sonar sees the wall */
        dist = d7;
        angle = 0.0;
      } else {
        dist = 0.5 * (d7 + d8);
        /* positive angle: front closer than rear -> heading into the wall */
        angle = atan2(d8 - d7, SONAR_SPACING);
      }
    }

    double y = get_robot_y();
    
    double error = target - dist;

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

    if(wb_robot_get_time() > TIME1){
      target = TARGET_RATIO*TARGET_DIST;
    }

    if(wb_robot_get_time() > TIME2){
      break; 
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
