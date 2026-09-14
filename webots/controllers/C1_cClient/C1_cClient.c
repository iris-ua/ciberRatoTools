#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <stdio.h>

#define NUM_DIST_SENSORS 8
#define NUM_GROUND_SENSORS 3
#define CRUISING_VELOCITY 3.0
#define WHITE_THRESHOLD 500

int main(int argc, char **argv) {
  wb_robot_init();

  // Get simulation step length.
  int time_step = (int)wb_robot_get_basic_time_step();

  // Get left and right wheel motors.
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");

  // Get frontal distance sensors.
  WbDeviceTag dist_sensors[NUM_DIST_SENSORS];
  char sensor_name[16];
  for (int i = 0; i < NUM_DIST_SENSORS; ++i) {
    snprintf(sensor_name, sizeof(sensor_name), "ps%d", i);
    dist_sensors[i] = wb_robot_get_device(sensor_name);
    wb_distance_sensor_enable(dist_sensors[i], time_step);
  }

  // Get ground sensors (also DistanceSensor devices in Webots).
  WbDeviceTag ground_sensors[NUM_GROUND_SENSORS];
  for (int i = 0; i < NUM_GROUND_SENSORS; ++i) {
    snprintf(sensor_name, sizeof(sensor_name), "gs%d", i);
    ground_sensors[i] = wb_robot_get_device(sensor_name);
    wb_distance_sensor_enable(ground_sensors[i], time_step);
  }

  // Enable the camera (same as the Python controller).
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, time_step);

  // Disable motor PID control mode.
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  // Set the initial velocity of the left and right wheel motors.
  wb_motor_set_velocity(left_motor, CRUISING_VELOCITY);
  wb_motor_set_velocity(right_motor, CRUISING_VELOCITY);

  // Main loop.
  while (wb_robot_step(time_step) != -1) {
    double dist_sensor_values[NUM_DIST_SENSORS];
    double ground_sensor_values[NUM_GROUND_SENSORS];

    for (int i = 0; i < NUM_DIST_SENSORS; ++i) {
      dist_sensor_values[i] = wb_distance_sensor_get_value(dist_sensors[i]);
    }

    for (int i = 0; i < NUM_GROUND_SENSORS; ++i) {
      ground_sensor_values[i] = wb_distance_sensor_get_value(ground_sensors[i]);
    }

    printf("[%.2f, %.2f, %.2f]\n",
           ground_sensor_values[0], ground_sensor_values[1], ground_sensor_values[2]);

    if (ground_sensor_values[0] > WHITE_THRESHOLD &&
        dist_sensor_values[2] > WHITE_THRESHOLD) {
      printf("rotate\n");
      wb_motor_set_velocity(left_motor, -CRUISING_VELOCITY);
      wb_motor_set_velocity(right_motor, CRUISING_VELOCITY);
    } else if (ground_sensor_values[2] > WHITE_THRESHOLD) {
      printf("turn left\n");
      wb_motor_set_velocity(left_motor, 0.1 * CRUISING_VELOCITY);
      wb_motor_set_velocity(right_motor, 1.2 * CRUISING_VELOCITY);
    } else if (ground_sensor_values[0] > WHITE_THRESHOLD) {
      printf("turn right\n");
      wb_motor_set_velocity(left_motor, 1.2 * CRUISING_VELOCITY);
      wb_motor_set_velocity(right_motor, 0.1 * CRUISING_VELOCITY);
    } else {
      printf("go\n");
      wb_motor_set_velocity(left_motor, CRUISING_VELOCITY);
      wb_motor_set_velocity(right_motor, CRUISING_VELOCITY);
    }
  }

  wb_robot_cleanup();
  return 0;
}