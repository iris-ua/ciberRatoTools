/**
* \file controller.c
*
* Controller implementation.
*
*/

#include "controller.h"

const float h = 0.050;    // h  - sampling interval

const controller_t activeController=PID;

/*
* Constants for the controller
*/

// PID constants:
// Kp is the same for P and PID controller
const float Kp = 1;       // Kp - proportional constant
// const float Ti = ;     // Ti - Integration time
//      set to FLT_MAX to disable I component
const float Ti = FLT_MAX;
// const float Ti = 1/h;

//const float Td = 0*h;     // Td - differential time
float Td = 0*h;     // Td - differential time
// float Td = h;     // Td - differential time

const float max_u = 10;  // max_u - saturation value for control signal

/* hysteresis for bang-bang controller */
const float deltah=0.05;
const float bangvalue = 0.5;

float controller(controller_t type, float r, float y)
{
  float u=0;    /**> Control signal */

  // Auxiliary constants for the PID controller
  //static const float K0 = Kp*(1+h/Ti+Td/h);
  //static const float K1 = -Kp*(1+2*Td/h);
  //static const float K2 = Kp*Td/h;
  float K0 = Kp*(1+h/Ti+Td/h);
  float K1 = -Kp*(1+2*Td/h);
  float K2 = Kp*Td/h;

  // memory for error
  static float e_m1 = 0;
  static float e_m2 = 0;

  // memory for the control signal
  static float u_m1 = 0;

  // Compute error signal
  float e = r - y;

  /* Implement control action depending on the type of control. */
  switch (type) {
    case NONE:
      /* No feedback action */
      /* Controller output is the reference input */
      u = r;
      break;
    case BANG:
      /* Bang-bang control, unidirectional */
      if(e>0){
        u = bangvalue;
      }
      else{
        u = 0;
      }
      break;
    case BANG2:
      /* Bang-bang control with bipolar output */
      if(e>0){
        u = bangvalue;
      }
      else if (e<0){
        u = -bangvalue;
      }
      else{
        u = 0;
      }
      break;
    case BANGH:
      /* Bang-bang control with hysteresis */

      /* deltah is the histeresis value */
      if(e>0.5*deltah){
        u = bangvalue;
      }
      else if (e<-0.5*deltah){
        u = -bangvalue;
      }
      else{
        /* If error within histeresis limits, keep the value */
        u = u_m1;
      }
      u_m1 = u;
      break;

    case P:
    /* Proportional control */
      u = Kp*e;
      break;

    case PID:

      /* Compute control signal */
      u = u_m1 + K0*e + K1*e_m1 + K2*e_m2;

      /* store values for next iterations */
      e_m2 = e_m1;
      e_m1 = e;
      u_m1 = u;

      // Clip the control signal to avoid saturation
      if(u > max_u){
        u = max_u;
      }
      if (u < -max_u){
        u = -max_u;
      }

      break;


  }

  return u;
}
