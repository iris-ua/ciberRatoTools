/**
* \file controller.h
*
* Provides controller(), a function to perform a collection of control actions.
*/

#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <float.h>



/**
* \typedef controller_t
*
* Defines the type of control action performed by the controller.
*
* \sa controller()
*
* \sa deltah
*/
typedef enum {
  NONE,           /**< No control action */
  BANG,           /**< Bang-bang control, unidirectional */
  BANG2,          /**< Bang-bang control, bipolar */
  BANGH,          /**< Bang-bang control with histeresis. */
  P,              /**< Proportional control */
  PID             /**< PID (Proportional-Integral-Derivative) control */
} controller_t;

extern const controller_t activeController;

/**
* float controller(controller_t type, float r, float y)
*
* Implements the control action, dependening on type.
*
* \param type   Controller type.
* \param r      Set point
* \param y      Feedback value
*
* If type is set to NONE, no feedback is used. The setpoint is propagated
* to the output (r=u)
*
* \sa controller_t
*/
float controller(controller_t type, float r, float y);

#endif
