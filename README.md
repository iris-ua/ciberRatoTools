
# CiberRato Robot Simulation Environment <br/> Universidade de Aveiro / IEETA

## Information

CiberRato Robot Simulation Environment simulates the movement
of robots inside a labyrinth.  Robots objective is to go from their
starting position to beacon area and then return to their start position.

The MicroRato competition
[http://microrato.ua.pt/], held annually at Aveiro University,
uses these these tools for its Explorer league.

### `linef` branch

The `linef` branch (this branch) is a demo of control systems based on the CiberRato tools, by hacking a collection of source files.

The system simulates a mouse that follows a line using several a controller. The types of available controllers are defined by the `controller_t` type, define in `robsample/controller.h`:

```C
typedef enum {
  NONE,           /**< No control action */
  BANG,           /**< Bang-bang control, unidirectional */
  BANG2,          /**< Bang-bang control, bipolar */
  BANGH,          /**< Bang-bang control with histeresis. */
  P,              /**< Proportional control */
  PID             /**< PID (Proportional-Integral-Derivative) control */
} controller_t;
```
A global variable in `robsample/controller.c` defines the type of controller to apply to the system.

```C
const controller_t activeController=P;
```

The controller parameters, and in particular the PID parameters, are defined in `robsample/controller.c`. The constant `Kp` is common to the P and PID controllers.
```C
// PID constants:
// Kp is the same for P and PID controller
const float Kp = 1;       // Kp - proportional constant
// const float Ti = ;     // Ti - Integration time
//      set to FLT_MAX to disable I component
const float Ti = FLT_MAX;
// const float Ti = 0.1/h;
const float Td = 0*h;     // Td - differential time
```

In order to provide a more challenging problem for the control systems, the mouse motor model can be of 1st or 2nd order, by definining the value of the variable `controlOrder` in `simulator/cbmotor.cpp`:     
```C
controlOrder=1;
```
`controlOrder` can take the value of 1 or 2, to select a 1st or 2nd order model for the motor. Note that the 1st order model used in `linef` is different from the motor model used in the `main` branch.

Note that, in this experiment, the robot may not start correctly positioned over the line and aligned in the proper direction. Check `linef-grid.xml` that defines the starting position and orientation. You can try to change these values to test the performance of a controller. 

The speed set-point for the motors is defined in [mainrob.c](robsample/mainRob.c), line 36:
```C
float velSetPoint=0.5; /* Velocity set point */ 
```


## Contents

* simulator -           The simulator source code
* Viewer -              The Visualizer source code
* logplayer -           The logplayer source code
* GUISample -           Graphical robot agent (C++) source code
* robsample -           robot agent (C) source code
* jClient -             robot agent (Java) source code
* pClient -             robot agent (Python) source code
* Labs -                examples of labyrinths used in previous competitions
* startAll -            script that runs the simulator, the visualizer and 5 GUISamples
* startSimViewer -      script that runs the simulator and the Viewer

## Install

The source code was compiled with gcc/g++ - Gnu Project C/C++ Compiler
(gcc version  9.3.0) using the Qt libraries (release 5.12.8) on Ubuntu 20.04.

It is required to have the development version of gcc/g++, cmake, Qt libraries
release 5.x installed in the system prior to compilation.
On Ubuntu 20.04 run the following:
```bash
sudo apt-get install build-essential cmake qtmultimedia5-dev
```

Then in the repository base dir, execute:
```bash
mkdir build
cd build
cmake ..
make
```

To run the control systems demo, Viewer and C++ agent, execute (at the repository base dir):
```bash
./startLinef
```

The start control launches the C agent, that will travel a path following a straight line. When the STOP button is pressed the simulation is concluded and a graph is displayed, plotting the position over the line (which, in control terms, corresponds to the error signal) and the commands applied to the motors.

The `startLinef` script uses [Octave](https://www.octave.org) for creating the plots.  

## Authors

* Nuno Lau,
  University of Aveiro,
  nunolau@ua.pt

* Artur C. Pereira,
  University of Aveiro,
  artur@ua.pt

* Andreia Melo,
  University of Aveiro,
  abmelo@criticalsoftware.com

* Antonio Neves,
  University of Aveiro,
  an@ua.pt

* Joao Figueiredo,
  University of Aveiro
  joao.figueiredo@ieeta.pt

* Miguel Rodrigues,
  University of Aveiro,
  miguel.rodrigues@ua.pt

* Eurico Pedrosa,
  University of Aveiro,
  efp@ua.pt

* Pedro Fonseca (for the `speedc` and `linef`branches),  University of Aveiro,  pf@ua.pt

 Copyright (C) 2001-2022 Universidade de Aveiro
