
# CiberRato Robot Simulation Environment <br/> Universidade de Aveiro / IEETA 

## Information

CiberRato Robot Simulation Environment simulates the movement
of robots inside a labyrinth.  Robots objective is to go from their
starting position to beacon area and then return to their start position.

## Contents

* simulator -           The cb_webots_pc library source code
* webots -              The cb_webots_pc worlds and controllers

## Install

The source code was compiled with gcc/g++ - Gnu Project C/C++ Compiler
(gcc version  9.3.0) using the Qt libraries (release 5.12.8) on Ubuntu 20.04.

To use the tools in this branch, you should first install webots simulator [https://cyberbotics.com/]


It is required to have the development version of gcc/g++, cmake, Qt libraries
release 5.x installed in the system prior to compilation.
On Ubuntu 24.04 run the following:
```bash
sudo apt-get install build-essential cmake qtmultimedia5-dev git
```

Then in the repository base dir, execute:
```bash
git switch cb_webots_pc
mkdir build
cd build
cmake ..
make -j4
```

To run the webots, execute:
```bash
webots
```

In the webots File menu select "Open World" and open the world at `ciberRatoTools\webots\worlds\rmi_challenge_1.wbt`

To compile the supervisor controller, click the ![wheel](wheel.png) button over the Edit window.

All set! To start the simulation click ![start](start.png) button.


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

 Copyright (C) 2001-2024 Universidade de Aveiro


