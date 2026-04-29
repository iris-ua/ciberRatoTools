***************************************************************************
*  Java Interface for CiberRato Robot Simulation Environment              *
*  Concurso Micro-Rato 2008 - Modalidade Ciber-Rato University of Aveiro  *
***************************************************************************

### Information

  This package contains a Java Interface to the CiberRato Robot Simulation
  Environment. The CiberRato 2008 tools (simulator and viewer) should be 
  installed in order to use this interface
  CiberRato Robot Simulation Environment simulates the movement
  of robots inside a labirinth.  Robots objective is to go from their
  starting position to beacon area and then return to their start position.
  This environment is used in
  Concurso Micro-Rato 2006 - Modalidade Ciber-Rato - University of Aveiro
  http://microrato.ua.pt/

### Version Info

    CiberRato Java Interface v0.1.4 - December 29, 2007

### Contents

    jClient.java     Source of basic reactive agent using the Java interface
    ciberIF/         ciberIF package that implements the CiberRato Java Interface
    README.txt	     This README file
    doc/             Documentation in HTML 

### How to use

    * Compilation

    To compile the jClient execute the following command from the command line:

       javac jClient.java

    the file JClient.java is created

    * Execution

    To execute the jClient, first start the Ciber-Rato simulator from the
    CiberTools and the execute the command:

       java jClient -host localhost -pos 1 -robname jClient
       or
       java jClient -host localhost:3000 -pos 1 -robname jClient


    * Change Behaviour

    Edit jClient.java, recompile and reexecute.

### Operating System and Compiler

    This interface has been testes in Windows and Linux using jre 1.4.2
    and jre 1.5.0

### More Information

    Please consult:
        http://microrato.ua.pt/

    or Please Contact :
        Nuno Lau
        University of Aveiro
        email: lau@det.ua.pt

Copyright (c) 2008 -  Ciber-Rato Team


