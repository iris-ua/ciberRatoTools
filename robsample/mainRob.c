/* mainRob.C
*
* Basic Robot Agent
* Very simple version for demonstration
*
* For more information about the CiberRato Robot Simulator
* please see http://microrato.ua.pt/ or contact us.
*/

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <float.h>

#include "RobSock.h"

#include "robfunc.h"
#include "controller.h"

#define true 1
#define false 0

float getLinePos(bool line[]);

int main(int argc, char *argv[])
{
  char host[100]="localhost";
  char rob_name[20]="robsample";
  float lPow,rPow;
  int rob_id = 1;
  char lmap[CELLROWS*2-1][CELLCOLS*2-1]; // in this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to lmap[i*2][j*2].
  // to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of lmap[i*2+1][j*2] is space or not

  float posOverLine;                    /* Position over the line */
  float velSetPoint=0.15;                /* Velocity set point */
  bool moving = false;

  printf( " Sample Robot\n Copyright (C) 2001-2022 Universidade de Aveiro\n" );

  /* processing arguments */
  while (argc > 2) /* every option has a value, thus argc must be 1, 3, 5, ... */
  {
    if (strcmp(argv[1], "--host") == 0 || strcmp(argv[1], "-h") == 0)
    {
      strncpy(host, argv[2], 99);
      host[99]='\0';
    }
    else if (strcmp(argv[1], "--robname") == 0 || strcmp(argv[1], "-r") == 0)
    {
      strncpy(rob_name, argv[2], 19);
      rob_name[19]='\0';
    }
    else if (strcmp(argv[1], "--pos") == 0 || strcmp(argv[1], "-p") == 0)
    {
      if(sscanf(argv[2], "%d", &rob_id)!=1)
      argc=0; /* error message will be printed */
    }
    else if (strcmp(argv[1], "--map") == 0 || strcmp(argv[1], "-m") == 0)
    {
      ReadMap(argv[2],lmap);
      for(int r=CELLROWS*2-2; r>=0; r--) {
        for(int c=0; c<CELLCOLS*2-1; c++) {
          printf("%c", lmap[r][c]);
        }
        printf("\n");
      }
    }
    else
    {
      break; /* the while */
    }
    argc -= 2;
    argv += 2;
  }

  if (argc != 1)
  {
    fprintf(stderr, "Bad number of parameters\n"
    "SYNOPSIS: mainRob [--host hostname] [--robname robotname] [--pos posnumber]\n");

    return 1;
  }

  /* Connect Robot to simulator */
  if(InitRobot(rob_name, rob_id, host)==-1)
  {
    printf( "%s Failed to connect\n", rob_name);
    exit(1);
  }
  printf( "%s Connected\n", rob_name );


  /* Open a file for writing values */
  FILE *fd=fopen("output.txt","w+");


  /* Main cycle */
  while(1)
  {

    /* Reading next values from Sensors */
    ReadSensors();

    bool line[7];

    GetLineSensor(line);

    for (int i = 0; i < N_LINE_ELEMENTS; i++) {
      fprintf(stderr, "%s", line[i] ? "1" : "0");
    }
    fprintf(stderr, "\n");

    if(GetFinished()) /* Simulator has received Finish() or Robot Removed */
    {
      printf(  "%s Exiting\n", rob_name );
      break;
    }

    /* Test if reached end of labyrinth */
    if(GetX() > 27.0){
      printf("Reached end of maze! Terminating...\n");
      break;
    }


    if(GetStopButton() && GetTime()>0 ){
      printf("Stop button pressed! Terminating...\n");
      break;
    }

    /* Compute position over the line */
    posOverLine = getLinePos(line);

    /* Compute left and right command for steering the robot, using the
       active controller */
    float u = controller(activeController, 0, posOverLine);
    /* controller() may return a NaN.
    lPow and rPow are updated only if the returned value is a valid number. */
    if (!isnan(u)) {
      lPow = velSetPoint - u;
      rPow = velSetPoint + u;
    }

    /* Act on the system */
    DriveMotors(lPow,rPow);

    /* Test if experiment has started */
    if( GetStartButton()){
      moving = true;
    }

    /* Store values in history file */
    if(moving){
      /* Current time*/
      fprintf(fd, "%u\t",GetTime());
      /* Actual position error = Current Y Position - Line Position */
      fprintf(fd, "%4.5f\t", GetY() - 10);
      /* Sensor readings */
      fprintf(fd, "%4.5f\t", posOverLine);
      /* Motor commands */
      fprintf(fd, "%4.5f\t", lPow);
      fprintf(fd, "%4.5f\t", rPow);
      fprintf(fd, "%4.5f\n", u);
      
    }

  }

  fclose(fd);
  return 1;
}

/**
 * Compute estimate of robot position over the line.
 *
 * Computes the posiition of the line relative to the line sensor. The line
 * sensor contains 7 detectors, line[0] to line[6], being line[3] the central
 * detector.
 * The result is scaled for the robot dimensions, using the value defined in
 * LINESENSORELDIST in cbsensor.h. Current value is 0.08
 *
 * If no line is detected under the sensor, returns NaN (result is 0/0)
 *
 * \param line   array (size N_LINE_ELEMENTS) containing the line sensor readings
 * \returns Position of the line relative to the center of the sensor.
 *
 */
float getLinePos(bool line[])
{
  float posOverLine=0;
  int nActiveSensors=0;

  /* Read sensors */
  for (int i = 0; i < N_LINE_ELEMENTS; i++) {
    if(line[i]){
      posOverLine += (float) (i-3);
      nActiveSensors++;
    }
  }
  /* Compute the position and scale the measure for the distance between
   * sensors.
   */
  posOverLine = 0.08*posOverLine/nActiveSensors;

  return posOverLine;

}
