/*
 * world_builder C++ supervisor controller.
 */
#include <webots/Supervisor.hpp>
#include <iostream>
#include <string>
#include <math.h>
#include <stdlib.h>

#include <QFile>
#include <QXmlSimpleReader>
#include <QXmlInputSource>

#include "cblabhandler.h"

// Define the default maximum duration of the simulation in seconds
// This value can be overridden by the envoronment variable MAX_TIME
// Example: export MAX_TIME=60
double MAX_TIME_SECONDS = 300.0;

#define M_PI 3.14159265358979323846

struct cell_t
{
    int x, y;
} controlCellPath[1024], newCell;

int nCellPath = 0;
int nextPathInd = 0;
int scoreControl = 0;

// Get the e-puck node by DEF name (e.g., "EPUCK")
webots::Node *epuck_node;

#define PATHCUBESIZE (0.15)

struct cell_t getInitialCell(cbLab *lab)
{
    struct cell_t cell;

    cell.x = lab->Target(0)->Center().x / PATHCUBESIZE;
    cell.y = lab->Target(0)->Center().y / PATHCUBESIZE;
    std::cout << "initial cell: x=" << cell.x << " y=" << cell.y << std::endl;

    return cell;
}

int main(int argc, char **argv)
{
    // ---
    // 1. INITIALIZATION
    // ---
    webots::Supervisor *supervisor = new webots::Supervisor();
    int timeStep = (int)supervisor->getBasicTimeStep();

    // random offsets to avoid GPS informing on exact position in maze
    int offsetX = 0;
    int offsetY = 0;

    srand(time(0));
    offsetX = 0;
    offsetY = 0;


    // ---
    // 1a. GET MAXIMUM SIMULATION TIME
    // ---

    char *maxTimeEnv = getenv("MAX_TIME");
    if (maxTimeEnv != NULL)
    {
        char *pend;
        double maxTime = strtod(maxTimeEnv, &pend);
        if(*pend=='\0' && pend != maxTimeEnv) {
             MAX_TIME_SECONDS = maxTime;
        }
        else {
            fprintf(stderr,"Could not get max_time from MAX_TIME value (%s)\n", maxTimeEnv);
        }
    }
    std::cerr << "Maximum Simulation Time: " << MAX_TIME_SECONDS << " s\n";

    // ---
    // 2. GET SCENE TREE NODES
    // ---

    // Get the root node of the scene tree
    webots::Node *root_node = supervisor->getRoot();
    // Get the 'children' field of the root node, where we'll add new objects.
    webots::Field *children_field = root_node->getField("children");

    // ---
    // 3. DEFINE AND CREATE NEW OBJECTS
    // ---

    char lab_filename[1024 * 8] = "C1-lab.xml";
    QXmlInputSource *source;

    QFile srcFile(lab_filename);

    if (!srcFile.exists())
    {
        std::cerr << "Could not open " << lab_filename << "\n";
        exit(0);
    }
    if ((source = new QXmlInputSource(&srcFile)) == 0)
    {
        std::cerr << "Fail sourcing lab file\n";
        exit(0);
    }

    cbLabHandler *labHandler = new cbLabHandler;
    labHandler->setChildrenField(children_field);
    labHandler->setOffsets(offsetX, offsetY);

    QXmlSimpleReader xmlParser;
    xmlParser.setContentHandler(labHandler);

    xmlParser.setErrorHandler(labHandler);

    if (!xmlParser.parse(source))
    {
        std::cerr << "Error parsing lab file\n";
        exit(0);
    }

    epuck_node = supervisor->getFromDef("EPUCK");

    webots::Field *translationField = epuck_node->getField("translation");
    if (translationField) {
        // Define the new position (e.g., move to X=1.0, Y=2.0)
        double newTranslation[3] = {labHandler->getLab()->Target(0)->Center().x + offsetX * PATHCUBESIZE, 
                                    labHandler->getLab()->Target(0)->Center().y + offsetY * PATHCUBESIZE, 
                                    0.0}; 
        translationField->setSFVec3f(newTranslation);
        std::cout << "E-puck repositioned to " << newTranslation[0] << " " << newTranslation[1] <<std::endl;
    }

    // ---
    // 4. MAIN LOOP (Optional)
    // ---

    // The main work is done, but the controller must keep running
    // for the simulation to continue.
    while (supervisor->step(timeStep) != -1)
    {
        // Get the current simulation time.
        double currentTime = supervisor->getTime();

        std::string scoreText;

        // Check if the simulation time has exceeded the maximum duration.
        if (currentTime >= MAX_TIME_SECONDS)
        {
            // The argument 0 indicates a successful exit.
            scoreText = "Time's up!";
            supervisor->simulationSetMode(webots::Supervisor::SIMULATION_MODE_PAUSE);
        }

        // Display the label
        supervisor->setLabel(0, scoreText, 0.6, 0.01, 0.1, 0xFF0000, 0.0, "Arial");
    }

    delete supervisor;
    return 0;
}
