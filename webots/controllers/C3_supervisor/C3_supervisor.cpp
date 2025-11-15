/*
 * world_builder C++ supervisor controller.
 */
#include <webots/Supervisor.hpp>
#include <iostream>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <signal.h>



#include <QFile>
#include <QXmlSimpleReader>
#include <QXmlInputSource>

#include "cblabhandler.h"
#include "cbgraph.h"

// Define the default maximum duration of the simulation in seconds
// This value can be overridden by the envoronment variable MAX_TIME
// Example: export MAX_TIME=60
double MAX_TIME_SECONDS = 450.0;

#define M_PI 3.14159265358979323846

#ifndef MAXINT
#define MAXINT (0x7fffffff)
#endif

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

pid_t get_sibling_pid() {
    FILE *fp;
    char buffer[1024];
    char command[1024];
    pid_t sibling_pid=0;

    // Construct the command string:
    // pgrep -P [my_parent_pid] | grep -v [my_pid]
    snprintf(command, sizeof(command), 
             "pgrep -P %d | grep -v %d", getppid(), getpid());

    // Open a pipe to execute the command
    fp = popen(command, "r");
    if (fp == NULL) {
        perror("popen failed");
        return 0;
    }

    // Read the output line by line (each line contains one PID)
    while (fgets(buffer, sizeof(buffer), fp) != NULL) {
        // Convert the string PID to an integer
        sibling_pid = atoi(buffer);
    }

    // Close the pipe
    pclose(fp);
    
    return sibling_pid;
}

bool is_pid_running(int pid) {
    if (pid <= 0) return false;
    // Sending signal 0 to a PID checks if the process is alive without killing it.
    // Returns 0 if process exists, -1 if it doesn't.
    return (kill(pid, 0) == 0);
}
struct cell_t getInitialCell(cbLab *lab)
{
    struct cell_t cell;

    cell.x = lab->Target(0)->Center().x / PATHCUBESIZE;
    cell.y = lab->Target(0)->Center().y / PATHCUBESIZE;
    std::cout << "initial cell: x=" << cell.x << " y=" << cell.y << std::endl;

    return cell;
}


void determine_lab_map_centered_on_robot_initial_pos(cbLab *lab)
{
    int cells_width = int(lab->Width() / PATHCUBESIZE + 0.5);
    int cells_height = int(lab->Height() / PATHCUBESIZE + 0.5);
    int lmap_width = cells_width * 4 - 1;
    int lmap_height = cells_height * 4 - 1;
    char lmap[lmap_height][lmap_width];

    memset(lmap, ' ', sizeof(lmap));

    struct cell_t initCell = getInitialCell(lab);

    // find vertical walls
    for (int cy = 0; cy < cells_height; cy++)
    {
        for (int cx = 0; cx < cells_width; cx++)
        {
            if (!lab->reachable(cbPoint((cx + 0.5) * PATHCUBESIZE, (cy + 0.5) * PATHCUBESIZE),
                                cbPoint((cx + 1.5) * PATHCUBESIZE, (cy + 0.5) * PATHCUBESIZE)))
            {
                // fprintf(stderr,"not reachable %d %d -> %d %d, lmap %d %d\n", cy, cx, cy, cx+1,
                //          (cy-initCell.y)*2+lmap_height/2,(cx-initCell.x)*2+1+lmap_width/2);
                lmap[(cy - initCell.y) * 2 + lmap_height / 2][(cx - initCell.x) * 2 + 1 + lmap_width / 2] = '|';
            }
        }
    }
    // vertical left wall
    for (int cy = 0; cy < cells_height; cy++)
    {
        int cx = 0;
        if (!lab->reachable(cbPoint((cx + 0.5) * PATHCUBESIZE, (cy + 0.5) * PATHCUBESIZE),
                            cbPoint((cx - 0.5) * PATHCUBESIZE, (cy + 0.5) * PATHCUBESIZE)))
        {
            lmap[(cy - initCell.y) * 2 + lmap_height / 2][(cx - initCell.x) * 2 - 1 + lmap_width / 2] = '|';
        }
    }

    // find horizontal walls
    for (int cy = 0; cy < cells_height; cy++)
    {
        for (int cx = 0; cx < lmap_width; cx++)
        {
            if (!lab->reachable(cbPoint((cx + 0.5) * PATHCUBESIZE, (cy + 0.5) * PATHCUBESIZE),
                                cbPoint((cx + 0.5) * PATHCUBESIZE, (cy + 1.5) * PATHCUBESIZE)))
            {
                lmap[(cy - initCell.y) * 2 + 1 + lmap_height / 2][(cx - initCell.x) * 2 + lmap_width / 2] = '-';
            }
        }
    }
    // horizontal lower wall
    int cy = 0;
    for (int cx = 0; cx < lmap_width; cx++)
    {
        if (!lab->reachable(cbPoint((cx + 0.5) * PATHCUBESIZE, (cy + 0.5) * PATHCUBESIZE),
                            cbPoint((cx + 0.5) * PATHCUBESIZE, (cy - 0.5) * PATHCUBESIZE)))
        {
            lmap[(cy - initCell.y) * 2 - 1 + lmap_height / 2][(cx - initCell.x) * 2 + lmap_width / 2] = '-';
        }
    }

    // mark initial pos as known
    lmap[lmap_height / 2][lmap_width / 2] = 'X';

    // mark reachable positions
    int changes = 1;
    while (changes)
    {
        changes = 0;
        for (int ly = 1; ly < lmap_height - 1; ly++)
        {
            for (int lx = 1; lx < lmap_width; lx++)
            {
                if (lx % 2 == 0 && ly % 2 == 0)
                    continue;
                if (lmap[ly][lx] == ' ' &&
                    (lmap[ly][lx + 1] == 'X' || lmap[ly][lx - 1] == 'X' || lmap[ly + 1][lx] == 'X' || lmap[ly - 1][lx] == 'X'))
                {
                    changes = 1;
                    lmap[ly][lx] = 'X';
                }
            }
        }
    }

    // unmark unseen walls
    for (int ly = 1; ly < lmap_height - 1; ly++)
    {
        for (int lx = 1; lx < lmap_width - 1; lx++)
        {
            if (lmap[ly][lx] == '-' && lmap[ly - 1][lx] != 'X' && lmap[ly + 1][lx] != 'X')
            {
                lmap[ly][lx] = ' ';
            }
            if (lmap[ly][lx] == '|' && lmap[ly][lx - 1] != 'X' && lmap[ly][lx + 1] != 'X')
            {
                lmap[ly][lx] = ' ';
            }
        }
    }

    int ly = 0;
    for (int lx = 0; lx < lmap_width; lx++)
    {
        if (lmap[ly][lx] == '-' && lmap[ly + 1][lx] != 'X')
        {
            lmap[ly][lx] = ' ';
        }
    }
    ly = lmap_height - 1;
    for (int lx = 0; lx < lmap_width; lx++)
    {
        if (lmap[ly][lx] == '-' && lmap[ly - 1][lx] != 'X')
        {
            lmap[ly][lx] = ' ';
        }
    }

    int lx = 0;
    for (int ly = 0; ly < lmap_height; ly++)
    {
        if (lmap[ly][lx] == '|' && lmap[ly][lx + 1] != 'X')
        {
            lmap[ly][lx] = ' ';
        }
    }

    lx = lmap_width - 1;
    for (int ly = 0; ly < lmap_height; ly++)
    {
        if (lmap[ly][lx] == '|' && lmap[ly][lx - 1] != 'X')
        {
            lmap[ly][lx] = ' ';
        }
    }

    // mark targets
    for (unsigned int bi = 0; bi < lab->nTargets(); bi++)
    {
        struct cell_t targetCell;
        targetCell.x = lab->Target(bi)->Center().X() / PATHCUBESIZE;
        targetCell.y = lab->Target(bi)->Center().Y() / PATHCUBESIZE;
        lmap[(targetCell.y - initCell.y) * 2 + lmap_height / 2][(targetCell.x - initCell.x) * 2 + lmap_width / 2] = '0' + bi;
    }

    FILE *fp = fopen("planning.out", "w");
    if (fp == NULL)
    {
        fprintf(stderr, "Could not create planning file\n");
    }
    else
    {
        for (int ly = lmap_height - 1; ly >= 0; ly--)
        {
            for (int lx = 0; lx < lmap_width; lx++)
            {
                fprintf(fp, "%c", lmap[ly][lx]);
            }
            fprintf(fp, "\n");
        }
        fclose(fp);
    }

    cbGraph grLab;

    // add horizontal links
    for (float x = PATHCUBESIZE * 0.5; x < lab->Width() - PATHCUBESIZE; x += PATHCUBESIZE)
    {
        for (float y = PATHCUBESIZE * 0.5; y < lab->Height(); y += PATHCUBESIZE)
        {
            cbPoint from(x, y);
            cbPoint to(x + PATHCUBESIZE, y);

            if (lab->reachable(from, to))
            {
                grLab.addLink(cbNode(from, MAXINT, MAXINT), cbNode(to, MAXINT, MAXINT), PATHCUBESIZE);
                grLab.addLink(cbNode(to, MAXINT, MAXINT), cbNode(from, MAXINT, MAXINT), PATHCUBESIZE);
            }
        }
    }

    // add vertical links
    for (float x = PATHCUBESIZE * 0.5; x < lab->Width(); x += PATHCUBESIZE)
    {
        for (float y = PATHCUBESIZE * 0.5; y < lab->Height() - PATHCUBESIZE; y += PATHCUBESIZE)
        {
            cbPoint from(x, y);
            cbPoint to(x, y + PATHCUBESIZE);

            if (lab->reachable(from, to))
            {
                grLab.addLink(cbNode(from, MAXINT, MAXINT), cbNode(to, MAXINT, MAXINT), PATHCUBESIZE);
                grLab.addLink(cbNode(to, MAXINT, MAXINT), cbNode(from, MAXINT, MAXINT), PATHCUBESIZE);
            }
        }
    }

    //// DEBUG
    // cbPoint from(0.225,0.825);
    // cbPoint to  (0.375,0.825);
    // fprintf(stderr, "dist is %f\n", grLab.dist (cbNode(from,MAXINT,MAXINT),
    // cbNode(to   ,MAXINT,MAXINT)));

    // store distances between targets
    std::vector<std::vector<double>> targetDists;
    std::vector<double> aux(lab->nTargets(), MAXINT);
    for (unsigned int bi = 0; bi < lab->nTargets(); bi++)
    {
        targetDists.push_back(aux);
    }
    for (unsigned int bi = 0; bi < lab->nTargets() - 1; bi++)
    {
        for (unsigned int bf = bi + 1; bf < lab->nTargets(); bf++)
        {
            double dist = grLab.dist(cbNode(lab->Target(bi)->Center(), MAXINT, MAXINT),
                                     cbNode(lab->Target(bf)->Center(), MAXINT, MAXINT));
            targetDists[bi][bf] = dist;
            targetDists[bf][bi] = dist;
            // fprintf(stderr,"distance from targets %d -> %d = %f\n", bi, bf, dist);
        }
    }

    // find best path

    std::vector<int> path, bestPath;
    double bestDist = MAXINT;
    for (unsigned int bi = 0; bi < lab->nTargets(); bi++)
    {
        path.push_back(bi);
    }
    path.push_back(0);

    if (path.size() > 2)
    {
        do
        {
            double dist = 0.0;
            for (unsigned int p = 0; p < path.size() - 1; p++)
            {
                dist += targetDists[path[p]][path[p + 1]];
            }
            if (dist < bestDist)
            {
                bestDist = dist;
                bestPath = path;
            }

        } while (std::next_permutation(path.begin() + 1,
                                       path.begin() + lab->nTargets()));

        FILE *fp = fopen("planning.out", "a");
        if (fp == NULL)
        {
            fprintf(stderr, "Could not append planning file\n");
        }
        else
        {
            for (unsigned int p = 0; p < bestPath.size(); p++)
            {
                fprintf(fp, " %d", bestPath[p]);
            }
            fprintf(fp, "\n%f\n", bestDist/PATHCUBESIZE);
        }
        fclose(fp);
    }
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
    offsetX = rand() % 40;
    offsetY = rand() % 19;

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

    // Read lab file and create maze walls in webots
    char lab_filename[1024 * 8] = "C3-lab.xml";
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
    if (epuck_node == NULL)
    {
        std::cerr << "No DEF EPUCK node found in the current world file\n";
        exit(0);
    }
    
    // Reposition robot to first target area of maze
    webots::Field *translationField = epuck_node->getField("translation");
    if (translationField) {
        // Define the new position (e.g., move to X=1.0, Y=2.0)
        double newTranslation[3] = {labHandler->getLab()->Target(0)->Center().x + offsetX * PATHCUBESIZE, 
                                    labHandler->getLab()->Target(0)->Center().y + offsetY * PATHCUBESIZE, 
                                    0.0}; 
        translationField->setSFVec3f(newTranslation);
        //std::cout << "E-puck repositioned." << std::endl;
    }

    // Get the PID of robot controller
    int robot_pid = get_sibling_pid();

    if (robot_pid == 0) {
        fprintf(stderr,"Could not get Robot Controller PID.\n");
        supervisor->simulationQuit(2);
        exit(2);
    }    

    
    // Set custom data
    webots::Field *customDataField = epuck_node->getField("customData");
    
    if (!customDataField) {
        std::cerr << "Error: 'customData' field not found on EPUCK node." << std::endl;
        return 1;
    }
    
    char str_custom_data[4*1024];
    
    sprintf(str_custom_data,"FinalTime %.1f NTargets %d", MAX_TIME_SECONDS, labHandler->getLab()->nTargets());
    
    customDataField->setSFString(str_custom_data);


    determine_lab_map_centered_on_robot_initial_pos(labHandler->getLab());

    // ---
    // 4. MAIN LOOP (Optional)
    // ---

    // The main work is done, but the controller must keep running
    // for the simulation to continue.
    bool finished = false;
    float finalTime = 0.0;
    while (supervisor->step(timeStep) != -1)
    {
        std::string scoreText;
    
        // Get the current simulation time.
        double currentTime = supervisor->getTime();

        // Check if robot controller is alive
        if (robot_pid > 0 && !is_pid_running(robot_pid)) {
            if(! finished) {
                printf("Robot controller finished at time %f.\n", currentTime);
                finalTime = currentTime;
            }

            finished = true;

            scoreText = "GAME OVER at "+ std::to_string(finalTime);

            //supervisor->simulationQuit(0); 
        }
        // Check if the simulation time has exceeded the maximum duration.
        else if (currentTime >= MAX_TIME_SECONDS)
        {
            // The argument 0 indicates a successful exit.
            if (epuck_node) {
                epuck_node->remove();
                epuck_node = NULL;
            }
            scoreText = "GAME OVER";
        }
        else {
            scoreText = "";
        }

        // Display the label
        supervisor->setLabel(0, scoreText, 0.4, 0.01, 0.1, 0xFF0000, 0.0, "Arial");     
    
    }

    delete supervisor;
    return 0;
}
