/*
    This file is part of ciberRatoToolsSrc.

    Copyright (C) 2001-2011 Universidade de Aveiro

    ciberRatoToolsSrc is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    ciberRatoToolsSrc is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef CBROBOT_H
#define CBROBOT_H

#include <iostream>

#include "cbclient.h"
#include "cbactionhandler.h"
#include "cbposition.h"
#include "cbmotor.h"
#include "cbsensor.h"

using std::ostream;

/**
  *@author Nuno Lau, Artur Pereira & Andreia Melo, Miguel Rodrigues
  */

#define ROBOT_RADIUS 0.5
#define NUM_IR_SENSORS 4

class cbSimulator;

const double irSensorDefaultAngles[NUM_IR_SENSORS]={0, M_PI/3, -M_PI/3, M_PI};

/**
 * This class represents the robot state (name, position, score, actions, 
 * sensors, etc) inside the simulator.
 */
class cbRobot : public cbClient
{
    Q_OBJECT    // enable slots and signals
public:
	// enum State and StrState (in cbrobot.cpp) must be compatible!!!
	enum State {STOPPED=0, RUNNING, WAITOTHERS, RETURNING, FINISHED, REMOVED};

	cbRobot(const double irSensorAngles[NUM_IR_SENSORS]);
	virtual ~cbRobot();

    void resetRobot();

	void setName(const char *name);
	const char *Name();
	void setId(unsigned int id);
	unsigned int Id();
	void setIRSensorAngle(unsigned int irId, double irAngle);

	void setSimulator(cbSimulator *s);
	inline cbSimulator *getSimulator() { return simulator; }

	virtual bool readAction(cbRobotAction *);

	void setLeftMotor(double);
	void setRightMotor(double);
    inline void setEndLed(bool l) { recEndLed=true; endLed = l; }
    void setReturningLed(bool l);
    void setVisitingLed(bool l);

    void setSayMessage(QString message);
    QString getSayMessage(void);

    void resetReceivedFlags(void);
    inline bool receivedLeftMotor(void) { return recLeftMotor; }
    inline bool receivedRightMotor(void) { return recRightMotor; }
    inline bool receivedEndLed(void) { return recEndLed; }
    inline bool receivedReturningLed(void) { return recReturningLed; }
    inline bool receivedVisitingLed(void) { return recVisitingLed; }
    inline bool receivedAction(void) { return recReturningLed || recEndLed || recLeftMotor || recRightMotor; }

    void resetRequestedSensors(void);
    void requestSensor(QString sId);

	inline bool endLedOn() { return endLed; }
	inline bool returningLedOn() { return returningLed; }
	inline bool visitingLedOn() { return visitingLed; }

	inline void remove() { removed = true; }
	inline bool isRemoved() { return _state == REMOVED; }
	inline bool hasFinished() { return _state == FINISHED; }
    inline bool active() { return _state == STOPPED || _state == RUNNING || _state == RETURNING; }
	inline bool blocked() { return _state == STOPPED || _state == FINISHED; }
	void changeActiveState();

    /*
    // old code - collisions are now discriminated for score flexibility
    inline void resetCollision() { collision = false; }
	inline void setCollision()   { collision = true; }
	inline bool hasCollide() { return collision; }
    */

    inline void resetCollision() { collisionWall = false; collisionRobot = false; }
    inline void setCollisionWall() { collisionWall = true; }
    inline void setCollisionRobot() { collisionRobot = true; }
    inline bool hasCollideWall() { return collisionWall; }
    inline bool hasCollideRobot() { return collisionRobot; }
    inline bool hasCollide() { return (collisionWall || collisionRobot); }

//	inline void setOnTarget(bool v) { onTarget = v; }
	bool isOnTarget(int &targId);
    int  targetAtPos(void);
    bool visitedTarget(int targId) { return targetVisited[targId];}
    bool visitedAllTargets(void);
    int  nVisitedTargets(void);

	inline cbPoint &Center() { return curPos.Coord(); }
	inline cbPoint &nextCenter() { return nextPos.Coord(); }

	inline double X() { return curPos.X(); }
	inline double Y() { return curPos.Y(); }
	inline double Dir() { return curPos.Direction(); }
	inline double Degrees() { return Dir() * 180 / M_PI; }

	inline cbMotor &LeftMotor() { return leftMotor; }
	inline cbMotor &RightMotor() { return rightMotor; }

    inline cbPosition &position() { return curPos; }
    inline void setPosition(const cbPosition &pos) { curPos = nextPos = pos; }

	inline unsigned int Score() { return score; }
	inline void incrementScore(unsigned int inc) { score += inc; }

	void computeNextPosition();
	void commitNextPosition();

	bool isMovingTowards(cbPoint &);

	void updateScore();
	void updateScoreCompetitive();
	void updateScoreControl();
	void updateScoreLineControl2022();
    void updateScoreLineMappingPlanning2022();
	void updateScoreLineControl2023();


	void updateState();
	void updateStateCompetitive();
	void updateStateControl();
    void updateStateMapping();
    void updateStatePlanning();
    void updateStateLineControl2022();
    void updateStateLineMapping2022();
    void updateStateLinePlanning2022();
    void updateStateLineControl2023();

	void updateSensors();

	virtual void sendSensors();

	unsigned int toXml(char *, unsigned int);

	inline State state() { return _state; }
    const char *curStateAsString();

	void showAllAttributes();
	void Log(ostream &Log, bool withactions=true);
 

signals:

    void robCollisionsChanged(int);
    void robReturnTimeChanged(int);
    void robArrivalTimeChanged(int);
    void robScoreChanged(int);
    void robStateChanged(cbRobot::State);
    void robStateChanged(QString);

protected:
    unsigned int distFromInitScore(void);
    unsigned int distFromInitScoreCompetitive(void);
    unsigned int returnTimeScore(void);
    unsigned int returnTimeScoreCompetitive(void);
    unsigned int arrivalTimeScore(void);

public:
	// global robot parameters
	
	static unsigned int  maxSensorsRequested;
	static bool GPSOn;
	static bool GPSDirOn;
	static bool beaconSensorOn;
	static bool compassSensorOn;
	static bool scoreSensorOn;
	static bool showActions;
    static bool showMeasures;
    static bool ignoreOthers;

    static int returnTimePenalty;
    static int arrivalTimePenalty;
    static int collisionWallPenalty;
    static int collisionRobotPenalty;
    static int targetReward;
    static int homeReward;

protected:  // class data members

	char *name;
	unsigned int id;

    unsigned int score, scorePenalties,
            arrivalTime,         // time until turned on returning Led
            startReturningTime,  // instant of last retuning Led turn on
            returningTime;       // time from startReturningTime until finish
	double distAtArrival;
	double distGridToBeacon;
	double distHomeToTarget;
	vector <bool>   targetVisited;

	State _state;		// current state of robot
	State _unstoppedState;	// state of robot before stopped

	cbMotor leftMotor, rightMotor;	// the motors
    bool returningLed;
    bool visitingLed;
	bool endLed;

    QString sayMessage;

    bool recLeftMotor, recRightMotor, recVisitingLed, recReturningLed, recEndLed;

	bool removed;	// indicates that robot has been removed

	cbGroundSensor *groundSensor;
	cbCompassSensor *compassSensor;
	cbCollisionSensor *collisionSensor;
	vector<cbBeaconSensor *> beaconSensors;
	cbIRSensor *irSensors[NUM_IR_SENSORS];
	cbGPSSensor *GPSSensor;
	cbLineSensor *lineSensor;

	vector <cbSensor *> sensors;  // all robot sensors should have their pointer added to this vector


	cbSimulator *simulator;

	cbPosition curPos;	// current position of robot
	cbPosition nextPos;	// next position of robot
	double vel;	        // pretended linear velocity of robot 
    //bool collision;		// indicates that next position results in a collision
    bool collisionWall;     // indicates that next position results in a collision with wall
    bool collisionRobot;    // indicates that next position results in a collision with another robot
	bool collisionPrevCycle;// indicates that in previous cycle robot has collided
	unsigned int collisionCount;	// number of collisions

	unsigned int nSensorsRequested;

	cbActionHandler handler;

        struct cell_t {
               int x,y;
        } controlCellPath[1024], newCell;

       struct cell_t getRobotCell();
        
        int nCellPath;
        int scoreControl;
        int nextPathInd;

};

class cbRobotBin : public cbRobot
{
public:
    cbRobotBin() : cbRobot(irSensorDefaultAngles) {}
    virtual ~cbRobotBin() {}
	virtual bool readAction(cbRobotAction *);
	virtual void sendSensors();
	virtual bool Reply(QHostAddress &addr, unsigned short &port, cbParameters *param);
	virtual bool Refuse(QHostAddress &addr, unsigned short &port);

};

#endif
