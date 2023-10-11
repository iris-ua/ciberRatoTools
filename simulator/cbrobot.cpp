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

#include "cbrobot.h"
#include "cbsensor.h"
#include "cbtarget.h"
#include "cbsimulator.h"
#include "cblab.h"
#include "cbgraph.h"

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <QTime>

#include <QTextEdit>
#include <QMessageBox>
#include <QCoreApplication>

using std::cerr;
using std::cout;

//#define DEBUG_ROBOT

// enum State and StrState (in cbrobot.cpp) must be compatible!!!
static const char *StrState[] =
{
    "Stopped", "Running", "Waiting", "Returning", "Finished", "Removed"
};

cbRobot::cbRobot(const double irSensorAngle[]) : cbClient()
{
	int i;

	collisionSensor = new cbCollisionSensor(this, "Collision");
	sensors.push_back(collisionSensor);

	collisionSensor->setRequestable(false);

	collisionSensor->setRequestable(cbCollisionSensor::sensorRequestable); 
	collisionSensor->setFifoLatency(cbCollisionSensor::sensorLatency);

	groundSensor    = new cbGroundSensor(this, "Ground");
	sensors.push_back(groundSensor);

	groundSensor->setRequestable(cbGroundSensor::sensorRequestable); 
	groundSensor->setFifoLatency(cbGroundSensor::sensorLatency);

	compassSensor   = new cbCompassSensor(this, "Compass");
	sensors.push_back(compassSensor);

	compassSensor->setRequestable(cbCompassSensor::sensorRequestable); 
	compassSensor->setFifoLatency(cbCompassSensor::sensorLatency);

	/* init IR sensor relative positions */
	for(i=0; i < NUM_IR_SENSORS ; i++) {
	   irSensors[i] = new cbIRSensor(this, QString("IRSensor") + QString::number(i));
	   sensors.push_back(irSensors[i]);

	   irSensors[i]->setRequestable(cbIRSensor::sensorRequestable); 
	   irSensors[i]->setFifoLatency(cbIRSensor::sensorLatency);

	   irSensors[i]->setPosition(ROBOT_RADIUS, irSensorAngle[i]);
	}

	//GPSOn = true;
	//beaconSensorOn = false;
	//compassSensorOn = true;
	//GPSDirOn = false;
	GPSSensor   = new cbGPSSensor(this, "GPS");
	sensors.push_back(GPSSensor);

    GPSSensor->setRequestable(cbGPSSensor::sensorRequestable);
	GPSSensor->setFifoLatency(cbGPSSensor::sensorLatency);

	lineSensor   = new cbLineSensor(this, "LineSensor");
	sensors.push_back(lineSensor);

    lineSensor->setRequestable(cbLineSensor::sensorRequestable);
	lineSensor->setFifoLatency(cbLineSensor::sensorLatency);
    lineSensor->setPosition(0.438); // Adjusted for 7 elements and 0.08 interelemdist

    resetRobot();

}

void cbRobot::resetRobot()
{
	score          = 0;
	scorePenalties = 0;
    scoreControl   = 0;
	arrivalTime=0;
	startReturningTime=0;
	returningTime=0;
	distAtArrival=0.0;
	distGridToBeacon=0.0;
	distHomeToTarget=0.0;

    //collision = false;
    collisionWall = false;
    collisionRobot = false;
	collisionPrevCycle = false;
	collisionCount = 0;

	_state = STOPPED;
	_unstoppedState = RUNNING;
	removed=false;

	endLed = false;
    returningLed=false;
    visitingLed=false;

	resetReceivedFlags();

	vel=0.0;

    nextPathInd = 1;  // used for CONTROL scoring
}

cbRobot::~cbRobot()
{
}

void cbRobot::setName(const char *nm)
{
	name = strdup(nm);
}

const char *cbRobot::Name()
{
	return name;
}

void cbRobot::setId(unsigned int theId)
{
	id = theId;
}

unsigned int cbRobot::Id()
{
	return id;
} 

const char *cbRobot::curStateAsString()
{
    return StrState[state()];
}

void cbRobot::setSimulator(cbSimulator *s)
{
	simulator = s; 

    connect(this, &QUdpSocket::readyRead, simulator, &cbSimulator::RobotActions);

	// Set the number of beacon sensors
	beaconSensors.resize(simulator->Lab()->nBeacons());
	for(unsigned int b=0; b<beaconSensors.size();b++) {
		beaconSensors[b] = new cbBeaconSensor(this, QString("Beacon")+QString::number(b));
		sensors.push_back(beaconSensors[b]);

		beaconSensors[b]->setBeaconToFollow(b);
        beaconSensors[b]->setFifoLatency(cbBeaconSensor::sensorLatency);
        beaconSensors[b]->setRequestable(cbBeaconSensor::sensorRequestable);

	}

	//set the number of visited target flags and initialize
	targetVisited.resize(simulator->Lab()->nTargets(), false);

    if(simulator->getScoring()==0) {
        scorePenalties = 100 + 100 * simulator->Lab()->nTargets(); //100 for each target area + 100 for returning COMPETITIVE
    }
    else {
        scorePenalties = 0;  //CONTROL
        scoreControl = 0;
    }
    //scorePenalties = targetReward * (simulator->Lab()->nTargets() - 1) + homeReward; // COOPERATIVE
	
}

void cbRobot::setIRSensorAngle(unsigned int irId, double irAngle)
{
	irSensors[irId]->setPosition(ROBOT_RADIUS, irAngle);
}

void cbRobot::setLeftMotor(double p)
{
    recLeftMotor=true;

	leftMotor.setInPower(p);
}

void cbRobot::setRightMotor(double p)
{
    recRightMotor=true;

	rightMotor.setInPower(p);
}

void cbRobot::setReturningLed(bool l)
{ 
    recReturningLed=true;

    returningLed=l;
}

void cbRobot::setVisitingLed(bool l)
{ 
    recVisitingLed=true;

    visitingLed=l;
}

void cbRobot::setSayMessage(QString msg)
{ 
    if(msg.length()<=100)
        sayMessage=msg;
}

QString cbRobot::getSayMessage(void)
{ 
    return sayMessage;
}

void cbRobot::resetReceivedFlags(void)
{ 
    recLeftMotor=recRightMotor=recVisitingLed=recReturningLed=recEndLed=false;
    sayMessage="";
}

void cbRobot::resetRequestedSensors(void)
{ 
	unsigned int s;
	for (s = 0; s < sensors.size(); s++ )
		sensors[s]->requested = false;
	nSensorsRequested = 0;
}

void cbRobot::requestSensor(QString sensorStrId)
{ 
	unsigned int s;

	if (nSensorsRequested == maxSensorsRequested) {
		cerr << "Robot " << id << " trying to request more sensors than permitted\n";
		return;	
	}

	for (s = 0; s < sensors.size(); s++ ) {
		if ( sensors[s]->getStrId() == sensorStrId ) {
            sensors[s]->requested = true;
            nSensorsRequested++;
		    break;
		}
	}

    if(s==sensors.size())
        cerr << "Robot " << id << " requested unrecognized sensor (" << sensorStrId.toStdString() << ")\n";
}

void cbRobot::changeActiveState()
{
	if (_state == RUNNING) _state = STOPPED;
	else if (_state == STOPPED) _state = RUNNING;
}

/*!
	Try to read an action from the incoming socket.
	If one, fill in the area pointed to by action and
	return TRUE.
	Otherwise return FALSE.
*/
bool cbRobot::readAction(cbRobotAction *action)
{
	/* look for an incoming message */
    char xmlBuff[1024];
    int xmlSize;

    if (!hasPendingDatagrams())
        return false;

    if ((xmlSize=readDatagram(xmlBuff, 1023)) < 0)
    {
        cerr << "Error reading from Robot Socket - " << errorString().toStdString();
        //cerr << "Error no. " << error() << " reading from robot socket\n";
        //if(showActions)
        //   simulator->GUI()->appendMessage((QString("Error no. ") + error()
        //                                          + " reading from robot socket");
		return false;
	}
	else xmlBuff[xmlSize]='\0';

#ifdef DEBUG_ROBOT
	cerr << "cbRobot: " << xmlBuff << "\n";
#endif

    if (showActions)
        simulator->GUI()->writeOnBoard(QString(name) + " : " + xmlBuff, (int) id, 1);

	/* parse xml message */
    parser.setContentHandler(&handler);
    source.setData(QByteArray(xmlBuff));
	if (!parser.parse(source))
	{
        cerr << "cbRobot::Fail parsing xml action message: \"" << xmlBuff << "\"\n";
        simulator->GUI()->appendMessage( "cbRobot: Fail parsing xml action message:" , true);
        simulator->GUI()->appendMessage( QString(" \"")+xmlBuff+"\"" , true);

		return false;
	}
	
	*action = handler.parsedAction();
	return true;
}

/*!
	Compute next position for robot.
*/
void cbRobot::computeNextPosition()
{
	if (_state == REMOVED) return;

	vel = (rightMotor.outPower() + leftMotor.outPower()) / 2.0;
	double rot = (rightMotor.lastOutPower() - leftMotor.lastOutPower()) / (2.0 * ROBOT_RADIUS); // outPower should be called only once per cycle

	double theta = curPos.Direction();
    /* a direc��o angular do robot
     * � dada em rela��o ao eixo dos xx. */
	double x = curPos.X() + vel * cos(theta);
	double y = curPos.Y() + vel * sin(theta);
	theta += rot;
	if (theta > M_PI) theta -= (2.0*M_PI);
	else if (theta < -M_PI) theta += (2.0*M_PI);

	nextPos.set(x,y,theta);
}

/*!
	Commit next position for robot.
	If robot is a colliding robot only angular position
	is changed.
	Otherwise next position becomes current.
*/
void cbRobot::commitNextPosition()
{
	if(blocked()) return;   // stopped or finished or removed 
    //if (collision)
    if (hasCollide())
	{
        curPos.setRadianDirection(nextPos.radianDirection());
	}
	else
	{
		curPos = nextPos;
	}
}

bool cbRobot::isOnTarget(int &targId) 
{ 
	for(unsigned int t=0; t<simulator->Lab()->nTargets(); t++) {
		if(simulator->Lab()->Target(t)->contains(curPos.Coord(),ROBOT_RADIUS)) {
			targId=t;
			return true;
		}
	}
	return false;
}

int cbRobot::targetAtPos(void) 
{ 
	for(unsigned int t=0; t<simulator->Lab()->nTargets(); t++) {
		if(simulator->Lab()->Target(t)->contains(curPos.Coord(),ROBOT_RADIUS)) {
			return t;
		}
	}
	return -1;
}

bool cbRobot::visitedAllTargets(void) 
{
	for(unsigned int t=0; t<simulator->Lab()->nTargets(); t++) {
		if(!targetVisited[t]) return false;
	}
	return true;
}

int cbRobot::nVisitedTargets(void) 
{
	int count=0;
	for(unsigned int t=0; t<simulator->Lab()->nTargets(); t++) {
		if(targetVisited[t]) count++;
	}
	return count;
}

/*!
	Determine if robot is moving towards the given point.
	Return TRUE if so, and FALSE otherwise.
*/
bool cbRobot::isMovingTowards(cbPoint &p)
{
	cbPoint &c = Center();
	/* return false if robot doesn't move */
	if (c == p) return false;

	/* determine angular deviation */
	double alpha = atan2(p.y-c.y, p.x-c.x);
	alpha = fabs(alpha - curPos.radianDirection());

	/* return TRUE if deviation lower than PI/2 */
	if(vel>0)
	    return alpha < M_PI/2.0 || alpha > 3.0*M_PI/2.0;
	else if(vel<0)
	    return alpha > M_PI/2.0 && alpha < 3.0*M_PI/2.0;
	else {
	   return false; //vel=0;
	}
}

/*!
	Update robot state.
*/
void cbRobot::updateState()
{
    State prevState = _state;

    switch(_state) {
        case RUNNING:
            if(removed) _state = REMOVED;

            else if (simulator->state() == cbSimulator::STOPPED) {
                _unstoppedState=_state;
                _state=STOPPED;
            }

            else if(visitingLedOn()) {
                     if(targetAtPos()==0) {
                        if(simulator->allRobotsVisitedOrVisitingTarget(0)
                           && simulator->allRobotsOnTarget(0)) {
			    setReturningLed(true);
		            _state = RETURNING;
                        }
                        else {
		            _state = WAITOTHERS;
                        }
		     }
                     else _state = FINISHED;
            }
            else if (endLed) {
	        _state = FINISHED;
	    }

            break;
        case WAITOTHERS:
            if(removed) _state = REMOVED;

            else if (simulator->state() == cbSimulator::STOPPED) {
                _unstoppedState=_state;
                _state=STOPPED;
            }

            else if(simulator->allRobotsVisitedOrVisitingTarget(0)
                    && simulator->allRobotsOnTarget(0)) {
			    setReturningLed(true);
		            _state = RETURNING;
            }
            else if (endLed) {
	        _state = FINISHED;
	    }

            break;
	case RETURNING:
           if(removed) _state = REMOVED;

           else if (simulator->state() == cbSimulator::STOPPED) {
                _unstoppedState=_state;
                _state=STOPPED;
           }

           else if(endLed) _state = FINISHED;

	   break;
        case STOPPED:
            if(removed) _state = REMOVED;
            else if(simulator->state() != cbSimulator::STOPPED)
                      _state=_unstoppedState;
            break;
        case FINISHED:
            if(removed) _state = REMOVED;
	default:
	    break;
    }

    if (prevState != _state)
    {
        emit robStateChanged(_state);
        emit robStateChanged(QString(curStateAsString()));
    }
}

/*!
	Determines the score component determined by distance to initial position.
*/
unsigned int cbRobot::distFromInitScore(void)
{
     // add current position to grAux
     simulator->grAux->addFinalPoint(id,curPos.Coord());
     // determine distance score, id determines initial position
     //return (unsigned int)(100*simulator->grAux->dist(id)/distAtArrival);
     return (unsigned int)(arrivalTimePenalty*simulator->grAux->dist(id)/distAtArrival);
}

unsigned int cbRobot::distFromInitScoreCompetitive(void)
{
     // add current position to grAux
     simulator->grAux->addFinalPoint(id,curPos.Coord());
     // determine distance score, id determines initial position
     float distToStart = simulator->grAux->dist(id);
     //fprintf(stderr,"distFromInitScore distToStart %f distAtArrival %f\n", distToStart, distAtArrival);
     return (unsigned int)(100*distToStart/distAtArrival);
     //return (unsigned int)(arrivalTimePenalty*simulator->grAux->dist(id)/distAtArrival);
}

/*!
	Determines the score component determined by the time taken to return
*/

//#define RETURNTIMEPENALTY 25

unsigned int cbRobot::returnTimeScore(void)
{
	if(returningTime<distHomeToTarget/0.15)
		return 0;
	
    return (int)(returningTime-((int)(distHomeToTarget/0.15)))/returnTimePenalty;
}

unsigned int cbRobot::returnTimeScoreCompetitive(void)
{
	if(returningTime<distAtArrival/0.15)
		return 0;
	
    return (int)(returningTime-((int)(distAtArrival/0.15)))/returnTimePenalty;
}

//#define ARRIVALTIMEPENALTY 100

unsigned int cbRobot::arrivalTimeScore(void)
{
	return 0;   //HACK - CiberRato 2008 will not consider this score factor

	if(arrivalTime<distGridToBeacon/0.15)
		return 0;
	
    return (int)(arrivalTime-((int)(distGridToBeacon/0.15)))/arrivalTimePenalty;
}

/*!
	Update robot score. Called before updateState!
*/

//#define COLLISION_PENALTY   2
//#define NOTFINISH_PENALTY  15
//#define BASESCORE         0

/*!
	Update robot state.
*/
void cbRobot::updateStateCompetitive()
{
    switch(_state) {
        case RUNNING:
            if(removed) _state = REMOVED;

            else if (simulator->state() == cbSimulator::STOPPED) {
                _unstoppedState=_state;
                _state=STOPPED;
            }

            else if (endLed) {
	        _state = FINISHED;
	    }

            else if (returningLed) {
                    //fprintf(stderr,"RETURNINGLEDON targetAtPos %d visitall %d\n",targetAtPos(), visitedAllTargets());
                    if(targetAtPos()!=-1 && visitedAllTargets()) {
		        _state = RETURNING;
		    }
                    else _state = FINISHED;
	    }

            break;
        case RETURNING:
            if(removed) _state = REMOVED;

            else if (simulator->state() == cbSimulator::STOPPED) {
                _unstoppedState=_state;
                _state=STOPPED;
            }

            else if(endLed) _state = FINISHED;

            break;
        case STOPPED:
            if(removed) _state = REMOVED;
            else if(simulator->state() != cbSimulator::STOPPED)
                      _state=_unstoppedState;
            break;
        case FINISHED:
            if(removed) _state = REMOVED;
	default:
	    break;
    }
}



#define COLLISION_PENALTY   5
#define NOTFINISH_PENALTY  15
#define BASESCORE         100

void cbRobot::updateScoreCompetitive()
{
    if (isRemoved() || hasFinished()) return;
    
    if (hasCollide() && !collisionPrevCycle)
    {
        //DEBUG
	//simulator->grAux->addFinalPoint(id,curPos.Coord());
        //double distCol=simulator->grAux->dist(id);
	//cerr << simulator->curTime() << ": R" << id << " distCol=" << distCol <<"\n";
        scorePenalties += (collisionWallPenalty * hasCollideWall()) + (collisionRobotPenalty * hasCollideRobot());
	collisionCount++;

        emit robCollisionsChanged((int) collisionCount);
    }
    collisionPrevCycle=hasCollide();

    switch(_state) {
	    case RUNNING:
		 {
		    //score = BASESCORE + scorePenalties;  // BASESCORE must be equal to 
		                                         // distFromInitScore+returnTimeScore if robot finishes just after signaling return
		    score = scorePenalties;  // BASESCORE must be equal to 

		    if(endLedOn() && targetAtPos()>=0 && visitedAllTargets()) { // robot finishing at Beacon
                        simulator->grAux->addFinalPoint(id,curPos.Coord());
                        distAtArrival=simulator->grAux->dist(id);
			//cerr << simulator->curTime()<< ": R" << id << " distAtArrival=" << distAtArrival << "\n";
			arrivalTime=simulator->curTime();
			startReturningTime = simulator->curTime();
			returningTime=0;
                        //score=scorePenalties + distFromInitScoreCompetitive()+returnTimeScoreCompetitive();
                        score=scorePenalties + distFromInitScoreCompetitive();
		    }
		    else if(returningLed && targetAtPos()>=0 && visitedAllTargets()) { // robot starting to return
                        simulator->grAux->addFinalPoint(id,curPos.Coord());
                        distAtArrival=simulator->grAux->dist(id);
			//cerr << simulator->curTime()<< ": R" << id << " distAtArrival=" << distAtArrival << "\n";
			startReturningTime = simulator->curTime();
			returningTime=0;

			scorePenalties -= 100;
                        //score=scorePenalties + distFromInitScoreCompetitive()+returnTimeScoreCompetitive();
                        score=scorePenalties + distFromInitScoreCompetitive();
                        //fprintf(stderr,"RUNRETURNLED pen %d dist %d time %d\n", scorePenalties, distFromInitScoreCompetitive(), returnTimeScoreCompetitive());
		    }
		    else if(visitingLedOn()) {
			    int t=targetAtPos();
			    if(t>=0) {
				    if(!targetVisited[t]) {
				        targetVisited[t] = true;
				        scorePenalties -= 100;
			            }
			    }
			    else scorePenalties+=5; 
		            score = BASESCORE + scorePenalties;
                            //fprintf(stderr,"RUNVISITLED pen %d base %d\n", scorePenalties, BASESCORE);
		    }
		    if(nVisitedTargets()==0)
		        arrivalTime = simulator->curTime(); 
		    break;
		 }
            case RETURNING:
		 {
		    if(simulator->curTime()==simulator->simTime() 
				    && !endLed) // check if robot has just turned endLed on
			    scorePenalties+=NOTFINISH_PENALTY;

		    returningTime=simulator->curTime() - startReturningTime;
		    //score = scorePenalties + distFromInitScoreCompetitive() + returnTimeScoreCompetitive();
		    score = scorePenalties + distFromInitScoreCompetitive() ;
                        //fprintf(stderr,"RETURN pen %d dist %d time %d\n", scorePenalties, distFromInitScoreCompetitive(), returnTimeScoreCompetitive());

		    if(returningLed && receivedReturningLed() && targetAtPos()>=0 && visitedAllTargets()) { // robot restarting to return
                        simulator->grAux->addFinalPoint(id,curPos.Coord());
                        distAtArrival=simulator->grAux->dist(id);
			//cerr << simulator->curTime()<< ": R" << id << " distAtArrival=" << distAtArrival << "\n";
			startReturningTime = simulator->curTime();
			returningTime=0;
                        //score=scorePenalties + distFromInitScoreCompetitive() + returnTimeScoreCompetitive();
                        score=scorePenalties + distFromInitScoreCompetitive() ;
                        //fprintf(stderr,"RESTART pen %d dist %d time %d\n", scorePenalties, distFromInitScoreCompetitive(), returnTimeScoreCompetitive());
		    }
		    else if(visitingLedOn()) {
			    int t=targetAtPos();
			    if(t>=0) {
				    if(!targetVisited[t]) {
				        targetVisited[t] = true;
				        scorePenalties -= 100;
			            }
			    }
			    else scorePenalties+=5;
		            //score = scorePenalties + distFromInitScoreCompetitive() + returnTimeScoreCompetitive();
		            score = scorePenalties + distFromInitScoreCompetitive() ;
                            //fprintf(stderr,"VISITON pen %d dist %d time %d\n", scorePenalties, distFromInitScoreCompetitive(), returnTimeScoreCompetitive());
		    }
		    break;
		 }
	   default:
		    break;
    }
    emit robArrivalTimeChanged((int) arrivalTime);
    emit robReturnTimeChanged((int) returningTime);
    emit robScoreChanged((int) score);
}

cbRobot::cell_t cbRobot::getRobotCell()
{
     struct cell_t cell;
     cell.x = curPos.X()/2;
     cell.y = curPos.Y()/2;

    return cell;
}

void cbRobot::updateStateControl()
{
    switch(_state) {
        case RUNNING:
            if(removed) _state = REMOVED;

            else if (simulator->state() == cbSimulator::STOPPED) {
                _unstoppedState=_state;
                _state=STOPPED;
            }

            else if (endLed) {
	        _state = FINISHED;
	    }

            break;
        case STOPPED:
            if(removed) _state = REMOVED;
            else if(simulator->state() != cbSimulator::STOPPED)
                      _state=_unstoppedState;
            // determine path first time RUNNING
            if(_state==RUNNING && simulator->curTime()==0) {
                  controlCellPath[0] = getRobotCell();
                  struct cell_t newCell = controlCellPath[0];
                  newCell.x++;
                  nCellPath=1;
                  int dir = 0;
                  while(newCell.x!=controlCellPath[0].x || newCell.y!=controlCellPath[0].y) {
                      int d;
                      struct cell_t cell = newCell;
                      controlCellPath[nCellPath]=cell;
                      nCellPath++;
                      for(d=-90; d<=90; d+=90) { 
                         if(simulator->Lab()->reachable(cbPoint(newCell.x*2.0+1,newCell.y*2.0+1),
                                                     cbPoint(newCell.x*2+1+cos((d+dir)*M_PI/180.0)*2.0,newCell.y*2+1+sin((d+dir)*M_PI/180.0)*2.0))){
                             newCell.x = round(cell.x + cos((dir+d)*M_PI/180.0));
                             newCell.y = round(cell.y + sin((dir+d)*M_PI/180.0));
                             dir = (dir + d + 360) % 360;

                             break;
                         } 
                      }
                      if(d==180) {
                            fprintf(stderr, "Lab has no Loop! Does not fit for this challenge!\n"); // TODO: add Graphical Window Warning
                            QMessageBox::critical(0,"Error", "Lab has no Loop! Does not fit for this challenge!\n", QMessageBox::Ok, Qt::NoButton, Qt::NoButton);

                            exit(1);
                      }
                  }

                  //debug
                  //for(int i=0; i<nCellPath; i++) {
                  //       printf("pathcell %d %d\n",controlCellPath[i].x, controlCellPath[i].y);
                  //}
            }
            break;
        case FINISHED:
            if(removed) _state = REMOVED;
	default:
	    break;
    }
}

void cbRobot::updateStateMapping()
{
    switch(_state) {
        case RUNNING:
            if(removed) _state = REMOVED;

            else if (simulator->state() == cbSimulator::STOPPED) {
                _unstoppedState=_state;
                _state=STOPPED;
            }

            else if (endLed) {
	        _state = FINISHED;
	    }

            break;
        case STOPPED:
            if(removed) _state = REMOVED;
            else if(simulator->state() != cbSimulator::STOPPED)
                      _state=_unstoppedState;
            // determine lab map centered on robot initial pos
            if(_state==RUNNING && simulator->curTime()==0) {
                  int  cells_width  = int(simulator->Lab()->Width())/2;
                  int  cells_height = int(simulator->Lab()->Height())/2;
                  int  lmap_width   = cells_width*4-1;
                  int  lmap_height  = cells_height*4-1;
                  char lmap[lmap_height][lmap_width];

                  memset(lmap,' ',sizeof(lmap));

                  //debug

                  struct cell_t initCell = getRobotCell();;

                  //fprintf(stderr,"initCell %d %d\n", initCell.x, initCell.y);

                  // find vertical walls
                  for(int cy = 0; cy < cells_height; cy++) {
                       for(int cx = 0; cx < cells_width; cx++) {
                            if(!simulator->Lab()->reachable(cbPoint(cx*2.0+1.0,cy*2.0+1.0), cbPoint((cx+1)*2.0+1.0,cy*2.0+1.0))){
                                 //fprintf(stderr,"not reachable %d %d -> %d %d, lmap %d %d\n", cy, cx, cy, cx+1, 
                                 //         (cy-initCell.y)*2+lmap_height/2,(cx-initCell.x)*2+1+lmap_width/2);
                                 lmap[(cy-initCell.y)*2+lmap_height/2][(cx-initCell.x)*2+1+lmap_width/2] = '|';
                            }
                       } 
                  }
                  // vertical left wall
                  for(int cy = 0; cy < cells_height; cy++) {
                            int cx=0;
                            if(!simulator->Lab()->reachable(cbPoint(cx*2.0+1.0,cy*2.0+1.0), cbPoint((cx-1)*2.0+1.0,cy*2.0+1.0))){
                                 lmap[(cy-initCell.y)*2+lmap_height/2][(cx-initCell.x)*2-1+lmap_width/2] = '|';
                            }
                  } 

                  // find horizontal walls
                  for(int cy = 0; cy < cells_height; cy++) {
                       for(int cx = 0; cx < lmap_width; cx++) {
                            if(!simulator->Lab()->reachable(cbPoint(cx*2.0+1.0,cy*2.0+1.0), cbPoint(cx*2.0+1.0,(cy+1)*2.0+1.0))){
                                 lmap[(cy-initCell.y)*2+1+lmap_height/2][(cx-initCell.x)*2+lmap_width/2] = '-';
                            }
                       } 
                  }
                  // horizontal lower wall
                  int cy=0;
                  for(int cx = 0; cx < lmap_width; cx++) {
                            if(!simulator->Lab()->reachable(cbPoint(cx*2.0+1.0,cy*2.0+1.0), cbPoint(cx*2.0+1.0,(cy-1)*2.0+1.0))){
                                 lmap[(cy-initCell.y)*2-1+lmap_height/2][(cx-initCell.x)*2+lmap_width/2] = '-';
                            }
                  } 


                  //mark initial pos as known
                  lmap[lmap_height/2][lmap_width/2] = 'X';

                  // mark reachable positions
                  int changes=1;
                  while(changes) {
                     changes = 0;
                     for(int ly = 1;ly<lmap_height-1; ly++) {
                       for(int lx = 1; lx < lmap_width; lx++) {
                            if (lx%2 == 0 && ly%2==0) continue;
                            if(lmap[ly][lx] == ' ' &&
                               (lmap[ly][lx+1] == 'X' 
                                || lmap[ly][lx-1] == 'X' 
                                || lmap[ly+1][lx] == 'X' 
                                || lmap[ly-1][lx] == 'X')
                               ) {
                               changes = 1;
                               lmap[ly][lx] = 'X';
                            }
                       } 
                     }
                  }


                  //unmark unseen walls
                  for(int ly = 1;ly<lmap_height-1; ly++) {
                      for(int lx = 1; lx < lmap_width-1; lx++) {
                            if(lmap[ly][lx] == '-' 
                               && lmap[ly-1][lx] != 'X' 
                               && lmap[ly+1][lx] != 'X' 
                               ) {
                               lmap[ly][lx] = ' ';
                            }
                            if(lmap[ly][lx] == '|' 
                               && lmap[ly][lx-1] != 'X' 
                               && lmap[ly][lx+1] != 'X' 
                               ) {
                               lmap[ly][lx] = ' ';
                            }
                       } 
                  }

                  int ly=0;
                  for(int lx = 0; lx < lmap_width; lx++) {
                            if(lmap[ly][lx] == '-' 
                               && lmap[ly+1][lx] != 'X' 
                               ) {
                               lmap[ly][lx] = ' ';
                            }
                  }
                  ly=lmap_height-1;
                  for(int lx = 0; lx < lmap_width; lx++) {
                            if(lmap[ly][lx] == '-' 
                               && lmap[ly-1][lx] != 'X' 
                               ) {
                               lmap[ly][lx] = ' ';
                            }
                  }

                  int lx=0;
                  for(int ly = 0;ly<lmap_height; ly++) {
                            if(lmap[ly][lx] == '|' 
                               && lmap[ly][lx+1] != 'X' 
                               ) {
                               lmap[ly][lx] = ' ';
                            }
                  }

                  lx = lmap_width-1;
                  for(int ly = 0;ly<lmap_height; ly++) {
                            if(lmap[ly][lx] == '|' 
                               && lmap[ly][lx-1] != 'X' 
                               ) {
                               lmap[ly][lx] = ' ';
                            }
                  }

                  //mark initial pos as I
                  lmap[lmap_height/2][lmap_width/2] = 'I';


                  FILE *fp=fopen("mapping.out","w");
                  if(fp==NULL) {
                       fprintf(stderr,"Could not create mapping file\n");
                  }
                  else {
                     for(int ly = lmap_height-1; ly>=0; ly--) {
                        for(int lx = 0; lx < lmap_width; lx++) {
                              fprintf(fp,"%c",lmap[ly][lx]);
                        } 
                        fprintf(fp,"\n");
                     }
                     fclose(fp);
                  }
                  
            }
            break;
        case FINISHED:
            if(removed) _state = REMOVED;
	default:
	    break;
    }
}

#ifndef MAXINT
#define MAXINT (0x7fffffff)
#endif

//TODO merge with updateStateMapping (too much common code)
void cbRobot::updateStatePlanning()
{
    switch(_state) {
        case RUNNING:
            if(removed) _state = REMOVED;

            else if (simulator->state() == cbSimulator::STOPPED) {
                _unstoppedState=_state;
                _state=STOPPED;
            }

            else if (endLed) {
	        _state = FINISHED;
	    }

            break;
        case STOPPED:
            if(removed) _state = REMOVED;
            else if(simulator->state() != cbSimulator::STOPPED)
                      _state=_unstoppedState;
            // determine lab map centered on robot initial pos
            if(_state==RUNNING && simulator->curTime()==0) {
                  int  cells_width  = int(simulator->Lab()->Width())/2;
                  int  cells_height = int(simulator->Lab()->Height())/2;
                  int  lmap_width   = cells_width*4-1;
                  int  lmap_height  = cells_height*4-1;
                  char lmap[lmap_height][lmap_width];

                  memset(lmap,' ',sizeof(lmap));

                  //debug

                  struct cell_t initCell = getRobotCell();;

                  //fprintf(stderr,"initCell %d %d\n", initCell.x, initCell.y);

                  // find vertical walls
                  for(int cy = 0; cy < cells_height; cy++) {
                       for(int cx = 0; cx < cells_width; cx++) {
                            if(!simulator->Lab()->reachable(cbPoint(cx*2.0+1.0,cy*2.0+1.0), cbPoint((cx+1)*2.0+1.0,cy*2.0+1.0))){
                                 //fprintf(stderr,"not reachable %d %d -> %d %d, lmap %d %d\n", cy, cx, cy, cx+1, 
                                 //         (cy-initCell.y)*2+lmap_height/2,(cx-initCell.x)*2+1+lmap_width/2);
                                 lmap[(cy-initCell.y)*2+lmap_height/2][(cx-initCell.x)*2+1+lmap_width/2] = '|';
                            }
                       } 
                  }
                  // vertical left wall
                  for(int cy = 0; cy < cells_height; cy++) {
                            int cx=0;
                            if(!simulator->Lab()->reachable(cbPoint(cx*2.0+1.0,cy*2.0+1.0), cbPoint((cx-1)*2.0+1.0,cy*2.0+1.0))){
                                 lmap[(cy-initCell.y)*2+lmap_height/2][(cx-initCell.x)*2-1+lmap_width/2] = '|';
                            }
                  } 

                  // find horizontal walls
                  for(int cy = 0; cy < cells_height; cy++) {
                       for(int cx = 0; cx < lmap_width; cx++) {
                            if(!simulator->Lab()->reachable(cbPoint(cx*2.0+1.0,cy*2.0+1.0), cbPoint(cx*2.0+1.0,(cy+1)*2.0+1.0))){
                                 lmap[(cy-initCell.y)*2+1+lmap_height/2][(cx-initCell.x)*2+lmap_width/2] = '-';
                            }
                       } 
                  }
                  // horizontal lower wall
                  int cy=0;
                  for(int cx = 0; cx < lmap_width; cx++) {
                            if(!simulator->Lab()->reachable(cbPoint(cx*2.0+1.0,cy*2.0+1.0), cbPoint(cx*2.0+1.0,(cy-1)*2.0+1.0))){
                                 lmap[(cy-initCell.y)*2-1+lmap_height/2][(cx-initCell.x)*2+lmap_width/2] = '-';
                            }
                  } 


                  //mark initial pos as known
                  lmap[lmap_height/2][lmap_width/2] = 'X';

                  // mark reachable positions
                  int changes=1;
                  while(changes) {
                     changes = 0;
                     for(int ly = 1;ly<lmap_height-1; ly++) {
                       for(int lx = 1; lx < lmap_width; lx++) {
                            if (lx%2 == 0 && ly%2==0) continue;
                            if(lmap[ly][lx] == ' ' &&
                               (lmap[ly][lx+1] == 'X' 
                                || lmap[ly][lx-1] == 'X' 
                                || lmap[ly+1][lx] == 'X' 
                                || lmap[ly-1][lx] == 'X')
                               ) {
                               changes = 1;
                               lmap[ly][lx] = 'X';
                            }
                       } 
                     }
                  }


                  //unmark unseen walls
                  for(int ly = 1;ly<lmap_height-1; ly++) {
                      for(int lx = 1; lx < lmap_width-1; lx++) {
                            if(lmap[ly][lx] == '-' 
                               && lmap[ly-1][lx] != 'X' 
                               && lmap[ly+1][lx] != 'X' 
                               ) {
                               lmap[ly][lx] = ' ';
                            }
                            if(lmap[ly][lx] == '|' 
                               && lmap[ly][lx-1] != 'X' 
                               && lmap[ly][lx+1] != 'X' 
                               ) {
                               lmap[ly][lx] = ' ';
                            }
                       } 
                  }

                  int ly=0;
                  for(int lx = 0; lx < lmap_width; lx++) {
                            if(lmap[ly][lx] == '-' 
                               && lmap[ly+1][lx] != 'X' 
                               ) {
                               lmap[ly][lx] = ' ';
                            }
                  }
                  ly=lmap_height-1;
                  for(int lx = 0; lx < lmap_width; lx++) {
                            if(lmap[ly][lx] == '-' 
                               && lmap[ly-1][lx] != 'X' 
                               ) {
                               lmap[ly][lx] = ' ';
                            }
                  }

                  int lx=0;
                  for(int ly = 0;ly<lmap_height; ly++) {
                            if(lmap[ly][lx] == '|' 
                               && lmap[ly][lx+1] != 'X' 
                               ) {
                               lmap[ly][lx] = ' ';
                            }
                  }

                  lx = lmap_width-1;
                  for(int ly = 0;ly<lmap_height; ly++) {
                            if(lmap[ly][lx] == '|' 
                               && lmap[ly][lx-1] != 'X' 
                               ) {
                               lmap[ly][lx] = ' ';
                            }
                  }

                  //mark targets
                  for(unsigned int bi=0; bi < simulator->Lab()->nBeacons(); bi++) {
                      struct cell_t targetCell;
                      targetCell.x = simulator->Lab()->Target(bi)->Center().X()/2.0;
                      targetCell.y = simulator->Lab()->Target(bi)->Center().Y()/2.0;
                      lmap[(targetCell.y-initCell.y)*2+lmap_height/2][(targetCell.x-initCell.x)*2+lmap_width/2] = '0'+bi;
                  }


                  FILE *fp=fopen("planning.out","w");
                  if(fp==NULL) {
                       fprintf(stderr,"Could not create planning file\n");
                  }
                  else {
                     for(int ly = lmap_height-1; ly>=0; ly--) {
                        for(int lx = 0; lx < lmap_width; lx++) {
                              fprintf(fp,"%c",lmap[ly][lx]);
                        } 
                        fprintf(fp,"\n");
                     }
                     fclose(fp);
                  }

                  cbGraph grLab;

                  // add nodes
                  /*for(float x = 1.0; x < simulator->Lab()->Width(); x+=2.0) {
                      for(float y = 1.0; y < simulator->Lab()->Height(); y+=2.0) {
                          cbPoint cellCenter(x,y);
                          grLab.addNode(cbNode(cellCenter,MAXINT,MAXINT));
                      }
                  }
                  */

                  // add horizontal links
                  for(float x = 1.0; x < simulator->Lab()->Width()-2.0; x+=2.0) {
                      for(float y = 1.0; y < simulator->Lab()->Height(); y+=2.0) {
                          cbPoint from(x,y);
                          cbPoint to  (x+2.0,y);

                          if(simulator->Lab()->reachable(from, to)){
                              grLab.addLink(cbNode(from,MAXINT,MAXINT), cbNode(to,MAXINT,MAXINT),   2.0);
                              grLab.addLink(cbNode(to,MAXINT,MAXINT),   cbNode(from,MAXINT,MAXINT), 2.0);
                          }
                      }
                  }
                        
                  // add vertical links
                  for(float x = 1.0; x < simulator->Lab()->Width(); x+=2.0) {
                      for(float y = 1.0; y < simulator->Lab()->Height()-2.0; y+=2.0) {
                          cbPoint from(x,y);
                          cbPoint to  (x,y+2.0);

                          if(simulator->Lab()->reachable(from, to)){
                              grLab.addLink(cbNode(from,MAXINT,MAXINT), cbNode(to,MAXINT,MAXINT),   2.0);
                              grLab.addLink(cbNode(to,MAXINT,MAXINT),   cbNode(from,MAXINT,MAXINT), 2.0);
                          }
                      }
                  }

                  //// DEBUG  
                  //cbPoint from(5.0,7.0);
                  //cbPoint to  (9.0,13.0);
                  //fprintf(stderr, "dist is %f\n", grLab.dist (cbNode(from,MAXINT,MAXINT),
                  //                                            cbNode(to   ,MAXINT,MAXINT)));


                  // store distances between beacons
                  std::vector<std::vector<double> > beaconDists;
                  std::vector<double> aux(simulator->Lab()->nBeacons(), MAXINT);  
                  for(unsigned int bi=0; bi < simulator->Lab()->nBeacons(); bi++) {
                      beaconDists.push_back(aux);
                  }
                  for(unsigned int bi=0; bi < simulator->Lab()->nBeacons()-1; bi++) {
                      for(unsigned int bf=bi+1; bf < simulator->Lab()->nBeacons(); bf++) {
                          double dist=grLab.dist(cbNode(simulator->Lab()->Beacon(bi)->Center(), MAXINT, MAXINT),
                                                 cbNode(simulator->Lab()->Beacon(bf)->Center(), MAXINT, MAXINT));
                          beaconDists[bi][bf]=dist;
                          beaconDists[bf][bi]=dist;
                          //fprintf(stderr,"distance from beacons %d -> %d = %f\n", bi, bf, dist);
                      }
                  }

                  // find best path

                  std::vector<int> path, bestPath;
                  double bestDist=MAXINT;
                  for(unsigned int bi=0; bi < simulator->Lab()->nBeacons(); bi++) {
                      path.push_back(bi);
                  }
                  path.push_back(0);

                  if(path.size()>2) {
                      do {
                          double dist=0.0;
                          for(unsigned int p=0; p<path.size()-1; p++) {
                              dist+=beaconDists[path[p]][path[p+1]];
                          }
                          if (dist<bestDist) {
                              bestDist = dist;
                              bestPath = path;
                          }

                      } while(std::next_permutation(path.begin()+1, 
                                                    path.begin()+simulator->Lab()->nBeacons()));
                      
                      FILE *fp=fopen("planning.out","a");
                      if(fp==NULL) {
                       fprintf(stderr,"Could not append planning file\n");
                      }
                      else {
                          for(unsigned int p=0; p<bestPath.size(); p++) {
                              fprintf(fp," %d", bestPath[p]);
                          }
                          fprintf(fp,"\n%f\n",bestDist);
                     }
                     fclose(fp);
                  }
            }
            break;
        case FINISHED:
            if(removed) _state = REMOVED;
	default:
	    break;
    }
}

void cbRobot::updateStateLineControl2022()
{
    switch(_state) {
        case RUNNING:
            if(removed) _state = REMOVED;

            else if (simulator->state() == cbSimulator::STOPPED) {
                _unstoppedState=_state;
                _state=STOPPED;
            }

            else if (endLed) {
	        _state = FINISHED;
	    }

            break;
        case STOPPED:
            if(removed) _state = REMOVED;
            else if(simulator->state() != cbSimulator::STOPPED)
                      _state=_unstoppedState;
            // determine path first time RUNNING
            if(_state==RUNNING && simulator->curTime()==0) {
                  controlCellPath[0].x = curPos.X()+0.5;   // THESE CELLS are 1x1 and centered on the possible line positions
                  controlCellPath[0].y = curPos.Y()+0.5;   // this is different from cell concept used in updateStateControl (2021)
                  struct cell_t newCell = controlCellPath[0];
                  newCell.x++;
                  nCellPath=1;
                  int dir = 0;
                  while(newCell.x!=controlCellPath[0].x || newCell.y!=controlCellPath[0].y) {
                      int d;
                      struct cell_t cell = newCell;
                      controlCellPath[nCellPath]=cell;
                      nCellPath++;
                      for(d=-90; d<=90; d+=90) { 
                         if(simulator->Lab()->isInside(cbPoint(newCell.x+cos((d+dir)*M_PI/180.0)*0.5,newCell.y+sin((d+dir)*M_PI/180.0)*0.5))){
                             newCell.x = round(cell.x + cos((dir+d)*M_PI/180.0));
                             newCell.y = round(cell.y + sin((dir+d)*M_PI/180.0));
                             dir = (dir + d + 360) % 360;

                             break;
                         } 
                      }
                      if(d==180) {
                            fprintf(stderr, "Lab has no Loop! Does not fit for this challenge!\n");
                            QMessageBox::critical(0,"Error", "Lab has no Loop! Does not fit for this challenge!\n", QMessageBox::Ok, Qt::NoButton, Qt::NoButton);

                            exit(1);
                      }
                  }

                  //debug
                  //for(int i=0; i<nCellPath; i++) {
                  //       printf("pathcell %d %d\n",controlCellPath[i].x, controlCellPath[i].y);
                  //}
            }
            break;
        case FINISHED:
            if(removed) _state = REMOVED;
	default:
	    break;
    }
}

void cbRobot::updateStateLineControl2023()
{
    switch(_state) {
        case RUNNING:
            if(removed) _state = REMOVED;

            else if (simulator->state() == cbSimulator::STOPPED) {
                _unstoppedState=_state;
                _state=STOPPED;
            }

            else if (endLed) {
	        _state = FINISHED;
	    }

            break;
        case STOPPED:
            if(removed) _state = REMOVED;
            else if(simulator->state() != cbSimulator::STOPPED)
                      _state=_unstoppedState;
            // determine path first time RUNNING
            if(_state==RUNNING && simulator->curTime()==0) {
                  controlCellPath[0].x = curPos.X()*2.0+0.5;   // THESE CELLS are 0.5x0.5 and centered on the possible line positions
                  controlCellPath[0].y = curPos.Y()*2.0+0.5;   // this is different from cell concept used in updateStateControl2022
                                    // values of 'x' and 'y' fields in controlCellPath and newCell do NOT REPRESENT the actual coordinates but their doubles
                  struct cell_t newCell = controlCellPath[0];
                  newCell.x++;
                  nCellPath=1;
                  int dir = 0;
                  while(newCell.x!=controlCellPath[0].x || newCell.y!=controlCellPath[0].y) {
                      int d;
                      struct cell_t cell = newCell;
                      controlCellPath[nCellPath]=cell;
                      //fprintf(stderr,"pathcell %5d %4.1f %4.1f\n",nCellPath, controlCellPath[nCellPath].x*0.5, controlCellPath[nCellPath].y*0.5);
                      nCellPath++;
                      if(nCellPath > 500) exit(1);
                      for(d=-135; d<=135; d+=45) { 
                         if(simulator->Lab()->isInside(cbPoint(newCell.x*0.5+cos((d+dir)*M_PI/180.0)*0.2,newCell.y*0.5+sin((d+dir)*M_PI/180.0)*0.2))){
                            newCell.x = round(cell.x + cos((dir+d)*M_PI/180.0));
                            newCell.y = round(cell.y + sin((dir+d)*M_PI/180.0));
                            dir = (dir + d + 360) % 360;

                            break;
                         } 
                      }
                      if(d > 135) {
                            fprintf(stderr, "Lab has no Loop! Does not fit for this challenge!\n");
                            QMessageBox::critical(0,"Error", "Lab has no Loop! Does not fit for this challenge!\n", QMessageBox::Ok, Qt::NoButton, Qt::NoButton);

                            exit(1);
                      }
                  }

                  //debug
                  //for(int i=0; i<nCellPath; i++) {
                  //       fprintf(stderr,"pathcell %4.1f %4.1f\n",controlCellPath[i].x*0.5, controlCellPath[i].y*0.5);
                  //}
            }
            break;
        case FINISHED:
            if(removed) _state = REMOVED;
	default:
	    break;
    }
}

void cbRobot::updateStateLineMapping2022()
{
    switch(_state) {
        case RUNNING:
            if(removed) _state = REMOVED;

            else if (simulator->state() == cbSimulator::STOPPED) {
                _unstoppedState=_state;
                _state=STOPPED;
            }

            else if (endLed) {
	        _state = FINISHED;
	    }

            break;
        case STOPPED:
            if(removed) _state = REMOVED;
            else if(simulator->state() != cbSimulator::STOPPED)
                      _state=_unstoppedState;
            // determine lab map centered on robot initial pos
            if(_state==RUNNING && simulator->curTime()==0) {
                  int  cells_width  = int(simulator->Lab()->Width())/2;
                  int  cells_height = int(simulator->Lab()->Height())/2;
                  int  lmap_width   = (cells_width-2)*4+1;
                  int  lmap_height  = (cells_height-2)*4+1;
                  char lmap[lmap_height][lmap_width];

                  memset(lmap,' ',sizeof(lmap));

                  //debug

                  struct cell_t initCell = getRobotCell();;

                  //fprintf(stderr,"initCell %d %d\n", initCell.x, initCell.y);

                  // find vertical lines
                  for(int cy = 1; cy < cells_height-1; cy++) {
                       for(int cx = 1; cx < cells_width; cx++) {
                            if(simulator->Lab()->isInside(cbPoint(cx*2.0,cy*2.0+0.5))){
                                 //fprintf(stderr,"not reachable %d %d -> %d %d, lmap %d %d\n", cy, cx, cy, cx+1, 
                                 //         (cy-initCell.y)*2+lmap_height/2,(cx-initCell.x)*2+1+lmap_width/2);
                                 lmap[(cy-initCell.y)*2+lmap_height/2+1][(cx-initCell.x)*2+lmap_width/2] = '|';
                            }
                       } 
                  }

                  // find horizontal lines
                  for(int cy = 1; cy < cells_height; cy++) {
                       for(int cx = 1; cx < lmap_width-1; cx++) {
                            if(simulator->Lab()->isInside(cbPoint(cx*2.0+0.5,cy*2.0))){
                                 lmap[(cy-initCell.y)*2+lmap_height/2][(cx-initCell.x)*2+1+lmap_width/2] = '-';
                            }
                       } 
                  }

                  // find diagonal lines '/'
                  for(int cy = 1; cy < cells_height-1; cy++) {
                       for(int cx = 1; cx < lmap_width-1; cx++) {
                            if(simulator->Lab()->isInside(cbPoint(cx*2.0+0.5,cy*2.0+0.5))){
                                lmap[(cy-initCell.y)*2+1+lmap_height/2][(cx-initCell.x)*2+1+lmap_width/2] = '/';
                            }
                       } 
                  }

                  // find diagonal lines '\'
                  for(int cy = 2; cy < cells_height; cy++) {
                       for(int cx = 1; cx < lmap_width-1; cx++) {
                            if(simulator->Lab()->isInside(cbPoint(cx*2.0+0.5,cy*2.0-0.5))){
                                 lmap[(cy-initCell.y)*2-1+lmap_height/2][(cx-initCell.x)*2+1+lmap_width/2] = '\\';
                            }
                       } 
                  }

                  //mark initial pos as I
                  lmap[lmap_height/2][lmap_width/2] = 'I';


                  FILE *fp=fopen("mapping.out","w");
                  if(fp==NULL) {
                       fprintf(stderr,"Could not create mapping file\n");
                  }
                  else {
                     for(int ly = lmap_height-1; ly>=0; ly--) {
                        for(int lx = 0; lx < lmap_width; lx++) {
                              fprintf(fp,"%c",lmap[ly][lx]);
                        } 
                        fprintf(fp,"\n");
                     }
                     fclose(fp);
                  }
                  
            }
            break;
        case FINISHED:
            if(removed) _state = REMOVED;
	default:
	    break;
    }
}

void cbRobot::updateStateLinePlanning2022()
{
    switch(_state) {
        case RUNNING:
            if(removed) _state = REMOVED;

            else if (simulator->state() == cbSimulator::STOPPED) {
                _unstoppedState=_state;
                _state=STOPPED;
            }

            else if (endLed) {
	        _state = FINISHED;
	    }

            break;
        case STOPPED:
            if(removed) _state = REMOVED;
            else if(simulator->state() != cbSimulator::STOPPED)
                      _state=_unstoppedState;
            // determine lab map centered on robot initial pos
            if(_state==RUNNING && simulator->curTime()==0) {
                  int  cells_width  = int(simulator->Lab()->Width())/2;
                  int  cells_height = int(simulator->Lab()->Height())/2;
                  int  lmap_width   = (cells_width-2)*4+1;
                  int  lmap_height  = (cells_height-2)*4+1;
                  char lmap[lmap_height][lmap_width];

                  memset(lmap,' ',sizeof(lmap));

                  //debug

                  struct cell_t initCell = getRobotCell();;

                  //fprintf(stderr,"initCell %d %d\n", initCell.x, initCell.y);

                  // find vertical lines
                  for(int cy = 1; cy < cells_height-1; cy++) {
                       for(int cx = 1; cx < cells_width; cx++) {
                            if(simulator->Lab()->isInside(cbPoint(cx*2.0,cy*2.0+0.5))){
                                 //fprintf(stderr,"not reachable %d %d -> %d %d, lmap %d %d\n", cy, cx, cy, cx+1, 
                                 //         (cy-initCell.y)*2+lmap_height/2,(cx-initCell.x)*2+1+lmap_width/2);
                                 lmap[(cy-initCell.y)*2+lmap_height/2+1][(cx-initCell.x)*2+lmap_width/2] = '|';
                            }
                       } 
                  }

                  // find horizontal lines
                  for(int cy = 1; cy < cells_height; cy++) {
                       for(int cx = 1; cx < lmap_width-1; cx++) {
                            if(simulator->Lab()->isInside(cbPoint(cx*2.0+0.5,cy*2.0))){
                                 lmap[(cy-initCell.y)*2+lmap_height/2][(cx-initCell.x)*2+1+lmap_width/2] = '-';
                            }
                       } 
                  }

                  // find diagonal lines '/'
                  for(int cy = 1; cy < cells_height-1; cy++) {
                       for(int cx = 1; cx < lmap_width-1; cx++) {
                            if(simulator->Lab()->isInside(cbPoint(cx*2.0+0.5,cy*2.0+0.5))){
                                lmap[(cy-initCell.y)*2+1+lmap_height/2][(cx-initCell.x)*2+1+lmap_width/2] = '/';
                            }
                       } 
                  }

                  // find diagonal lines '\'
                  for(int cy = 2; cy < cells_height; cy++) {
                       for(int cx = 1; cx < lmap_width-1; cx++) {
                            if(simulator->Lab()->isInside(cbPoint(cx*2.0+0.5,cy*2.0-0.5))){
                                 lmap[(cy-initCell.y)*2-1+lmap_height/2][(cx-initCell.x)*2+1+lmap_width/2] = '\\';
                            }
                       } 
                  }

                  //mark initial pos as I
                  lmap[lmap_height/2][lmap_width/2] = 'I';

                  //mark targets
                  for(unsigned int bi=0; bi < simulator->Lab()->nBeacons(); bi++) {
                      struct cell_t targetCell;
                      targetCell.x = simulator->Lab()->Target(bi)->Center().X()/2.0;
                      targetCell.y = simulator->Lab()->Target(bi)->Center().Y()/2.0;
                      lmap[(targetCell.y-initCell.y)*2+lmap_height/2][(targetCell.x-initCell.x)*2+lmap_width/2] = '0'+bi;
                  }

                  FILE *fp=fopen("planning.out","w");
                  if(fp==NULL) {
                       fprintf(stderr,"Could not create planning file\n");
                  }
                  else {
                     for(int ly = lmap_height-1; ly>=0; ly--) {
                        for(int lx = 0; lx < lmap_width; lx++) {
                              fprintf(fp,"%c",lmap[ly][lx]);
                        } 
                        fprintf(fp,"\n");
                     }
                     fclose(fp);
                  }

                  cbGraph grLab;

                  // add horizontal links
                  for(float x = 2.0; x < simulator->Lab()->Width()-2.0; x+=2.0) {
                      for(float y = 2.0; y < simulator->Lab()->Height(); y+=2.0) {
                          cbPoint from(x,y);
                          cbPoint to  (x+2.0,y);
                          cbPoint ptLine(x+0.5,y);

                          if(simulator->Lab()->isInside(ptLine)){
                              grLab.addLink(cbNode(from,MAXINT,MAXINT), cbNode(to,MAXINT,MAXINT),   2.0);
                              grLab.addLink(cbNode(to,MAXINT,MAXINT),   cbNode(from,MAXINT,MAXINT), 2.0);
                          }
                      }
                  }
                        
                  // add vertical links
                  for(float x = 2.0; x < simulator->Lab()->Width(); x+=2.0) {
                      for(float y = 2.0; y < simulator->Lab()->Height()-2.0; y+=2.0) {
                          cbPoint from(x,y);
                          cbPoint to  (x,y+2.0);
                          cbPoint ptLine(x, y+0.5);

                          if(simulator->Lab()->isInside(ptLine)){
                              grLab.addLink(cbNode(from,MAXINT,MAXINT), cbNode(to,MAXINT,MAXINT),   2.0);
                              grLab.addLink(cbNode(to,MAXINT,MAXINT),   cbNode(from,MAXINT,MAXINT), 2.0);
                          }
                      }
                  }

                  // add diagonal links '/'
                  for(float x = 2.0; x < simulator->Lab()->Width()-2.0; x+=2.0) {
                      for(float y = 2.0; y < simulator->Lab()->Height()-2.0; y+=2.0) {
                          cbPoint from(x,y);
                          cbPoint to  (x+2.0,y+2.0);
                          cbPoint ptLine(x+0.5, y+0.5);

                          if(simulator->Lab()->isInside(ptLine)){
                              grLab.addLink(cbNode(from,MAXINT,MAXINT), cbNode(to,MAXINT,MAXINT),   2.0*sqrt(2.0));
                              grLab.addLink(cbNode(to,MAXINT,MAXINT),   cbNode(from,MAXINT,MAXINT), 2.0*sqrt(2.0));
                              //fprintf(stderr,"add link '/' %.1f %.1f-> %.1f %.1f = %f\n", from.X(), from.Y(), to.X(),to.Y(),2.0*sqrt(2));
                          }
                      }
                  }

                  // add diagonal links '\'
                  for(float x = 2.0; x < simulator->Lab()->Width()-2.0; x+=2.0) {
                      for(float y = 4.0; y < simulator->Lab()->Height(); y+=2.0) {
                          cbPoint from(x,y);
                          cbPoint to  (x+2.0,y-2.0);
                          cbPoint ptLine(x+0.5, y-0.5);

                          if(simulator->Lab()->isInside(ptLine)){
                              grLab.addLink(cbNode(from,MAXINT,MAXINT), cbNode(to,MAXINT,MAXINT),   2.0*sqrt(2.0));
                              grLab.addLink(cbNode(to,MAXINT,MAXINT),   cbNode(from,MAXINT,MAXINT), 2.0*sqrt(2.0));
                              //fprintf(stderr,"add link '\\' %.1f %.1f-> %.1f %.1f = %f\n", from.X(), from.Y(), to.X(),to.Y(),2.0*sqrt(2));
                          }
                      }
                  }

                  // store distances between beacons
                  std::vector<std::vector<double> > beaconDists;
                  std::vector<double> aux(simulator->Lab()->nBeacons(), MAXINT);  
                  for(unsigned int bi=0; bi < simulator->Lab()->nBeacons(); bi++) {
                      beaconDists.push_back(aux);
                  }
                  for(unsigned int bi=0; bi < simulator->Lab()->nBeacons()-1; bi++) {
                      for(unsigned int bf=bi+1; bf < simulator->Lab()->nBeacons(); bf++) {
                          double dist=grLab.dist(cbNode(simulator->Lab()->Beacon(bi)->Center(), MAXINT, MAXINT),
                                                 cbNode(simulator->Lab()->Beacon(bf)->Center(), MAXINT, MAXINT));
                          beaconDists[bi][bf]=dist;
                          beaconDists[bf][bi]=dist;
                          //fprintf(stderr,"distance from beacons %d -> %d = %f\n", bi, bf, dist);
                      }
                  }

                  // find best path

                  std::vector<int> path, bestPath;
                  double bestDist=MAXINT;
                  for(unsigned int bi=0; bi < simulator->Lab()->nBeacons(); bi++) {
                      path.push_back(bi);
                  }
                  path.push_back(0);

                  if(path.size()>2) {
                      do {
                          double dist=0.0;
                          for(unsigned int p=0; p<path.size()-1; p++) {
                              dist+=beaconDists[path[p]][path[p+1]];
                          }
                          if (dist<bestDist) {
                              bestDist = dist;
                              bestPath = path;
                          }

                      } while(std::next_permutation(path.begin()+1, 
                                                    path.begin()+simulator->Lab()->nBeacons()));
                      
                      FILE *fp=fopen("planning.out","a");
                      if(fp==NULL) {
                       fprintf(stderr,"Could not append planning file\n");
                      }
                      else {
                          for(unsigned int p=0; p<bestPath.size(); p++) {
                              fprintf(fp," %d", bestPath[p]);
                          }
                          fprintf(fp,"\n%f\n",bestDist);
                     }
                     fclose(fp);
                  }
            }
            break;
        case FINISHED:
            if(removed) _state = REMOVED;
	default:
	    break;
    }
}



#define COLLISION_PENALTY   5
#define NOTFINISH_PENALTY  15
#define BASESCORE         100

void cbRobot::updateScoreControl()
{
    if (isRemoved() || hasFinished()) return;
    
    if (hasCollide() && !collisionPrevCycle)
    {
        //DEBUG
	//simulator->grAux->addFinalPoint(id,curPos.Coord());
        //double distCol=simulator->grAux->dist(id);
	//cerr << simulator->curTime() << ": R" << id << " distCol=" << distCol <<"\n";
        scorePenalties += -(collisionWallPenalty * hasCollideWall()) - (collisionRobotPenalty * hasCollideRobot());
	collisionCount++;

        emit robCollisionsChanged((int) collisionCount);
    }
    collisionPrevCycle=hasCollide();

    switch(_state) {
	    case RUNNING:
		 {

                    struct cell_t curCell = getRobotCell();
                    if(curCell.x == controlCellPath[nextPathInd].x && curCell.y == controlCellPath[nextPathInd].y) {
                        nextPathInd++;
                        if (nextPathInd >= nCellPath) nextPathInd=0;
                        scoreControl += 10;
                    }

		    score = scorePenalties + scoreControl;  

		    if(endLedOn()) { // robot finishing at Beacon
                        score=scorePenalties;
		    }
		    break;
		 }
	   default:
		    break;
    }
    emit robArrivalTimeChanged((int) arrivalTime);
    emit robReturnTimeChanged((int) returningTime);
    emit robScoreChanged((int) score);
}

void cbRobot::updateScoreLineControl2022()
{
    if (isRemoved() || hasFinished()) return;
    
    if (hasCollide() && !collisionPrevCycle)
    {
        //DEBUG
	//simulator->grAux->addFinalPoint(id,curPos.Coord());
        //double distCol=simulator->grAux->dist(id);
	//cerr << simulator->curTime() << ": R" << id << " distCol=" << distCol <<"\n";
        scorePenalties += -(collisionWallPenalty * hasCollideWall()) - (collisionRobotPenalty * hasCollideRobot());
	collisionCount++;

        emit robCollisionsChanged((int) collisionCount);
    }
    collisionPrevCycle=hasCollide();

    switch(_state) {
	    case RUNNING:
		 {

                    struct cell_t curCell;
                    curCell.x = curPos.X()+0.5;
                    curCell.y = curPos.Y()+0.5;
                    if(curCell.x == controlCellPath[nextPathInd].x && curCell.y == controlCellPath[nextPathInd].y) {
                        nextPathInd++;
                        if (nextPathInd >= nCellPath) nextPathInd=0;
                        scoreControl += 10;
                    }

                    // check if robot is outside line
                    bool outside=true;
                    for(int c=0 ; c < nCellPath; c++) {
                        if(controlCellPath[c].x == curCell.x && controlCellPath[c].y==curCell.y){
                            outside = false;
                            break;
                        }
                    }
                    if (outside) {
                        scoreControl -=10;
                    }

		    score = scorePenalties + scoreControl;  

		    if(endLedOn()) { // robot finishing at Beacon
                        score=scorePenalties;
		    }
		    break;
		 }
	   default:
		    break;
    }
    emit robArrivalTimeChanged((int) arrivalTime);
    emit robReturnTimeChanged((int) returningTime);
    emit robScoreChanged((int) score);
}

void cbRobot::updateScoreLineControl2023()
{
    if (isRemoved() || hasFinished()) return;
    
    if (hasCollide() && !collisionPrevCycle)
    {
        //DEBUG
	//simulator->grAux->addFinalPoint(id,curPos.Coord());
        //double distCol=simulator->grAux->dist(id);
	//cerr << simulator->curTime() << ": R" << id << " distCol=" << distCol <<"\n";
        scorePenalties += -(collisionWallPenalty * hasCollideWall()) - (collisionRobotPenalty * hasCollideRobot());
	collisionCount++;

        emit robCollisionsChanged((int) collisionCount);
    }
    collisionPrevCycle=hasCollide();

    switch(_state) {
	    case RUNNING:
		 {

                    struct cell_t curCell;
                    // values of 'x' and 'y' fields in currCell, controlCellPath and newCell do NOT REPRESENT the actual coordinates but their doubles
                    curCell.x = curPos.X()*2.0+0.5;
                    curCell.y = curPos.Y()*2.0+0.5;
                    if(fabs(controlCellPath[nextPathInd].x - curPos.X()*2.0) < 1.0 && fabs(controlCellPath[nextPathInd].y-curPos.Y()*2.0) < 1.0) {
                        nextPathInd++;
                        if (nextPathInd >= nCellPath) nextPathInd=0;
                        scoreControl += 10;
                    }

                    // check if robot is outside line
                    bool outside=true;
                    for(int c=0 ; c < nCellPath; c++) {
                        if(fabs(controlCellPath[c].x - curPos.X()*2.0) < 1.0 && fabs(controlCellPath[c].y-curPos.Y()*2.0) < 1.0){
                            outside = false;
                            break;
                        }
                    }
                    if (outside) {
                        scoreControl -=10;
                    }

		    score = scorePenalties + scoreControl;  

		    if(endLedOn()) { // robot finishing at Beacon
                        score=scorePenalties;
		    }
		    break;
		 }
	   default:
		    break;
    }
    emit robArrivalTimeChanged((int) arrivalTime);
    emit robReturnTimeChanged((int) returningTime);
    emit robScoreChanged((int) score);
}


void cbRobot::updateScoreLineMappingPlanning2022()
{
    if (isRemoved() || hasFinished()) return;
    
    if (hasCollide() && !collisionPrevCycle)
    {
        //DEBUG
	//simulator->grAux->addFinalPoint(id,curPos.Coord());
        //double distCol=simulator->grAux->dist(id);
	//cerr << simulator->curTime() << ": R" << id << " distCol=" << distCol <<"\n";
        scorePenalties += -(collisionWallPenalty * hasCollideWall()) - (collisionRobotPenalty * hasCollideRobot());
	collisionCount++;

        emit robCollisionsChanged((int) collisionCount);
    }
    collisionPrevCycle=hasCollide();

    switch(_state) {
	    case RUNNING:
		 {

            if (simulator->Lab()->wallDistanceAboveHeight(Center(),-1.0) > 0.5) {
                scorePenalties += 10;
            }

		    score = scorePenalties + scoreControl;  

		    if(endLedOn()) { // robot finishing at Beacon
                        score=scorePenalties;
		    }
		    break;
		 }
	   default:
		    break;
    }
    emit robScoreChanged((int) score);
}


void cbRobot::updateScore()
{
    if (isRemoved() || hasFinished()) return;
    
    if (hasCollide() && !collisionPrevCycle)
    {
        //DEBUG
        //simulator->grAux->addFinalPoint(id,curPos.Coord());
        //double distCol=simulator->grAux->dist(id);
        //cerr << simulator->curTime() << ": R" << id << " distCol=" << distCol <<"\n";
        //scorePenalties += COLLISION_PENALTY;
        scorePenalties += (collisionWallPenalty * hasCollideWall()) + (collisionRobotPenalty * hasCollideRobot());
        collisionCount++;

        emit robCollisionsChanged((int) collisionCount);
    }
    //collisionPrevCycle=collision;
    collisionPrevCycle = hasCollide();

    switch(_state) {
	    case RUNNING:
        case WAITOTHERS:
        {
            if(distGridToBeacon==0.0) {
                simulator->grAux->addFinalPoint(id,getSimulator()->Lab()->Beacon(0)->Center()); // VALID ONLY FOR ONE BEACON MAZES
                distGridToBeacon = simulator->grAux->dist(id);
            }

            if(distHomeToTarget==0.0) {
                simulator->grAux->addFinalPoint(1,getSimulator()->Lab()->Beacon(0)->Center()); // VALID ONLY FOR ONE BEACON MAZES
                distHomeToTarget = simulator->grAux->dist(1); // ROBOT 1 will always be at center of Home area
		    }

            score = scorePenalties + arrivalTimeScore();

		    if(visitingLedOn() && targetAtPos()==0 && _state == RUNNING) { // robot visiting Target Area

                targetVisited[targetAtPos()] = true;
                if(simulator->curTime() < simulator->keyTime()
                        || simulator->keyTime()>=simulator->simTime())
                    //scorePenalties -= 100;
                    scorePenalties -= targetReward;
                else
                    //scorePenalties -= 50 + 50 * (simulator->simTime() - simulator->curTime())/(simulator->simTime()-simulator->keyTime());
                    scorePenalties -= targetReward/2 + targetReward/2 * (simulator->simTime() - simulator->curTime())/(simulator->simTime()-simulator->keyTime());

                simulator->grAux->addFinalPoint(id,curPos.Coord());
                //cerr << simulator->curTime()<< ": R" << id << " distAtArrival=" << distAtArrival << "\n";
                arrivalTime=simulator->curTime();

		        score = scorePenalties + arrivalTimeScore();  
		    }
		    if(nVisitedTargets()==0 && _state == RUNNING)
		        arrivalTime = simulator->curTime(); 
		    if(nVisitedTargets()==1)
		        startReturningTime = simulator->curTime(); 
		    break;

		 }

	   case RETURNING: {
	            returningTime = simulator->curTime() - startReturningTime; 
		    if(endLedOn() && targetAtPos()==1) { // robot finishing at Home Area

		        targetVisited[targetAtPos()] = true;
                //scorePenalties -= 100;
                scorePenalties -= homeReward;

		    }
		    score = scorePenalties + returnTimeScore();  
		
           }
	   default:
		    break;
    }

    emit robArrivalTimeChanged((int) arrivalTime);
    emit robReturnTimeChanged((int) returningTime);
    emit robScoreChanged((int) score);
}

/*!
	Update all sensors in robot.
*/
void cbRobot::updateSensors()
{
	unsigned int s;
	for(s=0; s < sensors.size(); s++)
		sensors[s]->update();
}

/*!
	Send sensor measures and led states to robots.
*/
void cbRobot::sendSensors()
{
	char xml[16*1024];
	unsigned int n;
	n = sprintf(xml, "<Measures Time=\"%u\">\n", simulator->curTime());
	/* add sensor information */
	n += sprintf(xml+n, "\t<Sensors");

	if(!collisionSensor->requestable || collisionSensor->requested)
	     n += sprintf(xml+n, " Collision=\"%s\"", collisionSensor->Value()?"Yes":"No");

    if(compassSensorOn) {
	     if(!compassSensor->requestable || compassSensor->requested)
	         n += sprintf(xml+n, " Compass=\"%g\"", compassSensor->Degrees());
    }

	if(!groundSensor->requestable || groundSensor->requested)
	     n += sprintf(xml+n, " Ground=\"%d\"", groundSensor->Value());

	n += sprintf(xml+n, ">\n"); //end of attributes

	//IRSensors
	for(int i=0; i < NUM_IR_SENSORS; i++) {
		if(!irSensors[i]->requestable ||irSensors[i]->requested) {
		    n += sprintf(xml+n,"\t\t<IRSensor Id=\"%d\" Value=\"%g\"/>\n",
				i, irSensors[i]->Value());
		}
	}

	//beaconSensors
    //printf("Robot beac %d\n", cbRobot::beaconSensorOn);
    if(beaconSensorOn) {
	for(unsigned int b=0; b<beaconSensors.size();b++)
	{
	    if(!beaconSensors[b]->requestable || beaconSensors[b]->requested) {
	        if(beaconSensors[b]->Ready()){
		    n += sprintf(xml+n, "\t\t<BeaconSensor Id=\"%d\" Value=", b);
	            if(beaconSensors[b]->BeaconVisible())
	                n += sprintf(xml+n, "\"%g\"", beaconSensors[b]->Degrees());
	            else
	                n += sprintf(xml+n, "\"NotVisible\"");
		    n += sprintf(xml+n, "/>\n");
	        }
	    }
       }
    }

    //GPS
    if(GPSOn) {
	   if(!GPSSensor->requestable || GPSSensor->requested) {
	       n += sprintf(xml+n, "\t\t<GPS X=\"%g\" Y=\"%g\" ", GPSSensor->X(), GPSSensor->Y());
	       if(GPSDirOn)
	           n += sprintf(xml+n, " Dir=\"%g\" ", GPSSensor->Degrees());
	       n += sprintf(xml+n, "/>\n"); 
       }
    }

    //LineSensor
    char lineSensorStr[NLINESENSORELEMENTS+1]="";
    for(int i=0;i<NLINESENSORELEMENTS;i++){
        strcat(lineSensorStr,lineSensor->Value()[i]?"1":"0");
    }
	   if(!lineSensor->requestable || lineSensor->requested) {
	       n += sprintf(xml+n, "\t\t<LineSensor Value=\"%s\" />\n", lineSensorStr);
       }

    //Message
    unsigned int nRobots=simulator->Robots().size();
    for(unsigned int r=0; r < nRobots; r++) {
        if(simulator->Robots()[r]
            && simulator->Robots()[r]->Center().distance(curPos.Coord()) < 8.0  // MESSAGE_DIST_LIMIT
            && simulator->Robots()[r]->getSayMessage()!="")
            n += sprintf(xml+n, "\t\t<Message From=\"%d\"><![CDATA[%s]]></Message>\n" ,
                         r+1, simulator->Robots()[r]->getSayMessage().toLatin1().constData());
    }

    if(scoreSensorOn) {
	   char visitedMask[256];
	   int t;
	   //determine visitedMask
	   for(t = 0; t < (int) simulator->Lab()->nTargets(); t++) {
	        visitedMask[t]= '0' + (targetVisited[t] ? 1: 0);
	   }
     	   visitedMask[t]='\0';
	   n += sprintf(xml+n, "\t\t<Score Score=\"%d\" ArrivalTime=\"%d\" ReturningTime=\"%d\" "
			                  "Collisions=\"%d\" Collision=\"%s\" VisitedMask=\"%s\" />\n", 
                    score, arrivalTime,returningTime,collisionCount, (hasCollide()?"True":"False"), visitedMask);
    }

	n += sprintf(xml+n, "\t</Sensors>\n");
	/* add end led information */
	n += sprintf(xml+n, "\t<Leds EndLed=\"%s\" ReturningLed=\"%s\" VisitingLed=\"%s\"/>\n", 
			    endLed?"On":"Off", returningLed?"On":"Off", visitingLed?"On":"Off");
	/* add buttons information */
	n += sprintf(xml+n, "\t<Buttons Start=\"%s\" Stop=\"%s\"/>\n",
			simulator->getNextState()==cbSimulator::RUNNING?"On":"Off",
                        simulator->getNextState()==cbSimulator::STOPPED?"On":"Off");
	n += sprintf(xml+n, "</Measures>\n");

	/* send XML message to client */
	send(xml, n+1);
	
#ifdef DEBUG_ROBOT
	cerr << "Measures sent to robot " << id << "\n" << xml;
#endif

    if (showMeasures)
        simulator->GUI()->writeOnBoard("Measures sent to " + QString(name) + "(robot " + QString::number(id) + ")" + ":\n" + xml, (int) id, 0);

}


/*!
	Fill in given xml buffer with robot state, that is, name, id, 
	score, number of collisions, the collision state, removed states, and current position.
	Return the length of the xml message.
*/
unsigned int cbRobot::toXml(char *xml, unsigned int len) // len = buffer size not used
{
	unsigned int n;
	int t;
	n = sprintf(xml, "<Robot");
	/* add attributes */
	n += sprintf(xml+n, " Name=\"%s\"", name);
	n += sprintf(xml+n, " Id=\"%d\"", id);
	n += sprintf(xml+n, " Time=\"%u\"", simulator->curTime());
	n += sprintf(xml+n, " Score=\"%u\"", score);
	n += sprintf(xml+n, " ArrivalTime=\"%u\"", arrivalTime);
	n += sprintf(xml+n, " ReturningTime=\"%u\"", returningTime);
	n += sprintf(xml+n, " Collisions=\"%u\"", collisionCount);
    n += sprintf(xml+n, " Collision=\"%s\"", hasCollide() ? "True" : "False");

	n += sprintf(xml+n, " VisitedMask=\"");
	for(t = 0; t < (int) simulator->Lab()->nTargets(); t++) {
	    n += sprintf(xml+n, "%d", targetVisited[t] ? 1: 0);
	}
	n += sprintf(xml+n, "\"");

	n += sprintf(xml+n, " State=\"%s\">\n", StrState[_state]);

	/* add position */
	double dir = curPos.directionInDegrees();
	n += sprintf(xml+n, "\t<Position X=\"%g\" Y=\"%g\" Dir=\"%g\"/>\n", curPos.X(), 
			curPos.Y(), dir);
	n += sprintf(xml+n, "</Robot>\n");

    if(n > len-1) {
        fprintf(stderr,"cbRobot::toXml message is too long\n");
        abort();
    }

	//if(returningTime > 0) cerr << "returnTime=" << returningTime << " startRet=" << startReturningTime << "\n";
	//cerr << xml;

	return n;
}


void cbRobot::showAllAttributes()
{
	char xml[16*1024];
	/* add global robot information */
	unsigned int n = toXml(xml, 1024);
	n = n - 9;	// go back </Robot> tag
	double dir = nextPos.directionInDegrees();
	n += sprintf(xml+n, "\t<NextPosition X=\"%g\" Y=\"%g\" Dir=\"%g\"/>\n", 
			nextPos.X(), nextPos.Y(), dir);
	n += sprintf(xml+n, "\t<LeftMotor InPower=\"%g\" OutPower=\"%g\"/>\n", 
			leftMotor.inPower(), leftMotor.lastOutPower());
	n += sprintf(xml+n, "\t<RightMotor InPower=\"%g\" OutPower=\"%g\"/>\n", 
			rightMotor.inPower(), rightMotor.lastOutPower());
	n += sprintf(xml+n, "\t<EndLed State=\"%s\"/>\n", endLed?"On":"Off");
	n += sprintf(xml+n, "\t<Measures Time=\"%u\">\n", simulator->curTime());
	/* add sensor information */
	n += sprintf(xml+n, "\t\t<Sensors");
	n += sprintf(xml+n, " Compass=\"%g\"", compassSensor->Degrees());
	n += sprintf(xml+n, " Collision=\"%s\"", collisionSensor->Value()?"Yes":"No");
	n += sprintf(xml+n, " Ground=\"%d\">\n", groundSensor->Value());
	for(int i=0; i < NUM_IR_SENSORS; i++)
		n += sprintf(xml+n,"\t\t<IRSensor Id=\"%d\" Value=\"%g\"/>",
				i, irSensors[i]->Value());
	for(unsigned int b=0; b < beaconSensors.size(); b++) {
	    if(beaconSensors[b]->Ready()){
		n += sprintf(xml+n, "\t\t<BeaconSensor Id=\"%d\" Value=", b);
	        if(beaconSensors[b]->BeaconVisible())
	            n += sprintf(xml+n, "\"%g\"", beaconSensors[b]->Degrees());
	        else
	            n += sprintf(xml+n, "\"NotVisible\"");
		n += sprintf(xml+n, "/>\n");
	    }
	}
	n += sprintf(xml+n, "\t\t</Sensors>\n");
	/* add end led information */
	n += sprintf(xml+n, "\t\t<Leds EndLed=\"%s\"/>\n", endLed?"On":"Off");
	/* add buttons information */
	n += sprintf(xml+n, "\t\t<Buttons Start=\"%s\" Stop=\"%s\"/>\n",
            simulator->getNextState()==cbSimulator::RUNNING?"On":"Off", 
            simulator->getNextState()==cbSimulator::STOPPED?"On":"Off");
	n += sprintf(xml+n, "\t</Measures>\n");
	n += sprintf(xml+n, "</Robot>\n");

	cout << xml;
}

#define LOGWITHMEASURES

void cbRobot::Log(ostream &log, bool withActions)
{
	char visitedMask[256];
    int  t;
    log << "\t<Robot Name=\"" << Name() <<"\""
		       << " Id=\"" << Id() << "\"" 
		       << " State=\"" << StrState[_state] <<"\"" 
		       << ">\n";
	log << "\t\t<Pos X=\""<< X()
	          << "\" Y=\"" << Y() 
	          << "\" Dir=\"" << Degrees() << "\"/>\n";

	//determine visitedMask
	for(t = 0; t < (int) simulator->Lab()->nTargets(); t++) {
	    visitedMask[t]= '0' + (targetVisited[t] ? 1: 0);
	}
	visitedMask[t]='\0';

	log << "\t\t<Scores Score=\"" << score << "\""
		        <<" ArrivalTime=\"" << arrivalTime <<"\""
		        <<" ReturningTime=\"" << returningTime <<"\""
		        <<" Collisions=\"" << collisionCount <<"\""
                <<" Collision=\"" << (hasCollide() ? "True" : "False") << "\""
		        <<" VisitedMask=\"" << visitedMask << "\""
		        << "/>\n";

    if( withActions && receivedAction()  )
    {
        log << "\t\t<Action";
	    if(receivedLeftMotor())
	        log << " LeftMotor=\"" << LeftMotor().inPower() << "\"";
	    if(receivedRightMotor())
	        log << " RightMotor=\"" << RightMotor().inPower() << "\""; 
	    if(receivedEndLed())
	        log << " EndLed=\"" << (endLedOn()?"On":"Off") << "\"" ;
	    if(receivedReturningLed())
	        log << " ReturningLed=\"" << (returningLedOn()?"On":"Off") << "\"" ;
	    if(receivedVisitingLed())
	        log << " VisitingLed=\"" << (visitingLedOn()?"On":"Off") << "\"" ;
	    log << " />\n";
	}

#ifdef LOGWITHMEASURES
	char xml[1024*16];
	unsigned int n;
	n = sprintf(xml, "<Measures Time=\"%u\">\n", simulator->curTime());
	/* add sensor information */
	n += sprintf(xml+n, "\t<Sensors");
	n += sprintf(xml+n, " Compass=\"%g\"", compassSensor->Degrees());

	n += sprintf(xml+n, " Collision=\"%s\"", collisionSensor->Value()?"Yes":"No");
	n += sprintf(xml+n, " Ground=\"%d\">\n", groundSensor->Value());

	for(int i=0; i < NUM_IR_SENSORS; i++)
		n += sprintf(xml+n,"\t\t<IRSensor Id=\"%d\" Value=\"%g\"/>\n",
				i, irSensors[i]->Value());

    if(beaconSensorOn) {
	for(unsigned int b=0; b < beaconSensors.size(); b++) {
        if(beaconSensors[b]->Ready()){
            n += sprintf(xml+n, "\t\t<BeaconSensor Id=\"%d\" Value=", b);
	        if(beaconSensors[b]->BeaconVisible())
	            n += sprintf(xml+n, "\"%g\"", beaconSensors[b]->Degrees());
	        else
                n += sprintf(xml+n, "\"NotVisible\"");
            n += sprintf(xml+n, "/>\n");
	    }
	}
    }

    if(GPSOn) {
	   n += sprintf(xml+n, "\t\t<GPS X=\"%g\" Y=\"%g\" ", 
                    GPSSensor->X(), GPSSensor->Y());
       if(GPSDirOn) n += sprintf(xml+n, " Dir=\"%g\" ", GPSSensor->Degrees());
       n += sprintf(xml+n, "/>\n");
    }

    //LineSensor
    char lineSensorStr[NLINESENSORELEMENTS+1]="";
    for(int i=0;i<NLINESENSORELEMENTS;i++){
        strcat(lineSensorStr,lineSensor->Value()[i]?"1":"0");
    }
	n += sprintf(xml+n, "\t\t<LineSensor Value=\"%s\" />\n", lineSensorStr);

	n += sprintf(xml+n, "\t</Sensors>\n");
	/* add end led information */
	n += sprintf(xml+n, "\t<Leds EndLed=\"%s\" ReturningLed=\"%s\" VisitingLed=\"%s\"/>\n", 
			    endLed?"On":"Off", returningLed?"On":"Off", visitingLed?"On":"Off");
	/* add buttons information */
	n += sprintf(xml+n, "\t<Buttons Start=\"%s\" Stop=\"%s\"/>\n",
			simulator->getNextState()==cbSimulator::RUNNING?"On":"Off",
                        simulator->getNextState()==cbSimulator::STOPPED?"On":"Off");
	n += sprintf(xml+n, "</Measures>\n");

	log << xml;
#endif

    log << "\t</Robot>\n";
}

// cbRobotBin

#include "netif.h"


bool cbRobotBin::Reply(QHostAddress &a, unsigned short &p, cbParameters *param)
{
	//cout.form("Sending reply for client to %s:%hd\n", a.toString().latin1(), p);
	/* set peer address and peer port */
    address = a;
	port = p;

	/* constructing reply message */
   CommMessage commMsg;

   commMsg.comm=htons(OK_COMM);

   commMsg.u.params.cycle_time= htons(param->cycleTime);
   //commMsg.u.params.sim_time_final  =htons(sim_time);
   //commMsg.u.params.noise_obstacles =htons( (unsigned short) (noise_obstacles*10.0+0.5) );
   //commMsg.u.params.noise_beacon    =htons( (unsigned short) (noise_beacon+0.5));
   //commMsg.u.params.noise_compass   =htons( (unsigned short) (noise_compass+0.5));
   //commMsg.u.params.noise_motors    =htons( (unsigned short) (noise_motors*10.0+0.5));
   

    /* send reply to client */
   if (writeDatagram((char *)&commMsg, sizeof(CommMessage), address, port) != sizeof(CommMessage))
	{
        cerr << "Fail replying to client\n";
        simulator->GUI()->appendMessage( "Fail replying to client", true);
		return false;
	}

   //cout << "Reply sent\n" << reply;
	return true;
}


bool cbRobotBin::readAction(cbRobotAction *action)
{
	/* look for an incoming message */
   ActMessage msg;

   int ret;

   if (!hasPendingDatagrams())
       return false;

	if ((ret=readDatagram((char *)&msg, sizeof(msg))) < 0)
    {
        cerr << "Error reading from robot socket - " << errorString().toStdString();
        //cerr << "Error no. " << error() << " reading from robot socket\n";
		return false;
	}

   // check message size
   if(ret!=sizeof(ActMessage)) {
       cerr << "Received bad ActMessage from " << Name() << "\n";
       simulator->GUI()->appendMessage( QString("Received bad ActMessage from ") + Name(), true );
   }

   action->leftMotor=((short)ntohs(msg.lPow))/1000.0-2.0;
   action->rightMotor=((short)ntohs(msg.rPow))/1000.0-2.0;
   action->endLed =(bool)ntohs(msg.end);
   action->leftMotorChanged=true;
   action->rightMotorChanged=true;
   action->endLedChanged =true;

#ifdef DEBUG_ROBOT
   cerr << "actions received from robot\n";
#endif

   return true;
}

void cbRobotBin::sendSensors()
{

   SenseMessage msg;

   msg.time=htons((unsigned short)simulator->curTime());

   msg.center   =htons((unsigned short) (irSensors[1]->Value()*10.0 + 0.5) );
   msg.left     =htons((unsigned short) (irSensors[0]->Value()*10.0 + 0.5) );
   msg.right    =htons((unsigned short) (irSensors[2]->Value()*10.0 + 0.5) );

   msg.light    =htons((unsigned short) (beaconSensors[0]->Degrees()+360.0 + 0.5) );
   msg.compass  =htons((unsigned short) (compassSensor->Degrees()+360.0 + 0.5) );
   msg.ground   =htons((unsigned short) groundSensor->Value() );
   msg.collision=htons((unsigned short) collisionSensor->Value() );
   msg.start    = htons((unsigned short) (simulator->getNextState()==cbSimulator::RUNNING) );
   msg.stop     = htons((unsigned short) (simulator->getNextState()==cbSimulator::STOPPED) );
   msg.endLed   = htons((unsigned short) endLed);


	/* send XML message to client */
	send((char *)&msg, sizeof(msg));
#ifdef DEBUG_ROBOT
	cerr << "Measures sent to robot (bin protocol)\n";
#endif
}

/*!
	Send the Refused reply message to client.
*/
bool cbRobotBin::Refuse(QHostAddress &a, unsigned short &p)
{
	//cout.form("Sending refuse for client to %s:%hd\n", a.toString().latin1(), p);
	/* set peer address and peer port */
	address = a;
    port = p;

    CommMessage commMsg;

    commMsg.comm=htons(OK_COMM+1);

    send((char *)&commMsg,sizeof(commMsg));

	return true;
}
