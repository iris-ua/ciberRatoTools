/*
    This file is part of ciberRatoToolsSrc.

    Copyright (C) 2001-2022 Universidade de Aveiro

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
/*
 * class cbSimulator
 */



#include "cbposition.h"
#include "cbparameters.h"

#include "cblab.h"

#include "cbreceptionist.h"
#include "cbview.h"
#include "cbpanel.h"
#include "cbrobot.h"
#include "cbrobotbeacon.h"
#include "cbgrid.h"
#include "cbgraph.h"
#include "cbutils.h"
#include "cblabhandler.h"
#include "cbgridhandler.h"
#include "cbparamhandler.h"

#include "cbsimulator.h"

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <assert.h>


#include <QtGui>
#include <QVector>
#include <QMessageBox>
#include <QMouseEvent>
#include <QFileDialog>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsRectItem>

using std::cerr;
using std::cout;
using std::ofstream;

/*!
	\constant LAB
	\brief Default lab.
*/
static const char *LAB =
	"<Lab Name=\"Default LAB\" Height=\"14\" Width=\"28\">\n"
	"\t<Beacon X=\"25\" Y=\"7.0\" Height=\"4.0\"/>\n"
	"\t<Target X=\"25\" Y=\"7.0\" Radius=\"1.0\"/>\n"
	//		"\t<Row  Pos=\"12\" Pattern=\"                 |                       \"  />\n"
	"\t<Row  Pos=\"12\" Pattern=\"  |        |     |                 |     \"  />\n"
	"\t<Row  Pos=\"11\" Pattern=\"  �--�  �--�--�  �  �--�--.--�--�  �--�--\"  />\n"
	"\t<Row  Pos=\"10\" Pattern=\"  |        |              |        |     \"  />\n"
	"\t<Row  Pos=\"9\"  Pattern=\"--�--�  �--�  �--�--�--�  �--�--�  �--�  \"  />\n"
	"\t<Row  Pos=\"8\"  Pattern=\"        |        |     |     |     |  |  \"  />\n"
	"\t<Row  Pos=\"7\"  Pattern=\"--�--�--�  �--�  .--�--�--�  �  �  �--�  \"  />\n"
	"\t<Row  Pos=\"6\"  Pattern=\"  |                                      \"  />\n"
	"\t<Row  Pos=\"5\"  Pattern=\"  �--�--�--�--�--�--�  �  �--�  �--�--�--\"  />\n"
	"\t<Row  Pos=\"4\"  Pattern=\"     |  |              |     |     |     \"  />\n"
	//		"\t<Row  Pos=\"4\"  Pattern=\"        |              |     |     |     \"  />\n"
	"\t<Row  Pos=\"3\"  Pattern=\"  �--�  �--�--�--�--�  �--�  �  �--�  �  \"  />\n"
	"\t<Row  Pos=\"2\"  Pattern=\"  |                 |           |     |  \"  />\n"
	"\t<Row  Pos=\"1\"  Pattern=\"  �--�  �--�--�--�  �--�--�  �--�  �--�  \"  />\n"
	"\t<Row  Pos=\"0\"  Pattern=\"  |  |           |           |     |     \"  />\n"
	/*		"\t<Wall Height=\"5.0\">\n"
				"\t\t<Corner X=\"10.0\" Y=\"6.0\"/>\n"
				"\t\t<Corner X=\"11.0\" Y=\"6.0\"/>\n"
				"\t\t<Corner X=\"11.0\" Y=\"8.0\"/>\n"
				"\t\t<Corner X=\"10.0\" Y=\"8.0\"/>\n"
			"\t</Wall>\n"
			"\t<Wall Height=\"5.0\">\n"
				"\t\t<Corner X=\"4.0\" Y=\"4.0\"/>\n"
				"\t\t<Corner X=\"5.0\" Y=\"4.0\"/>\n"
				"\t\t<Corner X=\"5.0\" Y=\"10.0\"/>\n"
				"\t\t<Corner X=\"4.0\" Y=\"10.0\"/>\n"
			"\t</Wall>\n"
	*/
	"</Lab>\n";

/*!
	\constant GRID
	\brief Default grid
*/
static const char *GRID =
	"<Grid>\n"
	"\t<Position X=\"5.0\" Y=\"7.0\" Dir=\"0.0\"/>\n"
	"\t<Position X=\"9.0\" Y=\"11.0\" Dir=\"0.0\"/>\n"
	"\t<Position X=\"7.0\" Y=\"9.0\" Dir=\"0.0\"/>\n"
	"\t<Position X=\"7.0\" Y=\"5.0\" Dir=\"0.0\"/>\n"
	"\t<Position X=\"9.0\" Y=\"3.0\" Dir=\"0.0\"/>\n"
	"</Grid>\n";

/*! \constant SIMPARAM
	\brief Default simulation parameteres
*/
static const char *SIMPARAM =
	"<Parameters SimTime=\"1800\" KeyTime=\"1350\" CycleTime=\"50\"\n"
	"\t\tCompassNoise=\"2.0\" BeaconNoise=\"2.0\" ObstacleNoise=\"0.1\"\n"
	"\t\tNRequestsPerCycle=\"0\"\n"
	"\t\tCompassLatency=\"0\" BeaconLatency=\"0\"\n"
	"\t\tMotorsNoise=\"1.5\"\n"
	"\t\tReturnTimePenalty=\"25\" ArrivalTimePenalty=\"100\"\n"
	"\t\tCollisionWallPenalty=\"2\" CollisionRobotPenalty=\"2\"\n"
	"\t\tTargetReward=\"100\" HomeReward=\"100\"/>\n";

cbSimulator::cbSimulator()
{
	lab = 0;
	curCycle = 0;
	endCycle = 3000; // provis�rio
	cycle = 50;

	curState = nextState = INIT;

	logging = false;
	logStream = 0;

	distMaxToTarget = 0.0;

	lab = 0;
	grid = 0;
	param = 0;
	receptionist = 0;
	gui = 0;
	graph = 0;
	grAux = 0;
	labScene = 0;
	labView = 0;

	defaultParam = 0;
	defaultLab = 0;
	defaultGrid = 0;
	QXmlInputSource *sourceParam = new QXmlInputSource;
	QXmlInputSource *sourceLab = new QXmlInputSource;
	QXmlInputSource *sourceGrid = new QXmlInputSource;

	sourceParam->setData(QByteArray(SIMPARAM));
	sourceLab->setData(QByteArray(LAB));
	sourceGrid->setData(QByteArray(GRID));

	QXmlSimpleReader xmlParserParam, xmlParserLab, xmlParserGrid;

	cbParamHandler *paramHandler = new cbParamHandler(defaultParam);
	cbLabHandler *labHandler = new cbLabHandler;
	cbGridHandler *gridHandler = new cbGridHandler;

	xmlParserParam.setContentHandler(paramHandler);
	xmlParserLab.setContentHandler(labHandler);
	xmlParserGrid.setContentHandler(gridHandler);

	scoring = 0;

	if (xmlParserParam.parse(*sourceParam))
		defaultParam = paramHandler->parsedParameters();
	else
	{
		cerr << "Error parsing DEFAULT parameters\n";
		gui->appendMessage(QString("Error parsing DEFAULT parameters"), true);
		assert(0);
	}

	if (xmlParserLab.parse(*sourceLab))
		defaultLab = labHandler->parsedLab();
	else
	{
		cerr << "Error parsing DEFAULT lab\n";
		gui->appendMessage(QString("Error parsing DEFAULT lab"), true);
		assert(0);
	}

	if (xmlParserGrid.parse(*sourceGrid))
		defaultGrid = gridHandler->parsedGrid();
	else
	{
		cerr << "Error parsing DEFAULT grid\n";
		gui->appendMessage(QString("Error parsing DEFAULT grid"), true);
		assert(0);
	}

	delete sourceParam;
	delete sourceLab;
	delete sourceGrid;
	delete paramHandler;
	delete labHandler;
	delete gridHandler;

	setDefaultParameters();
	setDefaultGrid();
	setDefaultLab();

	setRegistrations(true);
	setShowPositions(false);
}

cbSimulator::~cbSimulator()
{
	if (logging && logStream != 0)
	{
		*logStream << "</Log>\n";
		delete logStream;
		logStream = 0;
	}
}

void cbSimulator::reset()
{
	unsigned int j;

	// Close Log
	closeLog();

	// notify viewers
	for (j = 0; j < views.size(); j++)
	{
		views[j]->send("<Restart/>", 11);
	}

	// delete Viewers
	// for (j = 0; j < views.size(); j++)
	// {
	// 	views.clear();
	// }

	// delete robots
	// for (unsigned int i = 0; i < robots.size(); i++)
	// {
	// 	/*if(robots[i] != 0) {
	// 		delete robots[i];
	// 		robots[i]=0;
	// 	}*/
	// 	deleteRobot(i + 1);
	// }

	for (unsigned int i = 0; i < robots.size(); i++)
	{
	    if(robots[i] != 0) {
    		robots[i]->resetRobot();  
			robots[i]->setPosition(grid->at(i));
	 	}
	 }


	gui->appendMessage("RESETTING");
	cout << "RESETTING\n";

	curCycle = 0;
	//curState = nextState = INIT;

	curState = INIT;
	nextState = RUNNING;

	cbGPSSensor::offsetX = randUniform(0.0, 1000.0);
	cbGPSSensor::offsetY = randUniform(0.0, 1000.0);

	// Open New Log
	if (logging)
	{
		QString logFilename = QFileDialog::getSaveFileName(0, "Choose New Log Filename", ".", "Logs (*.log)");
		if (!logFilename.isNull())
		{
			setLogFilename(logFilename);
			// openLog(logFilename.toLatin1().constData());
			gui->appendMessage(QString("Logfile changed to ") + logFilename);
		}
		else
		{
			logging = false;
			gui->appendMessage("Logging disabled");
		}
	}

	emit stateChanged(curStateAsString());
	emit simReset(true);
	emit simReady(false);
	emit simRunning(false);
	emit curTimeChanged(0);
}

void cbSimulator::setLab(cbLab *l)
{
	if (lab != 0)
		delete lab;
	lab = l;

	// update parameters
	param->nBeacons = Lab()->nBeacons();

	// emit signal
	emit labChanged(Lab()->Name());
}

void cbSimulator::setGrid(cbGrid *g)
{
	unsigned int i;
	for (i = 0; i < robots.size(); i++)
		if (robots[i] != 0)
			delete robots[i];

	if (grid != 0)
		delete grid;
	/* set the new grid */
	grid = g;
	/* resize robot array to new grid size */
	robots.resize(grid->size());
	for (i = 0; i < robots.size(); i++)
		robots[i] = 0;

	emit gridChanged(grid->size());
}

void cbSimulator::setParameters(cbParameters *p)
{
	if (param != 0)
		delete param;

	param = p;

	// update simulator variables according to parameters
	processEditParameters();

	if (!param->labFilename.isNull())
		changeLab(param->labFilename);

	if (!param->gridFilename.isNull())
		changeGrid(param->gridFilename);
}

void cbSimulator::setReceptionist(cbReceptionist *r)
{
	receptionist = r;
	connect(r, &QUdpSocket::readyRead, this, &cbSimulator::CheckIn);

}
void cbSimulator::setReceptionistAt(int port)
{
	cbReceptionist *receptionist = new cbReceptionist(port);
	if (receptionist == 0 || receptionist->bad())
	{
		cerr << "Could not initialize receptionist\n";
		QMessageBox::critical(0, "Error",
							  QString("Error creating socket. Please check running processes!"),
							  QMessageBox::Ok, Qt::NoButton, Qt::NoButton);
		exit(-1);
		assert(0);
	}
	setReceptionist(receptionist);
}

void cbSimulator::setGPS(bool g)
{
	if (g == param->GPSOn)
		return;

	if (curState == INIT)
	{
		param->GPSOn = g;
		cbRobot::GPSOn = g;
	}
	else
		gui->appendMessage(QString("Cannot Change Configuration After Start - Use Reset"), true);

	emit toggleGPS(param->GPSOn);
}

void cbSimulator::setBeaconSensor(bool g)
{
	if (g == param->beaconSensorOn)
		return;

	if (curState == INIT)
	{
		param->beaconSensorOn = g;
		cbRobot::beaconSensorOn = g;
	}
	else
		gui->appendMessage(QString("Cannot Change Configuration After Start - Use Reset"), true);

	emit toggleBeaconSensor(param->beaconSensorOn);
}

void cbSimulator::setCompassSensor(bool g)
{
	if (g == param->compassSensorOn)
		return;

	if (curState == INIT)
	{
		param->compassSensorOn = g;
		cbRobot::compassSensorOn = g;
	}
	else
		gui->appendMessage(QString("Cannot Change Configuration After Start - Use Reset"), true);

	emit toggleCompassSensor(param->compassSensorOn);
}

bool cbSimulator::getGPS(void)
{
	return param->GPSOn;
}

bool cbSimulator::getBeaconSensor(void)
{
	return param->beaconSensorOn;
}

bool cbSimulator::getCompassSensor(void)
{
	return param->compassSensorOn;
}

void cbSimulator::setScoreSensor(bool g)
{
	if (g == param->scoreSensorOn)
		return;

	if (curState == INIT)
	{
		param->scoreSensorOn = g;
		cbRobot::scoreSensorOn = g;
	}
	else
		gui->appendMessage(QString("Cannot Change Configuration After Start - Use Reset"), true);

	emit toggleScoreSensor(param->scoreSensorOn);
}

bool cbSimulator::getScoreSensor(void)
{
	return param->scoreSensorOn;
}

void cbSimulator::setTime(bool timeOn)
{
	// if (timeOn == isTimed()) return;

	endCycle = timeOn ? param->simTime : 0;

	emit simTimeChanged(endCycle);
	emit toggleTime(timeOn);
}

bool cbSimulator::isTimed()
{
	return simTime() > 0;
}

void cbSimulator::setCollisions(bool collisions)
{
	if (cbRobot::ignoreOthers == !collisions)
		return;

	cbRobot::ignoreOthers = !collisions;
	emit toggleCollisions(collisions);
}

bool cbSimulator::collisionsIgnored()
{
	return cbRobot::ignoreOthers;
}

void cbSimulator::setRegistrations(bool allow)
{
	if (allow == allowRegistrations)
		return;

	allowRegistrations = allow;
	emit toggleRegistrations(allow);
}

bool cbSimulator::isRegistrationAllowed()
{
	return allowRegistrations;
}

void cbSimulator::setShowActions(bool s)
{
	cbRobot::showActions = s;
	param->showActions = s;
}

void cbSimulator::setShowMeasures(bool s)
{
	cbRobot::showMeasures = s;
}

void cbSimulator::setShowPositions(bool s)
{
	showPositions = s;
}

int cbSimulator::openLog(const char *logFilename)
{
	char buff[1024 * 128];

	if (logFilename != 0)
		logStream = new ofstream(logFilename);
	else
		logStream = 0;

	if (!logStream)
	{
		cerr << "ERROR: Could not open " << logFilename << " for writing\n";
		gui->appendMessage(QString("ERROR: Could not open ") + logFilename + " for writing", true);
		logging = false;
		return -1;
	}

	cout << "Logfile \"" << logFilename;
	cout << "\" opened.\n";

	logging = true;

#ifndef MicWindows
	char datestr[1024];
	time_t datet = time(0);
	strcpy(datestr, ctime(&datet));
	datestr[strlen(datestr) - 1] = '\0'; // discard '\n'
	*logStream << "<Log Date=\"" << datestr << "\" >\n";
#else
	*logStream << "<Log>\n";
#endif

	param->toXml(buff, sizeof(buff));
	*logStream << buff;
	lab->toXml(buff, sizeof(buff));
	*logStream << buff;
	grid->toXml(buff, sizeof(buff));
	*logStream << buff;

	return 0;
}

int cbSimulator::closeLog(void)
{
	if (logging && logStream != 0)
	{
		gui->appendMessage("Logfile  closed.");
		*logStream << "</Log>\n";
		delete logStream;
		logStream = 0;
	}

	return 0;
}

/*!
	Add given robot to robot list.
	Registration is refused if (1) no grid was set; (2) robot id is
	not valid; (3) another robot with the same id already exist.
*/
bool cbSimulator::registerRobot(cbRobot *robot)
{
	unsigned int i;

	bool isRobotBeacon = ((dynamic_cast<cbRobotBeacon *>(robot)) != 0);

	if (isRobotBeacon && lab->nBeacons() != 0)
	{
		cerr << "beacon already exists";
		return false;
	}

	/* check existence of grid */
	if (grid == 0)
		return false;
	if (curState == RUNNING && !allowRegistrations)
		return false;
	if (curState == INIT)
	{
		nextState = STOPPED;
		if (logging)
			openLog(logFilename.toLatin1().constData());
	}

	/* check id */
	unsigned int id = robot->Id();
	if (id == 0)
	{ // first available position in the grid is occupied
		for (i = 1; i < robots.size() + 1 && robots[i - 1] != 0; i++)
			;
		if (i == robots.size() + 1)
			return false;
		id = i;
		robot->setId(id);
	}
	else
	{
		if (id > robots.size())
			return false;

		/* check existence of another robot with the same id */
		if (robots[id - 1] != 0)
			return false;

		/* add robot and set attributes */
	}
	robots[id - 1] = robot;
	robot->setSimulator(this);
	const cbPosition &pos = grid->at(id - 1);
	robot->setPosition(pos);

	if (isRobotBeacon)
		lab->addBeacon(dynamic_cast<cbRobotBeacon *>(robot));

	emit robotRegistered((int)id);
	return true;
}

unsigned int cbSimulator::curTime()
{
	return curCycle;
}

unsigned int cbSimulator::simTime()
{
	return endCycle;
}

unsigned int cbSimulator::cycleTime()
{
	return cycle;
}

unsigned int cbSimulator::keyTime()
{
	return param->keyTime;
}

const char *cbSimulator::curStateAsString()
{
	static const char *sas[] = {"Ready", "Stopped", "Running", "Finished"};
	return sas[curState];
}

void cbSimulator::step()
{
	// cout.form("Reading robot actions (%u)\n", curCycle);
	// for SYNC RobotActions();
	if (logging)
		Log(*logStream);
	// cout.form("Checking new registrations (%u)\n", curCycle);
	CheckIn();
	// cout.form("Reading view commands (%u)\n", curCycle);
	ViewCommands();
	PanelCommands();

	if (curState == RUNNING)
	{
		// cout.form("Processing a RUNNING cycle (%u)\n", curCycle);
		curCycle++;
		emit curTimeChanged(curCycle);
		// cout.form("Computing next positions (%u)\n", curCycle);
		NextPositions();
		// cout.form("Check collisions (%u)\n", curCycle);
		CheckCollisions();
		// cout.form("Commiting (%u)\n", curCycle);
		Commit();
		// cout.form("Updating sensors (%u)\n", curCycle);
		UpdateSensors();
		// cout.form("Updating scores (%u)\n", curCycle);
		UpdateScores();
	}
	else
	{
		// cout.form("Processing a STOPPING cycle (%u)\n", curCycle);
		// cout.form("Updating sensors (%u)\n", curCycle);
		UpdateSensors();
	}

	// cout.form("Sending sensors to robots(%u)\n", curCycle);
	SendSensors();
	// cout.form("Updating views (%u)\n", curCycle);
	UpdateViews();
	// cout.form("Updating state (%u)\n", curCycle);
	UpdateState();
}

/*
 * Private auxiliary member functions
 */

/*!
	Process registration requests of all
	clients waiting at reception.
*/
void cbSimulator::CheckIn()
{
	while (receptionist->CheckIn())
	{
		cbClientForm &form = receptionist->Form();
		int cnt;
		switch (form.type)
		{
		case cbClientForm::VIEW:
			// cout << "View form is going to be processed\n";
			cnt = views.size();
			views.resize(cnt + 1);
			views[cnt] = form.client.view;
			views[cnt]->Reply(form.addr, form.port, param);
			if (curState == INIT)
			{
				nextState = STOPPED;
				if (logging)
					openLog(logFilename.toLatin1().constData());
			}
			cout << "Viewer has been registered\n";
			gui->appendMessage("Viewer has been registered\n");
			break;
		case cbClientForm::PANEL:
			// cout << "Panel form is going to be processed\n";
			cnt = panels.size();
			panels.resize(cnt + 1);
			panels[cnt] = form.client.panel;
			panels[cnt]->Reply(form.addr, form.port, param);
			break;
		case cbClientForm::ROBOT:
		case cbClientForm::ROBOTBEACON:
		{
			// cout << "Robot form is going to be processed\n";
			cbRobot *robot = form.client.robot;
			if (registerRobot(robot))
			{
				robot->Reply(form.addr, form.port, param);
				cout << robot->Name() << " has been registered\n";
				gui->appendMessage(QString(robot->Name()) + " has been registered");
			}
			else // robot was refused
			{
				robot->Refuse(form.addr, form.port);
				cout << robot->Name() << " has been refused\n";
				gui->appendMessage(QString(robot->Name()) + " has been refused", true);
				delete robot;
			}
			nextState=RUNNING;
			step();
			break;
		}
		case cbClientForm::UNKNOWN:
			cerr << "UNKNOWN form was received, and discarded\n";
			gui->appendMessage("UNKNOWN form was received, and discarded", true);
			// a refused replied must be sent
			break;
		case cbClientForm::NOBODY:
			cerr << "NOBODY form was received, and discarded\n";
			gui->appendMessage("NOBODY form was received, and discarded", true);
			break;
		}
	}
}

void cbSimulator::start()
{
	if (curState == STOPPED)
	{
		nextState = RUNNING;
		emit simRunning(true);
		emit simReset(false);
	}
}

void cbSimulator::stop()
{
	if (curState == RUNNING)
	{
		nextState = STOPPED;
		emit simRunning(false);
	}
}

/*!
	Execute every pending view command.
*/
void cbSimulator::ViewCommands()
{
	cbCommand command;
	char xml[4096 * 16];
	unsigned int cnt;
	for (unsigned int i = 0; i < views.size(); i++)
	{
		while (views[i]->readCommand(&command))
		{
			switch (command.type)
			{
			case cbCommand::START:
				// cout << "View command = Start\n";
				start();
				break;
			case cbCommand::STOP:
				// cout << "View command = Stop\n";
				stop();
				break;
			case cbCommand::LABRQ:
				// cout << "View command = LabReq\n";
				cnt = lab->toXml(xml, sizeof(xml) - 1);
				// cerr << xml;
				views[i]->send(xml, cnt + 1);
				break;
			case cbCommand::GRIDRQ:
				// cout << "View command = GridReq\n";
				cnt = grid->toXml(xml, sizeof(xml) - 1);
				views[i]->send(xml, cnt + 1);
				break;
			case cbCommand::ROBOTDEL:
			{
				// cout << "View command = RobotDel\n";
				unsigned int id = command.robot.id;
				if (id >= 1 && id <= robots.size())
				{
					cbRobot *robot = robots[id - 1];
					if (robot != 0)
						robot->remove();
				}
				break;
			}
			case cbCommand::UNKNOWN:
				// cout << "View command = Unknown\n";
				break;
			}
		}
	}
}

/*!
	For each robot read and process every action message.
	These messages give new values for left and right motors
	and for the end led.
*/
void cbSimulator::RobotActions()
{
	cbRobotAction action;
	bool resetSim = false;
	for (unsigned int i = 0; i < robots.size(); i++)
	{
		cbRobot *robot = robots[i];
		if (robot == 0)
			continue;
		robot->resetReceivedFlags();
		robot->resetRequestedSensors();
		while (robot->readAction(&action))
		{
			// cerr << "Robot action received l=" << action.leftMotor
			//      << " r=" << action.rightMotor << "\n";
			if (action.leftMotorChanged)
				robot->setLeftMotor(action.leftMotor);
			if (action.rightMotorChanged)
				robot->setRightMotor(action.rightMotor);
			if (action.endLedChanged)
				robot->setEndLed(action.endLed);
			if (action.returningLedChanged)
				robot->setReturningLed(action.returningLed);
			if (action.visitingLedChanged)
				robot->setVisitingLed(action.visitingLed);

			for (unsigned int r = 0; r < action.sensorRequests.size(); r++)
			{
				robot->requestSensor(action.sensorRequests[r]);
			}

			if (action.sayReceived)
				robot->setSayMessage(action.sayMessage);

			if (action.resetSim) {
				resetSim = true;
			}

			action.sensorRequests.clear();
		}
		
		if(resetSim) { 
			reset(); 
		}

		step();
	}
}

/*!
	Make robots determine next positions in Lab.
	The robots themselves are not moved, only the new
	positions are determined.
*/
void cbSimulator::NextPositions()
{
	for (unsigned int i = 0; i < robots.size(); i++)
	{
		cbRobot *robot = robots[i];
		if (robot == 0 || robot->isRemoved())
			continue;
		robot->computeNextPosition();
#ifdef DEBUG_ROBOT
		robot->showAllAttributes();
#endif
	}
}

/*!
	Check and set which robots collide with walls or
	other robots.
*/
void cbSimulator::CheckCollisions()
{
	unsigned int n = robots.size();
	unsigned int i;

	/* reset collision flag for every robot */
	for (i = 0; i < n; i++) // for all robots
	{
		/* skip unregistered robots */
		cbRobot *robot = robots[i];
		if (robot == 0)
			continue;

		/* discard removed robots */
		if (robot->isRemoved())
			continue;

		/* reset collision flag */
		robot->resetCollision();
	}

	/* check collision with walls */
	for (i = 0; i < n; i++) // for all robots
	{
		/* skip unregistered robots */
		cbRobot *robot = robots[i];
		if (robot == 0)
			continue;

		/* discard removed robots */
		if (robot->isRemoved())
			continue;

		/* check collision with lab walls */
		if (lab->wallDistance(robots[i]->nextCenter()) < ROBOT_RADIUS)
		{
			// cout.form("Collision set (1)\n");
			// robot->setCollision();
			robot->setCollisionWall();
		}
	}

	/* check collision between robots */
	for (i = 0; i < n - 1 && !collisionsIgnored(); i++) // for all robots
	{
		/* skip unregistered robots */
		cbRobot *robot = robots[i];
		if (robot == 0)
			continue;

		/* skip removed robots */
		if (robot->isRemoved())
			continue;

		/* check collision with other robots */
		cbPoint nc = robot->nextCenter();
		for (unsigned int j = i + 1; j < n; j++)
		{
			/* skip unregistered robots */
			cbRobot *otherRobot = robots[j];
			if (otherRobot == 0)
				continue;

			/* skip removed robots */
			if (otherRobot->isRemoved())
				continue;

			/* check distance between centers */
			cbPoint onc = otherRobot->nextCenter();
			double d = nc.distance(onc);
			// cout.form("Distance netween robot centers %u and %u = %g\n", i+1, j+1, d);
			if (d >= 2 * ROBOT_RADIUS)
				continue; // too far

			/* look for collision responsability */
			if (robot->isMovingTowards(otherRobot->Center()))
			{
				robot->setCollisionRobot();
				// robot->setCollision();
				// cout.form("Collision set (2)\n");
				// robot->showAllAttributes();
			}
			if (otherRobot->isMovingTowards(robot->Center()))
			{
				otherRobot->setCollisionRobot();
				// otherRobot->setCollision();
				// cout.form("Collision set (3)\n");
				// otherRobot->showAllAttributes();
			}
		}
	}
}

/*!
	Make current the next position, except for
	colliding robots, which do not change position.
*/
void cbSimulator::Commit()
{
	for (unsigned int i = 0; i < robots.size(); i++) // for all robots
	{
		cbRobot *robot = robots[i];
		if (robot == 0)
			continue;
		if (robot->isRemoved())
			continue;

		robot->commitNextPosition();
	}
}

void cbSimulator::UpdateScores()
{
	*grAux = *graph;								 // all computation is based in grAux, graph keeps the graph nodes without final points
	for (unsigned int i = 0; i < robots.size(); i++) // for all robots
	{
		cbRobot *robot = robots[i];
        if (robot == 0) continue;

        switch(scoring) {
          case 1:
            robot->updateScoreControl();         // CONTROL
            break;
          case 4:
            robot->updateScoreLineControl2022(); // LINE CONTROL
            break;
          case 5:
          case 6:
            robot->updateScoreLineMappingPlanning2022(); // LINE MAPPING PLANNING
            break;
          case 7:
            robot->updateScoreLineControl2023(); // LINE CONTROL 2023 (lines at 45 degrees are allowed)
            break;

		default:
			robot->updateScoreCompetitive(); // COMPETITIVE
			// robot->updateScore();               // COOPERATIVE
			break;
		}
	}
}

/*!
	Update sensors for every robot not removed.
*/
void cbSimulator::UpdateSensors()
{
	unsigned int n = robots.size();
	for (unsigned int i = 0; i < n; i++)
	{
		cbRobot *robot = robots[i];
		if (robot == 0)
			continue;
		if (robot->state() == cbRobot::REMOVED)
			continue;
		robot->updateSensors();
	}
}

/*!
	Intruct each active robot to send measures to its client.
	A robot is active if it is not removed not finnished.
*/
void cbSimulator::SendSensors()
{
	unsigned int n = robots.size();
	// cerr << "SEND Sensors " << curTime() << "\n";
	for (unsigned int i = 0; i < n; i++)
	{
		cbRobot *robot = robots[i];
		if (robot == 0)
			continue;
		if (robot->state() == cbRobot::REMOVED)
			continue;
		// if (!robot->active()) continue;
		robot->sendSensors();
	}
}

/*!
	Send status of every robot to every view.
*/
void cbSimulator::UpdateViews()
{
	char xml[1024];
	for (unsigned int i = 0; i < robots.size(); i++)
	{
		cbRobot *robot = robots[i];
		if (robot == 0)
			continue;
		unsigned int n = robot->toXml(xml, sizeof(xml));
		for (unsigned int j = 0; j < views.size(); j++)
		{
			cbView *view = views[j];
			view->send(xml, n + 1);
		}

		if (showPositions)
			gui->writeOnBoard("Position of " + QString(robot->Name()) + " (robot " + QString::number(robot->Id()) + ") sent to Viewer(s):\n" + xml, (int)robot->Id(), 2);
	}
}

/*!
	Update state of simulator and update active state of every robot.
	If next state is equal to current one do nothing.
*/
void cbSimulator::UpdateState()
{
	if (simTime() <= curTime() && isTimed())
	{
		nextState = FINISHED;
		if (logging)
			Log(*logStream, false); // last loginfo item - should not contain robot actions
		closeLog();
	}

	if (nextState != curState)
	{
		if (curState == INIT)
			emit simReady(true);
		curState = nextState;
		emit stateChanged(curStateAsString());
	}

	for (unsigned int i = 0; i < robots.size(); i++)
	{
		cbRobot *robot = robots[i];
		if (robot == 0) continue;

        switch(scoring) {
          case 1:
            robot->updateStateControl();                  // CONTROL
             break;
          case 2:
            robot->updateStateMapping();                  // MAPPING
            break;
          case 3:
            robot->updateStatePlanning();                 // PLANNING
            break;
          case 4:
            robot->updateStateLineControl2022();          // LINE CONTROL
             break;
          case 5:
            robot->updateStateLineMapping2022();          // LINE MAPPING
             break;
          case 6:
            robot->updateStateLinePlanning2022();         // LINE PLANNING
             break;
          case 7:
            robot->updateStateLineControl2023();         // LINE CONTROL 2023 (lines at 45 degrees allowed)
             break;
          default:
            robot->updateStateCompetitive();    // COMPETITIVE
            //robot->updateState();               // COOPERATIVE
            break;
        }
        
	}
}

void cbSimulator::Log(ostream &log, bool withActions)
{
	unsigned int n = robots.size();
	if (curState == RUNNING)
	{
		log << "<LogInfo Time=\"" << curCycle << "\">\n";
		for (unsigned int i = 0; i < n; i++)
		{
			cbRobot *r = robots[i];
			if (r == 0)
				continue;
			r->Log(log, withActions);
		}
		log << "</LogInfo>\n";
	}
}

void cbSimulator::PanelCommands() {}

void cbSimulator::buildGraph(void)
{
	int g;

	if (graph != 0)
		delete graph;
	if (grAux != 0)
		delete grAux;

	graph = new cbGraph;

	graph->setLab(lab);

	graph->buildCornerGraph();

	assert(grid != 0);
	assert(grid->count() > 0);
	for (g = 0; g < grid->count(); g++)
		graph->addInitPoint(cbPosition(grid->at(g)).Coord());

	// graph->writeGraph();

	// test DANGEROUS - cannot be done while running a competition
	// cbPoint p(16.25,7.625);
	// graph->addFinalPoint(3,p);
	// graph->writeGraph();
	// fprintf(stderr,"graph test d=%6.3f\n",graph->dist(3));
	// exit(1);

	// fprintf(stderr,"graph set init\n");
	// graph->setInitState();  // Tentativa de optimizacao
	// fprintf(stderr,"graph setted init\n");

	grAux = new cbGraph;
}

double cbSimulator::calcDistMaxFromGridTo(cbPoint &p)
{
	int g;

	if (lab->nTargets() == 0)
		return 1e10;

	*grAux = *graph; // all computation is based in grAux, graph keeps the graph nodes without final points

	for (g = 0; g < grid->count(); g++)
		grAux->addFinalPoint(g + 1, p);

	double distMax = 0.0;
	for (g = 0; g < grid->count(); g++)
	{
		double distToGrid = grAux->dist(g + 1);
		if (distToGrid > distMax)
			distMax = distToGrid;
	}

	return distMax;
}

void cbSimulator::setDistMaxFromGridToTarget(void)
{
	if (lab->nTargets() == 0)
		return;

	distMaxToTarget = calcDistMaxFromGridTo(lab->Target(0)->Center()); // TODO: Hack para versao de 2006
																	   //  aparentemente este valor ja nao e usado para nada!!!
}

class cbGraphView : public QGraphicsView
{
public:
	cbGraphView(QGraphicsScene *scene, cbSimulator *sim);
	void mouseMoveEvent(QMouseEvent *e);

private:
	QGraphicsSimpleTextItem *distLabel;
	cbSimulator *simulator;
};

cbGraphView::cbGraphView(QGraphicsScene *scene, cbSimulator *sim) : QGraphicsView(scene)
{
	simulator = sim;

	/* distLabel = new QGraphicsSimpleTextItem(0, scene); */
	distLabel = new QGraphicsSimpleTextItem();
	distLabel->setText("AAAAA");
	distLabel->setZValue(10);
	distLabel->setPen(QPen(Qt::red));
	distLabel->setVisible(true);
	scene->addItem(distLabel);
}

void cbGraphView::mouseMoveEvent(QMouseEvent *e)
{
	double x, y, dist;
	int xi, yi;

	xi = (int)(e->x() / (simulator->labCanvasWidth / (double)GRIDSIZE));
	yi = (int)(e->y() / (simulator->labCanvasHeight / (double)GRIDSIZE));

	if (xi >= 0 && xi < GRIDSIZE && yi >= 0 && yi < GRIDSIZE)
	{
		x = (xi + 0.5) * simulator->Lab()->Width() / GRIDSIZE;
		y = (GRIDSIZE - yi - 0.5) * simulator->Lab()->Height() / GRIDSIZE;
		dist = simulator->distGrid[xi][yi];
		if (xi < GRIDSIZE / 2)
		{
			distLabel->setPos(e->x() + 10, e->y());
		}
		else
		{
			distLabel->setPos(e->x() - 200, e->y());
		}
		distLabel->setText(QString("(%1,%2) -> dist=%3").arg(x).arg(y).arg(dist));
	}
	else
	{
		distLabel->setText("Out of bounds");
	}

	scene()->update();
}

void cbSimulator::showGraph(int id)
{
	unsigned int w, c;
	QGraphicsPolygonItem *wallCanvas;
	QGraphicsRectItem *grCanvas;

	if (id < 1 || id > (int)robots.size())
	{
		cerr << "Cannot show graph of robot " << id << "\n";
		return;
	}

	labCanvasWidth = (int)(lab->Width() * 30);
	labCanvasHeight = (int)(lab->Height() * 30);

	labScene = new QGraphicsScene(0, 0, labCanvasWidth, labCanvasHeight);

	labView = new cbGraphView(labScene, this);
	labView->viewport()->setMouseTracking(true);

	QPolygon *pa;

	for (w = 1; w < lab->nWalls(); w++)
	{
		vector<cbPoint> &corners = lab->Wall(w)->Corners();
		// pa.resize(corners.size());
		pa = new QPolygon(corners.size());
		for (c = 0; c < corners.size(); c++)
			pa->setPoint(c, (int)(corners[c].X() * labCanvasWidth / lab->Width()),
						 (int)(labCanvasHeight - corners[c].Y() * labCanvasHeight / lab->Height()));

		/* wallCanvas = new QGraphicsPolygonItem(0, labScene); */
		wallCanvas = new QGraphicsPolygonItem();
		wallCanvas->setPolygon(*pa);
		wallCanvas->setBrush(QBrush(Qt::black));
		wallCanvas->setVisible(true);
		labScene->addItem(wallCanvas);
	}

	unsigned int x, y;
	double distMax = 0.0;

	// grAux=graph;  Tentativa de optimizacao
	for (x = 0; x < GRIDSIZE; x++)
		for (y = 0; y < GRIDSIZE; y++)
		{
			*grAux = *graph;

			// grAux->resetInitState();  Tentativa de optimizacao - testes indicam que fica muito mais lento!!
			// grAux->writeGraph();

			cbPoint p((0.5 + x) * lab->Width() / GRIDSIZE, (GRIDSIZE - 0.5 - y) * lab->Height() / GRIDSIZE);
			grAux->addFinalPoint(id, p);
			distGrid[x][y] = grAux->dist(id);
			if (distGrid[x][y] < 2000 && distGrid[x][y] > distMax)
				distMax = distGrid[x][y];
		}

	//        fprintf(stderr,"distmax=%f labCanvasWidth=%d labCanvasHeight=%d\n",
	//			 distMax,labCanvasWidth,labCanvasHeight);

	for (x = 0; x < GRIDSIZE; x++)
		for (y = 0; y < GRIDSIZE; y++)
		{

			if (distGrid[x][y] < 2000)
			{
				grCanvas = new QGraphicsRectItem(x * labCanvasWidth / GRIDSIZE, y * labCanvasHeight / GRIDSIZE,
												 (x + 1) * labCanvasWidth / GRIDSIZE - x * labCanvasWidth / GRIDSIZE,
												 (y + 1) * labCanvasHeight / GRIDSIZE - y * labCanvasHeight / GRIDSIZE,
												 0);
				QColor color((int)(0 + distGrid[x][y] / distMax * 250),
							 (int)(0 + distGrid[x][y] / distMax * 250),
							 (int)(0 + distGrid[x][y] / distMax * 250));
				grCanvas->setBrush(QBrush(color));
				grCanvas->setPen(QPen(color));
				grCanvas->setVisible(true);
				// debug
				// distGrid[x][y]=(int)(0+distGrid[x][y]/distMax*250);

				labScene->addItem(grCanvas);
			}
		}
	labScene->update();
}

bool cbSimulator::changeLab(QString labFilename)
{
	if (curState != INIT)
	{
		cerr << "Cannot open lab after start\n";
		gui->appendMessage("Cannot open lab after start", true);
		return false;
	}

	QXmlInputSource *source;

	// cout << " using given file...";
	QFile srcFile(labFilename);

	if (!srcFile.exists())
	{
		cerr << "Could not open " << labFilename.toStdString() << "\n";
		gui->appendMessage(QString("Could not open ") +
							   labFilename,
						   true);
		return false;
	}
	if ((source = new QXmlInputSource(&srcFile)) == 0)
	{
		cerr << "Fail sourcing lab file\n";
		gui->appendMessage("Fail sourcing lab file", true);
		return false;
	}

	QXmlSimpleReader xmlParser;

	cbLabHandler *labHandler = new cbLabHandler;
	xmlParser.setContentHandler(labHandler);

	cbLab *labnew = 0;
	if (xmlParser.parse(*source))
		labnew = labHandler->parsedLab();
	else
	{
		cerr << "Error parsing " << labFilename.toStdString() << "\n";
		gui->appendMessage(QString("Error parsing ") + labFilename, true);
		return false;
	}

	setLab(labnew);
	delete labHandler;
	delete source;

	// rebuild graph
	if (grid != 0)
	{
		buildGraph();
		// setDistMaxFromGridToTarget();
	}

	// update parameters
	param->labFilename = labFilename;

	return true;
	// cout << " done.\n";
}

bool cbSimulator::changeGrid(QString gridFilename)
{
	if (curState != INIT)
	{
		cerr << "Cannot open grid after start\n";
		gui->appendMessage("Cannot open grid after start", true);
		return false;
	}

	QXmlInputSource *source;
	QFile srcFile(gridFilename);

	if (!srcFile.exists())
	{
		cerr << "Could not open " << gridFilename.toStdString() << "\n";
		gui->appendMessage(QString("Could not open ") +
							   gridFilename,
						   true);
		return false;
	}
	if ((source = new QXmlInputSource(&srcFile)) == 0)
	{
		cerr << "Fail sourcing lab file\n";
		gui->appendMessage("Fail sourcing lab file", true);
		return false;
	}

	QXmlSimpleReader xmlParser;

	cbGridHandler *gridHandler = new cbGridHandler;
	xmlParser.setContentHandler(gridHandler);

	cbGrid *grid;
	if (xmlParser.parse(*source))
		grid = gridHandler->parsedGrid();
	else
	{
		cerr << "Error parsing " << gridFilename.toStdString() << "\n";
		gui->appendMessage(QString("Error parsing ") + gridFilename, true);
		return false;
	}

	setGrid(grid);
	delete gridHandler;
	delete source;

	// rebuild graph
	buildGraph();
	// setDistMaxFromGridToTarget();

	// update parameters
	param->gridFilename = gridFilename;

	// cout << " done.\n";
	return true;
}

bool cbSimulator::changeParameters(QString paramFilename)
{
	if (curState != INIT)
	{
		cerr << "Cannot open parameters after start\n";
		gui->appendMessage("Cannot open parameters after start", true);
		return false;
	}

	QXmlInputSource *source;

	QFile srcFile(paramFilename);

	if (!srcFile.exists())
	{
		cerr << "Could not open " << paramFilename.toStdString() << "\n";
		gui->appendMessage(QString("Could not open ") +
							   paramFilename,
						   true);
		return false;
	}
	if ((source = new QXmlInputSource(&srcFile)) == 0)
	{
		cerr << "Fail sourcing lab file\n";
		gui->appendMessage("Fail sourcing lab file", true);
		return false;
	}

	QXmlSimpleReader xmlParser;

	cbParamHandler *paramHandler = new cbParamHandler(param);
	xmlParser.setContentHandler(paramHandler);

	cbParameters *param;
	if (xmlParser.parse(*source))
		param = paramHandler->parsedParameters();
	else
	{
		cerr << "Error parsing " << paramFilename.toStdString() << "\n";
		gui->appendMessage(QString("Error parsing ") + paramFilename, true);
		return false;
	}

	setParameters(param);
	delete paramHandler;
	delete source;

	return true;
	// cout << " done.\n";
}

void cbSimulator::saveConfiguration(QString paramFilename)
{
	if (curState != INIT)
	{
		cerr << "Cannot save after start\n";
		gui->appendMessage("Cannot save after start", true);
	}

	FILE *fp = fopen(paramFilename.toLatin1().constData(), "wt");
	if (fp == 0)
	{
		cerr << "Cannot open " << paramFilename.toStdString() << " for writing\n";
		gui->appendMessage(QString("Cannot open ") + paramFilename + " for writing", true);
		return;
	}

	char buff[4096 * 16];

	param->toXml(buff, sizeof(buff));

	fprintf(fp, "%s", buff);

	fclose(fp);
}
void cbSimulator::processEditParameters(void)
{
	cycle = param->cycleTime;
	endCycle = param->simTime;

	// Noise
	cbMotor::noise = param->motorsNoise;
	cbCompassSensor::noise = param->compassNoise * M_PI / 180; // noise must be in radians
	cbBeaconSensor::noise = param->beaconNoise * M_PI / 180;   // noise must be in radians
	cbIRSensor::noise = param->obstacleNoise;
	cbGPSSensor::noiseLin = param->gpsLinNoise;
	cbGPSSensor::noiseAng = param->gpsDirNoise;
	cbLineSensor::lineTrueProb = param->lineSensorTrueProb;

	// Requests
	cbRobot::maxSensorsRequested = param->nReqPerCycle;
	cbIRSensor::sensorRequestable = param->obstacleRequestable;
	cbBeaconSensor::sensorRequestable = param->beaconRequestable;
	cbCompassSensor::sensorRequestable = param->compassRequestable;
	cbGroundSensor::sensorRequestable = param->groundRequestable;
	cbCollisionSensor::sensorRequestable = param->collisionRequestable;
	cbGPSSensor::sensorRequestable = param->gpsRequestable;

	// Latencies
	cbIRSensor::sensorLatency = param->obstacleLatency;
	cbBeaconSensor::sensorLatency = param->beaconLatency;
	cbCompassSensor::sensorLatency = param->compassLatency;
	cbGroundSensor::sensorLatency = param->groundLatency;
	cbCollisionSensor::sensorLatency = param->collisionLatency;
	cbGPSSensor::sensorLatency = param->gpsLatency;

	cbBeaconSensor::sensorAperture = param->beaconAperture;

	cbRobot::GPSOn = param->GPSOn;
	cbRobot::beaconSensorOn = param->beaconSensorOn;
	cbRobot::compassSensorOn = param->compassSensorOn;
	cbRobot::scoreSensorOn = param->scoreSensorOn;

	// Scores
	cbRobot::returnTimePenalty = param->returnTimePenalty;
	cbRobot::arrivalTimePenalty = param->arrivalTimePenalty;
	cbRobot::collisionWallPenalty = param->collisionWallPenalty;
	cbRobot::collisionRobotPenalty = param->collisionRobotPenalty;
	cbRobot::targetReward = param->targetReward;
	cbRobot::homeReward = param->homeReward;

	timer.setInterval(cycle);

	emit toggleGPS(param->GPSOn);

	emit toggleScoreSensor(param->scoreSensorOn);

	emit simTimeChanged(param->simTime);
	emit toggleTime(isTimed());
}

void cbSimulator::setDefaultLab(void)
{
	// assert(lab==0);

	QXmlInputSource *source;

	source = new QXmlInputSource;
	source->setData(QByteArray(LAB));

	QXmlSimpleReader xmlParser;

	cbLabHandler *labHandler = new cbLabHandler;
	xmlParser.setContentHandler(labHandler);

	cbLab *labnew = 0;
	if (xmlParser.parse(*source))
		labnew = labHandler->parsedLab();
	else
	{
		cerr << "Error parsing DEFAULT lab\n";
		gui->appendMessage(QString("Error parsing DEFAULT lab"), true);
		assert(0);
	}

	setLab(labnew);
	delete labHandler;
	delete source;

	// rebuild graph
	if (grid != 0)
	{
		buildGraph();
		// setDistMaxFromGridToTarget();
	}

	// update parameters
	// param->labFilename = "";

	// cout << " done.\n";
}

void cbSimulator::setDefaultGrid(void)
{
	QXmlInputSource *source;

	source = new QXmlInputSource;
	source->setData(QByteArray(GRID));

	QXmlSimpleReader xmlParser;

	cbGridHandler *gridHandler = new cbGridHandler;
	xmlParser.setContentHandler(gridHandler);

	cbGrid *grid;
	if (xmlParser.parse(*source))
		grid = gridHandler->parsedGrid();
	else
	{
		cerr << "Error parsing DEFAULT grid\n";
		gui->appendMessage(QString("Error parsing DEFAULT grid"), true);
		assert(0);
	}

	setGrid(grid);
	delete gridHandler;
	delete source;

	// rebuild graph
	if (lab != 0)
	{
		buildGraph();
		// setDistMaxFromGridToTarget();
	}

	// update parameters
	// param->gridFilename = "";
}

void cbSimulator::setDefaultParameters(void)
{
	QXmlInputSource *source;

	source = new QXmlInputSource;
	source->setData(QByteArray(SIMPARAM));

	QXmlSimpleReader xmlParser;

	cbParamHandler *paramHandler = new cbParamHandler(param);
	xmlParser.setContentHandler(paramHandler);

	cbParameters *param;
	if (xmlParser.parse(*source))
		param = paramHandler->parsedParameters();
	else
	{
		cerr << "Error parsing DEFAULT parameters\n";
		gui->appendMessage(QString("Error parsing DEFAULT parameters"), true);
		assert(0);
	}

	setParameters(param);
	delete paramHandler;
	delete source;

	// cout << " done.\n";
}

void cbSimulator::startTimer(void)
{
	timer.start(cycleTime());
	QObject::connect(&timer, SIGNAL(timeout()), this, SLOT(step()));
}

bool cbSimulator::allRobotsVisitedOrVisitingTarget(int targId)
{
	unsigned int n = robots.size();
	bool allVisit = true;
	for (unsigned int i = 0; i < n; i++)
	{
		if (robots[i] == 0)
			continue;
		if (!(robots[i]->visitedTarget(targId) || (robots[i]->isOnTarget(targId) && robots[i]->visitingLedOn())))
		{
			allVisit = false;
		}
	}

	return allVisit;
}

bool cbSimulator::allRobotsOnTarget(int targId)
{
	unsigned int n = robots.size();
	bool allOnTarget = true;
	for (unsigned int i = 0; i < n; i++)
	{
		if (robots[i] == 0)
			continue;
		if (!robots[i]->isOnTarget(targId))
		{
			allOnTarget = false;
		}
	}

	return allOnTarget;
}

void cbSimulator::deleteRobot(uint id)
{
	assert(id >= 1 && id <= robots.size());

	if (robots[id - 1] != 0)
	{
		const char *name = ((cbRobot *)robots[id - 1])->Name();
		delete robots[id - 1];
		robots[id - 1] = 0;
		gui->appendMessage(QString(name) + " has been deleted from position " + QString::number(id));

		emit robotDeleted((int)id);
	}
}

void cbSimulator::printTargetRelativePositions()
{
	if (lab == 0 || grid == 0)
		return;

	printf("\n");

	for (int p = 0; p < grid->size(); p++)
	{
		printf("Pos %d to target: (%6.2f,%6.2f)\n", p + 1,
			   lab->Target(0)->Center().X() - grid->at(p).X(),
			   lab->Target(0)->Center().Y() - grid->at(p).Y());
	}

	printf("\n");
}

void cbSimulator::setScoring(int sc)
{
	scoring = sc;
}

int cbSimulator::getScoring(void)
{
	return scoring;
}
