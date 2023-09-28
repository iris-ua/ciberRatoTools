/*
    This file is part of ciberRatoToolsSrc.

    Copyright (C) 2001-2015 Universidade de Aveiro

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

#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <signal.h>

#include <qapplication.h>
#include <qtimer.h>
#include <qpushbutton.h>
#include <QGraphicsScene>
#include <qmessagebox.h>
#include <QTextEdit>
#include <QLocale>

#ifndef MicWindows

#include <sys/time.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#else

#include <process.h>

#endif

#include <stdio.h>

#include <qxml.h>
#include <qstring.h>
#include <qfile.h>
#include <qvector.h>

#include "cblogplayer.h"
#include "cbloghandler.h"
#include "cblab.h"
#include "cblabhandler.h"
#include "cbgridhandler.h"
#include "cbparamhandler.h"
#include "cbreceptionist.h"
#include "cbgrid.h"

#include "ui_logplayerGUI.h"

#define LOG "simulation.log"

using std::cerr;


/*!
	\var logplayer
	\brief The main object of the logplayer.
*/
static cbLogplayer logplayer;


/*!
	\brief Xml tools, a parser and a source.
*/
//static QXmlInputSource xmlSource;	// source for xml data
static QXmlSimpleReader xmlParser;	// parser for xml data



/*
 * This is necessary in order the static data member becomes
 * real before the existence of any object of the class.
 */
void CommandLineError()
{
    QMessageBox::critical(0,"Error", "SYNOPSIS: logplayer [-log file] [-port portnumber]\n",
                          QMessageBox::Ok,Qt::NoButton,Qt::NoButton);
    exit(1);
}


/*
 * This program accepts the following command line options
 * -lab string: name of file with lab description (default, stdin);
 * -grid string: name of file with start position grid (default, stdin);
 * -log string: name of log file (default, server.log);
 * -nc real: compass noise coeficient (default, 0.0);
 * -nb real: beacon noise coeficient (default, 0.0);
 * -ni real: infrared noise coeficient (default, 0.0);
 * -nm real: motors noise coeficient (default, 0.0);
 * -st integer: simulation time, in number of cycle time units (default, 3000);
 * -ct integer: cycle time, in miliseconds (default, 75).
 */
int main(int argc, char *argv[])
{
	/* Copyright and wellcome message */
	printf(" CiberRato 2015 Logplayer\n Copyright (C) 2003-2015 Universidade de Aveiro\n");

	/* extract option values */
	char *logFilename = 0;
	int port = 6000;

    QApplication app(argc,argv);


    //cout << "Parsing command line..."

    int p=1;
	while (p < argc) 
	{
        if (strcmp(argv[p], "-log") == 0) {
            if (p+1 < argc) {
                logFilename = argv[p+1];
                p+=2;
            }
            else CommandLineError();
		}
        else if (strcmp(argv[p], "-port") == 0)	{
            if (p+1 < argc) {
                sscanf(argv[p+1], "%d", &port);
                p+=2;
            }
            else CommandLineError();
		}
        else {
            CommandLineError();
		}
	}
	
	//cout << " done.\n";

	/* create lab object and parse lab file */
	//cout << "Creating lab object...";
	QXmlInputSource *source;

    if(logFilename) {
        QFile srcFile(logFilename);

        if(!srcFile.exists())
        {
            cerr << "Could not open log file " << logFilename << "\n";
            QMessageBox::critical(0,"Error", QString("Could not open log file ") + logFilename,
                                  QMessageBox::Ok,Qt::NoButton,Qt::NoButton);
            return 1;
        }
        if ((source = new QXmlInputSource(&srcFile)) == 0)
        {
            cerr << "Fail sourcing log file\n";
                        QMessageBox::critical(0,"Error", QString("Failed sourcing log file "),
                               QMessageBox::Ok,Qt::NoButton,Qt::NoButton);
            return 1;
        }

        cbLogHandler *logHandler = new cbLogHandler(&xmlParser);
        xmlParser.setContentHandler(logHandler);

        if( ! xmlParser.parse(*source) )
        {
            cerr << "Error parsing log file\n";
                        QMessageBox::critical(0,"Error", QString("Error parsing log file"),
                                              QMessageBox::Ok,Qt::NoButton,Qt::NoButton);
            return 1;
        }
        vector< vector <cbRobot> > *log = logHandler->parsedLog();
        logplayer.setLog(log);
        if(logHandler->getLab()!=0)
            logplayer.setLab(logHandler->getLab());
        else
        {
            cerr << "Error parsing log file\n";
            QMessageBox::critical(0,"Error", QString("Error parsing log file"),
                                  QMessageBox::Ok,Qt::NoButton,Qt::NoButton);
            return 1;
        }
        if(logHandler->getGrid()!=0)
            logplayer.setGrid(logHandler->getGrid());
        else
        {
            cerr << "Error parsing log file\n";
            QMessageBox::critical(0,"Error", QString("Error parsing log file"),
                                  QMessageBox::Ok,Qt::NoButton,Qt::NoButton);
            return 1;
        }
        if(logHandler->getParameters()!=0)
            logplayer.setParameters(logHandler->getParameters());
        else
        {
            cerr << "Error parsing log file\n";
            QMessageBox::critical(0,"Error", QString("Error parsing log file"),
                                  QMessageBox::Ok,Qt::NoButton,Qt::NoButton);
            return 1;
        }
        delete logHandler;
        delete source;
    }
    else {
        CommandLineError();
        return 1;
    }

	/* preparing the receptionist */
	//cout << "Launching the receptionist...";
	cbReceptionist *receptionist = new cbReceptionist(port);
	if (receptionist == 0 || receptionist->bad())
    {
        QMessageBox::critical(0,"Error", QString("Error creating socket"),
                              QMessageBox::Ok,Qt::NoButton,Qt::NoButton);
		//cerr << "fail\n";
		return 1;
	}
	receptionist->setXmlParser(&xmlParser);
	logplayer.setReceptionist(receptionist);
	//cout << " done.\n";
	
	/* preparing the random generator */
#ifndef MicWindows
	srand(getpid());
#else
	srand(_getpid());
#endif

    /* preparing ALARM timer */
	QTimer timer;
	timer.start(logplayer.cycleTime());
	QObject::connect(&timer,SIGNAL(timeout()),&logplayer,SLOT(step()));

    QWidget *gui = new QWidget;
    Ui::logplayerGUI ui;
    ui.setupUi(gui);

    QObject::connect(ui.QuitButton,SIGNAL(clicked()),&app,SLOT(quit()));
    logplayer.setGUI(&ui);
    gui->setMaximumSize(gui->size());
    gui->setMinimumSize(gui->size());
    gui->show();

    setlocale(LC_ALL,"C");
    QLocale::setDefault(QLocale::c());


	app.exec();

	return 0;
}
