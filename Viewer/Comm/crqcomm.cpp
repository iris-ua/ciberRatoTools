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

#ifdef MicWindows

#include <WinSock2.h>

#else

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <unistd.h>
#include <netdb.h>

#endif

#include <iostream>
#include <QString>
#include <qxml.h>
#include <QObject>
#include "crqcomm.h"
#include "crqcommhandler.h"
#include "crqreplyhandler.h"

#include <QtGui>

using namespace std;

CRQComm::CRQComm()
{

}

/*============================================================================*/

CRQComm::CRQComm(CRQLabView *lb, CRQScene *commScene, CRLab *commLab,
  QString h, unsigned short port_, const char c , const char autoC,
  const char autoS)
    : QUdpSocket(), timer(this)
{
    scoreLayout = lb->findChild<QVBoxLayout *>("scoreLayout");
	autoConnect = autoC;
	autoStart = autoS;
    control = c;
	dataView = NULL;
	port = port_;
    scene = commScene;	// Scene passed by main
    lab = commLab;		// Lab passed by main
	cont = 0;
	host = h;
	skinFName = lb->getSkin();
    isConnected = false;

    QObject::connect (this, SIGNAL(readyRead()), SLOT(dataControler()));

	struct hostent *host_ent ;
    struct in_addr *addr_ptr ;

    QByteArray hostLatin = host.toLatin1();
    if ((host_ent = (struct hostent *)gethostbyname(hostLatin.constData())) == NULL)
    {
        /* Check if a numeric address */
        if (inet_addr(hostLatin.constData()) == INADDR_NONE)
        {
            cerr << "Invalid server address\n";
            exit(-1);
        }
    }
    else
    {
        addr_ptr = (struct in_addr *) *host_ent->h_addr_list ;
        host = QString(QLatin1String( inet_ntoa(*addr_ptr) )) ;
    }

    serverAddress.setAddress( host ); //Server address
	
	QHostAddress localAddress;			//Local Address
    localAddress.setAddress( QString( "127.0.0.1" ));

    if( bind( localAddress, 0 ) == FALSE )	//Bind for local address
    {
		cerr << "Failed to assign address" << endl;
        exit (-1);
    }

	if(autoConnect == 'y')
		this->connect();

   	// Connect Alarm event
    QObject::connect(&timer, SIGNAL(timeout()), SLOT(SendRequests()));
}

/*============================================================================*/

void CRQComm::connect(void)
{
    if (isConnected)
        return;

#ifdef DEBUG
    cout << "CRQComm::connect\n";
#endif

    //Registration <VIEW/>
    QObject::disconnect(this, SIGNAL(readyRead()), this, SLOT(dataControler()));
    QObject::connect(this, SIGNAL(readyRead()), SLOT(replyControler()));

	port=6000; // CORRIGIR!!!
    if( writeDatagram("<View/>\n", 9, serverAddress, port ) == -1 )
    {
		cerr << "Failure when writting <View/>" << endl;
        exit (-1);
    }

}

/*============================================================================*/

void CRQComm::replyControler()
{
#ifdef DEBUG
    cout << "CRQComm::replyControler\n";
#endif

    //char data[16384];
	//Read confirmation <REPLY STATUS="ok/refuse".../>
	
    //cerr << "reply controller \n";

    while (hasPendingDatagrams())
    {
        QByteArray datagram;
        datagram.resize(pendingDatagramSize());
        if( readDatagram( datagram.data(), datagram.size(), &serverAddress, &port ) == -1 )
        {
            cerr << "Failure to read confirmation from the socket " << endl;
            exit (-1);
        }
        QXmlInputSource source;
        source.setData( QString( datagram.data() ) );

        /* set replyHandler */
        CRQReplyHandler replyHandler;

        /* parse reply message with replyHandler */
        QXmlSimpleReader reader;
        reader.setContentHandler(&replyHandler);
        if(reader.parse(source))
        {
            reply = replyHandler.reply(); // The status of request
            if( reply->status )   // REPLY = "ok"
            {
                QObject::disconnect(this, SIGNAL(readyRead()), this, SLOT(replyControler()));
                QObject::connect(this, SIGNAL(readyRead()), SLOT(dataControler()));

                // Initialize timer
                timer.setSingleShot(true);
                if(autoConnect == 'y')
                    timer.start(500);
                else
                    timer.start(3000);

                isConnected = true;
                // emit signal
                emit viewerConnected(true);
                return;
            }
            else
            {
                cerr << "Connection refused.\n";
                exit(1);
            }
        }
        else
        {
            cerr << "Invalid Reply message\n";
        }
    }


    isConnected = false;
    emit viewerConnected(false);
}

/*============================================================================*/

void CRQComm::SendRequests()
{
#ifdef DEBUG
    cout << "CRQComm::sendRequests\n";
#endif

    //cerr << "Send requests \n";
    //Lab Request
    if( writeDatagram( "<LabReq/>", 10, serverAddress, port ) == -1 )
    {
		cerr << "Failure writting <LabReq/>" << endl;
        exit (-1);
    }
	//Grid Request
    if( writeDatagram( "<GridReq/>", 11, serverAddress, port ) == -1 )
    {
		cerr << "Failure writting <GRID_REQ/>" << endl;
        exit (-1);
    }

	if(autoStart == 'y' && autoConnect == 'y')
        this->sendMessage("<Start/>");

}

void CRQComm::resetSimulator(void)
{
    if (!isConnected)
        return;

#ifdef DEBUG
    cout << "CRQComm::resetSimulator\n";
#endif

    if( writeDatagram("<Reset/>\n", 9, serverAddress, port ) == -1 )
    {
		cerr << "Failure when writting <Reset/>" << endl;
        exit (-1);
    }

}

/*============================================================================*/


/*============================================================================*/

CRQComm::~CRQComm()
{

}

/*============================================================================*/

void CRQComm::dataControler() //Called when the socket receive something
{
    //char data[16384];

    while (hasPendingDatagrams())
    {
        QByteArray datagram;
        datagram.resize(pendingDatagramSize());
        if (readDatagram( datagram.data(), datagram.size(), &serverAddress, &port) == -1 ) //Read from socket
        {
            cerr << "Failure to read socket " << endl;
        }

        QXmlInputSource source;
        source.setData( QString( datagram.data() ) );
        // set commHandler for LAB, GRID and ROBOT
        CRQCommHandler commHandler;

        // parse received message with commHandler
        QXmlSimpleReader reader;
        reader.setContentHandler(&commHandler);
        if(reader.parse(source))
        {
            objectReceived = commHandler.objectType(); //Object type received

            switch (objectReceived)
            {
                case CRQCommHandler::LAB:
                {
#ifdef DEBUG
                    cout << "CRQComm::dataControler -- Lab\n";
#endif

                    if (lab != NULL )   // With this we can replace the last
                    {				//lab received
                        scene->clear();//function that delete all canvasItems
                        closeWindows();
                    }

                    lab = commHandler.getLab(); // Lab given by handler
                    scene->skin(skinFName);
                    scene->drawLab( lab );		// Draw lab in scene
                    //scene->update();
                    // the score window
                    dataView = new CRQDataView( reply, lab, skinFName, 0);
                    scoreLayout->addWidget(dataView, 1, Qt::AlignTop);
                    dataView->show();

                    // the control window - not supported / no need
                    /*commControlPanel = new CRQControlPanel( scene, this,
                                skinFName, mainWindow->soundStatus, mainWindow,
                                "Control Panel");
                    if(control == 'y')
                        commControlPanel->show();*/

                    break;
                }

                case CRQCommHandler::GRID:
                {
                    grid = commHandler.getGrid(); // Grid given by handler
                    lab->addGrid( grid );		  // Add grid to lab
                    scene->drawGrid( lab );	  // Draw grid in scene
                    //scene->update();
                    break;
                }

                case CRQCommHandler::ROBOT:
                {
                    robot = commHandler.getRobot(); // Robot given by handler
                    lab->addRobot( robot );
                    scene->drawRobot( lab );		// Draw a robot in scene
                    //scene->update();
                    if (dataView != NULL)
                        dataView->update( robot );	// update the info about robot
                    break;
                }

                case CRQCommHandler::RESTART:
                {
                    //FOR sync closeWindows();
                    break;
                }

                case CRQCommHandler::UNKNOWN:
                {
                    break;
                }

            } // End of switch (selecciona o objecto recebido)

        } // End of if (caso o parser tenha funcionado)

        else
        {
            cerr << "Invalid message\n";
        }
    }
}

//=============================================================================
void CRQComm::sendMessage( const char *mensagem )
{
    if( writeDatagram( mensagem, strlen(mensagem) + 1, serverAddress, port ) == -1 )
    {
		cerr << "Failure writting msg" << endl;
        exit (-1);
    }
}

//=============================================================================
void CRQComm::closeWindows(void)
{
#ifdef DEBUG
    cout << "CRQComm::closeWindows\n";
#endif

    if (dataView != NULL)
    {
        //dataView->close();
        dataView->hide();
        delete dataView;
        dataView = NULL;

        isConnected = false;
        emit viewerConnected(false);
    }
}

//=============================================================================
void CRQComm::skin(QString skinFileName)
{
    skinFName = skinFileName;
}
