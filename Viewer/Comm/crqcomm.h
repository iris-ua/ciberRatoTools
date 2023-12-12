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

#ifndef _CIBER_RATO_CRQCOMM_
#define _CIBER_RATO_CRQCOMM_
/*! \class CRQComm
 * 	\brief Implementation of the comunication class.
 *
 * 	This class is responsible for all conections in the viewer.
 * 	It creates one socket and waits for simulator messages.
 * 	When a new message is received, it calls the respective parser.
 * 	After that, the respective functions of scene, lab and dataview
 * 	are called,	according to the type of information received.
 */

#include <QUdpSocket>
#include <QHostAddress>
#include <QTimer>
#include "../Lab/crlab.h"
#include "../Lab/crrobot.h"
#include "../Lab/crgrid.h"
#include "../crqscene.h"
#include "../crqdataview.h"
#include "../crqlabview.h"
#include "crreply.h"
#include "crqcommhandler.h"

#include <QtGui>
#include <QVBoxLayout>

class CRQLabView;

class CRQComm : public QUdpSocket
{
Q_OBJECT
public:
	/*!	This is the constructor.
	 *	\param commScene the scene this class will work with.
	 *	\param commLab the lab this class will work with.
	 *	\param host the address of simulator.
	 *	\param port the port of simulator.
	 */
    CRQComm(CRQLabView *lb, CRQScene *commScene, CRLab *commLab,
	  QString host, unsigned short port, const char c, const char autoC,
	  const char autoS);
	/*! This is the destructor.
	 */
	~CRQComm();

	/*! One constructor without parameters.
	 */
	CRQComm( );

	 /*! This function will send information to simulator. It will be used by
	 *  the control panel.
	 * \param msg the message to send.
	 */
	void sendMessage( const char * msg );

	/*! Send the registration message to simulator and and wait for response.
	 * If OK, the aplication continue. Also connect the signal of notifier
	 * to slot replyControler.
	 * \sa replyControler
	 */
	void connect(void);

	/*! Send the reset message to simulator 
	 */
	void resetSimulator(void);

	/*! Close the score and control windows.
	 */
	void closeWindows(void);

	/*! This method send the skin information to dataControler
	 */
	void skin(QString skinFileName);

    CRQDataView *dataView;

signals:
    void viewerConnected(bool);

public slots:
	/*! Function called by the notifier to process the received information.
	 */
    void dataControler();

	/*! Function called by notifier to process the received information.
	 * If the reply is OK, this function connect the signal of notifier
	 * to the slot dataControler.
	 * \sa dataControler
	 */
    void replyControler();
	/*! Process the Lab and Grid requests.
	 */
    void SendRequests(void);
private:
    char control; // Aparecimento ou n�o da janela de controlo (not supported / no need)
	char autoConnect;
    char autoStart;
	CRRobot *robot;
	int cont;
	CRGrid *grid;
	CRReply *reply;
	CRQScene *scene;			//Scene
	CRLab *lab;					//Lab
	QHostAddress serverAddress; //Server Address
	unsigned short port;
	CRQCommHandler::Type objectReceived;
	QString host;
    QTimer timer;
    QString skinFName;
    QVBoxLayout *scoreLayout;
    bool isConnected;

};


#endif
