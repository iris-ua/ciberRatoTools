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

#ifndef _CIBER_RATO_CRQDATAVIEW_
#define _CIBER_RATO_CRQDATAVIEW_

/*! \class CRQDataView
 * 	\brief Implementation of the window to show the score information.
 *
 * 	This class will create one widget to show the time, scores, robots,
 * 	icons, etc.
 */

#include <QWidget>

#include "Lab/crlab.h"
#include "Comm/crreply.h"
#include "crqrobotinfo.h"

namespace Ui {
class CRQDataView;
}

class CRQDataView : public QWidget
{
    Q_OBJECT
    
public:
    /*!	This is the constructor.
     */
    explicit CRQDataView(CRReply *, CRLab *, QString, QWidget *parent = 0);
    /*!	This is the destructor.
     */
    ~CRQDataView();

    /*! This function will call the update function for the respective robot
     * received.
     */
    void update( CRRobot *);

    /*!	Function to change skin
     */
    void skin(QString skinFileName);

signals:
    /*! Signal emited everytime a new message is received.
     * It is used to update the time display.
     */
    void timeRemaining( int ); // Signal to update the time

protected:
    CRRobot::States robStatus;

private:
    int nRobots; 					 // number of robots
    int *usedId;					 // used robots ID
    CRLab *lab;
    CRReply *reply;
    vector< CRQRobotInfo *> robots;
    char backFile[64];
    char backFile2[64];
    char clockImage[64];
    char clockImage2[64];
    int finalTime, curTime;
    char skinName[32];
    int status;
    Ui::CRQDataView *ui;
};

#endif // _CIBER_RATO_CRQDATAVIEW_
