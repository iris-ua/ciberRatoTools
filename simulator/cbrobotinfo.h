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

/**
 * @author Miguel Rodrigues
 */

#ifndef CBROBOTSCORE_H
#define CBROBOTSCORE_H

#include <QWidget>
#include <cbrobot.h>

namespace Ui {
class cbRobotInfo;
}

/*!
    \class cbRobotInfo
    \brief Matching class for Robot info Widget
*/
class cbRobotInfo : public QWidget
{
    Q_OBJECT
    
public:
    explicit cbRobotInfo(cbRobot *, QWidget *parent = 0);
    ~cbRobotInfo();

private slots:
    void updateState(cbRobot::State state);

private:
    cbRobot *robot;
    QPixmap runPix;
    QPixmap waitPix;
    QPixmap stopPix;
    QPixmap returnPix;
    QPixmap removePix;
    QPixmap finishPix;
    Ui::cbRobotInfo *ui;
};

#endif // CBROBSCORE_H
