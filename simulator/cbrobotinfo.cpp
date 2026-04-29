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

#include "cbrobotinfo.h"
#include "ui_cbrobotinfo.h"

cbRobotInfo::cbRobotInfo(cbRobot *rob, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::cbRobotInfo)
{
    ui->setupUi(this);

    robot = rob;

    QSize scale = ui->label_State->size();
    stopPix = QPixmap(":/images/robot/states/stopped").scaled(scale);
    runPix = QPixmap(":/images/robot/states/running").scaled(scale);
    waitPix = QPixmap(":/images/robot/states/waiting").scaled(scale);
    removePix = QPixmap(":/images/robot/states/removed").scaled(scale);
    finishPix = QPixmap(":/images/robot/states/finished").scaled(scale);
    returnPix = QPixmap(":/images/robot/states/returning").scaled(scale);

    ui->label_Name->setText(robot->Name());
    ui->lcdNumber_ID->display((int) robot->Id());
    ui->label_State->setPixmap(stopPix);
    ui->label_State->setToolTip("Stopped");

    connect(robot, SIGNAL(robCollisionsChanged(int)), ui->lcdNumber_Collisions, SLOT(display(int)));
    connect(robot, SIGNAL(robScoreChanged(int)), ui->lcdNumber_Score, SLOT(display(int)));
    connect(robot, SIGNAL(robArrivalTimeChanged(int)), ui->lcdNumber_Time, SLOT(display(int)));
    connect(robot, SIGNAL(robStateChanged(cbRobot::State)), SLOT(updateState(cbRobot::State)));

}

cbRobotInfo::~cbRobotInfo()
{
    delete ui;
}

void cbRobotInfo::updateState(cbRobot::State state)
{
    switch (state)
    {
    case cbRobot::STOPPED:
        ui->label_State->setPixmap(stopPix);
        ui->label_State->setToolTip("Stopped");
        break;
    case cbRobot::RUNNING:
        ui->label_State->setPixmap(runPix);
        ui->label_State->setToolTip("Running");
        break;
    case cbRobot::WAITOTHERS:
        ui->label_State->setPixmap(waitPix);
        ui->label_State->setToolTip("Waiting");
        break;
    case cbRobot::RETURNING:
        ui->label_State->setPixmap(returnPix);
        ui->label_State->setToolTip("Returning");
        break;
    case cbRobot::FINISHED:
        ui->label_State->setPixmap(finishPix);
        ui->label_State->setToolTip("Finished");
        break;
    case cbRobot::REMOVED:
        ui->label_State->setPixmap(removePix);
        ui->label_State->setToolTip("Removed");
        break;
    }
}
