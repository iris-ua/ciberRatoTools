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

#include "cbcontrolpanel.h"
#include "ui_cbcontrolpanel.h"
#include "cbsimulator.h"
#include "cbmanagerobots.h"
#include "cbrobot.h"

#include <QtCore>
#include <QtGui>

cbControlPanel::cbControlPanel(cbSimulator *sim, QStringListModel *agents, QWidget *parent, Qt::WindowFlags fl) :
    QWidget(parent, fl),
    simulator(sim),
    ui(new Ui::cbControlPanel)
{
    ui->setupUi(this);

    startNext = true;

    actionStart = new QAction(this);
    actionStop = new QAction(this);
    connect(actionStart, SIGNAL(triggered()), simulator, SLOT(start()));
    connect(actionStop, SIGNAL(triggered()), simulator, SLOT(stop()));

    connect(ui->pushButton_StartStop, SIGNAL(clicked()), SLOT(triggerStartStop()));
    connect(ui->pushButton_Reset, SIGNAL(clicked()), simulator, SLOT(reset()));

    ui->pushButton_Time->setChecked(simulator->isTimed());
    ui->pushButton_Lock->setChecked(simulator->isRegistrationAllowed());
    ui->pushButton_Collisions->setChecked(!simulator->collisionsIgnored());
    connect(simulator, SIGNAL(toggleTime(bool)), ui->pushButton_Time, SLOT(setChecked(bool)));
    connect(simulator,SIGNAL(toggleRegistrations(bool)), ui->pushButton_Lock, SLOT(setChecked(bool)));
    connect(simulator, SIGNAL(toggleCollisions(bool)), ui->pushButton_Collisions, SLOT(setChecked(bool)));

    connect(ui->pushButton_Time, SIGNAL(toggled(bool)), simulator, SLOT(setTime(bool)));
    connect(ui->pushButton_Lock, SIGNAL(toggled(bool)), simulator, SLOT(setRegistrations(bool)));
    connect(ui->pushButton_Collisions, SIGNAL(toggled(bool)), simulator, SLOT(setCollisions(bool)));

    connect(simulator, SIGNAL(simReady(bool)), ui->pushButton_StartStop, SLOT(setEnabled(bool)));
    connect(simulator, SIGNAL(simReset(bool)), ui->pushButton_Lock, SLOT(setDisabled(bool)));
    connect(simulator, SIGNAL(simReset(bool)), ui->pushButton_Lock, SLOT(setChecked(bool)));
    connect(simulator, SIGNAL(simRunning(bool)), SLOT(setStartStopButton(bool)));

    ui->spinBox_Robot->setMinimum(1);
    ui->spinBox_Robot->setMaximum(simulator->Robots().size());
    connect(simulator, SIGNAL(gridChanged(int)), SLOT(resetRobSpinBox(int)));

    manRobWidget = new cbManageRobots(simulator, agents);
    ui->gridLayout->addWidget(manRobWidget,1,0,1,2);
    /* manRobWidget->setShown(false); */
    manRobWidget->hide();

    connect (manRobWidget, SIGNAL(closed()), ui->pushButton_ManageRobots, SLOT(toggle()));
    adjustSize();
}

cbControlPanel::~cbControlPanel()
{
    delete ui;
}

void cbControlPanel::setStartStopButton(bool running)
{
    if (running)
    {
        ui->pushButton_StartStop->setIcon(QIcon(":/images/control/stop"));
        ui->pushButton_StartStop->setToolTip("Stop simulation");
    }
    else
    {
        ui->pushButton_StartStop->setIcon(QIcon(":/images/control/start"));
        ui->pushButton_StartStop->setToolTip("Start simulation");
    }
    startNext = !running;
}

void cbControlPanel::triggerStartStop(void)
{
    if (startNext)
        actionStart->trigger();
    else
        actionStop->trigger();
}

void cbControlPanel::on_pushButton_ManageRobots_toggled(bool checked)
{
    if (checked)
        manRobWidget->show();
    else
        manRobWidget->hide();
    adjustSize();
}

void cbControlPanel::resetRobSpinBox(int size)
{
    ui->spinBox_Robot->setMaximum(size);
}

void cbControlPanel::on_pushButton_Remove_clicked()
{
    unsigned int id = ui->spinBox_Robot->value();
    if (id >= 1 && id <= simulator->Robots().size())
    {
        cbRobot *rob = simulator->Robots()[id-1];
        if (rob != 0)
            rob->remove();
    }
}

void cbControlPanel::closeEvent(QCloseEvent *)
{
    emit closed();
}
