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

#include "cbmanagerobots.h"
#include "ui_cbmanagerobots.h"

#include "cbsimulator.h"
#include "cbrobot.h"

#include <QtCore>
#include <QtGui>
#include <QFileDialog>

cbManageRobots::cbManageRobots(cbSimulator *sim, QStringListModel *agents, QWidget *parent, Qt::WindowFlags fl) :
    QWidget(parent, fl),
    simulator(sim),
    ui(new Ui::cbManageRobots)
{
    ui->setupUi(this);

    agentModel = agents;
    ui->listView_Agents->setModel(agentModel);

    connect(simulator, SIGNAL(toggleRegistrations(bool)), SLOT(tryEnableAddRobot(bool)));

    connect(simulator, SIGNAL(gridChanged(int)), SLOT(resetRobWidgets(int)));

    connect(ui->listView_Agents->selectionModel(),
            SIGNAL(currentRowChanged(QModelIndex,QModelIndex)), SLOT(currentAgentChanged(QModelIndex,QModelIndex)));

    resetRobWidgets(simulator->Robots().size());

    connect(simulator, SIGNAL(robotRegistered(int)), SLOT(addRobWidget(int)));
    connect(simulator, SIGNAL(robotDeleted(int)), SLOT(removeRobWidget(int)));

}

cbManageRobots::~cbManageRobots()
{
    delete ui;
}

void cbManageRobots::resetRobWidgets(int size)
{
    nRobots = size;

    ui->tableWidget_Robots->clear();

    ui->comboBox_Position->clear();

    QStringList headerLabels;
    headerLabels << "Id" << "Name" << "State";
    ui->tableWidget_Robots->setColumnCount(3);
    ui->tableWidget_Robots->setRowCount(nRobots);
    ui->tableWidget_Robots->setColumnWidth(0,30);
    ui->tableWidget_Robots->setColumnWidth(1,120);
    ui->tableWidget_Robots->setColumnWidth(2,60);
    ui->tableWidget_Robots->setHorizontalHeaderLabels(headerLabels);

    for (int i = 0; i < nRobots; i++)
    {
        QTableWidgetItem *idItem = new QTableWidgetItem(QString::number(i+1));
        idItem->setFlags(Qt::ItemIsSelectable);
        QTableWidgetItem *nameItem = new QTableWidgetItem("--Free--");
        nameItem->setFlags(Qt::ItemIsSelectable);

        ui->tableWidget_Robots->setItem(i, 0, idItem);
        ui->tableWidget_Robots->setItem(i, 1, nameItem);

        QLabel *state = new QLabel("");
        state->setEnabled(false);
        ui->tableWidget_Robots->setCellWidget(i,2,state);

        ui->comboBox_Position->addItem(QString::number(i + 1));
    }


}

void cbManageRobots::addRobWidget(int id)
{
    cbRobot *robot = simulator->Robots()[id-1];
    if (id >= 1 && id <= nRobots && robot != 0)
    {
        ui->tableWidget_Robots->item(id-1,0)->setFlags(Qt::ItemIsSelectable |  Qt::ItemIsEnabled);
        ui->tableWidget_Robots->item(id-1,1)->setText(QString(robot->Name()));
        ui->tableWidget_Robots->item(id-1,1)->setFlags(Qt::ItemIsSelectable |  Qt::ItemIsEnabled);

        QLabel *state = dynamic_cast<QLabel *> (ui->tableWidget_Robots->cellWidget(id-1, 2));
        state->setEnabled(true);
        state->setText(robot->curStateAsString());
        connect(robot, SIGNAL(robStateChanged(QString)), state, SLOT(setText(QString)));

        refreshPosComboBox();
    }
}

void cbManageRobots::removeRobWidget(int id)
{
    if (id >= 1 && id <= nRobots && simulator->Robots()[id-1] == 0)
    {
        ui->tableWidget_Robots->item(id-1,0)->setFlags(Qt::ItemIsSelectable);
        ui->tableWidget_Robots->item(id-1,1)->setText("--Free--");
        ui->tableWidget_Robots->item(id-1,1)->setFlags(Qt::ItemIsSelectable);
        QLabel *state = dynamic_cast< QLabel * > (ui->tableWidget_Robots->cellWidget(id-1, 2));
        state->setEnabled(false);
        state->setText("");

        refreshPosComboBox();
    }
}

void cbManageRobots::refreshPosComboBox(void)
{
    ui->comboBox_Position->clear();
    for (int i = 0; i < nRobots; i++)
    {
        if (simulator->Robots()[i] == 0)
            ui->comboBox_Position->addItem(QString::number(i + 1));
    }
}

void cbManageRobots::on_pushButton_LoadAgent_clicked()
{
    const QString robPath = QFileDialog::getOpenFileName(this, "Load a Robotic Agent", "../", "*");

    if (!robPath.isEmpty())
    {
        QStringList data = agentModel->stringList();
        if (!data.contains(robPath))
        {
            data.append(robPath);
            agentModel->setStringList(data);
        }

        int row = data.indexOf(robPath);

        ui->listView_Agents->setCurrentIndex(agentModel->index(row));
    }
}

void cbManageRobots::on_pushButton_RemoveAgent_clicked()
{
    int row;
    if ((row = ui->listView_Agents->currentIndex().row()) >= 0)
        agentModel->removeRow(row);

    if (agentModel->rowCount() == 0)
    {
        ui->groupBox_AddRobot->setEnabled(false);
        ui->pushButton_RemoveAgent->setEnabled(false);
    }
}

void cbManageRobots::currentAgentChanged(QModelIndex current, QModelIndex)
{
    if (current.isValid() && current.row() >= 0)
    {
        agentPath = agentModel->stringList().at(current.row());
        QFileInfo fileInf(agentPath);
        QString agentName(fileInf.baseName());
        ui->lineEdit_Name->setText(QDir::home().dirName() + "_" + agentName);
        ui->groupBox_AddRobot->setEnabled(simulator->isRegistrationAllowed());
        ui->label_Agent->setText(agentName);
        ui->pushButton_RemoveAgent->setEnabled(true);
    }
    else
    {
        ui->groupBox_AddRobot->setEnabled(false);
        ui->label_Agent->setText("NO AGENT SELECTED");
        ui->pushButton_RemoveAgent->setEnabled(false);
    }
}

void cbManageRobots::on_comboBox_Position_currentIndexChanged(int index)
{
    ui->pushButton_AddRobot->setEnabled(index >= 0);
}

void cbManageRobots::on_pushButton_AddRobot_clicked()
{
    QStringList robotArgs;
    robotArgs << "-pos" << ui->comboBox_Position->currentText();

    QString robName = ui->lineEdit_Name->text().toUtf8().constData();
    if (!robName.isEmpty())
        robotArgs << "-robname" << robName;

    QProcess *robProcess = new QProcess(this);
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();

#ifndef MicWindows
    env.insert("LD_LIBRARY_PATH", "../libRobSock/");
#endif
    robProcess->setProcessEnvironment(env);
    robProcess->start(agentPath, robotArgs);
    robProcess->waitForStarted(1000);
}

void cbManageRobots::on_tableWidget_Robots_itemSelectionChanged()
{
    if (ui->tableWidget_Robots->selectedItems().count() > 0)
        ui->pushButton_DeleteRobot->setEnabled(true);
    else
        ui->pushButton_DeleteRobot->setEnabled(false);
}

void cbManageRobots::on_pushButton_DeleteRobot_clicked()
{
    for (int i = 0; i < ui->tableWidget_Robots->rowCount(); i++)
    {
        QTableWidgetItem *curItem = ui->tableWidget_Robots->item(i,0);
        if (curItem->isSelected())
            simulator->deleteRobot(i+1);
    }
    ui->tableWidget_Robots->clearSelection();
}

void cbManageRobots::closeEvent(QCloseEvent *)
{
    emit closed(); // needed for toggling button back
}

void cbManageRobots::tryEnableAddRobot(bool registrationsAllowed)
{
    ui->groupBox_AddRobot->setEnabled(registrationsAllowed && ui->listView_Agents->currentIndex().row() >= 0);
}
