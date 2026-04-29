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

#ifndef CBMANAGEROBOTS_H
#define CBMANAGEROBOTS_H

#include <QWidget>
#include <QStringListModel>

class cbSimulator;

namespace Ui {
class cbManageRobots;
}

/*!
    \class cbManageRobots
    \brief Matching class for Robot Managing Form
*/
class cbManageRobots : public QWidget
{
    Q_OBJECT

public:
    explicit cbManageRobots(cbSimulator *sim, QStringListModel *agents, QWidget *parent = 0, Qt::WindowFlags fl= 0);
    ~cbManageRobots();

signals:
    void closed();

protected:
    virtual void closeEvent(QCloseEvent *);

private slots:
    void resetRobWidgets(int);
    void refreshPosComboBox();

    void addRobWidget(int id);
    void removeRobWidget(int id);

    void currentAgentChanged(QModelIndex,QModelIndex);
    void tryEnableAddRobot(bool registrationsAllowed);

    void on_pushButton_LoadAgent_clicked();
    void on_pushButton_RemoveAgent_clicked();
    void on_comboBox_Position_currentIndexChanged(int index);
    void on_pushButton_AddRobot_clicked();
    void on_tableWidget_Robots_itemSelectionChanged();
    void on_pushButton_DeleteRobot_clicked();

private:
    cbSimulator *simulator;
    QStringListModel *agentModel;
    QString agentPath;
    int currentAgentRow;
    int nRobots;
    Ui::cbManageRobots *ui;
};

#endif // CBMANAGEROBOTS_H
