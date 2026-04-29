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

#ifndef CBCONTROLPANEL_H
#define CBCONTROLPANEL_H

#include <QWidget>
#include <QAction>
#include <QStringListModel>


class cbSimulator;
class cbManageRobots;

namespace Ui {
class cbControlPanel;
}

/*!
    \class cbControlPanel
    \brief Matching class for Control Panel Form
*/
class cbControlPanel : public QWidget
{
    Q_OBJECT

public:
    explicit cbControlPanel(cbSimulator *sim, QStringListModel *agents, QWidget *parent = 0, Qt::WindowFlags fl= 0);
    ~cbControlPanel();

signals:
    void closed();

protected:
    virtual void closeEvent(QCloseEvent *);

private slots:
    void setStartStopButton(bool running);
    void triggerStartStop(void);
    void resetRobSpinBox(int);

    void on_pushButton_ManageRobots_toggled(bool checked);
    void on_pushButton_Remove_clicked();

private:
    cbSimulator *simulator;
    cbManageRobots *manRobWidget;
    QStringListModel *agentModel;
    QAction *actionStart, *actionStop;
    bool startNext;
    Ui::cbControlPanel *ui;
};

#endif // CBCONTROLPANEL_H
