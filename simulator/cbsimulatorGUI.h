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
 * \file cbsimulatorGUI.h
 *
 * Interface of application main window.
 *
 * @author Miguel Rodrigues
 */

#ifndef CBSIMULATORGUI_H
#define CBSIMULATORGUI_H

#include <QMainWindow>
#include <QProcess>
#include <QStringListModel>

using std::vector;

class cbSimulator;
class cbManageRobots;
class cbControlPanel;
class cbLabDialog;
class cbRobotInfo;
class QLabel;
class QLCDNumber;

namespace Ui {
    class cbSimulatorGUI;
}

/**
 * the simulator interface main window
 */

class cbSimulatorGUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit cbSimulatorGUI( cbSimulator *sim, QWidget* parent = 0, Qt::WindowFlags fl = 0 );
    ~cbSimulatorGUI();

    void appendMessage(const QString msg, const bool isErr = 0);
    void writeOnBoard(const QString msg, int id, int type);

private:
    cbSimulator *simulator;
    QProcess viewerProc;
    QStringListModel *agentModel;
    cbManageRobots *manRobDialog;
    cbControlPanel *controlPanel;
    cbLabDialog *labDialog;

    QLabel *stateLabel;
    QLabel *labLabel;
    QLabel *curTimeLabel;
    QLabel *simTimeLabel;
    void buildStatusBar();

    int simTime;
    bool startNext;
    QAction *actionStart, *actionStop;
    int selectedRobotId;
    int nRobots;
    vector< cbRobotInfo *> robotScores;
    Ui::cbSimulatorGUI *ui;

private slots:
    void setSimTime(int);
    void setRemainTime(int);

    void on_actionOpen_Lab_triggered();
    //void on_actionOpen_Grid_triggered(); // Grid is now chosen with lab
    void on_actionEdit_Configuration_triggered();
    void on_actionOpen_Configuration_triggered();
    void on_actionSave_Configuration_triggered();
    void on_actionAbout_Simulator_triggered();

    void setStartStopButton(bool running);
    void on_actionStartStop_triggered();
    void on_actionLaunch_Viewer_toggled(bool arg1);
    void processViewerError(QProcess::ProcessError);
    void handleViewerExit(int, QProcess::ExitStatus);

    void resetRobWidgets(int);
    void addRobWidget(int);
    void removeRobWidget(int);
    void refreshRobComboBox();
    void on_comboBox_Robot_currentIndexChanged(int index);

    void on_actionSimulator_Help_triggered();
    void on_actionLoad_Defaults_triggered();
};

#endif // CBSIMULATORGUI_H
