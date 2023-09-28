/*
    This file is part of ciberRatoToolsSrc.

    Copyright (C) 2001-2021 Universidade de Aveiro

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

#include "cbsimulatorGUI.h"
#include "ui_cbsimulatorGUI.h"

#include <QtGui>
#include <QProcess>
#include <QFileDialog>
#include <QMessageBox>

#include "cbsimulator.h"
#include "cbrobot.h"
#include "cbparamdialog.h"
#include "cbcontrolpanel.h"
#include "cbmanagerobots.h"
#include "cblabdialog.h"
#include "cblab.h"
#include "cbrobotinfo.h"

/**
 * cbSimulatorGUI contructor.
 */
cbSimulatorGUI::cbSimulatorGUI( cbSimulator *sim, QWidget* parent, Qt::WindowFlags fl ) :
    QMainWindow(parent, fl), simulator (sim),
    ui(new Ui::cbSimulatorGUI)
{
    ui->setupUi(this);

    QIcon ic(QCoreApplication::applicationDirPath() + "/simlogo.png");
    setWindowIcon(ic);

    this->tabifyDockWidget(ui->dockWidget_Score, ui->dockWidget_Mailbox);
    ui->dockWidget_Score->raise();

    buildStatusBar();

    simTime = simulator->simTime();
    ui->lcdNumber_TimeRemain->display(simTime);
    connect(simulator, SIGNAL(simTimeChanged(int)), SLOT(setSimTime(int)));
    connect(simulator, SIGNAL(curTimeChanged(int)), SLOT(setRemainTime(int)));

    ui->actionGPS_Enabled->setChecked(cbRobot::GPSOn);
    ui->actionScore_Sensor_Enabled->setChecked(simulator->getScoreSensor());
    ui->actionTime_Enabled->setChecked(simulator->isTimed());
    ui->actionRegistrations_Open->setChecked(simulator->isRegistrationAllowed());
    ui->actionCollisions_Enabled->setChecked(!simulator->collisionsIgnored());

    connect(simulator, SIGNAL(toggleGPS(bool)), ui->actionGPS_Enabled, SLOT(setChecked(bool)));
    connect(simulator, SIGNAL(toggleScoreSensor(bool)), ui->actionScore_Sensor_Enabled, SLOT(setChecked(bool)));
    connect(simulator, SIGNAL(toggleTime(bool)), ui->actionTime_Enabled, SLOT(setChecked(bool)));
    connect(simulator, SIGNAL(toggleRegistrations(bool)), ui->actionRegistrations_Open, SLOT(setChecked(bool)));
    connect(simulator, SIGNAL(toggleCollisions(bool)), ui->actionCollisions_Enabled, SLOT(setChecked(bool)));

    this->startNext = true;
    connect(simulator, SIGNAL(simReady(bool)), ui->actionStartStop, SLOT(setEnabled(bool)));
    connect(simulator, SIGNAL(simRunning(bool)), SLOT(setStartStopButton(bool)));

    connect(simulator, SIGNAL(simReset(bool)), ui->actionRegistrations_Open, SLOT(setDisabled(bool)));
    connect(simulator, SIGNAL(simReset(bool)), ui->actionRegistrations_Open, SLOT(setChecked(bool)));

    actionStart = new QAction(this);
    actionStop = new QAction(this);
    connect(actionStart, SIGNAL(triggered()), simulator, SLOT(start()));
    connect(actionStop, SIGNAL(triggered()), simulator, SLOT(stop()));
    connect(ui->actionReset, SIGNAL(triggered()), simulator, SLOT(reset()));

    connect(ui->actionGPS_Enabled, SIGNAL(toggled(bool)), simulator, SLOT(setGPS(bool)));
    connect(ui->actionScore_Sensor_Enabled, SIGNAL(toggled(bool)), simulator, SLOT(setScoreSensor(bool)));
    connect(ui->actionRegistrations_Open, SIGNAL(toggled(bool)), simulator, SLOT(setRegistrations(bool)));
    connect(ui->actionTime_Enabled, SIGNAL(toggled(bool)), simulator, SLOT(setTime(bool)));
    connect(ui->actionCollisions_Enabled, SIGNAL(toggled(bool)), simulator, SLOT(setCollisions(bool)));

    connect( ui->actionExit, SIGNAL( triggered() ), qApp, SLOT( closeAllWindows() ) );

    connect(&viewerProc, SIGNAL(error(QProcess::ProcessError)), this, SLOT(processViewerError(QProcess::ProcessError)));
    connect(&viewerProc, SIGNAL(finished(int,QProcess::ExitStatus)), this, SLOT(handleViewerExit(int,QProcess::ExitStatus)));

    agentModel = new QStringListModel(this);
    manRobDialog = new cbManageRobots (simulator, agentModel, this, Qt::Dialog);
    connect(ui->actionManageRobots, SIGNAL(toggled(bool)), manRobDialog, SLOT(setVisible(bool)));
    connect(manRobDialog, SIGNAL(closed()), ui->actionManageRobots, SLOT(toggle()));
    controlPanel = new cbControlPanel(simulator, agentModel, 0, Qt::Tool | Qt::WindowStaysOnTopHint | Qt::WindowCloseButtonHint | Qt::WindowMinimizeButtonHint);
    connect(ui->actionControl_Panel, SIGNAL(toggled(bool)), controlPanel, SLOT(setVisible(bool)));
    connect(controlPanel, SIGNAL(closed()), ui->actionControl_Panel, SLOT(toggle()));

    nRobots = simulator->Robots().size();
    robotScores.resize(nRobots);
    resetRobWidgets(nRobots);
    connect(simulator, SIGNAL(gridChanged(int)), SLOT(resetRobWidgets(int)));
    connect(simulator, SIGNAL(robotRegistered(int)), SLOT(addRobWidget(int)));
    connect(simulator, SIGNAL(robotDeleted(int)), SLOT(removeRobWidget(int)));

    connect(ui->checkBox_Actions, SIGNAL(toggled(bool)), simulator, SLOT(setShowActions(bool)));
    connect(ui->checkBox_Measures, SIGNAL(toggled(bool)), simulator, SLOT(setShowMeasures(bool)));
    connect(ui->checkBox_Positions, SIGNAL(toggled(bool)), simulator, SLOT(setShowPositions(bool)));


    connect(ui->pushButton_ClearMailBox, SIGNAL(clicked()), ui->textEdit_Mailbox, SLOT(clear()));

    appendMessage("Welcome, " + QDir::home().dirName());

    labDialog = new cbLabDialog(simulator, this);

    this->adjustSize();

    mailBoxSize = 0;

}

cbSimulatorGUI::~cbSimulatorGUI()
{
    delete ui;
}

void cbSimulatorGUI::appendMessage(const QString msg, const bool isErr)
{
    if (isErr)
        ui->messages->setTextColor(Qt::red);
    else
        ui->messages->setTextColor(Qt::black);

    ui->messages->append(msg);
}

void cbSimulatorGUI::writeOnBoard(const QString msg, int id, int type)
{
    QColor c;
    if (selectedRobotId == 0 || selectedRobotId == id)
    {
        switch (type)
        {
        case 0:
            if (!ui->checkBox_Measures->isChecked())
                return;
            c = Qt::blue;
            break;
        case 1:
            if (!ui->checkBox_Actions->isChecked())
                return;
            c = Qt::magenta;
            break;
        case 2:
            if (!ui->checkBox_Positions->isChecked())
                return;
            c = Qt::darkGreen;
            break;
        default:
            return;
        }


        ui->textEdit_Mailbox->setTextColor(c);
        ui->textEdit_Mailbox->append(msg);
        mailBoxSize += msg.size();

        if(mailBoxSize > MAILBOXLIMIT) {
            mailBoxSize = 0;
            ui->textEdit_Mailbox->clear();
            // QString newStr = ui->textEdit_Mailbox->toPlainText();
            // newStr = newStr.right(MAILBOXLIMIT/2);
            // ui->textEdit_Mailbox->setTextColor(Qt::black);
            // ui->textEdit_Mailbox->setText(newStr);
            // mailBoxSize = MAILBOXLIMIT/2;
            // QTextCursor cursor(ui->textEdit_Mailbox->textCursor());
            // cursor.movePosition(QTextCursor::End, QTextCursor::MoveAnchor);
            // ui->textEdit_Mailbox->setTextCursor(cursor);
        }       
    }
}

void cbSimulatorGUI::buildStatusBar()
{
    stateLabel = new QLabel(simulator->curStateAsString());
    stateLabel->setToolTip("Simulation State");
    connect (simulator, SIGNAL(stateChanged(QString)), stateLabel, SLOT (setText(QString)));

    labLabel = new QLabel(simulator->Lab()->Name());
    labLabel->setToolTip("Current Lab");
    connect (simulator, SIGNAL(labChanged(QString)), labLabel, SLOT(setText(QString)));
    //connect (simulator, SIGNAL(labChanged(QString)), simulator, SLOT(printTargetRelativePositions()));
    //connect(simulator, SIGNAL(gridChanged(int)), simulator, SLOT(printTargetRelativePositions()));

    curTimeLabel = new QLabel("0");
    curTimeLabel->setToolTip("Current simulation time");
    connect (simulator, SIGNAL(curTimeChanged(int)), curTimeLabel, SLOT(setNum(int)));

    simTimeLabel = new QLabel(QString::number(simulator->simTime()));
    simTimeLabel->setToolTip("Time limit");

    ui->statusbar->addPermanentWidget(stateLabel,5);
    ui->statusbar->addPermanentWidget(labLabel,15);
    ui->statusbar->addPermanentWidget(curTimeLabel);
    ui->statusbar->addPermanentWidget(simTimeLabel);
}

void cbSimulatorGUI::setSimTime(int time)
{
    simTime = time;
    if (!time)
    {
        //ui->actionTime_Enabled->setChecked(false);
        simTimeLabel->setText(trUtf8("\u221e"));    // c++ infinite char code
        simTimeLabel->setToolTip("No time limit");
        disconnect (simulator, SIGNAL(curTimeChanged(int)), this, SLOT(setRemainTime(int)));
        ui->lcdNumber_TimeRemain->display("----");
    }
    else
    {
        simTimeLabel->setNum(simTime);
        simTimeLabel->setToolTip("Time limit");
        ui->lcdNumber_TimeRemain->display(simTime - (int) simulator->curTime());
        connect (simulator, SIGNAL(curTimeChanged(int)), SLOT(setRemainTime(int)));
    }
}

void cbSimulatorGUI::setRemainTime(int time)
{
    ui->lcdNumber_TimeRemain->display(simTime - time);
}

void cbSimulatorGUI::on_actionOpen_Lab_triggered()
{
    if(simulator->state()==cbSimulator::INIT)
        labDialog->exec();
    else
        appendMessage(QString("Cannot Change Lab After Start - Use Reset"), true);
}

void cbSimulatorGUI::on_actionEdit_Configuration_triggered()
{
    if(simulator->state()==cbSimulator::INIT) {
        cbParamDialog paramDlg(simulator);

        paramDlg.exec();

        simulator->processEditParameters();
    }
    else {
        appendMessage(QString("Cannot Change Configuration After Start - Use Reset"), true);
    }
}

void cbSimulatorGUI::on_actionOpen_Configuration_triggered()
{
    if(simulator->state()==cbSimulator::INIT) {
        // start at the current working directory and with *.cpp as filter
        //QString f = QFileDialog::getOpenFileName( QString::null, "*.xml", this );
        QString f = QFileDialog::getOpenFileName(this, "Open Configuration", QString::null, "*.xml");
        if ( !f.isEmpty() ) {
            if(simulator->changeParameters(f))
                appendMessage( QString("Parameters Changed to ")+f );
            else
                appendMessage( QString("Error loading ")+f, true );
        }
    }
    else {
        appendMessage(QString("Cannot Change Parameters After Start - Use Reset"), true);
    }
}

void cbSimulatorGUI::on_actionSave_Configuration_triggered()
{
    if(simulator->state()==cbSimulator::INIT) {

        QString f = QFileDialog::getSaveFileName(this, "Save Configuration", QString::null, "*.xml" );

        if (!f.isNull())
            simulator->saveConfiguration(f);
    }
    else {
        appendMessage(QString("Cannot Save Configuration After Start - Use Reset"), true);
    }
}

void cbSimulatorGUI::on_actionAbout_Simulator_triggered()
{
    QMessageBox::about(this,"About Explorer 2023 simulator",
         "<b>Explorer 2023 simulator</b><br>"
         "Copyright 2001-2023 Universidade de Aveiro<br>"
         "http://microrato.ua.pt"
         );
}

void cbSimulatorGUI::setStartStopButton(bool running)
{
    if (running)
    {
        ui->actionStartStop->setIcon(QIcon(":/images/control/stop"));
        ui->actionStartStop->setText("S&top");
        ui->actionStartStop->setToolTip("Stop simulation");
    }
    else
    {
        ui->actionStartStop->setIcon(QIcon(":/images/control/start"));
        ui->actionStartStop->setText("&Start");
        ui->actionStartStop->setToolTip("Start simulation");
    }

    startNext = !running;

}

void cbSimulatorGUI::on_actionStartStop_triggered()
{
    if (startNext)
        actionStart->trigger();
    else
        actionStop->trigger();
}

void cbSimulatorGUI::on_actionLaunch_Viewer_toggled(bool arg1)
{
    if (arg1)
    {
        ui->actionLaunch_Viewer->setText("Close &Viewer");
        ui->actionLaunch_Viewer->setToolTip("Close Viewer");
        ui->actionControl_Panel->setChecked(true);
        QStringList viewerArgs;
        viewerArgs << "-autoconnect";
#ifdef MicWindows
        viewerProc.start("../Viewer/Viewer.exe", viewerArgs);
#else
        viewerProc.start("../Viewer/Viewer", viewerArgs);
#endif
    }
    else
    {
        ui->actionLaunch_Viewer->setText("Launch &Viewer");
        ui->actionLaunch_Viewer->setToolTip("Launch Viewer");
        ui->actionControl_Panel->setChecked(false);
        viewerProc.terminate();
    }
}

void cbSimulatorGUI::processViewerError(QProcess::ProcessError err)
{
    switch (err)
    {
    case QProcess::FailedToStart:
        appendMessage("Unable to show Viewer", true);
        break;
    case QProcess::Crashed:
        appendMessage("Viewer has crashed", true);
        break;
    default:
        appendMessage("An error occured with Viewer", true);
    }

    ui->actionLaunch_Viewer->setChecked(false);
}

void cbSimulatorGUI::handleViewerExit(int, QProcess::ExitStatus stat)
{
    switch (stat)
    {
    case QProcess::NormalExit:
        appendMessage("Viewer has been closed");
        break;
    case QProcess::CrashExit:
        appendMessage("Viewer has crashed", true);
    }
    ui->actionLaunch_Viewer->setChecked(false);
}

void cbSimulatorGUI::resetRobWidgets(int size)
{
    for (int i = 0; i < nRobots; i++)
        if (robotScores[i] != 0) delete robotScores[i];

    nRobots = size;
    robotScores.resize(nRobots);
    for (int i = 0; i < nRobots; i++)
        robotScores[i] = 0;

    refreshRobComboBox();
}

void cbSimulatorGUI::addRobWidget(int id)
{
    if (id >= 1 && id <= nRobots && robotScores[id-1] == 0)
    {
        robotScores[id-1] = new cbRobotInfo(simulator->Robots()[id-1]);
        ui->gridLayout_Scores->addWidget(robotScores[id-1],id-1,0, Qt::AlignTop);

        refreshRobComboBox();

        //Strange behavior when adjusting size
        //QApplication::processEvents();
        //ui->dockWidget_Score->adjustSize();
    }
}

void cbSimulatorGUI::removeRobWidget(int id)
{
    if (id >= 1 && id <= nRobots && robotScores[id-1] != 0)
    {
        ui->gridLayout_Scores->removeWidget(robotScores[id-1]);
        delete robotScores[id-1];
        robotScores[id-1] = 0;

        ui->comboBox_Robot->removeItem(ui->comboBox_Robot->findData(QVariant(id)));

        //Strange behavior when adjusting size
        //QApplication::processEvents();
        //ui->dockWidget_Score->adjustSize();
    }
}

void cbSimulatorGUI::refreshRobComboBox()
{
    ui->comboBox_Robot->clear();
    ui->comboBox_Robot->addItem("everyone", QVariant(0));
    for (int i = 0; i < nRobots; i++)
    {
        cbRobot *robot = simulator->Robots()[i];
        if (robot != 0)
            ui->comboBox_Robot->addItem(robot->Name(), QVariant(robot->Id()));
    }
}

void cbSimulatorGUI::on_comboBox_Robot_currentIndexChanged(int index)
{
    if (index >= 0)
        selectedRobotId = ui->comboBox_Robot->itemData(index).toInt();
    else
        selectedRobotId = -1;
}

void cbSimulatorGUI::on_actionSimulator_Help_triggered()
{
    if (!QDesktopServices::openUrl(QUrl("http://microrato.ua.pt/ciberrato#Documentos")))
        appendMessage("Unable to open Micro-Rato website", true);
}

void cbSimulatorGUI::on_actionLoad_Defaults_triggered()
{
    if(simulator->state()==cbSimulator::INIT)
    {
        simulator->setDefaultParameters();
        simulator->setDefaultGrid();
        simulator->setDefaultLab();
        appendMessage("Defaults Set");
    }
    else
        appendMessage("Cannot Set Defaults After Start - Use Reset",true);

}
