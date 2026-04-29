#include "crqrobotinfo.h"
#include "ui_crqrobotinfo.h"

#include <QtCore>
#include <QtGui>

using namespace std;

CRQRobotInfo::CRQRobotInfo(CRRobot *rob, QString skinName, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CRQRobotInfo)
{
    ui->setupUi(this);

    skin(skinName); /* Change skin */
    robot = rob;

    ui->lcdNumber_ID->display(rob->id());

    QPixmap *pixx = new QPixmap(robFiles[rob->id()]);
    QMatrix mat;
    QPixmap tmp = pixx->transformed(mat.rotate(-90));
    ui->label_Icon->setPixmap(tmp.scaled(ui->label_Icon->size()));

    // NOME
    ui->label_Name->setText( robot->name() );

    // RETURN TIME
    connect(this, SIGNAL(returnTime(int)), ui->lcdNumber_Time, SLOT(display(int)));

    // SCORE
    connect(this, SIGNAL(score(int)), ui->lcdNumber_Score, SLOT(display(int)));

    // COLISOES
    connect(this, SIGNAL(collisions(int)), ui->lcdNumber_Collisions, SLOT(display(int)));

    // ESTADO
    previousState = CRRobot::STOPPED;
    ui->label_State->setPixmap( stopPix );
    connect( this, SIGNAL( state(QPixmap) ), ui->label_State,
             SLOT(setPixmap(QPixmap)));

    ui->label_State->setToolTip(rob->stateAsString());

}

CRQRobotInfo::~CRQRobotInfo()
{
    delete ui;
}

void CRQRobotInfo::skin(QString skinName)
{
    char rob1File[4096];
    char rob2File[4096];
    char rob3File[4096];
    char rob4File[4096];
    char rob5File[4096];
    char backFile[4096];
    char runFile[4096];
    char waitFile[4096];
    char stopFile[4096];
    char returnFile[4096];
    char removeFile[4096];
    char finishFile[4096];


    strcpy(backFile, "skins/");
    strcat(backFile, skinName.toLatin1());
    strcat(backFile, "/robbg.png");

    strcpy(rob1File, "skins/");
    strcat(rob1File, skinName.toLatin1());
    strcat(rob1File, "/rob1/rob.png");

    strcpy(rob2File, "skins/");
    strcat(rob2File, skinName.toLatin1());
    strcat(rob2File, "/rob2/rob.png");

    strcpy(rob3File, "skins/");
    strcat(rob3File, skinName.toLatin1());
    strcat(rob3File, "/rob3/rob.png");

    strcpy(rob4File, "skins/");
    strcat(rob4File, skinName.toLatin1());
    strcat(rob4File, "/rob4/rob.png");

    strcpy(rob5File, "skins/");
    strcat(rob5File, skinName.toLatin1());
    strcat(rob5File, "/rob5/rob.png");

    robFiles << backFile << rob1File << rob2File << rob3File << rob4File << rob5File;

    strcpy(runFile, "skins/");
    strcat(runFile, skinName.toLatin1());
    strcat(runFile, "/states/running.png");

    strcpy(waitFile, "skins/");
    strcat(waitFile, skinName.toLatin1());
    strcat(waitFile, "/states/waiting.png");

    strcpy(stopFile, "skins/");
    strcat(stopFile, skinName.toLatin1());
    strcat(stopFile, "/states/stopped.png");

    strcpy(returnFile, "skins/");
    strcat(returnFile, skinName.toLatin1());
    strcat(returnFile, "/states/returning.png");

    strcpy(removeFile, "skins/");
    strcat(removeFile, skinName.toLatin1());
    strcat(removeFile, "/states/removed.png");

    strcpy(finishFile, "skins/");
    strcat(finishFile, skinName.toLatin1());
    strcat(finishFile, "/states/finished.png");

    QSize scale = ui->label_State->size();
    stopPix = QPixmap(stopFile).scaled(scale);
    runPix = QPixmap(runFile).scaled(scale);
    waitPix = QPixmap(waitFile).scaled(scale);
    returnPix = QPixmap(returnFile).scaled(scale);
    removePix = QPixmap(removeFile).scaled(scale);
    finishPix = QPixmap(finishFile).scaled(scale);

}

void CRQRobotInfo::update( CRRobot * rob)
{
    robot = rob;
    emit score( (int) robot->score() );
    emit collisions( (int) robot->collisions() );
    emit returnTime( (int) robot->returnTime() );
//2005	if(robot->state() == CRRobot::RUNNING)
//2005		emit time( (int) robot->currentTime() );
    emit time( (int) robot->arrivalTime() );

    if( previousState != robot->state() )
    {
        previousState = robot->state();
        switch ( robot->state() )
        {
        case CRRobot::STOPPED:
            emit state(stopPix);
            break;
        case CRRobot::RUNNING:
            emit state(runPix);
            break;
        case CRRobot::WAITINGOTHERS:
            emit state(waitPix);
            break;
        case CRRobot::REMOVED:
            emit state(removePix);
            break;
        case CRRobot::FINISHED:
            emit state(finishPix);
            break;
        case CRRobot::RETURNING:
            emit state(returnPix);
            break;
        }
        ui->label_State->setToolTip(robot->stateAsString());
    }
}
