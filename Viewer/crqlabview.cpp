#include "crqlabview.h"
#include "ui_crqlabview.h"

#include <QtGui>
#include <QMessageBox>
#include <QColorDialog>

#include"Comm/crqcomm.h"
#include "logotipo.xpm"

CRQLabView::CRQLabView(CRMainParameters *p, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::CRQLabView)
{
    ui->setupUi(this);

#ifdef DEBUG
    cout << "\nMainWindow::MainWindow\n";
#endif
    param = p;
    soundStatus = 'y';
    skinFileName = "skins/default/default.skin";
//    skinFileName = "skins/skin-base/skin-base.skin";
    QPixmap icon( ( const char** ) logotipo_xpm );
    setWindowIcon(icon);
    //setBackgroundRole( QColor( 127, 127, 127 ));

    //Built one empty lab
    lab = new CRLab( );

    //Built one empty scene
    scene = new CRQScene( lab );
    scene->skin( skinFileName );
    scene->colorSmall.setNamedColor( param->lowerColor );
    scene->colorBig.setNamedColor( param->higherColor );

    // Communications object
    QString host( param->serverAddr );
    comm = new CRQComm(this, scene, lab, host, param->port,
                       param->control, param->autoConnect, param->autoStart);

    ui->graphicsView_lab->setBackgroundBrush( QColor( 128, 128, 128 ));
    ui->graphicsView_lab->setScene(scene);
    ui->graphicsView_lab->fitInView(scene->sceneRect());

    connect (comm, SIGNAL(viewerConnected(bool)), ui->actionConnect, SLOT(setChecked(bool)));

}

CRQLabView::~CRQLabView()
{
    delete ui;
}


void CRQLabView::on_actionAbout_triggered()
{
    QMessageBox *about = new QMessageBox(QMessageBox::Information,
                                         "About the Viewer! v2.0",
                                         "<h2>Viewer</h2>"
                                         "<ul>"
                                         "<li>Universidade de Aveiro"
                                         "<li>Explorer 2018 Contest"
                                         "<li>http://microrato.ua.pt"
                                         "</ul>"
                                         "<b>Copyright 2002-2018 Antnio Neves, Joo Figueiredo, Miguel Rodrigues</b>",
                                         QMessageBox::Ok, this);
    about->show();
}

void CRQLabView::on_actionReset_Viewer_triggered()
{
    if( scene->lab != NULL && scene != NULL )
    {
#ifdef DEBUG
        static int nTimesReseted = 1;
        fprintf(stderr, "\nMainWindow::reset (%2d times)\n",
                nTimesReseted++);
#endif

        comm->closeWindows(); // close dataView and control windows
        scene->clear();
        scene->skin(skinFileName);
        scene->bgInitDraw();
        ui->graphicsView_lab->fitInView(scene->sceneRect());
    }

    ui->actionConnect->setChecked(false);
}

void CRQLabView::on_actionConnect_toggled(bool connected)
{
    if (connected)
    {

#ifdef DEBUG
    static int nTimesConnected = 1;
    fprintf(stderr, "\nMainWindow::connect (%2d times)\n",
            nTimesConnected++);
#endif
        ui->actionConnect->setText("&Disconnect");
        ui->actionConnect->setToolTip("Disconnect");
        ui->actionConnect->setShortcut(Qt::CTRL+Qt::Key_D);
        comm->connect();
    }
    else
    {
        comm->resetSimulator();
        ui->actionConnect->setText("&Connect");
        ui->actionConnect->setToolTip("Connect");
        ui->actionConnect->setShortcut(Qt::CTRL+Qt::Key_C);
        ui->actionReset_Viewer->trigger();
    }

}


void CRQLabView::on_actionLower_walls_color_triggered()
{
    QColor c = QColorDialog::getColor( QColor( 127, 127, 127 ), this,
                                       "SmallColors");
    if(c.isValid())
    {
        scene->colorSmall = c;
        if( scene->lab != NULL )
        {
            scene->clear();
            scene->skin(skinFileName);
            scene->drawLab( scene->lab );
            scene->drawGrid( scene->lab );
            scene->drawRobot( scene->lab );
        }
    }
}

void CRQLabView::on_actionHigher_walls_collor_triggered()
{
    QColor c = QColorDialog::getColor( QColor( 127, 127, 127 ), this,
                                       "BigColors");
    if(c.isValid())
    {
        scene->colorBig = c;
        if( scene->lab != NULL )
        {
            scene->clear();
            scene->skin(skinFileName);
            scene->drawLab( scene->lab );
            scene->drawGrid( scene->lab );
            scene->drawRobot( scene->lab );
        }
    }
}

void CRQLabView::on_actionSound_On_Off_toggled(bool soundOFF)
{
    if (soundOFF)
    {
        ui->actionSound_On_Off->setText("&Sound is muted");
        ui->actionSound_On_Off->setToolTip("Unmute");
        soundStatus = 'n';
    }
    else
    {
        ui->actionSound_On_Off->setText("&Sound is on");
        ui->actionSound_On_Off->setToolTip("Mute");
        soundStatus = 'y';
    }

    scene->sound = soundStatus;
}

void CRQLabView::on_actionChange_skin_triggered()
{
//	cout << "\nMainWindow::skin\n";
//	skinFileName = QFileDialog::getOpenFileName( "skins/", "Skins (*.skin)",
//                    this, "open file dialog", "Choose a file" );
//
//	if(!skinFileName.isEmpty())
//		{
//		scene->clear();
//		scene->skin(skinFileName);
//		comm->skin(skinFileName);
//		this->reset();
//		}
//
//	else
//		skinFileName = "skins/default/default.skin";
}

void CRQLabView::paintEvent(QPaintEvent *)
{
    ui->graphicsView_lab->fitInView(scene->sceneRect());
}

void CRQLabView::on_actionStart_triggered()
{
    comm->sendMessage("<Start/>");

}
