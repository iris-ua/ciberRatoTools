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

#include "cblabdialog.h"
#include "ui_cblabdialog.h"

#include "cbsimulator.h"
#include "cblab.h"
#include "cblabhandler.h"
#include "cbgridhandler.h"
#include "cbparameters.h"
#include "vector"
#include "assert.h"

cbLabDialog::cbLabDialog(cbSimulator *sim, QWidget *parent) :
    QDialog(parent),
    simulator(sim),
    ui(new Ui::cbLabDialog)
{
    ui->setupUi(this);

    previewLab = 0;
    previewGrid = 0;
    currentLab = simulator->Lab();
    currentGrid = simulator->Grid();
    defaultLab = simulator->getDefaultLab();
    defaultGrid = simulator->getDefaultGrid();

    beaconPix = QPixmap(":/images/icons/beacon");

    QString labPath = "../Labs/";
    labModel = new QFileSystemModel(this);
    QModelIndex idx = labModel->setRootPath(labPath);
    ui->treeView->setModel(labModel);
    ui->treeView->setRootIndex(idx);
    ui->treeView->setColumnHidden(1,true);
    ui->treeView->setColumnHidden(2,true);
    ui->treeView->setColumnHidden(3,true);
    ui->treeView->header()->hide();

    scene_Preview = new QGraphicsScene(this);
    scene_Lab = new QGraphicsScene(this);
    scene_Preview->setBackgroundBrush(QColor(0, 0, 0));
    scene_Lab->setBackgroundBrush(QColor(0, 0, 0));

    ui->graphicsView_Preview->setScene(scene_Preview);
    ui->graphicsView_Lab->setScene(scene_Lab);

    connect (ui->buttonBox->button(QDialogButtonBox::RestoreDefaults),
             SIGNAL(clicked()), SLOT(restoreDefaults()));
}

cbLabDialog::~cbLabDialog()
{
    delete ui;
}

void cbLabDialog::drawLab (cbLab *lab, QGraphicsScene *scene)
{
    assert (lab != 0 && scene != 0);

    scene->clear();

    int zoom = 34;
    int sizeInPixels = (int) (lab->Height() * zoom);

    scene->setSceneRect(0, 0, lab->Width() * zoom , lab->Height() * zoom);
    scene->setBackgroundBrush(QColor(0,0,0));

    QColor color;	// Color for the walls
    // Walls
    uint i, pos;
    for (i = 1; i < lab->nWalls(); i++)
    {
        cbWall *wall = lab->Wall(i);
        if (wall->Height() > lab->Beacon(0)->Height())
           color = Qt::green;
        else
           color = Qt::blue;

        vector< cbPoint > *points = &(wall->Corners());
        QPolygon *pointArray = new QPolygon ( points->size() );

        pos = 0;
        for (vector< cbPoint >::iterator ponto = points->begin();
            ponto != points->end(); ++ponto)
        {
            pointArray->setPoint(pos,
                                 (int) (ponto->X() * zoom),
                                 (int) (sizeInPixels - ponto->Y() * zoom));
            pos++;
        }

        QGraphicsPolygonItem *p = new QGraphicsPolygonItem();
        p->setPolygon(*pointArray );
        p->setVisible(true );
        p->setBrush(QBrush( color ));
        p->setZValue(6);
        scene->addItem(p);
    }

    for (i = 0; i < lab->nTargets(); i++)
    {
        cbTarget *target = lab->Target(i);
        double radius = target->Radius() * zoom;
        double diam = radius * 2;
        double target_x = target->Center().X() * zoom - radius;
        double target_y = sizeInPixels - target->Center().Y() * zoom - radius;
        QGraphicsEllipseItem *labArrival = new QGraphicsEllipseItem(target_x, target_y,
                                                                    diam, diam);
        labArrival->setBrush(Qt::cyan);
        labArrival->setZValue(1);
        labArrival->setVisible(true);
        scene->addItem(labArrival);

        QGraphicsEllipseItem *labArrival2 = new QGraphicsEllipseItem(target_x + 0.1 * zoom, target_y + 0.1 * zoom,
                                                                     diam - 0.2 * zoom, diam - 0.2 * zoom);
        labArrival2->setBrush(Qt::black);
        labArrival2->setZValue(2);
        labArrival2->setVisible( true );
        scene->addItem(labArrival2);
    }

    QList<QGraphicsPixmapItem *> *labLight = new QList<QGraphicsPixmapItem *>();
    for (i = 0; i < lab->nBeacons(); i++)
    {
        cbBeacon *beacon = lab->Beacon(i);

        QGraphicsPixmapItem *light = new QGraphicsPixmapItem (beaconPix);
        light->setX(beacon->Center().X() * zoom - zoom);
        light->setY(sizeInPixels - beacon->Center().Y() * zoom - zoom);
        light->setZValue( 4 );
        light->setVisible( true );
        scene->addItem(light);
        labLight->append(light);
    }

    scene->update();
}

void cbLabDialog::drawGrid (cbLab *lab, cbGrid *grid, QGraphicsScene *scene)
{
    assert (lab != 0 && grid != 0 && scene != 0);

    for (uint i = 0; i < startP.size(); i++)
        if (scene->items().contains(startP[i]))
            scene->removeItem(startP[i]);

    int zoom = 34;
    int sizeInPixels = (int) (lab->Height() * zoom);

    int nGrids = grid->size();
    startP.resize(nGrids);
    for (int i = 0; i < nGrids; i++)
    {
        double grid_x = ((cbPosition) grid->at(i)).X() * zoom - zoom /2;
        double grid_y = sizeInPixels - ((cbPosition) grid->at(i)).Y() * zoom - zoom /2;
        startP[i] = new QGraphicsEllipseItem(grid_x, grid_y,
                                             1 * zoom, 1 * zoom);
        startP[i]->setVisible( true );
        startP[i]->setBrush( Qt::cyan );
        startP[i]->setZValue( 3 );
        scene->addItem(startP[i]);
    }

    scene->update();
}

void cbLabDialog::on_treeView_clicked(const QModelIndex &index)
{
    QString fileName = labModel->fileInfo(index).absoluteFilePath();
    QXmlInputSource *source;
    QFile srcFile(fileName);
    if(!srcFile.exists() || (source = new QXmlInputSource(&srcFile)) == 0)
        return;

    QXmlSimpleReader labXmlParser;
    cbLabHandler *labHandler = new cbLabHandler;
    labXmlParser.setContentHandler(labHandler);

    QFile srcFileGrid(fileName);
    QXmlInputSource *sourceGrid = new QXmlInputSource(&srcFileGrid);
    QXmlSimpleReader gridXmlParser;
    cbGridHandler *gridHandler = new cbGridHandler;
    gridXmlParser.setContentHandler(gridHandler);

    if(labXmlParser.parse(*source))
    {
        if (previewLab != 0) delete previewLab;
        previewLab = labHandler->parsedLab();
        previewLabFileName = fileName;
        ui->label_Preview->setText(previewLab->Name());
    }
    else if (gridXmlParser.parse(*sourceGrid))
    {
        if (previewGrid != 0) delete previewGrid;
        previewGrid = gridHandler->parsedGrid();
        previewGridFileName = fileName;
    }

    if (previewLab != 0)
    {
        drawLab(previewLab, scene_Preview);
        if (previewGrid != 0)
        {
            drawGrid(previewLab, previewGrid, scene_Preview);
            ui->pushButton_Apply->setDisabled(previewLabFileName.isEmpty() || previewGridFileName.isEmpty());
        }
    }

    delete labHandler;
    delete gridHandler;
    delete source;
    delete sourceGrid;
}

void cbLabDialog::on_pushButton_Apply_clicked()
{
    if (previewLabFileName.isEmpty() || previewGridFileName.isEmpty())
        return;

    labFileName = previewLabFileName;
    gridFileName = previewGridFileName;
    drawLab(previewLab, scene_Lab);
    drawGrid(previewLab, previewGrid, scene_Lab);
    ui->label_Lab->setText(previewLab->Name());
}

void cbLabDialog::restoreDefaults(void)
{
    if (labFileName.isEmpty() && gridFileName.isEmpty())
        return;

    drawLab(defaultLab, scene_Lab);
    drawGrid(defaultLab, defaultGrid, scene_Lab);
    labFileName = "";
    gridFileName = "";
    ui->label_Lab->setText(defaultLab->Name());
}

void cbLabDialog::accept()
{
    if (labFileName.isEmpty())
    {
        simulator->setDefaultLab();
        simulator->GUI()->appendMessage(QString("Lab Changed to DEFAULT"));
    }
    else
    {
        if(simulator->changeLab(labFileName))
            simulator->GUI()->appendMessage(QString("Lab Changed to ") + labFileName );
        else
            simulator->GUI()->appendMessage(QString("Error loading ") + labFileName, true );
    }

    if (gridFileName.isEmpty())
    {
        simulator->setDefaultGrid();
        simulator->GUI()->appendMessage(QString("Grid Changed to DEFAULT"));
    }
    else
    {
        if(simulator->changeGrid(gridFileName))
            simulator->GUI()->appendMessage(QString("Grid Changed to ") + gridFileName );
        else
            simulator->GUI()->appendMessage(QString("Error loading ") + gridFileName, true );
    }
    close();
}

void cbLabDialog::showEvent(QShowEvent *)
{
    show();
    labFileName = simulator->getParameters()->labFilename;
    gridFileName = simulator->getParameters()->gridFilename;
    ui->label_Lab->setText(simulator->Lab()->Name());
    drawLab(simulator->Lab(), scene_Lab);
    drawGrid(simulator->Lab(), simulator->Grid(), scene_Lab);
}

void cbLabDialog::paintEvent(QPaintEvent *)
{
    ui->graphicsView_Preview->fitInView(scene_Preview->sceneRect());
    ui->graphicsView_Lab->fitInView(scene_Lab->sceneRect());
}
