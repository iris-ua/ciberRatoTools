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

#ifndef CBLABDIALOG_H
#define CBLABDIALOG_H

#include <QtCore>
#include <QtGui>
#include <QFileSystemModel>
#include <QDialog>
#include <QGraphicsScene>
#include <QGraphicsEllipseItem>

using namespace std;

class cbSimulator;
class cbLab;
class cbGrid;

namespace Ui {
class cbLabDialog;
}

/*!
    \class cbLabDialog
    \brief Matching class for choosing Lag and Grid Form
*/
class cbLabDialog : public QDialog
{
    Q_OBJECT

public:
    explicit cbLabDialog(cbSimulator *sim, QWidget *parent = 0);
    ~cbLabDialog();

protected slots:
    virtual void accept(void);

protected:
    virtual void showEvent(QShowEvent *);
    virtual void paintEvent(QPaintEvent *);

    void drawLab (cbLab *lab, QGraphicsScene *scene);
    void drawGrid (cbLab *lab, cbGrid *grid, QGraphicsScene *scene);

    QPixmap beaconPix;

    cbSimulator *simulator;
    cbLab *currentLab, *defaultLab;
    cbGrid *currentGrid, *defaultGrid;

    cbLab *previewLab;
    cbGrid *previewGrid;

    QString labFileName, gridFileName;
    QString previewLabFileName, previewGridFileName;

    QFileSystemModel *labModel;

    QGraphicsScene *scene_Preview;
    QGraphicsScene *scene_Lab;

    vector<QGraphicsEllipseItem *> startP;

private slots:
    void on_treeView_clicked(const QModelIndex &index);
    void on_pushButton_Apply_clicked();
    void restoreDefaults();

private:
    Ui::cbLabDialog *ui;
};

#endif // CBLABDIALOG_H
