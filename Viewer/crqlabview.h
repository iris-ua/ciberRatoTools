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

#ifndef _CIBER_RATO_CRQLABVIEW_
#define _CIBER_RATO_CRQLABVIEW_

/*! \class CRQLabView
 *  \brief Class that implements one main window where the application will run.
 */

#include <QMainWindow>

#include"crqscene.h"
#include"crmainparameters.h"
#include"Comm/crqcomm.h"
#include"Lab/crlab.h"

class CRQComm;

namespace Ui {
class CRQLabView;
}

class CRQLabView : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit CRQLabView(CRMainParameters*, QWidget *parent = 0);
    ~CRQLabView();

    QString getSkin() { return skinFileName; };
    
    char soundStatus;

protected:
    virtual void paintEvent(QPaintEvent *);

private slots:
    void on_actionReset_Viewer_triggered();

    void on_actionConnect_toggled(bool connect);

    void on_actionAbout_triggered();

    void on_actionLower_walls_color_triggered();

    void on_actionHigher_walls_collor_triggered();

    void on_actionSound_On_Off_toggled(bool soundON);

    void on_actionChange_skin_triggered();

    void on_actionStart_triggered();

private:
    /*! \var scene
     * Pointer to the scene that will contain the lab.
     */
    CRQScene *scene;
    /*! \var comm Pointer to the communication object.
     */
    CRQComm *comm;

    QString skinFileName;

    CRMainParameters *param;

    CRLab *lab;

    Ui::CRQLabView *ui;
};

#endif // _CIBER_RATO_CRQLABVIEW_
