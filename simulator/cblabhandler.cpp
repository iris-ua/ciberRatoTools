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
/*
 * class cbLabHandler
 */

#include "cblabhandler.h"
#include "cbpoint.h"
#include "cbwall.h"
#include "cbbeacon.h"
#include "cbtarget.h"
#include "cblab.h"
#include "cbsimulator.h"
#include "cbpoint.h"

#include <math.h>
#include <qstring.h>

#define PATHCUBESIZE  (2.0)
#define PATHWALLWIDTH (0.2)
#define PATHWALLGAP   (0.0)

bool cbLabHandler::startDocument()
{
	point = cbPoint();
	wall = 0;
	beacon = 0;
	target = 0;
	lab = 0;
    return TRUE;
}

bool cbLabHandler::endDocument()
{
	return TRUE;
}

bool cbLabHandler::startElement( const QString&, const QString&, const QString& qName,
                                    const QXmlAttributes& attr)
{
	/* process begin tag */
	const QString &tag = qName;

	if (lab==0 && tag!="Lab") return false;

	if (tag == "Lab" && lab==0)
	{
		lab = new cbLab;
		/* process attributes */
		const QString &name = attr.value(QString("Name"));
		if (!name.isNull())
        {
            lab->setName(name.toLatin1().constData());
		}
		const QString &width = attr.value(QString("Width"));
		if (!width.isNull())
		{
			lab->setWidth(width.toDouble());
		}
		const QString &height = attr.value(QString("Height"));
		if (!height.isNull())
		{
			lab->setHeight(height.toDouble());
		}
	}
	else if (tag == "Wall" && lab!=0)
	{
		wall = new cbWall;
		/* process attributes */
		const QString &height = attr.value(QString("Height"));
		if (!height.isNull()) wall->setHeight(height.toDouble());
	}
	else if (tag == "Beacon" && lab!=0)
	{
		beacon = new cbBeacon;
		point = cbPoint();
		/* process attributes */
		const QString &x = attr.value(QString("X"));
		if (!x.isNull()) point.setX(x.toDouble());
		const QString &y = attr.value(QString("Y"));
		if (!y.isNull()) point.setY(y.toDouble());
		const QString &height = attr.value(QString("Height"));
		if (!height.isNull()) beacon->setHeight(height.toDouble());
	}
	else if (tag == "Target" && lab!=0)
	{
		target = new cbTarget;
		point = cbPoint();
		/* process attributes */
		const QString &x = attr.value(QString("X"));
		if (!x.isNull()) point.setX(x.toDouble());
		const QString &y = attr.value(QString("Y"));
		if (!y.isNull()) point.setY(y.toDouble());
		const QString &radius = attr.value(QString("Radius"));
		if (!radius.isNull()) target->setRadius(radius.toDouble());
	}
	else if (tag == "Corner" && lab!=0)
	{
		point = cbPoint();
		/* process attributes */
		const QString &x = attr.value(QString("X"));
		if (!x.isNull()) point.setX(x.toDouble());
		const QString &y = attr.value(QString("Y"));
		if (!y.isNull()) point.setY(y.toDouble());
	}
	else if (tag == "Row" && lab!=0)
	{
        int row=0;
		double height=4.0;
		const QString &pos = attr.value(QString("Pos"));
		if (!pos.isNull()) row = pos.toInt();
		const QString &heightStr = attr.value(QString("Height"));
		if (!heightStr.isNull()) height = heightStr.toDouble();
		else height=4.0;
		const QString &pattern = attr.value(QString("Pattern"));
                const QChar *spec = pattern.data();
                int col=0;
                bool inHorizontalWall=false;
                int horWallStartCol, horWallEndCol;
                while (!spec->isNull()) {
                    if(row % 2 == 0) { // only vertical or diagonal walls are allowed here
                        if(spec->toLatin1()=='|') {
		            		wall = new cbWall;
                            wall->addCorner (((col+1)/3.0)*PATHCUBESIZE-PATHWALLWIDTH*0.5, (row*0.5+PATHWALLGAP)*PATHCUBESIZE);
                            wall->addCorner (((col+1)/3.0)*PATHCUBESIZE+PATHWALLWIDTH*0.5, (row*0.5+PATHWALLGAP)*PATHCUBESIZE);
                            wall->addCorner (((col+1)/3.0)*PATHCUBESIZE+PATHWALLWIDTH*0.5, (row*0.5+1.0-PATHWALLGAP)*PATHCUBESIZE);
                            wall->addCorner (((col+1)/3.0)*PATHCUBESIZE-PATHWALLWIDTH*0.5, (row*0.5+1.0-PATHWALLGAP)*PATHCUBESIZE);
							wall->setHeight(height);

                            lab->addWall(wall);
                       	}
	                	else if(spec->toLatin1()=='\\') {
		            		wall = new cbWall;
                            wall->addCorner (((col)/3.0)*PATHCUBESIZE  -PATHWALLWIDTH*0.5*cos(M_PI/4), (row*0.5+1.0+PATHWALLGAP)*PATHCUBESIZE-PATHWALLWIDTH*0.5*sin(M_PI/4));
                            wall->addCorner (((col)/3.0)*PATHCUBESIZE  +PATHWALLWIDTH*0.5*cos(M_PI/4), (row*0.5+1.0+PATHWALLGAP)*PATHCUBESIZE+PATHWALLWIDTH*0.5*sin(M_PI/4));
                            wall->addCorner (((col+3)/3.0)*PATHCUBESIZE+PATHWALLWIDTH*0.5*cos(M_PI/4), (row*0.5-PATHWALLGAP)*PATHCUBESIZE    +PATHWALLWIDTH*0.5*sin(M_PI/4));
                            wall->addCorner (((col+3)/3.0)*PATHCUBESIZE-PATHWALLWIDTH*0.5*cos(M_PI/4), (row*0.5-PATHWALLGAP)*PATHCUBESIZE    -PATHWALLWIDTH*0.5*sin(M_PI/4));
							wall->setHeight(height);

                            lab->addWall(wall);
                   		}
	                	else if(spec->toLatin1()=='/') {
		            		wall = new cbWall;
                            wall->addCorner (((col)/3.0)*PATHCUBESIZE  -PATHWALLWIDTH*0.5*cos(M_PI/4), (row*0.5+PATHWALLGAP)*PATHCUBESIZE    +PATHWALLWIDTH*0.5*sin(M_PI/4));
                            wall->addCorner (((col)/3.0)*PATHCUBESIZE  +PATHWALLWIDTH*0.5*cos(M_PI/4), (row*0.5+PATHWALLGAP)*PATHCUBESIZE    -PATHWALLWIDTH*0.5*sin(M_PI/4));
                            wall->addCorner (((col+3)/3.0)*PATHCUBESIZE+PATHWALLWIDTH*0.5*cos(M_PI/4), (row*0.5+1.0-PATHWALLGAP)*PATHCUBESIZE-PATHWALLWIDTH*0.5*sin(M_PI/4));
                            wall->addCorner (((col+3)/3.0)*PATHCUBESIZE-PATHWALLWIDTH*0.5*cos(M_PI/4), (row*0.5+1.0-PATHWALLGAP)*PATHCUBESIZE+PATHWALLWIDTH*0.5*sin(M_PI/4));
							wall->setHeight(height);

                            lab->addWall(wall);
                   		}
					}
                    else {// only horizontal walls are allowed at odd rows
                       if(col % 3 ==0) { // if there is a wall at this collumn then there must also be a wall in the next one

                           // start of horizontal wall
                           if(spec->toLatin1()=='-' && ! inHorizontalWall) {
                               inHorizontalWall = true;
                               horWallStartCol = col;
                           }

                           // end of horizontal wall
                           if((spec->toLatin1()==' ' && inHorizontalWall)
                              || (spec->toLatin1()=='-' && col == 39)) {

                               inHorizontalWall = false;
                               if(spec->toLatin1()=='-') {
                                     horWallEndCol = col;
                               }
                               else {
                                     horWallEndCol = col-3;
                               }
		               		   wall = new cbWall;

                               wall->addCorner ((horWallStartCol/3.0+PATHWALLGAP)*PATHCUBESIZE, ((row+1)*0.5)*PATHCUBESIZE-PATHWALLWIDTH*0.5);
                               wall->addCorner ((horWallEndCol/3.0+1.0-PATHWALLGAP)*PATHCUBESIZE, ((row+1)*0.5)*PATHCUBESIZE-PATHWALLWIDTH*0.5);
                               wall->addCorner ((horWallEndCol/3.0+1.0-PATHWALLGAP)*PATHCUBESIZE, ((row+1)*0.5)*PATHCUBESIZE+PATHWALLWIDTH*0.5);
                               wall->addCorner ((horWallStartCol/3.0+PATHWALLGAP)*PATHCUBESIZE, ((row+1)*0.5)*PATHCUBESIZE+PATHWALLWIDTH*0.5);
							   wall->setHeight(height);

                               lab->addWall(wall);

                           }

                       }
/*
                        wall->addCorner ((col*0.5)*CUBESIZE, (row*0.5+0.45)*CUBESIZE);
                        wall->addCorner ((col*0.5)*CUBESIZE, (row*0.5+0.55)*CUBESIZE);
                        wall->addCorner ((col*0.5+1.0)*CUBESIZE, (row*0.5+0.55)*CUBESIZE);
                        wall->addCorner ((col*0.5+1.0)*CUBESIZE, (row*0.5+0.45)*CUBESIZE);
*/
                   }
                   spec++;
                   col++;
               }
	}

	//// DEBUG
	//char buff[1024*1024];
	//lab->toXml(buff,sizeof(buff));
	//fprintf(stderr,buff);
    ////
	
	return TRUE;
}

bool cbLabHandler::endElement( const QString&, const QString&, const QString& qName)
{
	/* process end tag */
	const QString &tag = qName;
	if (tag == "Lab")
	{
		//simulator->setLab(lab);
	}
	else if (tag == "Wall")
	{
		if(wall->orderCornersAntiClockwise())
		     lab->addWall(wall);
	}
	else if (tag == "Beacon")
	{
		beacon->setCenter(point);
		lab->addBeacon(beacon);
	}
	else if (tag == "Target")
	{
		target->setCenter(point);
		lab->addTarget(target);
	}
	else if (tag == "Corner")
	{
		wall->addCorner(point);
	}
    return TRUE;
}

void cbLabHandler::setDocumentLocator(QXmlLocator *)
{
}


/* extra functions */

cbLab *cbLabHandler::parsedLab()
{
	return lab;
}
