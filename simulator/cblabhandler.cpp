/*
    This file is part of ciberRatoToolsSrc.

    Copyright (C) 2001-2025 Universidade de Aveiro

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


#include <math.h>
#include <qstring.h>

#include "cblabhandler.h"

#define PATHCUBESIZE  (0.15)
#define PATHWALLWIDTH (0.02)
#define PATHWALLGAP   (0.0)

#define TRUE 1


std::string target_string =
"Transform {"
"  translation  %7.4f %7.4f 0.00005"
"  rotation 0 0 1 0"
"  children ["
"    Shape {"
"      appearance PBRAppearance {"
"        baseColor %7.4f %7.4f %7.4f"
"        roughness 0.5"
"        metalness 0"
"      }"
"      geometry Box {"
"        size 0.14 0.14 0.0001"
"      }"
"    }"
"  ]"
"}";

    // "SolidBox {"
    // "  translation  %7.4f %7.4f 0.00005"
    // "  rotation 0 0 1 0"
    // "      appearance PBRAppearance {"
    // "        baseColor %7.4f %7.4f %7.4f"
    // "        roughness 0.5"
    // "        metalness 0"
    // "      }"
    //   "        size 0.14 0.14 0.0001"
    // "  name \"target\""
    // "}";


std::string vertical_wall_string =
"Transform {"
"  translation  %7.4f %7.4f 0.0001"
"  rotation 0 0 1 0"
"  children ["
"    Shape {"
"      appearance PBRAppearance {"
"        baseColor 0 0 0"
"        roughness 0.5"
"        metalness 0"
"      }"
"      geometry Box {"
"        size 0.014 %7.4f 0.0002"
"      }"
"    }"
"  ]"
"}";

std::string horizontal_wall_string =
"Transform {"
"  translation  %7.4f %7.4f 0.0001"
"  rotation 0 0 1 0"
"  children ["
"    Shape {"
"      appearance PBRAppearance {"
"        baseColor 0 0 0"
"        roughness 0.5"
"        metalness 0"
"      }"
"      geometry Box {"
"        size %7.4f 0.014 0.0002"
"      }"
"    }"
"  ]"
"}";


bool cbLabHandler::startDocument()
{
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

	if (tag == "Lab")
	{
		char wall_str[1024*8];

        lab = new cbLab;

	}
	else if (tag == "Wall")
	{
	}
	else if (tag == "Beacon")
	{
	}
	else if (tag == "Target")
	{
    #define N_TARGET_COLORS 7
		double x=0.0, y=0.0, radius=0.02;
		const QString &x_at = attr.value(QString("X"));
		if (!x_at.isNull()) x = x_at.toDouble();
		const QString &y_at = attr.value(QString("Y"));
		if (!y_at.isNull()) y = y_at.toDouble();
		const QString &radius_at = attr.value(QString("Radius"));
		if (!radius_at.isNull()) radius = radius_at.toDouble();
        float id_color[N_TARGET_COLORS][3] = {{1.0,0.0,0.0},
                          {0.2,1.0,1.0},
                          {1.0,1.0,0.0},
                          {1.0,0.0,1.0},
                          {0.45,1.0,1.0},
                          {1.0,0.45,0.0},
                          {0.0,0.0,0.0}};
        char target_str[1024*8];
        sprintf(target_str, target_string.c_str(),
                x * PATHCUBESIZE*0.5 + offsetX*PATHCUBESIZE, y * PATHCUBESIZE*0.5 + offsetY*PATHCUBESIZE,
                id_color[target_id%N_TARGET_COLORS][0], id_color[target_id%N_TARGET_COLORS][1], id_color[target_id%N_TARGET_COLORS][2]);
		children_field->importMFNodeFromString(-1, target_str);
        target_id++;

        cbTarget *target = new cbTarget;
        cbPoint center(x * PATHCUBESIZE*0.5, y * PATHCUBESIZE*0.5);
        target->setCenter(center);
        target->setRadius(radius * PATHCUBESIZE*0.5);
        lab->addTarget(target);
    }
	else if (tag == "Corner")
	{
	}
	else if (tag == "Row")
	{
		char wall_str[1024*8];
        int row=0;
		double height=0.05;
		const QString &pos = attr.value(QString("Pos"));
		if (!pos.isNull()) row = pos.toInt();
		const QString &heightStr = attr.value(QString("Height"));
		if (!heightStr.isNull()) height = heightStr.toDouble();
		else height=0.05;
		const QString &pattern = attr.value(QString("Pattern"));
                const QChar *spec = pattern.data();
                int col=0;
                bool inHorizontalWall=false;
                int horWallStartCol=0, horWallEndCol=0;
                while (!spec->isNull()) {
                        if(spec->toLatin1()=='|') {
		            		wall = new cbWall;
                            wall->addCorner (((col+1)/3.0)*PATHCUBESIZE-PATHWALLWIDTH*0.5, (row*0.5+PATHWALLGAP)*PATHCUBESIZE);
                            wall->addCorner (((col+1)/3.0)*PATHCUBESIZE+PATHWALLWIDTH*0.5, (row*0.5+PATHWALLGAP)*PATHCUBESIZE);
                            wall->addCorner (((col+1)/3.0)*PATHCUBESIZE+PATHWALLWIDTH*0.5, (row*0.5+1.0-PATHWALLGAP)*PATHCUBESIZE);
                            wall->addCorner (((col+1)/3.0)*PATHCUBESIZE-PATHWALLWIDTH*0.5, (row*0.5+1.0-PATHWALLGAP)*PATHCUBESIZE);
							wall->setHeight(height);

                            lab->addWall(wall);
							sprintf(wall_str, vertical_wall_string.c_str(),
							        ((col+1)/3.0)*PATHCUBESIZE  + offsetX*PATHCUBESIZE,
							        (row*0.5+0.5)*PATHCUBESIZE + offsetY*PATHCUBESIZE,
                                    PATHCUBESIZE,
							        "vertical_wall");
							children_field->importMFNodeFromString(-1, wall_str);
                       	}
                       	else if(col % 3 ==0) { // if there is a wall at this collumn then there must also be a wall in the next one

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
								if (row % 2 == 1) {
							         wall->setHeight(height);
								}
								else {
							         wall->setHeight(0.0);  // horizontal walls in middle of cells
								}

                               lab->addWall(wall);
							sprintf(wall_str, horizontal_wall_string.c_str(),
							        ((horWallStartCol+horWallEndCol)/2.0/3.0+0.5)*PATHCUBESIZE+ offsetX*PATHCUBESIZE,
							        ((row+1)*0.5)*PATHCUBESIZE+ offsetY*PATHCUBESIZE,
							        ((horWallEndCol - horWallStartCol)/3.0+1.0-2.0*PATHWALLGAP)*PATHCUBESIZE,		
							        "horizontal_wall");
							children_field->importMFNodeFromString(-1, wall_str);
                           }

                       }
/*
                        wall->addCorner ((col*0.5)*CUBESIZE, (row*0.5+0.45)*CUBESIZE);
                        wall->addCorner ((col*0.5)*CUBESIZE, (row*0.5+0.55)*CUBESIZE);
                        wall->addCorner ((col*0.5+1.0)*CUBESIZE, (row*0.5+0.55)*CUBESIZE);
                        wall->addCorner ((col*0.5+1.0)*CUBESIZE, (row*0.5+0.45)*CUBESIZE);
*/
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
	}
	else if (tag == "Beacon")
	{
	}
	else if (tag == "Target")
	{
	}
	else if (tag == "Corner")
	{
	}
    return TRUE;
}

void cbLabHandler::setDocumentLocator(QXmlLocator *)
{
}

void cbLabHandler::setChildrenField(webots::Field *field) {
	children_field = field;
}


/* extra functions */
