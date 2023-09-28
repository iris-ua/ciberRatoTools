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

#include "cbbeacon.h"
#include "cbtarget.h"
#include "cbrobot.h"
#include "cblab.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>


#include <algorithm>

cbLab::cbLab(void)
{
	/* init attributes */
	name = "NO NAMED LAB";
	height = width = 16.0;

	/* make border wall and add it to walls */
	cbWall *border = new cbWall;
	border->addCorner(0,0);
	border->addCorner(0,height);
	border->addCorner(width,height);
	border->addCorner(width,0);
	addWall(border);
}

cbLab::~cbLab(void)
{
	delete name;
}

void cbLab::setName(const char *aName)
{
	name = strdup(aName);
}

void cbLab::setWidth(double w)
{
	width = w;
	vector<cbPoint> &border = walls[0]->Corners();
	border[2].x = border[3].x = w;
}

void cbLab::setHeight(double h)
{
	height = h;
	vector<cbPoint> &border = walls[0]->Corners();
	border[1].y = border[2].y = h;
}

void cbLab::addBeacon(cbBeacon *b)
{
	beacons.push_back(b);
	//cerr << "cbLab::addBeacon now lab has"<< nBeacons() << "beacons\n";
}

void cbLab::addTarget(cbTarget *t)
{
	targets.push_back(t);
}

void cbLab::addWall(cbWall *wall)
{
//	unsigned int cnt = walls.size();
	unsigned int cnt = walls.size();
	walls.resize(cnt+1);
	walls[cnt] = wall;
}

/*!
	Determine and return minimum distance from given
	point to any wall above 0.0 height in lab
*/
double cbLab::wallDistance(cbPoint &p)
{
	return wallDistanceAboveHeight(p, 0.0);
}

/*!
	Determine and return minimum distance from given
	point to any wall in lab above given height
*/
double cbLab::wallDistanceAboveHeight(cbPoint &p, double height)
{
	double min = width+height;	// this is an upper bound
//	for (unsigned int i=0; i<walls.size(); i++)
	for (unsigned int i=0; i<walls.size(); i++)
	{
		if(walls[i]->Height() <= height) continue;

		double dist = walls[i]->distance(p);
		//cout.form("walls[%u]->distance(%g,%g) = %g\n", i, p.x, p.y, dist);
		if (dist < min) min = dist;
	}
	return min;
}


/*!
	Determine and return if point is inside wall
*/
bool cbLab::isInside(const cbPoint &p)
{
	bool inside = false;
	for (unsigned int i=1; i<walls.size(); i++) // wall 0 must not be considered as it models the outer border!!!
	{
		if (walls[i]->isInside(p)) { 
		  inside=true;
		}
	}
	return inside;
}

/*!
	Determine and return minimum distance from given
	point, in given direction to any wall in lab.
	Return -1 if no wall point is seen from the given point.
*/
double cbLab::wallDistance(cbPoint &p, double dir)
{
	double min = -1;
	for (unsigned int i=0; i<walls.size(); i++)
	{
		if(walls[i]->Height() <= 0.0) continue;

		double dist = walls[i]->distance(p, dir);
		//cout.form("min = %g, dist = walls[%u]->distance(%g,%g,%g) = %g\n", min, i, p.x, p.y, dir, dist);
		//fprintf(stderr,"min = %g, dist = walls[%u]->distance(%g,%g,%g) = %g\n", 
		//	min, i, p.x, p.y, dir, dist);
		if (dist != -1 && (min == -1 || dist < min)) min = dist;
	}
	return min;
}

/*!
	Determine and return minimum distance from given
	point, in given direction to a wall in lab higher, 
	or equally high, as height.
	Return -1 if no wall point is seen from the given point.
*/
double cbLab::wallDistanceAboveHeight(cbPoint &p, double dir, double height)
{
	double min = -1;
	for (unsigned int i=0; i<walls.size(); i++)
	{
		if(walls[i]->Height() < height) continue; // skip wall

		double dist = walls[i]->distance(p, dir);
		//cout.form("min = %g, dist = walls[%u]->distance(%g,%g,%g) = %g\n", min, i, p.x, p.y, dir, dist);
		//fprintf(stderr,"min = %g, dist = walls[%u]->distance(%g,%g,%g) = %g\n", 
		//	min, i, p.x, p.y, dir, dist);
		if (dist != -1 && (min == -1 || dist < min)) min = dist;
	}
	return min;
}


/*!
	Given an angular range determine and return distance 
	to closer corner of all walls in lab, cover by that range.
	Return -1 if no wall corner is seen from the given point.
*/
double cbLab::cornerDistance(cbPoint &p, double dir, double range)
{
	double min = -1;
	for (unsigned int i=0; i<walls.size(); i++)
	{
		if(walls[i]->Height() <= 0.0) continue; // skip wall

		double dist = walls[i]->cornerDistance(p, dir, range);
		//cout.form("min = %g, dist = walls[%u]->cornerDistance(%g,%g,%g,%g) = %g\n", min, i, p.x, p.y, dir, range, dist);
		if (dist != -1 && (min == -1 || dist < min)) min = dist;
	}
	return min;
}


/*!
	Fill in buff with a XML representation of the lab.
	Returns the number of positions filled in.
*/
int cbLab::toXml(char *buff, int n)
{
	unsigned int i;
	int cnt = 0;
	cnt += sprintf(buff, "<Lab Name=\"%s\" Height=\"%g\" Width=\"%g\">\n", name, height, width);
	for (i=0; i<beacons.size(); i++)
	{
		cbPoint &p = beacons[i]->Center();
		double h = beacons[i]->Height();
		cnt += sprintf(buff+cnt, "\t<Beacon X=\"%g\" Y=\"%g\" Height=\"%g\"/>\n", p.X(), p.Y(), h);
	}
	for (i=0; i<targets.size(); i++)
	{
		cbPoint &p = targets[i]->Center();
		double r = targets[i]->Radius();
		cnt += sprintf(buff+cnt, "\t<Target X=\"%g\" Y=\"%g\" Radius=\"%g\"/>\n", p.X(), p.Y(), r);
	}
	for (i=1; i<walls.size(); i++)
	{
		cbWall *wall = walls[i];
		cnt += sprintf(buff+cnt, "\t<Wall Height=\"%g\">\n", wall->Height());
		vector<cbPoint> *corners = &(wall->Corners());
		for (unsigned int j=0; j<corners->size(); j++)
		{
			cbPoint p = (*corners)[j];
			cnt += sprintf(buff+cnt, "\t\t<Corner X=\"%g\" Y=\"%g\"/>\n", p.X(), p.Y());
		}
		cnt += sprintf(buff+cnt, "\t</Wall>\n");
	}
	cnt += sprintf(buff+cnt, "</Lab>\n");

        if(cnt>n-1) {
             fprintf(stderr,"cbLab::toXml message too long\n");
             abort();
        }
	return cnt;
}

bool cbLab::reachable(cbPoint i,cbPoint f)
{
    double distIF,dirIF,distWall;

    dirIF=(f-i).angle();
    distIF=i.distance(f);

    distWall=wallDistance(i,dirIF);       

//    fprintf(stderr,"reach i(%5.3f,%5.3f),f(%5.3f,%5.3f) dist=%5.3f angle=%5.3f"
//		   " distWall=%5.3f\n",
//		   i.X(),i.Y(),f.X(),f.Y(),distIF,dirIF*180.0/M_PI,distWall);
    
    if(distWall>=0.0 && distWall < distIF)
           return false;

    return true;
}

bool cbLab::reachableRobot(cbPoint i,cbPoint f)
{
    cbPoint auxi,auxf, offset;

    if(wallDistance(i)<ROBOT_RADIUS*0.999) return false;
    if(wallDistance(f)<ROBOT_RADIUS*0.999) return false;

    if(!reachable(i,f)) return false;

    offset=(f-i).normalize().rotate(M_PI/2)*ROBOT_RADIUS*0.999;

    auxi=i+offset;
    auxf=f+offset;
    if(!reachable(auxi,auxf)) return false;

    auxi=i-offset;
    auxf=f-offset;
    if(!reachable(auxi,auxf)) return false;

    return true;
}

const char * cbLab::Name(void)
{
    return name;
}

