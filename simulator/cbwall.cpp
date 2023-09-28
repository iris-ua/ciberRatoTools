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

#include "cbwall.h"
#include "cbpoint.h"

#include <math.h>
#ifndef M_PI
#ifdef MicWindows
#define M_PI 3.14159265358979
#endif
#endif
#include <iostream>
#include <assert.h>

using std::cerr;


cbWall::cbWall()
{
	height = 4.0;
}

cbWall::~cbWall()
{
}

void cbWall::setHeight(double h)
{
	height = h;
}

void cbWall::addCorner(cbPoint &p)
{
	unsigned int cnt = corners.size();
	corners.resize(cnt+1);
	corners[cnt] = p;
	//cout.form("cbWall::addCorner(%g,%g)\n", p.x, p.y);
	//SHOWCORNERS
}

void cbWall::addCorner(double x, double y)
{
	unsigned int cnt = corners.size();
	corners.resize(cnt+1);
	corners[cnt].x = x;
	corners[cnt].y = y;
}

double cbWall::Height()
{
	return height;
}

vector<cbPoint> &cbWall::Corners()
{
	return corners;
}

/*!
	Determine and return the minimum distance from the given
	point to any of the wall faces.
*/
double cbWall::distance(cbPoint &p)
{
	//cout.form("cbWall::distance(%g,%g)\n", p.x, p.y);
	//showCorners();
	unsigned int n = corners.size();
	cbPoint *a = &corners[n-1];
	double min = p.distance(a);	// first value for minimum distance
	for (unsigned int i=0; i<n; i++)
	{
		/* determine distance to face */
		double dist;
		cbPoint *b = &corners[i];
//cout.form("Point = %g,%g\n", b->x, b->y);
		double abLen = a->distance(b);
		double abNormX = (b->x - a->x) / abLen;
		double abNormY = (b->y - a->y) / abLen;

//cout.form("Face (%g,%g) - (%g,%g) ", a->x, a->y, b->x, b->y);
		if (abNormX * (p.x-a->x) + abNormY * (p.y-a->y) < 0.0)// nearest point is a
		{
			dist = p.distance(a);
		}
		else if (abNormX * (p.x-b->x) + abNormY * (p.y-b->y) > 0.0)// nearest point is b
		{
			dist = p.distance(b);
		}
		else	// nearest point is somewhere between a and b
		{
			dist = fabs(abNormY * (p.x-a->x) - abNormX * (p.y-a->y));
		}
//cout.form(", dist = %g\n", dist);

		/* set minimum until now */
		if (dist < min) min = dist;

		/* set next point a */
		a = b;
	}
	//cout.form("cbWall::distance(%g,%g) = %g\n", p.x, p.y, min);
	return min;
}

/*!
	Determine and return the minimum distance from the given
	point, in the given direction to any of the wall faces.
	Return -1 if no face intercepts the line defined by given point
	and angle.
*/
/*
	Seja \p e \theta o ponto e a direc��o dados; seja \pa e \pb
	os pontos definidores de uma face da parede.
	\p e \theta definem uma recta no plano.
	\pa e \pb definem uma recta no plano
	As duas rectas anteriores podem ser coincidentes, paralelas ou 
	concorrentes. 
	No primeiro caso a dist�ncia a determinar � a menor dist�ncia
	de \p a \pa ou a \pb.
	No segundo caso n�o h� dist�ncia.
	No terceiro, � preciso verificar se o ponto de intercep��o est� dentro ou
	fora do segmento de recta correspondente � face da parede.
*/
double cbWall::distance(cbPoint &p, double theta)
{
	//cout.form("cbWall::distance(%g,%g,%g)\n", p.x, p.y,theta);
	//fprintf(stderr,"cbWall::distance(%g,%g,%g)\n", p.x, p.y,theta);
	//SHOWCORNERS
	unsigned int n = corners.size();
	/* get first corner data */
	cbPoint *a = &corners[n-1];

        while(theta>M_PI) theta-=2*M_PI;
        while(theta<-M_PI) theta+=2*M_PI;

        double alpha = p.angle(a);
	double dist_a = p.distance(a);
	/* init min */
	double min = -1, dist;
	/* process all wall faces */
	for (unsigned int i=0; i<n; i++)
	{
		/* get second corner data */
		cbPoint *b = &corners[i];
		double beta = p.angle(b);
		double dist_b = p.distance(b);
		//cout.form("Face %g,%g (%g) -- %g,%g (%g)\n", a->x, a->y, alpha, b->x, b->y, beta);
		//fprintf(stderr,"Face %g,%g (%g) -- %g,%g (%g)\n", a->x, a->y, alpha, b->x, b->y, beta);

		/* skip face if no interception exist */
		double delta = beta - alpha;
		//cout.form("Delta = %g\n", delta);
		if ( (delta > M_PI && theta > alpha && theta < beta) 
			|| (delta < -M_PI && theta > beta && theta < alpha)
			|| (delta <= M_PI && delta > 0 && (theta < alpha || theta > beta))
			|| (delta >= -M_PI && delta < 0 && (theta < beta || theta > alpha)) )
		{
			//cout.form("  Face skipped\n");
			//fprintf(stderr,"  Face skipped\n");
			a = b;
			alpha = beta;
			dist_a = dist_b;
			continue;
		}

		/* if lines are coincident determine minimum from \p to \a or from \p to \b */
		if (alpha == beta) // they are coincident 
		{
			if (theta == alpha)
			   dist = (dist_a < dist_b) ? dist_a : dist_b;
			else {
			   //cout.form("  Face skipped\n");
			   //fprintf(stderr,"  Face skipped\n");
			   a = b;
			   alpha = beta;
			   dist_a = dist_b;
			   continue;
			}
		}
		else //Lines are concurrent, but not coincident.
		{
			/* Determine interception point in face.
		 	 * Then determine distance from \p to interception point */
			double x,y;
			if (theta == M_PI/2)	// point+direction is vertical line
			{
				x = p.x;
				y = ((b->y-a->y)/(b->x-a->x))*(x-a->x) + a->y;
			}
			else if (a->x == b->x) // face is vertical line
			{
				x = a->x;
				y = tan(theta)*(x-p.x) + p.y;
			}
			else // no line is vertical
			{
				double m1 = tan(theta);
				double b1 = p.y - m1*p.x;
				double m2 = (b->y-a->y)/(b->x-a->x);
				double b2 = a->y - m2*a->x;
				x = (b1-b2)/(m2-m1);
				y = (m2*b1 - m1*b2)/(m2-m1);
			}
			//cout.form("else: x = %g, y = %g\n", x, y);
			dist = p.distance(x,y);
		}

		/* update minimum until now */
		//cout.form("dist = %g\n", dist);
		//fprintf(stderr,"dist = %g\n", dist);
		if (min == -1 || dist < min) min = dist;

		/* prepare next step */
		a = b;
		alpha = beta;
		dist_a = dist_b;
	}

	/* return minimum */
	return min;
}

/*!
	Determine and return the minimum distance from a given point, in a 
	given direction, with a given angle tolerance
	to all the corners in the wall.
	Return -1 if no corner is seen.
*/
double cbWall::cornerDistance(cbPoint &p, double dir, double margin)
{
	double min = -1;
	
	unsigned int n = corners.size();
	if (n == 0) return min;

	/* determine the minimum */
	for (unsigned int i=0; i<n; i++)
	{
		cbPoint corner = corners[i];
		
		/* skip corner if it is not seen by given cover cone */
		double theta = p.angle(corner);
		double delta = fabs(theta - dir);
		if (delta > M_PI) delta = 2*M_PI - delta;
		if (delta > margin) continue;

		/* determine distance from corner to given point and update
		 * min if distance is lower. */
		double dist = p.distance(corner);
		if (min == -1 || dist < min) min = dist;
	}

	return min;
}

// adapted from https://wrfranklin.org/Research/Short_Notes/pnpoly.html
//int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)
//{
//  int i, j, c = 0;
//  for (i = 0, j = nvert-1; i < nvert; j = i++) {
//    if ( ((verty[i]>testy) != (verty[j]>testy)) &&
//	 (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
//       c = !c;
//  }
//  return c;
//}
bool cbWall::isInside(const cbPoint &p)
{
	unsigned int n = corners.size();
	if (n == 0) return false;
	
	bool inside=false;

	int j;
  	for (int i = 0; i < n; i++) {
		cbPoint cornerI = corners[i];
		cbPoint cornerJ = corners[i-1<0?n-1:i-1];
    	if ( ((cornerI.Y()>p.Y()) != (cornerJ.Y()>p.Y())) &&
     		(p.X() < (cornerJ.X()-cornerI.X()) * (p.Y()-cornerI.Y()) / (cornerJ.Y()-cornerI.Y()) + cornerI.X()) )
             inside = ! inside;
  	}
  	return inside;
}
		
void cbWall::showCorners()
{
	for (unsigned i=0; i<corners.size(); i++)
	{
		std::cout << "corner " << i << " = " << corners[i].x << " "  << corners[i].y << std::endl;
	}
	std::cout << std::endl;
}

double cbWall::cornerAngle(unsigned int c)
{
	unsigned int cbefore, cafter;
	double angle;

	assert(corners.size()>2);
	assert(c<corners.size());

	if( c == 0 )
		cbefore=corners.size()-1;
	else cbefore=c-1;

	if( c == corners.size()-1 )
		cafter=0;
	else cafter=c+1;

	angle = (corners[cafter]-corners[c]).angle()
		-(corners[cbefore]-corners[c]).angle();

	if(angle>M_PI) angle-=2*M_PI;
	if(angle<-M_PI) angle+=2*M_PI;

	return angle;
}

bool cbWall::convexCorner(unsigned int c)
{
     return (cornerAngle(c)<0.0);
}

bool cbWall::orderCornersAntiClockwise(void)
{
	unsigned int c;
	double ang, diffang, sumAngles;

	if(corners.size()<3) return false;

	ang=(corners[0]-corners[corners.size()-1]).angle();
	sumAngles=0;
	for(c=0;c<corners.size()-1;c++) {

	    diffang=(corners[c+1]-corners[c]).rotate(-ang).angle();
	    ang=(corners[c+1]-corners[c]).angle();
	    sumAngles+=diffang;
	}

	diffang=(corners[0]-corners[corners.size()-1]).rotate(-ang).angle();
	sumAngles+=diffang;

	if(fabs(sumAngles-2*M_PI)<0.001) return true;
	if(fabs(sumAngles+2*M_PI)<0.001) {
//	        cerr << "Inverting corner order (anticlockwise)\n";
		invertCornerOrder();
		return true;
	}

	cerr << "Verify lab file, Found error in wall\n";
	return false;
}

void cbWall::invertCornerOrder(void)
{
	unsigned int c;
	cbPoint cornerAux;

	if(corners.size()<2) return;

	for(c=0;c<corners.size()/2;c++) {
               cornerAux=corners[c];
               corners[c]=corners[corners.size()-1-c];
               corners[corners.size()-1-c]=cornerAux;
	}
}
