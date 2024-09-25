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

#ifndef CBSENSOR_H
#define CBSENSOR_H

/*! \file cbsensors.h
	\brief Defini��o dos sensores que equipam um rob�.

	Existem os seguintes sensores
	<ul>
		<li> cbCollisionSensor </li>
		<li> cbCompassSensor </li>
		<li> cbBeaconSensor </li>
		<li> cbIRSensor </li>
		<li> cbGroundSensor </li>
		<li> cbGPSSensor </li>
	</ul>
*/

#include <math.h>

#include <queue>
#include <vector>

using std::queue;
using std::vector;

#include "cbsimulator.h"
#include "cbpoint.h"

class cbMeasure
{
	public:
	    cbMeasure(void);
        virtual ~cbMeasure(void) {}
};

class cbBoolMeasure : public cbMeasure
{
	public:
	    cbBoolMeasure(bool val);
	    bool value;
};

class cbBoolArrayMeasure : public cbMeasure
{
	public:
	    cbBoolArrayMeasure(vector<bool> val);
	    vector<bool> value;
};

class cbIntMeasure : public cbMeasure
{
	public:
	    cbIntMeasure(int val);
	    int value;
};

class cbDoubleMeasure : public cbMeasure
{
	public:
	    cbDoubleMeasure(double val);
	    double value;
};

struct beaconMeasure {
        bool   beaconVisible;  /* true if robot can see beacon */
        double beaconDir;      /* direction of beacon */
                               /*   only valid if beaconVisible is true */
};

class cbBeaconMeasure : public cbMeasure
{
	public:
	    cbBeaconMeasure(struct beaconMeasure val);
	    struct beaconMeasure value;
};

struct GPSMeasure {
   double x,y,degrees;
};

class cbGPSMeasure : public cbMeasure
{
	public:
	    cbGPSMeasure(struct GPSMeasure val);  
            struct GPSMeasure value;
};

class cbSensor 
{
public:
	cbSensor(cbRobot *rob, QString sId);
	virtual ~cbSensor();

	virtual void update()=0;

    QString &getStrId() { return strId; }
    void    setRequestable(bool v) {requestable = v; }

	void        setFifoLatency(unsigned int fLatency);
	void        pushMeasure (cbMeasure *val);
	void        popMeasure  (void);
	cbMeasure*  readMeasure (void);

	bool    requested;     //it was requested this cycle
	bool    requestable;   //it must be requested to be sent

protected:
	cbRobot *robot;
	QString strId;
	int fifoLatency;
	queue <cbMeasure *> measuresFIFO;
};

/*! \class cbCollisionSensor
	\brief Sensor de colis�es
	
	Detecta se o rob� se encosta a qualquer obst�culo,
	parede ou outro rob�.
*/
class cbCollisionSensor : public cbSensor
{
public:
	cbCollisionSensor(cbRobot *rob, QString sId);
	virtual ~cbCollisionSensor();

	void update(void);
	inline bool Value() { 
		cbBoolMeasure *pMeas = dynamic_cast<cbBoolMeasure *> (readMeasure());
		if(pMeas)
		   return pMeas->value; 
		else
		   return false;
        }

	static  int  sensorLatency;
	static  bool sensorRequestable;

//	cbArray<double> SensMeas;    // DEBUG only
private:
	inline void update(bool b) { value = b; }
	bool value;
};


/*! \class CBGroundSensor
		\brief Sensor de �rea de Chegada

		Sensor do tipo ON/OFF que indica se o ch�o por
		debaixo do rob� � escuro ou claro.
*/
class cbGroundSensor : public cbSensor
{
public:
	cbGroundSensor(cbRobot *rob, QString sId);
	virtual ~cbGroundSensor();

	void update(void);
	inline int Value() { 
		cbIntMeasure *pMeas = dynamic_cast<cbIntMeasure *> (readMeasure());
		if(pMeas)
		   return pMeas->value; 
		else
		   return -1;
        }
	static  int  sensorLatency;
	static  bool sensorRequestable;

private:
	int value;
};


/*! \class CBCompassSensor
	\brief B�ssula

*/
class cbCompassSensor : public cbSensor
{
public: // static data members
	static double noise;
public:
	cbCompassSensor(cbRobot *rob, QString sId);
	virtual ~cbCompassSensor();

	void update(void);
	inline double Degrees() { 
		cbDoubleMeasure *pMeas = dynamic_cast<cbDoubleMeasure *> (readMeasure());
		if(pMeas)
		   return pMeas->value; 
		else
		   return 0.0;
        }
	static  int  sensorLatency;
	static  bool sensorRequestable;

private:
	void update(double orientation);
	double ideal;	// real orientation of robot
	double value;	// orientation measured by sensor
	double degrees;	// value in degrees
};


/*! \class CBBeaconSensor
		\brief Sensor de Farol

		Sensor que indica a direc��o a que se encontra o farol.
*/
class cbBeaconSensor : public cbSensor
{
public: // static data members
	static double noise;
public:
	cbBeaconSensor(cbRobot *rob, QString sId);
	~cbBeaconSensor();

	void update();

	inline double Degrees() { 
		cbBeaconMeasure *pMeas = dynamic_cast<cbBeaconMeasure *> (readMeasure());
		if(pMeas)
		   return pMeas->value.beaconDir; 
		else
		   return 0.0;
        }
	inline bool BeaconVisible() { 
		cbBeaconMeasure *pMeas = dynamic_cast<cbBeaconMeasure *> (readMeasure());
		if(pMeas)
		   return pMeas->value.beaconVisible; 
		else
		   return false;
        }
	inline bool   Ready()         { return ready; }    // this function is deprecated, BeaconSensors are always ready!!
	inline void   setBeaconToFollow(int b) { beaconToFollow = b; }

	static  int    sensorLatency;
	static  bool   sensorRequestable;
	static  double sensorAperture;
private:
	void resetBeaconVisible(void) { measure.beaconVisible=false; } 

private:
	double ideal;	// real direction of beacon
	double value;	// direction measure by sensor before rounding
	int    beaconToFollow; // Id of beacon that is detected by this sensor

	//double degrees;	// value in degrees
	//bool   beaconVisible;
	struct beaconMeasure measure;

	int    latency;       // number of cycles for each measure
	int    cycToMeasure;  // number of cycles for next measure
	bool   ready;
};

/*! \class CBIRSensor
	\brief Sensor de Dist�ncia

	Sensor de infra-vermelhos que permite medir a dist�ncia
	aproximada a um obst�culo.
	O seu posicionamento � definido em coordenadas polares.
	Considera-se que o sensor radial relativamente a esse posicionamento.
*/
class cbIRSensor : public cbSensor
{
public: // static data members
    static double noise;

public:
	cbIRSensor(cbRobot *rob, QString sId);
	virtual ~cbIRSensor();

	void update(void);

	inline void setPosition(double r, double t) { pos.ro = r; pos.theta = t; }

	inline cbPolarPoint &position() { return pos; }

	inline double Value() { 
		cbDoubleMeasure *pMeas = dynamic_cast<cbDoubleMeasure *> (readMeasure());
		if(pMeas)
		   return pMeas->value; 
		else
		   return 0.0;
        }
	static  int  sensorLatency;
	static  bool sensorRequestable;

private:
	void update(double);
	cbPolarPoint pos; // (relative) position of sensor
	double value;
	double ideal;
};



/*! \class CBGPSSensor
	\brief GPS

*/
class cbGPSSensor : public cbSensor
{
public: // static data members
	static double noiseLin;
	static double noiseAng;

	static double offsetX;
	static double offsetY;
	static double offsetDeg;
public:
	cbGPSSensor(cbRobot *rob, QString sId);
	virtual ~cbGPSSensor();

	void update(void);
	inline double X() { 
		cbGPSMeasure *pMeas = dynamic_cast<cbGPSMeasure *> (readMeasure());
		if(pMeas)
		   return pMeas->value.x; 
		else
		   return 0.0;
        }
	inline double Y() { 
		cbGPSMeasure *pMeas = dynamic_cast<cbGPSMeasure *> (readMeasure());
		if(pMeas)
		   return pMeas->value.y; 
		else
		   return 0.0;
        }
	inline double Degrees() { 
		cbGPSMeasure *pMeas = dynamic_cast<cbGPSMeasure *> (readMeasure());
		if(pMeas)
		   return pMeas->value.degrees; 
		else
		   return 0.0;
        }
	static  int  sensorLatency;
	static  bool sensorRequestable;

private:
	void update(GPSMeasure ideal);
	GPSMeasure value;
};

/*! \class LineSensor
	\brief LineGroundSensor

*/

#define NLINESENSORELEMENTS 7
#define LINESENSORELDIST 0.08
#define LINETRUEPROB 0.98

class cbLineSensor : public cbSensor
{
public: // static data members
    static double noise;

public:
	cbLineSensor(cbRobot *rob, QString sId);
	virtual ~cbLineSensor();

	void update(void);

	inline void setPosition(double x) {
		for (int i=0; i< NLINESENSORELEMENTS; i++) {
			pos[i].x = x; 
			pos[i].y = -i * LINESENSORELDIST+(NLINESENSORELEMENTS/2)*LINESENSORELDIST; 
		} 
	}

	inline cbPoint &position(int i) { return pos[i]; }

	inline vector<bool> Value() { 
		cbBoolArrayMeasure *pMeas = dynamic_cast<cbBoolArrayMeasure *> (readMeasure());
		vector<bool> falseVals = vector<bool>(NLINESENSORELEMENTS,false);
		if(pMeas)
		   return pMeas->value; 
		else
		   return falseVals;
        }
	static  int  sensorLatency;
	static  bool sensorRequestable;

private:
	void update(vector<bool>);
	cbPoint pos[NLINESENSORELEMENTS]; // (relative) position of center of line sensor
	vector<bool> value;
};

#endif
