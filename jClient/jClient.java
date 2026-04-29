/*
    This file is part of ciberRatoToolsSrc.

    Copyright (C) 2001-2015 Universidade de Aveiro

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

import java.io.*;
import java.net.*;
import java.util.*;
import java.util.Vector;

import org.xml.sax.*;
import org.xml.sax.helpers.DefaultHandler;
import javax.xml.parsers.SAXParserFactory; 
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.parsers.SAXParser; 


import ciberIF.*;

/** 
 *  the map
 */
class Map {
    static final int CELLROWS = 7;
    static final int CELLCOLS = 14;

    /*! In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
     *  to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
     */
	char[][] labMap; 

    public Map()
    {
	    labMap = new char[CELLROWS*2-1][CELLCOLS*2-1];  
         
        for(int r=0; r < labMap.length; r++) {
            Arrays.fill(labMap[r],' ');
        }
    }
  
};

/**
 *  class MapHandler parses a XML file defining the labyrinth
 */
class MapHandler extends DefaultHandler {

	/**
	 */
	private Map map;

	/**
	 * returns the Parameters collected during parsing of message
	 */
	Map getMap()
	{
		return map;
	}

	public void startElement(String namespaceURI,
	                         String sName, // simple name
	                         String qName, // qualified name
	                         Attributes attrs)
	throws SAXException
	{
            
	    //Create map object to hold map
	    if(map == null) map = new Map();

		if(qName.equals("Row")) {  // Row Values
    
            if (attrs != null) {
                String rowStr=attrs.getValue("Pos"); 
                if(rowStr!=null) {
                    int row = Integer.valueOf(rowStr).intValue(); 
		            String pattern = attrs.getValue("Pattern");
                    for(int col=0; col < pattern.length(); col++) {
                       if(row % 2 == 0) { // only vertical walls are allowed here
                            if(pattern.charAt(col)=='|') {                 
                               map.labMap[row][(col+1)/3*2-1] = '|';
                            }
                       }
                       else {// only horizontal walls are allowed at odd rows 
                           if(col % 3 == 0) { // if there is a wall at this collumn then there must also be a wall in the next one
                               if(pattern.charAt(col)=='-') {  
                                  map.labMap[row][col/3*2] = '-';
                               }
                           }
                       }
                    }
                }
            }
        }
	}

	public void endElement(String namespaceURI,
			        String sName, // simple name
			        String qName  // qualified name
						        )
	throws SAXException
	{
	} 
};


/**
 * example of a basic agent
 * implemented using the java interface library.
 */
public class jClient {

    ciberIF cif;
    Map map;

    enum State {RUN, WAIT, RETURN}

    public static void main(String[] args) {

        String host, robName;
        int pos; 
        int arg;
        Map map;

        //default values
        host = "localhost";
        robName = "jClient";
        pos = 1;
        map = null;


        // parse command-line arguments
        try {
            arg = 0;
            while (arg<args.length) {
                if(args[arg].equals("--pos") || args[arg].equals("-p")) {
                        if(args.length > arg+1) {
                                pos = Integer.valueOf(args[arg+1]).intValue();
                                arg += 2;
                        }
                }
                else if(args[arg].equals("--robname") || args[arg].equals("-r")) {
                        if(args.length > arg+1) {
                                robName = args[arg+1];
                                arg += 2;
                        }
                }
                else if(args[arg].equals("--host") || args[arg].equals("-h")) {
                        if(args.length > arg+1) {
                                host = args[arg+1];
                                arg += 2;
                        }
                }
                else if(args[arg].equals("--map") || args[arg].equals("-m")) {
                        if(args.length > arg+1) {
                                 
                                MapHandler mapHandler = new MapHandler();

                                SAXParserFactory factory = SAXParserFactory.newInstance();
                                SAXParser saxParser = factory.newSAXParser();
                                FileInputStream fstream=new FileInputStream(args[arg+1]);
                                saxParser.parse( fstream, mapHandler ); 

                                map = mapHandler.getMap();

                                arg += 2;
                        }
                }
                else throw new Exception();
            }
        }
        catch (Exception e) {
                print_usage();
                return;
        }
        
        // create client
        jClient client = new jClient();

        client.robName = robName;

        // register robot in simulator
        client.cif.InitRobot(robName, pos, host);
        client.map = map;
        client.printMap();
        
        // main loop
        client.mainLoop();
        
    }

    // Constructor
    jClient() {
            cif = new ciberIF();
            beacon = new beaconMeasure();

            beaconToFollow = 0;
            ground=-1;

            state = State.RUN;
    }

    /** 
     * reads a new message, decides what to do and sends action to simulator
     */
    public void mainLoop () {

        while(true) {
                cif.ReadSensors();
                decide();
        }
    }

    public void wander(boolean followBeacon) {
        if(irSensor0>4.0 || irSensor1>4.0 ||  irSensor2>4.0) 
                cif.DriveMotors(-0.1,+0.1);
        else if(irSensor1>0.7) cif.DriveMotors(0.15,0.0);
        else if(irSensor2>0.7) cif.DriveMotors(0.0,0.15);
        else if(followBeacon && beacon.beaconVisible && beacon.beaconDir > 20.0) 
            cif.DriveMotors(0.0,0.1);
        else if(followBeacon && beacon.beaconVisible && beacon.beaconDir < -20.0) 
            cif.DriveMotors(0.1,0.0);
        else cif.DriveMotors(0.15,0.15);
    }

    /**
     * basic reactive decision algorithm, decides action based on current sensor values
     */
    public void decide() {
            if(cif.IsObstacleReady(0))
                    irSensor0 = cif.GetObstacleSensor(0);
            if(cif.IsObstacleReady(1))
                    irSensor1 = cif.GetObstacleSensor(1);
            if(cif.IsObstacleReady(2))
                    irSensor2 = cif.GetObstacleSensor(2);

            if(cif.IsCompassReady())
                    compass = cif.GetCompassSensor();
            if(cif.IsGroundReady())
                    ground = cif.GetGroundSensor();

            if(cif.IsBeaconReady(beaconToFollow))
                    beacon = cif.GetBeaconSensor(beaconToFollow);

            //System.out.println("Measures: ir0=" + irSensor0 + " ir1=" + irSensor1 + " ir2=" + irSensor2 + "\n");

            //System.out.println(robName + " state " + state);
            //System.err.print("LineSensor ");
            //for(int i=0;i<7;i++) System.err.print(" "+cif.GetLineSensor()[i]);
            //System.err.println();


            switch(state) {
                 case RUN:    /* Go */
                     if( cif.GetVisitingLed() ) state = State.WAIT;
                     if( ground == 0 ) {         /* Visit Target */
                         cif.SetVisitingLed(true);
                         System.out.println(robName + " visited target at " + cif.GetTime() + "\n");
                     }

                     else {
                         wander(false);
                     }
                     break;
                 case WAIT: /* set returning led and check that it is on */
                     cif.SetReturningLed(true);
                     if(cif.GetVisitingLed()) cif.SetVisitingLed(false);
                     if(cif.GetReturningLed()) state = State.RETURN;

                     cif.DriveMotors(0.0,0.0);
                     break;
                 case RETURN: /* Return to home area */
                     cif.SetVisitingLed(false);
                     cif.SetReturningLed(false);
                     wander(false);
                     break;

            }


            //for(int i=1; i<6; i++)
            //  if(cif.NewMessageFrom(i))
            //      System.out.println("Message: From " + i + " to " + robName + " : \"" + cif.GetMessageFrom(i)+ "\"");


            //cif.Say(robName);

            //if(cif.GetTime() % 2 == 0) {
            //     cif.RequestIRSensor(0);
            //     if(cif.GetTime() % 8 == 0 || state == State.RETURN )
            //         cif.RequestGroundSensor();
            //     else
            //         cif.RequestBeaconSensor(beaconToFollow);
            //}
            //else {
            //   cif.RequestIRSensor(1);
            //   cif.RequestIRSensor(2);
            //}
    }

    static void print_usage() {
             System.out.println("Usage: java jClient [--robname <robname>] [--pos <pos>] [--host <hostname>[:<port>]] [--map <map_filename>]");
    }

    public void printMap() {
           if (map==null) return;
        
           for (int r=map.labMap.length-1; r>=0 ; r--) {
               System.out.println(map.labMap[r]);
           }
    }

    private String robName;
    private double irSensor0, irSensor1, irSensor2, compass;
    private beaconMeasure beacon;
    private int    ground;
    private boolean collision;

    private State state;

    private int beaconToFollow;
};

