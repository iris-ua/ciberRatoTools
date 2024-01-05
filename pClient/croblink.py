
import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 6000

NUM_IR_SENSORS = 4
NUM_LINE_ELEMENTS = 7

class CRobLink:


    def __init__ (self, robName, robId, host, port=UDP_PORT):
        self.robName = robName
        self.robId = robId
        self.host = host
        self.sim_port = port

        self.sock = socket.socket(socket.AF_INET, # Internet
                             socket.SOCK_DGRAM) # UDP

        self.sock.settimeout(2.0)
        
        msg = '<Robot Id="'+str(robId)+'" Name="'+robName+'" />'
        
        self.sock.sendto(msg.encode(), (host, self.sim_port)) 
        data, (host,self.port) = self.sock.recvfrom(1024)
        # print("received message:", data, " port ", self.port)

        parser = sax.make_parser()
        
        # Tell it what handler to use
        handler = StructureHandler()
        parser.setContentHandler( handler )
        
        # Parse reply 

        d2 = data[:-1]
#        try:
        sax.parseString( d2, handler )
#        except SAXParseException:
#            self.status = -1
#            return 
        self.status = handler.status
        if self.status==0:
            self.nBeacons = handler.nBeacons
            self.simTime  = handler.simTime
            #print("nBeacons", self.nBeacons)

    def readSensors(self):
        try:
            data, (host,port) = self.sock.recvfrom(4096)
        except socket.timeout:
            exit(1)
        d2 = data[:-1]

        #print("RECV : \"" + d2.decode() +'"')
        parser = sax.make_parser()
        # Tell it what handler to use
        handler = StructureHandler()
        parser.setContentHandler( handler )

#        try:
        sax.parseString( d2, handler )
#        except SAXParseException:
#            status = -1
#            return 
        self.status    = handler.status
        self.measures  = handler.measures
        
    def driveMotors(self, lPow, rPow):
        msg = '<Actions LeftMotor="'+str(lPow)+'" RightMotor="'+str(rPow)+'"/>'
        self.sock.sendto(msg.encode(),(self.host,self.port))

    def setReturningLed(self,val):
        msg = '<Actions LeftMotor="0.0" RightMotor="0.0" ReturningLed="'+ ("On" if val else "Off") +'"/>'
        self.sock.sendto(msg.encode(),(self.host,self.port))

    def setVisitingLed(self,val):
        msg = '<Actions LeftMotor="0.0" RightMotor="0.0" VisitingLed="'+ ("On" if val else "Off") +'"/>'
        self.sock.sendto(msg.encode(),(self.host,self.port))

    def finish(self):
        msg = '<Actions LeftMotor="0.0" RightMotor="0.0" EndLed="On"/>'
        self.sock.sendto(msg.encode(),(self.host,self.port))

    def reset(self):
        msg = '<Actions Reset="On"/>'
        self.sock.sendto(msg.encode(),(self.host,self.port))

    #my_status = lambda self : self.status


class CRobLinkAngs(CRobLink):


    def __init__ (self, robName, robId, angs, host, port=UDP_PORT):
        self.robName = robName
        self.robId = robId
        self.angs = angs
        self.host = host
        self.sim_port = port


        self.sock = socket.socket(socket.AF_INET, # Internet
                             socket.SOCK_DGRAM) # UDP
        
        self.sock.settimeout(2.0)

        msg = '<Robot Id="'+str(robId)+'" Name="'+robName+'">'
        for ir in range(NUM_IR_SENSORS):
            msg+='<IRSensor Id="'+str(ir)+'" Angle="'+str(angs[ir])+'" />'
        msg+='</Robot>'

        #print "msg ", msg
        
        self.sock.sendto(msg.encode(), (host, self.sim_port))  
        data, (host,self.port) = self.sock.recvfrom(1024)
        #print("received message:", data, " port ", self.port)

        parser = sax.make_parser()
        
        # Tell it what handler to use
        handler = StructureHandler()
        parser.setContentHandler( handler )
        
        # Parse reply 

        d2 = data[:-1]
#        try:
        sax.parseString( d2, handler )
#        except SAXParseException:
#            self.status = -1
#            return 
        self.status = handler.status
        if self.status==0:
            self.nBeacons = handler.nBeacons
            self.simTime  = handler.simTime
            #print("nBeacons", self.nBeacons)
            #print("simTime", self.simTime)

class CMeasures:

    def __init__ (self):
        self.compassReady=False
        self.compass=0.0; 

        self.irSensorReady=[False for i in range(NUM_IR_SENSORS)]
        self.irSensor=[0.0 for i in range(NUM_IR_SENSORS)]

        self.beaconReady = [] # False   # TODO consider more than one beacon
        self.beacon =      [] # (False, 0.0)

        self.time = 0

        self.groundReady = False
        self.ground = -1

        self.collisionReady = False
        self.collision = False 

        self.lineSensorReady = False
        self.lineSensor=["0" for i in range(NUM_LINE_ELEMENTS)]

        self.start = False 
        self.stop = False 

        self.endLed = False
        self.returningLed = False
        self.visitingLed = False

        self.x = 0.0   
        self.y = 0.0   
        self.dir = 0.0

        self.scoreReady = False
        self.score = 100000

        self.arrivalTimeReady = False
        self.arrivalTime = 10000
        
        self.returningTimeReady = False
        self.returningTime = 10000
        
        self.collisionsReady = False
        self.collisions = 0
        
        self.gpsReady = False
        self.gpsDirReady = False

        self.hearMessage=''



from xml import sax

class StructureHandler(sax.ContentHandler):

    def __init__ (self):
        self.status = 0
        self.measures = CMeasures()

    def startElement(self, name, attrs):
        if name == "Reply":
            if "Status" not in attrs.keys():
                self.status = -1
                return
                
            if attrs["Status"]=="Ok":
                self.status = 0
                return
            self.status = -1
        elif name == "Parameters":
            self.nBeacons = int(attrs["NBeacons"])
            self.simTime  = int(attrs["SimTime"])
        elif name=="Measures":
            self.measures.time = int(attrs["Time"])
        elif name=="Sensors":
            self.measures.compassReady         = ("Compass" in attrs.keys())
            if self.measures.compassReady:
                self.measures.compass =   float(attrs["Compass"])

            self.measures.collisionReady         = ("Collision" in attrs.keys())
            if self.measures.collisionReady:
                self.measures.collision = (attrs["Collision"] == "Yes")

            self.measures.groundReady         = ("Ground" in attrs.keys())
            if self.measures.groundReady:
                self.measures.ground =    int(attrs["Ground"])

        elif name == "IRSensor":
            id = int(attrs["Id"])
            #print "IRSensor id ", id
            if id < NUM_IR_SENSORS: 
                self.measures.irSensorReady[id] = True
                #print "IRSensor val ", attrs["Value"]
                self.measures.irSensor[id] = float(attrs["Value"])
            else: 
                self.status = -1
        elif name == "BeaconSensor":
            id = int(attrs["Id"])
            #if id<self.measures.beaconReady.len():
            if id==len(self.measures.beacon):   # only works if BeaconSensor is not requestable  TODO: make it work for requestable beaconSensors!
                self.measures.beaconReady.append(True)
                if attrs["Value"] == "NotVisible":
                    #self.measures.beaconReady[id]=(False,0.0)
                    self.measures.beacon.append((False,0.0))
                else:
                    #self.measures.beaconReady[id]=(True,attrs["Value"])
                    self.measures.beacon.append( (True,float(attrs["Value"])) )
            else:
                self.status = -1
                print('BeaconSensor cannot be correctly parsed', id, len(self.measures.beacon))
                quit()
        elif name == "GPS":
            if "X" in attrs.keys():
                self.measures.gpsReady = True
                self.measures.x = float(attrs["X"])
                self.measures.y = float(attrs["Y"])
                if "Dir" in attrs.keys():
                     self.measures.gpsDirReady = True
                     self.measures.dir = float(attrs["Dir"])
                else:
                     self.measures.gpsDirReady = False
            else:
                self.measures.gpsReady = False
        elif name == "LineSensor":
            self.measures.lineSensorReady = True
            self.measures.lineSensor = list(attrs["Value"])
        elif name == "Leds":
            self.measures.endLed = (attrs["EndLed"] == "On")
            self.measures.returningLed = (attrs["ReturningLed"] == "On")
            self.measures.visitingLed = (attrs["VisitingLed"] == "On")
        elif name == "Buttons":
            self.measures.start = (attrs["Start"] == "On")
            self.measures.stop = (attrs["Stop"] == "On")
        elif name == "Score":
            self.measures.scoreReady         = ("Score" in attrs.keys())
            if self.measures.scoreReady:
                 self.measures.score=int(attrs["Score"])
            self.measures.arrivalTimeReady         = ("ArrivalTime" in attrs.keys())
            if self.measures.arrivalTimeReady:
                 self.measures.arrivalTime=int(attrs["ArrivalTime"])
            self.measures.returningTimeReady         = ("ReturningTime" in attrs.keys())
            if self.measures.returningTimeReady:
                 self.measures.returningTime=int(attrs["ReturningTime"])
            self.measures.collisionsReady         = ("Collisions" in attrs.keys())
            if self.measures.collisionsReady:
                 self.measures.collisions=int(attrs["Collisions"])
        elif name == "Message":
            self.hearFrom = int(attrs["From"])


#    def endElement(self, name):
        #print 'End of element:', name

