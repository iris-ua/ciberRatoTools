
all: jClient

SOURCES = jClient.java ciberIF/ciberIF.java ciberIF/beaconMeasure.java ciberIF/gpsMeasure.java


JARFILES = ciberIF/beaconMeasure.class  ciberIF/gpsMeasure.class\
	   ciberIF/Parameters.class  ciberIF/Measures.class\
	   ciberIF/ReplyHandler.class  ciberIF/SensorHandler.class  ciberIF/xmlParser.class\
           ciberIF/ciberIF.class

jClient:  
	javac $(SOURCES)
	jar -cf ciberIF.jar $(JARFILES)
	#rm -f $(JARFILES)

javadoc:
	javadoc -d doc -public $(SOURCES)

clean:
	rm -rf *.class ciberIF/*.class *.jar *~ ciberIF/*~ core* doc
