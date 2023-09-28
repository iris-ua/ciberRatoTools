/*
    This file is part of ciberRatoToolsSrc.

    Copyright (C) 2001-2021 Universidade de Aveiro

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

#include <qapplication.h>
#include <QLocale>
#include <qstring.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include "crqscene.h"
#include "crqlabview.h"
#include "crmainparameters.h"
#include "crqparamhandler.h"
#include "./Lab/crlab.h"
#include "./Comm/crqcomm.h"

using namespace std;

void ParameterError( char *appName )
{
    cerr << "\n\tInvalid parameters\n";
	cerr << "\tTry `" << appName << " -help' for more information.\n\n";
    exit(1);
}

int main( int argc, char **argv )
{
	QApplication Visualizador( argc, argv );

	char *paramFile = NULL;
	
	CRMainParameters *param = new CRMainParameters(); // Parametros do viewer

    int i=1;
    while( i < argc ) // opcao mais valor
    {
        if( strcmp(argv[i], "--host") == 0 )
        {
			 if(argc<i+2 || argv[i+1][0]=='-')
				  ParameterError( argv[0] ); 
			 char svrName[100];        
			 sscanf( (argv[i+1]), "%79[^:]:%hd",svrName ,
					 &(param->port) ); //Read the host 	
			 param->serverAddr = svrName;
             i+=2;
        }
        else if( strcmp(argv[i], "--lowercolor") == 0 )
        {
            if(argc<i+2 || argv[i+1][0]=='-')
                ParameterError( argv[0] );
            param->lowerColor = argv[i+1];
            i+=2;
        }
        else if( strcmp(argv[i], "--highercolor") == 0 )
        {
			if(argc<i+2 || argv[i+1][0]=='-')
				  ParameterError( argv[0] ); 
			param->higherColor = argv[i+1];
            i+=2;
        }
        else if( strcmp(argv[i], "--nocontrol") == 0 )
        {
			i+=1;
            param->control = 'n';
        }
        else if( strcmp(argv[i], "--autostart") == 0 )
        {
			i+=1;
            param->autoStart = 'y';
        }
        else if( strcmp(argv[i], "--autoconnect") == 0 )
        {
			i+=1;
            param->autoConnect = 'y';
        }
        else if( strcmp(argv[i], "--paramfile") == 0 )
        {
			if(argc<i+2 || argv[i+1][0]=='-')
				  ParameterError( argv[0] ); 
			paramFile = argv[i+1];
            i+=2;
        }
        else if( strcmp(argv[i], "--help") == 0)
        {
			cout << "viewer-1.1.0       " << "[--host simulatorAddress:port]\n";
			cout << "                   [--lowercolor color]\n";
			cout << "                   [--highercolor color]\n";
			cout << "                   [--paramfile file.xml]\n";
			cout << "                   [--nocontrol]\n";
			cout << "                   [--autoconnect]\n";
			cout << "                   [--autostart]\n";
			cout << "                   [--help]\n";
            exit(0);
        }
        else ParameterError( argv[0] );
    }

	cout << "\n\tUniversidade de Aveiro\n";
	cout << "\tDepartamento de Electronica e Telecomunicacoes\n";
    cout << "\tExplorer 2023\n";
    cout << "\n\tViewer v2.0\n\n";
    cout << "\tAntonio Neves \t\tan@ieeta.pt\n";
    cout << "\tJoao Figueiredo \tjoao.figueiredo@hotpop.com\n";
    cout << "\tMiguel Rodrigues \tmiguel.rodrigues@ua.pt\n\n";

// *******************************

    if ( paramFile != NULL )
    {
		QFile parFile(paramFile);
        QXmlInputSource *source= new QXmlInputSource( &parFile );
        if( source == 0)
        {
			cout << "Failure when sourcing parameters file\n";
            return -1;
        }

		CRQParamHandler *paramHandler = new CRQParamHandler();
		
		QXmlSimpleReader xmlParser;
		xmlParser.setContentHandler( paramHandler );
		xmlParser.parse( *source );
        param = paramHandler->parameters();
    }
// *************************************

    //printf( "Creating MainWindow...");
    //fflush( stdout );
	
    CRQLabView *labGUI = new CRQLabView(param);  //Built view
    labGUI->setWindowTitle("Explorer 2023");
    labGUI->show();

    Visualizador.connect( &Visualizador, SIGNAL( lastWindowClosed() ),
                          &Visualizador, SLOT( quit() ) );

    setlocale(LC_ALL,"C");
    QLocale::setDefault(QLocale::c());

	//cout << "done.\nRunning...\n";

	return Visualizador.exec();

}

