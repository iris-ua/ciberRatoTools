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
#include "cbviewhandler.h"

#include <iostream>
#include <qstring.h>

using std::cerr;

cbViewHandler::cbViewHandler()
{
}
 
bool cbViewHandler::startDocument()
{
    return true;
}

bool cbViewHandler::endDocument()
{
	return true;
}

bool cbViewHandler::startElement( const QString&, const QString&, const QString& qName, const QXmlAttributes& attr)
{
	//cout << "cbViewHandler::startElement:: " << qName << endl;
	/* process begin tag */
	const QString &tag = qName;
	if (tag == "Start")
	{
		command.type = cbCommand::START;
	}
	else if (tag == "Stop")
	{
		command.type = cbCommand::STOP;
	}
	else if (tag == "Robot")
	{
		/* process attributes */
		const QString &removed = attr.value(QString("Removed"));
		if (!removed.isNull() && removed == "Yes")
		{
			command.type = cbCommand::ROBOTDEL;
			command.robot.id = 0;
		}
		const QString &id = attr.value(QString("Id"));
		if (!id.isNull()) command.robot.id = id.toInt();
	}
	else if (tag == "LabReq")
	{
		command.type = cbCommand::LABRQ;
	}
	else if (tag == "GridReq")
	{
		command.type = cbCommand::GRIDRQ;
	}
	else if (tag == "Reset")
	{
		command.type = cbCommand::RESET;
	}
	else
	{
		command.type = cbCommand::UNKNOWN;
		return false;
	}
    return true;
}

bool cbViewHandler::endElement( const QString&, const QString&, const QString& qName)
{
	//cout << "cbViewHandler::endElement:: " << qName << endl;
	/* process end tag */
	const QString &tag = qName;
	if (tag == "Start")
	{
		if (command.type != cbCommand::START)
		{
			cerr << "Missmatched end Start tag\n";
			return false;
		}
	}
	else if (tag == "Stop")
	{
		if (command.type != cbCommand::STOP)
		{
			cerr << "Missmatched end Stop tag\n";
			return false;
		}
	}
	else if (tag == "LabReq")
	{
		if (command.type != cbCommand::LABRQ)
		{
			cerr << "Missmatched end LabReq tag\n";
			return false;
		}
	}
	else if (tag == "GridReq")
	{
		if (command.type != cbCommand::GRIDRQ)
		{
			cerr << "Missmatched end GridReq tag\n";
			return false;
		}
	}
	else if (tag == "Robot")
	{
		if (command.type != cbCommand::ROBOTDEL)
		{
			cerr << "Missmatched end RobotRemove tag\n";
			return false;
		}
	}
	else if (tag == "Reset")
	{
		if (command.type != cbCommand::RESET)
		{
			cerr << "Missmatched end Reset tag\n";
			return false;
		}
	}
	else
	{
		cerr << "Unknown tag\n";
		return false;
	}
    return true;
}

void cbViewHandler::setDocumentLocator(QXmlLocator *)
{
	return ;
}

cbCommand &cbViewHandler::Command()
{
	return command;
}

