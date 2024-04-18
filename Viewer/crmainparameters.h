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

#ifndef _CIBER_RATO_CRMAINPARAMETERS_
#define _CIBER_RATO_CRMAINPARAMETERS_

/*!
 * \class CRMainParameters
 * \brief Class to store viewer parameters.
 *
 * This class contains the information gathered from the command line
 * or from the configuration file when the application starts.
 * Attributes like walls color, background image, server address and port
 * are stored inside.
 */

#include <qstring.h>

class CRMainParameters
{
public:
	/*! This is the contructor.
	 */
	CRMainParameters();
	/*! This is the destructor.
	 */
	~CRMainParameters();

	/*! Lower walls color. A lower wall is a wall whose height is inferior
	 * to the beacon height.
	 */
	QString lowerColor;
	/*! Higher walls color. A higher wall is a wall whose height is higher
	 * than the beacon height.
	 * \sa <a href="../Qt_html/qstring.html>QString</a>
	 */
	QString higherColor;
	/*! This is the filename for the background image.
	 * \sa <a href="../Qt_html/qstring.html>QString</a>
	 */
	QString image;
	/*! This is the server port. The remote simulator port.
	 * \sa <a href="../Qt_html/qstring.html>QString</a>
	 */
	unsigned short port;
	/*! This is the server name or IP address.
	 * \sa <a href="../Qt_html/qstring.html>QString</a>
	 */
	QString serverAddr;

	char autoStart;

	char autoConnect;

	char control;

	char resetOnConnectAction;
};
#endif

 
