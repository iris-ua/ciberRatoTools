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

#ifndef _CIBER_RATO_CRQPARAMHANDLER_
#define _CIBER_RATO_CRQPARAMHANDLER_

/*! \class CRQParamHandler
 *  \brief This class implements one simple parser for information in XML
 *  format.
 *
 *  This is one parser prepared to receive information that represents the 
 *  parameters of Viewer 
 */

#include <qxml.h>
#include "crmainparameters.h"

class QString;
class QXmlLocator;

class CRQParamHandler : public QXmlDefaultHandler
{
public:
	/*! This function is called at the begining of the XML document.
	 */
    bool startDocument();
	/*! This function is called at the end of the XML document.
	 */
    bool endDocument();
	/*! This function is called when a new tag arrives. It is inside this
	 * function that all parsing is done. According to the tag received,
	 * a new object will be created.
	 */
    bool startElement( const QString&, const QString&, const QString& , 
					const QXmlAttributes& );
	/*! For internal use only.
	 */
    void setDocumentLocator(QXmlLocator *);
	/*! This function returns one mainparameters object.
	 */
	CRMainParameters * parameters(); // Returns the reply structure

private:

	CRMainParameters *paramObject; 
};

#endif 
