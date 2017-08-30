/*
 * apMbtOgreXmlParser.h
 *
 *  Created on: May 10, 2011
 *      Author: Antoine Petit
 */

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#ifndef apMbtOgreXmlParser_HH
#define apMbtOgreXmlParser_HH

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_XML2

#include <libxml/xmlmemory.h>      /* Fonctions de la lib XML.                */

#include <visp/vpMbtXmlParser.h>
#include "apRend.h"



/*!
  \class apMbtOgreXmlParser

  Data parser for the model based tracker usin.

 */
class VISP_EXPORT apMbtOgreXmlParser: public vpMbtXmlParser
{
protected:

  //! Rendering parameters
  apRend rendparam;

    
  typedef enum{
	rendering,
    edgeRend_threshold,
    clipDist,
    sampleRend,
    scaleModel,
    xDir,
    yDir,
    zDir
  } dataToParse;


public:

	apMbtOgreXmlParser();
	virtual ~apMbtOgreXmlParser();

	void parse(const char * filename);

	void readMainClass(xmlDocPtr doc, xmlNodePtr node);
	void writeMainClass(xmlNodePtr node);

	void lecture_rendering(xmlDocPtr doc, xmlNodePtr node);
	
	void getRendParameters(apRend& _rend) const { _rend = this->rendparam;}
	
protected:
  void init();

};

#endif

#endif

#endif /* NMBTXMLPARSER_H_ */



