/*
 * apMbtOgreXmlParser.cpp
 *
 *  Created on: May 10, 2011
 *      Author: Antoine Petit
 */

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_XML2

#include "apMbtOgreXmlParser.h"

#include <libxml/xmlmemory.h>      /* Fonctions de la lib XML.                */
#include <iostream>
#include <map>

/*!
  Default constructor. 
  
*/
apMbtOgreXmlParser::apMbtOgreXmlParser()
{
  vpMbtXmlParser::init();
  init();
}

/*!
  Default destructor.
*/
apMbtOgreXmlParser::~apMbtOgreXmlParser()
{
}

/*!
  Initialise internal variables (including the map).
*/
void 
apMbtOgreXmlParser::init()
{
  nodeMap["rendering"] = rendering;
  nodeMap["edgeRend_threshold"] =edgeRend_threshold;
  nodeMap["clipDist"] =clipDist;
  nodeMap["sampleRend"] =sampleRend;
  nodeMap["scaleModel"] =scaleModel;
  nodeMap["xDir"] =xDir;
  nodeMap["yDir"] =yDir;
  nodeMap["zDir"] =zDir;
}

/*!
  Parse the file in parameters.
  This method is deprecated, use parse() instead.
  
  \paran filename : File to parse.
*/
void
apMbtOgreXmlParser::parse(const char * filename)
{
  std::string file = filename;
  vpXmlParser::parse(file);
}

/*!
  Write info to file.
  
  \waning Useless, so not yet implemented => Throw exception.
*/
void 
apMbtOgreXmlParser::writeMainClass(xmlNodePtr /*node*/)
{
  throw vpException(vpException::notImplementedError, "Not yet implemented." );
}

/*!
  Read the parameters of the class from the file given by its document pointer 
  and by its root node. 
  
  \param doc : Document to parse.
  \param node : Root node. 
*/
void
apMbtOgreXmlParser::readMainClass(xmlDocPtr doc, xmlNodePtr node)
{

	unsigned int nb=0;
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second){
        case ecm:{
          this->lecture_ecm (doc, dataNode);
          nb++;
          }break;
        case sample:{
          this->lecture_sample (doc, dataNode);
          nb++;
          }break;
        case camera:{
          this->lecture_camera (doc, dataNode);
          nb++;
          }break;
        case rendering:{
          this->lecture_rendering (doc, dataNode);
          nb++;
          }break;
        default:{
//          vpTRACE("unknown tag in lecture_sample : %d, %s", iter_data->second, (iter_data->first).c_str());
          }break;
        }
      }
    }
  }

  if(nb != 4){
		std::cout <<"ERROR in 'ECM' field:\n";
		std::cout << "it must contain 4 parameters\n";
    throw vpException(vpException::fatalError, "Bad number of data to extract ECM informations.");
	}
}




/*!
  Read rendering informations.

  \throw vpException::fatalError if there was an unexpected number of data.

  \param doc : Pointer to the document.
  \param node : Pointer to the node of the camera informations.
*/
void
apMbtOgreXmlParser::lecture_rendering (xmlDocPtr doc, xmlNodePtr node)
{
    // current data values.
	double thresh = this->rendparam.edgeR_th;
	double cDist = this->rendparam.clipDist;
	int s_Rend = this->rendparam.sampleR;
	double scaleM = this->rendparam.scaleModel;
	int x_D = this->rendparam.Normx;
	int y_D = this->rendparam.Normy;
	int z_D = this->rendparam.Normz;

	unsigned int nb=0;
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second){
        case edgeRend_threshold:{
          thresh = xmlReadDoubleChild(doc, dataNode);
          nb++;
          }break;
        case clipDist:{
          cDist = xmlReadDoubleChild(doc, dataNode);
          nb++;
          }break;
        case sampleRend:{
          s_Rend  = xmlReadIntChild(doc, dataNode);
          nb++;
          }break;
        case scaleModel:{
          scaleM  = xmlReadDoubleChild(doc, dataNode);
          nb++;
          }break;
        case xDir:{
          x_D = xmlReadIntChild(doc, dataNode);
          nb++;
          }break;
        case yDir:{
          y_D = xmlReadIntChild(doc, dataNode);
          nb++;
          }break;
        case zDir:{
          z_D = xmlReadIntChild(doc, dataNode);
          nb++;
          }break;
        default:{
//          vpTRACE("unknown tag in lecture_camera : %d, %s", iter_data->second, (iter_data->first).c_str());
          }break;
        }
      }
    }
  }

  if(nb == 7){
	  this->rendparam.edgeR_th=thresh;
	  this->rendparam.clipDist=cDist;
	  this->rendparam.sampleR=s_Rend;
	  this->rendparam.scaleModel=scaleM;
	  this->rendparam.Normx=x_D;
	  this->rendparam.Normy=y_D;
	  this->rendparam.Normz=z_D;

	  std::cout <<"**** Rendering: \n"<<nb <<std::endl;
      std::cout << "edgeRend_threshold "<< this->rendparam.edgeR_th <<std::endl;
	  std::cout << "clipDist "<< this->rendparam.clipDist <<std::endl;
	  std::cout << "sampleRend "<< this->rendparam.sampleR <<std::endl;
	  std::cout << "scaleModel "<< this->rendparam.scaleModel <<std::endl;
  }
	else{
		std::cout <<"ERROR in 'rendering' field:\n";
		std::cout << "it must contain  7 parameters\n";
    throw vpException(vpException::fatalError, "Bad number of data to extract camera informations.");
	}
}


#endif

#endif

