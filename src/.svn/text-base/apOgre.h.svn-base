/*
 * apOgre.h
 *
 *  Created on: April 10, 2011
 *      Author: Antoine Petit
 */

#ifndef APOGRE_H
#define APOGRE_H

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>

#include <visp/vpFrameGrabber.h>
#include <visp/vpRGBa.h>

#include <visp/vpImageTools.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpRxyzVector.h>

//#include <GL/glew.h>
//#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>

#include <OIS/OIS.h>
#include "apRend.h"
#include "Ogre.h"
#include "OgreFrameListener.h"
#include <SdkTrays.h>

#include "vpAROgre.h"


/**
* Type to determine if the background image will be coloured or in grey levels
*/
/*typedef enum{
	BACKGROUND_COLOR,
	BACKGROUND_GREY
}vpBackgroundType;*/


/*!
\class gcOgre : 
*/

class VISP_EXPORT apOgre : public vpAROgre
{

	public:
		// The constructor doesn't change here
		apOgre(vpFrameGrabber *grabber, vpCameraParameters *cam, apRend *rend, int width=512, int height=512, char *resourcePath = "");
		virtual ~apOgre( void);

};


#endif

