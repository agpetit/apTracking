/****************************************************************************
*
* $Id: gcOgre.h 2455 2010-06-14 $
*
* Copyright (C) 1998-2010 Inria. All rights reserved.
*
* This software was developed at:
* IRISA/INRIA Rennes
* Projet Lagadic
* Campus Universitaire de Beaulieu
* 35042 Rennes Cedex
* http://www.irisa.fr/lagadic
*
* This file is part of the ViSP toolkit.
*
* This file may be distributed under the terms of the Q Public License
* as defined by Trolltech AS of Norway and appearing in the file
* LICENSE included in the packaging of this file.
*
* Licensees holding valid ViSP Professional Edition licenses may
* use this file in accordance with the ViSP Commercial License
* Agreement provided with the Software.
*
* This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
* WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
*
* Contact visp@irisa.fr if any conditions of this licensing are
* not clear to you.
*
* Description:
* Derivation of Augmented Reality viewer using Ogre3D
*
* Author:
* Celine Teuliere
*
*****************************************************************************/

#ifndef __CT_OGRE__
#define __CT_OGRE__

#include <visp/vpConfig.h>

#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>

#include <visp/vpFrameGrabber.h>
#include <visp/vpRGBa.h>

#include <visp/vpImageTools.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpRxyzVector.h>

#include <GL/glew.h>
//#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>

#include <OIS/OIS.h>
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

class VISP_EXPORT ctOgre : public vpAROgre
{
public:
	
	ctOgre(const vpCameraParameters &cameraP, unsigned int width = 640,unsigned int height = 480,const char *resourcePath = ".", const char *pluginPath = ".");
	
	virtual ~ctOgre( void);

	void init(bool bufferedKeys = false);
	void createDepthRenderTexture();
	void setShadersMultiRTT(std::string name);
	
	void updateCameraProjection(void);
	void load(std::string name, std::string model);
	
	//

	void getCurrentImages(vpImage<vpRGBa> &I0, vpImage<unsigned char> &I1, vpHomogeneousMatrix&CMo);
	void testGetDepthMap(vpImage<unsigned char> &I, vpHomogeneousMatrix &CMo);
	//void testGetRenderTexture(vpImage<vpRGBa> &I, vpHomogeneousMatrix &CMo);

	//
	void displayLoc(vpImage<unsigned char> &srcI, vpHomogeneousMatrix& CMo);
	void displayLoc(vpImage<vpRGBa> &srcI, vpHomogeneousMatrix& CMo);

	static void verticalFlip(vpImage<unsigned char> & I);
	static void verticalFlip(vpImage<vpRGBa> & I);
	static void verticalFlip(vpImage<float> & I);
	void setClipDistances(float _zNear, float _zFar);
	void setComputeZ(){compute_Z = true;};
    bool processInputEvent(const Ogre::FrameEvent& /*evt*/);
	void setBackgroundType(vpBackgroundType type){BackgroundT = type;};

	void getPoseTransf (vpHomogeneousMatrix& M) const {M = poseTransf;}

	//bool frameEnded( const Ogre::FrameEvent& evt);
	
protected:
	float Z_max;
	vpImage<float> IZ;
	float Z_near;
	float Z_far;
	
	vpBackgroundType BackgroundT;                  /** Type of background (RGBa or grey level) */

private:
	vpHomogeneousMatrix poseTransf;

	bool compute_Z;
  
	// Render Texture
	Ogre::TexturePtr rtt_texture, depthTexture;
	Ogre::RenderTexture *renderTexture, *renderTarget;
};

#endif

