/****************************************************************************
*
* $Id: vpRAOgre.h 2455 2010-06-14 $
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
* Augmented Reality viewer using Ogre3D
*
* Author:
* Antoine Petit, Bertrand Delabarre
*
*****************************************************************************/

#ifndef __VP_AROGRE__
#define __VP_AROGRE__

#include <visp/vpConfig.h>

#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>

#include <visp/vpFrameGrabber.h>
#include <visp/vpRGBa.h>

#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpTime.h>




//#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>

#include <visp/vpImageTools.h>
#include <visp/vpImageIo.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpRxyzVector.h>

#include <OIS/OIS.h>
#include "Ogre.h"
#include "apRend.h"
#include "apEdgeMap.h"
#include "OgreFrameListener.h"
#include <SdkTrays.h>
#include "OgreGpuProgramManager.h"
#include "OgreHighLevelGpuProgramManager.h"


/**
* Type to determine if the background image will be coloured or in grey levels
*/
typedef enum{
	BACKGROUND_COLOR,
	BACKGROUND_GREY
}vpBackgroundType;

/*!
\class vpAROgre : This class provides methods to show a 3D scene in a real world. To do that you will need to initialise it 
with the parameters of your camera, then each frame you will need to compute a pose for your camera and give it to the application.
With that information and the image to be shown in background it will set up the 3D scene correspondingly.
*/

class VISP_EXPORT vpAROgre : public Ogre::FrameListener, public Ogre::WindowEventListener, public OIS::KeyListener, OgreBites::SdkTrayListener
{

public:
typedef enum
{
DEPTH,
DEPTHTEXTURE,
DEPTHCOLOUR
} EdgeType;

public:
	
	vpAROgre(vpFrameGrabber *grab, vpCameraParameters *cameraP, vpBackgroundType type = BACKGROUND_GREY, int width=512, int height=512, char *resourcePath = "");
	
	virtual void init(bool bufferedKeys = false);

	virtual ~vpAROgre( void);

	virtual bool frameStarted(const Ogre::FrameEvent& evt);

	virtual bool frameEnded( const Ogre::FrameEvent& evt);

	/**
	* Default event handler
	*/
	virtual bool keyPressed( const OIS::KeyEvent &e) { return true; }
	/**
	* Default event handler
	*/
	virtual bool keyReleased( const OIS::KeyEvent &e) {  return true; }
	
	virtual void windowClosed(Ogre::RenderWindow* rw);

	virtual void display(vpImage<unsigned char> &srcI, vpHomogeneousMatrix *CMo);

	virtual void display(vpImage<vpRGBa> &srcI, vpHomogeneousMatrix* CMo);

	virtual bool continueRendering(void);

	void setRend(apRend *rend);
	void setCameraParameters( vpCameraParameters *cameraP );
	void deleteE(std::string name);
	void load(std::string name, std::string model);
	void setLights();
	void setLightA();
	void setEdgeThreshold();
	void setResolution();
	void setRendering(std::string name, const EdgeType &edges, int width, int height);
	void setMat(std::string name, std::string material);
	void removeMat(std::string material);
	void setRTT(std::string compositor);
	void setRTTex(std::string compositor);
	void setMultiRTT(std::string compositor0, std::string compositor1);
	void setMRTTCol(std::string compositor0, std::string compositor1);
	void setRTTHyb(std::string compositor0, std::string compositor1);

	void setPosition(std::string name, vpTranslationVector position);

	vpTranslationVector getPosition(std::string name);

	void setRotation(std::string name, vpRotationMatrix rotation);

	void addRotation(std::string name, vpRotationMatrix rotation);

	void setPosition(std::string name, vpHomogeneousMatrix position);

	void setVisibility(std::string name, bool isVisible);

	void setScale(std::string name, float factorx, float factory, float factorz);

	void updateClipDistances0(const double &Zd, const double &m);

	void updateClipDistances(const double &Zd);
	virtual void cast(vpImage<vpRGBa> &Inormd,int iinf, int isup, int jinf, int jsup);
	virtual void rtt0(vpImage<unsigned char> &I, vpHomogeneousMatrix *CMo);
	virtual void rtt(vpImage<unsigned char> &I, vpHomogeneousMatrix *CMo);
	virtual void setShaders(std::string name);
	virtual void setShadersT(std::string name);
	virtual void setShadersCol(std::string name);
	virtual void updateRendering(vpImage<vpRGBa> &Inormd, vpImage<unsigned char> &Ior, vpImage<unsigned char> &Itex,vpHomogeneousMatrix *cMo);
	virtual void updateRTT(vpImage<vpRGBa> &I0, vpImage<unsigned char> &I1, vpHomogeneousMatrix *CMo);
	virtual void updateMultiRTT(vpImage<vpRGBa> &I1, vpImage<unsigned char> &I0, vpHomogeneousMatrix *CMo);
	virtual void updateRTTex(vpImage<unsigned char> &I1, vpImage<vpRGBa> &I2, vpImage<unsigned char> &I0, vpHomogeneousMatrix *CMo);
	virtual void updateRTTV(vpImage<unsigned char> &I0, vpHomogeneousMatrix *CMo);
	virtual void updateRTTGrad(vpImage<unsigned char> &I0, vpHomogeneousMatrix *CMo);
	virtual void writeFile(const std::string path);
	virtual void copyToImage(vpImage<unsigned char> &I);

protected:

	virtual void createCamera( void);

	virtual void createScene(void);

	/**
	* Update the 3D scene
	* \param evt : Event to process
	* \return True if everything went well
	*/
	virtual bool updateScene( const Ogre::FrameEvent& evt) {return true;};

	//virtual void ShaderDraw(void);

	//virtual void setupShaders(char *frag, GLuint p);

	/**
	* Check for keyboard, mouse and joystick inputs
	* \param evt : Event to process
	* \return True if everything went well
	*/
	virtual bool processInputEvent(const Ogre::FrameEvent& evt) {return true;};

	/**
	* Clean up the 3D scene
	* \param evt : Event to process
	* \return True if everything went well
	*/
	virtual bool destroyScene(void) {return true;};




private:

	virtual void createBackground(void);

	virtual void closeOIS(void);

	virtual void updateCameraProjection(void);

	virtual void updateBackgroundTexture(vpImage<unsigned char> &I);

	virtual void updateBackgroundTexture(vpImage<vpRGBa> &I);

	virtual void updateCameraParameters (vpHomogeneousMatrix cMo);

protected:
	// Attributes

	// OGRE 3D System
	Ogre::Root*	    mRoot;                     /** Application's root */
	Ogre::Camera*	    mCamera;                   /** Camera */
	Ogre::SceneManager* mSceneMgr;                 /** Scene manager */
	Ogre::RenderWindow* mWindow;                   /** Display window */
	Ogre::String	    mResourcePath;             /** Path to resources.cfg */

	Ogre::MultiRenderTarget* mrttex;

	// OIS Input manager and devices
	OIS::InputManager* mInputManager;
	OIS::Keyboard*	   mKeyboard;

	// ViSP AR System
	vpBackgroundType BackgroundT;                  /** Type of background (RGBa or grey level) */
	bool keepOn;                                   /** Has the application recieved a signal to stop(false) or not (true) */
	vpImage<vpRGBa> mImageRGBA;                    /** vpImage to store grabbed image */
	vpImage<unsigned char> mImage;                 /** vpImage to store grabbed image */
	vpFrameGrabber *mGrabber;                      /** Frame grabber from camera */
	Ogre::HardwarePixelBufferSharedPtr mPointer;   /** Pointer to the texture we want to update */
	Ogre::Rectangle2D* mBackground;                /** Background image */
	int mHeight;                                   /** Height of the acquired image */
	int mWidth;                                    /** Width of the acquired image */

	// Camera calculations
	vpCameraParameters *mcam;                       /** The intrinsic camera parameters */

	apRend *mrend;                                  /** Rendering Parameters  */

	EdgeType edgeT;

};

#endif
