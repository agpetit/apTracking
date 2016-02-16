/****************************************************************************
*
* $Id: vpRAOgre.cpp 2455 2010-06-14 $
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
* Bertrand Delabarre
*
*****************************************************************************/

#include <GL/glew.h>
#include "vpAROgre.h"



#define __glewGenFramebuffersEXT
#define __glewBindFramebufferEXT
#define __glewGenRenderbuffersEXT
#define __glewBindRenderbufferEXT
#define __glewRenderbufferStorageEXT
#define	__glewFramebufferRenderbufferEXT
#define __glewFramebufferTexture2DEXT

#define __glewGenBuffersARB
#define __glewBindBufferARB
#define __glewBufferDataARB
#define __glewMapBufferARB
#define __glewMapBufferARB

#define BUFFER_OFFSET(i) ((char *)NULL + (i))




/**
* Constructor
* \param grab : Framegrabber to get background images
* \param type : Either BACKGROUND_GREY for grey level image in background or BACKGROUND_COLOR for a RGBa one
* \param width : Width of the grabbed image
* \param height : Height of the grabbed image
* \param ressourcepath : Path to the resources.cfg file telling Ogre where to look for resources
*/
vpAROgre::vpAROgre(vpFrameGrabber *grab, vpCameraParameters *cameraP, vpBackgroundType type, int width, int height, char *resourcePath)
	: mRoot(0), mCamera(0), mSceneMgr(0), mWindow(0), mInputManager(0), mKeyboard(0)
{
	// Get resources.cfg path
	mResourcePath = resourcePath;
	// Initialise frame grabber
	mGrabber = grab;
	// Set intrinsic camera parameters
	mcam = cameraP;
	// Is the background colored or not
	BackgroundT = type;
	// When created no reason to stop displaying
	keepOn = true;
	// Set Dimensions
	mWidth =width;
	mHeight = height;
	//Set Rend Parameters
	//mrend = rend;
}

/**
* Initialisation of Ogre
* \param bufferedKeys : If true, use of buffered input for the keybord (see Ogre documentation)
*/
void vpAROgre::init(bool bufferedKeys)
{
	// Create the root
	mRoot = new Ogre::Root(mResourcePath+"plugins.cfg", mResourcePath+"ogre.cfg", "Ogre.log");
	// Load resource paths from config file

	// File format is:
	//  [ResourceGroupName]
	//  ArchiveType=Path
	//  .. repeat
	// For example:
	//  [General]
	//  FileSystem=media/
	//  Zip=packages/level1.zip
	Ogre::ConfigFile cf;
	cf.load("resources.cfg");

	// Go through all sections & settings in the file
	Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();

	Ogre::String secName, typeName, archName;
	while (seci.hasMoreElements())
	{
		secName = seci.peekNextKey();
		Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
		Ogre::ConfigFile::SettingsMultiMap::iterator i;
		for (i = settings->begin(); i != settings->end(); ++i)
		{
			typeName = i->first;
			archName = i->second;
			Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
					archName, typeName, secName);
		}
	}
	// Create the window

	// If we don't find a previous configuration we ask for a new one
	// To create a new one just delete your ogre.cfg file previously created
	if(!mRoot->restoreConfig())
	{
		if(!mRoot->showConfigDialog())
		throw "ConfigDialog arbored"; // Exit the application on cancel
	}

	mWindow = mRoot->initialise(true, "Visp - Augmented Reality");

	//Ogre::RenderWindow* mWindow1 = mRoot->initialise(true, "Z-buffer");

	// Initialise resources
	Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

	//-----------------------------------------------------
	// 4 Create the SceneManager
	//
	//		ST_GENERIC = octree
	//		ST_EXTERIOR_CLOSE = simple terrain
	//		ST_EXTERIOR_FAR = nature terrain (depreciated)
	//		ST_EXTERIOR_REAL_FAR = paging landscape
	//		ST_INTERIOR = Quake3 BSP
	//-----------------------------------------------------
	mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);

	// Create the camera
	createCamera();

	// Create a viewport
	Ogre::Viewport* viewPort = mWindow->addViewport(mCamera);
			viewPort->setClearEveryFrame(true);


	// Set the projection parameters to match the camera intrinsic parameters
	updateCameraProjection();

	// Create the 3D scene
	//createScene();

	// Initialise and register event handlers
	mRoot->addFrameListener( this);

	// Register as a Window listener
	Ogre::WindowEventUtilities::addWindowEventListener(mWindow, this);

	//Ogre::WindowEventUtilities::addWindowEventListener(mWindow1, this);

	// Initialise
	Ogre::LogManager::getSingletonPtr()->logMessage("*** Initializing OIS ***");
	OIS::ParamList pl;
	size_t windowHnd = 0;
	std::ostringstream windowHndStr;

	// Initialise window
	mWindow->getCustomAttribute("WINDOW", &windowHnd);
	windowHndStr << windowHnd;
	pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));

	//mInputManager = OIS::InputManager::createInputSystem( pl );


	// Initialise window
		/*mWindow1->getCustomAttribute("WINDOW1", &windowHnd);
		windowHndStr << windowHnd;
		pl.insert(std::make_pair(std::string("WINDOW1"), windowHndStr.str()));*/

		mInputManager = OIS::InputManager::createInputSystem( pl );


	//Create all devices
	mKeyboard = static_cast<OIS::Keyboard*>(
			mInputManager->createInputObject( OIS::OISKeyboard, bufferedKeys ));
	if ( !bufferedKeys ) mKeyboard->setEventCallback ( this);

	// Here we only consider the keybord input

	// Create the background image which will come from the grabber
	//createBackground();




}

/**
* Destructor
*/
vpAROgre::~vpAROgre( void)
{
	// Destroy 3D scene
	destroyScene();
	// Free the video device
	mGrabber->close();
	// Close OIS
	closeOIS();
	if ( mWindow) {
		Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);
		windowClosed(mWindow);
	}
	// Delete root
	if (mRoot) delete mRoot;
}

/**
* Function telling what to do before each frame
* \param evt : Frame event to process
* \return True if everything went well
*/
// This gets called before every frame rendered by Ogre
bool vpAROgre::frameStarted(const Ogre::FrameEvent& evt)
{
	if(mWindow->isClosed())	return false;
	mKeyboard->capture();

	if(mKeyboard->isKeyDown(OIS::KC_ESCAPE))
		return false;

	if(keepOn){
		return processInputEvent( evt);
	}
	else
		return keepOn;
}


/**
* Function telling what to do after each frame
* \param evt : Frame event to process
* \return True if everything went well
*/
bool vpAROgre::frameEnded( const Ogre::FrameEvent& evt)
{
	if(keepOn){
		return updateScene(evt);
	}
	else
		return keepOn;
}


/**
* Unattach OIS before window shutdown (very important under Linux)
* \param rw : RenderWindow to close
*/
void vpAROgre::windowClosed(Ogre::RenderWindow* rw)
{
	//Only close for window that created OIS (the main window in these demos)
	if( rw == mWindow ) closeOIS();
}


/**
* Display a frame
* \param srcI : Grey level image to show in background
* \param CMo : Camera parameters
*/
void vpAROgre::display(vpImage<unsigned char> &srcI, vpHomogeneousMatrix *CMo)
{
	// Update the background to match the situation
	updateBackgroundTexture(srcI);

	// Update the camera parameters to match the grabbed image
	updateCameraParameters(*CMo);

	// Display on Ogre Window
	if(mRoot->renderOneFrame()){
		mWindow->update();
		keepOn = true;
	}
	else
		keepOn = false;
}

/**
* Display a frame
* \param srcI : RGBa image to show in background
* \param CMo : Camera parameters
*/

/*void vpAROgre::setupShaders(char *frag, GLuint p) {
GLuint v, f;
char *vs = NULL,*fs = NULL;
v = glCreateShader(GL_VERTEX_SHADER);
f = glCreateShader(GL_FRAGMENT_SHADER);
//vs = textFileRead(vert);
fs = textFileRead(frag);
const char * ff = fs;
//const char * vv = vs;
//glShaderSource(v, 1, &vv, NULL);
glShaderSource(f, 1, &ff, NULL);
//free(vs);
free(fs);
//glCompileShader(v);
glCompileShader(f);
p = glCreateProgram();
glAttachShader(p,f);
//glAttachShader(p,v);
glLinkProgram(p);
glUseProgram(p);
}*/


/*void vpAROgre::ShaderDraw(void){
glBegin(GL_QUADS);
glTexCoord2f(0.0, 0.0); glVertex3f(0.0, 0.0, 0.0);
glTexCoord2f(0.0, 1.0); glVertex3f(0.0, height, 0.0);
glTexCoord2f(1.0, 1.0); glVertex3f(width, height, 0.0);
glTexCoord2f(1.0, 0.0); glVertex3f(width, height, 0.0);
glEnd();
}*/



void vpAROgre::display(vpImage<vpRGBa> &srcI, vpHomogeneousMatrix* CMo)
{
	// Update the background to match the situation
	//updateBackgroundTexture(srcI);
	// Update the camera parameters to match the grabbed image
	updateCameraParameters(*CMo);


	// Display on Ogre Window
	if(mRoot->renderOneFrame()){
	    double t0= vpTime::measureTimeMs();

		mWindow->update();

	    double t1= vpTime::measureTimeMs();
		keepOn = true;

		std::cout<<"time1 "<<t1-t0<<std::endl;
	}
	else
		keepOn = false;



/*	GLuint fbo;
	GLuint depthbuffer;
	GLuint depthbuffer1;
	GLuint p1;

    glGenFramebuffersEXT(1, &fbo);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);

     glGenRenderbuffersEXT(1, &depthbuffer);
     glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, depthbuffer);
     glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT, width, height);
     glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0);

     // attach a texture to FBO depth attachement point
     glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, depthbuffer1, 0);

     // attach a renderbuffer to depth attachment point
     //glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, rboId);

     //@ disable color buffer if you don't attach any color buffer image,
     //@ for example, rendering depth buffer only to a texture.
     //@ Otherwise, glCheckFramebufferStatusEXT will not be complete.
     glDrawBuffer(GL_NONE);
     glReadBuffer(GL_NONE);


     glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);



     // create, init and enable the shader
     setupShaders("sobelShader.frag", p1);
     checkGLErrors ("Shaders 1");
     // attach the input texture(read texture) to the first texture unit
     glActiveTexture(GL_TEXTURE0);
     glBindTexture(GL_TEXTURE_2D,depthbuffer1);
     GLuint texLoc;
     texLoc = glGetUniformLocation(p1,"tex");
     glUniform1i(texLoc, 0);
     // draw a square with the texture on it so to perform the computation
     ShaderDraw();*/





	// frame buffer object

	/*int width = 640;
	int height = 480;

	GLuint fbo;
	glGenFramebuffersEXT(1, &fbo);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
	GLuint depthbuffer;
	GLuint depthbuffer1;
	glGenRenderbuffersEXT(1, &depthbuffer);
	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, depthbuffer);
	glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT, width, height);
	glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, depthbuffer);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, depthbuffer1, 0);
    //glDrawBuffer(GL_NONE);
    //glReadBuffer(GL_NONE)
	for (int i=0;i<10000;i++)
	{
	std::cout << " depth "<< depthbuffer1<< std::endl;
	}
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);

	/*glPushAttrib(GL_VIEWPORT_BIT);
	glViewport(0,0,width, height);*/


	// Render as normal here
	// output goes to the FBO and it’s attached buffers

	/*glPopAttrib();
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

	const int SCREEN_WIDTH = 640;
	const int SCREEN_HEIGHT = 480;
	const int CHANNEL_COUNT = 4;
	const int DATA_SIZE = SCREEN_WIDTH * SCREEN_HEIGHT * CHANNEL_COUNT;
	const GLenum PIXEL_FORMAT = GL_RGBA;
	const int PBO_COUNT = 2;
	GLubyte* colorBuffer = 0;
    colorBuffer = new GLubyte[DATA_SIZE];
    memset(colorBuffer, 255, DATA_SIZE);

	GLuint pboIds[PBO_COUNT];

    glGenBuffersARB(PBO_COUNT, pboIds);
    glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, pboIds[0]);
    glBufferDataARB(GL_PIXEL_PACK_BUFFER_ARB, DATA_SIZE, 0, GL_STREAM_READ_ARB);
    glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, pboIds[1]);
    glBufferDataARB(GL_PIXEL_PACK_BUFFER_ARB, DATA_SIZE, 0, GL_STREAM_READ_ARB);
    glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, 0);


    static int shift = 0;
    static int index = 0;
    int nextIndex = 0;                  // pbo index used for next frame

    // brightness shift amount
    shift = ++shift % 200;

    // increment current index first then get the next index
    // "index" is used to read pixels from a framebuffer to a PBO
    // "nextIndex" is used to process pixels in the other PBO
    index = (index + 1) % 2;
    nextIndex = (index + 1) % 2;

    //float pixels[height][width][4];

    // set the framebuffer to read
    glReadBuffer(GL_FRONT);

    GLvoid* buf=0;
        // read framebuffer ///////////////////////////////

        // copy pixels from framebuffer to PBO
        // Use offset instead of ponter.
        // OpenGL should perform asynch DMA transfer, so glReadPixels() will return immediately.

    //mWindow->update();

        glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, pboIds[index]);
        //glReadPixels(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, PIXEL_FORMAT, GL_FLOAT, &pixels);
		double t0= vpTime::measureTimeMs();
        glReadPixels(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, 0);
        double t1= vpTime::measureTimeMs();
        //std::cout<<(double)pixels[470][320][1]<<std::endl;
        std::cout<<t1-t0<<std::endl;

        /*vpDisplayX display;
        unsigned char* output;
        vpImage<unsigned char> I1(480,640);
        display.init(I1, 200, 200, "X11 display");
        output = (unsigned char*)I1.bitmap;*/

        /*for (unsigned int i=0;i < 480 ; i++)
        {
                for (unsigned int j=0;j < 640  ; j++)
                {
                   *(output++) = colorBuffer[((height-1-i)*width+j)];
                   *(output++) = colorBuffer[((height-1-i)*width+j)+1];
                   *(output++) = colorBuffer[((height-1-i)*width+j)+2];
                   *(output++) = colorBuffer[((height-1-i)*width+j)+3];
                }
        }*/
        /*vpDisplay::display(I1);
        vpDisplay::flush(I1);*/


        /*getchar();
        // measure the time reading framebuffer
        // process pixel data /////////////////////////////

        // map the PBO that contain framebuffer pixels before processing it

        glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, pboIds[nextIndex]);
        GLubyte* src = (GLubyte*)glMapBufferARB(GL_PIXEL_PACK_BUFFER_ARB, GL_READ_ONLY_ARB);
        std::cout<<colorBuffer[11]<<std::endl;*/

        /*if(src)
        {
            // change brightness
            add(src, SCREEN_WIDTH, SCREEN_HEIGHT, shift, colorBuffer);
            glUnmapBufferARB(GL_PIXEL_PACK_BUFFER_ARB);     // release pointer to the mapped buffer
        }*/

        // measure the time reading framebuffer

        /*glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, 0);

        // clear buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);*/




	/*int pboMemory1;
	int pboMemory2;
	GLuint imageBuffers[2];
	int imageSize = 640*480*4;
	GLubyte* colorBuffer = 0;
    colorBuffer = new GLubyte[imageSize];
    memset(colorBuffer, 255, imageSize);

	glGenBuffersARB(2, imageBuffers);

	        glBindBufferARB(GL_PIXEL_PACK_BUFFER_EXT, imageBuffers[0]);
	        glBufferDataARB(GL_PIXEL_PACK_BUFFER_EXT, imageSize/2, NULL, GL_STATIC_READ);

	        glBindBufferARB(GL_PIXEL_PACK_BUFFER_EXT, imageBuffers[1]);
	        glBufferDataARB(GL_PIXEL_PACK_BUFFER_EXT, imageSize/2, NULL, GL_STATIC_READ);

	        // render to framebuffer
	        //glDrawBuffer(GL_BACK);
	        //mWindow->update();
	        //renderScene();

	        // Bind two different buffer objects and start the ReadPixels
	        // asynchronously. Each call will return directly after starting the
	        // DMA transfer.
	        glBindBufferARB(GL_PIXEL_PACK_BUFFER_EXT, imageBuffers[0]);
	        glReadPixels(0, 0, width, height/2,
	            GL_BGRA, GL_UNSIGNED_BYTE, colorBuffer);

	        //std::cout <<colorBuffer<<std::endl;


	        glBindBufferARB(GL_PIXEL_PACK_BUFFER_EXT, imageBuffers[1]);
	        glReadPixels(0, height/2, width, height/2, GL_BGRA, GL_UNSIGNED_BYTE,
	                   colorBuffer);

	        // process partial images
	        pboMemory1 = glMapBufferARB(GL_PIXEL_PACK_BUFFER_EXT, GL_READ_ONLY);

	        //processImage(pboMemory1);
	        pboMemory2 = glMapBufferARB(GL_PIXEL_PACK_BUFFER_EXT, GL_READ_ONLY);
	        //processImage(pboMemory2);*/

}

/**
* Ask the program if we can continue to render
* \return True if nothing stopped the rendering loop
*/
bool vpAROgre::continueRendering(void)
{
	return keepOn;
}



/**
* Set rendering parameters
*/
void vpAROgre::setRend( apRend *rend )
{
	mrend = rend;
}


/**
* Set the camera intrinsic parameters
*/
void vpAROgre::setCameraParameters( vpCameraParameters *cameraP )
{	
	mcam = cameraP;
}

/**
* Function to destroy entity
*/

void vpAROgre::deleteE(std::string name)
{
	mSceneMgr->destroyEntity(name);
}

/**
* Load a mesh in the 3D world
* \param name : Name of the Entity and SceneNode to create
* \param model : 3D model to load
*/

void vpAROgre::load(std::string name, std::string model)
{
    //mSceneMgr->setAmbientLight(Ogre::ColourValue(0, 0, 0));
    mSceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE);

	Ogre::Entity *newEntity = mSceneMgr->createEntity(name, model);
    //Ogre::MaterialPtr DepthMaterial = Ogre::MaterialManager::getSingleton().load("DepthMap3", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	//newEntity->setMaterialName("DepthMap3");
    //newEntity->setMaterialName("NormalMap");
	//newEntity->setMaterialName("DepthMap30");
	//newEntity->setCastShadows(false);

	Ogre::SceneNode *newNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(name);
	newNode->attachObject(newEntity);
	double scale = mrend->scaleModel;
	setScale(name,scale,scale,scale);
}

/**
* Change position of a ScneneNode
* \param name : Name of the SceneNode to move
* \param position : New position of the node
*/
void vpAROgre::setPosition(std::string name, vpTranslationVector position)
{
	// Reset the position
	Ogre::SceneNode *node = mSceneMgr->getSceneNode(name);
	node->setPosition(position[0], position[1], position[2]);
}

/**
* Get position of a SceneNode
* \param name : Name of the SceneNode in the scene graph
* \return The position of the node
*/
vpTranslationVector vpAROgre::getPosition(std::string name)
{
	Ogre::Vector3 translation = mSceneMgr->getSceneNode(name)->getPosition();
	return vpTranslationVector(translation[0], translation[1], translation[2]);
}

/**
* Set the orientation of a SceneNode
* \param name : Name of the SceneNode to rotate
* \param rotation : The rotation matrix representing the rotation to apply
*/
void vpAROgre::setRotation(std::string name, vpRotationMatrix rotation)
{
	// Get the node in its original position
	mSceneMgr->getSceneNode(name)->resetOrientation();
	// Apply the new rotation
	vpRxyzVector VecRotation(rotation);
	mSceneMgr->getSceneNode(name)->rotate(Ogre::Vector3(1,0,0), Ogre::Radian(VecRotation[0]));
	mSceneMgr->getSceneNode(name)->rotate(Ogre::Vector3(0,1,0), Ogre::Radian(VecRotation[1]));
	mSceneMgr->getSceneNode(name)->rotate(Ogre::Vector3(0,0,1), Ogre::Radian(VecRotation[2]));
}

/**
* Add a rotation to a SceneNode
* \param name : Name of the SceneNode to rotate
* \param rotation : The rotation matrix representing the rotation to apply
*/
void vpAROgre::addRotation(std::string name, vpRotationMatrix rotation)
{
	// Apply the new rotation
	vpRxyzVector VecRotation(rotation);
	mSceneMgr->getSceneNode(name)->rotate(Ogre::Vector3(1,0,0), Ogre::Radian(VecRotation[0]));
	mSceneMgr->getSceneNode(name)->rotate(Ogre::Vector3(0,1,0), Ogre::Radian(VecRotation[1]));
	mSceneMgr->getSceneNode(name)->rotate(Ogre::Vector3(0,0,1), Ogre::Radian(VecRotation[2]));
}

/**
* Set the position and the orientation of a SceneNode
* \param name : Name of the SceneNode to rotate
* \param position : The homogeneous matrix representing the rotation and translation to apply
*/
void vpAROgre::setPosition(std::string name, vpHomogeneousMatrix position)
{
	// Extract the position and orientation data
	vpRotationMatrix rotations;
	vpTranslationVector translation;
	position.extract(rotations);
	position.extract(translation);
	// Apply them to the node
	setPosition(name, translation);
	setRotation(name, rotations);
}

/**
* Tell if a SceneNode is shown on the screen or not
* \param name : Name of the SceneNode
* \param isVisble : If true we show the node, if false we don't
*/
void vpAROgre::setVisibility(std::string name, bool isVisible)
{
	mSceneMgr->getSceneNode(name)->setVisible(isVisible);
}

/**
* Scale a SceneNode
* \param name : Name of the SceneNode
* \param factorx : Scale factor along the x-axis
* \param factory : Scale factor along the x-axis
* \param factorz : Scale factor along the x-axis
*/
void vpAROgre::setScale(std::string name, float factorx, float factory, float factorz)
{
	// Reset the scale to its original value
	mSceneMgr->getSceneNode(name)->scale(Ogre::Vector3(1,1,1)/mSceneMgr->getSceneNode(name)->getScale());
	// Apply the new scale
	mSceneMgr->getSceneNode(name)->scale(Ogre::Vector3(factorx, factory, factorz));
}

/**
* Create the Ogre camera
*/
void vpAROgre::createCamera( void)
{
	mCamera = mSceneMgr->createCamera("Camera");
}

/**
* Build the 3D scene
* Override this to show what you want
*/
void vpAROgre::createScene(void){}


/**
* Create the background to show the real scene
*/
void vpAROgre::createBackground(void)
{
	// Create a rectangle to show the incoming images from the camera
	mBackground = new Ogre::Rectangle2D(true); // true = textured
	mBackground->setCorners(-1.0, 1.0, 1.0, -1.0); // Spread all over the window
	mBackground->setBoundingBox(Ogre::AxisAlignedBox(-100000.0*Ogre::Vector3::UNIT_SCALE, 100000.0*Ogre::Vector3::UNIT_SCALE)); // To be shown everywhere

	// Texture options
	Ogre::MaterialManager::getSingleton().setDefaultTextureFiltering(Ogre::TFO_NONE);
	Ogre::MaterialManager::getSingleton().setDefaultAnisotropy(1);

	// Dynamic texture
	if(BackgroundT == BACKGROUND_COLOR){ // RGBA texture
		Ogre::TextureManager::getSingleton().createManual("BackgroundTexture",
								Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
								Ogre::TEX_TYPE_2D,
								mWidth,//width
								mHeight,//height
								0,  // num of mip maps
								Ogre::PF_BYTE_BGRA,
								Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
	}
	else{ // Grey level texture
		Ogre::TextureManager::getSingleton().createManual("BackgroundTexture",
								Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
								Ogre::TEX_TYPE_2D,
								mWidth,//width
								mHeight,//height
								0,  // num of mip maps
								Ogre::PF_BYTE_L,
								Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
	}


	// Pointer to the dynamic texture
	Ogre::TexturePtr dynTexPtr = Ogre::TextureManager::getSingleton().getByName("BackgroundTexture");

	// Memory of the texture
	mPointer = dynTexPtr->getBuffer();

	// Material to apply the texture to the background
	Ogre::MaterialPtr Backgroundmaterial = Ogre::MaterialManager::getSingleton().create("BackgroundMaterial",
										Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	Ogre::Technique *Backgroundtechnique = Backgroundmaterial->createTechnique();
	Backgroundtechnique->createPass();
	Backgroundmaterial->getTechnique(0)->getPass(0)->setLightingEnabled(false);
	Backgroundmaterial->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false); // Background
	Backgroundmaterial->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false); // Background
	Backgroundmaterial->getTechnique(0)->getPass(0)->createTextureUnitState("BackgroundTexture");
	mBackground->setMaterial("BackgroundMaterial"); // Attach the material to the rectangle
	mBackground->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND); // To be rendered in Background

	// Add the background to the Scene Graph so it will be rendered
	Ogre::SceneNode *BackgroundNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("BackgoundNode");
	BackgroundNode->attachObject(mBackground);
}

/**
* Close the input manager
*/
void vpAROgre::closeOIS(void)
{
	if( mInputManager )
	{
		mInputManager->destroyInputObject(mKeyboard);

		OIS::InputManager::destroyInputSystem(mInputManager);
		mInputManager = 0;
	}
}

/**
* Update the projection parameters of the camera
*/
void vpAROgre::updateCameraProjection(void)
{
	//mcam->init(mcam->get_px(),mcam->get_py(),mcam->get_u0(),mcam->get_v0());
	// Set the virtual camera parameters to match the real camera ones
	mCamera->setFOVy(2*Ogre::Math::ATan(mHeight/2.0/mcam->get_py()));
	mCamera->setAspectRatio(((Ogre::Real)mWidth/(Ogre::Real)mHeight));
	mCamera->setNearClipDistance(0.1);
	mCamera->setFarClipDistance(10);
}

void vpAROgre::updateClipDistances(const double &Zd)
{
	//std::cout<<"Zd = "<<Zd<<std::endl;
	double clipD = mrend->clipDist;
	mCamera->setNearClipDistance(Zd-clipD);
	mCamera->setFarClipDistance(Zd+clipD);
}

void vpAROgre::updateClipDistances0(const double &Zd, const double &m)
{
	//std::cout<<"Zd = "<<Zd<<std::endl;
	mCamera->setNearClipDistance(Zd-m);
	    mCamera->setFarClipDistance(Zd+m);
}

void vpAROgre::cast(vpImage<vpRGBa> &Inormd, int iinf, int isup, int jinf, int jsup)
{

	iinf=480;
	isup=0;
	jinf=640;
	jsup=0;
	int i,j;
	int w=Inormd.getWidth();
	int h = Inormd.getHeight();
	for (i=0 ; i < h ; i++){
	for (j=0 ; j < w ; j++){
	double val =0.2*Inormd[i][j].R + 0.3*Inormd[i][j].G + 0.5*Inormd[i][j].B ;
	if (val>0)
	{
		if (i<iinf)
		{
			if (i>29)
		{iinf=i;}
			else {iinf=30;}
		}
		if (i>isup)
		{
			if (i<451)
		{isup=i;}
			else {isup=450;}
		}
		if (j<jinf)
		{
			//if (j>109)
			if (j>29)
		{jinf=j;}
			//else {bornj1=111;}
			else {jinf=30;}
		}
		if (j>jsup)
		{
			if (j<611)
		{jsup=j;}
			else {jsup=610;}
		}
	}
	}
	}
	//std::cout << " iinf " << iinf << " isup " << isup << std::endl;
}

/**
* Update the texture we see in background with a grey level image
*/
void vpAROgre::updateBackgroundTexture(vpImage<unsigned char> &I)
{
	// Inspired from Ogre wiki : http://www.ogre3d.org/wiki/index.php/Creating_dynamic_textures
	// Lock the pixel buffer and get a pixel box
	mPointer->lock(Ogre::HardwareBuffer::HBL_DISCARD); // Lock the buffer
	const Ogre::PixelBox& pixelBox = mPointer->getCurrentLock();
	// Buffer data
	Ogre::uint8* pDest = static_cast<Ogre::uint8*>(pixelBox.data);
	// Fill in the data
	for(int i=0; i<480; i++){
		for(int j=0; j<640; j++){
			// Grey Level Image
			*pDest++=(char)I[i][j];
		}
	}
	// Unlock the pixel buffer
	mPointer->unlock();
}

/**
* Update the texture we see in background with a RGBa image
*/
void vpAROgre::updateBackgroundTexture(vpImage<vpRGBa> &I)
{
	// Inspired from Ogre wiki : http://www.ogre3d.org/wiki/index.php/Creating_dynamic_textures
	// Lock the pixel buffer and get a pixel box
	mPointer->lock(Ogre::HardwareBuffer::HBL_DISCARD); // Lock the buffer
	const Ogre::PixelBox& pixelBox = mPointer->getCurrentLock();
	// Buffer data
	Ogre::uint8* pDest = static_cast<Ogre::uint8*>(pixelBox.data);
	// Fill in the data
	for(int i=0; i<480; i++){
		for(int j=0; j<640; j++){
			// Color Image
			*pDest++=(char)I[i][j].B; // Blue component
			*pDest++=(char)I[i][j].G; // Green component
			*pDest++=(char)I[i][j].R; // Red component
			*pDest++ = 255;           // Alpha component
			}
	}
	// Unlock the pixel buffer
	mPointer->unlock();
}

/**
* Update Camera parameters from a pose calculation
*/
void vpAROgre::updateCameraParameters (vpHomogeneousMatrix cMo)
{
	// As y from ViSP is -y from Ogre and z from ViSP is -z from Ogre
	// we have to change the matrix a little bit.
	// Rotations between x and y will therefore see their sign inverted
	// and so will rotations between x and z.
	// Rotations between y and z don't have to move as their relation is
	// the same in ViSP and Ogre.
	// As for translations, ty=-ty and tz=-tz as y=-y and z=-z
	//cMo.inverse();
	Ogre::Matrix4 ModelView = Ogre::Matrix4( cMo[0][0], -cMo[0][1], -cMo[0][2],  cMo[0][3],
						-cMo[1][0],  cMo[1][1],  cMo[1][2], -cMo[1][3],
						-cMo[2][0],  cMo[2][1],  cMo[2][2], -cMo[2][3],
						         0,          0,          0,          1);
	mCamera->setCustomViewMatrix(true, ModelView);

}

void vpAROgre::setEdgeThreshold()
{
float edgeth = mrend->edgeR_th;
Ogre::MaterialPtr dynamic_materialPtr = Ogre::MaterialPtr(Ogre::MaterialManager::getSingleton().getByName("EdgeGradMap"));
dynamic_materialPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("edgeTh",edgeth);
}

void vpAROgre::setResolution()
{
Ogre::MaterialPtr dynamic_materialPtr = Ogre::MaterialPtr(Ogre::MaterialManager::getSingleton().getByName("EdgeGradMap"));
dynamic_materialPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("iwidth",(float)mWidth);
dynamic_materialPtr->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("iheight",(float)mHeight);
}

/**
* Set rendering shaders, textures.
* \param name : Name of the SceneNode
* \param edgees : Type of rendered edgewether the case involves rendering only depth edges, depth and texture edges or depth and colour edges
*/
void vpAROgre::setRendering(std::string name, const EdgeType &edges, int width, int height)
{
	mWidth=width;
	mHeight=height;
	edgeT = edges;
	setEdgeThreshold();
	switch(edgeT){
	case DEPTH :
	setMat(name,"NormalMap");
	setRTT("DepthGradNormMap0");
	break;

	case DEPTHTEXTURE :
	setShaders(name);
	setMultiRTT("CannyOrient","DepthGradNormMap");
    break;

	case DEPTHCOLOUR :
	setLights();
	setShadersCol(name);
	setMRTTCol("CannyOrient","DepthGradNormMap1");
	break;
	}

}

/**
* Function to set lights on the scene, when rendering coloured objects
*/

void vpAROgre::setLights()
{

Ogre::Light* directionalLight0 = mSceneMgr->createLight("directionalLight0");
	    directionalLight0->setType(Ogre::Light::LT_DIRECTIONAL);
	    directionalLight0->setDiffuseColour(Ogre::ColourValue(0.2, 0.2, 0.2));
	    directionalLight0->setSpecularColour(Ogre::ColourValue(0.5, 0.5, 0.5));
	    directionalLight0->setDirection(Ogre::Vector3( 1, 0, 0 ));
Ogre::Light* directionalLight1 = mSceneMgr->createLight("directionalLight1");
	    directionalLight1->setType(Ogre::Light::LT_DIRECTIONAL);
		directionalLight1->setDiffuseColour(Ogre::ColourValue(0.2, 0.2, 0.2));
	    directionalLight1->setSpecularColour(Ogre::ColourValue(0.5, 0.5, 0.5));
		directionalLight1->setDirection(Ogre::Vector3( 0, 1, 0 ));
Ogre::Light* directionalLight2 = mSceneMgr->createLight("directionalLight2");
		directionalLight2->setType(Ogre::Light::LT_DIRECTIONAL);
		directionalLight2->setDiffuseColour(Ogre::ColourValue(0.2, 0.2, 0.2));
		directionalLight2->setSpecularColour(Ogre::ColourValue(0.5, 0.5, 0.5));
		directionalLight2->setDirection(Ogre::Vector3( 0, 0, 1 ));
Ogre::Light* directionalLight3 = mSceneMgr->createLight("directionalLight3");
		directionalLight3->setType(Ogre::Light::LT_DIRECTIONAL);
		directionalLight3->setDiffuseColour(Ogre::ColourValue(0.2, 0.2, 0.2));
		directionalLight3->setSpecularColour(Ogre::ColourValue(0.5, 0.5, 0.5));
		directionalLight3->setDirection(Ogre::Vector3(-1, 0, 0 ));
Ogre::Light* directionalLight4 = mSceneMgr->createLight("directionalLight4");
		directionalLight4->setType(Ogre::Light::LT_DIRECTIONAL);
		directionalLight4->setDiffuseColour(Ogre::ColourValue(0.2, 0.2, 0.2));
		directionalLight4->setSpecularColour(Ogre::ColourValue(0.5, 0.5, 0.5));
		directionalLight4->setDirection(Ogre::Vector3( 0, -1, 0 ));
Ogre::Light* directionalLight5 = mSceneMgr->createLight("directionalLight5");
		directionalLight5->setType(Ogre::Light::LT_DIRECTIONAL);
		directionalLight5->setDiffuseColour(Ogre::ColourValue(0.2, 0.2, 0.2));
		directionalLight5->setSpecularColour(Ogre::ColourValue(0.5, 0.5, 0.5));
		directionalLight5->setDirection(Ogre::Vector3( 0, 0, -1 ));
}

void vpAROgre::setLightA()
{

Ogre::Light* pointLight = mSceneMgr->createLight("pointLight");
pointLight->setType(Ogre::Light::LT_POINT);
pointLight->setPosition(Ogre::Vector3(1, 0, 0));
pointLight->setDiffuseColour(1.0, 1.0, 1.0);
pointLight->setSpecularColour(0.5, 0.5, 0.5);

}

/**
* Set material on the scene with a script.
* \param name : Name of the SceneNode
* \param material : Name of the material script
*/
void vpAROgre::setMat(std::string name, std::string material)
{
	Ogre::Entity *newEntity = mSceneMgr->getEntity(name);
    newEntity->setMaterialName(material);
    //newEntity->getSubEntity(0)->getMaterial()->getBestTechnique()->setCullingMode(Ogre::CULL_NONE);
}

/**
* Remove material from the scene.
* \param material : Name of the material script
*/
void vpAROgre::removeMat(std::string material)
{
	//Ogre::Entity *newEntity = mSceneMgr->getEntity(name);
    //newEntity->setMaterialName(material);
    Ogre::MaterialPtr Mat = Ogre::MaterialManager::getSingleton().getByName(material);
    Mat->removeAllTechniques();
}


/**
* Set render to texture to render depth edges, and the normal map of the scene, through a compositor.
* \param compositor : Name of the compositor script
*/
void vpAROgre::setRTT(std::string compositor)
{
    Ogre::TexturePtr Texture = Ogre::TextureManager::getSingleton().createManual(
	                    "rtf", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
	                    Ogre::TEX_TYPE_2D, mWidth, mHeight,
	                            0,  Ogre::PF_BYTE_L, Ogre::TU_RENDERTARGET);
    Ogre::RenderTexture* RTarget = Texture->getBuffer()->getRenderTarget();
    Ogre::Viewport* Viewport = RTarget->addViewport(mCamera);
    RTarget->getViewport(0)->setClearEveryFrame(true);
    //RTarget->getViewport(0)->setBackgroundColour(Ogre::ColourValue::White);
    RTarget->getViewport(0)->setOverlaysEnabled(false);

    Ogre::CompositorInstance* pCompositor = Ogre::CompositorManager::getSingleton().addCompositor(Viewport, compositor);
    Ogre::CompositorManager::getSingleton().setCompositorEnabled(Viewport, compositor, true);

    pCompositor->getTechnique()->getTargetPass(0)->setInputMode(Ogre::CompositionTargetPass::IM_PREVIOUS);


}

void vpAROgre::setRTTex(std::string compositor)
{
    Ogre::TexturePtr Texture = Ogre::TextureManager::getSingleton().createManual(
	                    "rtf", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
	                    Ogre::TEX_TYPE_2D, mWidth, mHeight,
	                            0,  Ogre::PF_A8R8G8B8, Ogre::TU_RENDERTARGET);
    Ogre::RenderTexture* RTarget = Texture->getBuffer()->getRenderTarget();
    Ogre::Viewport* Viewport = RTarget->addViewport(mCamera);


    RTarget->getViewport(0)->setClearEveryFrame(true);
    //depthTarget1->getViewport(0)->setBackgroundColour(Ogre::ColourValue::White);
    RTarget->getViewport(0)->setOverlaysEnabled(false);
}

/**
* Set render to texture to render depth edges, colour edges, and the normal map of the scene, through compositors.
* \param compositor0 : Name of the compositor script to render colour edges
* \param compositor0 : Name of the compositor script to render depth edges and the normal map
*/
void vpAROgre::setMRTTCol(std::string compositor0, std::string compositor1)
{
    Ogre::TexturePtr Texture0 = Ogre::TextureManager::getSingleton().createManual(
	                    "rtf0", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
	                    Ogre::TEX_TYPE_2D, mWidth, mHeight,
	                            0,  Ogre::PF_BYTE_L, Ogre::TU_RENDERTARGET);
    Ogre::RenderTexture* RTarget0 = Texture0->getBuffer()->getRenderTarget();
    Ogre::Viewport* Viewport0 = RTarget0->addViewport(mCamera);

    Viewport0->setMaterialScheme("MRT");

    RTarget0->getViewport(0)->setClearEveryFrame(true);
    RTarget0->getViewport(0)->setBackgroundColour(Ogre::ColourValue(0.5,0.5,0.5));
    //depthTarget1->getViewport(0)->setBackgroundColour(Ogre::ColourValue::White);
    RTarget0->getViewport(0)->setOverlaysEnabled(false);

    //Ogre::CompositorInstance* pCompositor0 = Ogre::CompositorManager::getSingleton().addCompositor(Viewport0, compositor0);
    //Ogre::CompositorManager::getSingleton().setCompositorEnabled(Viewport0, compositor0, true);

    //pCompositor0->getTechnique()->getTargetPass(0)->setInputMode(Ogre::CompositionTargetPass::IM_PREVIOUS);

    Ogre::TexturePtr Texture1 = Ogre::TextureManager::getSingleton().createManual(
	                    "rtf1", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
	                    Ogre::TEX_TYPE_2D, mWidth, mHeight,
	                            0,  Ogre::PF_BYTE_L, Ogre::TU_RENDERTARGET);
    Ogre::RenderTexture* RTarget1 = Texture1->getBuffer()->getRenderTarget();
    Ogre::Viewport* Viewport1 = RTarget1->addViewport(mCamera);

    Viewport1->setMaterialScheme("NormMap0");

    RTarget1->getViewport(0)->setClearEveryFrame(true);
    //RTarget1->getViewport(0)->setBackgroundColour(Ogre::ColourValue::White);
    RTarget1->getViewport(0)->setOverlaysEnabled(false);

    Ogre::CompositorInstance* pCompositor1 = Ogre::CompositorManager::getSingleton().addCompositor(Viewport1, compositor1);
    Ogre::CompositorManager::getSingleton().setCompositorEnabled(Viewport1, compositor1, true);

    //pCompositor->getTechnique()->getTargetPass(0)->setInputMode(Ogre::CompositionTargetPass::IM_PREVIOUS);

    /*mrttex = Ogre::Root::getSingleton().getRenderSystem()->createMultiRenderTarget("MRT0");

	RTarget0->setAutoUpdated(false);
	RTarget1->setAutoUpdated(false);

	mrttex->bindSurface(0, RTarget0);
	mrttex->bindSurface(1, RTarget1);
	mrttex->setAutoUpdated(true);
	Ogre::Viewport* viewmrt = mrttex->addViewport(mCamera);
	viewmrt->setMaterialScheme("MRT");
	viewmrt->setClearEveryFrame(true);
	viewmrt->setOverlaysEnabled(false);
	viewmrt->setSkiesEnabled(false);*/

}

/**
* Set render to texture to render depth edges, texture edges, and the normal map of the scene, through compositors.
* \param compositor0 : Name of the compositor script to render texture edges
* \param compositor0 : Name of the compositor script to render depth edges and the normal map
*/
void vpAROgre::setMultiRTT(std::string compositor0, std::string compositor1)
{
    Ogre::TexturePtr Texture0 = Ogre::TextureManager::getSingleton().createManual(
	                    "rtf0", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
	                    Ogre::TEX_TYPE_2D, mWidth, mHeight,
	                            0,  Ogre::PF_BYTE_L, Ogre::TU_RENDERTARGET);
    Ogre::RenderTexture* RTarget0 = Texture0->getBuffer()->getRenderTarget();
    Ogre::Viewport* Viewport0 = RTarget0->addViewport(mCamera);
    Viewport0->setMaterialScheme("MRT");

    RTarget0->getViewport(0)->setClearEveryFrame(true);
    //RTarget0->getViewport(0)->setBackgroundColour(Ogre::ColourValue::White);
    RTarget0->getViewport(0)->setOverlaysEnabled(false);

    /*Ogre::CompositorInstance* pCompositor0 = Ogre::CompositorManager::getSingleton().addCompositor(Viewport0, compositor0);
    Ogre::CompositorManager::getSingleton().setCompositorEnabled(Viewport0, compositor0, true);

    pCompositor0->getTechnique()->getTargetPass(0)->setInputMode(Ogre::CompositionTargetPass::IM_PREVIOUS);*/

    Ogre::TexturePtr Texture1 = Ogre::TextureManager::getSingleton().createManual(
	                    "rtf1", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
	                    Ogre::TEX_TYPE_2D, mWidth, mHeight,
	                            0,  Ogre::PF_BYTE_L, Ogre::TU_RENDERTARGET);
    Ogre::RenderTexture* RTarget1 = Texture1->getBuffer()->getRenderTarget();
    Ogre::Viewport* Viewport1 = RTarget1->addViewport(mCamera);

    Viewport1->setMaterialScheme("NormMap");

    RTarget1->getViewport(0)->setClearEveryFrame(true);
    //RTarget1->getViewport(0)->setBackgroundColour(Ogre::ColourValue::White);
    RTarget1->getViewport(0)->setOverlaysEnabled(false);

    Ogre::CompositorInstance* pCompositor1 = Ogre::CompositorManager::getSingleton().addCompositor(Viewport1, compositor1);
    Ogre::CompositorManager::getSingleton().setCompositorEnabled(Viewport1, compositor1, true);

    //pCompositor->getTechnique()->getTargetPass(0)->setInputMode(Ogre::CompositionTargetPass::IM_PREVIOUS);

    /*mrttex = Ogre::Root::getSingleton().getRenderSystem()->createMultiRenderTarget("MRT0");

	RTarget0->setAutoUpdated(false);
	RTarget1->setAutoUpdated(false);

	mrttex->bindSurface(0, RTarget0);
	mrttex->bindSurface(1, RTarget1);
	mrttex->setAutoUpdated(true);
	Ogre::Viewport* viewmrt = mrttex->addViewport(mCamera);
	viewmrt->setMaterialScheme("MRT");
	viewmrt->setClearEveryFrame(true);
	viewmrt->setOverlaysEnabled(false);
	viewmrt->setSkiesEnabled(false);*/

}

/*void vpAROgre::setRTTHyb0(std::string compositor0, std::string compositor1)
{
    Ogre::TexturePtr Texture0 = Ogre::TextureManager::getSingleton().createManual(
	                    "rtf0", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
	                    Ogre::TEX_TYPE_2D, mWindow->getWidth(), mWindow->getHeight(),
	                            0,  Ogre::PF_BYTE_L, Ogre::TU_RENDERTARGET);
    Ogre::RenderTexture* RTarget0 = Texture0->getBuffer()->getRenderTarget();
    Ogre::Viewport* Viewport0 = RTarget0->addViewport(mCamera);

    Viewport0->setMaterialScheme("MRT");

    RTarget0->getViewport(0)->setClearEveryFrame(true);
    //depthTarget1->getViewport(0)->setBackgroundColour(Ogre::ColourValue::White);
    RTarget0->getViewport(0)->setOverlaysEnabled(false);

    Ogre::CompositorInstance* pCompositor0 = Ogre::CompositorManager::getSingleton().addCompositor(Viewport0, compositor0);
    Ogre::CompositorManager::getSingleton().setCompositorEnabled(Viewport0, compositor0, true);

    pCompositor0->getTechnique()->getTargetPass(0)->setInputMode(Ogre::CompositionTargetPass::IM_PREVIOUS);

    Ogre::TexturePtr Texture1 = Ogre::TextureManager::getSingleton().createManual(
	                    "rtf", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
	                    Ogre::TEX_TYPE_2D, mWindow->getWidth(), mWindow->getHeight(),
	                            0,  Ogre::PF_BYTE_L, Ogre::TU_RENDERTARGET);
    Ogre::RenderTexture* RTarget1 = Texture1->getBuffer()->getRenderTarget();
    Ogre::Viewport* Viewport1 = RTarget1->addViewport(mCamera);

    Viewport1->setMaterialScheme("NormMap0");

    RTarget1->getViewport(0)->setClearEveryFrame(true);
    //RTarget1->getViewport(0)->setBackgroundColour(Ogre::ColourValue::White);
    RTarget1->getViewport(0)->setOverlaysEnabled(false);

    Ogre::CompositorInstance* pCompositor1 = Ogre::CompositorManager::getSingleton().addCompositor(Viewport1, compositor1);
    Ogre::CompositorManager::getSingleton().setCompositorEnabled(Viewport1, compositor1, true);

    //pCompositor->getTechnique()->getTargetPass(0)->setInputMode(Ogre::CompositionTargetPass::IM_PREVIOUS);



}*/

/**
* Set render to texture to render depth edges, texture edges, and the normal map of the scene, through compositors.
* \param compositor0 : Name of the compositor script to render texture edges
* \param compositor0 : Name of the compositor script to render depth edges and the normal map
*/
void vpAROgre::setRTTHyb(std::string compositor0, std::string compositor1)
{
    Ogre::TexturePtr Texture0 = Ogre::TextureManager::getSingleton().createManual(
	                    "rtf0", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
	                    Ogre::TEX_TYPE_2D, mWidth, mHeight,
	                            0,  Ogre::PF_BYTE_L, Ogre::TU_RENDERTARGET);
    Ogre::RenderTexture* RTarget0 = Texture0->getBuffer()->getRenderTarget();
    Ogre::Viewport* Viewport0 = RTarget0->addViewport(mCamera);

    RTarget0->getViewport(0)->setClearEveryFrame(true);
    RTarget0->getViewport(0)->setBackgroundColour(Ogre::ColourValue(0.0,0.0,0.0));
    RTarget0->getViewport(0)->setOverlaysEnabled(false);

    Ogre::CompositorInstance* pCompositor0 = Ogre::CompositorManager::getSingleton().addCompositor(Viewport0, compositor0);
    Ogre::CompositorManager::getSingleton().setCompositorEnabled(Viewport0, compositor0, true);

    pCompositor0->getTechnique()->getTargetPass(0)->setInputMode(Ogre::CompositionTargetPass::IM_PREVIOUS);

    Ogre::TexturePtr Texture1 = Ogre::TextureManager::getSingleton().createManual(
	                    "rtf", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
	                    Ogre::TEX_TYPE_2D, mWidth, mHeight,
	                            0,  Ogre::PF_BYTE_L, Ogre::TU_RENDERTARGET);
    Ogre::RenderTexture* RTarget1 = Texture1->getBuffer()->getRenderTarget();
    Ogre::Viewport* Viewport1 = RTarget1->addViewport(mCamera);

    RTarget1->getViewport(0)->setClearEveryFrame(true);
    //RTarget1->getViewport(0)->setBackgroundColour(Ogre::ColourValue::White);
    RTarget1->getViewport(0)->setOverlaysEnabled(false);

    Ogre::CompositorInstance* pCompositor1 = Ogre::CompositorManager::getSingleton().addCompositor(Viewport1, compositor1);
    Ogre::CompositorManager::getSingleton().setCompositorEnabled(Viewport1, compositor1, true);

    pCompositor1->getTechnique()->getTargetPass(0)->setInputMode(Ogre::CompositionTargetPass::IM_PREVIOUS);


}


/**
* Set material, techniques, passes and shaders manually to render each texture attached to the scenenode.
* Two different schemes are necessary to both render the normal map (+depth edge map) and the texture map
* \param name : Name of the SceneNode
*/
void vpAROgre::setShaders(std::string name)
{

	Ogre::Entity *newEntity = mSceneMgr->getEntity(name);

	Ogre::HighLevelGpuProgramPtr vertexShader = Ogre::HighLevelGpuProgramManager::getSingleton().createProgram("TexProjectionV",
                                                                                                                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                                                                "glsl",
                                                                                                                Ogre::GPT_VERTEX_PROGRAM);


    //Code GLSL du vertex shader
    Ogre::String vertSrc;
    //vertSrc = "varying vec2 texture_coordinate;\n";
    //vertSrc += "varying vec3 oNormal;\n";
    vertSrc += "void main(void)\n";
    vertSrc += "{\n";
    vertSrc += "gl_Position = ftransform();\n";
    vertSrc += "gl_TexCoord[0] = gl_MultiTexCoord0;\n";
    //vertSrc += "oNormal = gl_Normal;\n";
    vertSrc += "}\n";

    //Affectation et chargement du code pour le vertex shader
    vertexShader->setSource(vertSrc);
    vertexShader->load();

    //Fragment shader (mêmes commentaires que ci-dessus)
    Ogre::HighLevelGpuProgramPtr fragmentShader = Ogre::HighLevelGpuProgramManager::getSingleton().createProgram("TexProjectionF",
                                                                                                                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                                                                "glsl",
                                                                                                                Ogre::GPT_FRAGMENT_PROGRAM);

    Ogre::String fragSrc;
    //fragSrc = "uniform sampler2D myTex; \n void main(void)\n { \n vec4 fcol = texture2D(myTex, gl_TexCoord[0].xy); \n float lum = 0.1*(fcol.x) + 0.3*(fcol.y) + 0.6*(fcol.z); \n gl_FragCoord = vec4(lum,lum,lum,1);\n }\n";
    fragSrc = "uniform sampler2D myTex; \n void main(void)\n { \n gl_FragColor = texture2D(myTex, gl_TexCoord[0].xy);\n }\n";
    //fragSrc = "uniform sampler2D myTex; \n void main(void)\n { \n gl_FragColor = vec4(0,0,1,1); \n }\n";

    fragmentShader->setSource(fragSrc);
    fragmentShader->load();



    Ogre::HighLevelGpuProgramPtr vertexShader1 = Ogre::HighLevelGpuProgramManager::getSingleton().createProgram("NormalMapV",
                                                                                                                    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                                                                    "glsl",
                                                                                                                    Ogre::GPT_VERTEX_PROGRAM);


        //Code GLSL du vertex shader
        Ogre::String vertSrc1;
        //vertSrc = "varying vec2 texture_coordinate;\n";
        vertSrc1 += "varying vec3 oNormal;\n";
        vertSrc1 += "void main(void)\n";
        vertSrc1 += "{\n";
        vertSrc1 += "gl_Position = ftransform();\n";
        //vertSrc += "gl_TexCoord[0] = gl_MultiTexCoord0;\n";
        vertSrc1 += "oNormal = gl_Normal;\n";
        vertSrc1 += "}\n";

        //Affectation et chargement du code pour le vertex shader
        vertexShader1->setSource(vertSrc1);
        vertexShader1->load();

        //Fragment shader (mêmes commentaires que ci-dessus)
        Ogre::HighLevelGpuProgramPtr fragmentShader1 = Ogre::HighLevelGpuProgramManager::getSingleton().createProgram("NormalMapF",
                                                                                                                    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                                                                    "glsl",
                                                                                                                    Ogre::GPT_FRAGMENT_PROGRAM);

        Ogre::String fragSrc1;
        fragSrc1 = "varying vec3 oNormal; \n void main(void)\n {\n float fDepth = gl_FragCoord.z; \n gl_FragColor = vec4(oNormal.z/2+0.5, oNormal.y/2+0.5, oNormal.x/2+0.5,fDepth); \n }\n";
        //fragSrc1 = "varying vec3 oNormal; \n void main(void)\n {\n float fDepth = gl_FragCoord.z; \n gl_FragColor = vec4(fDepth, fDepth, fDepth,fDepth); \n }\n";

        //fragSrc = "uniform sampler2D myTex; \n void main(void)\n { \n gl_FragColor = vec4(0,0,1,1); \n }\n";

        fragmentShader1->setSource(fragSrc1);
        fragmentShader1->load();

    //Affectation du vertex shader et du fragment shader aux passes des techniques du material de chaque sous entité
    //Et passage d'un paramètre float au vertex shader
    //et du paramètre pour le sampler 2D de texture pour le fragment shader
    for(int j = 0 ; j < newEntity->getNumSubEntities() ; j++)
    {
        for(int k = 0 ; k < newEntity->getSubEntity(j)->getMaterial()->getNumTechniques() ; k++){
            for(int i = 0 ; i < newEntity->getSubEntity(j)->getMaterial()->getTechnique(k)->getNumPasses() ; i++)
            {
            	newEntity->getSubEntity(j)->getMaterial()->getTechnique(k)->setSchemeName("MRT");
            	newEntity->getSubEntity(j)->getMaterial()->getTechnique(k)->getPass(i)->setVertexProgram("TexProjectionV");
                Ogre::GpuProgramParametersSharedPtr vParams = newEntity->getSubEntity(j)->getMaterial()->getTechnique(k)->getPass(i)->getVertexProgramParameters();
                //vParams->setNamedConstant("xi", xi);
                std::cout << " ok " << std::endl;
                newEntity->getSubEntity(j)->getMaterial()->getTechnique(k)->getPass(i)->setCullingMode(Ogre::CULL_NONE);

                newEntity->getSubEntity(j)->getMaterial()->getTechnique(k)->getPass(i)->setFragmentProgram("TexProjectionF");
                Ogre::GpuProgramParametersSharedPtr fParams = newEntity->getSubEntity(j)->getMaterial()->getTechnique(k)->getPass(i)->getFragmentProgramParameters();
                fParams->setNamedConstant("myTex", 0);
            }
        }
    Ogre::Technique* tech = newEntity->getSubEntity(j)->getMaterial()->createTechnique();
    tech->setSchemeName("NormMap");
    Ogre::Pass* pass = tech->createPass();
    pass->setVertexProgram("NormalMapV");
    pass->setCullingMode(Ogre::CULL_NONE);
    pass->setFragmentProgram("NormalMapF");
    }

}

/**
* Set material, techniques and shaders.
* Two different schemes are necessary to both render the normal map (+depth edge map) and the color map.
* \param name : Name of the SceneNode
*/
void vpAROgre::setShadersCol(std::string name)
{

	Ogre::Entity *newEntity = mSceneMgr->getEntity(name);

	Ogre::HighLevelGpuProgramPtr vertexShader = Ogre::HighLevelGpuProgramManager::getSingleton().createProgram("TexProjectionV",
                                                                                                                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                                                                "glsl",
                                                                                                                Ogre::GPT_VERTEX_PROGRAM);
    //Code GLSL du vertex shader
    Ogre::String vertSrc;
    //vertSrc = "varying vec2 texture_coordinate;\n";
    //vertSrc += "varying vec3 oNormal;\n";
    vertSrc += "void main(void)\n";
    vertSrc += "{\n";
    vertSrc += "gl_Position = ftransform();\n";
    vertSrc += "gl_TexCoord[0] = gl_MultiTexCoord0;\n";
    //vertSrc += "oNormal = gl_Normal;\n";
    vertSrc += "}\n";

    //Affectation et chargement du code pour le vertex shader
    vertexShader->setSource(vertSrc);
    vertexShader->load();

    //Fragment shader (mêmes commentaires que ci-dessus)
    Ogre::HighLevelGpuProgramPtr fragmentShader = Ogre::HighLevelGpuProgramManager::getSingleton().createProgram("TexProjectionF",
                                                                                                                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                                                                "glsl",
                                                                                                                Ogre::GPT_FRAGMENT_PROGRAM);

    Ogre::String fragSrc;
    //fragSrc = "uniform sampler2D myTex; \n void main(void)\n { \n vec4 fcol = texture2D(myTex, gl_TexCoord[0].xy); \n float lum = 0.1*(fcol.x) + 0.3*(fcol.y) + 0.6*(fcol.z); \n gl_FragCoord = vec4(lum,lum,lum,1);\n }\n";
    //fragSrc = "uniform sampler2D myTex; \n void main(void)\n { \n gl_FragColor = texture2D(myTex, gl_TexCoord[0].xy);\n }\n";
    //fragSrc = "uniform sampler2D myTex; \n void main(void)\n { \n gl_FragColor = vec4(0,0,1,1); \n }\n";

    fragmentShader->setSource(fragSrc);
    fragmentShader->load();



    Ogre::HighLevelGpuProgramPtr vertexShader1 = Ogre::HighLevelGpuProgramManager::getSingleton().createProgram("NormalMapV",
                                                                                                                    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                                                                    "glsl",
                                                                                                                    Ogre::GPT_VERTEX_PROGRAM);
        //Code GLSL du vertex shader
        Ogre::String vertSrc1;
        //vertSrc = "varying vec2 texture_coordinate;\n";
        vertSrc1 += "varying vec3 oNormal;\n";
        vertSrc1 += "void main(void)\n";
        vertSrc1 += "{\n";
        vertSrc1 += "gl_Position = ftransform();\n";
        //vertSrc += "gl_TexCoord[0] = gl_MultiTexCoord0;\n";
        vertSrc1 += "oNormal = gl_Normal;\n";
        vertSrc1 += "}\n";

        //Affectation et chargement du code pour le vertex shader
        vertexShader1->setSource(vertSrc1);
        vertexShader1->load();

        //Fragment shader (mêmes commentaires que ci-dessus)
        Ogre::HighLevelGpuProgramPtr fragmentShader1 = Ogre::HighLevelGpuProgramManager::getSingleton().createProgram("NormalMapF",
                                                                                                                    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                                                                    "glsl",
                                                                                                                    Ogre::GPT_FRAGMENT_PROGRAM);

        Ogre::String fragSrc1;
        fragSrc1 = "varying vec3 oNormal; \n void main(void)\n {\n float fDepth = gl_FragCoord.z; \n gl_FragColor = vec4(oNormal.z/2+0.5, oNormal.y/2+0.5, oNormal.x/2+0.5,fDepth); \n }\n";
        //fragSrc1 = "varying vec3 oNormal; \n void main(void)\n {\n float fDepth = gl_FragCoord.z; \n gl_FragColor = vec4(fDepth, fDepth, fDepth,fDepth); \n }\n";

        //fragSrc = "uniform sampler2D myTex; \n void main(void)\n { \n gl_FragColor = vec4(0,0,1,1); \n }\n";

        fragmentShader1->setSource(fragSrc1);
        fragmentShader1->load();

    //Affectation du vertex shader et du fragment shader aux passes des techniques du material de chaque sous entité
    //Et passage d'un paramètre float au vertex shader
    //et du paramètre pour le sampler 2D de texture pour le fragment shader
    for(int j = 0 ; j < newEntity->getNumSubEntities() ; j++)
    {
        for(int k = 0 ; k < newEntity->getSubEntity(j)->getMaterial()->getNumTechniques() ; k++){
            for(int i = 0 ; i < newEntity->getSubEntity(j)->getMaterial()->getTechnique(k)->getNumPasses() ; i++)
            {
            	newEntity->getSubEntity(j)->getMaterial()->getTechnique(k)->setSchemeName("MRT");
            }
        }
    Ogre::Technique* tech = newEntity->getSubEntity(j)->getMaterial()->createTechnique();
    tech->setSchemeName("NormMap0");
    Ogre::Pass* pass = tech->createPass();
    pass->setVertexProgram("NormalMapV");
    //pass->setCullingMode(Ogre::CULL_NONE);
    pass->setFragmentProgram("NormalMapF");
    }
}

void vpAROgre::setShadersT(std::string name)
{

	Ogre::Entity *newEntity = mSceneMgr->getEntity(name);

	Ogre::HighLevelGpuProgramPtr vertexShader = Ogre::HighLevelGpuProgramManager::getSingleton().createProgram("TexProjectionV",
                                                                                                                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                                                                "glsl",
                                                                                                                Ogre::GPT_VERTEX_PROGRAM);


    //Code GLSL du vertex shader
    Ogre::String vertSrc;
    //vertSrc = "varying vec2 texture_coordinate;\n";
    vertSrc += "varying vec3 oNormal;\n";
    vertSrc += "void main(void)\n";
    vertSrc += "{\n";
    vertSrc += "gl_Position = ftransform();\n";
    vertSrc += "gl_TexCoord[0] = gl_MultiTexCoord0;\n";
    vertSrc += "oNormal = gl_Normal;\n";
    vertSrc += "}\n";

    //Affectation et chargement du code pour le vertex shader
    vertexShader->setSource(vertSrc);
    vertexShader->load();

    //Fragment shader (mêmes commentaires que ci-dessus)
    Ogre::HighLevelGpuProgramPtr fragmentShader = Ogre::HighLevelGpuProgramManager::getSingleton().createProgram("TexProjectionF",
                                                                                                                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                                                                "glsl",
                                                                                                                Ogre::GPT_FRAGMENT_PROGRAM);

    Ogre::String fragSrc;
    fragSrc = "uniform sampler2D myTex;\n varying vec3 oNormal; \n void main(void)\n { \n vec4 fcol = texture2D(myTex, gl_TexCoord[0].xy); \n float lum = 0.1*(fcol.x) + 0.3*(fcol.y) + 0.6*(fcol.z); \n gl_FragColor = vec4(oNormal.z/2+0.5, oNormal.y/2+0.5, oNormal.x/2+0.5,lum);\n }\n";
    //fragSrc = "uniform sampler2D myTex; \n void main(void)\n { \n gl_FragColor = texture2D(myTex, gl_TexCoord[0].xy);\n }\n";
    //fragSrc = "uniform sampler2D myTex; \n void main(void)\n { \n gl_FragColor = vec4(0,0,1,1); \n }\n";

    fragmentShader->setSource(fragSrc);
    fragmentShader->load();


    //Affectation du vertex shader et du fragment shader aux passes des techniques du material de chaque sous entité
    //Et passage d'un paramètre float au vertex shader
    //et du paramètre pour le sampler 2D de texture pour le fragment shader
    for(int j = 0 ; j < newEntity->getNumSubEntities() ; j++)
    {
        for(int k = 0 ; k < newEntity->getSubEntity(j)->getMaterial()->getNumTechniques() ; k++){
            for(int i = 0 ; i < newEntity->getSubEntity(j)->getMaterial()->getTechnique(k)->getNumPasses() ; i++)
            {
            	newEntity->getSubEntity(j)->getMaterial()->getTechnique(k)->getPass(i)->setVertexProgram("TexProjectionV");
                Ogre::GpuProgramParametersSharedPtr vParams = newEntity->getSubEntity(j)->getMaterial()->getTechnique(k)->getPass(i)->getVertexProgramParameters();
                //vParams->setNamedConstant("xi", xi);
                std::cout << " ok " << std::endl;
                newEntity->getSubEntity(j)->getMaterial()->getTechnique(k)->getPass(i)->setCullingMode(Ogre::CULL_NONE);

                newEntity->getSubEntity(j)->getMaterial()->getTechnique(k)->getPass(i)->setFragmentProgram("TexProjectionF");
                Ogre::GpuProgramParametersSharedPtr fParams = newEntity->getSubEntity(j)->getMaterial()->getTechnique(k)->getPass(i)->getFragmentProgramParameters();
                fParams->setNamedConstant("myTex", 0);
            }
        }
    }

}

/**
* Update render to texture.
* Three cases : depth edges, depth + texture edges, depth + color edges
* \param Inormd : RGBA map containing the normal map + depth map of the rendered scene.
* \param Ior : depth edges map, with edge orientations
* \param Itex : texture or color map
*/
void vpAROgre::updateRendering(vpImage<vpRGBa> &Inormd, vpImage<unsigned char> &Ior, vpImage<unsigned char> &Itex, vpHomogeneousMatrix *cMo)
{
    vpTranslationVector translat;
    cMo->extract(translat);
	updateClipDistances(translat[2]);

	switch(edgeT){
	case DEPTH :
	updateRTT(Inormd,Ior,cMo);
	break;

	case DEPTHTEXTURE :
	updateRTTex(Itex,Inormd,Ior,cMo);
	apEdgeMap::edgeOrientMap(Itex);
    break;

	case DEPTHCOLOUR :
	updateRTTex(Itex,Inormd,Ior,cMo);
	apEdgeMap::edgeOrientMap(Itex);
	break;
	}

}


/**
* Update render to texture for depth edges
* \param Inormd : RGBA map containing the normal map + depth map of the rendered scene.
* \param Ior : depth edges map, with edge orientations
* \param CMo : camera parameters
*/
void vpAROgre::updateRTT(vpImage<vpRGBa> &I1, vpImage<unsigned char> &I0, vpHomogeneousMatrix *CMo)
{
updateCameraParameters(*CMo);
Ogre::TexturePtr dynTexPtr0 = Ogre::TextureManager::getSingleton().getByName("rtf");
Ogre::RenderTexture* RTarget = dynTexPtr0->getBuffer()->getRenderTarget();
mWindow->update();
RTarget->update();
std::string texture = Ogre::CompositorManager::getSingleton().getCompositorChain(dynTexPtr0->getBuffer()->getRenderTarget()->getViewport(0))->getCompositor(0)->getTextureInstanceName("rt2", 0);
Ogre::TexturePtr dynTexPtr1 = Ogre::TextureManager::getSingleton().getByName(texture);
Ogre::HardwarePixelBufferSharedPtr mPixelBuffer0 = dynTexPtr0->getBuffer();
Ogre::HardwarePixelBufferSharedPtr mPixelBuffer1 = dynTexPtr1->getBuffer();

mPixelBuffer1->lock(Ogre::HardwareBuffer::HBL_DISCARD);
mPixelBuffer0->lock(Ogre::HardwareBuffer::HBL_DISCARD);
const Ogre::PixelBox& pixelBox1 = mPixelBuffer1->getCurrentLock();
const Ogre::PixelBox& pixelBox0 = mPixelBuffer0->getCurrentLock();
double t0= vpTime::measureTimeMs();
dynTexPtr0->getBuffer()->blitToMemory(pixelBox0);
double t1= vpTime::measureTimeMs();
dynTexPtr1->getBuffer()->blitToMemory(pixelBox1);
Ogre::uint32* pDest0 = static_cast<Ogre::uint32*>(pixelBox0.data);
Ogre::uint32* pDest1 = static_cast<Ogre::uint32*>(pixelBox1.data);
memcpy(I0.bitmap, pDest0, mHeight*mWidth);
memcpy(I1.bitmap, pDest1, mHeight*mWidth*4);
// Unlock the pixel buffer
mPixelBuffer1->unlock();
mPixelBuffer0->unlock();
std::cout<<" timeBlit "<<t1-t0<<std::endl;

/*char buf[FILENAME_MAX];
sprintf(buf, opath.c_str(), im);
std::string filename(buf);
std::cout << "Write: " << filename << std::endl;*/
//vpImageIo::writePPM(I0, "texture0.pgm");
//vpImageIo::writePPM(I1, "texture10.pgm");


	                /*for (int n=100; n < 300 ; n++)
	                            {
	                            for (int m = 200 ; m < 400; m++)
	                              {
	                                //I1[n][m] =  (double)I2[n][m]/255;
	                            	std::cout << (double)I1[n][m].A << std::endl;
	                              }
	                            }*/
}

void vpAROgre::updateRTTGrad(vpImage<unsigned char> &I0, vpHomogeneousMatrix *CMo)
{
double t0= vpTime::measureTimeMs();
updateCameraParameters(*CMo);
Ogre::TexturePtr dynTexPtr0 = Ogre::TextureManager::getSingleton().getByName("rtf0");
Ogre::RenderTexture* RTarget = dynTexPtr0->getBuffer()->getRenderTarget();
//mWindow->update();
RTarget->update();
Ogre::HardwarePixelBufferSharedPtr mPixelBuffer0 = dynTexPtr0->getBuffer();

mPixelBuffer0->lock(Ogre::HardwareBuffer::HBL_DISCARD);

const Ogre::PixelBox& pixelBox0 = mPixelBuffer0->getCurrentLock();

dynTexPtr0->getBuffer()->blitToMemory(pixelBox0);
Ogre::uint32* pDest0 = static_cast<Ogre::uint32*>(pixelBox0.data);
memcpy(I0.bitmap, pDest0, mHeight*mWidth);

mPixelBuffer0->unlock();

double t1= vpTime::measureTimeMs();
std::cout<<" time "<<t1-t0<<std::endl;

}


void vpAROgre::updateMultiRTT(vpImage<vpRGBa> &I1, vpImage<unsigned char> &I0, vpHomogeneousMatrix *CMo)
{
double t0= vpTime::measureTimeMs();
updateCameraParameters(*CMo);
Ogre::TexturePtr dynTexPtr = Ogre::TextureManager::getSingleton().getByName("rtf");
Ogre::RenderTexture* RTarget = dynTexPtr->getBuffer()->getRenderTarget();
mWindow->update();
RTarget->update();

//Ogre::TexturePtr dynTexPtr0 = Ogre::TextureManager::getSingleton().getByName("rtf1");
//std::string texture = Ogre::CompositorManager::getSingleton().getCompositorChain(dynTexPtr0->getBuffer()->getRenderTarget()->getViewport(0))->getCompositor(0)->getTextureInstanceName("rt2", 0);
//Ogre::TexturePtr dynTexPtr1 = Ogre::TextureManager::getSingleton().getByName(texture);
Ogre::HardwarePixelBufferSharedPtr mPixelBuffer0 = dynTexPtr->getBuffer();
//Ogre::HardwarePixelBufferSharedPtr mPixelBuffer1 = dynTexPtr1->getBuffer();

//mPixelBuffer1->lock(Ogre::HardwareBuffer::HBL_DISCARD);
mPixelBuffer0->lock(Ogre::HardwareBuffer::HBL_DISCARD);
//const Ogre::PixelBox& pixelBox1 = mPixelBuffer1->getCurrentLock();
const Ogre::PixelBox& pixelBox0 = mPixelBuffer0->getCurrentLock();

dynTexPtr->getBuffer()->blitToMemory(pixelBox0);
//dynTexPtr1->getBuffer()->blitToMemory(pixelBox1);
Ogre::uint32* pDest0 = static_cast<Ogre::uint32*>(pixelBox0.data);
//Ogre::uint32* pDest1 = static_cast<Ogre::uint32*>(pixelBox1.data);
memcpy(I1.bitmap, pDest0, mHeight*mWidth*4);
//memcpy(I1.bitmap, pDest1, mHeight*mWidth*4);

// Unlock the pixel buffer
//mPixelBuffer1->unlock();
mPixelBuffer0->unlock();

double t1= vpTime::measureTimeMs();
std::cout<<" time "<<t1-t0<<std::endl;

/*char buf[FILENAME_MAX];
sprintf(buf, opath.c_str(), im);
std::string filename(buf);
std::cout << "Write: " << filename << std::endl;*/
//vpImageIo::writePPM(I0, "texture0.pgm");
//vpImageIo::writePPM(I1, "texture10.pgm");


	                /*for (int n=100; n < 300 ; n++)
	                            {
	                            for (int m = 200 ; m < 400; m++)
	                              {
	                                //I1[n][m] =  (double)I2[n][m]/255;
	                            	std::cout << (double)I1[n][m].A << std::endl;
	                              }
	                            }*/
}


void vpAROgre::updateRTTex(vpImage<unsigned char> &I1, vpImage<vpRGBa> &I2, vpImage<unsigned char> &I0, vpHomogeneousMatrix *CMo)
{
double t0= vpTime::measureTimeMs();
updateCameraParameters(*CMo);
Ogre::TexturePtr dynTexPtr0 = Ogre::TextureManager::getSingleton().getByName("rtf0");
Ogre::TexturePtr dynTexPtr1 = Ogre::TextureManager::getSingleton().getByName("rtf1");
Ogre::RenderTexture* RTarget0 = dynTexPtr0->getBuffer()->getRenderTarget();
Ogre::RenderTexture* RTarget1 = dynTexPtr1->getBuffer()->getRenderTarget();
RTarget0->update();
mWindow->update();
RTarget1->update();

Ogre::HardwarePixelBufferSharedPtr mPixelBuffer0 = dynTexPtr0->getBuffer();

mPixelBuffer0->lock(Ogre::HardwareBuffer::HBL_DISCARD);
const Ogre::PixelBox& pixelBox0 = mPixelBuffer0->getCurrentLock();
dynTexPtr0->getBuffer()->blitToMemory(pixelBox0);

std::string texture = Ogre::CompositorManager::getSingleton().getCompositorChain(dynTexPtr1->getBuffer()->getRenderTarget()->getViewport(0))->getCompositor(0)->getTextureInstanceName("rt2", 0);
Ogre::TexturePtr dynTexPtr2 = Ogre::TextureManager::getSingleton().getByName(texture);

Ogre::HardwarePixelBufferSharedPtr mPixelBuffer1 = dynTexPtr1->getBuffer();

Ogre::HardwarePixelBufferSharedPtr mPixelBuffer2 = dynTexPtr2->getBuffer();

mPixelBuffer1->lock(Ogre::HardwareBuffer::HBL_DISCARD);
mPixelBuffer2->lock(Ogre::HardwareBuffer::HBL_DISCARD);


const Ogre::PixelBox& pixelBox1 = mPixelBuffer1->getCurrentLock();
const Ogre::PixelBox& pixelBox2 = mPixelBuffer2->getCurrentLock();


dynTexPtr1->getBuffer()->blitToMemory(pixelBox1);
dynTexPtr2->getBuffer()->blitToMemory(pixelBox2);

Ogre::uint8* pDest0 = static_cast<Ogre::uint8*>(pixelBox0.data);
Ogre::uint32* pDest1 = static_cast<Ogre::uint32*>(pixelBox1.data);
Ogre::uint32* pDest2 = static_cast<Ogre::uint32*>(pixelBox2.data);

memcpy(I1.bitmap, pDest0, mHeight*mWidth);
memcpy(I0.bitmap, pDest1, mHeight*mWidth);
memcpy(I2.bitmap, pDest2, mHeight*mWidth*4);

// Unlock the pixel buffer
mPixelBuffer0->unlock();
mPixelBuffer1->unlock();
mPixelBuffer2->unlock();


double t1= vpTime::measureTimeMs();
std::cout<<" time "<<t1-t0<<std::endl;

/*char buf[FILENAME_MAX];
sprintf(buf, opath.c_str(), im);
std::string filename(buf);
std::cout << "Write: " << filename << std::endl;*/
//vpImageIo::writePPM(I0, "texture0.pgm");
//vpImageIo::writePPM(I1, "texture10.pgm");


	                /*for (int n=100; n < 300 ; n++)
	                            {
	                            for (int m = 200 ; m < 400; m++)
	                              {
	                                //I1[n][m] =  (double)I2[n][m]/255;
	                            	std::cout << (double)I1[n][m].A << std::endl;
	                              }
	                            }*/
}

void vpAROgre::updateRTTV(vpImage<unsigned char> &I0, vpHomogeneousMatrix *CMo)
{

vpImage<vpRGBa> I1(480,640);
//double t0= vpTime::measureTimeMs();
updateCameraParameters(*CMo);
Ogre::TexturePtr dynTexPtr0 = Ogre::TextureManager::getSingleton().getByName("rtf");
Ogre::RenderTexture* RTarget = dynTexPtr0->getBuffer()->getRenderTarget();
RTarget->update();
//mWindow->update();
Ogre::HardwarePixelBufferSharedPtr mPixelBuffer0 = dynTexPtr0->getBuffer();

mPixelBuffer0->lock(Ogre::HardwareBuffer::HBL_DISCARD);
const Ogre::PixelBox& pixelBox0 = mPixelBuffer0->getCurrentLock();
double t0= vpTime::measureTimeMs();
dynTexPtr0->getBuffer()->blitToMemory(pixelBox0);
double t1= vpTime::measureTimeMs();
Ogre::uint32* pDest0 = static_cast<Ogre::uint32*>(pixelBox0.data);
memcpy(I0.bitmap, pDest0, mHeight*mWidth);
// Unlock the pixel buffer
mPixelBuffer0->unlock();

/*std::string texture = Ogre::CompositorManager::getSingleton().getCompositorChain(dynTexPtr0->getBuffer()->getRenderTarget()->getViewport(0))->getCompositor(0)->getTextureInstanceName("rt1", 0);
Ogre::TexturePtr dynTexPtr1 = Ogre::TextureManager::getSingleton().getByName(texture);
Ogre::HardwarePixelBufferSharedPtr mPixelBuffer1 = dynTexPtr1->getBuffer();
mPixelBuffer1->lock(Ogre::HardwareBuffer::HBL_DISCARD);
const Ogre::PixelBox& pixelBox1 = mPixelBuffer1->getCurrentLock();
dynTexPtr1->getBuffer()->blitToMemory(pixelBox1);
Ogre::uint32* pDest1 = static_cast<Ogre::uint32*>(pixelBox1.data);
memcpy(I1.bitmap, pDest1, mHeight*mWidth*4);
mPixelBuffer1->unlock();*/

std::cout<<" timeblit "<<t1-t0<<std::endl;

/*char buf[FILENAME_MAX];
sprintf(buf, opath.c_str(), im);
std::string filename(buf);
std::cout << "Write: " << filename << std::endl;*/
//vpImageIo::writePPM(I0, "texture0.pgm");
//vpImageIo::writePPM(I1, "texture10.pgm");


	                /*for (int n=100; n < 300 ; n++)
	                            {
	                            for (int m = 200 ; m < 400; m++)
	                              {
	                                //I1[n][m] =  (double)I2[n][m]/255;
	                            	std::cout << (double)I1[n][m].A << std::endl;
	                              }
	                            }*/
}


void vpAROgre::rtt0(vpImage<unsigned char> &I, vpHomogeneousMatrix *CMo)
{
	updateCameraParameters(*CMo);
    //Ogre::TexturePtr depthTarget;
    //Ogre::RenderTexture *depthTexture;
	double t0= vpTime::measureTimeMs();

	    // Create the depth render texture
     Ogre::TexturePtr depthTexture = Ogre::TextureManager::getSingleton().createManual(
 	                    "DepthMapTexture", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
 	                    Ogre::TEX_TYPE_2D, mWindow->getWidth(), mWindow->getHeight(),
 	                            0,  Ogre::PF_R8G8B8, Ogre::TU_RENDERTARGET);

 	    // Get its render target and add a viewport to it
     Ogre::RenderTexture* depthTarget = depthTexture->getBuffer()->getRenderTarget();
 	            //Ogre::Viewport* depthViewport = depthTarget->addViewport(mCamera, 40);
     Ogre::Viewport* depthViewport = depthTarget->addViewport(mCamera);
     depthTarget->getViewport(0)->setClearEveryFrame(true);
     depthTarget->getViewport(0)->setBackgroundColour(Ogre::ColourValue::White);
     depthTarget->getViewport(0)->setOverlaysEnabled(false);

     //depthTarget->update();

     getchar();

     Ogre::TexturePtr mTexturePtr = Ogre::TextureManager::getSingleton().createManual( "edgeMapTexture",
    		 Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
    		  	                    Ogre::TEX_TYPE_2D, mWindow->getWidth(), mWindow->getHeight(),
    		  	                            0,  Ogre::PF_R8G8B8, Ogre::TU_RENDERTARGET);

        Ogre::RenderTarget *mRenderTarget = mTexturePtr->getBuffer()->getRenderTarget();

        Ogre::Viewport* rttVP = mRenderTarget->addViewport(mCamera);
        mRenderTarget->getViewport(0)->setClearEveryFrame(true);
        mRenderTarget->getViewport(0)->setBackgroundColour(Ogre::ColourValue::Black);
        mRenderTarget->getViewport(0)->setOverlaysEnabled(false);


        Ogre::CompositorPtr compositor = Ogre::CompositorManager::getSingleton().create(
           "OldTVCompositor", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

        Ogre::CompositionTechnique* mTechnique = compositor->createTechnique();
        {
           {
              Ogre::CompositionTechnique::TextureDefinition* definition = mTechnique->createTextureDefinition("scene");
              definition->width = 640;  // target width
              definition->height = 480; // target height
              //definition->format = Ogre::PF_R8G8B8;
           }
           {
              Ogre::CompositionTargetPass* target = mTechnique->createTargetPass();
              target->setInputMode(Ogre::CompositionTargetPass::IM_PREVIOUS);
              target->setOutputName("scene");
           }
           //mTexturePtr = compositor->getTextureInstance("scene",0);
           {
              Ogre::CompositionTargetPass* output = mTechnique->getOutputTargetPass();
              output->setInputMode(Ogre::CompositionTargetPass::IM_NONE);
              {
                 // Create a output pass
                 Ogre::CompositionPass* pass = output->createPass();
                 pass->setType(Ogre::CompositionPass::PT_RENDERQUAD);
                 pass->setMaterialName("DepthMap30");
                 pass->setInput(0, "scene");
              }
           }

        }

        Ogre::CompositorInstance* instance = Ogre::CompositorManager::getSingleton().addCompositor(
              rttVP, "OldTVCompositor");//mCompositorType->name);
           Ogre::CompositorManager::getSingleton().setCompositorEnabled(rttVP,
                          "OldTVCompositor", true);
           mRenderTarget->update();


     //depthTarget->writeContentsToTimestampedFile("target", ".png");




     double t1= vpTime::measureTimeMs();

	            mWindow->update();


	            depthTarget->writeContentsToTimestampedFile("target1", ".png");


	            std::cout << "time  "<< t1-t0<<std::endl;

	            vpImage<unsigned char> I1(480,640);
	            Ogre::TexturePtr dynTexPtr = Ogre::TextureManager::getSingleton().getByName("scene");
	            Ogre::HardwarePixelBufferSharedPtr mPixelBuffer = dynTexPtr->getBuffer();

	            	            // Inspired from Ogre wiki : http://www.ogre3d.org/tikiwiki/Creating+dynamic+textures
	            	            // Lock the pixel buffer and get a pixel box. HBL_DISCARD is to use for best
	            	            // performance than HBL_NORMAL
	            	            mPixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD); // Lock the buffer
	            	            const Ogre::PixelBox& pixelBox = mPixelBuffer->getCurrentLock();
	            	            depthTexture->getBuffer()->blitToMemory(pixelBox);
	            	            // Buffer data
	            	            Ogre::uint8* pDest = static_cast<Ogre::uint8*>(pixelBox.data);
	            	            // Fill in the data in the grey level texture

	            	            memcpy(I1.bitmap, pDest, mWindow->getHeight()*mWindow->getWidth());
	            	            // Unlock the pixel buffer
	            	            mPixelBuffer->unlock();

	            	            /*char buf[FILENAME_MAX];
							    sprintf(buf, opath.c_str(), im);
								std::string filename(buf);
								std::cout << "Write: " << filename << std::endl;*/
								vpImageIo::writePPM(I1, "texture0.pgm");



}

void vpAROgre::rtt(vpImage<unsigned char> &I, vpHomogeneousMatrix *CMo)
{
	updateCameraParameters(*CMo);
    //Ogre::TexturePtr depthTarget;
    //Ogre::RenderTexture *depthTexture;

	    // Create the depth render texture
     /*Ogre::TexturePtr depthTexture = Ogre::TextureManager::getSingleton().createManual(
 	                    "DepthMapTexture", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
 	                    Ogre::TEX_TYPE_2D, mWindow->getWidth(), mWindow->getHeight(),
 	                            0,  Ogre::PF_FLOAT32_RGBA, Ogre::TU_RENDERTARGET);

 	    // Get its render target and add a viewport to it
     Ogre::RenderTexture* depthTarget = depthTexture->getBuffer()->getRenderTarget();
 	            //Ogre::Viewport* depthViewport = depthTarget->addViewport(mCamera, 40);
     Ogre::Viewport* depthViewport = depthTarget->addViewport(mCamera);
     depthTarget->getViewport(0)->setClearEveryFrame(true);
     depthTarget->getViewport(0)->setBackgroundColour(Ogre::ColourValue::White);
     depthTarget->getViewport(0)->setOverlaysEnabled(false);


	           // depthViewport->setBackgroundColour(Ogre::ColourValue::Black);

	    // Register 'this' as a render target listener
	            depthTarget->addListener(this->listener);

	    // Get the technique to use when rendering the depth render texture

     Ogre::MaterialPtr DepthMaterial = Ogre::MaterialManager::getSingleton().create("DepthMapMat", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);


     DepthMaterial->getTechnique(0)->getPass(0)->createTextureUnitState("DepthMapTexture");
     DepthMaterial->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
     DepthMaterial->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
     DepthMaterial->getTechnique(0)->getPass(0)->setLightingEnabled(false);*/

     //depthTexture = Ogre::TextureManager::getSingleton().getByName("DepthMapTexture");
     //depthTarget = depthTexture->getBuffer()->getRenderTarget();



     /*Ogre::MaterialPtr mDepthMaterial = Ogre::MaterialManager::getSingleton().getByName("DepthMap4");
	             Ogre::Technique* matTechnique = mDepthMaterial->createTechnique();
	             matTechnique->createPass();
	             mDepthMaterial->getTechnique(0)->getPass(0)->setLightingEnabled(false);
	             mDepthMaterial->getTechnique(0)->getPass(0)->createTextureUnitState("DepthMapTexture");
	             mDepthMaterial->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
	             mDepthMaterial->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);*/
	            //mDepthMaterial->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTextureName("A330DepthMap");
	            //mDepthMaterial->compile();
	            //mDepthMaterial->load(); // needs to be loaded manually
	            //Ogre::Technique* mDepthTechnique = mDepthMaterial->getBestTechnique();
	        //Ogre::MaterialPtr DepthMaterial = Ogre::MaterialManager::getSingleton().load("DepthMap4", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
     //depthTarget->update();



     /*Ogre::TexturePtr mTexturePtr = Ogre::TextureManager::getSingleton().createManual( "edgeMap",
              ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, TEX_TYPE_2D,
              mWindow->getWidth(), mWindow->getHeight(), 0, Ogre::PF_R8G8B8, TU_RENDERTARGET );
        Ogre::RenderTarget *mRenderTarget = mTexturePtr->getBuffer()->getRenderTarget();

        rttVP = mRenderTarget->addViewport(mCamera);

        CompositorPtr compositor = CompositorManager::getSingleton().create(
           "OldTVCompositor", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

        mTechnique = compositor->createTechnique();
        {
           {
              CompositionTechnique::TextureDefinition* definition = mTechnique->createTextureDefinition("scene");
              definition->width = 128;  // target width
              definition->height = 128; // target height
              definition->format = Ogre::PF_R8G8B8;
           }
           {
              CompositionTargetPass* target = mTechnique->createTargetPass();
              target->setInputMode(CompositionTargetPass::IM_PREVIOUS);
              target->setOutputName("scene");
           }
           {
              CompositionTargetPass* output = mTechnique->getOutputTargetPass();
              output->setInputMode(CompositionTargetPass::IM_NONE);
              {
                 // Create a output pass
                 CompositionPass* pass = output->createPass();
                 pass->setType(CompositionPass::PT_RENDERQUAD);
                 pass->setMaterialName("Ogre/Compositor/OldTV");
                 pass->setInput(0, "scene");
              }
           }

        }*/

     //depthTarget->writeContentsToTimestampedFile("target", ".png");


     Ogre::TexturePtr depthTexture1 = Ogre::TextureManager::getSingleton().createManual(
 	                    "DepthMapTexture1", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
 	                    Ogre::TEX_TYPE_2D, mWindow->getWidth(), mWindow->getHeight(),
 	                            0,  Ogre::PF_BYTE_L, Ogre::TU_RENDERTARGET);

 	    // Get its render target and add a viewport to it
     Ogre::RenderTexture* depthTarget1 = depthTexture1->getBuffer()->getRenderTarget();
 	            //Ogre::Viewport* depthViewport = depthTarget->addViewport(mCamera, 40);
     Ogre::Viewport* depthViewport1 = depthTarget1->addViewport(mCamera);
     depthTarget1->getViewport(0)->setClearEveryFrame(true);
     //depthTarget1->getViewport(0)->setBackgroundColour(Ogre::ColourValue::White);
     depthTarget1->getViewport(0)->setOverlaysEnabled(false);

     Ogre::CompositorInstance* pCompositor = Ogre::CompositorManager::getSingleton().addCompositor(depthViewport1, "DepthGradNormMap");
     Ogre::CompositorManager::getSingleton().setCompositorEnabled(depthViewport1, "DepthGradNormMap", true);
     pCompositor->getTechnique()->getTargetPass(0)->setInputMode(Ogre::CompositionTargetPass::IM_PREVIOUS);



     /*Ogre::CompositorInstance* pCompositor = Ogre::CompositorManager::getSingleton().addCompositor(depthViewport1, "DepthGradNormMap");
          Ogre::CompositorManager::getSingleton().setCompositorEnabled(depthViewport1, "DepthGradNormMap", true);*/

     //Ogre::TexturePtr dynTexPtr = Ogre::TextureManager::getSingleton().getByName("rt0");
     double t0= vpTime::measureTimeMs();
     depthTarget1->update();



	            /*Ogre::Rectangle2D *mMiniScreen = new Ogre::Rectangle2D(true);
	             mMiniScreen->setCorners(0.5f, -0.5f, 1.0f, -1.0f);
	             mMiniScreen->setBoundingBox(Ogre::AxisAlignedBox(-100000.0f * Ogre::Vector3::UNIT_SCALE, 100000.0f * Ogre::Vector3::UNIT_SCALE));

	             Ogre::SceneNode* miniScreenNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("MiniScreenNode");
	             miniScreenNode->attachObject(mMiniScreen);


	             //Ogre::MaterialPtr renderMaterial = Ogre::MaterialManager::getSingleton().create("A330DepthMat", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);


	            mMiniScreen->setMaterial("DepthMapMat");*/

	             //mSceneMgr->_suppressRenderStateChanges(true);

     //depthTarget1->writeContentsToTimestampedFile("target1", ".png");



	    // Create a custom render queue invocation sequence for the depth render texture
	           /* Ogre::RenderQueueInvocationSequence* invocationSequence =
	                    Ogre::Root::getSingleton().createRenderQueueInvocationSequence("DepthMap");

	    // Add a render queue invocation to the sequence, and disable shadows for it
	            Ogre::RenderQueueInvocation* invocation = invocationSequence->add(Ogre::RENDER_QUEUE_MAIN, "main");
	            invocation->setSuppressShadows(true);

	    // Set the render queue invocation sequence for the depth render texture viewport
	            depthViewport->setRenderQueueInvocationSequenceName("DepthMap");
	            //Ogre::Viewport* depthViewport2->setRenderQueueInvocationSequenceName("DepthMap");

	            Ogre::CompositorManager::getSingleton().setCompositorEnabled(depthViewport, "DepthMap", true);*/

	            //mWindow->update();

	            //depthTarget1->writeContentsToTimestampedFile("target1", ".png");
	            //vpImage<vpRGBa> I1(480,640);
	            vpImage<unsigned char> I0(480,640);
	            vpImage<vpRGBa> I1(480,640);
                //getchar();
	            std::string texture = Ogre::CompositorManager::getSingleton().getCompositorChain(depthViewport1)->getCompositor(0)->getTextureInstanceName("rt2", 0);

	            //Ogre::TexturePtr dynTexPtr = pCompositor->getTextureInstance("rt0",0);
	            Ogre::TexturePtr dynTexPtr = Ogre::TextureManager::getSingleton().getByName(texture);
	            //Ogre::RenderTexture* rtt = dynTexPtr->getBuffer()->getRenderTarget();
	            //rtt->writeContentsToTimestampedFile("target2", ".png");
	            //depthTarget1->writeContentsToTimestampedFile("target3", ".png");

	            Ogre::TexturePtr dynTexPtr0 = Ogre::TextureManager::getSingleton().getByName("DepthMapTexture1");
	            Ogre::HardwarePixelBufferSharedPtr mPixelBuffer0 = dynTexPtr0->getBuffer();
	            Ogre::HardwarePixelBufferSharedPtr mPixelBuffer = dynTexPtr->getBuffer();
	            //Ogre::HardwarePixelBufferSharedPtr mPixelBuffer = depthTexture1->getBuffer();

	                // Inspired from Ogre wiki : http://www.ogre3d.org/tikiwiki/Creating+dynamic+textures
	                // Lock the pixel buffer and get a pixel box. HBL_DISCARD is to use for best
	                // performance than HBL_NORMAL
	                mPixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD); // Lock the buffer
	                mPixelBuffer0->lock(Ogre::HardwareBuffer::HBL_DISCARD);
	                const Ogre::PixelBox& pixelBox = mPixelBuffer->getCurrentLock();
	                const Ogre::PixelBox& pixelBox0 = mPixelBuffer0->getCurrentLock();

	                dynTexPtr0->getBuffer()->blitToMemory(pixelBox0);
	                dynTexPtr->getBuffer()->blitToMemory(pixelBox);
	                // Buffer data
	                Ogre::uint8* pDest0 = static_cast<Ogre::uint8*>(pixelBox0.data);
	                Ogre::uint32* pDest = static_cast<Ogre::uint32*>(pixelBox.data);
	                /*for (int n=100; n < 300 ; n++)
	                            {
	                            for (int m = 200 ; m < 400; m++)
	                              {
	                                //I1[n][m] =  (double)I2[n][m]/255;
	                            	std::cout << (double)pDest0[n+m*640] << std::endl;
	                            	std::cout << (double)pDest0[n+m*640+1] << std::endl;
	                            	std::cout << (double)pDest0[n+m*640+2] << std::endl;
	                              }
	                            }*/

	                /*int w=640;
	                int h=480;
	                int width=640;
					int height=480;*/
	                /*unsigned char* output;
	                output = (unsigned char*)I1.bitmap;*/
	                /*for(int i=0; i<300; i++)
	                  {
	                      //uint32 *data = static_cast<uint32*>(pb.data) + i*pb.rowPitch;
	                      for(int j=0; j<w; j++)
	                      {
	                         //data[(j + (i*w))] = iplImg->imageData[j+(i*w)];

	                //this:
	                    	  *(output++) = (unsigned char)pDest[(j+(i*w))+1];
	                    	  *(output++)  = (unsigned char)pDest[(j+(i*w))+2];
	                    	  *(output++)  = (unsigned char)pDest[(j+(i*w))+3];
	                    	  *(output++)  = (unsigned char)pDest[(j+(i*w))];
	                    	/* I1[i][j].R=pDest[(j+(i*w))+1];
	                    	  I1[i][j].G=pDest[(j+(i*w))+2];
	                    	  I1[i][j].B=pDest[(j+(i*w))+3];
	                    	  I1[i][j].A=pDest[(j+(i*w))];
	                    	  //std::cout<<pDest[(j+(i*w))+1]<<std::endl;
	                    	  std::cout<<(double)I1[i][j].R<<" "<<(double)I1[i][j].G<<" "<<(double)I1[i][j].B<<std::endl;
	                //or: (this one is probably faster)
	                                        //memcpy(data+((j+(i*w))*4), ipllmg->imageData+((j+(i*w))*4), 4);*/
	                    /* }
	                   }*/

	                // Fill in the data in the grey level texture

	                memcpy(I0.bitmap, pDest0, mWindow->getHeight()*mWindow->getWidth());
	                memcpy(I1.bitmap, pDest, mWindow->getHeight()*mWindow->getWidth()*4);

	                // Unlock the pixel buffer
	                mPixelBuffer->unlock();
	                mPixelBuffer0->unlock();
	                double t1= vpTime::measureTimeMs();

	                std::cout<<" time "<<t1-t0<<std::endl;

	            	            /*char buf[FILENAME_MAX];
							    sprintf(buf, opath.c_str(), im);
								std::string filename(buf);
								std::cout << "Write: " << filename << std::endl;*/
								vpImageIo::writePPM(I0, "texture0.pgm");
								vpImageIo::writePPM(I1, "texture1.pgm");


}


/*void vpAROgre::rtt2(vpImage<unsigned char> &I, vpHomogeneousMatrix *CMo)
{
	updateCameraParameters(*CMo);
    //Ogre::TexturePtr depthTarget;
    //Ogre::RenderTexture *depthTexture;

	    // Create the depth render texture

	TexturePtr texPtr = TextureManager::getSingleton().createManual("DepthMap", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, TEX_TYPE_2D, 512, 512, 0, PF_R8G8B8, TU_RENDERTARGET);

	    // Get its render target and add a viewport to it
    Ogre::RenderTarget* depthTarget = depthTexture->getBuffer()->getRenderTarget();
	            //Ogre::Viewport* depthViewport = depthTarget->addViewport(mCamera, 40);
    Ogre::Viewport* depthViewport = depthTarget->addViewport(mCamera);
    depthTarget->getViewport(0)->setClearEveryFrame(true);
    depthTarget->getViewport(0)->setBackgroundColour(Ogre::ColourValue::Black);
    depthTarget->getViewport(0)->setOverlaysEnabled(false);

    mSceneMgr->setMarerial()

    Ogre::Rectangle2D *mMiniScreen = new Ogre::Rectangle2D(true);
     mMiniScreen->setCorners(0.5f, -0.5f, 1.0f, -1.0f);
     mMiniScreen->setBoundingBox(Ogre::AxisAlignedBox(-100000.0f * Ogre::Vector3::UNIT_SCALE, 100000.0f * Ogre::Vector3::UNIT_SCALE));

     Ogre::SceneNode* miniScreenNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("MiniScreenNode");
     miniScreenNode->attachObject(mMiniScreen);


     //Ogre::MaterialPtr renderMaterial = Ogre::MaterialManager::getSingleton().create("A330DepthMat", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);


     mMiniScreen->setMaterial("DepthMap2");



    depthTarget->update();*/



	           // depthViewport->setBackgroundColour(Ogre::ColourValue::Black);

	    /*// Register 'this' as a render target listener
	            depthTarget->addListener(this->listener);*/

	    // Get the technique to use when rendering the depth render texture

	           /* Ogre::MaterialPtr mDepthMaterial = Ogre::MaterialManager::getSingleton().getByName("DepthMapMat");
	             Ogre::Technique* matTechnique = mDepthMaterial->createTechnique();
	             matTechnique->createPass();
	             mDepthMaterial->getTechnique(0)->getPass(0)->setLightingEnabled(false);
	             mDepthMaterial->getTechnique(0)->getPass(0)->createTextureUnitState("DepthMap");
	            //mDepthMaterial->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTextureName("A330DepthMap");
	            mDepthMaterial->compile();
	            mDepthMaterial->load(); // needs to be loaded manually
	            Ogre::Technique* mDepthTechnique = mDepthMaterial->getBestTechnique();*/

	    // Create a custom render queue invocation sequence for the depth render texture
	           /* Ogre::RenderQueueInvocationSequence* invocationSequence =
	                    Ogre::Root::getSingleton().createRenderQueueInvocationSequence("DepthMap");

	    // Add a render queue invocation to the sequence, and disable shadows for it
	            Ogre::RenderQueueInvocation* invocation = invocationSequence->add(Ogre::RENDER_QUEUE_MAIN, "main");
	            invocation->setSuppressShadows(true);

	    // Set the render queue invocation sequence for the depth render texture viewport
	            depthViewport->setRenderQueueInvocationSequenceName("DepthMap");
	            //Ogre::Viewport* depthViewport2->setRenderQueueInvocationSequenceName("DepthMap");

	            Ogre::CompositorManager::getSingleton().setCompositorEnabled(depthViewport, "DepthMap", true);*/






	            /*mWindow->update();



	            //depthTarget->writeContentsToTimestampedFile("target", ".png");
	            getchar();

	        	double t0= vpTime::measureTimeMs();
	            // Pointer to the dynamic texture
	            //Ogre::TexturePtr dynTexPtr = Ogre::TextureManager::getSingleton().getByName("A330DepthMap");

	            // Get the pixel buffer
	            Ogre::HardwarePixelBufferSharedPtr mPixelBuffer = depthTexture->getBuffer();

	            // Inspired from Ogre wiki : http://www.ogre3d.org/tikiwiki/Creating+dynamic+textures
	            // Lock the pixel buffer and get a pixel box. HBL_DISCARD is to use for best
	            // performance than HBL_NORMAL
	            mPixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD); // Lock the buffer
	            const Ogre::PixelBox& pixelBox = mPixelBuffer->getCurrentLock();
	            depthTexture->getBuffer()->blitToMemory(pixelBox);
	            // Buffer data
	            Ogre::uint8* pDest = static_cast<Ogre::uint8*>(pixelBox.data);
	            // Fill in the data in the grey level texture

	            memcpy(I.bitmap, pDest, mWindow->getHeight()*mWindow->getWidth());
	            // Unlock the pixel buffer
	            mPixelBuffer->unlock();
	            double t1= vpTime::measureTimeMs();

	            std::cout << "time  "<< t1-t0<<std::endl;


}*/


void vpAROgre::writeFile(const std::string path){
    //depthTarget->update();
    //depthTarget->writeContentsToFile(path);
}

void vpAROgre::copyToImage(vpImage<unsigned char> &I){

/*    //depthTarget->update();

    // Pointer to the dynamic texture
    Ogre::TexturePtr dynTexPtr = Ogre::TextureManager::getSingleton().getByName("A330DepthMap");

    // Get the pixel buffer
    Ogre::HardwarePixelBufferSharedPtr mPixelBuffer = dynTexPtr->getBuffer();

    // Inspired from Ogre wiki : http://www.ogre3d.org/tikiwiki/Creating+dynamic+textures
    // Lock the pixel buffer and get a pixel box. HBL_DISCARD is to use for best
    // performance than HBL_NORMAL
    mPixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD); // Lock the buffer
    const Ogre::PixelBox& pixelBox = mPixelBuffer->getCurrentLock();
    depthTarget->getBuffer()->blitToMemory(pixelBox);
    // Buffer data
    Ogre::uint8* pDest = static_cast<Ogre::uint8*>(pixelBox.data);
    // Fill in the data in the grey level texture
    memcpy(I.bitmap, pDest, mWindow->getHeight()*mWindow->getWidth());
    // Unlock the pixel buffer
    mPixelBuffer->unlock();*/
}





