#include "ctOgre.h"
#include <visp/vpImageConvert.h>
//#include <GL/gl.h>

/**
* Constructor
* \param grab : Framegrabber to get background images
* \param type : Either BACKGROUND_GREY for grey level image in background or BACKGROUND_COLOR for a RGBa one
* \param width : Width of the grabbed image
* \param height : Height of the grabbed image
* \param resourcepath : Path to the resources.cfg file telling Ogre where to look for resources
*/


ctOgre::ctOgre(const vpCameraParameters &cameraP, unsigned int width, unsigned int height, const char *resourcePath, const char *pluginPath)
        : vpAROgre(cameraP, width, height, resourcePath, pluginPath),poseTransf()
{


	BackgroundT=BACKGROUND_GREY;

	Z_max = 1.0f;
	IZ.resize(height, width);

	Z_near = 1.0f; //0.1f;
	Z_far = 500.0f;//0.6f;

	compute_Z = true;
}

ctOgre::~ctOgre( void)
{
}

/**
* Load a mesh in the 3D world
* \param name : Name of the Entity and SceneNode to create
* \param model : 3D model to load
*/
void ctOgre::load(std::string name, std::string model)
{
	Ogre::Entity *newEntity = mSceneMgr->createEntity(name, model);
	//newEntity->getSubEntity(0)->getMaterial()->getBestTechnique()->getPass(0)->setLightingEnabled(true);
    //newEntity->setCastShadows(true);

//    newEntity->setMaterialName("Texture");
    //setShadersMultiRTT(name);

	newEntity->setMaterialName("myTest");
	//newEntity->setMaterialName("myDepth");
	//newEntity->setMaterialName("Ogre/DepthShadowmap/Caster/Float");
    //newEntity->setMaterialName("NormalMap");


	newEntity->getSubEntity(0)->getMaterial()->getBestTechnique()->setCullingMode(Ogre::CULL_NONE);

	Ogre::SceneNode *newNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(name);
	newNode->attachObject(newEntity);
	//Render Texture depth and texture
	createDepthRenderTexture();
}

void ctOgre::init(bool bufferedKeys)
{
  vpAROgre::init(bufferedKeys);

  Ogre::ColourValue colval = Ogre::ColourValue(0.5, 0.5, 0.5);


  Ogre::Light* directionalLight0 =
    mSceneMgr->createLight("directionalLight0");
         directionalLight0->setType(Ogre::Light::LT_DIRECTIONAL);
         directionalLight0->setDiffuseColour(colval);
         directionalLight0->setSpecularColour(colval);
         directionalLight0->setDirection(Ogre::Vector3( 1, 0, 0 ));

  Ogre::Light* directionalLight1 =
    mSceneMgr->createLight("directionalLight1");
         directionalLight1->setType(Ogre::Light::LT_DIRECTIONAL);
         directionalLight1->setDiffuseColour(colval);
         directionalLight1->setSpecularColour(colval);
         directionalLight1->setDirection(Ogre::Vector3( -1, 0, 0 ));

  Ogre::Light* directionalLight2 =
    mSceneMgr->createLight("directionalLight2");
         directionalLight2->setType(Ogre::Light::LT_DIRECTIONAL);
         directionalLight2->setDiffuseColour(colval);
         directionalLight2->setSpecularColour(colval);
         directionalLight2->setDirection(Ogre::Vector3( 0, 1, 0 ));

  Ogre::Light* directionalLight3 =
    mSceneMgr->createLight("directionalLight3");
         directionalLight3->setType(Ogre::Light::LT_DIRECTIONAL);
         directionalLight3->setDiffuseColour(colval);
         directionalLight3->setSpecularColour(colval);
         directionalLight3->setDirection(Ogre::Vector3( 0, -1, 0 ));

  Ogre::Light* directionalLight4 =
    mSceneMgr->createLight("directionalLight4");
         directionalLight4->setType(Ogre::Light::LT_DIRECTIONAL);
         directionalLight4->setDiffuseColour(colval);
         directionalLight4->setSpecularColour(colval);
         directionalLight4->setDirection(Ogre::Vector3( 0, 0, 1 ));

  Ogre::Light* directionalLight5 =
    mSceneMgr->createLight("directionalLight5");
         directionalLight5->setType(Ogre::Light::LT_DIRECTIONAL);
         directionalLight5->setDiffuseColour(colval);
         directionalLight5->setSpecularColour(colval);
         directionalLight5->setDirection(Ogre::Vector3( 0, 0, -1 ));


    mSceneMgr->setAmbientLight(Ogre::ColourValue(0, 0, 0));
    mSceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);

//  //Render Texture depth and texture
//    createDepthRenderTexture();

}

void ctOgre::setShadersMultiRTT(std::string name)
{

    Ogre::Entity *newEntity = mSceneMgr->getEntity(name);

    Ogre::HighLevelGpuProgramPtr vertexShader = Ogre::HighLevelGpuProgramManager::getSingleton().createProgram("TexProjectionV",
                                                                                                                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                                                                "glsl",
                                                                                                                Ogre::GPT_VERTEX_PROGRAM);

    //Code GLSL du vertex shader
    Ogre::String vertSrc;
    vertSrc += "void main(void)\n";
    vertSrc += "{\n";
    vertSrc += "gl_Position = ftransform();\n";
    vertSrc += "gl_TexCoord[0] = gl_MultiTexCoord0;\n";
    vertSrc += "}\n";

    //Affectation et chargement du code pour le vertex shader
    vertexShader->setSource(vertSrc);
    vertexShader->load();

    //**************

    //Fragment shader (mêmes commentaires que ci-dessus)
    Ogre::HighLevelGpuProgramPtr fragmentShader = Ogre::HighLevelGpuProgramManager::getSingleton().createProgram("TexProjectionF",
                                                                                                                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                                                                "glsl",
                                                                                                                Ogre::GPT_FRAGMENT_PROGRAM);

    Ogre::String fragSrc;
    //fragSrc = "uniform sampler2D myTex; \n void main(void)\n { \n vec4 fcol = texture2D(myTex, gl_TexCoord[0].xy); \n float lum = 0.1*(fcol.x) + 0.3*(fcol.y) + 0.6*(fcol.z); \n gl_FragCoord = vec4(lum,lum,lum,1);\n }\n";
    fragSrc = "uniform sampler2D myTex; \n void main(void)\n { \n gl_FragColor = texture2D(myTex, gl_TexCoord[0].xy);\n }\n";
    fragmentShader->setSource(fragSrc);
    fragmentShader->load();

    //******************************

    Ogre::HighLevelGpuProgramPtr vertexShader1 = Ogre::HighLevelGpuProgramManager::getSingleton().createProgram("DepthMapV",
                                                                                                                    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                                                                    "glsl",
                                                                                                                    Ogre::GPT_VERTEX_PROGRAM);


    //Code GLSL du vertex shader
	Ogre::String vertSrc1;
	//vertSrc1 += "varying vec3 oNormal;\n";
	vertSrc1 += "void main(void)\n";
	vertSrc1 += "{\n";
	vertSrc1 += "gl_Position = ftransform();\n";
	//vertSrc += "gl_TexCoord[0] = gl_MultiTexCoord0;\n";
	//vertSrc1 += "oNormal = gl_Normal;\n";
	vertSrc1 += "}\n";

	//Affectation et chargement du code pour le vertex shader
	vertexShader1->setSource(vertSrc1);
	vertexShader1->load();

	//**************

	//Fragment shader (mêmes commentaires que ci-dessus)
	Ogre::HighLevelGpuProgramPtr fragmentShader1 = Ogre::HighLevelGpuProgramManager::getSingleton().createProgram("DepthMapF",
																												Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
																												"glsl",
																												Ogre::GPT_FRAGMENT_PROGRAM);

	Ogre::String fragSrc1;
	//fragSrc1 = "void main(void)\n {\n float fDepth = gl_FragCoord.z; \n gl_FragColor = vec4(0.2, 50, 0.2,fDepth); \n }\n";

	fragSrc1 = "void main(void)\n {\n float fDepth = gl_FragCoord.z; \n gl_FragColor = vec4(fDepth, fDepth, fDepth,1); \n }\n";
	//fragSrc1 = "void main(void)\n {\n float fDepth = gl_FragCoord.z; \n gl_FragColor = vec4(20, 20, 20,1); \n }\n";
	//fragSrc1 = "uniform sampler2D myTex; \n void main(void)\n { \n gl_FragColor = texture2D(myTex, gl_TexCoord[0].xy);\n }\n";
	//fragSrc1 = "uniform sampler2D myTex; \n void main(void)\n { \n vec4 tmp0 = texture2D(myTex, gl_TexCoord[0].st); \n gl_FragColor = vec4(tmp0.z, tmp0.y, tmp0.x, 1);\n }\n";
	//fragSrc1 = "uniform sampler2D myTex; \n void main(void)\n { \n vec4 tmp0 = texture2D(myTex, gl_TexCoord[0].st); \n gl_FragColor = vec4(0.2, 20, 0.2, 1);\n }\n";

	fragmentShader1->setSource(fragSrc1);
	fragmentShader1->load();

    //*****************************************

    //Affectation du vertex shader et du fragment shader aux passes des techniques du material de chaque sous entité
    //Et passage d'un paramètre float au vertex shader
    //et du paramètre pour le sampler 2D de texture pour le fragment shader
    for(unsigned int j = 0 ; j < newEntity->getNumSubEntities() ; j++)
    {
        for(unsigned int k = 0 ; k < newEntity->getSubEntity(j)->getMaterial()->getNumTechniques() ; k++){
            for(unsigned int i = 0 ; i < newEntity->getSubEntity(j)->getMaterial()->getTechnique(k)->getNumPasses() ; i++)
            {
                newEntity->getSubEntity(j)->getMaterial()->getTechnique(k)->setSchemeName("MRT");

            	/*newEntity->getSubEntity(j)->getMaterial()->getTechnique(k)->getPass(i)->setVertexProgram("TexProjectionV");
                Ogre::GpuProgramParametersSharedPtr vParams = newEntity->getSubEntity(j)->getMaterial()->getTechnique(k)->getPass(i)->getVertexProgramParameters();
                //vParams->setNamedConstant("xi", xi);
                std::cout << " ok " << std::endl;
                newEntity->getSubEntity(j)->getMaterial()->getTechnique(k)->getPass(i)->setCullingMode(Ogre::CULL_NONE);

                newEntity->getSubEntity(j)->getMaterial()->getTechnique(k)->getPass(i)->setFragmentProgram("TexProjectionF");
                Ogre::GpuProgramParametersSharedPtr fParams = newEntity->getSubEntity(j)->getMaterial()->getTechnique(k)->getPass(i)->getFragmentProgramParameters();
                //fParams->setNamedConstant("myTex", 0);*/
            }
        }
		Ogre::Technique* tech = newEntity->getSubEntity(j)->getMaterial()->createTechnique();
		tech->setSchemeName("DepthMap");
		Ogre::Pass* pass = tech->createPass();
		pass->setVertexProgram("DepthMapV");
		pass->setCullingMode(Ogre::CULL_NONE);
		pass->setFragmentProgram("DepthMapF");
    }
}



void ctOgre::setClipDistances(float _Z_near, float _Z_far)
{
  Z_near = _Z_near;
  Z_far = _Z_far;

  updateCameraProjection();
}

/**
* Update the projection parameters of the camera
*/
void ctOgre::updateCameraProjection(void)
{

  Ogre::Real f,n,f_m_n,f_p_n,px,py,u0,v0;
  f = (Ogre::Real)Z_far; // Far clip distance
  n = (Ogre::Real)Z_near; // Near clip distance
  f_m_n = (Ogre::Real)(f-n);
  f_p_n = (Ogre::Real)(f+n);
  px = (Ogre::Real)mcam.get_px();
  py = (Ogre::Real)mcam.get_py();
  u0 = (Ogre::Real)mcam.get_u0();
  v0 = (Ogre::Real)mcam.get_v0();
  Ogre::Matrix4 Projection
    = Ogre::Matrix4( (Ogre::Real)(2.0*px/mWidth), 0,  (Ogre::Real)(2.0*(u0/mWidth)-1.0), 0,
		     0, (Ogre::Real)(2.0*py/mHeight), (Ogre::Real)(2.0*(v0/mHeight)-1.0),0,
		     0, 0, (Ogre::Real)(-1.0*f_p_n/f_m_n), (Ogre::Real)(-2.0*f*n/f_m_n),
		     0, 0, -1.0, 0);
  mCamera->setCustomProjectionMatrix(true, Projection);
}


void ctOgre::createDepthRenderTexture()
{
//    Ogre::TexturePtr Texture0 = Ogre::TextureManager::getSingleton().createManual(
//	                    "ColorTexture", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
//	                    Ogre::TEX_TYPE_2D, mWindow->getWidth(), mWindow->getHeight(),
//	                            0,  Ogre::PF_R8G8B8A8, Ogre::TU_RENDERTARGET);
//    Ogre::RenderTexture* RTarget0 = Texture0->getBuffer()->getRenderTarget();
//    Ogre::Viewport* Viewport0 = RTarget0->addViewport(mCamera);
//    Viewport0->setMaterialScheme("MRT");
//
//    RTarget0->getViewport(0)->setClearEveryFrame(true);
//    RTarget0->getViewport(0)->setOverlaysEnabled(false);

    //***************
    Ogre::TexturePtr Texture1 = Ogre::TextureManager::getSingleton().createManual(
	                    "DepthMapTexture", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
	                    Ogre::TEX_TYPE_2D, mWindow->getWidth(), mWindow->getHeight(),
	                            0,  Ogre::PF_BYTE_L, Ogre::TU_RENDERTARGET);
    Ogre::RenderTexture* RTarget1 = Texture1->getBuffer()->getRenderTarget();
    Ogre::Viewport* Viewport1 = RTarget1->addViewport(mCamera);

    //Viewport1->setMaterialScheme("DepthMap");
    //Viewport1->setMaterialScheme("myDepth");

    RTarget1->getViewport(0)->setClearEveryFrame(true);
    //RTarget1->getViewport(0)->setBackgroundColour(Ogre::ColourValue::White);
    RTarget1->getViewport(0)->setOverlaysEnabled(false);

}



void ctOgre::getCurrentImages(vpImage<vpRGBa> &I0, vpImage<unsigned char> &I1, vpHomogeneousMatrix &CMo)
{
	//Render Textures
	updateCameraParameters(CMo);
	Ogre::TexturePtr dynTexPtr0 = Ogre::TextureManager::getSingleton().getByName("ColorTexture");
	Ogre::TexturePtr dynTexPtr1 = Ogre::TextureManager::getSingleton().getByName("DepthMapTexture");
	Ogre::RenderTexture* RTarget0 = dynTexPtr0->getBuffer()->getRenderTarget();
	Ogre::RenderTexture* RTarget1 = dynTexPtr1->getBuffer()->getRenderTarget();
	RTarget0->update();
	RTarget1->update();
	//mWindow->update();

	//Get image of colorTexture
	Ogre::HardwarePixelBufferSharedPtr mPixelBuffer0 = dynTexPtr0->getBuffer();
	mPixelBuffer0->lock(Ogre::HardwareBuffer::HBL_DISCARD);
	const Ogre::PixelBox& pixelBox0 = mPixelBuffer0->getCurrentLock();
	dynTexPtr0->getBuffer()->blitToMemory(pixelBox0);
	Ogre::uint32* pDest0 = static_cast<Ogre::uint32*>(pixelBox0.data);
	memcpy(I0.bitmap, pDest0, mWindow->getHeight()*mWindow->getWidth()*4);
	// Unlock the pixel buffer
	mPixelBuffer0->unlock();

	//Get image of DepthMap
	Ogre::HardwarePixelBufferSharedPtr mPixelBuffer1 = dynTexPtr1->getBuffer();
	mPixelBuffer1->lock(Ogre::HardwareBuffer::HBL_DISCARD);
	const Ogre::PixelBox& pixelBox1 = mPixelBuffer1->getCurrentLock();
	dynTexPtr1->getBuffer()->blitToMemory(pixelBox1);
	Ogre::uint8* pDest1 = static_cast<Ogre::uint8*>(pixelBox1.data);
	memcpy(I1.bitmap, pDest1, mWindow->getHeight()*mWindow->getWidth());
	// Unlock the pixel buffer
	mPixelBuffer1->unlock();
}

void ctOgre::testGetDepthMap(vpImage<unsigned char> &I, vpHomogeneousMatrix &CMo)
{
	//Render Textures
	updateCameraParameters(CMo);
	Ogre::TexturePtr dynTexPtr1 = Ogre::TextureManager::getSingleton().getByName("DepthMapTexture");
	Ogre::RenderTexture* RTarget1 = dynTexPtr1->getBuffer()->getRenderTarget();
	mWindow->update();
	RTarget1->update();

	//Get image of DepthMap
	Ogre::HardwarePixelBufferSharedPtr mPixelBuffer1 = dynTexPtr1->getBuffer();
	mPixelBuffer1->lock(Ogre::HardwareBuffer::HBL_DISCARD);
	const Ogre::PixelBox& pixelBox1 = mPixelBuffer1->getCurrentLock();
	dynTexPtr1->getBuffer()->blitToMemory(pixelBox1);
	Ogre::uint8* pDest1 = static_cast<Ogre::uint8*>(pixelBox1.data);
	//Ogre::uint32* pDest1 = static_cast<Ogre::uint32*>(pixelBox1.data);
	memcpy(I.bitmap, pDest1, mWindow->getHeight()*mWindow->getWidth());
	// Unlock the pixel buffer
	mPixelBuffer1->unlock();


}

//void ctOgre::testGetRenderTexture(vpImage<vpRGBa> &I, vpHomogeneousMatrix &CMo)
//{
//	//Render Textures
//	updateCameraParameters(CMo);
//	Ogre::TexturePtr dynTexPtr1 = Ogre::TextureManager::getSingleton().getByName("DepthMapTexture");
//	Ogre::RenderTexture* RTarget1 = dynTexPtr1->getBuffer()->getRenderTarget();
//
//	RTarget1->update();
//
//	//Get image of DepthMap
//	Ogre::HardwarePixelBufferSharedPtr mPixelBuffer1 = dynTexPtr1->getBuffer();
//	mPixelBuffer1->lock(Ogre::HardwareBuffer::HBL_DISCARD);
//	const Ogre::PixelBox& pixelBox1 = mPixelBuffer1->getCurrentLock();
//	dynTexPtr1->getBuffer()->blitToMemory(pixelBox1);
//	Ogre::uint8* pDest1 = static_cast<Ogre::uint8*>(pixelBox1.data);
//	memcpy(I.bitmap, pDest1, mWindow->getHeight()*mWindow->getWidth());
//	// Unlock the pixel buffer
//	mPixelBuffer1->unlock();
//}

bool ctOgre::processInputEvent(const Ogre::FrameEvent& /*evt*/) {
    mKeyboard->capture();
    if(mKeyboard->isKeyDown(OIS::KC_ESCAPE))
      return false;

    poseTransf.setIdentity();
    double t = 0.15, rr = M_PI/50.0;
    // Event telling that we will have to move, setting the animation to "walk", if false, animation goes to "Idle"
    bool event = false;
    // Check entries
    if(mKeyboard->isKeyDown(OIS::KC_Z)){
    	std::cout << "Zoom in" << std::endl;
    	poseTransf.buildFrom(0.0, 0.0, -t, 0.0, 0.0, 0.0);
      event = true;
    }
    if(mKeyboard->isKeyDown(OIS::KC_S)){
    	std::cout << "Zoom out" << std::endl;
    	poseTransf.buildFrom(0.0, 0.0, t, 0.0, 0.0, 0.0);
      event = true;
    }
//    if(mKeyboard->isKeyDown(OIS::KC_F)){
//    	std::cout << "Vertical Flip" << std::endl;
//    	verticalFlip(mImage);
//    	verticalFlip(mImageRGBA);
//      event = true;
//    }
    if(mKeyboard->isKeyDown(OIS::KC_LEFT)){
    	std::cout << "Move Left" << std::endl;
    	poseTransf.buildFrom(-t,0.0, 0.0, 0.0, 0.0, 0.0);
      event = true;
    }
    if(mKeyboard->isKeyDown(OIS::KC_RIGHT)){
    	std::cout << "Move Right" << std::endl;
    	poseTransf.buildFrom(t,0.0 , 0.0, 0.0, 0.0, 0.0);
      event = true;
    }
    if(mKeyboard->isKeyDown(OIS::KC_UP)){
    	std::cout << "Move Up" << std::endl;
    	poseTransf.buildFrom(0.0, -t, 0.0, 0.0, 0.0, 0.0);
      event = true;
    }
    if(mKeyboard->isKeyDown(OIS::KC_DOWN)){
    	std::cout << "Move Down" << std::endl;
    	poseTransf.buildFrom(0.0, t, 0.0, 0.0, 0.0, 0.0);
      event = true;
    }

    // Play the right animation
    if(event){
      //mAnimationState = robot->getAnimationState("Walk");
    }
    //else mAnimationState = robot->getAnimationState( "Idle" );

    return true;
  }

//
void ctOgre::verticalFlip(vpImage<unsigned char> & I)
{
  unsigned int larg = I.getWidth(), haut = I.getHeight();
  unsigned char *lig = new unsigned char[larg], *ptr_I_src, *ptr_I_dst;
  
  for(unsigned int l = 0 ; l < haut/2; l++)
  {
    ptr_I_src = I.bitmap+(larg*(haut-(l+1)));
    ptr_I_dst = I.bitmap+(larg*l);
    
    //sauvegarde de ligne
    memcpy(lig, ptr_I_dst, larg*sizeof(unsigned char));
    //on remplit cette ligne avec le bas de l'image
    memcpy(ptr_I_dst, ptr_I_src, larg*sizeof(unsigned char));
    //on remplit le bas de l'image avec ce qui a été sauvegardé dans lig
    memcpy(ptr_I_src, lig, larg*sizeof(unsigned char));
  }
  
  delete [] lig;
}

void ctOgre::verticalFlip(vpImage<vpRGBa> & I)
{
  unsigned int larg = I.getWidth(), haut = I.getHeight();
  vpRGBa *lig = new vpRGBa[larg], *ptr_I_src, *ptr_I_dst;
  
  for(unsigned int l = 0 ; l < haut/2; l++)
  {
    ptr_I_src = I.bitmap+(larg*(haut-(l+1)));
    ptr_I_dst = I.bitmap+(larg*l);
    
    //sauvegarde de ligne
    memcpy(lig, ptr_I_dst, larg*sizeof(vpRGBa));
    //on remplit cette ligne avec le bas de l'image
    memcpy(ptr_I_dst, ptr_I_src, larg*sizeof(vpRGBa));
    //on remplit le bas de l'image avec ce qui a été sauvegardé dans lig
    memcpy(ptr_I_src, lig, larg*sizeof(vpRGBa));
  }
  
  delete [] lig;
}

void ctOgre::verticalFlip(vpImage<float> & I)
{
  unsigned int larg = I.getWidth(), haut = I.getHeight();
  float *lig = new float[larg], *ptr_I_src, *ptr_I_dst;
  
  for(unsigned int l = 0 ; l < haut/2; l++)
  {
    ptr_I_src = I.bitmap+(larg*(haut-l-1));
    ptr_I_dst = I.bitmap+(larg*l);
    
    //sauvegarde de ligne
    memcpy(lig, ptr_I_dst, larg*sizeof(float));
    //on remplit cette ligne avec le bas de l'image
    memcpy(ptr_I_dst, ptr_I_src, larg*sizeof(float));
    //on remplit le bas de l'image avec ce qui a été sauvegardé dans lig
    memcpy(ptr_I_src, lig, larg*sizeof(float));
  }
  
  delete [] lig;
}

/**
* Display a frame
* \param srcI : Grey level image to show in background
* \param CMo : Camera parameters
*/
void ctOgre::displayLoc(vpImage<unsigned char> &srcI, vpHomogeneousMatrix& CMo)
{
	// Update the background to match the situation
	//updateBackgroundTexture(srcI);

	// Update the camera parameters to match the grabbed image
	updateCameraParameters(CMo);

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
void ctOgre::displayLoc(vpImage<vpRGBa> &srcI, vpHomogeneousMatrix &CMo)
{
	// Update the background to match the situation
	//updateBackgroundTexture(srcI);

	// Update the camera parameters to match the grabbed image
	updateCameraParameters(CMo);

	// Display on Ogre Window
	if(mRoot->renderOneFrame()){
		mWindow->update();
		keepOn = true;
	}
	else
		keepOn = false;
}

