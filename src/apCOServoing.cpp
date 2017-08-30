/*!
 * ======================================================
 *
 *
 * DESCRIPTION: Main program demonstrating 3D tracking using 3D rendering
 *
 *
 *
 * \authors Antoine Petit
 * \date 05/06/11
 *======================================================
 */

#include <QApplication>
#include "surrender/scenemanager.h"
#include "surrender/sceneviewer.h"

#include <visp/vpOpenCVGrabber.h>
#include <visp/vpVideoReader.h>
#include <cstdlib>
#include <visp/vpDebug.h>
#include <visp/vpConfig.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpPose.h>
#include <visp/vpPoint.h>
#include <visp/vpImagePoint.h>
#include <visp/vpDot2.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpImageConvert.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpVideoWriter.h>
#include <visp/vpRobotAfma6.h>
#include <visp/vp1394TwoGrabber.h>
#include <visp/vpServo.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpGenericFeature.h>
#include <visp/vpFeatureBuilder.h>

#include <fstream>
#include <math.h>
#include <string.h>
#include <highgui.h>


#include <visp/vpException.h>
#include <visp/vpImageException.h>
#include <visp/vpRGBa.h>
#include <visp/vpIoTools.h>
#include <visp/vpImageIo.h>
#include <visp/vpParseArgv.h>
#include <visp/vpImage.h>
#include <visp/vpImageFilter.h>
#include <visp/vpImageConvert.h>
#include <visp/vpTime.h>
#include <visp/vpFeatureLuminance.h>
#include <visp/vpPlot.h>

#ifdef True
#undef True
#undef False
#endif

#include <cv.h>


#include "apMbTracker.h"
#include "apDetector.h"
#include "apViews.h"
#include "apSegmentation.h"
#include "apKalmanFilter.h"
#include "apKalmanFilterSE3.h"
#include "apKalmanFilterSE33.h"
#include "apKalmanFilterQuat.h"


#include <iostream>
#include <sstream>

#include "luaconfig.h"

using namespace std;
using namespace cv;
#define USE_USB_CAMERA 0

using namespace luxifer;

#define GETOPTARGS  "o:i:c:s:l:m:I:dh"


void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Example of tracking based on the 3D model.\n\
\n\
SYNOPSIS\n\
  %s [-o <object name>]\n\
 [-i <image path>] \n\
 [-s <first image>] \n\
 [-c <config file>] \n\
 [-l <learn object>] \n\
 [-m <detect object>] \n\
 [-d] [-h]",
  name );

  fprintf(stdout, "\n\
OPTIONS:                                               \n\
  -o <object name>                                 \n\
     Specify the name of object or scene to track\n\
\n\
  -i <input image path>                                \n\
     Set image input path.\n\
\n\
  -s <first image>                                \n\
     Set the first image of the sequence.\n\
 \n\
  -c <config file>                                \n\
     Set the config file to use (Lua script).\n\
\n\
  -l \n\
     Learn the object for automatic initialization.\n\
\n\
  -m \n\
     Detect the object for automatic initialization.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -I <initial pose as inline 4x4 matrix>\n\
     Explicitly set initial pose.\n\
\n\
  -h \n\
     Print the help.\n\n");

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

bool getOptions(int argc, char **argv,
                std::string &object,
                std::string &ipath,
                bool &display,
                int &start_image,
                std::string &configfile,
                bool &learn,
                bool &detect,
                std::vector<double> &inline_init)
{
  const char *optarg;
  const char **argv1=(const char**)argv;
  int   c;
  while ((c = vpParseArgv::parse(argc, argv1, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'o': object = optarg; break;
    case 'i': ipath = optarg; break;
    case 'd': display = false; break;
    case 'c': configfile = optarg; break;
    case 's': start_image = atoi(optarg);   break;
    case 'l': learn = optarg;   break;
    case 'm': detect = optarg;   break;
    case 'h': usage(argv[0], NULL); return false; break;
    case 'I':
        inline_init.resize(16);
        std::stringstream(std::string(optarg))
                >> inline_init[0] >> inline_init[1] >> inline_init[2] >> inline_init[3]
                >> inline_init[4] >> inline_init[5] >> inline_init[6] >> inline_init[7]
                >> inline_init[8] >> inline_init[9] >> inline_init[10] >> inline_init[11]
                >> inline_init[12] >> inline_init[13] >> inline_init[14] >> inline_init[15];
        break;

    default:
      usage(argv[0], optarg);
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}


int main(int argc, char **argv)
{
    QApplication a(argc, argv);

    // Paths and file names
    std::string env_ipath;
    std::string env_ipath2;
    std::string opath;
    std::string vpath;
    std::string opt_ipath;
    std::string ipath;
    std::string isegpath;
    std::string scpath;
    std::string configFile;
    std::string configFile2;
    std::string s_path;
    std::string opt_object;
    std::string object;
    std::string initFile;
    std::string modelFile;
    std::string gdtpath;
    std::string trueposepath;
    std::string posepath;
    std::vector<double> inline_init;
//    std::string truthpath;
    bool opt_display;
    bool opt_learn = false;
    bool opt_detect = false;
    bool displayMovingEdge = true;
    bool opt_click_allowed = true;
    int start_image = 0;

    env_ipath="/local/agpetit/images";
    env_ipath2="/local/agpetit";
    s_path = "/local/agpetit/soft/apTracking/mbt-co/build/";
    gdtpath = "/local/agpetit/soft/apTracking/mbt-co/build/ADRH_orbitingTrajectory.txt";
    trueposepath = "/local/agpetit/soft/apTracking/mbt-co/build/truepose.txt";
    posepath = "/local/agpetit/soft/apTracking/mbt-co/build/poseatlantis.txt";

    // Read the command line options
    if (!getOptions(argc, argv, opt_object, opt_ipath, opt_display, start_image, configFile, opt_learn, opt_detect, inline_init)) {
      return (-1);
    }

    opt_display = true;
    // Get the option values

     if (!opt_ipath.empty())
     {
       ipath = opt_ipath;// + vpIoTools::path("/image%06d.png");
//       truthpath = opt_ipath + vpIoTools::path("/truth.txt");
     }
     else
     {
       ipath = env_ipath2 + vpIoTools::path("/soft/luxifer_20120111_livraison_inria/demo9/image%06d.png");
//       truthpath = env_ipath2 + vpIoTools::path("/truth.txt");
     }

     if (!opt_object.empty())
       object = opt_object;
     else
    	object = "spot5";

     if (configFile.empty())
         configFile = object + vpIoTools::path("/") + object + vpIoTools::path(".lua");
      modelFile = object + vpIoTools::path("/")+ object + vpIoTools::path(".obj");
      initFile = object + vpIoTools::path("/") + object;

      //configFile2 = object + vpIoTools::path("/") + object + vpIoTools::path("2.lua");

    opath = vpIoTools::path("out/image%06d.png");

    // Path the views of hierarchical graph are saved

    vpath = vpIoTools::path("views/image%06d.png");

    scpath = vpIoTools::path("sc_hist/hist%06d.txt");

    // Path to the segmented images, generated through matlab

    isegpath = "/local/agpetit/Downloads/FgSeg" + vpIoTools::path("/pics/AC%04d.png");

    apMbTracker tracker;
    // Set tracking and rendering parameters
    vpCameraParameters mcam;
    apRend mrend;
    apDetection detect;
    apSegmentationParameters seg;
    apLearn learn;
    tracker.loadConfigFile(configFile);
    tracker.getCameraParameters(mcam);
    tracker.getRendParameters(mrend);
    tracker.getDetectionParameters(detect);
    tracker.getSegmentationParameters(seg);
    tracker.getLearningParameters(learn);

//    apMbTracker tracker2;
    // Set tracking and rendering parameters
    /*vpCameraParameters mcam2;
    apRend mrend2;
    apDetection detect2;
    apLearn learn2;
    tracker2.loadConfigFile(configFile2);
    tracker2.getCameraParameters(mcam2);
    tracker2.getRendParameters(mrend2);
    tracker2.getDetectionParameters(detect2);
    tracker2.getLearningParameters(learn2);*/

    // OpenCVGrabber to grab images from USB Camera
    vpOpenCVGrabber grabber;

    SceneViewer viewer;
    SceneManager *mgr = new SceneManager(const_cast<QGLContext*>(viewer.context()));
    viewer.setSceneManager(mgr);
    viewer.show();
    viewer.move(200, 200);

    vpImage<unsigned char> Id;
    vpImage<unsigned char> Id1;
    vpImage<vpRGBa> Icol;
    vpImage<vpRGBa> Ioverlay;
    vpImage<vpRGBa> Ioverlaycol;

    /*grabber.setDeviceType(1);
      grabber.open(Id);
      grabber.acquire(Id);*/

    vpRobotAfma6 robot ;
    vpCameraParameters::vpCameraParametersProjType projModel = vpCameraParameters::perspectiveProjWithDistortion;
    robot.init(vpAfma6::TOOL_CCMOP, projModel);
    robot.setRobotState(vpRobot::STATE_POSITION_CONTROL) ;
    cout << " positionnememt Robot" <<endl ;
    //robot.getCameraParameters (mcam, Id);
    mcam.initPersProjWithoutDistortion(847,850,327,245);
    tracker.setCameraParameters(mcam);
    //std::string opath = "/udd/agpetit/soft/ViSP-build-Linux/testRobot/scenario_GRIPPER_Earth_servo_lum2/I%04d.pgm";
  vpColVector q ;
  robot.readPosFile("/udd/agpetit/soft/poseAmazonas_4.pos",q) ;  
  robot.setPositioningVelocity(5) ; 
  robot.setPosition(vpRobot::ARTICULAR_FRAME,q) ;
  vpTime::wait(1000) ; 
  

    vp1394TwoGrabber g(false);
    g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_YUV411);
    g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_60);
    g.open(Icol);

    std::string filename = "/udd/agpetit/soft/apTracking/mbt-co/build/imagesServo/I%04d.png";
    vpVideoWriter writer;
    writer.setFileName(filename.c_str());
    writer.open(Icol);

    vpImage<vpRGBa> *I0;
    std::vector< vpImage<vpRGBa> *> listIm;
    listIm.resize(1);

    for (int i=0 ; i < 1 ; i++) g.acquire(Icol) ; 

    vpImageConvert::convert(Icol,Id);

    const int width = g.getWidth();
    const int height = g.getHeight();

    viewer.resize(width, height);
    mgr->setApRend(&mrend);
    mgr->setImageSize(width, height);
    mgr->setFOV(atan((height * 0.5) / mcam.get_py()) * 360.0 / M_PI);
    mgr->setAspectRatio(width * mcam.get_py() / (height * mcam.get_px()));
    mgr->load(modelFile);

    // Depth edges map, with gradient orientation
    vpImage<unsigned char> Ior(height,width);
    // Normal map and texture map
    vpImage<vpRGBa> Inormd(height,width);
    // Texture edge map
    vpImage<unsigned char> Itex(height,width);
    for (int n=0; n <height ; n++)
    {
        for (int m = 0 ; m < width; m++)
        {
            Itex[n][m]=100;
        }
    }

    // Main window creation and displaying
    vpDisplayX display1;
    vpDisplayX display2;
    if (opt_display)
    {
        display1.init(Id, 10, 10, "Test tracking");
        vpDisplay::display(Id) ;
        vpDisplay::flush(Id);
        //if(tracker.getUseRGB())
        {
        display2.init(Icol, 10, 1000, "Test tracking");
        vpDisplay::display(Icol) ;
        vpDisplay::flush(Icol);
        }
    }


    vpHomogeneousMatrix cMo, cMo2, cMoFilt;

	cout << "time2 "<< opt_detect << endl;

    //Automatic initialization of the tracker
    if(opt_detect)
    {
    	// File where the graph is stored
    	char *hfile="hSpot.txt";
    	// File to store the data (pose...) of each view at the first level

    	char *data0file="dataSpot0.txt";
    	//File to store the data (pose...) of the views at each level of the hierarchical view graph
    	char *data1file="dataSpot1.txt";
    	// File to store the transition probabilities between each prototype view at the last level of the hierarchy
    	char *transProba = "transProbPA.txt";

    	/*char *hfile="hAtlantis.txt";
    	char *data0file="dataAtlantis0.txt";
    	char *data1file="dataAtlantis1.txt";
    	char *transProba = "transProbAtlantis.txt";*/

    	/*char *hfile="hSoyuz.txt";
    	char *data0file="dataSoyuz0.txt";
    	char *data1file="dataSoyuz1.txt";
    	char *transProba = "transProbSoyuz.txt";*/
    	if(opt_learn)
    	{
    		//Learn the 3D model to build the hierarchical view graph
        	apViews views;
        	// Distance camera/3d model in the view sphere
        	// Soyuz
        	//double dist = 3;
        	//Atlantis , spot
        	double dist = 50;
        	// Initialize the view sphere
        	views.initViewSphere(learn);
    		//Build the view graph - the resulting views are saved and the hierarchical graph is stored in txt file (hfile)
    		views.buildViewGraph(mcam,hfile,mgr,data0file,data1file,vpath, transProba, height, width, dist);
    	}

    	int fr;
    	apDetector detector;
    	detector.init(detect);
    	detector.loadViews(vpath,hfile,data0file,data1file);
    	detector.setFilters(vpath, mcam);
    	detector.computeTransitionProbV(transProba);
    	detector.setSegmentationParameters(seg);
    	detector.setTracker(tracker);
    	std::cout << " Ok particle filters set - Click to detect " << std::endl;
        while(!vpDisplay::getClick(Id,false)){
            vpDisplay::display(Id);
            vpDisplay::displayCharString(Id, 15, 10,
                                         "Ready to detect - click",
                                         vpColor::red);
            vpDisplay::flush(Id) ;
        }
    	double t0= vpTime::measureTimeMs();
    	//thld = 20;
    	detector.detect(ipath,isegpath,vpath, scpath,mcam,start_image,20,apDetector::TOP,apDetector::MEAN);
    	double t1= vpTime::measureTimeMs();
    	cMo = detector.getPose();
    	fr =  detector.getFrame();
    	//tracker.setPose(cMo);
    	cout << "detection time "<< t1-t0 << endl;
    	for (int k = 0; k<fr;k++)
    	    {
        	//reader.acquire(Id);
        	//readerRGB.acquire(Icol);
    		}
    	tracker.setPose(cMo);
    	tracker.init(Id,cMo);
    }

    cout << "detection time "<< endl;

    // Manual initialization of the tracker
    if (opt_display && opt_click_allowed && !opt_detect)
    {
        if (inline_init.empty())
        {
            while(!vpDisplay::getClick(Id,false))
            {
                vpDisplay::display(Id);
                vpDisplay::displayCharString(Id, 15, 10,
                                             "click after positioning the object",
                                             vpColor::red);
                vpDisplay::flush(Id) ;
            }
            tracker.initClick(Id, initFile.c_str(), true);
        }
        else
        {
            memcpy(cMo.data, inline_init.data(), sizeof(double) * inline_init.size());
            tracker.setPose(cMo);
        }
    }

	   if (opt_display && opt_click_allowed && !opt_detect)
	   {
	     tracker.initClick(Id, initFile.c_str(), true);
	   }
	   /*else
	   {
	     vpHomogeneousMatrix cMoi(-0.002774173802,-0.001058705951,0.2028195729,2.06760528,0.8287820106,-0.3974327515);
	     tracker.init(Id,cMoi);
	   }*/

	tracker.setIprec(Id);
	tracker.setIprecRGB(Icol);
    tracker.getPose(cMo);
    tracker.setGroundTruth(gdtpath, trueposepath, start_image);
    tracker.initKltTracker(Id);
    std::cout << " ok " << std::endl;

    vpDisplay::getClick(Id);
    /*cMo2 = cMo;
	tracker2.setPose(cMo2);*/

    //Init pose for Trakmark sequence

    /*cMo.buildFrom(11.7759940348,5.8999979250,5.5547190835,-2.6525344080,-0.0000000000,1.6833495227 );
       cMo.buildFrom( -0.768532741,  6.24302505,  13.54560648,  -2.683611579,  0.003069081378,  1.629208268 );
       cMo=cMo.inverse();*/
    //tracker.init(Id,cMo);
    if (opt_display)
        vpDisplay::flush(Id);
    double px = mcam.get_px() ;
    double py = mcam.get_py() ;


    // ----------------------------------------------------------

    //grabber.setDeviceType(1);
    /*grabber.open(Idisplay);
grabber.acquire(Idisplay);*/
    // Compute the initial pose of the camera
    //computeInitialPose(&mcam, Idisplay, &mPose, md, mcog, &cmo, mP);
    // Close the framegrabber
    //grabber.close();
    //grabber.open(Id);

    vpMatrix pose;
    vpTranslationVector tr, tr2, trT;
    vpRotationMatrix R,RT;
    vpRxyzVector Rxyz;
    cMo.extract(tr);

    bool useKalmanFilter = tracker.getUseKalman();
    apKalmanFilterQuat filt;
    apKalmanParameters kparam;
    tracker.getKalmanParameters(kparam);
    vpMatrix covMat;
    if(useKalmanFilter)
    {
    filt.initFilter(cMo, kparam);
    }
    int im=start_image;

    tracker.setFrame(start_image);

    vpColVector error(6);

    fstream fout("out/tracking.txt", std::ios_base::out);

    cMo2 = cMo;

    vpImage<double> Igrad, Igradx, Igrady;
    Igrad.resize(height,width);
    Igradx.resize(height,width);
    Igrady.resize(height,width);

    int sample = 6;
    double meantime =0;
    int meant = 1;

    vpHomogeneousMatrix fMoV;
    vpHomogeneousMatrix  fMc ;
  
    vpColVector crf(6); // current robot position
    vpColVector crfd(6);//current robot displacement
    {
    robot.getPosition( vpRobot::REFERENCE_FRAME,crf) ;
    vpTranslationVector tr( crf[0], crf[1], crf[2]) ;
    vpRxyzVector r( crf[3], crf[4], crf[5]) ;
    vpRotationMatrix R(r) ;
    //   vpHomogeneousMatrix fMc ;
    fMc.buildFrom(tr,R) ;
    }

// Definition de la loi de commande

  vpServo task ;

std::vector<vpPoseVector> poses;
poses.resize(0);
  vpTRACE("sets the desired camera locations " );
 vpHomogeneousMatrix cdMo;
vpPoseVector cdro0(cMo);
  //vpPoseVector cdro(-0.001422055661 , -0.002988709854 , 0.4485639623 , -0.01 , -0.03,  1.56);
  vpPoseVector cdro(0 , 0 , cdro0[2] /*0.70*/ , cdro0[3] , cdro0[4],  cdro0[5]);
//vpPoseVector cdro(cdro0[0] , cdro0[1],  cdro0[2], 0 , 0 , 1.57 );
cdMo.buildFrom(cdro0);
poses.push_back(cdro);
vpPoseVector cdro1(0, 0 , 0.70 ,0,0, 1.57);
poses.push_back(cdro1);
vpPoseVector cdro2(0, 0 , 0.12 ,0,0, 1.57);
poses.push_back(cdro2);

double residu=1;

  //----------------------------------------------------------------------
  // A 2 1/2 D visual servoing can be defined by
  // - the position of a point x,y
  // - the difference between this point depth and a desire depth
  //   modeled by log Z/Zd to be regulated to 0
  // - the rotation that the camera has to realized cdMc

  // Let us now defined the current value of these features


  // since we simulate we have to define a 3D point that will
  // forward-projected to define the current position x,y of the
  // reference point

  //------------------------------------------------------------------
  // First feature (x,y)
  //------------------------------------------------------------------
  vpTRACE("1st feature (x,y)");
  //------------------------------------------------------------------
  vpTRACE("\tsets the point coordinates in the world frame "  ) ;
  vpPoint P ;
  P.setWorldCoordinates(0,0,0) ;

  vpTRACE("\tproject : computes  the point coordinates in the camera frame and its 2D coordinates"  ) ;
  P.track(cMo) ;
  vpPoint Pd ;
  Pd.setWorldCoordinates(0,0,0) ;
  Pd.track(cdMo) ; 

  vpFeaturePoint p,pd ;
  vpFeatureBuilder::create(p,P)  ;
  vpFeatureBuilder::create(pd,Pd)  ;

  vpTRACE("2nd feature ThetaUz and 3rd feature t") ;
 
 
 vpHomogeneousMatrix cdMc ;
 vpTRACE("\tcompute the rotation that the camera has to realize "  ) ;
 cdMc = cdMo*cMo.inverse() ;

 
  vpFeatureThetaU tuz(vpFeatureThetaU::cdRc) ;
  tuz.buildFrom(cdMc);
  vpFeatureTranslation t(vpFeatureTranslation::cdMc) ;
  t.buildFrom(cdMc) ;
  

  vpTRACE("\tsets the desired rotation (always zero !) ") ;
  vpTRACE("\tsince s is the rotation that the camera has to realize ") ;


  vpTRACE("define the task") ;
  vpTRACE("\t we want an eye-in-hand control law") ;
  vpTRACE("\t robot is controlled in the camera frame") ;
  task.setServo(vpServo::EYEINHAND_CAMERA) ;
  task.setInteractionMatrixType(vpServo::CURRENT) ;
  
  task.addFeature(t) ;
  task.addFeature(p,pd) ;
  task.addFeature(tuz,vpFeatureThetaU::TUz) ;

  vpTRACE("\t set the gain") ;
  task.setLambda(0.02) ;
//task.setLambda(0.01) ;


  vpTRACE("Display task information " ) ;
  task.print() ;

  // ------------------------------------------------------
  // Acquisition de l'image de reference
  
robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;
vpColVector v(6);
    // Main tracking loop
    try
    {
	for(int ph = 0; ph < 3; ph++)
	{
	cdMo.buildFrom(poses[ph]);
	residu = 1;
        while (residu>0.001){


            // Render the 3D model, get the depth edges, normal and texture maps
            try{
                mgr->updateRTT(Inormd,Ior,&cMo);
                a.processEvents(QEventLoop::AllEvents, 1);
                //vpImageIo::writePNG(Inormd, "Inormd.png");
                //vpImageIo::writePNG(Ior, "Ior.png");
            }
            catch(...){
                vpTRACE("Error in 3D rendering") ;
                throw;
            }
            vpDisplay::display(Icol);
            tracker.displayRend(Icol,Inormd,Ior,vpColor::green, 1);
            vpDisplay::flush(Icol);
            vpDisplay::getImage(Icol,Ioverlaycol);

            if(useKalmanFilter)
        	{
                filt.predictPose();
                filt.getPredPose(cMo);
                tracker.setPose(cMo);
                mgr->updateRTT(Inormd,Ior,&cMo);
                a.processEvents(QEventLoop::AllEvents, 1);
            }
            // Acquire images
            try{

        	g.acquire(Icol) ;
		vpImageConvert::convert(Icol,Id);

      //Save the image
      //writer.saveFrame(I);
            }
            catch(...){
                break;
            }
            vpDisplay::display(Id);

           /* for (int i=0;i<height-0;i++)
            		{
            			for (int j=0;j<width-0;j++)
            			{
            				if(i>3 && i < height-3 && j>3 && j < width-3){
            				Igradx[i][j]=(apImageFilter::sobelFilterX(Id,i,j));
            				Igrady[i][j]=(apImageFilter::sobelFilterY(Id,i,j));
            				Igrad[i][j] = sqrt(Igradx[i][j]*Igradx[i][j] + Igrady[i][j]*Igrady[i][j]);
            				}
            				else
            				{
            				Igrad[i][j] = 0;
            				Igradx[i][j] = 0;
            				Igrady[i][j] = 0;
            				}
            			}
            	}*/

            //vpDisplay::getClick(Id,true);
            //tracker.setPose(cMo);
            cMo.extract(tr);
            // Pose tracking
            try{
                double t0= vpTime::measureTimeMs();
                tracker.track(Id,Icol,Inormd,Ior,Ior,tr[2]);

                tracker.displayKltPoints(Id);

                //tracker.track(Id, Igrad, Igradx, Igrady, Inormd, Ior, Ior, tr[2]);
                if (useKalmanFilter)
                {
                    tracker.getPose(cMo);
                    tracker.getCovarianceMatrix(covMat);
                    filt.estimatePose(cMo,covMat);
                    //tracker.setPose(filt.cMoEst);
                 	cMoFilt = filt.cMoEst;
                    tracker.display(Id,cMoFilt,mcam,vpColor::green,1);
                }
                double t1= vpTime::measureTimeMs();
                cout << "timeTrack "<<t1-t0<<endl;
                if(im>1200)
                {
                meantime += (t1-t0);
                std::cout << " mean " << (double)meantime/(double)(meant) << std::endl;
                meant++;
                }
            }
            catch(...){
                vpTRACE("Error in tracking") ;
                throw;
            }
            // Display 3D model
            tracker.getPose(cMo);

            //tracker.computeError(error);
            tracker.display(Id,cMo,mcam,vpColor::red,1);

            std::cout<<" cMo out" << cMo <<std::endl;
            std::cout<<" cMo filt" << cMoFilt <<std::endl;

            P.track(cMo) ;
            vpFeatureBuilder::create(p,P)  ;


            cdMc = cdMo*cMo.inverse() ;
            tuz.buildFrom(cdMc) ;
            t.buildFrom(cdMc) ;
            v = task.computeControlLaw() ;

            robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;
	    residu=task.error.sumSquare();
            std::cout << " Residu " << task.error.sumSquare() <<std::endl ; 
	
        robot.getPosition( vpRobot::REFERENCE_FRAME,crf) ;
	//robot.getCameraDisplacement(crfd) ;
	vpTranslationVector tr( crf[0], crf[1], crf[2]) ;
	vpTranslationVector tr1( crfd[0], crfd[1], crfd[2]) ;
	vpRxyzVector r( crf[3], crf[4], crf[5]) ;
	vpRxyzVector r1( crfd[3], crfd[4], crfd[5]) ;
	vpRotationMatrix R(r) ;
	vpRotationMatrix R1(r1) ;
        double crf1=crf[1];
        fMc.buildFrom(tr,R) ;
        vpHomogeneousMatrix cMoV=(fMc.inverse())*fMoV;


            cMo.extract(tr);
            cMo.extract(R);
            Rxyz.buildFrom(R);
            pose.resize(im+1,7,false);
            pose[im][0] = tr[0];
            pose[im][1] = tr[1];
            pose[im][2] = tr[2];
            pose[im][3] = Rxyz[0];
            pose[im][4] = Rxyz[1];
            pose[im][5] = Rxyz[2];
            pose[im][6] = im;

            std::cout << " im " << im << std::endl;
            //pose.saveMatrix(posepath,pose,false,"");
            /*mgr->updateRTT(Inormd,Ior,&cMo2);
            tracker2.setPose(cMo2);
            cMo2.extract(tr2);
            tracker2.track(Id,Icol,Inormd,Ior,Ior,tr2[2]);
            tracker2.getPose(cMo2);
            tracker2.display(Id,cMo2,mcam,vpColor::red,1);*/

            vpDisplay::flush(Id);
            vpDisplay::getImage(Id,Ioverlay);
            //vpDisplay::getClick(Id);
            fout << im << ' ' << cMo << std::endl;
            // Write images
             I0 = new vpImage<vpRGBa>;
		//vpDisplay::getImage(I,Ioverlay);
		*I0 = Ioverlaycol;
		listIm.push_back(I0);
            im++;
        }

	}

	task.print() ;
        task.kill();

	vpImage<vpRGBa> Iover(480,640);
	for(int im=0;im<listIm.size()-1;im++)
	{
	Iover=*(listIm[im+1]);
	writer.saveFrame(Iover);
	}

        grabber.close();
    }
    catch ( char const *e)
    {
        std::cerr << "Exception: " << e << "\n";
        return 1;
    }
    return EXIT_SUCCESS;


}

