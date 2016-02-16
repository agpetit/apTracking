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

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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

    vpath = vpIoTools::path("views2/image%06d.png");

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

//    vpMbPointsTracker tracker2;
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

    //VideoReader to read images from disk
    vpVideoReader reader;
    reader.setFileName(ipath.c_str());
    reader.setFirstFrameIndex(start_image);
    reader.open(Id);
    reader.acquire(Id);
    vpVideoReader readerRGB;
    //if(tracker.getUseRGB())
    {
    	readerRGB.setFileName(ipath.c_str());
        readerRGB.setFirstFrameIndex(start_image);
        readerRGB.open(Icol);
        readerRGB.acquire(Icol);
    }
    const int width = reader.getWidth();
    const int height = reader.getHeight();

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


    /*vpImage<vpRGBa> I;
    vpVideoWriter writer;
    //Initialize the writer.
    writer.setFileName("./images/image%06d.png");
    writer.open(I);

    int device = 2;
    std::cout << "Use device: " << device << std::endl;
    cv::VideoCapture cap(device); // open the default camera
    cap.set(CV_CAP_PROP_FPS, 30);
    if(!cap.isOpened())  // check if we succeeded
      return -1;
    cv::Mat frame;
    cap >> frame; // get a new frame from camera
    IplImage iplimage = frame;
    std::cout << "Image size: " << iplimage.width << " "
              << iplimage.height << std::endl;
    //vpImage<unsigned char> I; // for gray images
    vpImageConvert::convert(&iplimage, I);
    vpDisplayOpenCV d(I);

    for ( int img =0;img < 3000 ; img++)
    {
        cap >> frame; // get a new frame from camera
        iplimage = frame;
        // Convert the image in ViSP format and display it
        vpImageConvert::convert(&iplimage, I);
        vpDisplay::display(I);
        vpDisplay::flush(I);
        if (vpDisplay::getClick(I, false)) // a click to exit
          break;
      //Here the code to capture or create an image and stores it in I.
      //Save the image
      writer.saveFrame(I);
    }
    writer.close();


    getchar();*/


    vpHomogeneousMatrix cMo, cMo2, cMoFilt;

	cout << "time2 "<< opt_detect << endl;

    //Automatic initialization of the tracker
    if(opt_detect)
    {
    	// File where the graph is stored
    	/*char *hfile="hSpot.txt";
    	// File to store the data (pose...) of each view at the first level

    	char *data0file="dataSpot0.txt";
    	//File to store the data (pose...) of the views at each level of the hierarchical view graph
    	char *data1file="dataSpot1.txt";
    	// File to store the transition probabilities between each prototype view at the last level of the hierarchy
    	char *transProba = "transProbPA.txt";*/

    	/*char *hfile="hAtlantis.txt";
    	char *data0file="dataAtlantis0.txt";
    	char *data1file="dataAtlantis1.txt";
    	char *transProba = "transProbAtlantis.txt";*/

    	/*char *hfile = "hSoyuz.txt";
    	char *data0file = "dataSoyuz0.txt";
    	char *data1file = "dataSoyuz1.txt";
    	char *transProba = "transProbSoyuz.txt";*/

    	// File where the graph is stored
    	std::string hf = "h" + object + ".txt";
    	char *hfile = (char *)hf.c_str();
    	// File to store the data (pose...) of each view at the first level
    	std::string data0f = "data" + object + "0.txt";
    	char *data0file = (char *)data0f.c_str();
    	//File to store the data (pose...) of the views at each level of the hierarchical view graph
    	std::string data1f = "data" + object + "1.txt";
    	char *data1file = (char *)data1f.c_str();
    	// File to store the transition probabilities between each prototype view at the last level of the hierarchy
    	std::string transP = "transProb" + object + ".txt";
    	char *transProba = (char *)transP.c_str();

    	/*char *hfile="hAmazonas.txt";
    	    	char *data0file="dataAmazonas0.txt";
    	    	char *data1file="dataAmazonas1.txt";
    	    	char *transProba = "transProbAtlantis.txt";*/

    	if(opt_learn)
    	{
    		//Learn the 3D model to build the hierarchical view graph
        	apViews views;
        	// Initialize the view sphere
        	views.initViewSphere(learn);
    		//Build the view graph - the resulting views are saved and the hierarchical graph is stored in txt file (hfile)
    		views.buildViewGraph(mcam,hfile,mgr,data0file,data1file,vpath, transProba, height, width);
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
        	reader.acquire(Id);
        	readerRGB.acquire(Icol);
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

    //vpDisplay::getClick(Id);
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
    vpMatrix covariance;
    vpTranslationVector tr, tr2, trT;
    vpRotationMatrix R,RT;
    vpRxyzVector Rxyz;
    cMo.extract(tr);

    bool useKalmanFilter = tracker.getUseKalman();
    apKalmanFilter filt;
    apKalmanParameters kparam;
    tracker.getKalmanParameters(kparam);
    vpMatrix covMat;
    vpMatrix covMatME;

    if(useKalmanFilter)
    {
    filt.initFilter(cMo, kparam);
    }
    int im=start_image;

    tracker.setFrame(start_image);

    vpColVector error(6);

    fstream fout("out/tracking.txt", std::ios_base::out);
    /*fstream fin(truthpath, std::ios_base::in);
    for (int k = 0; k<firstFrame; k++)
    {
    	fout >> k >> ' ' << cMo << std::endl;
    }*/
    /*
    vpImage<vpRGBa> Ibgd;
    vpImage<unsigned char> IbgdG;
    vpImageIo::readPNG(Ibgd, "background.png");
    vpImageConvert::convert(Ibgd,IbgdG);
    apSegmentation seg;
    seg.init(Ibgd);
    seg.segmentFgdBgd(Ibgd,Id,argc,argv);*/
    /*vpImage<unsigned char> Iseg;
    vpImage<vpRGBa> Icol;
    vpVideoReader readerCol;
    readerCol.setFileName(ipath.c_str());
    readerCol.setFirstFrameIndex(start_image);
    readerCol.open(Icol);
    readerCol.acquire(Icol);*/

    cMo2 = cMo;

    vpImage<double> Igrad, Igradx, Igrady;
    Igrad.resize(height,width);
    Igradx.resize(height,width);
    Igrady.resize(height,width);

    int sample = 1;
    double meantime =0;
    int meant = 1;

    vpPlot plotPose;
    vpPlot plotCov;

	char *plotfile0 = "plotfile0.dat";
	char *plotfile1 = "plotfile1.dat";

    //Create a window (700 by 700) at position (100, 200) with two graphics
      vpPlot A(2, 700, 700, 100, 200, "Curves...");
      //The first graphic contains 1 curve and the second graphic contains 2 curves
      A.initGraph(0,2);
      A.initGraph(1,2);
      //The color of the curve in the first graphic is red
      A.setColor(0,0,vpColor::blue);
      A.setColor(0,1,vpColor::red);
      //The first curve in the second graphic is green
      A.setColor(1,0,vpColor::blue);
      //The second curve in the second graphic is blue
      A.setColor(1,1,vpColor::red);
      //Add the point (0,0) in the first graphic
      A.plot(0,0,0,0);
      A.plot(0,1,0,0);
      //Add the point (0,1) to the first curve of the second graphic
      A.plot(1,0,0,0);
      //Add the point (0,2) to the second curve of the second graphic
      A.plot(1,1,0,0);

      std::cout << " ok ok " << std::endl;

  	//create the cascade classifier object used for the face detection
  	CascadeClassifier face_cascade;
  	//use the haarcascade_frontalface_alt.xml library
  	face_cascade.load("/local/agpetit/soft/OpenCV-2.4.3/data/haarcascades/haarcascade_frontalface_alt.xml");

    std::cout << " ok ok ok" << std::endl;



  	//setup image files used in the capture process
  	Mat captureFrame;
  	Mat grayscaleFrame;

  	//vpImage<vpRGBa> I3(1080,1440);
  	vpImage<vpRGBa> I3(height,width);
	 vpDisplayX displayF;
	 displayF.init(I3, 1500, 10, "Face");
	 vpDisplay::display(I3);
	 vpDisplay::flush(I3);
	    std::cout << " ok ok ok ok" << std::endl;



    // Main tracking loop
    try
    {
        while(true){
/*

            // Render the 3D model, get the depth edges, normal and texture maps
            try{
                mgr->updateRTT(Inormd,Ior,&cMo);
                a.processEvents(QEventLoop::AllEvents, 1);
                //vpImageIo::writePNG(Inormd, "Inormd.png");
                //vpImageIo::writePNG(Ior, "Ior.png");
            }
            catch(...){
                vpTRACE("Error in 3D rendering");
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

            */
            // Acquire images
            try{
            	for (int sp = 0; sp < sample ; sp++)
            	{
                    reader.acquire(Id);
                    if(tracker.getUseRGB())
                    	readerRGB.acquire(Icol);
            	}
            }
            catch(...){
                break;
            }
            vpDisplay::display(Id);

            std::cout << " ok ok " << std::endl;

            IplImage* Ip = NULL;
            IplImage* Ip1;
            vpImageConvert::convert(Icol, Ip);
            Mat dst(Ip);

            //convert captured image to gray scale and equalize
            cvtColor(dst, grayscaleFrame, CV_BGR2GRAY);
            equalizeHist(grayscaleFrame, grayscaleFrame);

            		//create a vector array to store the face found
            std::vector<Rect> faces;

            		//find faces and store them in the vector array
            face_cascade.detectMultiScale(grayscaleFrame, faces, 1.1, 2, CV_HAAR_FIND_BIGGEST_OBJECT|CV_HAAR_SCALE_IMAGE, Size(10,10), Size(20,20));

            		//draw a rectangle for all found faces in the vector array on the original image
            for(int i = 0; i < faces.size(); i++)
            {
            Point pt1(faces[i].x + faces[i].width, faces[i].y + faces[i].height);
            Point pt2(faces[i].x, faces[i].y);
            rectangle(dst, pt1, pt2, cvScalar(0, 255, 0, 0), 1, 8, 0);
            }
			 Ip1 = new IplImage(dst);
			 vpImageConvert::convert(Ip1, I3);
			 delete Ip1;

            			vpImage<vpRGBa> Ioverlay;
            			 Ip1 = new IplImage(dst);
            			 vpImageConvert::convert(Ip1, I3);
            			 delete Ip1;
            			 vpDisplay::display(I3);
            			 vpDisplay::flush(I3);
            			 vpDisplay::getImage(I3,Ioverlay);


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

            /*

            //tracker.setPose(cMo);
            cMo.extract(tr);
            // Pose tracking
            try{
                double t0= vpTime::measureTimeMs();
                tracker.track(Id,Icol,Inormd,Ior,Ior,tr[2]);

                tracker.displayKltPoints(Id);

                //tracker.track(Id, Igrad, Igradx, Igrady, Inormd, Ior, Ior, tr[2]);
                {
                    tracker.getCovarianceMatrix(covMat);
                    tracker.getCovarianceMatrixME(covMatME);
                }
                if (useKalmanFilter)
                {
                    tracker.getPose(cMo);
                    tracker.getCovarianceMatrix(covMat);
                    tracker.getCovarianceMatrixME(covMatME);
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

            */


            //covariance.resize(im+1,5);
            std::cout << " covmatME " << covMatME << " covmat " << covMat << std::endl;
            //A.plot(0,0,im-start_image+1,50*(covMat[0][0] + covMat[1][1] + covMat[2][2]));
            //A.plot(1,0,im-start_image+1,50*(covMat[3][3] + covMat[4][4] + covMat[5][5]));
            //A.plot(0,1,im-start_image+1,(covMatME[0][0] + covMatME[1][1] + covMatME[2][2]));
            //A.plot(1,1,im-start_image+1,covMatME[3][3] + covMatME[4][4] + covMatME[5][5]);
            //covariance[im][0] = covMat[0][0] + covMat[1][1] + covMat[2][2];
            //covariance[im][1] = covMat[2][2] + covMat[3][3] + covMat[4][4];


            std::cout << " im " << im << std::endl;
            //pose.saveMatrix(posepath,pose,false,"");
            /*mgr->updateRTT(Inormd,Ior,&cMo2);
            tracker2.setPose(cMo2);
            cMo2.extract(tr2);
            tracker2.track(Id,Icol,Inormd,Ior,Ior,tr2[2]);
            tracker2.getPose(cMo2);
            tracker2.display(Id,cMo2,mcam,vpColor::red,1);*/

            vpDisplay::flush(Id);
            //vpDisplay::getImage(Id,Ioverlay);
            //vpDisplay::getClick(Id);
            fout << im << ' ' << cMo << std::endl;
            // Write images
            char buf4[FILENAME_MAX];
            sprintf(buf4, opath.c_str(), im-start_image);
            std::string filename4(buf4);
            //std::cout << "Write: " << filename4 << std::endl;
            //vpImageIo::write(Ioverlaycol, filename4);
            im++;
            //vpDisplay::getClick(Id,true);
            //A.saveData(0,plotfile0);
            //A.saveData(1,plotfile1);
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

