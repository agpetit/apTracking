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

#include "structures.h"
#include <QApplication>
#include "surrender/scenemanager.h"
#include "surrender/sceneviewer.h"

#include <visp/vpOpenCVGrabber.h>
#include <visp/vpVideoReader.h>
#include <cstdlib>
#include <visp/vpDebug.h>
#include <visp/vpConfig.h>
#include <visp/vpDisplayX.h>
//#include <visp/vpDisplayGTK.h>
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

#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

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
#include "p_helper.h"
#include <queue>
#include <condition_variable>


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

typedef struct point_struct{

  double x;
  double y;
  double z;
}point_struct;


int main(int argc, char **argv)
{
    QApplication a(argc, argv);

    // Paths and file names
    std::string env_ipath;
    std::string env_ipath2;
    std::string opath,opath1;
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
    std::string covariancepath;
    std::string timepath;
    std::vector<double> inline_init;
//std::string truthpath;
    bool opt_display;
    bool opt_learn = false;
    bool opt_detect = false;
    bool displayMovingEdge = true;
    bool opt_click_allowed = true;
    int start_image = 0;

    env_ipath="/local/agpetit/images";
    env_ipath2="/local/agpetit";
    s_path = "/home/soft/apTracking/mbt-co/build2/";
    gdtpath = "/home/soft/apTracking/mbt-co/build2/ADRH_orbitingTrajectory.txt";
    trueposepath = "/home/soft/apTracking/mbt-co/build2/truepose_x7_.txt";
    posepath = "/home/soft/apTracking/mbt-co/build2/pose";//atlantis.txt";
    covariancepath = "/home/soft/apTracking/mbt-co/build2/covariance";
    timepath = "time";



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
        object = "soyuz";

     if (configFile.empty())
         configFile = object + vpIoTools::path("/") + object + vpIoTools::path(".lua");
      modelFile = object + vpIoTools::path("/")+ object + vpIoTools::path(".obj");
      initFile = object + vpIoTools::path("/") + object;

      std::string filep = vpIoTools::path("mh_ccd_klt_prev_x1_f1_h2.txt");


      posepath = posepath + object + filep;
      covariancepath = covariancepath + object + filep;
      timepath = timepath + object + filep;


      //configFile2 = object + vpIoTools::path("/") + object + vpIoTools::path("2.lua");

    opath = vpIoTools::path("out") + vpIoTools::path("/image%06d.png");
    opath1 = vpIoTools::path("out") + vpIoTools::path("/imageKalman%06d.png");


    // Path the views of hierarchical graph are saved

    vpath = vpIoTools::path("views1/image%06d.png");

    scpath = vpIoTools::path("sc_hist/hist%06d.txt");


    // Path to the segmented images, generated through matlab

    isegpath = "/local/agpetit/Downloads/FgSeg" + vpIoTools::path("/pics/AC%04d.png");

    std::string plot0 = object + vpIoTools::path("ccdColx3plotfile0.dat");
    std::string plot1 = object + vpIoTools::path("ccdColx3plotfile1.dat");

        char *plotfile0 = (char *)plot0.c_str();
        char *plotfile1 = (char *)plot1.c_str();

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
    //vpOpenCVGrabber grabber;

    vpMatrix lik;
    vpMatrix outlik;


    /*lik.loadMatrix("filtlikelihoodspot6k.txt",lik,false);
    outlik.resize(lik.getCols(),lik.getRows());
    for (int i =0; i<10;i++)
        for (int j =0; j<53;j++)
                outlik[i][j] = lik[j][i];

                         outlik.saveMatrix("filtlikelihoodspot6kT.txt",outlik,false);

                         getchar();*/

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

       // tx="192.169",ty="-350",tz="226", R00= "-0.231176", R01="2.1603e-16", R02="0.972912" , R10="0.972912", R11="1.73796e-16" , R12="0.231176", R20="-1.19147e-16" , R21="1" , R22="-2.50356e-16"

    /*-7.372876909e-17  1  -1.766268742e-16  226
    0.231176  -1.5479766e-16  -0.972912  -192.169
    -0.972912  -1.1256366e-16  -0.231176  350
    0  0  0  1 */

       cMo[0][3] = 192.169;
       cMo[1][3] = -350;
       cMo[2][3] = 226;
       cMo[0][0] = -0.231176;
       cMo[0][1] = 2.1603e-16;
       cMo[0][2] = 0.972912;
       cMo[1][0] = 0.972912;
       cMo[1][1] = 1.73796e-16;
       cMo[1][2] = 0.231176;
       cMo[2][0] = -1.19147e-16;
       cMo[2][1] = 1;
       cMo[2][2] = -2.50356e-16;

       // pose simulated image, amera frame

       cMo[0][3] =-175.4532513;
       cMo[1][3] = -239.2093021 ;
       cMo[2][3] = 350;
       cMo[0][0] = -0.231176;
       cMo[0][1] = 2.1603e-16;
       cMo[0][2] = 0.972912;
       cMo[1][0] = 0.972912;
       cMo[1][1] = 1.73796e-16;
       cMo[1][2] = 0.231176;
       cMo[2][0] = -1.19147e-16;
       cMo[2][1] = 1;
       cMo[2][2] = -2.50356e-16;

       // pose real image liver1.png -222.375 154.59 379.542 [0.989822 -3.09578e-17 0.142311,0.136547 0.281733 -0.949727,-0.0400937 0.959493 0.278865

       cMo[0][3] =-222.375;
       cMo[1][3] = 154.59;
       cMo[2][3] = 379.542;
       cMo[0][0] = 0.989822;
       cMo[0][1] = -3.09578e-17;
       cMo[0][2] = 0.142311;
       cMo[1][0] = 0.136547;
       cMo[1][1] = 0.281733;
       cMo[1][2] = -0.949727;
       cMo[2][0] = -0.0400937;
       cMo[2][1] = 0.959493;
       cMo[2][2] = 0.278865;

       /*0.9958088264  0.08254072254  0.03939508347  -199.9189609
       0.01511400765  0.2763081441  -0.9609501357  201.4935037
       -0.09020270353  0.9575182186  0.2739020766  390.7839107*/

       cMo[0][3] =-209.9189609;
       cMo[1][3] = 201.4935037;
       cMo[2][3] = 390.7839107;
       cMo[0][0] = 0.9958088264;
       cMo[0][1] = 0.08254072254;
       cMo[0][2] = 0.03939508347;
       cMo[1][0] = 0.01511400765;
       cMo[1][1] = 0.2763081441;
       cMo[1][2] = -0.9609501357;
       cMo[2][0] = -0.09020270353;
       cMo[2][1] = 0.9575182186;
       cMo[2][2] = 0.2739020766;


       //pose liver5.png 0.9594919448  -2.630000004e-08  -0.2817363055  -85.00534936 0.263972639  -0.3494652134  0.8989953551  276.0144642 -0.09845705498  -0.936949413  -0.3353090272  352.4944063


       //0.994456 0.103823 0.016656 -213.757,-0.017116 0.316122 -0.948564 194.718,-0.103748 0.943021 0.316146 365.778
       /*cMo[0][3] =-213;
       cMo[1][3] = 194;
       cMo[2][3] = 365;
       cMo[0][0] = 0.989822;
       cMo[0][1] = -3.09578e-17;
       cMo[0][2] = 0.142311;
       cMo[1][0] = 0.136547;
       cMo[1][1] = 0.281733;
       cMo[1][2] = -0.949727;
       cMo[2][0] = -0.0400937;
       cMo[2][1] = 0.959493;
       cMo[2][2] = 0.278865;*/

       //0.989821  -1.09198e-16  -0.142318  -85.00534936
       //-0.133345  0.349464  -0.927412  276.0144642
      // 0.0497352  0.93695  0.345907  352.4944063

       /*cMo[0][3] = -85.00534936;
       cMo[1][3] = 276.0144642;
       cMo[2][3] = 352.4944063;
       cMo[0][0] =  0.989821;
       cMo[0][1] = -1.09198e-16;
       cMo[0][2] = -0.142318;
       cMo[1][0] = -0.133345;
       cMo[1][1] = 0.349464;
       cMo[1][2] =  -0.927412;
       cMo[2][0] = 0.0497352;
       cMo[2][1] = 0.93695;
       cMo[2][2] = 0.345907 ;*/


       // pose liver2.png 0.989821  -4.74415e-17  -0.142318  -108.9137742 -0.141956  0.0713392  -0.987299  281.6685308 0.0101529  0.997452  0.070613  436.65255

       /*cMo[0][3] = -108.9137742;
       cMo[1][3] = 281.6685308;
       cMo[2][3] = 436.65255;
       cMo[0][0] =  0.989821;
       cMo[0][1] = -1.09198e-16;
       cMo[0][2] = -0.142318;
       cMo[1][0] = -0.141956 ;
       cMo[1][1] = 0.0713392;
       cMo[1][2] = -0.987299;
       cMo[2][0] = 0.0101529;
       cMo[2][1] = 0.997452 ;
       cMo[2][2] = 0.070613 ;*/



       // lung Belarus Rt [-1 1.58413e-17 3.67321e-06 182.168,-3.66385e-06 -0.0713392 -0.997452 166.304,2.62043e-07 -0.997452 0.0713392 1038.49]

       /*cMo[0][3] = 202.168;
       cMo[1][3] = 156.304;
       cMo[2][3] = 1038.49;
       cMo[0][0] = -1;
       cMo[0][1] = 1.58413e-17;
       cMo[0][2] = 3.67321e-06;
       cMo[1][0] = -3.66385e-06;
       cMo[1][1] = -0.0713392;
       cMo[1][2] = -0.997452;
       cMo[2][0] = 2.62043e-07;
       cMo[2][1] = -0.997452;
       cMo[2][2] = 0.0713392;*/

   /* -0.231176  0.972912  -1.19147e-16  384.9440607
    2.1603e-16  1.73796e-16  1  -226
    0.972912  0.231176  -2.50356e-16  -106.0519261
    0  0  0  1*/

       /*cMo[0][3] = 384.9440607 ;
       cMo[1][3] = -226;
       cMo[2][3] = -106.0519261;
       cMo[0][0] = -0.231176;
       cMo[0][1] = 2.1603e-16;
       cMo[0][2] = 0.972912;
       cMo[1][0] = 0.972912;
       cMo[1][1] = 1.73796e-16;
       cMo[1][2] = 0.231176;
       cMo[2][0] = -1.19147e-16;
       cMo[2][1] = 1;
       cMo[2][2] = -2.50356e-16;*/



      /* std::cout << " cmo inv " << cMo.inverse() << std::endl;
            getchar();*/

    /*vpRotationMatrix R0;
           R0[0][0] = -0.231176;
           R0[0][1] = 2.1603e-16;
           R0[0][2] = 0.972912;
           R0[1][0] = 0.972912;
           R0[1][1] = 1.73796e-16;
           R0[1][2] = 0.231176;
           R0[2][0] = -1.19147e-16;
           R0[2][1] = 1;
           R0[2][2] = -2.50356e-16;

    vpTranslationVector ta(-192.169,350,-226);*/

       /*cMo[0][3] = -236 ;
       cMo[1][3] = 192.169;
       cMo[2][3] = 350;
       cMo[0][0] = 1.049915526e-16;
       cMo[0][1] = -1;
       cMo[0][2] = 3.099296783e-16;
       cMo[1][0] = -0.231176;
       cMo[1][1] = 2.7726234e-16;
       cMo[1][2] = 0.972912;
       cMo[2][0] = -0.972912 ;
       cMo[2][1] =  -1.73796e-16;
       cMo[2][2] = -0.231176 ;*/


      /* -1.333024474e-16  1  -1.907823217e-16  226
       0.231176  -1.5479766e-16  -0.972912  -192.169
       -0.972912  -1.73796e-16  -0.231176  350
       0  0  0  1 */

       /*1.049915526e-16  -1  3.099296783e-16  -226
       -0.231176  2.7726234e-16  0.972912  192.169
       -0.972912  -1.73796e-16  -0.231176  350*/

        //std::cout << " cmo inv " << R0*ta << std::endl;
             // getchar();

        vpRxyzVector vdelta(0.0,0.0,-0.0);
        vpTranslationVector tdelta(0,0,0);

        vpRxyzVector vdelta1(0.,0.0,-0.0);
        vpTranslationVector tdelta1(0,0,0);


        vpRotationMatrix Rdelta;
        Rdelta.buildFrom(vdelta);
        vpHomogeneousMatrix Mdelta;
        Mdelta.buildFrom(tdelta,Rdelta);

        vpRotationMatrix Rdelta1;
        Rdelta1.buildFrom(vdelta1);
        vpHomogeneousMatrix Mdelta1;
        Mdelta1.buildFrom(tdelta1,Rdelta1);


        cMo = Mdelta1*cMo*Mdelta;

        std::cout << " cmo " << cMo << std::endl;

        int wdth = 764;
        int hght = 800;


    cout << "detection time "<< endl;

        vpImage<vpRGBa> Icol1(height,width);

    Icol1 = Icol;
    for (int i = 0; i<height; i++)
        for (int j = 0; j<width; j++)
        {

            Icol1[i][j].R = 0.2126*Icol[i][j].R + 0.7152*Icol[i][j].G + 0.0722*Icol[i][j].B;
            Icol1[i][j].G = Icol1[i][j].R;
            Icol1[i][j].B = Icol1[i][j].R;

            /*Icol1[i][j].R = Icol2[359-i][j].R;
            Icol1[i][j].G = 0;//Icol2[359-i][j].G;
            Icol1[i][j].B = 0;//Icol2[359-i][j].B;
            Id[i][j] = 0.2126*Icol2[359-i][j].R + 0.7152*Icol2[359-i][j].G + 0.0722*Icol2[359-i][j].B;*/

        }
    //Icol = Icol1;

    tracker.setPose(cMo);

    tracker.setIprec(Id);
    tracker.cMoprec = cMo;
    tracker.setIprecRGB(Icol);
    tracker.getPose(cMo);
    tracker.setGroundTruth(gdtpath, trueposepath, start_image);
    //tracker.initKltTracker(Id);
    std::cout << " ok " << std::endl;

    //vpDisplay::getClick(Id);
    /*cMo2 = cMo;
    tracker2.setPose(cMo2);*/

    //Init pose for Trakmark sequence

    /*cMo.buildFrom(11.7759940348,5.8999979250,5.5547190835,-2.6525344080,-0.0000000000,1.6833495227 );
       cMo.buildFrom( -0.768532741,  6.24302505,  13.54560648,  -2.683611579,  0.003069081378,  1.629208268 );
       cMo=cMo.inverse();*/

    tracker.init(Id,cMo);
    tracker.initComm();
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

    /*vpImage<unsigned char> Isat,Ior3;
    apViews view_;
    vpImageIo::readPNG(Isat, "raw.png");
    view_.edgeOrientMap(Isat);
    Ior3.resize(Isat.getHeight(),Isat.getWidth());
    vpImage<vpRGBa> *imArg;
    imArg=view_.dt0(&Isat);
    for (int y = 0; y < Isat.getHeight(); y++){
      for (int x = 0; x < Isat.getWidth(); x++){
//    	  (*imArg)[y][x].A = (double)(*I00)[y][x];
          Ior3[y][x]=(*imArg)[y][x].B;
          //Ior4[y][x]=Iseg[y][x];
      }
    }
    vpImageIo::writePNG(Ior3,"Isatdt.png");

    getchar();*/

    vpMatrix pose;
    vpMatrix covariance;
    vpMatrix timep;
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

    vpImage<unsigned char> Ig(height,width);
    double mtime = 0;
    double timetrack, timerender,timeextract,timevvs, timetrack1;
    double timeKalman = 0;
    double t0,t1;
    vpImage<vpRGBa> Icol2(height,width);
    Icol2 = Icol;

    // Main tracking loop
    try
    {
        while(true){


            // Render the 3D model, get the depth edges, normal and texture maps
            try{
                tracker.getPose(cMo);
                t0= vpTime::measureTimeMs();

                t1= vpTime::measureTimeMs();
                timerender = t1-t0;
                std::cout << "timerender " << t1 - t0 << std::endl;
                //vpImageIo::writePNG(Inormd, "Inormd.png");
                //vpImageIo::writePNG(Ior, "Ior.png");
                tracker.cMoprec = cMo;

            }
            catch(...){
                vpTRACE("Error in 3D rendering");
                throw;
            }
            vpDisplay::display(Icol);
            std::cout << " disp " << im-start_image << std::endl;
            if(im-start_image>0)
            tracker.displayRend(Icol,Inormd,Ior,vpColor::green, 1);
            vpDisplay::flush(Icol);
            vpDisplay::getImage(Icol,Ioverlaycol);

            t0= vpTime::measureTimeMs();

            if(useKalmanFilter)
            {
                filt.cMoEst = cMo;
                filt.predictPose();
                filt.getPredPose(cMo);
                tracker.setPose(cMo);
                tracker.predictKLT = true;
                mgr->updateRTT(Inormd,Ior,&cMo);
                a.processEvents(QEventLoop::AllEvents, 1);
            }

            t1= vpTime::measureTimeMs();
            cout << "timeKalman "<<t1-t0<<endl;
            timeKalman = t1-t0;

            // Acquire images
            try{
                for (int sp = 0; sp < sample ; sp++)
                {

                    reader.acquire(Id);
                    //if(tracker.getUseRGB())
                        /*readerRGB.acquire(Icol);
                        Icol1 = Icol;
                        for (int i = 0; i<height; i++)
                            for (int j = 0; j<width; j++)
                            {

                                Icol1[i][j].R = 0.2126*Icol[i][j].R + 0.7152*Icol[i][j].G + 0.0722*Icol[i][j].B;
                                Icol1[i][j].G = Icol1[i][j].R;
                                Icol1[i][j].B = Icol1[i][j].R;

                            }*/
                        //Icol = Icol1;
                }
            }
            catch(...){
                break;
            }

            //vpImageIo::read(Id,"socketimage10.png");
            //vpImageIo::read(Id,"imagePig3.png");
            //vpImageIo::read(Id,"lung936.png");
            vpDisplay::display(Id);

            //tracker.setPose(cMo);
            cMo.extract(tr);
            // Pose tracking
            try{
                t0= vpTime::measureTimeMs();
                tracker.trackXray(Id, tr[2]);

                //tracker.displayKltPoints(Id);

                //tracker.track(Id, Igrad, Igradx, Igrady, Inormd, Ior, Ior, tr[2]);
                {
                    tracker.getCovarianceMatrix(covMat);
                    tracker.getCovarianceMatrixME(covMatME);
                }
                if (useKalmanFilter)
                {
                    tracker.getPose(cMo);
                    tracker.display(Id,cMo,mcam,vpColor::red,1);
                    tracker.getCovarianceMatrix(covMat);
                    tracker.getCovarianceMatrixME(covMatME);
                    filt.estimatePose(cMo,covMat);
                    tracker.setPose(filt.cMoEst);
                    cMoFilt = filt.cMoEst;
                    //cMo = cMoFilt;
                    //tracker.display(Id,cMoFilt,mcam,vpColor::green,1);
                    //tracker.display(Id,filt.cMoPred_0,mcam,vpColor::blue,1);
                }
                t1= vpTime::measureTimeMs();
                cout << "timeTrack "<<t1-t0 + timeKalman<<endl;
                //if(im>1000)
                {
                meantime += (t1-t0);
                timetrack = t1-t0;
                mtime = (double)meantime/(double)(meant);
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
            tracker.display(Id,cMo,mcam,vpColor::green,1);

            std::cout<<" cMo out" << cMo <<std::endl;
            std::cout<<" cMo filt" << cMoFilt <<std::endl;

            cMo.extract(tr);
            cMo.extract(R);
            Rxyz.buildFrom(R);
            pose.resize(im+1,7,false);
            covariance.resize(im+1,25,false);
            timep.resize(im+1,9,false);
            pose[im][0] = tr[0];
            pose[im][1] = tr[1];
            pose[im][2] = tr[2];
            pose[im][3] = Rxyz[0];
            pose[im][4] = Rxyz[1];
            pose[im][5] = Rxyz[2];
            pose[im][6] = im;
            timep[im][0] = timetrack;
            timep[im][1] = timerender;
            timep[im][2] = tracker.timeextract;
            timep[im][3] = tracker.timetrack;
            timep[im][4] = tracker.timevvs;
            timep[im][5] = tracker.getNbPoints();
            timep[im][6] = tracker.getNbPointsCCD();
            timep[im][7] = tracker.getNbPointsKLT();
            timep[im][8] = timeKalman;


            covariance[im][0] = tracker.covarianceMatrixME[0][0];
            covariance[im][1] = tracker.covarianceMatrixME[1][1];
            covariance[im][2] = tracker.covarianceMatrixME[2][2];
            covariance[im][3] = tracker.covarianceMatrixME[3][3];
            covariance[im][4] = tracker.covarianceMatrixME[4][4];
            covariance[im][5] = tracker.covarianceMatrixME[5][5];
            covariance[im][6] = tracker.covarianceMatrixCCD[0][0];
            covariance[im][7] = tracker.covarianceMatrixCCD[1][1];
            covariance[im][8] = tracker.covarianceMatrixCCD[2][2];
            covariance[im][9] = tracker.covarianceMatrixCCD[3][3];
            covariance[im][10] = tracker.covarianceMatrixCCD[4][4];
            covariance[im][11] = tracker.covarianceMatrixCCD[5][5];
            covariance[im][12] = tracker.covarianceMatrixKLT[0][0];
            covariance[im][13] = tracker.covarianceMatrixKLT[1][1];
            covariance[im][14] = tracker.covarianceMatrixKLT[2][2];
            covariance[im][15] = tracker.covarianceMatrixKLT[3][3];
            covariance[im][16] = tracker.covarianceMatrixKLT[4][4];
            covariance[im][17] = tracker.covarianceMatrixKLT[5][5];
            covariance[im][18] = tracker.covarianceMatrix[0][0];
            covariance[im][19] = tracker.covarianceMatrix[1][1];
            covariance[im][20] = tracker.covarianceMatrix[2][2];
            covariance[im][21] = tracker.covarianceMatrix[3][3];
            covariance[im][22] = tracker.covarianceMatrix[4][4];
            covariance[im][23] = tracker.covarianceMatrix[5][5];
            covariance[im][24] = im;


            //covariance[im][0] = covMat[0][0] + covMat[1][1] + covMat[2][2];
            //covariance[im][1] = covMat[2][2] + covMat[3][3] + covMat[4][4];

            std::cout << " im " << im << std::endl;
            //pose.saveMatrix(posepath,pose,false,"");
            //covariance.saveMatrix(covariancepath,covariance,false,"");
            timep.saveMatrix(timepath,timep,false,"");
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
            char buf4[FILENAME_MAX];
            sprintf(buf4, opath.c_str(), im-start_image);
            std::string filename4(buf4);

            char buf5[FILENAME_MAX];
            sprintf(buf5, opath1.c_str(), im-start_image);
            std::string filename5(buf5);
            //std::cout << "Write: " << filename4 << std::endl;
            //if(im%5==0)
            {
            vpImageIo::write(Ioverlaycol, filename4);
            //vpImageIo::write(Ioverlay, filename5);
            }

            im++;
            //vpDisplay::getClick(Id,true);
            //A.saveData(0,plotfile0);
            //A.saveData(1,plotfile1);
        }
        //grabber.close();
    }
    catch ( char const *e)
    {
        std::cerr << "Exception: " << e << "\n";
        return 1;
    }
    return EXIT_SUCCESS;


}

