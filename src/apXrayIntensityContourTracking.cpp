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

#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


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

#include "apMbTracker.h"
#include "apDetector.h"
#include "apViews.h"
#include "apSegmentation.h"
#include "apKalmanFilter.h"
#include "apKalmanFilterSE3.h"
#include "apKalmanFilterSE33.h"
#include "apKalmanFilterQuat.h"
#include "apHOGDetector.h"

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

#define GETOPTARGS  "o:i:j:c:s:l:m:I:dh"


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
                std::string &ipathN,
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
    case 'j': ipathN = optarg; break;
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
    std::string opt_ipath,opt_ipathN;
    std::string ipath, ipathN;
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
    posepath = "poseerror";//atlantis.txt";
    covariancepath = "/home/soft/apTracking/mbt-co/build2/covariance";
    timepath = "time";



    // Read the command line options
    if (!getOptions(argc, argv, opt_object, opt_ipath, opt_ipathN, opt_display, start_image, configFile, opt_learn, opt_detect, inline_init)) {
      return (-1);
    }

    opt_display = true;
    // Get the option values

     if (!opt_ipath.empty())
     {
       ipath = opt_ipath;// + vpIoTools::path("/image%06d.png");
//       truthpath = opt_ipath + vpIoTools::path("/truth.txt");
       ipathN = opt_ipathN;// + vpIoTools::path("/image%06d.png");

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

      //std::string filep = vpIoTools::path("mh_ccd_klt_prev_x1_f1_h2.txt");

      std::string filep = vpIoTools::path("_1.txt");


      posepath = posepath + object + filep;
      covariancepath = covariancepath + object + filep;
      timepath = timepath + object + filep;


      //configFile2 = object + vpIoTools::path("/") + object + vpIoTools::path("2.lua");

    opath = vpIoTools::path("out60") + vpIoTools::path("/image%06d.png");
    opath1 = vpIoTools::path("out60") + vpIoTools::path("/imageKalman%06d.png");


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

    // OpenCVGrabber to grab images from USB Camera
    //vpOpenCVGrabber grabber;

    SceneViewer viewer;
    SceneManager *mgr = new SceneManager(const_cast<QGLContext*>(viewer.context()));
    viewer.setSceneManager(mgr);
    viewer.show();
    viewer.move(200, 200);

    vpImage<unsigned char> Id,IdFlip;
    vpImage<unsigned char> IdN, IdNFlip;
    vpImage<vpRGBa> Icol, IcolFlip;
    vpImage<vpRGBa> Ioverlay;
    vpImage<vpRGBa> Ioverlaycol;

        std::cout << " reader " << ipath.c_str() << std::endl;

    //VideoReader to read images from disk
    vpVideoReader reader;
    reader.setFileName(ipath.c_str());
    reader.setFirstFrameIndex(start_image);
    reader.open(Id);
    reader.acquire(Id);

    //VideoReader to read images from disk
    vpVideoReader readerN;
    readerN.setFileName(ipathN.c_str());
    readerN.setFirstFrameIndex(start_image);
    readerN.open(IdN);
    readerN.acquire(IdN);


    vpVideoReader readerRGB;

    //if(tracker.getUseRGB())
    {
    	readerRGB.setFileName(ipath.c_str());
        readerRGB.setFirstFrameIndex(start_image);
        readerRGB.open(Icol);
        readerRGB.acquire(Icol);
    }



    cv::Mat id, idflip;
    cv::Mat icol, icolflip;
    cv::Mat idn, idnflip;

    vpImageConvert::convert(IdN,idn);
    vpImageConvert::convert(Id,id);
    vpImageConvert::convert(Icol,icol);

    cv::flip(idn,idnflip,0);
    cv::flip(id,idflip,0);
    cv::flip(icol,icolflip,0);

    vpImageConvert::convert(idnflip,IdNFlip);
    vpImageConvert::convert(idflip,IdFlip);
    vpImageConvert::convert(icolflip,IcolFlip);


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
        display1.init(IdN, 10, 10, "Test tracking");
        vpDisplay::display(IdN) ;
        vpDisplay::flush(IdN);
        //if(tracker.getUseRGB())
        {
        display2.init(Icol, 10, 1000, "Test tracking");
        vpDisplay::display(Icol) ;
        vpDisplay::flush(Icol);
        }
    }

    vpHomogeneousMatrix cMo, cMct, cMo2, cMoFilt;

       // pose real image liver1.png -222.375 154.59 379.542 [0.989822 -3.09578e-17 0.142311,0.136547 0.281733 -0.949727,-0.0400937 0.959493 0.278865

       cMct[0][3] =-209.9189609;
       cMct[1][3] = 201.4935037;
       cMct[2][3] = 390.7839107;
       cMct[0][0] = 0.9958088264;
       cMct[0][1] = 0.08254072254;
       cMct[0][2] = 0.03939508347;
       cMct[1][0] = 0.01511400765;
       cMct[1][1] = 0.2763081441;
       cMct[1][2] = -0.9609501357;
       cMct[2][0] = -0.09020270353;
       cMct[2][1] = 0.9575182186;
       cMct[2][2] = 0.2739020766;


       /*cMct[0][3] =-149.9189609;
       cMct[1][3] = 200.4935037;
       cMct[2][3] = 440.7839107;
       cMct[0][0] = 0.9958088264;
       cMct[0][1] = 0.08254072254;
       cMct[0][2] = 0.03939508347;
       cMct[1][0] = 0.01511400765;
       cMct[1][1] = 0.2763081441;
       cMct[1][2] = -0.9609501357;
       cMct[2][0] = -0.09020270353;
       cMct[2][1] = 0.9575182186;
       cMct[2][2] = 0.2739020766;*/

       // pose real image liver2.png -222.375 154.59 379.542 [0.989822 -3.09578e-17 0.142311,0.136547 0.281733 -0.949727,-0.0400937 0.959493 0.278865

       /*0.9216657489  -0.3875581562  0.01819937037  -27.08472031
       -0.0003092703658  -0.04764536662  -0.9988638197  288.357135
       0.3879851763  0.9206133542  -0.04403458774  459.7173749*/

       cMct[0][0] = 0.9216657489;
       cMct[0][1] =-0.3875581562;
       cMct[0][2] = 0.01819937037;
       cMct[0][3] = -27.08472031;
       cMct[1][0] = -0.0003092703658;
       cMct[1][1] = -0.04764536662;
       cMct[1][2] = -0.9988638197;
       cMct[1][3] = 288.357135;
       cMct[2][0] = 0.3879851763;
       cMct[2][1] = 0.9206133542;
       cMct[2][2] = -0.04403458774;
       cMct[2][3] = 459.7173749;


       // pose cMct liverarterial

       /*0.9727540217  -0.2313178327  0.01557545484  -102.8474708
       0.001544650804  -0.06076014755  -0.9981489679  286.2505651
       0.2318372463  0.9709801039  -0.05875412225  472.1949346
       0  0  0  1*/

       cMct[0][0] = 0.9727540217;
       cMct[0][1] =-0.2313178327;
       cMct[0][2] = 0.01557545484;
       cMct[0][3] = -102.8474708;
       cMct[1][0] = 0.001544650804;
       cMct[1][1] = -0.06076014755;
       cMct[1][2] = -0.9981489679;
       cMct[1][3] = 286.2505651;
       cMct[2][0] = 0.2318372463;
       cMct[2][1] = 0.9709801039 ;
       cMct[2][2] = -0.05875412225;
       cMct[2][3] = 472.1949346;

       // pose cMct liverarterial2

       /*0.9975085117  -0.06574516179  0.02563329375  -164.6963909
       0.004403865048  -0.3049865928  -0.9523383771  332.0290646
       0.07043377305  0.9500841341  -0.3039573124  487.3573959*/

       cMct[0][0] = 0.9975085117;
       cMct[0][1] =-0.06574516179;
       cMct[0][2] =0.02563329375 ;
              cMct[0][3] =-164.6963909;
              cMct[1][0] = 0.004403865048;
              cMct[1][1] =-0.3049865928;
              cMct[1][2] = -0.9523383771;
              cMct[1][3] = 332.0290646;
              cMct[2][0] = 0.07043377305;
              cMct[2][1] = 0.9500841341;
              cMct[2][2] = -0.3039573124;
              cMct[2][3] = 487.3573959;


              /*0.9707461703  -0.2400005999  0.007207701924  -118.9353003
              0.03417684341  0.1678255873  0.985224089  -303.1761337
              -0.2376635332  -0.9561555736  0.1711187519  855.7857378*/

              cMct[0][0] =0.9707461703;
              cMct[0][1] =-0.2400005999;
              cMct[0][2] = 0.007207701924;
                     cMct[0][3] =-118.935300;
                     cMct[1][0] = 0.03417684341;
                     cMct[1][1] =0.1678255873;
                     cMct[1][2] = 0.985224089 ;
                     cMct[1][3] = -303.1761337;
                     cMct[2][0] = -0.2376635332;
                     cMct[2][1] = -0.9561555736;
                     cMct[2][2] = 0.1711187519;
                     cMct[2][3] = 855.7857378;




              apHOGDetector HOGDetector;

               std::string pos_dir, pos, neg_dir, neg, path_pose, path_pose_gt;

               pos_dir = "/home/antoine/soft/Liver/";
               pos = "Train/pos.lst";
               neg_dir = "/home/antoine/soft/Liver/";
               neg = "Train/neg.lst";

               path_pose = "/home/antoine/soft/Liver/Train/posLiver4/poses.txt";
               path_pose_gt = "/home/antoine/soft/Liver/Test/images/poses.txt";


               std::ifstream file;

               std::string line;

               file.open(path_pose_gt.c_str(), std::ifstream::in);
               double dvalue0,dvalue1,dvalue2,dvalue3,dvalue4,dvalue5,dvalue6,dvalue7,dvalue8,dvalue9,dvalue10,dvalue11;

               vpMatrix truePose(700,6);

               for (int i = 0; i < 700; i++)
               {


                   std::string filename4;
                   getline(file, line);

                   //std::cout << line << std::endl;

                   file >> filename4;
                   file >> dvalue0;
                   file >> dvalue1;
                   file >> dvalue2;
                   file >> dvalue3;
                   file >> dvalue4;
                   file >> dvalue5;
                   file >> dvalue6;
                   file >> dvalue7;
                   file >> dvalue8;
                   file >> dvalue9;
                   file >> dvalue10;
                   file >> dvalue11;


                   vpTranslationVector tr;
                   vpRotationMatrix Rot;

                   tr[0] = dvalue0;
                   tr[1] = dvalue1;
                   tr[2] = dvalue2;

                   Rot[0][0] = dvalue3;
                   Rot[0][1] = dvalue4;
                   Rot[0][2] = dvalue5;
                   Rot[1][0] = dvalue6;
                   Rot[1][1] = dvalue7;
                   Rot[1][2] = dvalue8;
                   Rot[2][0] = dvalue9;
                   Rot[2][1] = dvalue10;
                   Rot[2][2] = dvalue11;

                   vpHomogeneousMatrix cMo, cMoTrans;
                   cMo.buildFrom(tr,Rot);

                   vpRxyzVector rottrue;
                   rottrue.buildFrom(Rot);

                   truePose[i][0] = tr[0];
                   truePose[i][1] = tr[1];
                   truePose[i][2] = tr[2];

                   truePose[i][3] = rottrue[0];
                   truePose[i][4] = rottrue[1];
                   truePose[i][5] = rottrue[2];
               }

               tracker.setTruePose(truePose);

              // Automatic initialization of the tracker
              if (opt_detect)
              {

                if(opt_learn)
                {
                  HOGDetector.train(pos_dir, pos, neg_dir, neg);
                }

                  HOGDetector.detect(IdN, path_pose, cMct);

              }

              //001_test.png -192.168 286.001 360 1 -8.15615e-22 -3.67321e-06 -3.67321e-06 -9.95799e-17 -1 4.49838e-22 1 -9.95799e-17

              /*cMct[0][0] =1;
              cMct[0][1] =-8.15615e-22;
              cMct[0][2] = -3.67321e-06;
                     cMct[0][3] =-192.168;
                     cMct[1][0] = -3.67321e-06;
                     cMct[1][1] =-9.95799e-17 ;
                     cMct[1][2] = -1 ;
                     cMct[1][3] = 286.001;
                     cMct[2][0] = 4.49838e-22;
                     cMct[2][1] = 1;
                     cMct[2][2] = -9.95799e-17 ;
                     cMct[2][3] = 360;*/


        vpRxyzVector vdelta(-0.0,-0.0,-0.0);
        vpTranslationVector tdelta(0,0,-0);

        vpRxyzVector vdelta1(-0.0,0.0,-0.0);
        vpTranslationVector tdelta1(0,0,0);

        vpRotationMatrix Rdelta;
        Rdelta.buildFrom(vdelta);
        vpHomogeneousMatrix Mdelta;
        Mdelta.buildFrom(tdelta,Rdelta);

        vpRotationMatrix Rdelta1;
        Rdelta1.buildFrom(vdelta1);
        vpHomogeneousMatrix Mdelta1;
        Mdelta1.buildFrom(tdelta1,Rdelta1);

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

    vpHomogeneousMatrix oMct;

    vpQuaternionVector qoct;
    vpRotationMatrix Roct;

    vpHomogeneousMatrix ctMo = oMct.inverse();

    vpHomogeneousMatrix ctMo1, o1Mo, oMo1, o1Mct ;

    ctMo1[0][0] = 1;
    ctMo1[0][1] =  0;
    ctMo1[0][2] =  0;
    ctMo1[0][3] = 192.633;
    ctMo1[1][0] = 0;
    ctMo1[1][1] = 1;
    ctMo1[1][2] =  0;
    ctMo1[1][3] = 313.133;
    ctMo1[2][0] = 0;
    ctMo1[2][1] = 0;
    ctMo1[2][2] = 1;
    ctMo1[2][3] = 967.5;

    vpRxyzVector vdelta2(-0.0,0.0,M_PI);
    vpTranslationVector tdelta2(0,0,0);

    vpRotationMatrix Rdelta2;
    Rdelta2.buildFrom(vdelta2);
    vpHomogeneousMatrix Mdelta2;
    Mdelta2.buildFrom(tdelta2,Rdelta2);

    ctMo1 = ctMo1*Mdelta2;


    oMo1[0][0] = 1;
    oMo1[0][1] =  0;
    oMo1[0][2] =  0;
    oMo1[0][3] = 12.50376;
    oMo1[1][0] = 0;
    oMo1[1][1] = 1;
    oMo1[1][2] =  0;
    oMo1[1][3] = -109.75430;
    oMo1[2][0] = 0;
    oMo1[2][1] = 0;
    oMo1[2][2] = 1;
    oMo1[2][3] = 663.00940;

    o1Mo = oMo1.inverse();

    ctMo = ctMo1*o1Mo;

    oMct = ctMo.inverse();

    Roct.buildFrom(oMct);
    qoct.buildFrom(Roct);
    vpRxyzVector rxyz;
    rxyz.buildFrom(Roct);

    std::cout << "ctmo " << oMct.inverse() << " q oct " <<  oMct <<  std::endl;

    cMo = cMct*oMct.inverse();

    cMo = Mdelta*cMo*Mdelta1;

    cMct = cMo*oMct;


    tracker.setPose(cMo);


    // Manual initialization of the tracker

    /*if (opt_display && opt_click_allowed && !opt_detect)
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
    }*/


    tracker.setIprec(Id);
    tracker.cMoprec = cMo;
    tracker.setIprecRGB(Icol);
    tracker.getPose(cMo);
    tracker.setGroundTruth(gdtpath, trueposepath, start_image + 2);
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

    /*-0.9725939864  0.07627547874  -0.2196428671  -0.04488710902
    -0.05719987694  -0.994120001  -0.09194344866  -0.1764133453
    -0.2253643978  -0.07686010028  0.9712380827  6.745699447*/

    /*cMo[0][3] = -0.04488710902;
    cMo[1][3] = -0.1764133453;
    cMo[2][3] = 6.745699447;
    cMo[0][0] = -0.9725939864;
    cMo[0][1] = 0.07627547874;
    cMo[0][2] = -0.2196428671;
    cMo[1][0] = -0.05719987694;
    cMo[1][1] = -0.994120001;
    cMo[1][2] = -0.09194344866;
    cMo[2][0] = -0.2253643978;
    cMo[2][1] = -0.07686010028;
    cMo[2][2] = 0.9712380827;
    oMct = cMo.inverse()*cMct;

    std::cout << " cMo " << cMo << " cMct " <<  cMct << " oMct " << oMct << std::endl;*/

   // getchar();
    if (opt_display)
        vpDisplay::flush(IdN);
    double px = mcam.get_px() ;
    double py = mcam.get_py() ;

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

    vpHomogeneousMatrix cMobj;

    std::cout << " cmct " << cMct << std::endl;

    // Main tracking loop
    try
    {
        while(true){

            // Render the 3D model, get the depth edges, normal and texture maps
            try{
                std::cout << "render " << std::endl;
                tracker.getPose(cMo);
                t0= vpTime::measureTimeMs();
                mgr->updateRTTCol(Icol2,Inormd,Ior,&cMo);
                t1= vpTime::measureTimeMs();
                timerender = t1-t0;
                std::cout << "timerender " << t1 - t0 << std::endl;
                a.processEvents(QEventLoop::AllEvents, 1);
                //vpImageIo::writePNG(Inormd, "Inormd.png");
                //vpImageIo::writePNG(Ior, "Ior.png");
                tracker.Inormdprec = Inormd;
                tracker.Iorprec = Ior;
                tracker.Itexprec = Ior;
                tracker.cMoprec = cMo;
                tracker.oMct = oMct;

            }
            catch(...){
                vpTRACE("Error in 3D rendering");
                throw;
            }


            t0= vpTime::measureTimeMs();

            if(useKalmanFilter)
            {
                filt.cMoEst = cMo;
                filt.predictPose();
                //filt.getPredPose(cMo);
                //tracker.setPose(cMo);
                tracker.predictKLT = true;
                mgr->updateRTTCol(Icol2,Inormd,Ior,&cMo);
                a.processEvents(QEventLoop::AllEvents, 1);
            }

            t1= vpTime::measureTimeMs();
            cout << "timeKalman "<<t1-t0<<endl;
            timeKalman = t1-t0;

            cMct = cMo*oMct;
            std::cout << "cMct " << cMct << std::endl;


            vpDisplay::display(Icol);
            std::cout << " disp " << im-start_image << std::endl;

            tracker.itert = im-start_image;
            if(im-start_image>0)
            tracker.displayRend(Icol,Inormd,Ior,vpColor::green, 1);
            vpDisplay::flush(Icol);
            vpDisplay::getImage(Icol,Ioverlaycol);


            // Acquire images
            try{
                for (int sp = 0; sp < sample ; sp++)
                {

                    //if (im < 20)
                    {
                    //reader.acquire(Id);
                    //readerN.acquire(IdN);
                    //if(tracker.getUseRGB())
                    //readerRGB.acquire(Icol);

                    vpImageConvert::convert(IdN,idn);
                    vpImageConvert::convert(Id,id);
                    vpImageConvert::convert(Icol,icol);


                    cv::flip(idn,idnflip,0);
                    cv::flip(id,idflip,0);
                    cv::flip(icol,icolflip,0);


                    vpImageConvert::convert(idnflip,IdNFlip);
                    vpImageConvert::convert(idflip,IdFlip);
                    vpImageConvert::convert(icolflip,IcolFlip);


                    }
                        Icol1 = Icol;
                        for (int i = 0; i<height; i++)
                            for (int j = 0; j<width; j++)
                            {

                                Icol1[i][j].R = 0.2126*Icol[i][j].R + 0.7152*Icol[i][j].G + 0.0722*Icol[i][j].B;
                                Icol1[i][j].G = Icol1[i][j].R;
                                Icol1[i][j].B = Icol1[i][j].R;

                            }
                        //Icol = Icol1;
                }
            }
            catch(...){
                break;
            }
            //vpImageIo::read(Id,"imagePig3.png");

            std::cout << "cMct " << cMct << std::endl;

            vpDisplay::display(IdN);

            //tracker.setPose(cMo);
            cMo.extract(tr);
            // Pose tracking
            try{
                t0= vpTime::measureTimeMs();

                //tracker.trackXray(Id, tr[2]);
                tracker.IdN = IdN;
                if (tracker.itert>0)
                tracker.trackXrayIntensityContour(Id,Icol,Inormd,Ior,Ior,tr[2]);

                //tracker.displayKltPoints(Id);
                //tracker.track(Id, Igrad, Igradx, Igrady, Inormd, Ior, Ior, tr[2]);
                {
                    tracker.getCovarianceMatrix(covMat);
                    tracker.getCovarianceMatrixME(covMatME);
                }

                {
                    tracker.getCovarianceMatrix(covMat);
                    tracker.getCovarianceMatrixME(covMatME);
                }
                if (useKalmanFilter && tracker.itert > 10)
                {
                    tracker.getPose(cMo);
                    tracker.display(Id,cMo,mcam,vpColor::red,1);
                    tracker.getCovarianceMatrix(covMat);
                    tracker.getCovarianceMatrixME(covMatME);
                    filt.estimatePose(cMo,covMat);
                    tracker.setPose(filt.cMoEst);
                    cMoFilt = filt.cMoEst;
                    cMo = cMoFilt;
                    tracker.display(Id,cMoFilt,mcam,vpColor::red,1);
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

            cMct = cMo*oMct;
            // Display 3D model
            tracker.getPose(cMo);
            tracker.computeError(error,cMct);
            tracker.display(Id,cMo,mcam,vpColor::green,1);

            std::cout<<" cMo out" << cMo <<std::endl;
            std::cout<<" cMo filt" << cMoFilt <<std::endl;
            std::cout<<" error " << error <<std::endl;


            cMo.extract(tr);
            cMo.extract(R);
            Rxyz.buildFrom(R);
            pose.resize(im+1,7,false);
            covariance.resize(im+1,25,false);
            timep.resize(im+1,9,false);
            /*pose[im][0] = tr[0];
            pose[im][1] = tr[1];
            pose[im][2] = tr[2];
            pose[im][3] = Rxyz[0];
            pose[im][4] = Rxyz[1];
            pose[im][5] = Rxyz[2];*/
            pose[im][0] = error[0];
            pose[im][1] = error[1];
            pose[im][2] = error[2];
            pose[im][3] = error[3];
            pose[im][4] = error[4];
            pose[im][5] = error[5];
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
            pose.saveMatrix(posepath,pose,false,"");
            //covariance.saveMatrix(covariancepath,covariance,false,"");
            timep.saveMatrix(timepath,timep,false,"");
            /*mgr->updateRTT(Inormd,Ior,&cMo2);
            tracker2.setPose(cMo2);
            cMo2.extract(tr2);
            tracker2.track(Id,Icol,Inormd,Ior,Ior,tr2[2]);
            tracker2.getPose(cMo2);
            tracker2.display(Id,cMo2,mcam,vpColor::red,1);*/

            vpDisplay::flush(IdN);
            vpDisplay::getImage(IdN,Ioverlay);
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
            vpImageIo::write(Icol2, filename5);
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

