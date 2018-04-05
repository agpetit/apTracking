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

//    std::string truthpath;
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

      opath = vpIoTools::path("out0") + vpIoTools::path("/image%06d.png");
      opath1 = vpIoTools::path("out0") + vpIoTools::path("/imageKalman%06d.png");

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

    vpImage<unsigned char> Id;
    vpImage<unsigned char> Id1;
    vpImage<vpRGBa> Icol;
    vpImage<vpRGBa> Ioverlay;
    vpImage<vpRGBa> Ioverlaycol;

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


    // Main window creation and displaying
    vpDisplayX display1;
    vpDisplayX display2;

    if (opt_display)
    {
        display1.init(Id, 10, 10, "Test tracking L");
        vpDisplay::display(Id) ;
        vpDisplay::flush(Id);
        //if(tracker.getUseRGB())
        {
        display2.init(Icol, 10, 1000, "Test tracking RGB");
        vpDisplay::display(Icol) ;
        vpDisplay::flush(Icol);
        }
    }


    vpHomogeneousMatrix cMo, cMo2, cMoFilt;

    cMo[0][0] = 1;
    cMo[0][1] = 0;
    cMo[0][2] = 0;

    cMo[1][0] = 0;
    cMo[1][1] = 1;
    cMo[1][2] = 0;

    cMo[2][0] = 0;
    cMo[2][1] = 0;
    cMo[2][2] = 1;

    cMo[2][3] = 60;
    cMo[0][3] = 0;
    cMo[1][3] = 0;

    vpImage<vpRGBa> Icol1(height,width);

    Icol1 = Icol;
    for (int i = 0; i<height; i++)
        for (int j = 0; j<width; j++)
        {

            Icol1[i][j].R = 0.2126*Icol[i][j].R + 0.7152*Icol[i][j].G + 0.0722*Icol[i][j].B;
            Icol1[i][j].G = Icol1[i][j].R;
            Icol1[i][j].B = Icol1[i][j].R;

        }
    Icol = Icol1;

    tracker.setPose(cMo);
    tracker.setIprec(Id);
    tracker.cMoprec = cMo;
    tracker.setIprecRGB(Icol);
    tracker.getPose(cMo);
    tracker.setGroundTruth(gdtpath, trueposepath, start_image);
    //tracker.initKltTracker(Id);
    tracker.init(Id,cMo);
    tracker.initComm();

    if (opt_display)
        vpDisplay::flush(Id);
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

    int im=start_image;
    tracker.setFrame(start_image);

    vpColVector error(6);

    cMo2 = cMo;

    int sample = 1;
    double meantime =0;
    int meant = 1;

    vpImage<unsigned char> Ig(height,width);
    double mtime = 0;
    double timetrack, timerender,timeextract,timevvs, timetrack1;
    double timeKalman = 0;
    double t0,t1;
    vpImage<vpRGBa> Icol2(height,width);

    cv::Mat image;
    std::vector<std::vector<point2d>> points2d;
    std::vector<std::vector<point2dd>> normals2d;

    std::vector<std::vector<point2d>> trackededgesIm;
    std::vector<std::vector<int>> suppressIm;

    trackededgesIm.resize(0);
    suppressIm.resize(0);

    vpRotationMatrix Rop;
    Rop.setIdentity();
    vpTranslationVector top;

    vpHomogeneousMatrix opMo;

    tracker.loadPointsNormals2d(points2d, normals2d);

    std::vector<point2d> trackededges;
    std::vector<int> suppress;

    int nimages = points2d.size();

    std::cout << " nimages " << nimages << std::endl;
    // Main tracking loop
    try
    {
        //for (int nimage = 0; nimage < nimages; nimage++)
        {

            vpImageConvert::convert(image,Id);
            vpDisplay::display(Id);
            try{
                t0= vpTime::measureTimeMs();
                //tracker.trackDef2D(Id,Icol,points2d[nimage], normals2d[nimage], trackededges, suppress);
                t1= vpTime::measureTimeMs();

                point2d pt;
                pt.i = 2;
                pt.j = 4;
                trackededges.push_back(pt);
                int supp=8;
                suppress.push_back(supp);

                trackededgesIm.push_back(trackededges);
                suppressIm.push_back(suppress);
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

            std::cout << " im " << im << std::endl;
            vpDisplay::flush(Id);
            vpDisplay::getImage(Id,Ioverlay);
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
            //vpImageIo::write(Ioverlaycol, filename4);
            //vpImageIo::write(Ioverlay, filename5);
            }

            im++;
        }

         tracker.exportCorrespondencesEdges2D(trackededgesIm, suppressIm);
        //grabber.close();
    }
    catch ( char const *e)
    {
        std::cerr << "Exception: " << e << "\n";
        return 1;
    }
    return EXIT_SUCCESS;


}

