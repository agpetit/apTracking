#include <visp/vpOpenCVGrabber.h>
#include <visp/vpVideoReader.h>
#include <stdlib.h>
#include <visp/vpDebug.h>
#include <visp/vpConfig.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpPose.h>
#include <visp/vpPoint.h>
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpImagePoint.h>
#include <visp/vpDot2.h>
#include <visp/vpPixelMeterConversion.h>
//#include <GL/gl.h>
//#include <GL/glu.h>
#include <visp/vpImageConvert.h>


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
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpTime.h>

#include <cv.h>

#include <iostream>
//using namespace std;
using namespace cv;

#include "vpAROgre.h"
#include "apZBuffer.h"
#include "apEdgeMap.h"
#include "apLineExtractor.h"


// ------------------------------------------------------------
//    New class which will define its scene as a simple grid
// ------------------------------------------------------------
class exampleVpAROgre : public vpAROgre
{
public:
	// The constructor doesn't change here
	exampleVpAROgre(vpFrameGrabber *grab, vpCameraParameters *mcam, vpBackgroundType type = BACKGROUND_GREY, int width = 640, int height = 480, char *resourcePath = "")
		: vpAROgre(grab, mcam, type, width, height, resourcePath){}
	
void createScene()
	{
	  Ogre::ManualObject * grid = mSceneMgr->createManualObject("Grid");
	  // Lines
	  for(float i = -0.05; i<0.06; i+=0.01){
	      grid->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
	      grid->position(i,-0.05,0);
	      grid->position(i,0.05,0);
	      grid->end();
	  }
	  // Columns
	  for(float i = -0.05; i<0.06; i+=0.01){
	      grid->begin("BaseWhiteNoLighting",Ogre:: RenderOperation::OT_LINE_STRIP);
	      grid->position(-0.05,i,0);
	      grid->position(0.05,i,0);
	      grid->end();
	  }
	  mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(grid);
	}

};

/*!
This function computes a pose from four black points.
Here to keep dimensions coherency you will need those four dots to be situated at (-5,5,0),(5,5,0),(5,-5,0),(-5,-5,0) (unit = cm) in your real world
*/
void computeInitialPose(vpCameraParameters *mcam, vpImage<unsigned char> &I, vpPose * mPose, vpDot2 *md, vpImagePoint *mcog, vpHomogeneousMatrix *cmo,
			vpPoint *mP)
{
	// ---------------------------------------------------
	//    Code inspired from ViSP example of camera pose
	// ----------------------------------------------------
	bool opt_display = true;
	
	#if defined VISP_HAVE_X11
	  vpDisplayX display;
	#elif defined VISP_HAVE_GTK
	  vpDisplayGTK display;
	#elif defined VISP_HAVE_GDI
	  vpDisplayGDI display;
	#endif
	
}


int main(int argc, char **argv) 
{ 


	  std::string env_ipath;
	  std::string opt_ipath;
	  std::string ipath;
	  std::string opt_configFile;
	  std::string configFile;
	  std::string opt_modelFile;
	  //std::string modelFile;
	  std::string opt_initFile;
	  std::string initFile;
	  bool displayMovingEdge = true;
	  bool opt_click_allowed = true;
	  bool opt_display = true;
      configFile = "cube.xml";
	  //configFile = "amazonas-2-14-gripper.xml";

	  initFile = "cube";
      //initFile = "amazonas-2-14";

	  env_ipath="/local/agpetit/images";
	    // Set the default input path
	      ipath = env_ipath + vpIoTools::path("/ViSP-images/mbt/cube/image%04d.pgm");
	  //ipath = env_ipath + vpIoTools::path("/imagesAmazonas/scenario_GRIPPER_Earth_lum1/I%04d.pgm");



      // OpenCVGrabber to gather images
      vpOpenCVGrabber grabber;
      vpVideoReader reader;

	  //vpCameraParameters mcam(547.7367575,542.0744058,338.7036994,234.5083345);
      vpCameraParameters mcam(547.7367575,542.0744058,339,235);
      //vpCameraParameters mcam(857.48,859.03,322,256);

   	  // Create a vpRAOgre object with color background
   	  exampleVpAROgre ogre(&grabber,&mcam,BACKGROUND_COLOR);

	  ogre.init();
	  ogre.load("Cube","cube.mesh");
	  //ogre.load("Satel", "ShapeIndexedFaceS.002.mesh");
	  //ogre.load("Satel", "Mesh.002.mesh");
	  //ogre.setPosition("Cube", vpTranslationVector(0.0, -0.0, 0.0));
	  ogre.setScale("Cube", 0.00036,0.00036,0.00036);
	  //ogre.setScale("Satel", 0.00002,0.00002,0.00002);
	  //ogre.setScale("Satel", 0.00002,0.00002,0.00002);
	  //ogre.load("Panels","ShapeIndexedFaceS.mesh");
	  //ogre.load("Panels1","ShapeIndexedFaceSet.mesh");
	  //ogre.load("Panels2","ShapeIndexedFaceS.001.mesh");
	  //ogre.load("Panels3","ShapeIndexedFaceS.0020.mesh");
	  //ogre.load("Panels4","ShapeIndexedFaceS.008.mesh");
	  //ogre.load("Panels5","ShapeIndexedFaceS.010.mesh");
	  //ogre.setScale("Panels", 0.02,0.02,0.02);
	  //ogre.setRotation("Cube", vpRotationMatrix(vpRxyzVector(M_PI/4, -M_PI/8, -M_PI/4)));
	  /*ogre.setRotation("Satel", vpRotationMatrix(vpRxyzVector(M_PI/2, M_PI, M_PI)));
	  ogre.setRotation("Panels2", vpRotationMatrix(vpRxyzVector(M_PI/2,M_PI, M_PI)));
	  ogre.setRotation("Panels3", vpRotationMatrix(vpRxyzVector(M_PI/2,M_PI, M_PI)));
	  ogre.setPosition("Satel", vpTranslationVector(0.05, -0.00, 0.0));
	  	  ogre.setPosition("Panels2", vpTranslationVector(0.05, -0.00, 0.0));
	  	  ogre.setPosition("Panels3", vpTranslationVector(0.05, -0.00, 0.0));*/

	  vpImage<unsigned char> Id;
	  reader.setFileName(ipath.c_str());
	  reader.open(Id);
	  //for (int m=0;m<300;m++){
	  reader.acquire(Id);
	  //}
	  /*grabber.setDeviceType(1);
	  grabber.open(Id);
      grabber.acquire(Id);*/

	// Grey level image to track points
	vpImage<unsigned char> I;
	// RGBa image to get background
	vpImage<vpRGBa> IC;
	vpImage<double> I1;
	vpImage<unsigned char> I2;
	vpImage<unsigned char> I3(480,640);
	vpHomogeneousMatrix cmo;
	// Matrix representing camera parameters
	vpRxyzVector r(0,0,0);
	vpRotationMatrix R(r);
	vpTranslationVector t(0,0,0.6);
	cmo.buildFrom(t,R);

	      vpDisplayX display1;
	      vpDisplayX display;
	      vpDisplayX display2;
	      vpDisplayX display3;
	      I.resize(480,640);
	      I1.resize(480,640);
	      I2.resize(480,640);
	      I3.resize(480,640);
	    display1.init(I, 10, 10, "X11 display");
	    display2.init(I2, 1500, 10, "X11 display");
	    display3.init(I3, 1500, 1000, "X11 display");
	    if (opt_display)
	    {
	      display.init(Id, 10, 1000, "Test tracking") ;
	      vpDisplay::display(Id) ;
	      vpDisplay::flush(Id);
	    }

   	  apZBuffer zbuf;
   	  apLineExtractor LEx;
   	  LEx.setParameterSpace(180,400,I2);
   	  zbuf.set(480,640,0.1,1);

   		//exampleVpAROgre ogre2(&grabber,&mcam,BACKGROUND_COLOR);
   		// Initialize it

	// CameraParameters we got from calibration
	// Keep u0 and v0 as center of the screen

	//vpHomogeneousMatrix cMo(0,0,0.5,3*M_PI/2,0,0);
   	vpHomogeneousMatrix cMo;//(0.008924150268,-0.03937067413,1.859247036,0.2544105297,-0.213655404-M_PI,2.278629041-M_PI);
   	//vpHomogeneousMatrix cMo(0.008924150268,-0.03937067413,1.859247036,-M_PI/2,-M_PI/4,M_PI/2+M_PI/3);
	 vpMbEdgeTracker tracker;
	 tracker.loadConfigFile(configFile);
	   // Display the moving edges, see documentation for the significations of the colour
	   tracker.setDisplayMovingEdges(displayMovingEdge);
	   // initialise an instance of vpCameraParameters with the parameters from the tracker
	   tracker.getCameraParameters(mcam);
	   // Loop to position the cube

	   if (opt_display && opt_click_allowed)
	   {
	     while(!vpDisplay::getClick(Id,false)){
	     vpDisplay::display(Id);
	     vpDisplay::displayCharString(Id, 15, 10,
	 		 "click after positioning the object",
	 		 vpColor::red);
	     vpDisplay::flush(Id) ;
	     }
	   }

	   // Load the 3D model (either a vrml file or a .cao file)

	   // Initialise the tracker by clicking on the image
	   // This function looks for
	   //   - a ./cube/cube.init file that defines the 3d coordinates (in meter, in the object basis) of the points used for the initialisation
	   //   - a ./cube/cube.ppm file to display where the user have to click (optionnal, set by the third parameter)
	   if (opt_display && opt_click_allowed)
	   {
	     tracker.initClick(Id, initFile.c_str(), true);

	     // display the 3D model at the given pose
	     //tracker.display(Id,cMo, cam, vpColor::red);
	   }
	   else
	   {
	     vpHomogeneousMatrix cMoi(-0.002774173802,-0.001058705951,0.2028195729,2.06760528,0.8287820106,-0.3974327515);
	     tracker.init(Id,cMoi);
	   }

	     //track the model
	   //tracker.track(Id);
	   tracker.getPose(cMo);
	   //cmo=cMo;
       cout<<cMo<<endl;
	   if (opt_display)
	     vpDisplay::flush(Id);


	// Variables used for pose computation purposes
	IplImage* Ip = NULL;
	IplImage *Ip1;
	Mat dst;
	Mat color_dst;
	vector<Vec4i> lines;
	std::vector< vpMbtDistanceLine* > Lines;
	vpMatrix Zc;
	// Change your device number here depending on which number your device is (video0, video1,...) 
	//grabber.setDeviceType(1);
	//grabber.open(Idisplay);
	//grabber.acquire(Idisplay);
	// Compute the initial pose of the camera
	//computeInitialPose(&mcam, Idisplay, &mPose, md, mcog, &cmo, mP);
	// Close the framegrabber
	//grabber.close();
	//grabber.open(Id);

	// Associate the grabber to the RGBa image
vpMatrix Vm;
vpTranslationVector tr;
double t0;
double t1;
double t2;

	try
	{
		// Rendering loop
		while(ogre.continueRendering()){


			// Display with ogre
			            t0= vpTime::measureTimeMs();
			            cMo.extract(tr);
			            ogre.updateClipDistances(tr[2]);
			            ogre.display(IC,&cMo);
			            //t1= vpTime::measureTimeMs();
			            cout<<"dt render = "<<t1-t0<<endl;
			            t1= vpTime::measureTimeMs();
                        zbuf.getDepth(I1,I);
                        vpDisplay::display(I);
                        vpDisplay::flush(I);
                        //getchar();
			            apEdgeMap::edgeMapLap(I1,I2,0.5);
                        //apEdgeMap::edgeMapLap(I,I2,1);
			            t2= vpTime::measureTimeMs();
			            cout<<"dt disp = "<<t2-t1<<endl;
                        vpImageConvert::convert(I2,Ip);
                        Mat dst(Ip);
                        /*LEx.buildHoughVote(I2);
                        vpDisplay::display(I2);
                        vpDisplay::flush(I2);
                        vpDisplay::display(I3);
                       LEx.setLinePoints(I3,30,zbuf,mcam);
                        //vpDisplay::displayLine(I3,ip1,ip2,vpColor::red,3);
                        vpDisplay::flush(I3);*/
                        //dst=Ip;
                        //namedWindow( "Detected Edges", 1 );
                        //imshow( "Detected Edges", dst );
                        cvtColor( dst, color_dst, CV_GRAY2BGR );
                        HoughLinesP(dst, lines, 1, CV_PI/180, 20, 10, 4);
                        t1= vpTime::measureTimeMs();
                        cout<<"dt hough = "<<t1-t2<<endl;
                        for( size_t i = 0; i < lines.size(); i++)
                            {
                                line( color_dst, Point(lines[i][0], lines[i][1]),
                                    Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 2, 8 );
                            }
                        cout<<lines[0][0]<<" "<<lines[0][1]<<" "<<lines[0][2]<<" "<<lines[0][3]<<endl;
                       //LEx.setLines(lines,zbuf,cam);
                       //Lines=LEx.getLines();

                        //cout<<lines.size()<<endl;
                            Ip1=new IplImage(color_dst);
                            vpImageConvert::convert(Ip1,I3);
                            vpDisplay::display(I3);
                            vpDisplay::flush(I3);
                            //vpDisplay::getClick(Id,true);
                            Zc=zbuf.getZ(tr[2]);
                            tracker.loadLines(Id,lines,Zc,cMo);
                            //vpDisplay::flush(Id);
                            getchar();


                            try
                           		    {
                           		      // acquire a new image
                           		      //grabber.acquire(Id);
                            	      reader.acquire(Id);
                           		      // display the image
                           		      if (opt_display)
                           		        vpDisplay::display(Id);
                           		      // track the object
                           		      tracker.track(Id);
                           		      tracker.getPose(cMo);
                           		      //cout<<cMo<<endl;
                           		      // display the 3D model
                           		      if (opt_display)
                           		      {
                           		        tracker.display(Id, cMo, mcam, vpColor::darkRed, 2);
                           		     //getchar();
                           		        // display the frame
                           		        vpDisplay::displayFrame (Id, cMo, mcam, 0.05, vpColor::blue);
                           		      }
                           		    }
                           		    catch(...)
                           		    {
                           		      std::cout << "error caught" << std::endl;
                           		      break;
                           		    }
                           		    tracker.resetLines();
                           		    vpDisplay::flush(Id) ;
                           		 t2= vpTime::measureTimeMs();
                           		 cout<<"dt total "<<t2-t0<<endl;
                           		 //getchar();




              //Vm=LEx.getVMatrix();
              //cout<<"ok"<<endl;
              //Vm.print(std::cout, 1);
              /*for(int u=0;u<Vm.getRows();u++){
            	  for(int v=0;v<Vm.getCols();v++){
            		  if(Vm[u][v]>100){
            			  cout<<u<<" "<<v<<endl;
            		  }
            	  }
            	  }*/

		      // getchar();
 
		}
		// Close the framegrabber
		 grabber.close();
	}
	catch (Ogre::Exception& e)
	{
	        std::cerr << "Exception:\n";
	       	std::cerr << e.getFullDescription().c_str() << "\n";
		return 1;	
	}
	catch ( char const *e)
	{
		std::cerr << "Exception: " << e << "\n";
		return 1;
	}
	return EXIT_SUCCESS;


}

