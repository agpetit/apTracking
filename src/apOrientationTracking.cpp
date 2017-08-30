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
#include <visp/vpExponentialMap.h>


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
#include <visp/vpFeatureLuminance.h>
#include <visp/vpPlot.h>

#include <cv.h>

#include <iostream>
//using namespace std;
using namespace cv;

#include "vpAROgre.h"
#include "apZBuffer.h"
#include "apEdgeMap.h"
#include "apLineExtractor.h"
#include "vpFeatureGradient.h"
#include "apFeatureGradientOrientation.h"


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
	  std::string opath;
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

      // Define the config file
      configFile = "cube.xml";
	  //configFile = "amazonas-2-14-gripper.xml";

	  // Initialization file
      initFile = "cube";
      //initFile = "amazonas-2-14";
      //initFile = "amazonas-2-15";

	  env_ipath="/local/agpetit/images";
	    // Set the default input path
	     // ipath = env_ipath + vpIoTools::path("/ViSP-images/mbt/cube/image%04d.pgm");
	  //ipath = env_ipath + vpIoTools::path("/imagesAmazonas/scenario_GRIPPER_speed0009/I%04d.pgm");
	  ipath = env_ipath + vpIoTools::path("/ViSP-images/calib/I%04d.pgm");
	  opath = env_ipath + vpIoTools::path("/ViSP-images/orientgeom/I%04d.pgm");



      // OpenCVGrabber to grab images
      vpOpenCVGrabber grabber;
      // VideoReader to read images
      vpVideoReader reader;

      //Define camera parameters

	  //vpCameraParameters mcam(547.7367575,542.0744058,338.7036994,234.5083345);
      vpCameraParameters mcam(547.7367575,542.0744058,320,240);
      //vpCameraParameters mcam(857.48,859.03,322,256);
      //vpCameraParameters mcam(857.48,859.03,322,280);

   	  // Create a vpRAOgre object with color background
   	  exampleVpAROgre ogre(&grabber,&mcam,BACKGROUND_COLOR);

	  ogre.init();
	  ogre.load("Cube","cube.mesh");
	  //ogre.load("Satel", "ShapeIndexedFaceS.002.mesh");
	  //ogre.load("Satel", "Mesh.004.mesh");
	  //ogre.load("Satel", "Mesh.148.mesh");
	  //ogre.setRotation("Satel", vpRotationMatrix(vpRxyzVector(M_PI/2,M_PI,0)));
	  //ogre.load("Satel", "Mesh.002.mesh");
	  //ogre.setPosition("Cube", vpTranslationVector(0.0, -0.0, 0.0));
	  ogre.setScale("Cube", 0.00036,0.00036,0.00036);
	    //ogre.setScale("Cube", 0.004,0.004,0.004);
	  //ogre.setScale("Satel", 2,2,2);


	  vpImage<unsigned char> Id;
	  vpImage<unsigned char> Id1;
	  vpImage<unsigned char> I;
	  vpImage<vpRGBa> IC;
	  vpImage<double> I1;
	  vpImage<unsigned char> I2;
	  vpImage<unsigned char> I3(480,640);
	  vpImage<vpRGBa> I4(480,640);
	  vpImage<vpRGBa> I5(480,640);
      vpImage<unsigned char> Idiff ;

	  reader.setFileName(ipath.c_str());
	  reader.open(Id);
	  /*for (int m=0;m<500;m++){
	  reader.acquire(Id);
	  }*/
	 /* grabber.setDeviceType(1);
	  grabber.open(Id);
      grabber.acquire(Id);*/
	  vpImageIo::readPGM(Id,"imageV.pgm");

	      vpDisplayX display1;
	      vpDisplayX display;
	      vpDisplayX display2;
	      vpDisplayX display3;
	      I.resize(480,640);
	      I1.resize(480,640);
	      I2.resize(480,640);
	      I4.resize(480,640);
	      Idiff.resize(480,640);
	    display1.init(I, 10, 10, "X11 display");
	    display2.init(Idiff, 1500, 10, "X11 display");
	    display3.init(I3, 1500, 1000, "X11 display");
	    //display3.init(I2, 1500, 1000, "X11 display");
	    if (opt_display)
	    {
	      display.init(Id, 10, 1000, "Test tracking") ;
	      vpDisplay::display(Id) ;
	      vpDisplay::flush(Id);
	    }

   	  apZBuffer zbuf;
   	  zbuf.set(480,640,0.1,1);

   		//exampleVpAROgre ogre2(&grabber,&mcam,BACKGROUND_COLOR);
   		// Initialize it

	// CameraParameters we got from calibration
	// Keep u0 and v0 as center of the screen

   	vpHomogeneousMatrix cMo;//(0.008924150268,-0.03937067413,1.859247036,0.2544105297,-0.213655404-M_PI,2.278629041-M_PI);
   	//vpHomogeneousMatrix cMo(0,0,1.859247036,0,0,0);
	 vpMbEdgeTracker tracker;
	 tracker.loadConfigFile(configFile);
	   // Display the moving edges, see documentation for the significations of the colour
	   tracker.setDisplayMovingEdges(displayMovingEdge);
	   // initialise an instance of vpCameraParameters with the parameters from the tracker
	   tracker.setCameraParameters(mcam);
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


	   if (opt_display && opt_click_allowed)
	   {
	     tracker.initClick(Id, initFile.c_str(), true);


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
	   double px = mcam.get_px() ;
	     double py = mcam.get_py() ;
double a,b,c,filter[15];
//vpImageFilter::coefficientGaussianDerivative(filter,15);
cout<<" ok"<<endl;
vpImage<unsigned char> Ip(200,300);
vpImagePoint ip0(100,100);
//I.resize(240,320);
		for (int i=3;i<I.getHeight()-3;i++)
		{
			for (int j=3;j<I.getWidth()-3;j++)
			{
				a=vpImageFilter::derivativeFilterX(Id,i,j);
				b=vpImageFilter::derivativeFilterY(Id,i,j);
				//cout<< " im "<< sqrt(a*a+b*b) <<endl;
				I[i][j]=(unsigned char)sqrt(a*a+b*b);
			}
		}

Id1=Id;

// desired visual feature built from the image
//vpFeatureGradient sId ;
//vpFeatureLuminance sId;
apFeatureGradientOrientation sId;
sId.init(I.getHeight(), I.getWidth(),  1) ;
sId.setCameraParameters(mcam) ;

vpFeatureLuminance sId1;
sId1.init(I.getHeight(), I.getWidth(),  1) ;
sId1.setCameraParameters(mcam);

vpTRACE(" ") ;
/*sId.buildFrom(I);*/

vpMatrix Lsd;   // matrice d'interaction a la position desiree
vpMatrix Hsd;  // hessien a la position desiree
vpMatrix H ; // Hessien utilise pour le levenberg-Marquartd
vpColVector error ; // Erreur I-I*
int n = 6 ;
vpMatrix diagHsd(n,n) ;

vpMatrix Lsd1;   // matrice d'interaction a la position desiree
vpMatrix Hsd1;  // hessien a la position desiree
vpMatrix H1 ; // Hessien utilise pour le levenberg-Marquartd
vpColVector error1 ; // Erreur I-I*
vpMatrix diagHsd1(n,n) ;

//vpFeatureGradient sI;
//vpFeatureLuminance sI;
apFeatureGradientOrientation sI;
sI.init(I.getHeight(),I.getWidth(),1);
sI.setCameraParameters(mcam);
mcam.printParameters();

vpFeatureLuminance sI1;
sI1.init(I.getHeight(),I.getWidth(),1);
sI1.setCameraParameters(mcam);

double lambda ; //gain
vpColVector e ;
vpColVector v ; // camera velocity send to the robot

vpColVector e1 ;
vpColVector v1 ;

// ----------------------------------------------------------
// Minimisation

double mu ;  // mu = 0 : Gauss Newton ; mu != 0  : LM
double lambdaGN;


//mu       =  0.001;
//lambda   = 3 ;
//mu       =  0.2;
mu=0.2;

lambda   =  0.002;
double lambda1   =  1;
lambdaGN = 1;

double temps = 0 ;
int iter ;
int iterGN = 500 ;

vpMatrix Zc;

vpMatrix Vm;
vpTranslationVector tr;
double t0;
double t1;
double t2;
vpImage<unsigned char> If;
vpImage<unsigned char> If1;
vpImage<unsigned char> Ii;
vpImage<vpRGBa> Irgb;
vpImage<unsigned char> I2f;
vpImage<unsigned char> I2f1;
vpImage<unsigned char> Idifff;
vpImage<unsigned char> Id0;
Ii.resize(480,640);
Irgb.resize(480,640);
vpMatrix Zcoord(480,640);
vpMatrix Ori(480,640);
vpMatrix Ori1(480,640);
int im=0;
//for (int m=0;m<10;m++) reader.acquire(Id);

//grabber.setDeviceType(1);
/*grabber.open(Idisplay);
grabber.acquire(Idisplay);*/
// Compute the initial pose of the camera
//computeInitialPose(&mcam, Idisplay, &mPose, md, mcog, &cmo, mP);
// Close the framegrabber
//grabber.close();
//grabber.open(Id);

vpPlot A(1);
A.initGraph(0,1);
A.setColor(0,0,vpColor::red);
	try
	{
		// Rendering loop
		while(ogre.continueRendering()){


			//grabber.acquire(Id);
            //reader.acquire(Id);
            vpDisplay::display(Id);
			t0= vpTime::measureTimeMs();
            vpDisplay::flush(Id);
            I=Ii;
            Idiff=Ii;
            I4=Irgb;
            vpImageIo::readPGM(Id,"imageV.pgm");
            cMo.extract(tr);
            ogre.updateClipDistances(tr[2]);
            ogre.display(IC,&cMo);

            //zbuf.cast(Id0,Id,I3);
            /*I.insert(Id0,ip0);
            vpDisplay::display(I);
            vpDisplay::flush(I);*/
            If.resize(I3.getHeight(),I3.getWidth());
            I2f.resize(I3.getHeight(),I3.getWidth());
            If1.resize(I3.getHeight(),I3.getWidth());
            I2f1.resize(I3.getHeight(),I3.getWidth());
            Idifff.resize(I3.getHeight(),I3.getWidth());
            I5.resize(I3.getHeight(),I3.getWidth());
            Zcoord.resize(I3.getHeight(),I3.getWidth());
            Ori.resize(I3.getHeight(),I3.getWidth());
            Ori1.resize(I3.getHeight(),I3.getWidth());
            for (int i=3;i<I3.getHeight()-3;i++)
            		{
            			for (int j=3;j<I3.getWidth()-3;j++)
            			{
            				a=2*(vpImageFilter::derivativeFilterX(Id,i,j));
            				b=2*(vpImageFilter::derivativeFilterY(Id,i,j));
            				if (sqrt((vpMath::sqr(a) + vpMath::sqr(b)))>0)
            				{Ori[i][j]=atan(b/a)+M_PI/2;}
            				//else {If[i][j]=0;}
            				else {Ori[i][j]=M_PI/2;}
            				If1[i][j]=(unsigned char)sqrt((vpMath::sqr(a)+vpMath::sqr(b)));
            			}
            		}
            ip0.set_i(zbuf.borni1-30);
            ip0.set_j(zbuf.bornj1-30);
            //I.insert(If,ip0);
            //I.insert(If1,ip0);
            I=If1;
            vpDisplay::display(I);

            sId.init(Id.getHeight(), Id.getWidth(),tr[2]);

            //sId.init(Id0.getHeight(), Id0.getWidth(),tr[2]);

            sId1.init(Id.getHeight(), Id.getWidth(),tr[2]);
            //sId.buildFrom(Id0);
            //sId.buildOr1From(Id0);
            sId.buildFrom(Ori,I);
            sId1.buildFrom(If1);

            vpDisplay::flush(I);
            /*sId.buildFrom(If);
            sId1.buildFrom(If1);*/

            sI.init(Id.getHeight(), Id.getWidth(),tr[2]);

            //sI.init(Id0.getHeight(), Id0.getWidth(),tr[2]);

            sI1.init(Id.getHeight(), Id.getWidth(),tr[2]);
            //sI.computeGradients(Id0);

            sId1.interaction(Lsd1);

            // Compute the Hessian H = L^TL
            //Hsd = Lsd.AtA();
            Hsd1 = Lsd1.AtA();
            // Compute the Hessian diagonal for the Levenberg-Marquartd
            // optimization process
            /*diagHsd.eye(n);
            for(int i = 0 ; i < n ; i++) diagHsd[i][i] = Hsd[i][i];*/
            diagHsd1.eye(n);
            for(int i = 0 ; i < n ; i++) diagHsd1[i][i] = Hsd1[i][i];
            //getchar();
            iter=0;
            while(iter < 100)
            {
	            cMo.extract(tr);
	            ogre.updateClipDistances(tr[2]);
	            ogre.display(IC,&cMo);
	            //zbuf.getLuminance(I1,I3);
	            //zbuf.getCastLuminance(I3);
	            zbuf.getCastRGBLuminance(I3,I5,Zcoord,tr[2]);
                //vpDisplay::display(I3);
                //vpDisplay::flush(I3);
	            for (int i=3;i<I3.getHeight()-3;i++)
	            		{
	            			for (int j=3;j<I3.getWidth()-3;j++)
	            			{
	            				a=2*(vpImageFilter::derivativeFilterX(I3,i,j));
	            				b=2*(vpImageFilter::derivativeFilterY(I3,i,j));
	            				if (sqrt((vpMath::sqr(a) + vpMath::sqr(b)))>0)
	            				{Ori1[i][j]=(atan(b/a)+M_PI/2);
	            				}
	            				 //else {I2f[i][j]=0;}
	            				else {Ori1[i][j]=M_PI/2;}
	            				Idiff[i][j]=(unsigned char)sqrt((vpMath::sqr(a) + vpMath::sqr(b)));
	            			}
	            		}
                vpDisplay::display(Idiff);
	            vpImageTools::imageDifference(Idiff,If1,Idifff);
	            I3=Idifff;

	                         //Idiff.insert(Idifff,ip0);
	                         //Idiff=I;
	                         //vpDisplay::flush(Idiff);
                /*vpDisplay::display(I3);
                vpDisplay::flush(I3);*/


	            //sId.buildFrom(Id0);
	            //sI.update(I3.getHeight(), I3.getWidth(),tr[2]) ;
	            //sI.init(Id0.getHeight(), Id0.getWidth(),tr[2]);
                //sI.buildOr1From(I3);
	            //sI.buildFrom(I3);
	            //sI.buildThFrom(Id0,I3,10);
	            sI.buildFrom(Ori1,I5,Zcoord,cMo, Idiff);
	            //sI.buildFrom(I2f);
	            sI1.buildFrom(Idiff);


	            //sI.interactionTh(Lsd) ;
	            sI.interaction(Lsd,Idiff) ;
	            //Idiff.insert(Idifff,ip0);
	                         vpDisplay::flush(Idiff);
	                         vpDisplay::display(I3);
	                          vpDisplay::flush(I3);
	                           char buf[FILENAME_MAX];
	                         	sprintf(buf, opath.c_str(), im);
	                         	std::string filename(buf);
	                         	std::cout << "Write: " << filename << std::endl;
	                         	vpImageIo::write(I3, filename);
	                         	im++;

	                        // Compute the Hessian H = L^TL
	                        Hsd = Lsd.AtA();
	                        // Compute the Hessian diagonal for the Levenberg-Marquartd
	                        // optimization process
	                        diagHsd.eye(n);
	                        for(int i = 0 ; i < n ; i++) diagHsd[i][i] = Hsd[i][i];

                //sI.buildFrom(I2f);
                //sI.buildFrom(I2);
                /*vpImageTools::imageDifference(If,I2f,Idifff);
                Idiff.insert(Idifff,ip0);
                //Idiff=Idifff;
                vpDisplay::display(Idiff);
                vpDisplay::flush(Idiff);*/
                //sI.errorTh(error);
                sI.error(sId, error);
                sI1.error(sId1, error1);
            	H = ((mu * diagHsd) + Hsd).pseudoInverse();
            	H1 = ((mu * diagHsd1) + Hsd1).pseudoInverse();
            	//	compute the control law
            	e = H * Lsd.t() *error;
            	e1 = H1 * Lsd1.t() *error1;
                double normeError = (error.sumSquare());
            	//if (normeError < 0.5e7) lambda = 1 ;
            	v =  -lambda*e;
            	v1 =  -lambda1*e1;

            	cMo =  vpExponentialMap::direct(v).inverse()*cMo;
            	iter++;
            	//cout<<lambda<<endl;
                cout<<cMo<<endl;
            }

            getchar();
            I4.insert(I5,ip0);
            vpDisplay::display(I4);
            vpDisplay::flush(I4);
            t1= vpTime::measureTimeMs();
             cout << "time  "<< t1-t0<<endl;
             /*vpImageTools::imageDifference(If1,I2f1,Idifff);
             Idiff.insert(Idifff,ip0);
             //Idiff=Idifff;
                             vpDisplay::display(Idiff);
                             vpDisplay::flush(Idiff);*/
                             //getchar();
            /*for (int i=3;i<I3.getHeight()-3;i++)
            	            		{
            	            			for (int j=3;j<I3.getWidth()-3;j++)
            	            			{
            	            				a=vpImageFilter::derivativeFilterX(I3,i,j);
            	            				b=vpImageFilter::derivativeFilterY(I3,i,j);
            	            				I2f[i][j]=(unsigned char)sqrt((vpMath::sqr(a) + vpMath::sqr(b)));
            	            				//I2[i][j]=(unsigned char)(vpMath::sqr(a) + vpMath::sqr(b));
            	            			}
            	            		}

            		//vpDisplay::display(I3);
                           // vpDisplay::flush(I3);
            /*vpImageTools::imageDifference(I,I2,Idiff);
                            vpDisplay::display(Idiff);
                            vpDisplay::flush(Idiff);*/
            /*vpImageTools::imageDifference(If,I2f,Idifff);
            Idiff.insert(Idifff,ip0);
            vpDisplay::display(Idiff);
            vpDisplay::flush(Idiff);*/
            //getchar();
            //vpImageIo::writePGM(I3,"imageV.pgm");
            //vpTime::wait(1000);
            //getchar();
            //vpDisplay::display(I3);
            //vpDisplay::flush(I3);


			            /*for (int i=7;i<I3.getHeight()-7;i++)
			            		{
			            			for (int j=7;j<I3.getWidth()-7;j++)
			            			{
			            				a=vpImageFilter::derivativeFilterX(I3,i,j);
			            				b=vpImageFilter::derivativeFilterY(I3,i,j);
			            				I2[i][j]=(unsigned char)(a*a+b*b);
			            			}
			            		}
                        vpDisplay::display(I2);
                        vpDisplay::flush(I2);*/
			            //getchar();

           		        //tracker.display(Id, cMo, mcam, vpColor::darkRed, 2);
           		        //vpDisplay::displayFrame (Id, cMo, mcam, 0.05, vpColor::blue);
           		        //vpDisplay::flush(Id);
 
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

