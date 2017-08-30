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
#include <visp/vpImageConvert.h>
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
#include "vpMbPointsTracker.h"
#include "apDomOrientation.h"
#include "apViewGeneration.h"


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
	  std::string env_ipath1;
	  std::string opath;
	  std::string opath1;
	  std::string opath2;
	  std::string opath3;
	  std::string opath4;
	  std::string opath5;
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
      //configFile = "cube.xml";
	  configFile = "amazonas-2-14-gripper.xml";

	  //initFile = "cube";
      //initFile = "amazonas-2-14";
      //initFile = "amazonas-2-15";
	  //initFile = "atv-3";
	  //initFile = "a330_5";

	  //initFile = "progress";

	  initFile = "a380_2";

	  //initFile = "chateau";
	  //initFile= "discovery-2";
	  //initFile= "starwars";
	  env_ipath="/local/agpetit/images";
      env_ipath="/local/agpetit/Desktop";

      env_ipath1="/local/agpetit/images";


	    // Set the default input path

      ipath = env_ipath1 + vpIoTools::path("/imageTrakMark/motion.%04d.jpg");
      ipath = env_ipath1 + vpIoTools::path("/imagesA380/I%04d.pgm");
	  //opath = env_ipath1 + vpIoTools::path("/imagesAmazonas/speed_out/I%04d.pgm");
      opath = env_ipath1 + vpIoTools::path("/imageTrakMark/out/I%04d.pgm");
      opath = env_ipath1 + vpIoTools::path("/imagesA380/out/I%04d.pgm");
	  opath1 = env_ipath1 + vpIoTools::path("/imagesSoyuz/soyuz1/Izbuf%04d.pgm");
	  opath2 = env_ipath1 + vpIoTools::path("/imagesSoyuz/soyuz1/Iem%04d.pgm");
	  opath3 = env_ipath1 + vpIoTools::path("/imagesSoyuz/soyuz1/Icps%04d.pgm");
	  opath4 = env_ipath1 + vpIoTools::path("/imagesSoyuz/soyuz1/Iup%04d.pgm");
	  opath5 = env_ipath1 + vpIoTools::path("/imagesSoyuz/Hierarchy/I%04d.pgm");
	  //ipath = env_ipath + vpIoTools::path("/ViSP-images/calib/I%04d.pgm");



      // OpenCVGrabber to gather images
      vpOpenCVGrabber grabber;
      vpVideoReader reader;

	  //vpCameraParameters mcam(547.7367575,542.0744058,338.7036994,234.5083345);
      //vpCameraParameters mcam(547.7367575,542.0744058,320,240);
      //vpCameraParameters mcam(847.7367575,842.0744058,320,240);
      //vpCameraParameters mcam(857.48,859.03,322,256);
      //vpCameraParameters mcam(1057.48,1059.03,320,240);


      //vpCameraParameters mcam(761.6069225,757.4794236,320,240);

      //vpCameraParameters mcam(415.692,415.692,320,240);

      //vpCameraParameters mcam(761.6069225,757.4794236,320,240);


      vpCameraParameters mcam(1700.6069225,1700.4794236,320,240);

   	  // Create a vpRAOgre object with color background
   	  exampleVpAROgre ogre(&grabber,&mcam,BACKGROUND_COLOR);

	  ogre.init();
	  //ogre.load("Cube","cube.mesh");
	  //ogre.load("A330","A330.mesh");
	  //ogre.load("progress","progress.mesh");
	  //ogre.load("chateau","chateauBordSimple.mesh");
	  //ogre.load("soyouz0","soyuz0.mesh");


	  //ogre.load("soyouz","soyuz.mesh");


	  //ogre.load("trakmark","trakmark2.mesh");


	  ogre.load("A380","a380_3.mesh");

	  //ogre.load("discovery","ss.001.mesh");
	  //ogre.load("discovery","Untitled.mesh");
	  //ogre.load("starwars","Eta-2.mesh");
	  //ogre.load("Satel", "ShapeIndexedFaceS.002.mesh");
	  //ogre.load("Satel", "Mesh.004.mesh");
	  //ogre.load("Satel", "Mesh.148.mesh");
	  //ogre.load("Satel", "satelsd3.mesh");
	  //ogre.loadE("Satel2", "satelsd.mesh");
	  //ogre.load("Satel", "Mesh.025.mesh");
	  //ogre.setRotation("Satel", vpRotationMatrix(vpRxyzVector(M_PI/2,M_PI,0)));
	  //ogre.setRotation("Satel", vpRotationMatrix(vpRxyzVector(-M_PI/2,0,0)));
	  //ogre.load("Satel", "Mesh.002.mesh");
	  //ogre.load("ATV", "Jules_Verne_ATV_1.mesh");
	  //ogre.setRotation("ATV", vpRotationMatrix(vpRxyzVector(3*M_PI/2,0,0)));
	  //ogre.setRotation("soyouz", vpRotationMatrix(vpRxyzVector(0,M_PI,0)));
	  //ogre.setPosition("chateau", vpTranslationVector(-0.05, -0.1, 0));
	  //ogre.setScale("Cube", 0.00036,0.00036,0.00036);
	    //ogre.setScale("Cube", 0.004,0.004,0.004);
	  //ogre.setScale("Satel", 2,2,2);
	  //ogre.setScale("ATV", 0.01,0.01,0.01);
	  //ogre.setScale("soyouz0", 0.10,0.10,0.10);

	  //ogre.setScale("soyouz", 0.10,0.10,0.10);

	  //ogre.setScale("trakmark",0.0001,0.0001,0.0001);

	  ogre.setScale("A380",0.1,0.1,0.1);


	  //ogre.setScale("progress", 0.10,0.10,0.10);
	  //ogre.setScale("chateau", 0.10,-0.10,0.1);
	  //ogre.setRotation("A330", vpRotationMatrix(vpRxyzVector(0,M_PI/1.8,0)));
	  //ogre.setPosition("progress", vpTranslationVector(0.0, 0.5, 0.0));
	  //ogre.setRotation("discovery", vpRotationMatrix(vpRxyzVector(0,-M_PI/15,0)));
	  //ogre.setScale("discovery", 10,10,10);


	  vpImage<unsigned char> Id;
	  vpImage<unsigned char> Id1;
	  vpImage<vpRGBa> Id2;
	  vpImage<vpRGBa> Ioverlay;
	  reader.setFileName(ipath.c_str());
	  reader.open(Id);
	  //for (int m=0;m<600;m++){
	  for (int m=0;m<400;m++){
	  reader.acquire(Id);
	  }
	  //vpImageConvert::convert(Id2,Id);
	  /*grabber.setDeviceType(1);
	  grabber.open(Id);
      grabber.acquire(Id);*/
	  //vpImageIo::readPGM(Id,"imageV.pgm");

	// Grey level image to track points
	vpImage<unsigned char> I;
	// RGBa image to get background
	vpImage<vpRGBa> IC;
	vpImage<vpRGBa> Inorm(480,640);
	vpImage<double> I1;
	vpImage<unsigned char> I2;
	vpImage<unsigned char> I3(480,640);
	vpImage<unsigned char> I4(480,640);
	vpImage<unsigned char> Idiff ;


	      vpDisplayX display1;
	      vpDisplayX display;
	      vpDisplayX display2;
	      vpDisplayX display3;
	      I.resize(480,640);
	      I1.resize(480,640);
	      I2.resize(480,640);
	      I3.resize(220,480);
	      Idiff.resize(480,640);
	    display1.init(I, 10, 10, "X11 display");
	    display2.init(I2, 1500, 10, "X11 display");
	    display3.init(I3, 1500, 1000, "X11 display");
	    //display3.init(I2, 1500, 1000, "X11 display");
	    if (opt_display)
	    {
	      display.init(Id, 10, 10, "Test tracking") ;
	      vpDisplay::display(Id) ;
	      vpDisplay::flush(Id);
	    }

   	  apZBuffer zbuf;
   	  zbuf.set(480,640,0.1,1);

	// CameraParameters we got from calibration
	// Keep u0 and v0 as center of the screen

   	vpHomogeneousMatrix cMo;//(0.008924150268,-0.03937067413,1.859247036,0.2544105297,-0.213655404-M_PI,2.278629041-M_PI);
	 vpMbPointsTracker tracker;
	 tracker.loadConfigFile(configFile);
	 //tracker.loadModel("a330.wrl");
	   // Display the moving edges, see documentation for the significations of the colour
	   //tracker.setDisplayMovingEdges(displayMovingEdge);
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


	   tracker.getPose(cMo);

	   /*cMo.buildFrom(11.7759940348,5.8999979250,5.5547190835,-2.6525344080,-0.0000000000,1.6833495227 );
	   cMo.buildFrom( -0.768532741,  6.24302505,  13.54560648,  -2.683611579,  0.003069081378,  1.629208268 );
	   cMo=cMo.inverse();*/

	   tracker.init(I,cMo);


	   if (opt_display)
		  vpDisplay::flush(Id);
	   double px = mcam.get_px() ;
	   double py = mcam.get_py() ;
double a,b,c,filter[15];
//vpImageFilter::coefficientGaussianDerivative(filter,15);

Id1=Id;


vpTRACE(" ") ;

// ----------------------------------------------------------
// Minimisation

double mu ;  // mu = 0 : Gauss Newton ; mu != 0  : LM
double lambdaGN;

double temps = 0 ;
int iter ;
int iterGN = 500 ;

vpMatrix Zc;

vpMatrix Vm;
vpTranslationVector tr;
double t0;
double t1;
double t2;
//Ii.resize(480,640);
int im=0;
vpMatrix Ori(480,640);
vpMatrix Zcoord(480,640);
//for (int m=0;m<10;m++) reader.acquire(Id);

//grabber.setDeviceType(1);
/*grabber.open(Idisplay);
grabber.acquire(Idisplay);*/
// Compute the initial pose of the camera
//computeInitialPose(&mcam, Idisplay, &mPose, md, mcog, &cmo, mP);
// Close the framegrabber
//grabber.close();
//grabber.open(Id);

apDomOrientation DomI;
apDomOrientation DomItemp;
DomI.init(10,2,10,30);
//DomItemp.init(10,3,10,30);
std::vector<vpColVector> DomOrientations;
//std::vector<vpColVector> DomOrientationsTemplate;
vpImage<unsigned char> Itemp(220,480);
vpImage<unsigned char> Ior(480,640);
vpImage<unsigned char> Itex(480,640);
vpImage<unsigned char> Ior0(200,200);
vpImageIo::readPGM(Ior0, "input.pgm");
vpImage<vpRGBa> Inormd(480,640);
vpImage<vpRGBa> Inormd1(480,640);
vpImage<unsigned char> Ied(480,640);
//vpImageIo::readPPM(Ied, "texture10.pgm");


//ogre.setLights();

//ogre.setMat("soyouz","NormalMap");


//ogre.setRotation("trakmark", vpRotationMatrix(vpRxyzVector(M_PI/2,0,0)));


//ogre.setMat("trakmark","_auto_29");
//ogre.setMat("trakmark","_auto_46");
/*ogre.setMat("trakmark","_auto_45");
ogre.setMat("trakmark","_auto_43");
ogre.setMat("trakmark","_auto_49");
ogre.setMat("trakmark","_auto_48");
ogre.setMat("trakmark","_auto_44");
ogre.setMat("trakmark","_auto_52");
ogre.setMat("trakmark","_auto_53");*/
//ogre.setMat("trakmark","_auto_50");
/*ogre.setMat("trakmark","_auto_51");
ogre.setMat("trakmark","_auto_47");
ogre.setMat("trakmark","_auto_54");
ogre.setMat("trakmark","_auto_55");*/
//ogre.setMat("trakmark","Scene.material");



ogre.setMat("A380","NormalMap");


//ogre.setShaders("trakmark");

//ogre.setShadersT("A380");

//ogre.setShadersCol("A380");

//ogre.setShadersT("trakmark");

ogre.setRTT("DepthGradNormMap0");

//ogre.setMRTTCol("CannyOrient","DepthGradNormMap1");

//ogre.setRTT("TexDepthEdges");

//ogre.setMultiRTT("CannyOrient","DepthGradNormMap");


//ogre.setScale("trakmark",0.0001,0.0001,0.0001);

//ogre.setRotation("trakmark", vpRotationMatrix(vpRxyzVector(M_PI/2,0,0)));

//ogre.setMat("trakmark","NormalMap");

vpColVector Orimax(2);
vpPoseVector posin;
posin.buildFrom(cMo);
apViewGeneration Views;
Views.init(4,4,4,1);
vpImage<vpRGBa> imArg(480,640);

//Views.edgeOrientMap(Ied);
//Views.createViews(ogre,posin,opath5);
//Views.createViewsDT(ogre,posin,opath5);
//Views.buildHierarchy(13.8, "hierarchy.txt");
//Views.buildHierarchy(13.8, "hierarchy.txt");
//Views.buildHierarchy(12.8, "hierarchy.txt");
//Views.dTTemplates0();
t0= vpTime::measureTimeMs();
//Views.detect(Id,opath5);
t1= vpTime::measureTimeMs();
cout << "time2 "<< t1-t0 << endl;
//getchar();
//Views.edgeOrientMap(Id);
//Views.computeSimilarity0(Id,opath5);

//Views.merge();
//Views.dT(Ior,imArg);
//std::cout<<" ok0 "<<std::endl;

//cout << "time2 "<< t1-t0 <<" "<<(double)(Ior[100][100])<<" " <<((double)(imArg[100][100]).R-127)*2 <<" " <<((double)(imArg[100][100]).G-127)*2 <<endl;
//getchar();
cMo.extract(tr);
ogre.updateClipDistances0(tr[2],6);
ogre.updateRTT(Inormd,Ior,&cMo);
tracker.track(Id,Inormd,Ior,Ior,tr[2]);



	try
	{
		// Rendering loop
		while(ogre.continueRendering()){

			//grabber.acquire(Id);
            reader.acquire(Id);
            vpDisplay::display(Id);
            /*vpImageIo::readPGM(Itemp,"template2.pgm");
            DomI.buildFrom(Itemp,I3,1);
            vpDisplay::display(I3);
            vpDisplay::flush(I3);*/
            //DomI.buildFrom(Itemp,I3,1);
            //Orimax = DomI.computeCostFunction();
            cout << Orimax<< endl;
            //getchar();

            //DomOrientations=DomI.getDominantOrientations();
            //std::cout<<
            //vpDisplay::display(I3);
            //vpImageIo::readPGM(Id,"imageV.pgm");
            cMo.extract(tr);


           //tracker.trackPred(Id);


           /* if (im==5){
            	ogre.deleteE("Satel");
      	  ogre.loadE("Satel2", "satelsd3.mesh");
      	  ogre.setRotation("Satel2", vpRotationMatrix(vpRxyzVector(-M_PI/2,0,0)));
      	  }*/
            /*if (tr[2]<0.40 && tr[2]>0.39){tracker.loadConfigFile("amazonas-2-15-gripper.xml");}
            if (tr[2]<0.40){
            ogre.updateClipDistances0(tr[2],0.1);
            }*/
            //else

            ///{ogre.updateClipDistances0(tr[2],2);}

            //{ogre.updateClipDistances0(70,69.8);}

            //{ogre.updateClipDistances0(100,95);}

            {ogre.updateClipDistances0(tr[2],6);}

            t0= vpTime::measureTimeMs();


            //ogre.updateRTTex(Itex,Inormd,Ior,&cMo);

            ogre.updateRTT(Inormd,Ior,&cMo);

            //ogre.updateMultiRTT(Inormd,Ior,&cMo);
            //vpImageConvert::convert(Inormd1,Itex);
            //vpImageIo::writePPM(Ior, "text1.pgm");

            //Views.edgeOrientMap(Itex);

            //ogre.updateRTT(Inormd,Ior,&cMo);
            t1= vpTime::measureTimeMs();

            cout << "timeRend "<< t1-t0 << endl;

            //ogre.updateMultiRTT(Inormd,Ior,&cMo);

            //vpImageIo::writePPM(Itex, "text00.pgm");
            //vpImageIo::writePPM(Inormd, "text4.pgm");
            /*for (int n=0; n < 480 ; n++)
                        {
                        for (int m = 0 ; m < 640; m++)
                          {
                        	if(Inormd[n][m].A<255 && Inormd[n][m].A>0)
                        	{
                        cout << (double)Inormd[n][m].A << endl;
                        	}
                        }
                        }*/
            //ogre.removeMat("NormalMap");
            //ogre.updateRTTex(Inormd,&cMo);
            //vpImageIo::writePPM(Inormd, "text1.pgm");
            //getchar();
            /*if (tr[2]<0.40){
            zbuf.getRGBDepth(I1,I2,I4,Inorm,Zcoord,tr[2],1);}
            else {zbuf.getRGBDepth(I1,I2,I4,Inorm,Zcoord,tr[2],1);}*/
            //vpImageIo::readPPM(Ior, "texture0.pgm");
            //vpImageIo::writePPM(Inormd, "texture100.pgm");
            //getchar();


            /*for (int n=0; n < 480 ; n++)
            {
            for (int m = 0 ; m < 640; m++)
              {
            I2[n][m]=Inormd[n][m].A;
            }
            }
            vpDisplay::display(I2);
            vpDisplay::flush(I2);*/

            /*for (int n=0; n < 480 ; n++)
            {
            for (int m = 0 ; m < 640; m++)
              {
                //I1[n][m] =  (double)I2[n][m]/255;
            	if ((double)Ior[n][m]<100 || (double)Ior[n][m]>100)
            	std::cout << (double)Ior[n][m]  << std::endl;
              }
            }*/


            //apEdgeMap::edgeMapLap(I1,I4,Ori,I3,0.8);

            //apEdgeMap::edgeMapLap2(I4,Ori,I3,50);
            //vpDisplay::display(I3);
            //vpDisplay::flush(I3);

            //tracker.track(Id,I3,Zcoord,Ori,Inorm);
            //tracker.track(Id,Inormd,Ior,tr[2],2);
            //tracker.track(Id,Inormd,Ior,70,69.8);

            //tracker.track(Id,Inormd,Ior,Itex,100,95);


            //tracker.trackMH(Id,Inormd,Ior,Ior,tr[2],6);

            //vpImageConvert::convert(Inormd,Itex);
            //Views.edgeOrientMap(Itex);

            //getchar();

            t1= vpTime::measureTimeMs();
                        cout << "time2 "<<t1-t0<<endl;

            tracker.getPose(cMo);

            tracker.display(Id,cMo,mcam,vpColor::yellow,1);
            std::cout<<cMo<<std::endl;
            //vpDisplay::getClick(Id,true);


            vpDisplay::flush(Id);
            //getchar();
            vpDisplay::getImage(Id,Ioverlay);
        	//A.saveData(0,"A1.txt");
            //A.resetPointList(0,0);

             //vpImageTools::imageDifference(If,I2f,Idifff);
             //Idiff.insert(Idifff,ip0);
             //Idiff=Idifff;
                             //vpDisplay::display(Idiff);
                             //vpDisplay::flush(Idiff);

                             /*char buf[FILENAME_MAX];
                         	sprintf(buf, opath.c_str(), im);
                         	std::string filename(buf);
                         	std::cout << "Write: " << filename << std::endl;
                         	vpImageIo::writePPM(Ioverlay, filename);

                            char buf1[FILENAME_MAX];
                        	sprintf(buf1, opath1.c_str(), im);
                        	std::string filename1(buf1);
                        	std::cout << "Write: " << filename1 << std::endl;
                        	vpImageIo::writePPM(I2, filename1);

                            char buf2[FILENAME_MAX];
                        	sprintf(buf2, opath2.c_str(), im);
                        	std::string filename2(buf2);
                        	std::cout << "Write: " << filename2 << std::endl;
                        	vpImageIo::writePPM(I3, filename2);

                            char buf3[FILENAME_MAX];
                        	sprintf(buf3, opath3.c_str(), im);
                        	std::string filename3(buf3);
                        	std::cout << "Write: " << filename3 << std::endl;
                        	vpImageIo::writePPM(Inorm, filename3);*/

                            char buf4[FILENAME_MAX];
                        	sprintf(buf4, opath.c_str(), im);
                        	std::string filename4(buf4);
                        	//std::cout << "Write: " << filename4 << std::endl;
                        	//vpImageIo::writePPM(Ioverlay, filename4);
                         	im++;



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

