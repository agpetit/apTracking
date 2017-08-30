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
#include "apOgre.h"
#include "apZBuffer.h"
#include "apEdgeMap.h"
#include "apLineExtractor.h"
#include "vpFeatureGradient.h"
#include "apMbHybridTracker.h"
#include "apDomOrientation.h"
#include "apViewGeneration.h"
#include "vpSE3Kalman.h"



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
	  //initFile = "tank2";
	  initFile = "tank4";
	  //initFile = "atlantis";

	  //initFile = "atlantis";

	  //initFile = "atv-3";
	  //initFile = "a330_5";
	  //initFile = "progress";
	  //initFile = "a380_2";
	  //initFile = "chateau";
	  //initFile= "discovery-2";
	  //initFile= "starwars";
	  env_ipath="/local/agpetit/images";
      //env_ipath="/local/agpetit/Desktop";
      env_ipath1="/local/agpetit/images";


	    // Set the default input path
	     // ipath = env_ipath1 + vpIoTools::path("/ViSP-images/mbt/cube/image%04d.pgm");
	  //ipath = env_ipath + vpIoTools::path("/imagesAmazonas/scenario_GRIPPER_speed0009/I%04d.pgm");
      //ipath = env_ipath + vpIoTools::path("/imagesAmazonas/scenario_GRIPPER_speed0009/I%04d.pgm");
      ipath = env_ipath + vpIoTools::path("/imageTests/I%04d.pgm");

      //tank
      ipath = env_ipath + vpIoTools::path("/imagesTank/I%04d.pgm");



      //ipath = env_ipath + vpIoTools::path("/atv/atv3/I%04d.pgm");

      //ipath = env_ipath + vpIoTools::path("/atv/soyuz/I%04d.pgm");

      //ipath = env_ipath + vpIoTools::path("/atlantis/I%04d.pgm");

      //ipath = env_ipath + vpIoTools::path("/starwars/I%04d.pgm");
	  //ipath = env_ipath + vpIoTools::path("/imagesRefuel/Refuel1/I%04d.pgm");

	  //ipath = env_ipath + vpIoTools::path("/ViSP-images/calib/I%04d.pgm");

	  //opath = env_ipath1 + vpIoTools::path("/imagesSoyuz/soyuz1/I%04d.pgm");
      //ipath = env_ipath1 + vpIoTools::path("/imageTrakMark/motion.%04d.jpg");
      //ipath = env_ipath1 + vpIoTools::path("/imagesA380/I%04d.pgm");
      //ipath = env_ipath1 + vpIoTools::path("/imagesSoyuz/I%04d.pgm");
	  //opath = env_ipath1 + vpIoTools::path("/imagesAmazonas/speed_out/I%04d.pgm");
      opath = env_ipath1 + vpIoTools::path("/imageTrakMark/out/I%04d.pgm");
      opath = env_ipath1 + vpIoTools::path("/imageTrakMark/out5/I%04d.pgm");
      opath = env_ipath1 + vpIoTools::path("/imagesSoyuz/soyuz3/I%04d.pgm");
      opath = env_ipath1 + vpIoTools::path("/imagesAmazonas/outHyb2/I%04d.pgm");
      opath = env_ipath1 + vpIoTools::path("/imageTests/out2/I%04d.pgm");

      opath = env_ipath1 + vpIoTools::path("/imagesAtlantis/out2/I%04d.pgm");
      opath = env_ipath1 + vpIoTools::path("/imagesTank/out1/I%04d.pgm");
      //opath = env_ipath1 + vpIoTools::path("/imagesA380/out3/I%04d.pgm");
	  opath1 = env_ipath1 + vpIoTools::path("/imagesSoyuz/soyuz1/Izbuf%04d.pgm");
	  opath2 = env_ipath1 + vpIoTools::path("/imagesSoyuz/soyuz1/Iem%04d.pgm");
	  opath3 = env_ipath1 + vpIoTools::path("/imagesSoyuz/soyuz1/Icps%04d.pgm");
	  opath4 = env_ipath1 + vpIoTools::path("/imagesSoyuz/soyuz1/Iup%04d.pgm");
	  opath5 = env_ipath1 + vpIoTools::path("/imagesSoyuz/Hierarchy/I%04d.pgm");
	  std::string posepath ="./Poses/PoseTrakmark.txt";
	  std::string posepath3 ="./Poses/PoseAmazonasHyb2.txt";
	  std::string posepath1 ="./Poses/TransTrakmarkEdgeTex.txt";
	  std::string posepath2 ="./Poses/errorTrakmarkEdgeTex.txt";
	  //ipath = env_ipath + vpIoTools::path("/ViSP-images/calib/I%04d.pgm");



      // OpenCVGrabber to gather images
      vpOpenCVGrabber grabber;
      vpVideoReader reader;

	  //vpCameraParameters mcam(547.7367575,542.0744058,338.7036994,234.5083345);
      //vpCameraParameters mcam(547.7367575,542.0744058,320,240);
      //vpCameraParameters mcam(847.7367575,842.0744058,320,240);

      //vpCameraParameters mcam(857.48,859.03,320,240);
      //vpCameraParameters mcam(1057.48,1059.03,320,240);

      vpCameraParameters mcam(2057.48,2059.03,320,240);

      //vpCameraParameters mcam(761.6069225,757.4794236,320,240);

      //vpCameraParameters mcam(415.692,415.692,320,240);
      /*vpMatrix Cam = mcam.get_K();
      Cam=Cam.pseudoInverse();*/


      //vpCameraParameters mcam(761.6069225,757.4794236,320,240);


      //vpCameraParameters mcam(1900.6069225,1900.4794236,320,240);

   	  // Create a vpRAOgre object with color background
   	  //exampleVpAROgre ogre(&grabber,&mcam,BACKGROUND_COLOR);
      vpAROgre ogre(&grabber,&mcam,BACKGROUND_COLOR);

	  ogre.init();
	  //ogre.load("Cube","cube.mesh");
	  //ogre.load("A330","A330.mesh");
	  //ogre.load("progress","progress.mesh");
	  //ogre.load("chateau","chateauBordSimple.mesh");
	  //ogre.load("soyouz0","soyuz0.mesh");


	  //ogre.load("soyouz","soyuz.mesh");


	  //ogre.load("trakmark","trakmark2.mesh");


	  //ogre.load("A380","a380_3.mesh");

	  //ogre.load("discovery","ss.001.mesh");
	  //ogre.load("discovery","Untitled.mesh");
	  //ogre.load("starwars","Eta-2.mesh");
	  //ogre.load("Satel", "ShapeIndexedFaceS.002.mesh");
	  //ogre.load("Satel", "Mesh.004.mesh");
	  //ogre.load("Satel", "Mesh.148.mesh");
	  //ogre.load("Satel", "satel5.mesh");
	  //ogre.load("Satel", "satelsd3.mesh");
	  //ogre.load("Satel", "satel4.mesh");
	  //ogre.loadE("Satel2", "satelsd.mesh");
	  //ogre.load("Satel", "Mesh.025.mesh");

	  //tank

	  ogre.load("tank", "tank4.mesh");

	  //ogre.load("atlantis", "atlantis.mesh");

	  //ogre.setRotation("Satel", vpRotationMatrix(vpRxyzVector(M_PI/2,M_PI,0)));
	  //ogre.setRotation("Satel", vpRotationMatrix(vpRxyzVector(-M_PI/2,0,0)));
	  //ogre.load("Satel", "Mesh.002.mesh");
	  //ogre.load("ATV", "Jules_Verne_ATV_1.mesh");

	  //ogre.setRotation("tank", vpRotationMatrix(vpRxyzVector(0,-M_PI/2,M_PI/2)));

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

	  //ogre.setScale("A380",0.1,0.1,0.1);

	  ogre.setScale("tank",0.1,0.1,0.1);

	  //ogre.setScale("atlantis",0.01,0.01,0.01);

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
	  //for (int m=0;m<1500;m++){
	  for (int m=0;m<3000;m++){
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

	// CameraParameters we got from calibration
	// Keep u0 and v0 as center of the screen

   	vpHomogeneousMatrix cMo;//(0.008924150268,-0.03937067413,1.859247036,0.2544105297,-0.213655404-M_PI,2.278629041-M_PI);
   	vpMbPointsTracker tracker;

   	//apMbHybridTracker tracker(640,480);
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

//grabber.setDeviceType(1);
/*grabber.open(Idisplay);
grabber.acquire(Idisplay);*/
// Compute the initial pose of the camera
//computeInitialPose(&mcam, Idisplay, &mPose, md, mcog, &cmo, mP);
// Close the framegrabber
//grabber.close();
//grabber.open(Id);


vpImage<unsigned char> Itemp(220,480);
vpImage<unsigned char> Ior(480,640);
vpImage<unsigned char> Itex(480,640);
vpImage<vpRGBa> Inormd(480,640);
vpImage<vpRGBa> Inormd1(480,640);
vpImage<unsigned char> Ied(480,640);
//vpImageIo::readPPM(Ied, "texture10.pgm");
vpMatrix Pose(2000,7);
vpThetaUVector thetau;
vpTranslationVector tr;
cMo.extract(tr);
//satel
ogre.updateClipDistances0(tr[2],2.5);

//tan
ogre.updateClipDistances0(tr[2],2.5);

//ogre.setMat("soyouz","NormalMap");


//ogre.setRotation("trakmark", vpRotationMatrix(vpRxyzVector(M_PI/2,0,0)));


//ogre.setMat("A380","NormalMap");
//ogre.setMat("trakmark","NormalMap");


//ogre.setMat("Cube","NormalMap");

//ogre.setMat("Satel","NormalMap");
//ogre.setLights();

//ogre.setMat("atlantis","NormalMap");

ogre.setMat("tank","NormalMap");

//ogre.setShadersCol("soyouz");

//ogre.setShadersCol("tank");



//ogre.setMat("Cube","NormalMap");


//ogre.setShaders("trakmark");

//ogre.setShadersT("A380");

//ogre.setShadersCol("A380");

ogre.setRTT("DepthGradNormMap0");

//ogre.setRTTHyb("Grad","DepthGradNormMap0");

//ogre.setMRTTCol("Grad","DepthGradNormMap1");

/*ogre.updateRTTV(Ior,&cMo);
vpImageIo::writePPM(Ior, "text1.pgm");

getchar();*/

//ogre.setMRTTCol("CannyOrient","DepthGradNormMap1");

//ogre.setRTT("TexDepthEdges");

//ogre.setMultiRTT("CannyOrient","DepthGradNormMap");


//ogre.setScale("trakmark",0.0001,0.0001,0.0001);

//ogre.setRotation("trakmark", vpRotationMatrix(vpRxyzVector(M_PI/2,0,0)));

//ogre.setMat("trakmark","NormalMap");


//tracker.loadOgre(ogre);

vpPoseVector posin;
posin.buildFrom(cMo);
apViewGeneration Views;
Views.init(4,4,4,1);
vpImage<vpRGBa> imArg(480,640);
double t0,t1;

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


vpMatrix HMVraie(2000,12);
HMVraie.loadMatrix("./Poses/GroundTruth4.txt",HMVraie,false);
vpRotationMatrix R1(vpRxyzVector(-M_PI/2,0,0));
//cout << HMVraie << endl;
vpRotationMatrix RV;
vpTranslationVector trV;
vpMatrix RVc;
vpColVector trVc;
vpMatrix transl(2000,7);
vpMatrix error(2000,4);
vpMatrix PoseVraie(2000,6);
vpMatrix PoseHyb(2000,7);
vpMatrix PoseGeom(2000,7);
/*PoseVraie.loadMatrix("./Poses/PoseVraie_speed0009.txt",PoseVraie,false);
PoseHyb.loadMatrix("./Poses/PoseAmazonasHyb1.txt",PoseHyb,false);
PoseGeom.loadMatrix("./Poses/PoseAmazonasGeom1.txt",PoseGeom,false);
PoseVraie.resize(700,3,false);
PoseGeom.resize(700,3,false);
PoseHyb.resize(700,3,false);
for (int i =0;i<700;i++)
{
	error[i][0]=PoseHyb[i][0]-PoseVraie[i][0];
	error[i][1]=PoseHyb[i][1]-PoseVraie[i][1];
	error[i][2]=PoseHyb[i][2]-PoseVraie[i][2];
	error[i][3]=sqrt(error[i][0]*error[i][0]+error[i][2]*error[i][2]);
}
error.resize(700,4,false);
error.saveMatrix("./Poses/errorHyb1.txt",error,false);


getchar();*/
vpHomogeneousMatrix cMoV;
/*for(int i = 0; i < rows; i++)
{
for(int j = 0; j < cols; j++)
{
HMVraie[i][j] = value;
}
}*/

fstream file ;
fstream finit ;

double rap =0;
double rap0=rap;
double rap1=rap;
rap=253.888;
vpHomogeneousMatrix cMo0;
vpHomogeneousMatrix oMc;
vpRotationMatrix R0;
int im=0;

vpPoseVector _Pose;
vpSE3Kalman filterK;
vpColVector Xv(6);
vpMatrix Qv(6,6);
vpColVector X(12);
Qv.setIdentity();
oMc = cMo.inverse();
posin.buildFrom(oMc);
Xv[0]=posin[0];
Xv[1]=posin[1];
Xv[2]=posin[2];
Xv[3]=posin[3];
Xv[4]=posin[4];
Xv[5]=posin[5];
filterK.setXv(Xv);
filterK.setQv(Qv);
filterK.init();

Qv*=0.01;

vpImage<double> Igrad(480,640);
	try
	{
		// Rendering loop
		while(ogre.continueRendering()){
		//while(im<1259){
		//while(im<360){

			//grabber.acquire(Id);
			try{
				for (int ima = 0;ima<1;ima++)
            reader.acquire(Id);
			}
			catch(...){
				break;
			}
            vpDisplay::display(Id);
            /*vpImageIo::readPGM(Itemp,"template2.pgm");
            DomI.buildFrom(Itemp,I3,1);
            vpDisplay::display(I3);
            vpDisplay::flush(I3);*/
            //DomI.buildFrom(Itemp,I3,1);
            //Orimax = DomI.computeCostFunction();
            //getchar();

            //DomOrientations=DomI.getDominantOrientations();
            //std::cout<<
            //vpDisplay::display(I3);
            //vpImageIo::readPGM(Id,"imageV.pgm");
            cMo.extract(tr);
            cMo.extract(R0);
            vpRxyzVector Rxyz(R0);

            /*cMo.extract(thetau);
            RV[0][0]=HMVraie[im][0];
            RV[0][1]=HMVraie[im][1];
            RV[0][2]=HMVraie[im][2];
            RV[1][0]=HMVraie[im][4];
            RV[1][1]=HMVraie[im][5];
            RV[1][2]=HMVraie[im][6];
            RV[2][0]=HMVraie[im][8];
            RV[2][1]=HMVraie[im][9];
            RV[2][2]=HMVraie[im][10];
            trV[0]=HMVraie[im][3];
            trV[1]=HMVraie[im][7];
            trV[2]=HMVraie[im][11];

            cMo.extract(R0);
            vpRotationMatrix R2 = R0*R1;
            cMo0.buildFrom(tr,R2);
            cMoV.buildFrom(trV,RV);
            vpThetaUVector thetauV(cMoV);
            vpMatrix Rd=RV*(R2.transpose());
            //vpColVector eigv = Rd.eigenValues();
            double trace = Rd[0][0]+Rd[1][1]+Rd[2][2];
            double roterror = acos((trace-1)/2);
            cout << Rd*(Rd.transpose()) << endl;
            vpRotationMatrix Rd0;
            Rd0 = Rd;
            double transerror = sqrt((rap*tr[0]-trV[0])*(rap*tr[0]-trV[0])+(rap*tr[1]-trV[1])*(rap*tr[1]-trV[1])+(rap*tr[2]-trV[2])*(rap*tr[2]-trV[2]));

            vpThetaUVector tuerror(Rd0);*/

            /*error[im][0] = transerror;
            error[im][1] = roterror;
            error[im][2] = im;

            Pose[im][0]=rap*tr[0]-trV[0];
            Pose[im][1]=rap*tr[1]-trV[1];
            Pose[im][2]=rap*tr[2]-trV[2];
            Pose[im][3]=tuerror[0];
            Pose[im][4]=tuerror[1];
            Pose[im][5]=tuerror[2];
            Pose[im][6]=im;
            cMo0 = cMo0.inverse();
            cMo0.extract(trV);
            transl[im][0]=rap*trV[0];
            transl[im][1]=rap*trV[1];
            transl[im][2]=rap*trV[2];
            cMoV = cMoV.inverse();
            cMoV.extract(trV);
            transl[im][3]=trV[0];
            transl[im][4]=trV[1];
            transl[im][5]=trV[2];
            transl[im][6]=im;*/

            /*cout << " okP " << cMo0 << endl;
            cout << " okV " << cMoV << endl;
            cout << " okV " << tuerror << endl;*/

            /*rap += trV[0]/tr[0];
            rap0 += trV[1]/tr[1];
            rap1 += trV[2]/tr[2];*/



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

            //{ogre.updateClipDistances0(tr[2],1);}

            //{ogre.updateClipDistances0(70,69.8);}

            //{ogre.updateClipDistances0(100,95);}

            //{ogre.updateClipDistances0(tr[2],6);}

           /*if (tr[2]>0.30)
           {
        	   ogre.updateClipDistances0(tr[2],0.10);
           }
           else
           {
        	   ogre.updateClipDistances0(tr[2],0.10);
           }*/

           if (tr[2]>0.30)
           {
        	   ogre.updateClipDistances0(tr[2],2.5);
           }
           else
           {
        	   ogre.updateClipDistances0(tr[2],2.5);
           }

            /*Pose[im][0]=tr[0];
            Pose[im][1]=tr[1];
            Pose[im][2]=tr[2];
            Pose[im][3]=0;
            Pose[im][4]=0;
            Pose[im][5]=0;
            Pose[im][6]=im;*/
            t0= vpTime::measureTimeMs();
            //ogre.display(IC,&cMo);
            //ogre.rtt(I2,&cMo);

            //ogre.updateRTTex(Inormd,&cMo);
            //vpImageIo::writePPM(Inormd, "text1.pgm");
            t0= vpTime::measureTimeMs();

            //ogre.updateRTTex(Itex,Inormd,Ior,&cMo);

            std::cout<<" cMo1 " <<cMo<<std::endl;

            ogre.updateRTT(Inormd,Ior,&cMo);

            //ogre.updateMultiRTT(Inormd,Ior,&cMo);
            //vpImageConvert::convert(Inormd1,Itex);
            /*for (int n=0; n < 480 ; n++)
                                    {
                                    for (int m = 0 ; m < 640; m++)
                                      {Ior[n][m]=Inormd[n][m].A;

                                      }
                                    }
            vpImageIo::writePPM(Ior, "imtext.pgm");
            //vpImageIo::writePPM(Inormd, "imtext.pgm");
            getchar();*/


            //Views.edgeOrientMap(Itex);

            //ogre.updateRTT(Inormd,Ior,&cMo);
            t1= vpTime::measureTimeMs();

            cout << "timeRend "<< t1-t0 << endl;

            //ogre.updateMultiRTT(Inormd,Ior,&cMo);

            //vpImageIo::writePPM(Itex, "text00.pgm");
            //vpImageIo::writePPM(Ior, "text01.pgm");
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
            //vpImageIo::writePPM(Inormd, "SoyNorm.pgm");
            //getchar();


            /*for (int n=0; n < 480 ; n++)
            {
            for (int m = 0 ; m < 640; m++)
              {
            	if(Inormd[n][m].A>0)
            I2[n][m]=Inormd[n][m].A;
            	else I2[n][m]=255;

            }
            }*/
            //vpImageIo::writePPM(I2, "Soydepthmap.pgm");
            /*vpDisplay::display(I2);
            vpDisplay::flush(I2);*/

           /* for (int n=0; n < 480 ; n++)
            {
            for (int m = 0 ; m < 640; m++)
              {
                //I1[n][m] =  (double)I2[n][m]/255;
            	if ((double)Ior[n][m]!=100)
            	I2[n][m]=255;
            	else
            	I2[n][m]=0;
              }
            }*/
            //vpImageIo::writePPM(I2, "Soyedgemap.pgm");
            //vpImageIo::writePPM(Id, "Soyinit.pgm");



            //apEdgeMap::edgeMapLap(I1,I4,Ori,I3,0.8);

            //apEdgeMap::edgeMapLap2(I4,Ori,I3,50);
            //vpDisplay::display(I3);
            //vpDisplay::flush(I3);

            //tracker.track(Id,I3,Zcoord,Ori,Inorm);

            //Soyuz
            //tracker.trackHyb(Id,tr[2],1,ogre);

            //Cube
            //tracker.trackHyb(Id,tr[2],0.06,ogre);

            //Satel
            /*if(tr[2]>0.30)
            {
            tracker.trackHyb(Id,tr[2],0.30,ogre);
            }
            else
            {
            tracker.trackHyb(Id,tr[2],0.10,ogre);
            }*/

            //A380
            //tracker.trackHyb(Id,tr[2],6,ogre);

            //Trakmark
            //tracker.trackHyb(Id,100,95,ogre);
            //getchar();
            //tracker.track(Id,Inormd,Ior,Ior,tr[2],1);
            //tracker.track(Id,Inormd,Ior,Ior,tr[2],0.20);

            //Satel
            /*if(tr[2]>0.30)
            {
            tracker.track(Id,Inormd,Ior,Ior,tr[2],0.15);
            }
            else
            {
            tracker.track(Id,Inormd,Ior,Ior,tr[2],0.15);
            }*/

            tracker.setPose(cMo);

            //tank
            if(tr[2]>0.30)
            {
            tracker.track(Id,Inormd,Ior,Ior,tr[2]);
            }
            else
            {
            tracker.track(Id,Inormd,Ior,Ior,tr[2]);
            }

            //tracker.track(Id,Inormd,Ior,70,69.8);

            //tracker.track(Id,Inormd,Ior,Itex,100,95);

            //tracker.track(Id,Inormd,Ior,Ior,tr[2],6);

            t1= vpTime::measureTimeMs();
                        cout << "time2 "<<t1-t0<<endl;

            tracker.getPose(cMo);
            /*oMc = cMo.inverse();
            _Pose.buildFrom(oMc);
            Xv[0]=_Pose[0];
            Xv[1]=_Pose[1];
            Xv[2]=_Pose[2];
            Xv[3]=_Pose[3];
            Xv[4]=_Pose[4];
            Xv[5]=_Pose[5];
    		filterK.predict(1);
    		filterK.setXv(Xv);
    		filterK.setQv(Qv);
    		filterK.visionUpdate();
    		X=filterK.getState();

    		_Pose[0]=X[0];
    		_Pose[1]=X[1];
    		_Pose[2]=X[2];
    		_Pose[3]=X[3];
    		_Pose[4]=X[4];
    		_Pose[5]=X[5];

    		oMc.buildFrom(_Pose);
    		cMo=oMc.inverse();*/

            tracker.display(Id,cMo,mcam,vpColor::yellow,1);
            std::cout<<" cMo0 " <<cMo<<std::endl;



            vpDisplay::flush(Id);
            //getchar();
            vpDisplay::getImage(Id,Ioverlay);

            //vpDisplay::getClick(Id,true);
    		//getchar();
            //vpImageIo::writePPM(Ioverlay, "Soysearch.pgm");
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
                         	//Pose.saveMatrix(posepath3,Pose,false,"");
			            //getchar();

           		        //tracker.display(Id, cMo, mcam, vpColor::darkRed, 2);
           		        //vpDisplay::displayFrame (Id, cMo, mcam, 0.05, vpColor::blue);
           		        //vpDisplay::flush(Id);

		}
		/*cout << rap << " " << rap0 << " " << rap1 << endl;
		rap = (rap+rap0+rap1)/(3*im);

		cout << "rap " << rap << endl;*/

		Pose.resize(im,7,false);

		//char buf3[FILENAME_MAX];
		//sprintf(buf3, posepath.c_str(), m);
		Pose.saveMatrix(posepath3,Pose,false,"");

		transl.resize(im,6,false);
		//transl.saveMatrix(posepath1,transl,false,"");
		// Close the framegrabber

		error.resize(im,2,false);
		//error.saveMatrix(posepath2,error,false,"");



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

