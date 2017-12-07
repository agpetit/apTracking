/****************************************************************************
 *
 * $Id:  2457 2010-01-07 10:41:18Z nmelchio $
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 *
 * Authors:
 * Eric Marchand
 * Christophe Collewet
 *
 *****************************************************************************/

/*!
  \example luminance-LM.cpp

  Implemented from C. Collewet, E. Marchand, F. Chaumette. Visual
  servoing set free from image processing. In IEEE Int. Conf. on
  Robotics and Automation, ICRA'08, Pages 81-86, Pasadena, Californie,
  Mai 2008.
*/


#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#ifdef VISP_HAVE_AFMA6

#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpTime.h>
#include <visp/vpRobotAfma6.h>



#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDisplayX.h>


#include <tracking/vpFeatureGradient.h>
#include <visp/vpServo.h>
#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>
#include <visp/vp1394TwoGrabber.h>

#include <visp/vpImageTools.h>
#include <visp/vpImageIo.h>
#include <visp/vpPlot.h>



using namespace std ;



#define  Z             0.7


int
main(int argc, char *argv[])
{
  // 
  vpPlot plot(3);

  // ----------------------------------------------------------------------
  //
  // Create the plot for camera velocities, positions, and error vector
  //
  // ----------------------------------------------------------------------
  {
    // The first graphic contains 6 data to plot: camera velocity
    plot.initGraph(0, 6);
    // The second graphic contains 1 curve, the cost function ||I-I*||
    plot.initGraph(1, 1);
    // The third graphic contains 6 data to plot: camera position
    plot.initGraph(2, 6);

    // For the first graphic :
    // - along the x axis the expected values are between 0 and 200 and 
    //   the step is 10 
    // - along the y axis the expected values are between -0.01 and 0.01 and the
    //   step is 0.005
    plot.initRange(0,0,200,100,-0.001,0.001,0.005);
    plot.setTitle(0, "Camera Velocity");
    // For the first graphic :
    // - along the x axis the expected values are between 0 and 200 and 
    //   the step is 10 
    // - along the y axis the expected values are between -0.01 and 0.01 and the
    //   step is 0.005
    plot.initRange(2,0,200,100,-0.001,0.001,0.005);
    plot.setTitle(2, "Error on camera position");
    // For the second graphic :
    // - along the x axis the expected values are between 0 and 200 and 
    //   the step is 1 
    // - along the y axis the expected values are between 0 and 0.0001 and the
    //   step is 0.00001
    plot.initRange(1,0,200,10,0,10000,1e7);
    plot.setTitle(1, "||I-I*||");

    // For the first graphic, set the curves legend
    char legend[10];
    for (int i=0; i < 6; i++) {
      sprintf(legend, "v%d", i+1);
      plot.setLegend(0, i, legend);   
      sprintf(legend, "r%d", i+1);
      plot.setLegend(2, i, legend);
    }
  
 
    // Set the curves color
    plot.setColor(0, 0, vpColor::red); 
    plot.setColor(0, 1, vpColor::green); 
    plot.setColor(0, 2, vpColor::blue); 
    plot.setColor(0, 3, vpColor::orange); 
    plot.setColor(0, 4, 0, 128, 0); 
    plot.setColor(0, 5, vpColor::cyan); 
    // Set the curves color
    plot.setColor(2, 0, vpColor::red); 
    plot.setColor(2, 1, vpColor::green); 
    plot.setColor(2, 2, vpColor::blue); 
    plot.setColor(2, 3, vpColor::orange); 
    plot.setColor(2, 4, 0, 128, 0); 
    plot.setColor(2, 5, vpColor::cyan); 

    // For the second graphic, set the curves legend
    plot.setLegend(1, 0, "||I-I*||");
  }


  vp1394TwoGrabber  g;
  g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_320x240_YUV422) ;
    
 
  // create the robot (here the INRIA Rennes Afma6)
  vpRobotAfma6 robot ;
  robot.init(vpAfma6::TOOL_CCMOP, 
	     vpCameraParameters::perspectiveProjWithoutDistortion);
    
  // ----------------------------------------------------------
  // Fichiers de sauvegarde
  int      nb_fic = 3;
  ofstream fic[nb_fic];
  fic[0].open("/tmp/prog/axe");
  fic[1].open("/tmp/prog/ds");
  fic[2].open("/tmp/prog/pose_relative");

  vpTime::wait(1000) ;

  // ----------------------------------------------------------
  vpImage<unsigned char> I; // current image
  // Create the framegraber
 

  try {
    g.open(I) ;
  }
  catch(...) {
    cout << "The program was stopped..." << endl;
    return 0;
  }

  //aquire an image
  for (int i=0 ; i < 10 ; i++)  g.acquire(I) ;

  // ----------------------------------------------------------
  // display the image
  vpDisplayX d;
  d.init(I, 720, 10, "Photometric visual servoing : s") ;
  vpDisplay::display(I) ;
  vpDisplay::flush(I) ;

  // ----------------------------------------------------------
  // position the robot at the initial position
  // ----------------------------------------------------------


  robot.setRobotState(vpRobot::STATE_POSITION_CONTROL) ;
  cout << " positionnememt" <<endl ;

  vpColVector q ;
  robot.readPosFile("/udd/marchand/positions/luminance/init-v0.pos",q) ; 
  robot.setPositioningVelocity(10) ; 
  robot.setPosition(vpRobot::ARTICULAR_FRAME,q) ; 

  cout << "\n Click" << endl ;
  vpDisplay::getClick(I) ;

  
  for (int i=0 ; i < 10 ; i++) g.acquire(I) ;

  vpDisplay::display(I) ;
  vpDisplay::flush(I) ;
  
  // get the desired position fMcd in the reference from the robot odometry
  // will be used to compte the error cMcd between current and desired position
  // during the servoing task
  vpHomogeneousMatrix fMcd, fMc,cMcd ;
  

  vpColVector crf(6); // current robot position

  {
    robot.getPosition( vpRobot::REFERENCE_FRAME,crf) ;
    vpTranslationVector tr( crf[0], crf[1], crf[2]) ;
    vpRxyzVector r( crf[3], crf[4], crf[5]) ;
    vpRotationMatrix R(r) ;
    //    vpHomogeneousMatrix fMc ;
    fMcd.buildFrom(tr,R) ;
    fMc.buildFrom(tr,R) ;
  }


  

  // ------------------------------------------------------
  //  camera parameters

  vpCameraParameters cam;
  robot.getCameraParameters (cam, I);
  cam.printParameters() ;

  // ------------------------------------------------------
  // Acquisition de l'image de reference

  for (int i=0 ; i < 10 ; i++) g.acquire(I) ;

  vpImage<unsigned char> Id,Idiff ;
  Id = I ;
  Idiff = I ;


  vpImageTools::imageDifference(I,Id,Idiff) ;


  vpImageIo::writePGM(I,"/tmp/prog/Id.pgm");
  // Affiche de l'image de difference
  vpDisplayX d1;
  d1.init(Idiff, 720, 300, "LBVS : s-s* ") ;
  vpDisplay::display(Idiff) ;
  vpDisplay::flush(Idiff) ;


  // ------------------------------------------------------
  // Visual feature, interaction matrix, error
  // s, Ls, Lsd, Lt, Lp, etc
  // ------------------------------------------------------

  // current visual feature built from the image 
  // (actually, this is the image...)
  vpFeatureLuminance sI ;
  sI.init( I.getHeight(), I.getWidth(), Z) ;
  sI.setCameraParameters(cam) ;
  vpTRACE(" ") ;
  sI.buildFrom(I) ;
  

  // desired visual feature built from the image 
  vpFeatureLuminance sId ;
  sId.init(I.getHeight(), I.getWidth(),  Z) ;
  sId.setCameraParameters(cam) ;
  vpTRACE(" ") ;
  sId.buildFrom(Id) ;

 
  
  // Matrice d'interaction, Hessien, erreur,...
  vpMatrix Lsd;   // matrice d'interaction a la position desiree
  vpMatrix Hsd;  // hessien a la position desiree
  vpMatrix H ; // Hessien utilise pour le levenberg-Marquartd
  vpColVector error ; // Erreur I-I*

  // Compute the interaction matrix
  // link the variation of image intensity to camera motion

  // here it is computed at the desired position
  sId.interaction(Lsd) ;

  
  // Compute the Hessian H = L^TL
  Hsd = Lsd.AtA() ;

  // Compute the Hessian diagonal for the Levenberg-Marquartd 
  // optimization process
  int n = 6 ;
  vpMatrix diagHsd(n,n) ;
  diagHsd.eye(n);
  for(int i = 0 ; i < n ; i++) diagHsd[i][i] = Hsd[i][i];



  // ------------------------------------------------------
  // Control law
  double lambda ; //gain
  vpColVector e ;
  vpColVector v ; // camera velocity send to the robot


  // ----------------------------------------------------------
  // Minimisation

  double mu ;  // mu = 0 : Gauss Newton ; mu != 0  : LM
  double lambdaGN;


  mu       =  0.001;
  lambda   = 3 ;
  lambdaGN = 1;




  double k =2 ;

  // ---------------------------------------------------------------
  // Set the robot at its initial position 

  char posdes[FILENAME_MAX] ;
  
  cout << "Numero de la position desiree " <<endl ;
  int a ; 
  cin >> a;
  sprintf(posdes,"/udd/marchand/positions/luminance/desired-v%d.pos",a) ;
  robot.setPosition(posdes) ;
  


  for (int i=0 ; i < 10 ; i++) g.acquire(I) ; 

  vpImageIo::writePGM(I,"/tmp/prog/I0.pgm");

  vpImageTools::imageDifference(I,Id,Idiff) ;
  vpDisplay::display(I) ;
  vpDisplay::display(Idiff) ;
  vpDisplay::flush(I) ;
  vpDisplay::flush(Idiff) ;
  vpDisplay::getClick(I) ;




  // set a velocity control mode 
  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

  // ----------------------------------------------------------
  double temps = 0 ;
  int iter ;  iter     = 1;
  int iterGN = 500 ; // swicth to Gauss Newton after iterGN iterations

  do
    {

      cout << "-----------------------------------------------" << endl ;
      double t0 =  vpTime::measureTimeMs();


      //  Acquire the new image
      g.acquire(I) ;
      vpDisplay::display(I) ;
      vpDisplay::flush(I) ;

      vpImageTools::imageDifference(I,Id,Idiff) ;
      vpDisplay::display(Idiff) ;
      vpDisplay::flush(Idiff) ;

      // Compute current visual feature
      sI.buildFrom(I) ;

      
      // compute current error
      sI.error(sId,error) ;


      
      double normeError = (error.sumSquare());
      cout << "|e| "<<normeError <<endl ;
      
     
      double t = vpTime::measureTimeMs() ;


      // ---------- Methode Levenberg Marquardt --------------



      {
	if (iter > iterGN)
	  {
	    vpTRACE(" ") ;
	    mu /= 2;
	    lambda = 0.1 ;
	    
	  }

	// Compute the levenberg Marquartd term
	{
	  H = ((mu * diagHsd) + Hsd).pseudoInverse(); 
	}
	//	compute the control law 
	e = H * Lsd.t() *error ;
	if (normeError < 0.5e7) lambda = 1 ;
	v = - lambda*e;
      }

      cout << "temps = "<< temps << "s (" << iter << ")  "<<endl ;
      cout << "lambda = " << lambda << "  mu = " << mu ;
      cout <<" |s| = " << normeError <<" |Tc| = " << sqrt(v.sumSquare()) << endl;
      
      // send the robot velocity
      robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

      // get the current position
      {
	robot.getPosition( vpRobot::REFERENCE_FRAME,crf) ;
	vpTranslationVector tr( crf[0], crf[1], crf[2]) ;
	vpRxyzVector r( crf[3], crf[4], crf[5]) ;
	vpRotationMatrix R(r) ;

	fMc.buildFrom(tr,R) ;
	// and compute the error (for precision assessment only)
	cMcd = fMc.inverse()*fMcd ;
      }

      // save information
 	vpColVector r(6) ;
     {
	{
	  vpPoseVector frc(fMc) ;
	  for (int i=0 ; i < 6 ; i++)
	    fic[2] << frc[i] <<" ";
	  vpPoseVector crcd(cMcd) ;
	  for (int i=0 ; i < 6 ; i++)
	    {
	      fic[2] << crcd[i] <<" ";
	      r[i] = crcd[i] ;
	    }
	}
	fic[2] <<endl ;
    
	// Sauvegarde
	fic[0] <<  temps << " ";
	for(int n=0; n<6; n++) fic[0] << v[n] << " " ;
	fic[0] << lambda  << " " << mu << endl;

	fic[1] << temps <<" " << normeError << " " << log(vpMath::sqr(normeError))  <<  endl;
      }
      iter++;
      cout << "ctc* = "<< sqrt(vpMath::sqr(r[0]) +vpMath::sqr(r[1]) +vpMath::sqr(r[2]) )*1000 << "mm" <<endl ;

      double dt =  vpTime::measureTimeMs() - t0;
      cout  << "time " << dt <<endl ;
      temps+=dt ;
      plot.plot(0, iter, v); 
      plot.plot(2, iter, r); 
      plot.plot(1, 0, iter, normeError);
    }
  while(iter < 1800);


  v = 0 ;
  robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;
  robot.stopMotion() ;
 
  // Fichiers de sauvegarde
  for(int n=0;n<nb_fic; n++) fic[n].close();

}

#else

int
main(int argc, char *argv[])
{
  vpTRACE("Afma6 Robot not detected") ;
}
#endif
