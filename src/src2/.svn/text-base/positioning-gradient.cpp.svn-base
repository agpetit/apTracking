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



#define  Z             0.6


void
convert(const vpImage<double> &I, vpImage<unsigned char> &Ir)
{
  double max = I.getMaxValue() ;
  double min = I.getMinValue() ;
  float A; float vAinf; float  vA;
  float B; float vBsup; float  vB ;


  A = min ;
  B = max ;
  
  vAinf = 0 ; vA = 0 ;
  vBsup = 255 ; vB = 255 ;

  double pente=(double)(vB-vA)/(double)(B-A);

  int i ;
  for (i=0 ; i < I.getCols()*I.getRows() ; i++)
  {
    double vC ;
    vC = I.bitmap[i] ;
    if (vC <A) Ir.bitmap[i] = vAinf ;
    else
      if (vC>B) Ir.bitmap[i] = vBsup ;
      else
	Ir.bitmap[i] =(float)((double)vA+pente*(double)(vC-A));
  }

  //  cout << "max = " << max << endl ;
  //  for (int i = 0 ; i < I.getCols()*I.getRows() ; i++)
  //   Ir.bitmap[i] =  (unsigned char)(min + 255.0*I.bitmap[i]/(max-min)) ;

}

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
  vpImage<unsigned char> Ilap, Ilap_error; // current image



  try {
    g.open(I) ;
  }
  catch(...) {
    cout << "The program was stopped..." << endl;
    return 0;
  }

  Ilap.init(I.getRows(), I.getCols()) ;
  Ilap_error.init(I.getRows(), I.getCols()) ;

  //aquire an image
  for (int i=0 ; i < 10 ; i++)  g.acquire(I) ;

  // ----------------------------------------------------------
  // display the image
  vpDisplayX d, dL, dLe;
  d.init(I, 720, 10, "Laplacian-based  visual servoing : s") ;
  vpDisplay::display(I) ;
  vpDisplay::flush(I) ;

  dL.init(Ilap, 1220, 10, "Laplacian") ;
  dLe.init(Ilap_error, 1220, 300, "Error on Laplacian") ;

  // ----------------------------------------------------------
  // position the robot at the initial position
  // ----------------------------------------------------------


  robot.setRobotState(vpRobot::STATE_POSITION_CONTROL) ;
  cout << " positionnememt" <<endl ;

  vpColVector q ;
  robot.readPosFile("/udd/marchand/positions/luminance/laplacian-v0.pos",q) ; 
  robot.setPositioningVelocity(10) ; 
  robot.setPosition(vpRobot::ARTICULAR_FRAME,q) ; 

  cout << "\n Click" << endl ;
  //  vpDisplay::getClick(I) ;
  vpTime::wait(1000) ;
  
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


  vpImageIo::writeJPEG(I,"/tmp/prog/Id.jpg");
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
  vpFeatureGradient sL ;
  sL.init( I.getHeight(), I.getWidth(), Z) ;
  sL.setCameraParameters(cam) ;
  sL.buildFrom(I) ;
  
  convert(sL.imLap, Ilap) ;
  vpDisplay::display(Ilap) ;
  vpDisplay::flush(Ilap) ;

 


  // desired visual feature built from the image 
  vpFeatureGradient sLd ;
  sLd.init(I.getHeight(), I.getWidth(),  Z) ;
  sLd.setCameraParameters(cam) ;
  sLd.buildFrom(Id) ;

  convert(sL.imLap-sLd.imLap, Ilap_error) ;
  vpDisplay::display(Ilap_error) ;
  vpDisplay::flush(Ilap_error) ;  
  convert(sLd.imLap,I) ;

  vpImageIo::writeJPEG(I,"/tmp/prog/Ld.jpg");
  convert(sL.imLap,I) ;
  vpImageIo::writeJPEG(I,"/tmp/prog/d.jpg");
 g.acquire(I) ; 
  // Matrice d'interaction, Hessien, erreur,...
  vpMatrix Lsd;   // matrice d'interaction a la position desiree
  vpMatrix Hsd;  // hessien a la position desiree
  vpMatrix H ; // Hessien utilise pour le levenberg-Marquartd
  vpColVector error ; // Erreur I-I*

  // Compute the interaction matrix
  // link the variation of image intensity to camera motion

  // here it is computed at the desired position
  sLd.interaction(Lsd) ;

  
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

  double mu,muGN ;  // mu = 0 : Gauss Newton ; mu != 0  : LM
  double lambdaGN;

  // ----------------------------------------------------------
  // ----------------------------------------------------------
  // ----------------------------------------------------------
  // ----------------------------------------------------------

  mu       = 0.001 ;// 0.001;
  muGN       = 0.0000001 ;// 0.001;
  lambda   = 0.3 ;
  lambdaGN = 0.1;
  int iterGN = 170 ; // swicth to Gauss Newton after iterGN iterations
  int iterFin = 1000 ; // swicth to Gauss Newton after iterGN iterations


  // ----------------------------------------------------------
  // ----------------------------------------------------------
  // ----------------------------------------------------------
  // ----------------------------------------------------------
  // ----------------------------------------------------------

  double k =2 ;

  // ---------------------------------------------------------------
  // Set the robot at its initial position 

  char posdes[FILENAME_MAX] ;
  
  cout << "Numero de la position desiree " <<endl ;
  int a ; 
    a =3 ;//
    // cin >> a;
  sprintf(posdes,"/udd/marchand/positions/luminance/laplacian-v%d.pos",a) ;
  robot.setPosition(posdes) ;
  


  for (int i=0 ; i < 10 ; i++) g.acquire(I) ; 

  vpImageIo::writeJPEG(I,"/tmp/prog/I0.jpg");

  vpImageTools::imageDifference(I,Id,Idiff) ;
  vpDisplay::display(I) ;
  vpDisplay::display(Idiff) ;
  vpDisplay::flush(I) ;
  vpDisplay::flush(Idiff) ;




  // set a velocity control mode 
  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

  // ----------------------------------------------------------
  double temps = 0 ;
  int iter ;  iter     = 1;

  vpList<vpImage<unsigned char> > lIdiff, lIlap_error ;
  vpList<vpImage<unsigned char> > lI,lIlap ;

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
      sL.buildFrom(I) ;
      
      convert(sL.imLap, Ilap) ;
      vpDisplay::display(Ilap) ;
      vpDisplay::flush(Ilap) ;

         
      convert(sL.imLap-sLd.imLap, Ilap_error) ;
      vpDisplay::display(Ilap_error) ;
      vpDisplay::flush(Ilap_error) ;
   
      lIdiff += Idiff ;
      lI += I ;
   
      lIlap += Ilap ;
      lIlap_error += Ilap_error ;
   
      // compute current error
      sL.error(sLd,error) ;
      /*
       sL.interaction(Lsd) ;
      Hsd = Lsd.AtA() ;
      diagHsd.eye(n);
      for(int i = 0 ; i < 6 ; i++) diagHsd[i][i] = Hsd[i][i];
      */
      
      double normeError = (error.sumSquare());
      cout << "|e| "<<normeError <<endl ;
      
     
      double t = vpTime::measureTimeMs() ;


      // ---------- Methode Levenberg Marquardt --------------



      {
	if (iter > iterGN)
	  {
	    mu /= 1.002 ; //muGN;
	    lambda = lambdaGN ;
	    
	  }

	// Compute the levenberg Marquartd term
	{
	  H = ((mu * diagHsd) + Hsd).pseudoInverse(); 
	}
	//	compute the control law 
	e = H * Lsd.t() *error ;
	//	if (normeError < 0.5e7) lambda = 1 ;
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
      double tt =  sqrt(vpMath::sqr(r[0]) +vpMath::sqr(r[1]) +vpMath::sqr(r[2]) )*1000 ;
      cout << "ctc* = "<< tt<< "mm" <<endl ;
      //     if (tt < 1.0) iter = iterFin+1 ;
      double dt =  vpTime::measureTimeMs() - t0;
      cout  << "time " << dt <<endl ;
      temps+=dt ;
      plot.plot(0, iter, v); 
      plot.plot(2, iter, r); 
      plot.plot(1, 0, iter, sqrt(normeError));
    }
  while(iter < iterFin);


  v = 0 ;
  robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;
  robot.stopMotion() ;
 
  // Fichiers de sauvegarde
  for(int n=0;n<nb_fic; n++) fic[n].close();
  lIdiff.front() ;

  iter = 0 ;
  char st[FILENAME_MAX] ;
  while (!lIdiff.outside())
  {
    I = lIdiff.value() ; lIdiff.next() ;
   
    sprintf(st,"/tmp/prog/img/image.diff.%04d.jpg",iter++) ;

    vpImageIo::writeJPEG(I,st) ;
  }
  lI.front() ;iter =0 ;
  while (!lI.outside())
  {
    I = lI.value() ; lI.next() ;
    
    sprintf(st,"/tmp/prog/img/image.%04d.jpg",iter++) ;

    vpImageIo::writeJPEG(I,st) ;
  }
  iter = 0 ;
  lIlap_error.front() ;
  while (!lIlap_error.outside())
  {
    I = lIlap_error.value() ; lIlap_error.next() ;
   
    sprintf(st,"/tmp/prog/img/lap.diff.%04d.jpg",iter++) ;

    vpImageIo::writeJPEG(I,st) ;
  }
  lIlap.front() ;iter =0 ;
  while (!lIlap.outside())
  {
    I = lIlap.value() ; lIlap.next() ;
    
    sprintf(st,"/tmp/prog/img/lap.%04d.jpg",iter++) ;

    vpImageIo::writeJPEG(I,st) ;
  }

}

#else

int
main(int argc, char *argv[])
{
  vpTRACE("Afma6 Robot not detected") ;
}
#endif
