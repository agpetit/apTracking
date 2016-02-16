/*
 * vpMbPointsTracker.cpp
 *
 *  Created on: March 10, 2011
 *      Author: Antoine Petit
 */


#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#include <visp/vpPose.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpImageIo.h>
#include <visp/vpRobust.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpMatrixException.h>
#include <visp/vpMatrix.h>
#include <visp/vpImagePoint.h>

#include <visp/vpException.h>
#include <visp/vpTrackingException.h>

#include "vpMbPointsTracker.h"
#include "vpMbtControlPoint.h"
#include "luaconfig.h"


#include <string>
#include <sstream>


/*!
  Basic constructor
*/
vpMbPointsTracker::vpMbPointsTracker()
{
  compute_interaction=1;
  npoints = 0;
  lambda = 1;
  percentageGdPt = 0.1;
  points.resize(1);
  scales.resize(1);
  scales[0] = true;
  points[0].resize(0);
  Ipyramid.resize(0);
}

/*!
  Basic destructor useful to deallocate the memory.
*/
vpMbPointsTracker::~vpMbPointsTracker()
{
  vpMbtControlPoint *p ;
  
  for (unsigned int i = 0; i < points.size(); i += 1){
    if(scales[i]){
    for (int k = 0; k<points[i].size(); k++){
        p = (points[i])[k] ;
        if (p!=NULL) delete p ;
        p = NULL ;
      }
      points[i].resize(0) ;
    }
  }
  points.resize(0);
  cleanPyramid(Ipyramid);
}

/*void
vpMbPointsTracker::setOgre(const exampleVpAROgre &ogre_)
{
ogre = ogre_;
}*/

/*!
  Set the moving edge parameters.

  \param _me : an instance of vpMe containing all the desired parameters
*/
void
vpMbPointsTracker::setMovingEdge(const vpMe &_me)
{
  this->me = _me;

  for (unsigned int i = 0; i < points.size(); i += 1){
    if(scales[i]){
      vpMbtControlPoint *p ;
      for (int k = 0; k<points[i].size(); k++)
      {
        p = (points[i])[k] ;
        p->setMovingEdge(&me) ;
      }
    }
  }
}


/*!
  Compute the visual servoing loop to get the pose of the feature set.
  
  \exception vpTrackingException::notEnoughPointError if the number of detected 
  feature is equal to zero. 
  
  \param _I : The current image. 
 */
void
vpMbPointsTracker::computeVVS(const vpImage<unsigned char>& _I)
{
  double residu_1 =1e3;
  double r =1e3-1;
  vpMatrix LTL;
  vpColVector LTR;

  // compute the interaction matrix and its pseudo inverse
  vpMbtControlPoint *p ;

  vpColVector w;
  vpColVector weighted_error;
  vpColVector factor;

  unsigned int iter = 0;

  //Nombre de moving edges
  int nbrow  = 0;
  vpFeatureLine fli;
  
	  for (int k = 0; k<points[scaleLevel].size(); k++)
  {
    p = (points[scaleLevel])[k];
    nbrow += 1;
    p->initInteractionMatrixError();
    //
    //std::cout<<fli.getTheta()<<std::endl;
  }

  if (nbrow==0)
  {
    vpERROR_TRACE("\n\t\t Error-> not enough data in the interaction matrix...") ;
    throw vpTrackingException(vpTrackingException::notEnoughPointError, "\n\t\t Error-> not enough data in the interaction matrix...");
  }
  
  vpMatrix L(nbrow,6), Lp;

  // compute the error vector
  vpColVector error(nbrow);
  int nerror = error.getRows();
  vpColVector v ;

  double limite = 3; //Une limite de 3 pixels
  limite = limite / cam.get_px(); //Transformation limite pixel en limite metre.
  
  //Parametre pour la premiere phase d'asservissement
  double e_prev = 0, e_cur, e_next;
  bool reloop = true;
  double count = 0;

  /*** First phase ***/

  while ( reloop == true && iter<5)
  {
    if(iter==0)
    {
      weighted_error.resize(nerror) ;
      w.resize(nerror);
      w = 0;
      factor.resize(nerror);
      factor = 1;
    }
    
    count = 0;
    //;
    int n = 0;
    //reloop = false;
    reloop = true;
    for (int k = 0; k<points[scaleLevel].size(); k++)
    //for (int k = 0; k<points[scaleLevel].size(); k++)
    {
      //p = (points[scaleLevel])[k];
    	p = (points[scaleLevel])[k];
      p->computeInteractionMatrixError(cMo,_I);
      //p->computeInteractionMatrixError2(cMo,_I);
      //p->computeInteractionMatrixError4(cMo, I ,Igrad , Igradx, Igrady);
      
      double fac = 1;
      /*if (iter == 0)
      {
            fac = 0.2;
            break;
      }*/
      
      //std::cout << " fac " << fac << std::endl;

      if (iter == 0 && p != NULL)
      
      //for (int i=0 ; i < 1 ; i++)
      //{
        for (int j=0; j < 6 ; j++)
        {
          L[n][j] = p->L[0][j]; //On remplit la matrice d'interaction globale
        }
        error[n] = p->error[0]; //On remplit la matrice d'erreur

        if (error[n] <= limite) count = count+1.0; //Si erreur proche de 0 on incremente cur

        w[n] = 0;

        if (iter == 0)
        {
          factor[n] = fac;
          vpPointSite site = p->list[0];
          //vpPointSite site = p->list.value();
          //if (site.suppress != 0) factor[n] = 0;
          if (site.suppress != 0) factor[n] = 0.2;
        }

        //If pour la premiere extremite des moving edges
        /*if (i == 0)
        {
          e_cur = l->error[0];
          if (l->nbFeature > 1)
          {
            e_next = l->error[1];
            if ( fabs(e_cur - e_next) < limite && vpMath::sign(e_cur) == vpMath::sign(e_next) )
            {
              w[n+i] = 1/*0.5*/;
          /*  }
            e_prev = e_cur;
          }
          else w[n+i] = 1;
        }*/

        //If pour la derniere extremite des moving edges
        /*else if(i == l->nbFeature-1)
        {
          e_cur = l->error[i];
          if ( fabs(e_cur - e_prev) < limite && vpMath::sign(e_cur) == vpMath::sign(e_prev) )
          {
            w[n+i] += 1/*0.5*/;
          /*}
        }*/

        //else
        //{
          e_cur = p->error[0];
          e_next = p->error[0];
          if ( fabs(e_cur - e_prev) < limite )
          {
            w[n] += 0.5;
            //w[n]=1;
          }
          if ( fabs(e_cur - e_next) < limite )
          {
            w[n] += 0.5;
            //w[n]=1;
          }
          e_prev = e_cur;
        //}
      //}
      
      n+= 1 ;
      //
    }

    count = count / (double)nbrow;
    if (count < 0.85)
    {
      reloop = true;
    }

    double num=0;
    double den=0;

    double wi ; double eri ;
    for(int i = 0; i < nerror; i++)
    {
      wi = w[i]*factor[i];
      eri = error[i];
      num += wi*vpMath::sqr(eri);
      den += wi ;

      weighted_error[i] =  wi*eri ;
    }

    if((iter==0) || compute_interaction)
    {
      for (int i=0 ; i < nerror ; i++)
      {
        for (int j=0 ; j < 6 ; j++)
        {
          L[i][j] = w[i]*factor[i]*L[i][j] ;
        }
      }
    }

    LTL = L.AtA();
    computeJTR(L, weighted_error, LTR);
    v = -0.7*LTL.pseudoInverse(LTL.getRows()*DBL_EPSILON)*LTR;
    cMo =  vpExponentialMap::direct(v).inverse() * cMo;

    iter++;
  }
std::cout << "\t First minimization in " << iter << " iteration " << std::endl ;
  
/*** Second phase ***/

  vpRobust robust(nerror);
  robust.setIteration(0) ;
  iter = 0;
  //vpColVector error_px(nerror);

  while (((int)((residu_1 - r)*1e8) !=0 ) && (iter<20))
  {
    // ;
    int n = 0 ;
    for (int k = 0; k<points[scaleLevel].size(); k++)
    //for (int k = 0; k<points[scaleLevel].size(); k++)
    {
      //p = (points[scaleLevel])[k];
      p = (points[scaleLevel])[k];
      p->computeInteractionMatrixError(cMo,_I);
      //p->computeInteractionMatrixError2(cMo,_I);
      //p->computeInteractionMatrixError4(cMo, I ,Igrad , Igradx, Igrady);
      //for (int i=0 ; i < l->nbFeature ; i++)
      //{
        for (int j=0; j < 6 ; j++)
        {
          L[n][j] = p->L[0][j] ;
          error[n] = p->error[0] ;
    //error_px[n+i] = l->error[i] * cam.get_px();
        }
      //}
      n+= 1;
      //
    }
    
    if(iter==0)
    {
      weighted_error.resize(nerror) ;
      w.resize(nerror);
      w = 1;

       robust.setThreshold(2/cam.get_px()); // limite en metre
       robust.MEstimator(vpRobust::TUKEY, error,w);
       //robust.setThreshold(2); // limite en pixel
       //robust.MEstimator(vpRobust::TUKEY, error_px,w);
    }
    else
    {
      robust.setIteration(iter);
      robust.MEstimator(vpRobust::TUKEY, error,w);
      //robust.MEstimator(vpRobust::TUKEY, error_px,w);
    }

    residu_1 = r;

    double num=0;
    double den=0;
    double wi;
    double eri;
    for(int i=0; i<nerror; i++)
    {
      wi = w[i]*factor[i];
      eri = error[i];
      num += wi*vpMath::sqr(eri);
      den += wi;

      weighted_error[i] =  wi*eri ;
    }
    
    r = sqrt(num/den); //Le critere d'arret prend en compte le poids

    if((iter==0) || compute_interaction)
    {
      for (int i=0 ; i < nerror ; i++)
      {
        for (int j=0 ; j < 6 ; j++)
        {
          L[i][j] = w[i]*factor[i]*L[i][j];
        }
      }
    }

    LTL = L.AtA();
    computeJTR(L, weighted_error, LTR);
    v = -lambda*LTL.pseudoInverse(LTL.getRows()*DBL_EPSILON)*LTR;

    cMo =  vpExponentialMap::direct(v).inverse() * cMo;

    iter++;
  }

  //std::cout<<iter<<std::endl;
   ;
  int n =0 ;
  for (int k = 0; k<points[scaleLevel].size(); k++)
  {
    p = (points[scaleLevel])[k] ;
    {
      double wmean = 0 ;
      /*if (p->nbFeature > 0)*/ //
      
      //for (int i=0 ; i < p->nbFeature ; i++)
      //{
        wmean += w[n] ;
        vpPointSite s = p->list[0] ;
        if (w[n] < 0.5)
        {
          s.suppress = 4 ;
          p->list[0] = s ;
        }

      //}
      n+= 1 ;
      
        wmean = 1;
            
      //p->setMeanWeight(wmean);

    }

  }
//   cout << "\t Robust minimization in " << iter << " iteration " << endl ;
//    std::cout << "error: " << (residu_1 - r) << std::endl;
}

/*void
vpMbPointsTracker::computeVVSHyb(const vpImage<unsigned char>& _I, 	apOgre ogre_)
{
  double residu_1 =1e3;
  double r =1e3-1;
  vpMatrix LTL;
  vpColVector LTR;

  // compute the interaction matrix and its pseudo inverse
  vpMbtControlPoint *p ;

  vpColVector w;
  vpColVector weighted_error;
  vpColVector factor;

  vpTranslationVector tr;
  cMo.extract(tr);

  unsigned int iter = 0;

  //Nombre de moving edges
  int nbrow  = 0;
  vpFeatureLine fli;

  ;
  for (int k = 0; k<points[scaleLevel].size(); k++)
  {
    p = (points[scaleLevel])[k];
    nbrow += 1;
    p->initInteractionMatrixError();

    //std::cout<<fli.getTheta()<<std::endl;
  }

  if (nbrow==0)
  {
    vpERROR_TRACE("\n\t\t Error-> not enough data in the interaction matrix...") ;
    throw vpTrackingException(vpTrackingException::notEnoughPointError, "\n\t\t Error-> not enough data in the interaction matrix...");
  }

  vpMatrix L(nbrow,6), Lsd, LT;
 // matrice d'interaction a la position desiree
  vpMatrix Hsd;  // hessien a la position desiree
  vpMatrix H ; // Hessien utilise pour le levenberg-Marquartd

  vpColVector errorG ;

  // compute the error vector
  vpColVector error(nbrow);
  int nerror = error.getRows();
  int nerrorG;
  vpColVector v ;

  double limite = 3; //Une limite de 3 pixels
  limite = limite / cam.get_px(); //Transformation limite pixel en limite metre.

  //Parametre pour la premiere phase d'asservissement
  double e_prev = 0, e_cur, e_next;
  bool reloop = true;
  double count = 0;
  int nbr =480;
  int nbc = 640;
  vpImage<unsigned char> imG(480,640);
  vpImage<unsigned char> Igd(480,640);
  vpImage<unsigned char> Ig(480,640);
  vpImage<unsigned char> Idiff(480,640);
  vpColVector e;

  for (int i=3; i < nbr-3 ; i++)
{
  //   cout << i << endl ;
  for (int j = 3 ; j < nbc-3; j++)
    {
      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
      double Igf =   vpImageFilter::gaussianFilter(_I,i,j) ;
      imG[i][j] = Igf ;

    }
}*/

/*  for (int i=3; i < nbr-3 ; i++)
{
  for (int j = 3 ; j < nbc-3; j++)
    {*/
/*  for (int i=150; i < 350 ; i++)
 {
   for (int j = 150 ; j < 550; j++)
     {
      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
      double Ix =   1 * vpImageFilter::sobelFilterX(imG,i,j);
      double Iy =   1 * vpImageFilter::sobelFilterY(imG,i,j);
      Igd[i][j]= (unsigned char)sqrt(vpMath::sqr(Ix)+vpMath::sqr(Iy));
      //I2[i][j] = (unsigned char)Igrad[i][j];
    }
}

  sId.init(Igd.getHeight(), Igd.getWidth(), tr[2]);
  sI.init(Igd.getHeight(), Igd.getWidth(), tr[2]);
  sId.buildFrom(Igd);
  sId.interaction(Lsd);
  //Lsd=2*Lsd;
  nerrorG = Lsd.getRows();
  vpColVector errorT(nerror+nerrorG);
  vpDisplayX displayo;
  displayo.init(Idiff, 1500, 1000, "display");
  double mu = 0.003;
  vpMatrix diagHsd(6,6);
  vpMatrix diagLTL(6,6);


  Hsd = Lsd.AtA();
  diagHsd.eye(6);
  for(int i = 0 ; i < 6 ; i++) diagHsd[i][i] = Hsd[i][i];

  H = ((mu * diagHsd) + Hsd).pseudoInverse();

  /*** First phase ***/

/*  while ( reloop == true && iter<30)
  {

cMo.extract(tr);
ogre_.updateClipDistances0(tr[2],1);
ogre_.updateRTTGrad(Ig,&cMo);
sI.update(Ig.getHeight(), Ig.getWidth(),tr[2]);
sI.buildFrom(Ig);
sI.error(sId, errorG);

vpImageTools::imageDifference(Ig,Igd,Idiff);
vpDisplay::display(Idiff);
vpDisplay::flush(Idiff);*/


/*H = ((mu * diagHsd) + Hsd).pseudoInverse();
//	compute the control law
e = H * Lsd.t() *errorG;
v =  -0.5*e;
cMo =  vpExponentialMap::direct(v).inverse() * cMo;*/



/*	  if(iter==0)
    {
      weighted_error.resize(nerror+nerrorG) ;
      w.resize(nerror+nerrorG);
      w = 0;
      factor.resize(nerror+nerrorG);
      factor = 1;
    }*/

	/*  if(iter==0)
    {
      weighted_error.resize(nerror) ;
      w.resize(nerror);
      w = 0;
      factor.resize(nerror);
      factor = 1;
    }*/

    /*count = 0;
    ;
    int n = 0;
    //reloop = false;
    reloop = true;

    for (int k = 0; k<points[scaleLevel].size(); k++)
    {
      p = (points[scaleLevel])[k];
      p->computeInteractionMatrixError(cMo,_I);
      //p->computeInteractionMatrixError2(cMo,_I);

      double fac = 1;
      if (iter == 0)
      {
            fac = 0.2;
            break;
      }

      //std::cout << " fac " << fac << std::endl;

      if (iter == 0 && p != NULL)
        p->list.front();

      //for (int i=0 ; i < 1 ; i++)
      //{
        for (int j=0; j < 6 ; j++)
        {
          L[n][j] = p->L[0][j]; //On remplit la matrice d'interaction globale
        }
        error[n] = p->error[0]; //On remplit la matrice d'erreur

        if (error[n] <= limite) count = count+1.0; //Si erreur proche de 0 on incremente cur

        w[n] = 0;

        if (iter == 0)
        {
          factor[n] = fac;
          vpPointSite site = p->list.value();
          if (site.suppress != 0) factor[n] = 0.2;
          p->list.next();
        }

          e_cur = p->error[0];
          e_next = p->error[0];
          if ( fabs(e_cur - e_prev) < limite )
          {
            w[n] += 0.5;
          }
          if ( fabs(e_cur - e_next) < limite )
          {
            w[n] += 0.5;
          }
          e_prev = e_cur;


      n+= 1 ;

    }

    count = count / (double)nbrow;
    if (count < 0.85)
    {
      reloop = true;
    }

    double num=0;
    double den=0;

    double wi; double eri;
    for(int i = 0; i < nerror; i++)
    {
      wi = w[i]*factor[i];
      eri = error[i];
      num += wi*vpMath::sqr(eri);
      den += wi ;

      weighted_error[i] =50000* wi*eri;
      //weighted_error[i] =  wi*eri;
      //weighted_error[i] =  0*wi*eri;
    }
    for(int i = nerror; i < nerror+nerrorG; i++)
    {
      weighted_error[i] =  3*errorG[i-nerror];
    }


    if((iter==0) || compute_interaction)
    {
      for (int i=0 ; i < nerror ; i++)
      {
        for (int j=0 ; j < 6 ; j++)
        {
          L[i][j] = 50000*w[i]*factor[i]*L[i][j] ;
          //L[i][j] = w[i]*factor[i]*L[i][j] ;
        	//L[i][j] = 0*w[i]*factor[i]*L[i][j] ;
        }
      }
    }


    vpMatrix::stackMatrices(L,Lsd,LT);*/
    //LT=L;


   /* H = ((mu * diagHsd) + Hsd).pseudoInverse();
                	H1 = ((mu * diagHsd1) + Hsd1).pseudoInverse();
                	//	compute the control law
                	e = H * Lsd.t() *error;
                	e1 = H1 * Lsd1.t() *error1;
                    double normeError = (error1.sumSquare());
                    //A.plot(0,0,iter,sqrt((normeError)/(I3.getHeight()*I3.getWidth())));
                	//if (normeError < 0.5e7) lambda = 1 ;
                	v =  -lambda*e;*/

    /*double t0= vpTime::measureTimeMs();
    LTL = LT.AtA();
    double t1= vpTime::measureTimeMs();
    std::cout<<"timeiter"<< t1-t0<<std::endl;
    diagLTL.eye(6);
    for(int i = 0 ; i < 6 ; i++) diagLTL[i][i] = LTL[i][i];
    LTL=LTL + 0.003*diagLTL;
    //std::cout << " ltl " << LTL.pseudoInverse() << std::endl;
    //std::cout<<  " H " << H << std::endl;
    //computeJTR(LT, weighted_error, LTR);
    //v = -0.2*LTL.pseudoInverse()*LTR;
    v = -0.5*(LTL.pseudoInverse())*LT.t()*weighted_error;
    //std::cout << v << std::endl;
    cMo =  vpExponentialMap::direct(v).inverse() * cMo;


    iter++;
  }
std::cout << "\t First minimization in " << iter << " iteration00 " << std::endl ;*/


  //std::cout<<iter<<std::endl;
/*   ;
  int n =0 ;
  for (int k = 0; k<points[scaleLevel].size(); k++)
  {
    p = (points[scaleLevel])[k] ;
    {
      double wmean = 0 ;
      //if (p->nbFeature > 0)
      p->list.front();

      //for (int i=0 ; i < p->nbFeature ; i++)
      //{
        wmean += w[n] ;
        vpPointSite s = p->list.value() ;
        if (w[n] < 0.5)
        {
          s.suppress = 4 ;
          p->list.modify(s) ;
        }

        p->list.next();
      //}
      n+= 1 ;

        wmean = 1;

      //p->setMeanWeight(wmean);

    }

  }
//   cout << "\t Robust minimization in " << iter << " iteration " << endl ;
//    std::cout << "error: " << (residu_1 - r) << std::endl;
}*/



void
vpMbPointsTracker::computeVVSCorr(const vpImage<unsigned char>& I, const vpImage<double>& Igrad, const vpImage<double>& Igradx, const vpImage<double>& Igrady)
{
  double residu_1 =1e3;
  double r =1e3-1;
  vpMatrix LTL;
  vpColVector LTR;

  // compute the interaction matrix and its pseudo inverse
  vpMbtControlPoint *p ;

  vpColVector w;
  vpColVector weighted_error;
  vpColVector factor;

  unsigned int iter = 0;

  //Nombre de moving edges
  int nbrow  = 0;
  vpFeatureLine fli;

  for (int k = 0; k<points[scaleLevel].size(); k++)
  {
    p = (points[scaleLevel])[k];
    nbrow += 1;
    p->initInteractionMatrixError();
    //
    //std::cout<<fli.getTheta()<<std::endl;
  }

  if (nbrow==0)
  {
    vpERROR_TRACE("\n\t\t Error-> not enough data in the interaction matrix...") ;
    throw vpTrackingException(vpTrackingException::notEnoughPointError, "\n\t\t Error-> not enough data in the interaction matrix...");
  }

  vpMatrix L(nbrow,6), Lp;

  // compute the error vector
  vpColVector error(nbrow);
  int nerror = error.getRows();
  vpColVector v ;

  double limite = 3; //Une limite de 3 pixels
  limite = limite / cam.get_px(); //Transformation limite pixel en limite metre.

  //Parametre pour la premiere phase d'asservissement
  double e_prev = 0, e_cur, e_next;
  bool reloop = true;
  double count = 0;
  while ( reloop == true && iter<50)
  {
    if(iter==0)
    {
      weighted_error.resize(nerror) ;
      w.resize(nerror);
      w = 0;
      factor.resize(nerror);
      factor = 1;
    }

    count = 0;
    //;
    int n = 0;
    //reloop = false;
    reloop = true;
    vpImage<unsigned char> _I;

    //for (int k = 0; k<points[scaleLevel].size(); k++)
    for (int k = 0; k<points[scaleLevel].size(); k++)
    {
      p = (points[scaleLevel])[k];
      p->computeInteractionMatrixError3(cMo, I ,Igrad , Igradx, Igrady);
      //std::cout << "autoIm00 "   << std::endl;
      //p->computeInteractionMatrixError2(cMo,_I);
      //getchar();


      double fac = 1;
      if (iter == 0)
      {
            fac = 0.2;
            break;
      }


      //std::cout << " fac " << fac << std::endl;

      if (iter == 0 && p != NULL)
        p->list[0];



      //for (int i=0 ; i < 1 ; i++)
      //{
        for (int j=0; j < 6 ; j++)
        {
          L[n][j] = p->L[0][j]; //On remplit la matrice d'interaction globale
        }
        error[n] = p->error[0]; //On remplit la matrice d'erreur


        if (error[n] <= limite) count = count+1.0; //Si erreur proche de 0 on incremente cur

        w[n] = 1;

        if (iter == 0)
        {
          factor[n] = fac;
          vpPointSite site = p->list[0];
          if (site.suppress != 0) factor[n] = 0.2;
          //p->list.next();
        }


        //else
        //{
          e_cur = p->error[0];
          e_next = p->error[0];
          if ( fabs(e_cur - e_prev) < limite )
          {
            w[n] += 0.5;
          }
          if ( fabs(e_cur - e_next) < limite )
          {
            w[n] += 0.5;
          }
          e_prev = e_cur;
        //}
      //}

      n+= 1 ;
      //
    }

    //vpDisplay::flush(I);

    count = count / (double)nbrow;
    if (count < 0.85)
    {
      reloop = true;
    }

    double num=0;
    double den=0;

    double wi ; double eri ;
    for(int i = 0; i < nerror; i++)
    {
      wi = w[i]*factor[i];
      eri = error[i];
      num += wi*vpMath::sqr(eri);
      den += wi ;

      //weighted_error[i] =  wi*eri ;
      weighted_error[i] = 1 ;
    }

    if((iter==0) || compute_interaction)
    {
      for (int i=0 ; i < nerror ; i++)
      {
        for (int j=0 ; j < 6 ; j++)
        {
          L[i][j] = w[i]*factor[i]*L[i][j] ;
        }
      }
    }
    vpMatrix Hsd;  // hessien a la position desiree
    vpMatrix H ;

    LTL = L.AtA();
    //std::cout << weighted_error << std::endl;
    computeJTR(L, weighted_error, LTR);
    v = -200*LTL.pseudoInverse()*LTR;
    cMo =  vpExponentialMap::direct(v).inverse() * cMo;

    iter++;
  }
std::cout << "\t First minimization in " << iter << " iteration00 " << std::endl ;


  //std::cout<<iter<<std::endl;
  // ;
  int n =0 ;
  for (int k = 0; k<points[scaleLevel].size(); k++)
  {
	  p = (points[scaleLevel])[k] ;
    {
      double wmean = 0 ;
      p->list[0];

      //for (int i=0 ; i < p->nbFeature ; i++)
      //{
        wmean += w[n] ;
        vpPointSite s = p->list[0] ;
        if (w[n] < 0.5)
        {
          s.suppress = 4 ;
          p->list[0] = s; ;
        }

        //p->list.next();
      //}
      n+= 1 ;

        wmean = 1;

      //p->setMeanWeight(wmean);

    }
    //
  }
//   cout << "\t Robust minimization in " << iter << " iteration " << endl ;
//    std::cout << "error: " << (residu_1 - r) << std::endl;
}




void
vpMbPointsTracker::computeVVSMH(const vpImage<unsigned char>& _I)
{
  double residu_1 =1e3;
  double r =1e3-1;
  vpMatrix LTL;
  vpColVector LTR;

  // compute the interaction matrix and its pseudo inverse
  vpMbtControlPoint *p ;

  vpColVector w;
  vpColVector weighted_error;
  vpColVector factor;

  int iter = 0;

  //Nombre de moving edges
  int nbrow  = 0;
  vpFeatureLine fli;

  for (int k = 0; k<points[scaleLevel].size(); k++)
  {
    p = (points[scaleLevel])[k];
    nbrow += 1;
    p->initInteractionMatrixError();

    //std::cout<<fli.getTheta()<<std::endl;
  }
  if (nbrow==0)
  {
    vpERROR_TRACE("\n\t\t Error-> not enough data in the interaction matrix...") ;
    throw vpTrackingException(vpTrackingException::notEnoughPointError, "\n\t\t Error-> not enough data in the interaction matrix...");
  }

  vpMatrix L(nbrow,6), Lp;

  // compute the error vector
  vpColVector error(nbrow);
  int nerror = error.getRows();
  vpColVector v ;
  double limite = 3; //Une limite de 3 pixels
  limite = limite / cam.get_px(); //Transformation limite pixel en limite metre.

  //Parametre pour la premiere phase d'asservissement
  double e_prev = 0, e_cur, e_next;
  bool reloop = true;
  double count = 0;
  /*** First phase ***/

  while ( reloop == true && iter<5)
  {
    if(iter==0)
    {
      weighted_error.resize(nerror) ;
      w.resize(nerror);
      w = 0;
      factor.resize(nerror);
      factor = 1;
    }

    count = 0;
    ;
    int n = 0;
    //reloop = false;
    reloop = true;

    for (int k = 0; k<points[scaleLevel].size(); k++)
    {
      p = (points[scaleLevel])[k];
      p->computeInteractionMatrixErrorMH(cMo,_I);
      //p->computeInteractionMatrixError2(cMo,_I);

      double fac = 1;
      if (iter == 0)
      {
            fac = 0.2;
            break;
      }


      //std::cout << " fac " << fac << std::endl;

      if (iter == 0 && p != NULL)

      //for (int i=0 ; i < 1 ; i++)
      //{
        for (int j=0; j < 6 ; j++)
        {
          L[n][j] = p->L[0][j]; //On remplit la matrice d'interaction globale
        }
        error[n] = p->error[0]; //On remplit la matrice d'erreur

        //std::cout << " errorminvvs " << error[n] << std::endl;
        if (error[n] <= limite) count = count+1.0; //Si erreur proche de 0 on incremente cur

        w[n] = 0;
        if (iter == 0)
        {
          factor[n] = fac;
          vpPointSite site = p->list[0];
          p->list[0];

          if (site.suppress != 0) {
        	  factor[n] = 0.2;
          }
          //p->list.next();
        }

        //If pour la premiere extremite des moving edges
        /*if (i == 0)
        {
          e_cur = l->error[0];
          if (l->nbFeature > 1)
          {
            e_next = l->error[1];
            if ( fabs(e_cur - e_next) < limite && vpMath::sign(e_cur) == vpMath::sign(e_next) )
            {
              w[n+i] = 1/*0.5*/;
          /*  }
            e_prev = e_cur;
          }
          else w[n+i] = 1;
        }*/

        //If pour la derniere extremite des moving edges
        /*else if(i == l->nbFeature-1)
        {
          e_cur = l->error[i];
          if ( fabs(e_cur - e_prev) < limite && vpMath::sign(e_cur) == vpMath::sign(e_prev) )
          {
            w[n+i] += 1/*0.5*/;
          /*}
        }*/

        //else
        //{
          e_cur = p->error[0];
          e_next = p->error[0];
          if ( fabs(e_cur - e_prev) < limite )
          {
            w[n] += 0.5;
          }
          if ( fabs(e_cur - e_next) < limite )
          {
            w[n] += 0.5;
          }
          e_prev = e_cur;
        //}
      //}

      n+= 1 ;

    }

    count = count / (double)nbrow;
    if (count < 0.85)
    {
      reloop = true;
    }

    double num=0;
    double den=0;

    double wi ; double eri ;
    for(int i = 0; i < nerror; i++)
    {
      wi = w[i]*factor[i];
      eri = error[i];
      num += wi*vpMath::sqr(eri);
      den += wi ;
      weighted_error[i] =  wi*eri ;
    }

    if((iter==0) || compute_interaction)
    {
      for (int i=0 ; i < nerror ; i++)
      {
        for (int j=0 ; j < 6 ; j++)
        {
          //L[i][j] = w[i]*factor[i]*L[i][j] ;
            L[i][j] = factor[i]*L[i][j] ;
        }
      }
    }

    LTL = L.AtA();
    computeJTR(L, weighted_error, LTR);
    v = -1*LTL.pseudoInverse()*LTR;
    cMo =  vpExponentialMap::direct(v).inverse() * cMo;

    iter++;
  }
std::cout << "\t First minimization in " << iter << " iteration00 " << std::endl ;

/*** Second phase ***/

  vpRobust robust(nerror);
  robust.setIteration(0) ;
  iter = 0;
  //vpColVector error_px(nerror);

  while (((int)((residu_1 - r)*1e8) !=0 ) && (iter<15))
  {
    int n = 0 ;
    for (int k = 0; k<points[scaleLevel].size(); k++)
    {
      p = (points[scaleLevel])[k];
      p->computeInteractionMatrixErrorMH(cMo,_I);
      //p->computeInteractionMatrixError2(cMo,_I);
      //for (int i=0 ; i < l->nbFeature ; i++)
      //{
        for (int j=0; j < 6 ; j++)
        {
          L[n][j] = p->L[0][j] ;
          error[n] = p->error[0] ;
    //error_px[n+i] = l->error[i] * cam.get_px();
        }
      //}
      n+= 1;

    }

    if(iter==0)
    {
      weighted_error.resize(nerror) ;
      w.resize(nerror);
      w = 1;

       robust.setThreshold(2/cam.get_px()); // limite en metre
       robust.MEstimator(vpRobust::TUKEY, error,w);
       //robust.setThreshold(2); // limite en pixel
       //robust.MEstimator(vpRobust::TUKEY, error_px,w);
    }
    else
    {
      robust.setIteration(iter);
      robust.MEstimator(vpRobust::TUKEY, error,w);
      //robust.MEstimator(vpRobust::TUKEY, error_px,w);
    }

    residu_1 = r;

    double num=0;
    double den=0;
    double wi;
    double eri;
    for(int i=0; i<nerror; i++)
    {
      wi = w[i]*factor[i];
      eri = error[i];
      num += wi*vpMath::sqr(eri);
      den += wi;

      weighted_error[i] =  wi*eri ;
    }

    r = sqrt(num/den); //Le critere d'arret prend en compte le poids

    if((iter==0)|| compute_interaction)
    {
      for (int i=0 ; i < nerror ; i++)
      {
        for (int j=0 ; j < 6 ; j++)
        {
          L[i][j] = w[i]*factor[i]*L[i][j];
        }
      }
    }

    LTL = L.AtA();
    computeJTR(L, weighted_error, LTR);
    v = -lambda*LTL.pseudoInverse()*LTR;
    cMo =  vpExponentialMap::direct(v).inverse() * cMo;

    iter++;
  }

//  std::cout << "iter = " << iter <<std::endl;
  int n =0 ;
  for (int k = 0; k<points[scaleLevel].size(); k++)
  {
    p = (points[scaleLevel])[k] ;
    {
      double wmean = 0 ;
      /*if (p->nbFeature > 0)*/ //p->list.front();

      //for (int i=0 ; i < p->nbFeature ; i++)
      //{
        wmean += w[n] ;
        vpPointSite s = p->list[0] ;
        if (w[n] < 0.5)
        {
          s.suppress = 4 ;
          p->list[0] = s ;
        }

      //}
      n+= 1 ;

        wmean = 1;

      //p->setMeanWeight(wmean);

    }

  }
//   cout << "\t Robust minimization in " << iter << " iteration " << endl ;
//    std::cout << "error: " << (residu_1 - r) << std::endl;
}

/*!
  Check if the tracking failed.
  
  \throw vpTrackingException::fatalError if the test fails. 
*/
/*void
vpMbPointsTracker::testTracking()
{
  int nbExpectedPoint = 0;
  int nbGoodPoint = 0;
  int nbBadPoint = 0;
  
  vpMbtControlPoint *p ;
  
   ;
  for (int k = 0; k<points[scaleLevel].size(); k++)
  {
      nbExpectedPoint += expecteddensity;
        p = (points[scaleLevel])[k];
        if (p->suppress == 0) nbGoodPoint++;
        else nbBadPoint++;
    points[scaleLevel].next();
  }    
  
  if (nbGoodPoint < percentageGdPt *(nbGoodPoint+nbBadPoint) || nbExpectedPoint < 2)
  {
    std::cout << "nbGoodPoint :" << nbGoodPoint << std::endl;
    std::cout << "nbBadPoint :" << nbBadPoint << std::endl;
    std::cout << "nbExpectedPoint :" << nbExpectedPoint << std::endl;
    throw vpTrackingException(vpTrackingException::fatalError, "test Tracking fail");
  }

  TO DO
}*/


/*!
  Compute each state of the tracking procedure for all the feature sets.
  
  If the tracking is considered as failed an exception is thrown.
  
  \param I : The image.
 */


void
vpMbPointsTracker::track(const vpImage<unsigned char> &I, const vpImage<vpRGBa> &Inormd,const vpImage<unsigned char>& Ior,const vpImage<unsigned char>& Itex, const double dist)
{
  initPyramid(I, Ipyramid);
  //initPyramid(Iprec, Ipyramidprec);

  for (int lvl = (scales.size()-1); lvl >= 0; lvl -= 1){
    if(scales[lvl]){
      vpHomogeneousMatrix cMo_1 = cMo;
      try
      {
        downScale(lvl);
        try
        {
        double t0= vpTime::measureTimeMs();
        extractControlPoints(I, Inormd, Ior,Itex, dist);
        double t1= vpTime::measureTimeMs();
        std::cout << "timeextract "<<t1-t0<<std::endl;
        }
        catch(...)
        {
        	vpTRACE("Error in points extraction");
        }
        double t2= vpTime::measureTimeMs();
        try
        {
          trackControlPoints(*Ipyramid[lvl]);
        }
        catch(...)
        {
          vpTRACE("Error in moving edge tracking") ;
          throw ;
        }
        double t3= vpTime::measureTimeMs();
        std::cout << "timetrack "<<t3-t2<<std::endl;

        // initialize the vector that contains the error and the matrix that contains
        // the interaction matrix
        /*vpMbtControlPoint *p ;
        points[lvl].front() ;
        vpFeatureLine fli;
        while (!points[lvl].outside()){
          p = points[lvl].value() ;
            //p->initInteractionMatrixError();
            //fli=p->getFeatureLine();
            //fli.display(cam,I,vpColor::yellow,1);
            //fli=p->getFeatureLine();
            //fli.print();
            //std::cout<<fli.getTheta()<<std::endl;

          points[lvl].next() ;
        }*/
        try
        {
        	double t4= vpTime::measureTimeMs();
          computeVVS(*Ipyramid[lvl]);
          double t5= vpTime::measureTimeMs();
          std::cout << "timeVVS "<<t5-t4<<std::endl;
        }
        catch(...)
        {
          vpTRACE("Error in computeVVS") ;
          throw vpException(vpException::fatalError, "Error in computeVVS");
        }
       /* try
        {
          testTracking();
        }
        catch(...)
        {
          throw vpTrackingException(vpTrackingException::fatalError, "test Tracking fail");
        }*/

        try
        {
          //displayControlPoints();
        }
        catch(...)
        {
          vpTRACE("Error in moving edge updating") ;
          throw ;
        }

      }
      catch(...)
      {
        if(lvl != 0){
          cMo = cMo_1;
          reInitLevel(lvl);
          upScale(lvl);
        }
        else{
          upScale(lvl);
          throw ;
        }
      }
    }
  }

  cleanPyramid(Ipyramid);
  //cleanPyramid(Ipyramidprec);
  //Iprec=I;
}

/*void
vpMbPointsTracker::trackHyb(const vpImage<unsigned char> &I, const vpImage<vpRGBa> &Inormd,const vpImage<unsigned char>& Ior,const vpImage<unsigned char>& Itex, const double dist, const double m, apOgre ogre_)
{
  initPyramid(I, Ipyramid);
  //initPyramid(Iprec, Ipyramidprec);

  for (int lvl = (scales.size()-1); lvl >= 0; lvl -= 1){
    if(scales[lvl]){
      vpHomogeneousMatrix cMo_1 = cMo;
      try
      {
        downScale(lvl);
        try
        {
        double t0= vpTime::measureTimeMs();
        extractControlPoints(I, Inormd, Ior,Itex, dist);
        double t1= vpTime::measureTimeMs();
        std::cout << "timeextract "<<t1-t0<<std::endl;
        }
        catch(...)
        {
        	vpTRACE("Error in points extraction");
        }
        double t2= vpTime::measureTimeMs();
        try
        {
          trackControlPoints(*Ipyramid[lvl]);
        }
        catch(...)
        {
          vpTRACE("Error in moving edge tracking") ;
          throw ;
        }
        double t3= vpTime::measureTimeMs();
        std::cout << "timetrack "<<t3-t2<<std::endl;
        try
        {
        	double t4= vpTime::measureTimeMs();
          computeVVSHyb(*Ipyramid[lvl],ogre_);
          double t5= vpTime::measureTimeMs();
          std::cout << "timeVVS "<<t5-t4<<std::endl;
        }
        catch(...)
        {
          vpTRACE("Error in computeVVS") ;
          throw vpException(vpException::fatalError, "Error in computeVVS");
        }


        try
        {
          //displayControlPoints();
        }
        catch(...)
        {
          vpTRACE("Error in moving edge updating") ;
          throw ;
        }

      }
      catch(...)
      {
        if(lvl != 0){
          cMo = cMo_1;
          reInitLevel(lvl);
          upScale(lvl);
        }
        else{
          upScale(lvl);
          throw ;
        }
      }
    }
  }

  cleanPyramid(Ipyramid);
  //cleanPyramid(Ipyramidprec);
  Iprec=I;
}*/

/*!
  Compute each state of the tracking procedure for all the feature sets, in case of the multiple hypothesis solution.

  If the tracking is considered as failed an exception is thrown.

  \param I : The image.
 */
void
vpMbPointsTracker::trackMH(const vpImage<unsigned char> &I, const vpImage<vpRGBa> &Inormd,const vpImage<unsigned char>& Ior,const vpImage<unsigned char>& Itex, const double dist)
{
  initPyramid(I, Ipyramid);
  //initPyramid(Iprec, Ipyramidprec);

  for (int lvl = (scales.size()-1); lvl >= 0; lvl -= 1){
    if(scales[lvl]){
      vpHomogeneousMatrix cMo_1 = cMo;
      try
      {
        downScale(lvl);
        try
        {
        double t0= vpTime::measureTimeMs();
        extractControlPointsMH(I, Inormd, Ior,Itex, dist);
        double t1= vpTime::measureTimeMs();
        std::cout << "timeextract "<<t1-t0<<std::endl;
        }
        catch(...)
        {
        	vpTRACE("Error in points extraction");
        }
        double t2= vpTime::measureTimeMs();
        try
        {
          trackControlPointsMH(*Ipyramid[lvl]);
        }
        catch(...)
        {
          vpTRACE("Error in moving edge tracking") ;
          throw ;
        }
        double t3= vpTime::measureTimeMs();
        std::cout << "timetrack "<<t3-t2<<std::endl;

        try
        {
        	double t4= vpTime::measureTimeMs();
          computeVVSMH(*Ipyramid[lvl]);
          double t5= vpTime::measureTimeMs();
          std::cout << "timeVVS "<<t5-t4<<std::endl;
        }
        catch(...)
        {
          vpTRACE("Error in computeVVS") ;
          throw vpException(vpException::fatalError, "Error in computeVVS");
        }

      }
      catch(...)
      {
        if(lvl != 0){
          cMo = cMo_1;
          reInitLevel(lvl);
          upScale(lvl);
        }
        else{
          upScale(lvl);
          throw ;
        }
      }
    }
  }

  cleanPyramid(Ipyramid);
  //cleanPyramid(Ipyramidprec);
  //Iprec=I;
}

/*!
  Compute each state of the tracking procedure for all the feature sets, in case of dense tracking alon the edges normals.

  If the tracking is considered as failed an exception is thrown.

  \param I : The image.
 */
void
vpMbPointsTracker::track(const vpImage<unsigned char> &I, const vpImage<double> &Igrad, const vpImage<double> &Igradx, const vpImage<double> & Igrady, const vpImage<vpRGBa> &Inormd,const vpImage<unsigned char>& Ior,const vpImage<unsigned char>& Itex, const double dist)
{
  //initPyramid(I, Ipyramid);
  //initPyramid(Iprec, Ipyramidprec);

  for (int lvl = (scales.size()-1); lvl >= 0; lvl -= 1){
    if(scales[lvl]){
      vpHomogeneousMatrix cMo_1 = cMo;
      try
      {
        downScale(lvl);
        try
        {
        double t0= vpTime::measureTimeMs();
        extractControlPoints(I, Inormd, Ior,Itex, dist);
        double t1= vpTime::measureTimeMs();
        std::cout << "timeextract "<<t1-t0<<std::endl;
        }
        catch(...)
        {
        	vpTRACE("Error in points extraction");
        }
        double t2= vpTime::measureTimeMs();
        double t3= vpTime::measureTimeMs();
        std::cout << "timetrack "<<t3-t2<<std::endl;

        try
        {
        	double t4= vpTime::measureTimeMs();
          computeVVSCorr(I,Igrad,Igradx,Igrady);
          double t5= vpTime::measureTimeMs();
          std::cout << "timeVVS "<<t5-t4<<std::endl;
        }
        catch(...)
        {
          vpTRACE("Error in computeVVS") ;
          throw vpException(vpException::fatalError, "Error in computeVVS");
        }


        try
        {
          //displayControlPoints();
        }
        catch(...)
        {
          vpTRACE("Error in moving edge updating") ;
          throw ;
        }

      }
      catch(...)
      {
        if(lvl != 0){
          cMo = cMo_1;
          reInitLevel(lvl);
          upScale(lvl);
        }
        else{
          upScale(lvl);
          throw ;
        }
      }
    }
  }

  //cleanPyramid(Ipyramid);
  //cleanPyramid(Ipyramidprec);
  //Iprec=I;
}
/*!
  Compute each state of the tracking procedure for all the feature sets for the prediction phase.

  If the tracking is considered as failed an exception is thrown.

  \param I : The image.
 */
void
vpMbPointsTracker::trackPred(const vpImage<unsigned char> &I)
{
  initPyramid(I, Ipyramid);
  //initPyramid(Iprec, Ipyramidprec);

  for (int lvl = (scales.size()-1); lvl >= 0; lvl -= 1){
    if(scales[lvl]){
      vpHomogeneousMatrix cMo_1 = cMo;
      try
      {
        downScale(lvl);
        double t2= vpTime::measureTimeMs();
        try
        {
          trackControlPointsPred(*Ipyramid[lvl]);
        }
        catch(...)
        {
          vpTRACE("Error in moving edge tracking") ;
          throw ;
        }
        double t3= vpTime::measureTimeMs();
        std::cout << "timetrack "<<t3-t2<<std::endl;

        /*vpMbtControlPoint *p ;
        points[lvl].front() ;
        vpFeatureLine fli;
        while (!points[lvl].outside()){
          p = points[lvl].value() ;
            //p->initInteractionMatrixError();
            //fli=p->getFeatureLine();
            //fli.display(cam,I,vpColor::yellow,1);
            //fli=p->getFeatureLine();
            //fli.print();
            //std::cout<<fli.getTheta()<<std::endl;

          points[lvl].next() ;
        }*/
        try
        {
          computeVVS(*Ipyramid[lvl]);
        }
        catch(...)
        {
          vpTRACE("Error in computeVVS") ;
          throw vpException(vpException::fatalError, "Error in computeVVS");
        }
       /* try
        {
          testTracking();
        }
        catch(...)
        {
          throw vpTrackingException(vpTrackingException::fatalError, "test Tracking fail");
        }*/

        try
        {
          //displayControlPoints();
        }
        catch(...)
        {
          vpTRACE("Error in moving edge updating") ;
          throw ;
        }
      }
      catch(...)
      {
        if(lvl != 0){
          cMo = cMo_1;
          reInitLevel(lvl);
          upScale(lvl);
        }
        else{
          upScale(lvl);
          throw ;
        }
      }
    }
  }

  cleanPyramid(Ipyramid);
  //cleanPyramid(Ipyramidprec);
  //Iprec=I;
}



/*!
 Initialize the tracking thanks to the initial pose of the camera.
 
 \param I : The image.
 \param _cMo : The initial pose used to initialize the tracking.
*/
void
vpMbPointsTracker::init(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &_cMo)
{
  this->cMo = _cMo;

  sId.init(I.getHeight(), I.getWidth(),  1) ;
  sI.init(I.getHeight(), I.getWidth(),  1) ;
  sId.setCameraParameters(cam);
  sI.setCameraParameters(cam);
  /*vpMbtControlPoint *p;
  vpColVector norm(3);
  norm[0]=1;
  norm[1]=1;
  norm[2]=1;
  double a=1;

  for (unsigned int i = 0; i < points_1.size(); i += 1){
    if(scales[i]){
      vpMbtControlPoint *p ;
      p=new vpMbtControlPoint;
      p->setCameraParameters(&cam);
      p->setMovingEdge(&me);
      p->buildPoint(0,0,a,0.0,norm,cMo);
      points_1[i]+=p;
    }
  }*/
 //bool a = false;
  //initPyramid(I, Ipyramid);
  Iprec=I;
}
  
  /*cleanPyramid(Ipyramid);
}*/

/*!
  Load the xml configuration file.
  Write the parameters in the corresponding objects (Ecm, camera, 3D rendering).

  \param _filename : full name of the xml file.
*/
void 
vpMbPointsTracker::loadConfigFile(const std::string& _filename)
{
  loadConfigFile(_filename.c_str());
}

/*!
  Load the xml configuration file.
  Write the parameters in the corresponding objects (Ecm, camera, 3D rendering).
  
  \throw vpException::ioError if the file has not been properly parsed (file not
  found or wrong format for the data). 

  \param filename : full name of the xml file.
*/
void
vpMbPointsTracker::loadConfigFile(const char* filename)
{
    LuaConfig cfg(filename);
    vpCameraParameters camera(cfg.getAsNumber("conf.camera.px"),
                              cfg.getAsNumber("conf.camera.py"),
                              cfg.getAsNumber("conf.camera.u0"),
                              cfg.getAsNumber("conf.camera.v0"));
    vpMe meParser;
    meParser.sample_step = cfg.getAsNumber("conf.sample.step");
    meParser.ntotal_sample = cfg.getAsNumber("conf.sample.nb_sample");
    meParser.setMaskSize(cfg.getAsNumber("conf.ecm.mask.size"));
    meParser.setMaskNumber(cfg.getAsNumber("conf.ecm.mask.nb_mask"));
    meParser.range = cfg.getAsNumber("conf.ecm.range.tracking");
    meParser.mu1 = cfg.getAsNumber("conf.ecm.contrast.mu1");
    meParser.mu2 = cfg.getAsNumber("conf.ecm.contrast.mu2");
    meParser.threshold = cfg.getAsNumber("conf.ecm.contrast.edge_threshold");

    apRend rend;
    rend.edgeR_th = cfg.getAsNumber("conf.rendering.edgeRend_threshold");
    rend.clipDist = cfg.getAsNumber("conf.rendering.clipDist");
    rend.sampleR = cfg.getAsNumber("conf.rendering.sampleRend");
    rend.scaleModel = cfg.getAsNumber("conf.rendering.scaleModel");
    rend.Normx = cfg.getAsNumber("conf.rendering.xDir");
    rend.Normy = cfg.getAsNumber("conf.rendering.yDir");
    rend.Normz = cfg.getAsNumber("conf.rendering.zDir");

    setCameraParameters(camera);
    setMovingEdge(meParser);
    setRendParameters(rend);
}


/*!
  Display the 3D model from a given position of the camera.

  \param I : The image.
  \param _cMo : Pose used to project the 3D model into the image.
  \param cam : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the points.
*/
void
vpMbPointsTracker::display(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &_cMo, const vpCameraParameters &cam,
											const vpColor& col,
											const unsigned int thickness)
{
  vpMbtControlPoint *p ;
  for (unsigned int i = 0; i < scales.size(); i += 1){
    if(scales[i]){
    	for(int k = 0; k<points[i].size(); k++)
      {
        p = (points[i])[k];
        //p->display(I, vpColor::blue, thickness) ;
        p->update(_cMo);
        p->display(I, col, thickness) ;
      }
      break ; //displaying model on one clase only
    }
  }
}

/*!
  Display the 3D model from a given position of the camera.

  \param I : The image.
  \param _cMo : Pose used to project the 3D model into the image.
  \param cam : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the points.
*/
void
vpMbPointsTracker::display(const vpImage<vpRGBa>& I, const vpHomogeneousMatrix &_cMo, const vpCameraParameters &cam,
											const vpColor& col,
											const unsigned int thickness)
{
	  vpMbtControlPoint *p ;

	  for (unsigned int i = 0; i < scales.size(); i += 1){
	    if(scales[i]){
	    	for(int k = 0; k<points[i].size(); k++)
	      {
	        p = (points[i])[k];
	        p->display(I, col, thickness) ;
	      }
	      break ; //displaying model on one clase only
	    }
	  }
}


/*!
  Initialize the control points thanks to a given pose of the camera.
  
  \param I : The image.
  \param _cMo : The pose of the camera used to initialize the control points.
*/
/*void
vpMbPointsTracker::initControlPoints(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &_cMo)
{
  vpMbtControlPoint *p ;
  
   ;
  for (int k = 0; k<points[scaleLevel].size(); k++)
  {
    p = (points[scaleLevel])[k] ;
    p->initControlPoint(I,_cMo) ;
      }

  }   
}*/


/*!
  Track the control points in the image.
  
  \param I : the image.
*/
void
vpMbPointsTracker::trackControlPoints(const vpImage<unsigned char> &I)
{
  vpMbtControlPoint *p;
  for (int k = 0; k<points[scaleLevel].size(); k++){
    p = (points[scaleLevel])[k];
      p->track(I,Iprec) ;

  }    
}

/*!
  Track the control points in the image, in case of a multiple hypothesis solution.

  \param I : the image.
*/
void
vpMbPointsTracker::trackControlPointsMH(const vpImage<unsigned char> &I)
{
  vpMbtControlPoint *p;
  for (int k = 0; k<points[scaleLevel].size(); k++){
    p = (points[scaleLevel])[k];
      //if(p->meline == NULL){
      //p->initControlPoint(I);
      //}
      p->trackMH(I,Iprec);

  }
}

/*!
  Track the control points in the image, in case of a multiple hypothesis solution, for a prediction phase.

  \param I : the image.
*/
void
vpMbPointsTracker::trackControlPointsPred(const vpImage<unsigned char> &I)
{
  vpMbtControlPoint *p;
  for (int k = 0; k<points[scaleLevel].size(); k++){
    p = (points[scaleLevel])[k];
      //if(p->meline == NULL){
      //p->initControlPoint(I);
      //}
      p->trackPred(I,Iprec) ;

  }
}


/*!
  Update the control points at the end of the virtual visual servoing.
  
  \param I : the image.
*/
void
vpMbPointsTracker::updateControlPoints()
{

//points_1=points;
/*vpMbtControlPoint *p;
  vpMbtControlPoint *p1;
  vpPoint P;
  vpPoint P1;
  //vpColVector profile(3);
  ;
  for (int k = 0; k<points[scaleLevel].size(); k++){
    p = (points[scaleLevel])[k];
    points1[scaleLevel].front();
    P=p->cpoint;
  while (!points1[scaleLevel].outside()){
	p1 = points1[scaleLevel].value();
    P1=p->cpoint;
    if (!samePoint(P1,P2))
    {
    profile=p1->getProfile();
    }
    p->setProfile(p);

  }
    //if (l->nbFeature == 0 && l->isVisible()) l->Reinit = true;
    points[scaleLevel].next();
  }  */
	//delete *points;
	  vpMbtControlPoint *p;
	  for(unsigned int i=0; i<scales.size(); i++){
		  if(scales[i]){
		    	for(int k = 0; k<points[i].size(); k++)
		        {
		          p = (points[i])[k];
		          if (p!=NULL) delete p ;
		          p = NULL ;
		        }
		  }
		}

}



/*!
  Check if two vpPoints are similar.
  
  To be similar : \f$ (X_1 - X_2)^2 + (Y_1 - Y_2)^2 + (Z_1 - Z_2)^2 < threshold \f$.
  
  \param P1 : The first point to compare
  \param P2 : The second point to compare
  \param threshold : The threshold  used to decide if the points are similar or not.
*/
bool samePoint(const vpPoint &P1, const vpPoint &P2, double threshold=1e-4)
{
  double d = vpMath::sqr(P1.get_oX() - P2.get_oX())+
  vpMath::sqr(P1.get_oY() - P2.get_oY())+
  vpMath::sqr(P1.get_oZ() - P2.get_oZ()) ;
  if (d  < threshold)
    return true ;
  else
    return false ;
}


/*!
  Extract 3D control points from the depth edges, the texture or color edges, given the depth buffer and the normal map
  
  \param I : input image.
  \param Inormd : Normal map (RGB channels) and depth buffer (A channel)
  \param Ior : oriented edge map from depth edge.
  \param Itex : oriented edge map from texture or color edges.
  \param dist : Z coordinate of the center of the object, in order to define and update the near and far clip distances
*/

void
vpMbPointsTracker::extractControlPoints(const vpImage<unsigned char> &I, const vpImage<vpRGBa>& Inormd, const vpImage<unsigned char>& Ior, const vpImage<unsigned char>& Itex, const double dist)
{
  vpMbtControlPoint *p ;
  vpImagePoint ip;
  double px = cam.get_px();
  double py = cam.get_py();
  int jc = cam.get_u0() ;
  int ic = cam.get_v0() ;
  int i1,j1;
  vpPoint Pc;
  vpColVector Pc0(3);
  vpColVector ip0(2);
  double x,y,Z,rho;
  vpColVector Normc(3);
  double theta=0;
  int sample = rendparam.sampleR;
  double znear=dist-rendparam.clipDist;
  double zfar=dist+rendparam.clipDist;
  //std::cout<<" clipdist " << rendparam.clipDist<<std::endl;
  int bord = 20;

  for (unsigned int i = 0; i < scales.size(); i += 1){
    if(scales[i]){
      downScale(i);
  	  for(int k = 0; k<points[i].size(); k++)
      {
        p = (points[i])[k] ;
        if (p!=NULL) delete p ;
        p = NULL ;
      }
  	  points[i].resize(0);

      int rows = Ior.getHeight();
      int cols = Ior.getWidth();
      vpRotationMatrix R;
      cMo.extract(R);
      vpColVector norm(3);

      int w;
        for (int n=bord;n<rows-bord;n++)
        {
        	for (int m=bord;m<cols-bord;m++)
        	{
        	if(((double)n/sample-(double)floor(n/sample)<0.0005 || (double)m/sample-(double)floor(m/sample)<0.0005) && (Ior[n][m]!=100 || Itex[n][m]!=100) && ((double)Inormd[n][m].R>0||(double)Inormd[n][m].G>0||(double)Inormd[n][m].B>0))
        	{
        		if(Itex[n][m]!=100){
        	theta=3.1416*(double)((double)Itex[n][m]/255-0.5);}
        		else
        		{
        	theta=3.1416*(double)((double)Ior[n][m]/255-0.5);
        		}
        	Z = -(znear*zfar)/(((double)((double)Inormd[n][m].A)/255)*(zfar-znear)-zfar);

               //tank
            norm[0]=(rendparam.Normx)*((double)((double)Inormd[n][m].R)/255-0.5);
            norm[1]=(rendparam.Normy)*((double)((double)Inormd[n][m].G)/255-0.5);
            norm[2]=(rendparam.Normz)*((double)((double)Inormd[n][m].B)/255-0.5);

            //std::cout<<" norm " << norm[0] << " " << norm[1] << " "<< norm[2] << " Z " << Z << " n " << znear << " m " << zfar << std::endl;
            //getchar();
            p = new vpMbtControlPoint;
            p->setCameraParameters(&cam);
            p->setMovingEdge(&me);
            p->buildPoint(n,m,Z,theta,norm,cMo);
            p->initControlPoint(I,0);
            npoints +=1;
            points[i].push_back(p);
            //points[i] += p;
            //std::cout<<fline.getRho()<<std::endl;
            /*ip.set_i(n);
            ip.set_j(m);
  	        x=(m-jc)/px;
  	        y=(n-ic)/py;
            rho= x*cos(theta)+y*sin(theta);
            Normc=R*norm;
            Pc0[0]=x*Z+Normc[0]/0.6;
            Pc0[1]=y*Z+Normc[1]/0.6;
            Pc0[2]=Z+Normc[2]/0.6;
            Pc.projection(Pc0,ip0);
            j1=ip0[0]*px+jc;
            i1=ip0[1]*py+ic;
            //fline.buildFrom(rho,theta);
            //fline.display(cam,I,vpColor::yellow,1);
          	vpDisplay::displayArrow(I,n,m,i1,j1,vpColor::blue,4,2,1);*/
            //vpDisplay::displayCross(I,ip,2,vpColor::yellow,2);
            w++;
            }
            else {w++;}
//}

}
}
      }
      upScale(i);
    }

}

/*!
  Extract 3D control points from the depth edges, the texture or color edges, given the depth buffer and the normal map
  Case of the multiple hypothesis solution
  \param I : input image.
  \param Inormd : Normal map (RGB channels) and depth buffer (A channel)
  \param Ior : oriented edge map from depth edge.
  \param Itex : oriented edge map from texture or color edges.
  \param dist : Z coordinate of the center of the object, in order to define and update the near and far clip distances
*/

void
vpMbPointsTracker::extractControlPointsMH(const vpImage<unsigned char> &I, const vpImage<vpRGBa>& Inormd, const vpImage<unsigned char>& Ior, const vpImage<unsigned char>& Itex, const double dist)
{
  vpMbtControlPoint *p ;
  vpImagePoint ip;
  double px = cam.get_px();
  double py = cam.get_py();
  int jc = cam.get_u0() ;
  int ic = cam.get_v0() ;
  int i1,j1;
  vpPoint Pc;
  vpColVector Pc0(3);
  vpColVector ip0(2);
  double x,y,Z,rho;
  double theta=0;
  vpColVector Normc(3);
  int sample = rendparam.sampleR;
  double znear=dist-rendparam.clipDist;
  double zfar=dist+rendparam.clipDist;


  for (unsigned int i = 0; i < scales.size(); i += 1){
    if(scales[i]){
      downScale(i);

  	  for(int k = 0; k<points[i].size(); k++)
      {
        p = (points[i])[k] ;
        if (p!=NULL) delete p ;
        p = NULL ;
      }

      int rows = Ior.getHeight();
      int cols = Ior.getWidth();
      double n_sample;
      vpRotationMatrix R;
      cMo.extract(R);
      vpColVector norm(3);

      int w;
        for (int n=20;n<rows-20;n++)
        {
        	for (int m=20;m<cols-20;m++)
        	{
        /*if(!Ior[n][m]==100)
        {*/
        	//std::cout<<(double)Ior[n][m]<<std::endl;
        	if(((double)n/sample-(double)floor(n/sample)<0.0005 || (double)m/sample-(double)floor(m/sample)<0.0005) && ((Ior[n][m]!=100) || (Itex[n][m]!=100)))
        		{
        		if(Itex[n][m]!=100){
        	 theta=3.1416*(double)((double)Itex[n][m]/255-0.5);}
        		else
        		{
        	theta=3.1416*(double)((double)Ior[n][m]/255-0.5);
        		}
            Z = -(znear*zfar)/(((double)((double)Inormd[n][m].A)/255)*(zfar-znear)-zfar);

            norm[0]=(rendparam.Normx)*((double)((double)Inormd[n][m].R)/255-0.5);
            norm[1]=(rendparam.Normy)*((double)((double)Inormd[n][m].G)/255-0.5);
            norm[2]=(rendparam.Normz)*((double)((double)Inormd[n][m].B)/255-0.5);
            //std::cout<<norm[0]<<" "<<norm[1]<<" "<<norm[2]<<std::endl;
            if(((double)Inormd[n][m].R>0||(double)Inormd[n][m].G>0||(double)Inormd[n][m].B>0))
            {
            p = new vpMbtControlPoint;
            p->setCameraParameters(&cam);
            p->setMovingEdge(&me);
            p->buildPoint(n,m,Z,theta,norm,cMo);
            p->initControlPoint(I, 0);
            npoints +=1;
            points[i].push_back(p);
            //points[i] += p;
            //std::cout<<fline.getRho()<<std::endl;
            ip.set_i(n);
            ip.set_j(m);
  	        /*x=(m-jc)/px;
  	        y=(n-ic)/py;
            rho= x*cos(theta)+y*sin(theta);
            Normc=R*norm;
            Pc0[0]=x*Z+Normc[0]/1;
            Pc0[1]=y*Z+Normc[1]/1;
            Pc0[2]=Z+Normc[2]/1;
            Pc.projection(Pc0,ip0);
            j1=ip0[0]*px+jc;
            i1=ip0[1]*py+ic;
            //fline.buildFrom(rho,theta);
            //fline.display(cam,I,vpColor::yellow,1);
          	vpDisplay::displayArrow(I,n,m,i1,j1,vpColor::white,4,2,1);*/
            //vpDisplay::displayCross(I,n,m,3,vpColor::green);
            }
            w++;
            }
            else {w++;}
//}

}
}

      }
      upScale(i);
    }

}

/*!
  Load a 3D model contained in a file.
  
  \param file : Path to the file containing the 3D model description.
*/
void
vpMbPointsTracker::loadModel(const char* file)
{
  std::string model(file);
  vpMbTracker::loadModel(model);
}


/*void
vpMbPointsTracker::loadLines(const vpImage<unsigned char>& I, vector<Vec4i>& Lines, const vpMatrix &Zc, const vpHomogeneousMatrix &cMo)
{
vpPoint P1,P2,p1;
vpImagePoint ip1,ip2;
vpColVector P10,P20;
double x1,y1,x2,y2,z1,z2;
vpMbtDistanceLine *l;
for( size_t i = 0; i < Lines.size(); i++)
                            {
	ip1.set_i(Lines[i][1]-8);
	ip1.set_j(Lines[i][0]+18);
	ip2.set_i(Lines[i][3]-8);
	ip2.set_j(Lines[i][2]+18);
	z1=Zc[Lines[i][1]][Lines[i][0]]/2;
	z2=Zc[Lines[i][3]][Lines[i][2]]/2;
vpPixelMeterConversion::convertPoint(cam,ip1,x1,y1);
vpPixelMeterConversion::convertPoint(cam,ip2,x2,y2);
P1.setWorldCoordinates(x1*z1,y1*z1,z1);
P2.setWorldCoordinates(x2*z2,y2*z2,z2);
P1.changeFrame(cMo.inverse(),P10);
P2.changeFrame(cMo.inverse(),P20);
P1.setWorldCoordinates(P10[0],P10[1],P10[2]);
P2.setWorldCoordinates(P20[0],P20[1],P20[2]);
std::cout<<" ok "<<z1<<std::endl;
vpDisplay::displayPoint(I,ip1,vpColor::red);
addLine(P1,P2,-1);
                            }
lines[0].front();
l=lines[0].value();
l->p1;
//std::cout<<" ok "<<z1<<std::endl;
//std::cout<< "ok " << lines[0].size() << std::endl;
			//getchar();
}

/*!
  Reset the tracker. The model is removed and the pose is set to identity.
  The tracker needs to be initialized with a new model and a new pose. 
  
*/
void
vpMbPointsTracker::resetTracker()
{
  this->cMo.setIdentity();
  vpMbtControlPoint *p ;
  
  for (unsigned int i = 0; i < scales.size(); i += 1){
    if(scales[i]){
    for(int k = 0; k<points[i].size(); k++)
      {
        p = (points[i])[k] ;
        if (p!=NULL) delete p ;
        p = NULL ;
      }
    }
  }
  
  //faces.reset();
  
  //index_polygon =0;
  compute_interaction=1;
  npoints = 0;
  lambda = 1;
  //nbvisiblepolygone = 0;
  percentageGdPt = 0.4;
  
  // reinitialisation of the scales.
  this->setScales(scales);
}



/*!
  Re-initialise the model used by the tracker.  
  
  \param _I : The image containing the object to initialize.
  \param _cad_name : Path to the file containing the 3D model description.
  \param _cMo : The new vpHomogeneousMatrix between the camera and the new model
*/
/*void
vpMbPointsTracker::reInitModel(const vpImage<unsigned char>& _I, const char* _cad_name, const vpHomogeneousMatrix& _cMo)
{
  resetTracker();
  loadModel(_cad_name);
  init(_I, _cMo);
}

void
vpMbPointsTracker::reInitConfigModel(const vpImage<unsigned char>& I, const char* cad_name,const char* config_name, const vpHomogeneousMatrix& _cMo)
{
  resetTracker();
  loadConfigFile(cad_name);
  loadModel(config_name);
  init(I, _cMo);*/
//}

/*!
  Return the number of good points (vpPointSite) tracked. A good point is a
  vpPointSite with its flag "suppress" equal to 0. Only these points are used
  during the virtual visual servoing stage. 
  
  \exception vpException::dimensionError if _level does not represent a used 
  level.
  
  \return the number of good points. 
*/
unsigned int 
vpMbPointsTracker::getNbPoints(const unsigned int _level)
{
  if((_level > scales.size()) || !scales[_level]){
    throw vpException(vpException::dimensionError, "Level is not used");
  }
  
  unsigned int nbGoodPoints = 0;
  vpMbtControlPoint *p ;
  for (int k; k < points[_level].size(); k++)
  {
    p = (points[_level])[k] ;
    if (p!= NULL)
    {
      for (int kp = 0 ; kp < list.size(); kp ++)
      {
	      if ((p->list[kp]).suppress == 0) nbGoodPoints++;
      }
    }
  }
  return nbGoodPoints;
}


/*!
  Set the scales to use to realise the tracking. The vector of boolean activates
  or not the scales to se for the object tracking. The first element of the list
  correspond to the tracking on the full image, the second element corresponds 
  to the tracking on an image sbsampled by two. 
  
  Using multi scale tracking allows to track the object with greater moves. It 
  requires the computation of a pyramid of images, but the total tracking can be
  faster than a tracking based only on the full scale. The pose is computed from
  the smallest image to the biggest. This may be dangerous if the object to 
  track is small in the image, because the subsampled scale(s) will have only 
  few points to compute the pose (it could result in a loss of precision). 
  
  \warning This method must be used before the tracker has been initialised (
  before the call of the init() or the initClick() method). 
  
  \warning At least one level must be activated. 
  
  \param _scales : The vector describing the levels to use. 
*/
void 
vpMbPointsTracker::setScales(const std::vector<bool>& _scales)
{
  unsigned int nbActivatedLevels = 0;
  for (unsigned int i = 0; i < _scales.size(); i += 1){
    if(_scales[i]){
      nbActivatedLevels++;
    }
  }
  if((_scales.size() < 1) || (nbActivatedLevels == 0)){
    vpERROR_TRACE(" !! WARNING : must use at least one level for the tracking. Use the global one");
    scales.resize(0);
    scales.push_back(true);
    points.resize(1);
    points[0].resize(0);
  }
  else{
    scales = _scales;
    points.resize(_scales.size());
    for (unsigned int i = 0; i < points.size(); i += 1){
      points[i].resize(0);
    }
  }
}

/*!
  Compute the pyramid of image associated to the image in parameter. The scales 
  computed are the ones corresponding to the scales  attribte of the class. If 
  OpenCV is detected, the functions used to computed a smoothed pyramid come 
  from OpenCV, otherwise a simple subsampling (no smoothing, no interpolation) 
  is realised. 
  
  \warning The pyramid contains pointers to vpImage. To properly deallocate the
  pyramid. All the element but the first (which is a pointer to the input image)
  must be freed. A proper cleaning is implemented in the cleanPyramid() method. 
  
  \param _I : The input image.
  \param _pyramid : The pyramid of image to build from the input image.
*/
void 
vpMbPointsTracker::initPyramid(const vpImage<unsigned char>& _I, std::vector< const vpImage<unsigned char>* >& _pyramid)
{
  _pyramid.resize(scales.size());
  
  if(scales[0]){
    _pyramid[0] = &_I;
  }
  else{
    _pyramid[0] = NULL;
  }
  
  for(unsigned int i=1; i<_pyramid.size(); i += 1){
    if(scales[i]){
      unsigned int cScale = static_cast<unsigned int>(pow(2., (int)i));
      vpImage<unsigned char>* I = new vpImage<unsigned char>(_I.getHeight() / cScale, _I.getWidth() / cScale);
#ifdef VISP_HAVE_OPENCV
      IplImage* vpI0 = cvCreateImageHeader(cvSize(_I.getWidth(), _I.getHeight()), IPL_DEPTH_8U, 1);
      vpI0->imageData = (char*)(_I.bitmap);
      IplImage* vpI = cvCreateImage(cvSize(_I.getWidth() / cScale, _I.getHeight() / cScale), IPL_DEPTH_8U, 1);
      cvResize(vpI0, vpI, CV_INTER_NN);
      vpImageConvert::convert(vpI, *I);
      cvReleaseImage(&vpI);  
      vpI0->imageData = NULL;
      cvReleaseImageHeader(&vpI0);    
#else
      for (unsigned int k = 0, ii = 0; k < I->getHeight(); k += 1, ii += cScale){
        for (unsigned int l = 0, jj = 0; l < I->getWidth(); l += 1, jj += cScale){
          (*I)[k][l] = _I[ii][jj];
        }
      }
#endif   
      _pyramid[i] = I;
    }
    else{
      _pyramid[i] = NULL;
    }
  }
}

/*!
  Clean the pyramid of image allocated with the initPyramid() method. The vector
  has a size equal to zero at the end of the method. 
  
  \param _pyramid : The pyramid of image to clean.
*/
void 
vpMbPointsTracker::cleanPyramid(std::vector< const vpImage<unsigned char>* >& _pyramid)
{
  if(_pyramid.size() > 0){
    _pyramid[0] = NULL;
    for (unsigned int i = 1; i < _pyramid.size(); i += 1){
      if(_pyramid[i] != NULL){
        delete _pyramid[i];
        _pyramid[i] = NULL;
      }
    }
    _pyramid.resize(0);
  }
}


/*!
  Get the list of the control points tracked for the specified level.
  
  \throw vpException::dimensionError if the parameter does not correspond to an 
  used level. 
  
  \param _level : Level corresponding to the list to return. 
  \return Pointer to the list of the lines tracked. 
*/
std::vector<vpMbtControlPoint *>*
vpMbPointsTracker::getPpoint(const unsigned int _level)
{
  if(_level > scales.size() || !scales[_level]){
    std::ostringstream oss;
    oss << _level;
    std::string errorMsg = "level " + oss.str() + " is not used, cannot get its distance lines.";    
    throw vpException(vpException::dimensionError, errorMsg);
  }
  
  return &points[_level];
}

/*!
  Modify the camera parameters to have them corresponding to the current scale.
  The new parameters are divided by \f$ 2^{\_scale} \f$. 
  
  \param _scale : Scale to use. 
*/
void 
vpMbPointsTracker::downScale(const unsigned int _scale)
{
  const double ratio = pow(2., (int)_scale);
  scaleLevel = _scale;
  
  vpMatrix K = cam.get_K();
  
  K[0][0] /= ratio;
  K[1][1] /= ratio;
  K[0][2] /= ratio;
  K[1][2] /= ratio;

  cam.initFromCalibrationMatrix(K);
}

/*!
  Modify the camera parameters to have them corresponding to the current scale.
  The new parameters are multiplied by \f$ 2^{\_scale} \f$. 
  
  \param _scale : Scale to use. 
*/
void 
vpMbPointsTracker::upScale(const unsigned int _scale)
{
  const double ratio = pow(2., (int)_scale);
  scaleLevel = 0;
  
  vpMatrix K = cam.get_K();
  
  K[0][0] *= ratio;
  K[1][1] *= ratio;
  K[0][2] *= ratio;
  K[1][2] *= ratio;

  cam.initFromCalibrationMatrix(K);
}

/*!
  Re initialise the moving edges associated to a given level. This method is 
  used to re-initialise the level if the tracking failed on this level but 
  succedded on the other one. 
  
  \param _lvl : The level to re-initialise.
*/
void 
vpMbPointsTracker::reInitLevel(const unsigned int _lvl)
{
  unsigned int scaleLevel_1 = scaleLevel;
  scaleLevel = _lvl;

  vpMbtControlPoint *p;
  for (int k = 0; k<points[scaleLevel].size(); k++){
    p = (points[scaleLevel])[k] ;
    p->initControlPoint(*Ipyramid[_lvl],0);

  } 
  
  trackControlPoints(*Ipyramid[_lvl]);
  //updateMovingEdge(*Ipyramid[_lvl]);
  scaleLevel = scaleLevel_1;
}

