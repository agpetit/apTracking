/*!
  \file vpMbPointsTracker.cpp
  \brief Make the complete tracking of an object by using its CAD model.
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

#include <visp/vpException.h>
#include <visp/vpTrackingException.h>

#include "apMbHybridTracker.h"
//#include "vpMbtControlPoint.h"
#include <visp/vpMbtXmlParser.h>


#include <visp/vpImagePoint.h>

#include <string>
#include <sstream>


/*!
  Basic constructor
*/

apMbHybridTracker::apMbHybridTracker(int width, int heigth)
{
  //index_polygon =0;
 Inormd.resize(heigth, width);
 Ior.resize(heigth, width);
 iinf=0;
 isup=0;
 jinf=0;
 jsup=0;
}

/*!
  Basic destructor useful to deallocate the memory.
*/
apMbHybridTracker::~apMbHybridTracker()
{

}
/*vpMbPointsTracker::~vpMbPointsTracker()
{
  vpMbtControlPoint *p ;
  
  for (unsigned int i = 0; i < points.size(); i += 1){
    if(scales[i]){
      points[i].front() ;
      while (!points[i].outside()){
        p = points[i].value() ;
        if (p!=NULL) delete p ;
        p = NULL ;
        points[i].next() ;
      }
      points[i].kill() ;
    }
  }
  points.resize(0);

  cleanPyramid(Ipyramid);
}*/

/*! 
  Set the moving edge parameters.
  
  \param _me : an instance of vpMe containing all the desired parameters
*/

/*void
vpMbPointsTracker::setOgre(const exampleVpAROgre &ogre_)
{
ogre = ogre_;
}*/

void
apMbHybridTracker::cast()
{
	iinf=480;
	isup=0;
	jinf=640;
	jsup=0;
	int i,j;
	int w=Inormd.getWidth();
	int h = Inormd.getHeight();
	for (i=0 ; i < h ; i++){
	for (j=0 ; j < w ; j++){
	double val =0.2*Inormd[i][j].R + 0.3*Inormd[i][j].G + 0.5*Inormd[i][j].B ;
	if (val>0)
	{
		if (i<iinf)
		{
			if (i>19)
		{iinf=i-20;}
			else {iinf=20;}
		}
		if (i>isup)
		{
			if (i<461)
		{isup=i+20;}
			else {isup=460;}
		}
		if (j<jinf)
		{
			//if (j>109)
			if (j>19)
		{jinf=j-20;}
			//else {bornj1=111;}
			else {jinf=20;}
		}
		if (j>jsup)
		{
			if (j<621)
		{jsup=j+20;}
			else {jsup=620;}
		}
	}
	}
	}
}
void
apMbHybridTracker::updateRT(apOgre &_ogre)
{
vpTranslationVector tr;
cMo.extract(tr);
//_ogre.updateClipDistances0(tr[2],6);
//_ogre.updateClipDistances0(tr[2],1);

//_ogre.updateClipDistances0(tr[2],0.06);
if(tr[2]>0.30){
_ogre.updateClipDistances0(tr[2],0.30);
}
else{
_ogre.updateClipDistances0(tr[2],0.10);
}
//_ogre.updateClipDistances0(100,95);
_ogre.updateRTT(Inormd,Ior,&cMo);
//vpImageIo::writePPM(Inormd, "text1.pgm");
/*int ii,is,ji,js;
ii=0;
is=0;
ji=0;
js=0;*/
/*_ogre.cast(Inormd, ii,is,ji,js);
iinf=ii;
isup=is;
jinf=ji;
jsup=js;*/
cast();
std::cout << " inf " << iinf << " isup " << isup << std::endl;
//getchar();
}

void
apMbHybridTracker::computeVVSHyb(const vpImage<unsigned char>& _I, apOgre &_ogre)
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

  points[scaleLevel].front();
  while (!points[scaleLevel].outside())
  {
    p = points[scaleLevel].value();
    nbrow += 1;
    p->initInteractionMatrixError();
    points[scaleLevel].next() ;
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
  //for (int i=3+0; i < nbr-3 -120 ; i++)
{
  //   cout << i << endl ;
  for (int j = 3 ; j < nbc-3; j++)
    {
      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
      //imG[i][j] = apImageFilter::gaussianFilter(_I,i,j) ;
	  imG[i][j] = _I[i][j];

    }
}

double x = tr[0]/tr[2];
double y = tr[1]/tr[2];

double xc = cam.get_u0();
double yc = cam.get_v0();
double mx = 1.0/cam.get_px();
double my = 1.0/cam.get_py();

x = (x/mx+xc);
y = (y/my+yc);
int ii = (int)y;
int jj = (int)x;
/*int iinf;
int isup;
int jinf;
int jsup;
if (ii<120) iinf = 3;
else iinf = ii + 3 -120;

if (ii>360) isup = nbr-3;
else isup = ii - 3 +120;

if (jj<120) jinf = 3;
else jinf = jj + 3 -120;

if (jj>520) jsup = nbc-3;
else jsup = jj - 3 +120;*/


for (int i=iinf; i < isup ; i++)
{
  for (int j = jinf ; j < jsup; j++)
    {
/*  for (int i=3; i < nbr-3 ; i++)
{
  for (int j = 3 ; j < nbc-3; j++)
    {*/
  /*for (int i=200; i < 350 ; i++)
 {
   for (int j = 150 ; j < 550; j++)
     {*/
      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
      double Ix =   0.5 * apImageFilter::sobelFilterX(_I,i,j);
      double Iy =   0.5 * apImageFilter::sobelFilterY(_I,i,j);
      if (sqrt(vpMath::sqr(Ix)+vpMath::sqr(Iy))>2)
      Igd[i][j]= (unsigned char)sqrt(vpMath::sqr(Ix)+vpMath::sqr(Iy));
      if (sqrt(vpMath::sqr(Ix)+vpMath::sqr(Iy))>255)
    	  Igd[i][j]=255;
      //I2[i][j] = (unsigned char)Igrad[i][j];
    }
}

  sId.init(Igd.getHeight(), Igd.getWidth(), tr[2]);
  sI.init(Igd.getHeight(), Igd.getWidth(), tr[2]);
  sId.buildFrom(Igd);
  sId.interaction(Lsd);
  //Lsd=0*Lsd;
  Lsd = 1.5*0.1*0.00003255*Lsd;
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

  //H = ((mu * diagHsd) + Hsd).pseudoInverse();

  /*** First phase ***/

  while ( reloop == true && iter<30)
  {

cMo.extract(tr);
//_ogre.updateClipDistances0(tr[2],6);
//_ogre.updateClipDistances0(tr[2],1);
//_ogre.updateClipDistances0(tr[2],0.06);
if(tr[2]>0.30){
_ogre.updateClipDistances0(tr[2],0.30);
}
else{
_ogre.updateClipDistances0(tr[2],0.10);
}
//_ogre.updateClipDistances0(100,95);
_ogre.updateRTTGrad(Ig,&cMo);
//sI.update(Ig.getHeight(), Ig.getWidth(),tr[2]);
sI.buildFrom(Ig);
sI.error(sId, errorG);
vpImageIo::writePPM(Ig,"Igradient.pgm");
vpImageIo::writePPM(Igd,"Idgradient.pgm");
vpImageTools::imageDifference(Ig,Igd,Idiff);
vpDisplay::display(Idiff);
vpDisplay::flush(Idiff);


/*H = ((mu * diagHsd) + Hsd).pseudoInverse();
//	compute the control law
e = H * Lsd.t() *errorG;
v =  -0.5*e;
cMo =  vpExponentialMap::direct(v).inverse() * cMo;*/



	  if(iter==0)
    {
      weighted_error.resize(nerror+nerrorG) ;
      w.resize(nerror+nerrorG);
      w = 0;
      factor.resize(nerror+nerrorG);
      factor = 1;
    }

	/*  if(iter==0)
    {
      weighted_error.resize(nerror) ;
      w.resize(nerror);
      w = 0;
      factor.resize(nerror);
      factor = 1;
    }*/

    count = 0;
    points[scaleLevel].front();
    int n = 0;
    //reloop = false;
    reloop = true;

    while (!points[scaleLevel].outside())
    {
      p = points[scaleLevel].value();
      p->computeInteractionMatrixError(cMo,_I);
      //p->computeInteractionMatrixError2(cMo,_I);

      double fac = 1;
      /*if (iter == 0)
      {
            fac = 0.2;
            break;
      }*/

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
      points[scaleLevel].next() ;
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

      //weighted_error[i] =25000000*wi*eri;
      weighted_error[i] =  wi*eri;
      //weighted_error[i] =  0*wi*eri;
    }
    for(int i = nerror; i < nerror+nerrorG; i++)
    {
      //weighted_error[i] =  1*errorG[i-nerror];
      weighted_error[i] =  1.5*0.1*0.00003255*errorG[i-nerror];
    }

    std::cout<< errorG.getRows() << " ok " << error.getRows() << std::endl;


    if((iter==0) || compute_interaction)
    {
      for (int i=0 ; i < nerror ; i++)
      {
        for (int j=0 ; j < 6 ; j++)
        {
          //L[i][j] = 25000000*w[i]*factor[i]*L[i][j] ;
          L[i][j] = w[i]*factor[i]*L[i][j] ;
        	//L[i][j] = 0*w[i]*factor[i]*L[i][j] ;
        }
      }
    }


    vpMatrix::stackMatrices(L,Lsd,LT);
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

    double t0= vpTime::measureTimeMs();
    LTL = LT.AtA();
    double t1= vpTime::measureTimeMs();
    std::cout<<"timeiter"<< t1-t0<<std::endl;
    diagLTL.eye(6);
    for(int i = 0 ; i < 6 ; i++) diagLTL[i][i] = LTL[i][i];
    LTL=LTL + 0.003*diagLTL;
    //std::cout << " ltl " << LTL.pseudoInverse() << std::endl;
    //std::cout<<  " H " << H << std::endl;
    computeJTR(LT, weighted_error, LTR);
    v = -1*LTL.pseudoInverse()*LTR;
    //v = -0.7*(LTL.pseudoInverse())*LT.t()*weighted_error;
    //v = -0.5*(LTL.pseudoInverse())*LT.t()*weighted_error;
    //std::cout << v << std::endl;
    cMo =  vpExponentialMap::direct(v).inverse() * cMo;


    iter++;
  }
std::cout << "\t First minimization in " << iter << " iteration00 " << std::endl ;

/*** Second phase ***/

/*  vpRobust robust(nerror);
  robust.setIteration(0) ;
  iter = 0;
  //vpColVector error_px(nerror);

  while (((int)((residu_1 - r)*1e8) !=0 ) && (iter<15))
  {
    points[scaleLevel].front() ;
    int n = 0 ;
    while (!points[scaleLevel].outside())
    {
      p = points[scaleLevel].value();
      p->computeInteractionMatrixError(cMo,_I);
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
      points[scaleLevel].next() ;
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
  }*/


  //std::cout<<iter<<std::endl;
  points[scaleLevel].front() ;
  int n =0 ;
  while (!points[scaleLevel].outside())
  {
    p = points[scaleLevel].value() ;
    {
      double wmean = 0 ;
      /*if (p->nbFeature > 0)*/ p->list.front();

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
    points[scaleLevel].next() ;
  }
//   cout << "\t Robust minimization in " << iter << " iteration " << endl ;
//    std::cout << "error: " << (residu_1 - r) << std::endl;
}

/*void apMbHybridTracker::loadOgre(apOgre &_ogre)
{
	ogre = _ogre;
}*/


void
apMbHybridTracker::trackHyb(const vpImage<unsigned char> &I, const double dist, apOgre &_ogre)
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
        double t00= vpTime::measureTimeMs();
        updateRT(_ogre);
        double t10= vpTime::measureTimeMs();
        std::cout << "timeRTT "<<t10-t00<<std::endl;
        }
        catch(...)
        {
        	vpTRACE("Error in points extraction");
        }
        try
        {
        double t0= vpTime::measureTimeMs();
        extractControlPoints(I, Inormd, Ior,Ior, dist);
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
          computeVVSHyb(*Ipyramid[lvl], _ogre);
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

        // Looking for new visible face

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
}


/*void
vpMbPointsTracker::init(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &_cMo)
{
  this->cMo = _cMo;

  sId.init(I.getHeight(), I.getWidth(),  1) ;
  sI.init(I.getHeight(), I.getWidth(),  1) ;
  sId.setCameraParameters(cam);
  sI.setCameraParameters(cam);

  Iprec=I;

  }*/


