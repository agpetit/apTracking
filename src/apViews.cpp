#include <visp/vpMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDisplay.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpImageConvert.h>
#include <visp/vpImageFilter.h>
#include <visp/vpPoint.h>
#include <visp/vpTime.h>
#include <visp/vpRGBa.h>
#include <visp/vpIoTools.h>
#include <visp/vpDot2.h>
#include <visp/vpDot.h>
#include <visp/vpFeatureDisplay.h>
#include <visp/vpImageTools.h>
#include "apLogPolarHist.h"
//#include "vpAROgre.h"
#include <cv.h>
#include <math.h>
#include "apViews.h"
#define INF 1E6

#ifndef MINDOUBLE
#define MINDOUBLE 2.2250e-308
#endif
#ifndef MAXDOUBLE
#define MAXDOUBLE 1.7976e308
#endif

using namespace std ;
using namespace cv;

/*!
 * \Initialize apViews
*/
void
apViews::init()
{
EdgeOrientMaps.resize(1);
DTTemp.resize(1);
argDTTemp.resize(1);
mergeTemp.resize(1);
Ipyramid.resize(0);
Hviews.resize(1);
//views.resize(1);
//argDTTempNbhd.resize(1);
}


/*!
 * \Initialize the view sphere
 *
 * \param s_Rho : number of views on the view sphere along rho (distance virtual camera/view sphere center)
 * \param s_Theta : number of views on the view sphere along the longitude parameter theta
 * \param s_Phi : number of views on the view sphere along the lattitude parameter phi
 * \param nb_Theta : number of regions on the view sphere along theta
 * \param nb_Phi : number of regions on the view sphere along phi
 * \param overl : number of views overlapping over consecutive regions, along phi and theta
 *
 */
void
apViews::initViewSphere(apLearn &_learn, std::string _object)
{
sample_Rho = _learn.sampleViewsRho;
sample_Theta=_learn.sampleViewsTheta;
sample_Phi=_learn.sampleViewsPhi;
nbhd_Theta = _learn.sampleRViewsTheta;
nbhd_Phi = _learn.sampleRViewsPhi;
snbhd_Theta = (int)sample_Theta/nbhd_Theta;
snbhd_Phi =  (int)sample_Phi/nbhd_Phi;
overlap = _learn.nOverlap;
dist = _learn.dist;
object = _object;
}

void
apViews::initDetection(apDetection &_detect)
{
	muD = _detect.mud;
	lambdaO = _detect.lambdao;
	cannyTh1 = _detect.cannyTh1;
	cannyTh2 = _detect.cannyTh2;
}

/*! 
  Default constructor that build apViews.
*/
apViews::apViews()
{
    init();
}

/*! 
  Default destructor.
*/
apViews::~apViews()
{
vpImage<unsigned char> *Ior ;
vpImage<double> *IDT ;
vpImage<vpRGBa> *IargDT;

	 for (unsigned int i = 0; i < EdgeOrientMaps.size(); i += 1){
	        Ior = EdgeOrientMaps[i];
	        IDT = DTTemp[i];
	        IargDT = argDTTemp[i];
	        if (Ior!=NULL) delete Ior;
	        if (IDT!=NULL) delete IDT;
	        if (IargDT!=NULL) delete IargDT;
	        Ior = NULL;
	        IDT = NULL;
	        IargDT = NULL;
}
	  EdgeOrientMaps.resize(0);
	  DTTemp.resize(0);
	  argDTTemp.resize(0);
}


void apViews::dT(vpImage<unsigned char> &I0, vpImage<double> &I1, vpImage<vpRGBa> &I2)
{
	// compute dt
/*vpImage<unsigned char>* I00;
I00=EdgeOrientMaps[n];*/
//I00=&I0;
vpImage<unsigned char>* I00;
I00=&I0;
vpImage<unsigned char> I3(480,640);
vpImage<double> *out;
vpImage<vpRGBa> *imArg = new vpImage<vpRGBa>;
imArg->resize(480,640);
I1.resize(480,640);
I2.resize(480,640);
//std::cout<<" ok "<<std::endl;
out=dt(I00,imArg);
// take square roots
for (int y = 0; y < (&I0)->getHeight(); y++) {
  for (int x = 0; x < (&I0)->getWidth(); x++) {
	  //(*out)[y][x] = sqrt((*out)[y][x]);
	  //cout << (double)(*out)[y][x] << endl;
	  I1[y][x]=sqrt((*out)[y][x]);
	  //I3[y][x]=(unsigned char)(*out)[y][x];
	  (I2[y][x]).R=((*imArg)[y][x]).R;
	  (I2[y][x]).G=((*imArg)[y][x]).G;
	  //(I2[y][x]).B=(unsigned char) sqrt((*out)[y][x]);
	  //(I2[y][x]).A=(*I00)[y][x];
	  //std::cout<<" ok "<< (double)I1[y][x] << std::endl;
  }
}
//vpImageIo::writePPM(I3, "ImdT.pgm");
//std::cout<<" ok0 "<<std::endl;
}


float* apViews::dtx(float *f, float *g, int n, int *argx, int *argy) {
  float *d = new float[n];
  int *v = new int[n];
  float *z = new float[n+1];
  int k = 0;
  v[0] = 0;
  z[0] = -INF;
  z[1] = +INF;
  for (int q = 1; q <= n-1; q++) {
    float s  = ((f[q]+vpMath::sqr(q))-(f[v[k]]+vpMath::sqr(v[k])))/(2*q-2*v[k]);
    while (s <= z[k]) {
      k--;
      s  = ((f[q]+vpMath::sqr(q))-(f[v[k]]+vpMath::sqr(v[k])))/(2*q-2*v[k]);
    }
    k++;
    v[k] = q;
    z[k] = s;
    z[k+1] = +INF;
  }

  k = 0;
  for (int q = 0; q <= n-1; q++) {
    while (z[k+1] < q)
      k++;
    d[q] = vpMath::sqr(q-v[k]) + f[v[k]];
    argx[q] = q-v[k];
    argy[q] = g[v[k]];
  }

  delete [] v;
  delete [] z;
  return d;
}

float* apViews::dty(float *f, float *g, int n) {
  float *d = new float[n];
  int *v = new int[n];
  float *z = new float[n+1];
  int k = 0;
  v[0] = 0;
  z[0] = -INF;
  z[1] = +INF;
  for (int q = 1; q <= n-1; q++) {
    float s  = ((f[q]+vpMath::sqr(q))-(f[v[k]]+vpMath::sqr(v[k])))/(2*q-2*v[k]);
    while (s <= z[k]) {
      k--;
      s  = ((f[q]+vpMath::sqr(q))-(f[v[k]]+vpMath::sqr(v[k])))/(2*q-2*v[k]);
    }
    k++;
    v[k] = q;
    z[k] = s;
    z[k+1] = +INF;
  }

  k = 0;
  for (int q = 0; q <= n-1; q++) {
    while (z[k+1] < q)
      k++;
    d[q] = vpMath::sqr(q-v[k]) + f[v[k]];
    g[q] = q-v[k];
  }

  delete [] v;
  delete [] z;
  return d;
}

/* dt of 2d function using squared distance */
void apViews::dtf(vpImage<double> *im, vpImage<vpRGBa> *imArg) {
  int width = im->getWidth();
  int height = im->getHeight();
  float *f = new float[std::max(width,height)];
  float *g = new float[std::max(width,height)];
  vpColVector arg(2);
  // transform along columns
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      f[y] = (float)(*im)[y][x];
    }
    //int *argy = new int[height];
    float *d = dty(f,g, height);
    for (int y = 0; y < height; y++) {
    	 (*im)[y][x] = (double)d[y];
         ((*imArg)[y][x]).R = g[y]/2+127;
        /* if (argy[y] <10000)
         cout << sqrt(argy[y]) << endl;*/
    }
    delete [] d;
    //delete [] argy;
  }
  // transform along rows
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      f[x] =  (float)(*im)[y][x];
      g[x] =  ((*imArg)[y][x]).R;
    }
    int *argx = new int[width];
    int *argy = new int[width];
    float *d = dtx(f,g, width, argx, argy);
    for (int x = 0; x < width; x++) {
    	 (*im)[y][x]= (double)d[x];
    	 ((*imArg)[y][x]).R = argy[x];
    	 ((*imArg)[y][x]).G = argx[x]/2+127;
    }
    delete [] d;
    delete [] argx;
    delete [] argy;
  }
  delete f;
  delete g;
}


void apViews::dtf0(vpImage<double> *im, vpImage<vpRGBa> *imArg) {
  int width = im->getWidth();
  int height = im->getHeight();
  float *f = new float[std::max(width,height)];
  float *g = new float[std::max(width,height)];
  vpColVector arg(2);
  // transform along columns
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      f[y] = (float)(*im)[y][x];
    }
    //int *argy = new int[height];
    float *d = dty(f,g, height);
    for (int y = 0; y < height; y++) {
    	 (*im)[y][x] = (double)d[y];
         ((*imArg)[y][x]).R = g[y]/2+127;
        /* if (argy[y] <10000)
         cout << sqrt(argy[y]) << endl;*/
    }
    delete [] d;
    //delete [] argy;
  }
  // transform along rows
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      f[x] =  (float)(*im)[y][x];
      g[x] =  ((*imArg)[y][x]).R;
    }
    int *argx = new int[width];
    int *argy = new int[width];
    float *d = dtx(f,g, width, argx, argy);
    for (int x = 0; x < width; x++) {
    	if(sqrt(d[x])<255)
    	 (*imArg)[y][x].B= sqrt(d[x]);
    	else (*imArg)[y][x].B = 255;
    	 ((*imArg)[y][x]).R = argy[x];
    	 ((*imArg)[y][x]).G = argx[x]/2+127;
    }
    delete [] d;
    delete [] argx;
    delete [] argy;
  }
  delete f;
  delete g;
}


vpImage<double>* apViews::dt(vpImage<unsigned char> *im, vpImage<vpRGBa> *imArg){
  int width = im->getWidth();
  int height = im->getHeight();

  vpImage<double> *out = new vpImage<double>;
  out->resize(height,width);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      if ((*im)[y][x]==100)
      {
	(*out)[y][x] = INF;
      }
      else
      {
    (*out)[y][x] = 0;
      }
    }
  }
  dtf(out, imArg);
  return out;
}

/*!
 * \Compute the distance transform of an oriented edge map
 *
 * \param im : pointer to the input oriented edge map
 * \return an RGBA image with B corresponding to the distances to the closest edge, R and G to to the image coordinates of this closest edge point
 *
 */

vpImage<vpRGBa>* apViews::dt0(vpImage<unsigned char> *im){
  int width = im->getWidth();
  int height = im->getHeight();

  vpImage<double> *out = new vpImage<double>;
  vpImage<vpRGBa> *imArg = new vpImage<vpRGBa>;
  out->resize(height,width);
  imArg->resize(height,width);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      if ((*im)[y][x]==100)
      {
	(*out)[y][x] = INF;
      }
      else
      {
    (*out)[y][x] = 0;
      }
    }
  }
  dtf0(out, imArg);
  /*for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      if ((*im)[y][x]!=100)
      {
    	  (*imArg)[y][x].A!= *im[y][x];
      }
    }
  }*/
  return imArg;
}

void apViews::MSERs(vpImage<unsigned char> &I0)
{


}


void apViews::bin(vpImage<unsigned char> &I1)
{
	for (int n=0; n < I1.getHeight() ; n++)
	{
	for (int m=0 ; m < I1.getWidth(); m++)
	  {
		//cout << (double)I1[n][m] << endl;
		if(I1[n][m]>0)
	    I1[n][m]=255;
		else{
			I1[n][m]=0;
		}
	  }
	}
}

/*!
 * \Compute the dots of an input segmented image
 *
 * \param I1 : input image, segmented through motion/color segmentation (Criminisi2006)
 * \param cog : center of gravity of biggest dot
 * \param angle : orientation of the biggest dot
 * \param surface : area of the biggest dot
 *
 */

void apViews::computePosOri(vpImage<unsigned char> &I1, vpImagePoint &cog,double &angle, int &surface)
{
vpImage<vpRGBa> Ioverlay;
//vpDisplayX display1;
//display1.init(I1, 800, 10, "Dots");
for (int n=0; n < I1.getHeight() ; n++)
{
for (int m=0 ; m < I1.getWidth(); m++)
  {
	if(I1[n][m]>0)
    I1[n][m]=255;
	else{
		I1[n][m]=0;
	}
  }
}
double opt_sizePrecision = 0;
double opt_grayLevelPrecision = 0.9;
double opt_ellipsoidShapePrecision = 0;
vpDot2 d;
d.setGraphics(true);
d.setWidth(50.0);
d.setHeight(50.0);
d.setArea(5000);
d.setGrayLevelMin(200);
d.setGrayLevelMax(255);
d.setGrayLevelPrecision(opt_grayLevelPrecision);
d.setSizePrecision(opt_sizePrecision);
d.setEllipsoidShapePrecision(opt_ellipsoidShapePrecision);
d.setComputeMoments(true);
int surface0;
vpImagePoint cog0;
double angle0;
surface = 0;
angle = 0;
//vpDisplay::display(I1);
vpList<vpDot2>* list_d;
vpDot2 dotB;
dotB.setGraphics(true);
//list_d = d.searchDotsInArea(I1, 0, 0, I1.getWidth(), I1.getHeight());

    if( list_d->nbElement() == 0 )  {
      std::cout << "Dot auto detection did not work." << std::endl;
      //return ;
    }
    else {
      std::cout << std::endl << list_d->nbElement() << " dots are detected" << std::endl;
  {
	int i=0;
        // Parse all founded dots for display
        list_d->front();
        while (!list_d->outside()) {
          vpDot2 tmp_d;
          tmp_d.setComputeMoments(true);
          tmp_d.setGraphics(true);
          tmp_d = list_d->value();
          tmp_d.track(I1);
          //tmp_d.display(I1,vpColor::red,2);
          double theta;
	  cog0 = tmp_d.getCog();
	  angle0 = 0.5*atan2(2*tmp_d.mu11,(tmp_d.mu20-tmp_d.mu02));
          surface0 = tmp_d.getArea();
	  if (surface0>surface && cog0.get_i()>100 && cog0.get_j()>100 )
	  {
		  surface = surface0;
		  angle = angle0;
		  cog=cog0;
	  }

          list_d->next();
        }
      }
    }
    //vpDisplay::displayCross(I1, cog, 20, vpColor::green) ;
    //dotB.display(I1,vpColor::green,2);
    angle = angle;
    cout << " Dot of the segmented image : "  <<  " COG : " << cog0.get_u() << " " << cog0.get_v() << " Angle : " << angle << " Surface : " << surface << endl;

    // free memory allocated for the list of dots found in d.searchDotsInArea()
    list_d->kill();
    delete list_d;
    //vpDisplay::flush(I1);
    /*vpDisplay::getImage(I1,Ioverlay);
    vpImageIo::writePPM(I1, "Iseg0.pgm");*/
}

void apViews::computePosOriMean(vpImage<unsigned char> &I1, vpImagePoint &cog,double &angle, int &surface)
{
vpImage<vpRGBa> Ioverlay;
//vpDisplayX display1;
//display1.init(I1, 800, 10, "Dots");
for (int n=10; n < I1.getHeight()-10 ; n++)
{
for (int m=10 ; m < I1.getWidth()-10; m++)
  {
	if(I1[n][m]>0)
    I1[n][m]=255;
	else{
		I1[n][m]=0;
	}
  }
}
double opt_sizePrecision = 0;
double opt_grayLevelPrecision = 0.9;
double opt_ellipsoidShapePrecision = 0;
vpDot2 d;
d.setGraphics(true);
d.setWidth(50.0);
d.setHeight(50.0);
d.setArea(5000);
d.setGrayLevelMin(200);
d.setGrayLevelMax(255);
d.setGrayLevelPrecision(opt_grayLevelPrecision);
d.setSizePrecision(opt_sizePrecision);
d.setEllipsoidShapePrecision(opt_ellipsoidShapePrecision);
d.setComputeMoments(true);
int surface0,sumsurface;
vpImagePoint cog0,cogmean;
double angle0,anglemean, mu11mean,mu20mean, mu02mean,m11s,m20s,m02s;
surface = 0;
angle = 0;
sumsurface = 0;
cogmean.set_u(0);
cogmean.set_v(0);
anglemean = 0;
mu11mean = 0;
mu20mean = 0;
mu02mean = 0;
m11s = 0;
m20s = 0;
m02s = 0;
//vpDisplay::display(I1);
vpList<vpDot2>* list_d;
vpDot2 dotB;
dotB.setGraphics(true);
//list_d = d.searchDotsInArea(I1, 0, 0, I1.getWidth(), I1.getHeight());


    if( list_d->nbElement() == 0 )  {
      std::cout << "Dot auto detection did not work." << std::endl;
      //return ;
    }
    else {
      std::cout << std::endl << list_d->nbElement() << " dots are detected" << std::endl;
  {
	int i=0;
        // Parse all founded dots for display
        list_d->front();
        while (!list_d->outside()) {
          vpDot2 tmp_d;
          tmp_d.setComputeMoments(true);
          tmp_d.setGraphics(true);
          tmp_d = list_d->value();
          //tmp_d.track(I1);
          //tmp_d.display(I1,vpColor::red,2);
          double theta;
          surface0 = tmp_d.getArea();

	  if(surface0>100)
	  sumsurface = sumsurface + surface0;

          list_d->next();
        }
        list_d->front();
        while (!list_d->outside()) {
          vpDot2 tmp_d;
          tmp_d.setComputeMoments(true);
          tmp_d.setGraphics(true);
          tmp_d = list_d->value();
          //tmp_d.track(I1);
          //tmp_d.display(I1,vpColor::red,2);
          double theta;
          surface0 = tmp_d.getArea();
	  cog0 = tmp_d.getCog();
	  tmp_d.track(I1);
	  angle0 = 0.5*atan2(2*tmp_d.mu11,(tmp_d.mu20-tmp_d.mu02));
	  if(surface0>100)
	  {
	  cout << " Dot of the segmented image : "  <<  " COG : " << cog0.get_u() << " " << cog0.get_v() << " Angle : " << angle0 << " Surface : " << surface0 << endl;
	  m11s = m11s + tmp_d.m11;//*((double)surface0/sumsurface);
	  m20s = m20s + tmp_d.m20;//*((double)surface0/sumsurface);
	  m02s = m02s + tmp_d.m02;//*((double)surface0/sumsurface);
	  cogmean.set_u(cogmean.get_u()+cog0.get_u()*((double)surface0/sumsurface));
	  cogmean.set_v(cogmean.get_v()+(double)cog0.get_v()*((double)surface0/sumsurface));
	  //anglemean = anglemean + angle0*((double)surface0/sumsurface);
	  }
          list_d->next();
        }
        surface = sumsurface;
        mu11mean = (double)m11s/sumsurface-cogmean.get_u()*cogmean.get_v();
        mu20mean = (double)m20s/sumsurface-cogmean.get_u()*cogmean.get_u();
        mu02mean = (double)m02s/sumsurface-cogmean.get_v()*cogmean.get_v();
        angle = 0.5*atan2(2*mu11mean,(mu20mean-mu02mean));
        angle = 1.5;
        cog = cogmean;
      }
    }
    //vpDisplay::displayCross(I1, cog, 20, vpColor::green) ;
   // dotB.display(I1,vpColor::green,2);
    angle = angle;
    cout << " Dot of the segmented image ok : "  <<  " COG : " << cog.get_u() << " " << cog.get_v() << " Angle : " << angle << " Surface : " << surface << endl;
    //vpDisplay::flush(I1);
   // vpDisplay::getImage(I1,Ioverlay);
    //vpImageIo::writePNG(Ioverlay, "Idot10.png");
    // free memory allocated for the list of dots found in d.searchDotsInArea()
    list_d->kill();
    delete list_d;
}

void apViews::computePosOriMean2(vpImage<unsigned char> &I1, vpImagePoint &cog,double &angle, int &surface)
{

for (int n=10; n < I1.getHeight()-10 ; n++)
{
for (int m=10 ; m < I1.getWidth()-10; m++)
  {
	if(I1[n][m] == 0)
    I1[n][m]=0;
  }
}


double mu11,mu20,mu02,mup11,mup20,mup02,m00,m01,m10,m11,m02,m20;
surface = 0;
angle = 0;
m00=0;
m01 = 0;
m10 =0 ;
m11 = 0;
m20 = 0;
m02 = 0;
mu11 = 0;
mu20=0;
mu02=0;

for (int n=10; n < I1.getHeight()-10 ; n++)
{
for (int m=10 ; m < I1.getWidth()-10; m++)
  {
	if(I1[n][m] > 0)
	{
m00 ++;
m01 += n;
m10 += m;
m11 += m*n;
m02 += n*n;
m20 += m*m;
	}
  }
}
cog.set_u(m10/m00);
cog.set_v(m01/m00);

surface = m00;

mu11 = m11-m10*m01/m00;
mu20 = m20 - m10*m10/m00;
mu02 = m02 - m01*m01/m00;

mup11 = mu11/m00;
mup02 = mu02/m00;
mup20 = mu20/m00;

angle =0.5*atan2(2*mup11,(mup20-mup02));

    cout << " Dot of the segmented image ok : "  <<  " COG : " << cog.get_u() << " " << cog.get_v() << " Angle : " << angle << " Surface : " << surface << endl;
}


/*!
 * \Compute the dot of a binary image computed from a synthetic view
 *
 * \param Itemp : input binary synthetic view
 * \param cog : center of gravity the dot
 * \param angle : orientation of the dot
 * \param surface : area of the dot
 *
 */

void apViews::computeDotView(vpImage<unsigned char> &Itemp, vpImagePoint &cog,double &angle, int &surface)
{
vpImagePoint ip;
int it;
it=0;
for (int n=0; n < Itemp.getHeight() ; n++)
{
for (int m=0 ; m < Itemp.getWidth(); m++)
  {
	if(Itemp[n][m]!=0)
    {Itemp[n][m]=255;
    if(it==1500){
	ip.set_i(n);
	ip.set_j(m);}
    it++;
    }
	else{
		Itemp[n][m]=0;
	}
  }
}
cout << " niter " << it << endl;
//vpImageIo::writePNG(Itemp, "Iseg1.png");
double opt_sizePrecision = 0.1;
double opt_grayLevelPrecision = 0.5;
vpDot d;
d.setGraphics(true);
d.setComputeMoments(true);
d.setConnexity(vpDot::CONNEXITY_8);
d.setMaxDotSize(0.3);
d.initTracking(Itemp, ip);
d.track(Itemp);
cog = d.getCog();
angle = 0.5*atan2(2*d.mu11,(d.mu20-d.mu02));
surface = d.m00;
//vpDisplay::displayCross(Itemp, cog, 20, vpColor::red);
std::cout << "Dot synthetic view : " << " COG : " << cog.get_u() << " " << cog.get_v() << " Angle : " << angle << " Surface : " << surface << std::endl;

    /*vpDisplay::display(I1);
    vpDisplay::flush(I1);*/
    //vpDisplay::getImage(I1,Ioverlay);
    //vpImageIo::writePNG(Ioverlay, "Iseg0.png");
}

void apViews::computeDotView2(vpImage<unsigned char> &Itemp, vpImagePoint &cog,double &angle, int &surface)
{
vpImagePoint ip;
int it;
it=0;
for (int n=0; n < Itemp.getHeight() ; n++)
{
for (int m=0 ; m < Itemp.getWidth(); m++)
  {
	if(Itemp[n][m]!=0)
    {Itemp[n][m]=255;
    if(it==1500){
	ip.set_i(n);
	ip.set_j(m);}
    it++;
    }
	else{
		Itemp[n][m]=0;
	}
  }
}
cout << " niter " << it << endl;
double mu11,mu20,mu02,mup11,mup20,mup02,m00,m01,m10,m11,m02,m20;
surface = 0;
angle = 0;
m00=0;
m01 = 0;
m10 =0 ;
m11 = 0;
m20 = 0;
m02 = 0;
mu11 = 0;
mu20=0;
mu02=0;

for (int n=10; n < Itemp.getHeight()-10 ; n++)
{
for (int m=10 ; m < Itemp.getWidth()-10; m++)
  {
	if(Itemp[n][m] > 0)
	{
m00 ++;
m01 += n;
m10 += m;
m11 += m*n;
m02 += n*n;
m20 += m*m;
	}
  }
}
cog.set_u(m10/m00);
cog.set_v(m01/m00);

surface = m00;

mu11 = m11-m10*m01/m00;
mu20 = m20 - m10*m10/m00;
mu02 = m02 - m01*m01/m00;

mup11 = mu11/m00;
mup02 = mu02/m00;
mup20 = mu20/m00;

angle =0.5*atan2(2*mup11,(mup20-mup02));

std::cout << "Dot synthetic view : " << " COG : " << cog.get_u() << " " << cog.get_v() << " Angle : " << angle << " Surface : " << surface << std::endl;

    /*vpDisplay::display(I1);
    vpDisplay::flush(I1);*/
    //vpDisplay::getImage(I1,Ioverlay);
    //vpImageIo::writePNG(Ioverlay, "Iseg0.png");
}


/*!
 * \Compute the oriented edge map of an input image
 *
 * \param I0 : input image, returned into an oriented edge map with edge point pixel values corresponding to the edge orientation
 */

void apViews::edgeOrientMap(vpImage<unsigned char> &I0)
{
int width = I0.getWidth();
int height = I0.getHeight();
vpImage<unsigned char> I1(height,width);
vpImage<double> Iu;
vpImage<double> Iv;
vpImage<double> imG(height,width);
int n,m;
for (n=3; n < height-3 ; n++)
{
for (m = 3 ; m < width-3; m++)
  {
    imG[n][m] =   vpImageFilter::gaussianFilter(I0,n,m);
  }
}
for (n=3; n < height-3 ; n++)
{
for (m = 3 ; m < width-3; m++)
  {
   // I0[n][m] =   (int)imG[n][m];
  }
}

Mat Ip;
Mat dst0;
vpImageConvert::convert(I0, Ip);
Mat src0 = Ip.clone();
Mat dst ( Size(I0.getWidth(),I0.getHeight()), CV_8U);
Canny( Ip, dst, cannyTh1, cannyTh2, 3 );
//cvCanny( Ip, dst, 150,210, 3 );
vpImageConvert::convert(dst, I1);
//vpImageIo::writePNG(I1, "Iseg3.png");
double a,b;

/*for (n=3; n < height-3 ; n++)
{
for (m = 3 ; m < width-3; m++)
  {
    imG[n][m] =   vpImageFilter::gaussianFilter(I0,n,m);
  }
}*/
for (int i=0;i<height-0;i++)
		{
			for (int j=0;j<width-0;j++)
			{
				if(i>3 && i < height-3 && j>3 && j < width-3){
				a=(apImageFilter::sobelFilterX(I0,i,j));
				b=(apImageFilter::sobelFilterY(I0,i,j));
			    if ( I1[i][j]==255) //
				{

				 I1[i][j]=255*(-(atan(a/b))/M_PI+1/2);

				}
				 else {I1[i][j]=100;}
				}
				else{
					I1[i][j]=100;
				}
			}
	}

I0 = I1;


}

int apViews::edgeOrientMapN(vpImage<unsigned char> &I0)
{
int width = I0.getWidth();
int height = I0.getHeight();
vpImage<unsigned char> I1(height,width);
vpImage<double> Iu;
vpImage<double> Iv;
vpImage<double> imG(height,width);
int n,m;
int nedge;
for (n=3; n < height-3 ; n++)
{
for (m = 3 ; m < width-3; m++)
  {
    imG[n][m] =   vpImageFilter::gaussianFilter(I0,n,m);
  }
}
for (n=3; n < height-3 ; n++)
{
for (m = 3 ; m < width-3; m++)
  {
    I0[n][m] =   (int)imG[n][m];
  }
}

Mat Ip;
Mat dst0;
vpImageConvert::convert(I0, Ip);
Mat src0=Ip.clone();
Mat dst(Size(I0.getWidth(),I0.getHeight()), CV_8U);
Canny( Ip, dst, cannyTh1, cannyTh2, 3 );
vpImageConvert::convert(dst, I1);
//vpImageIo::writePNG(I1, "Iseg4.png");
double a,b;

/*for (n=3; n < height-3 ; n++)
{
for (m = 3 ; m < width-3; m++)
  {
    imG[n][m] =   vpImageFilter::gaussianFilter(I0,n,m);
  }
}*/
nedge = 0;
for (int i=0;i<height-0;i++)
		{
			for (int j=0;j<width-0;j++)
			{
				if(i>3 && i < height-3 && j>3 && j < width-3){
				a=(apImageFilter::sobelFilterX(I0,i,j));
				b=(apImageFilter::sobelFilterY(I0,i,j));
			    if ( I1[i][j]==255) //
				{

				 I1[i][j]=255*(-(atan(a/b))/M_PI+1/2);
				 nedge++;

				}
				 else {I1[i][j]=100;}
				}
				else{
					I1[i][j]=100;
				}
			}
	}

I0 = I1;

//std::cout << " nedge " << nedge << std::endl;
return nedge;

}

/*!
 * \brief Build clusters with Affinity Propagation (Dueck 2007)
 *
 * \param idx : table to store clusters and cluster centers
 */

void apViews::buildClusters(unsigned long *idx, const int maxits, const int convits, const double lam)
{;
    vpMatrix simMat,preferences;
    computeSimilarityMatrix(simMat,preferences);
    transProb = simMat;
    int flag, dn, it, conv, decit;//, maxits, convits;
    unsigned long i1,i2,j, *i, *k, m, n, l, **dec, *decsum, K;
    double tmp, *s, *a, *r, *p, *mx1, *mx2, *srp, netsim, dpsim, expref;//, lam;
    FILE *f;
    n=preferences.getRows();
    m=simMat.getRows();
    //cout<<simMat<<endl;
    // Allocate memory for similarities, preferences, messages, etc
    i=(unsigned long *)calloc(m+n,sizeof(unsigned long));
    k=(unsigned long *)calloc(m+n,sizeof(unsigned long));
    s=(double *)calloc(m+n,sizeof(double));
    a=(double *)calloc(m+n,sizeof(double));
    r=(double *)calloc(m+n,sizeof(double));
    mx1=(double *)calloc(n,sizeof(double));
    mx2=(double *)calloc(n,sizeof(double));
    srp=(double *)calloc(n,sizeof(double));
    dec=(unsigned long **)calloc(convits,sizeof(unsigned long *));
    for(j=0;j<convits;j++)
        dec[j]=(unsigned long *)calloc(n,sizeof(unsigned long));
    decsum=(unsigned long *)calloc(n,sizeof(unsigned long));
    //idx=(unsigned long *)calloc(n,sizeof(unsigned long));

    // Read similarities and preferences

    for(j=0;j<m;j++){
        (i[j]) = simMat[j][0];
        (k[j]) = simMat[j][1];
        (s[j]) = simMat[j][2];
        i[j]--; k[j]--;
    }

    for(j=0;j<n;j++){
        i[m+j]=j; k[m+j]=j;
        (s[m+j])=preferences[j][0];
    }

    m=m+n;
    // Include a tiny amount of noise in similarities to avoid degeneracies
    for(j=0;j<m;j++) s[j]=s[j]+(1e-16*s[j]+MINDOUBLE*100)*(rand()/((double)RAND_MAX+1));

    // Initialize availabilities to 0 and run affinity propagation
    for(j=0;j<m;j++) a[j]=0.0;
    for(j=0;j<convits;j++) for(i1=0;i1<n;i1++) dec[j][i1]=0;
    for(j=0;j<n;j++) decsum[j]=0;
    dn=0; it=0; decit=convits;
    while(dn==0){
        it++; // Increase iteration index

        // Compute responsibilities
        for(j=0;j<n;j++){ mx1[j]=-MAXDOUBLE; mx2[j]=-MAXDOUBLE; }
        for(j=0;j<m;j++){
            tmp=a[j]+s[j];
            if(tmp>mx1[i[j]]){
                mx2[i[j]]=mx1[i[j]];
                mx1[i[j]]=tmp;
            } else if(tmp>mx2[i[j]]) mx2[i[j]]=tmp;
        }
        for(j=0;j<m;j++){
            tmp=a[j]+s[j];
            if(tmp==mx1[i[j]]) r[j]=lam*r[j]+(1-lam)*(s[j]-mx2[i[j]]);
            else r[j]=lam*r[j]+(1-lam)*(s[j]-mx1[i[j]]);
        }
        // Compute availabilities
        for(j=0;j<n;j++) srp[j]=0.0;
        for(j=0;j<m-n;j++) if(r[j]>0.0) srp[k[j]]=srp[k[j]]+r[j];
        for(j=m-n;j<m;j++) srp[k[j]]=srp[k[j]]+r[j];
        for(j=0;j<m-n;j++){
            if(r[j]>0.0) tmp=srp[k[j]]-r[j]; else tmp=srp[k[j]];
            if(tmp<0.0) a[j]=lam*a[j]+(1-lam)*tmp; else a[j]=lam*a[j];
        }
        for(j=m-n;j<m;j++) a[j]=lam*a[j]+(1-lam)*(srp[k[j]]-r[j]);

        // Identify exemplars and check to see if finished
        decit++; if(decit>=convits) decit=0;
        for(j=0;j<n;j++) decsum[j]=decsum[j]-dec[decit][j];
        for(j=0;j<n;j++)
            if(a[m-n+j]+r[m-n+j]>0.0) dec[decit][j]=1; else dec[decit][j]=0;
        K=0; for(j=0;j<n;j++) K=K+dec[decit][j];
        for(j=0;j<n;j++) decsum[j]=decsum[j]+dec[decit][j];
        if((it>=convits)||(it>=maxits)){
            // Check convergence
            conv=1; for(j=0;j<n;j++) if((decsum[j]!=0)&&(decsum[j]!=convits)) conv=0;
            //Check to see if donefbuildview
            if(((conv==1)&&(K>0))||(it==maxits)) dn=1;
        }
    }
    // If clusters were identified, find the assignments and output them
    if(K>0){
        for(j=0;j<m;j++)
            if(dec[decit][k[j]]==1) a[j]=0.0; else a[j]=-MAXDOUBLE;
        for(j=0;j<n;j++) mx1[j]=-MAXDOUBLE;
        for(j=0;j<m;j++){
            tmp=a[j]+s[j];
            if(tmp>mx1[i[j]]){
                mx1[i[j]]=tmp;
                idx[i[j]]=k[j];
            }
        }
        for(j=0;j<n;j++) if(dec[decit][j]) idx[j]=j;
        for(j=0;j<n;j++) srp[j]=0.0;
        for(j=0;j<m;j++) if(idx[i[j]]==idx[k[j]]) srp[k[j]]=srp[k[j]]+s[j];
        for(j=0;j<n;j++) mx1[j]=-MAXDOUBLE;
        for(j=0;j<n;j++) if(srp[j]>mx1[idx[j]]) mx1[idx[j]]=srp[j];
        for(j=0;j<n;j++)
            if(srp[j]==mx1[idx[j]]) dec[decit][j]=1; else dec[decit][j]=0;
        for(j=0;j<m;j++)
            if(dec[decit][k[j]]==1) a[j]=0.0; else a[j]=-MAXDOUBLE;
        for(j=0;j<n;j++) mx1[j]=-MAXDOUBLE;
        for(j=0;j<m;j++){
            tmp=a[j]+s[j];
            if(tmp>mx1[i[j]]){
                mx1[i[j]]=tmp;
                idx[i[j]]=k[j];
            }
        }
        for(j=0;j<n;j++) if(dec[decit][j]) idx[j]=j;
        //f=fopen(filename2,"w");
        for(j=0;j<n;j++) {
            idx[j] = idx[j]+1;
        }

        dpsim=0.0; expref=0.0;
        for(j=0;j<m;j++){
            if(idx[i[j]]==k[j]){
                if(i[j]==k[j]) expref=expref+s[j];
                else dpsim=dpsim+s[j];
            }
        }
        netsim=dpsim+expref;
        printf("\nNumber of identified clusters: %d\n",K);
        printf("Fitness (net similarity): %f\n",netsim);
        printf("  Similarities of data points to exemplars: %f\n",dpsim);
        printf("  Preferences of selected exemplars: %f\n",expref);
        printf("Number of iterations: %d\n\n",it);
    } else printf("\nDid not identify any clusters\n");
    if(conv==0){
        printf("\n*** Warning: Algorithm did not converge. Consider increasing\n");
        printf("    maxits to enable more iterations. It may also be necessary\n");
        printf("    to increase damping (increase dampfact).\n\n");
    }
    /*free(i);
      free(k);
      free(s);
      free(a);
      free(r);
      free(mx1);
      free(mx2);
      free(srp);
      free(dec);*/
}


void apViews::centerView(vpImage<unsigned char> &Itemp, vpImagePoint &cog)
{
    int xc,yc;
    int height = Itemp.getHeight();
    int width = Itemp.getWidth();
    vpImage<unsigned char> Itransl(height, width);
    for (int y = 0; y < Itemp.getHeight(); y++){
        for (int x = 0; x < Itemp.getWidth(); x++){
            Itransl[y][x] = 100;
        }
    }
    for (int y = 0; y < Itemp.getHeight(); y++){
        for (int x = 0; x < Itemp.getWidth(); x++){
            xc = x-(int)(cog.get_j()-width/2);
            yc = y-(int)(cog.get_i()-height/2);
            if((Itemp)[y][x]!=100 && yc >0 && yc<height && xc >0 && xc <width)
            {
                Itransl[yc][xc]=(Itemp)[y][x];
            }
        }
    }
    Itemp=Itransl;
}

/*!
 * \brief Build the hierarchical view graph
 *
 * \param
 */

void apViews::buildViewGraph(vpCameraParameters &mcam, SceneManager *mgr, std::string opath, int height, int width)
{

	// File where the graph is stored
std::string hf = "h" + object + ".txt";
char *filename = (char *)hf.c_str();
// File to store the data (pose...) of each view at the first level
std::string data0f = "data" + object + "0.txt";
char *filename0 = (char *)data0f.c_str();
//File to store the data (pose...) of the views at each level of the hierarchical view graph
std::string data1f = "data" + object + "1.txt";
char *filename1 = (char *)data1f.c_str();
// File to store the transition probabilities between each prototype view at the last level of the hierarchy
std::string transP = "transProb" + object + ".txt";
char *filenameTP = (char *)transP.c_str();

vpImage<vpRGBa> * I0,*I1,*I2,*I3,*I4,*I5,*I6,*mv,*mv0;
vpImage<unsigned char> I(height,width);
int level,nrow,nm,lm,surface,ns,nos,nos0,mm,row,noos,nbimg,il,jl,nregion;
level=0;
nrow=0;
ns=0;
lm=1;
nregion=0;
bool flag = false;
double sim,simax,xx,yy,ori;
double rho, theta, phi;
double Rho, Theta, Phi, x, y, z;
vpMatrix hierarchy(5000,80);
unsigned long *idx;
vpDot2 dot;
vpImagePoint cog;
vpHomogeneousMatrix cMo;
vpPoseVector pose;
vpImage<unsigned char> Ior0;
vpImage<unsigned char> Ior1(height,width);
vpImage<unsigned char> Ior2(height,width);
vpImage<unsigned char> Ior3(height,width);
vpImage<unsigned char> Ior4(height,width);
vpImage<vpRGBa> *imArg;
vpImage<vpRGBa> Imap(height,width);
vpImage<vpRGBa> Itex(height,width);
vpImage<unsigned char> Iseg(height,width);
vpRotationMatrix R1;
vpTranslationVector trans;
vpMatrix dataTemp(100000,10);
vpMatrix dataTemp1(100000,10);
std::string opath01 = "/local/agpetit/images" + vpIoTools::path("/imagesPA/hierarchy/Itex%04d.png");
char buf0[FILENAME_MAX];
Rho=dist;
theta=0;
phi=0;
nbimg=0;
row=0;
//double offset=M_PI/40;
double offset = 0;
for (int i = 0; i<nbhd_Theta;i++){
for (int j=0; j<nbhd_Phi;j++){
Theta=M_PI*(-1/2+(double)i/nbhd_Theta) + offset;// - M_PI ;//- M_PI/2;// - M_PI/2;
Phi=M_PI*(1/2+(double)j/nbhd_Phi);//- M_PI/2;
//createViewsSphNb(ogre,Theta,Phi,Rho,nrow,opath);
//row=ns;
nregion++;
std::cout << " Generate synthetic views, region " << nregion << std::endl;
for (int k=0;k<sample_Rho;k++){
for (int l=0;l<snbhd_Theta+overlap;l++){
for (int m=0;m<snbhd_Phi+overlap;m++){
rho=Rho;
theta = Theta + (double)l*(M_PI/(nbhd_Theta))/(snbhd_Theta)+3*M_PI/2;
phi =  -Phi - (double)m*(M_PI/(nbhd_Phi))/(snbhd_Phi)+M_PI/2;
trans[0] = 0;
trans[1] = 0;
trans[2] = Rho;
vpRotationMatrix R2(vpRzyxVector(theta,0,phi));
R1 = R2.inverse();
cMo.buildFrom(trans,R1);
pose.buildFrom(cMo);
vpImage<unsigned char> *I00 = new vpImage<unsigned char>;
vpImage<vpRGBa> *I01 = new vpImage<vpRGBa>;
I00->resize(height,width);
I01->resize(height,width);
mgr->updateRTT(Imap,*I00,&cMo);
//mgr->updateRTTCol(*I01,Imap,*I00,&cMo);

for (int y = 0; y < Imap.getHeight(); y++){
  for (int x = 0; x < Imap.getWidth(); x++){
	  Iseg[y][x]=Imap[y][x].A;
  }
  }
//vpImageIo::writePNG(*I00,"Icol.png");
computeDotView2(Iseg,cog,ori,surface);
centerView(*I00,cog);
//centerView(*I01,cog);
//rotateView(*I00,Ior1,ori);
//*I00 = Ior1;
imArg=dt0(I00);
for (int y = 0; y < I00->getHeight(); y++){
  for (int x = 0; x < I00->getWidth(); x++){
	  (*imArg)[y][x].A = (double)(*I00)[y][x];
	  /*if ((*I00)[y][x]!=100)
	  Ior1[y][x]=0;//(*imArg)[y][x].B;
	  else {Ior1[y][x]=255;}
      Ior2[y][x]=Imap[y][x].A;*/
      //Ior3[y][x]=(*imArg)[y][x].B;
      //Ior4[y][x]=Iseg[y][x];
  }
}
/*vpImageIo::writePNG(Ior1,"Itransl.png");
vpImageIo::writePNG(Ior2,"Idepth.png");
vpImageIo::writePNG(Ior3,"Isegdt.png");
vpImageIo::writePNG(Ior4,"Idot.png");*/
argDTTemp.push_back(imArg);
vpPixelMeterConversion::convertPoint(mcam,cog,xx,yy);
//delete Ior;
dataTemp[row][0] = cog.get_i();
dataTemp[row][1] = cog.get_j();
dataTemp[row][2] = ori;
dataTemp[row][3] = surface;
dataTemp[row][4] = xx*pose[2];
dataTemp[row][5] = yy*pose[2];
dataTemp[row][6] = pose[2];
dataTemp[row][7] = pose[3];
dataTemp[row][8] = pose[4];
dataTemp[row][9] = pose[5];
Ior0=*I00;
delete I00;
char buf[FILENAME_MAX];
sprintf(buf, opath.c_str(),row);
std::string filename(buf);
vpImageIo::writePNG(Ior0, filename);
/*Itex = *I01;
delete I01;
char buf01[FILENAME_MAX];
sprintf(buf01, opath01.c_str(),row);
std::string filename01(buf01);
vpImageIo::writePNG(Itex, filename01);*/
row++;
		}
		}
		}
ns = argDTTemp.size()-1;
nos=nrow;
noos=nbimg;
idx=(unsigned long *)calloc(ns+1,sizeof(unsigned long));
std::cout << "Build clusters region " << nregion << std::endl;
buildClusters(idx,1000,50,0.5);
mm=1;
for (int n = 0;n<ns;n++)
{
if(n==0)
{
	I0=argDTTemp[(int)idx[n]];
	I2 = new vpImage<vpRGBa>;
	I2->resize(height,width);
	*I2=*I0;
	mergeTemp.push_back(I2);
	hierarchy[n+nos][0]=level+1;
	hierarchy[n+nos][1]=n+nos+1;
	hierarchy[n+nos][2]=idx[n]+noos;
	dataTemp1[n+nos][0] = dataTemp[idx[n]+noos-1][0];
	dataTemp1[n+nos][1] = dataTemp[idx[n]+noos-1][1];
	dataTemp1[n+nos][2] = dataTemp[idx[n]+noos-1][2];
	dataTemp1[n+nos][3] = dataTemp[idx[n]+noos-1][3];
	dataTemp1[n+nos][4] = dataTemp[idx[n]+noos-1][4];
	dataTemp1[n+nos][5] = dataTemp[idx[n]+noos-1][5];
	dataTemp1[n+nos][6] = dataTemp[idx[n]+noos-1][6];
	dataTemp1[n+nos][7] = dataTemp[idx[n]+noos-1][7];
	dataTemp1[n+nos][8] = dataTemp[idx[n]+noos-1][8];
	dataTemp1[n+nos][9] = dataTemp[idx[n]+noos-1][9];
	nrow++;
	I.resize(I0->getHeight(),I0->getWidth());
	for (int y=0; y<I0->getHeight();y++)
			{
				for (int x=0;x<I0->getWidth();x++)
				{
			I[y][x]=(*I0)[y][x].A;
				}
			}
	sprintf(buf0, opath.c_str(), (level+1)*10000 + nos);
	std::string filename0(buf0);
	vpImageIo::writePNG(I, filename0);

}
else{
	flag=false;
for (int m=0;m<mm+nos;m++)
{
if (idx[n]+noos==hierarchy[m][2])
{
 flag=true;
}
}
if(flag==false)
{
	I0=argDTTemp[idx[n]];
	I5 = new vpImage<vpRGBa>;
	I5->resize(height,width);
	*I5=*I0;
	mergeTemp.push_back(I5);
	hierarchy[mm+nos][0]=level+1;
	hierarchy[mm+nos][1]=mm+nos+1;
	hierarchy[mm+nos][2]=idx[n]+noos;
	dataTemp1[mm+nos][0] = dataTemp[idx[n]+noos-1][0];
	dataTemp1[mm+nos][1] = dataTemp[idx[n]+noos-1][1];
	dataTemp1[mm+nos][2] = dataTemp[idx[n]+noos-1][2];
	dataTemp1[mm+nos][3] = dataTemp[idx[n]+noos-1][3];
	dataTemp1[mm+nos][4] = dataTemp[idx[n]+noos-1][4];
	dataTemp1[mm+nos][5] = dataTemp[idx[n]+noos-1][5];
	dataTemp1[mm+nos][6] = dataTemp[idx[n]+noos-1][6];
	dataTemp1[mm+nos][7] = dataTemp[idx[n]+noos-1][7];
	dataTemp1[mm+nos][8] = dataTemp[idx[n]+noos-1][8];
	dataTemp1[mm+nos][9] = dataTemp[idx[n]+noos-1][9];
	for (int y=0; y<I0->getHeight();y++)
	{
		for (int x=0;x<I0->getWidth();x++)
		{
	I[y][x]=(*I0)[y][x].A;
		}
	}
	sprintf(buf0, opath.c_str(), (level+1)*10000 + mm + nos);
	std::string filename0(buf0);
	vpImageIo::writePNG(I, filename0);

	/*for (int y=0; y<I0->getHeight();y++)
			{
				for (int x=0;x<I0->getWidth();x++)
				{
					if((*I0)[y][x].A != 100)
			I[y][x]=0;
					else I[y][x] = 255;
				}
			}
			sprintf(buf0, opath.c_str(), (level+4)*10000 + mm + nos);
			std::string filename000(buf0);
			vpImageIo::writePPM(I, filename000);*/

	mm++;
	nrow++;
}
}
nbimg++;
}
std::cout << "OK clustering regions" << std::endl;

for (int nu = 0;nu<mm;nu++)
{
int nn=3;
for (int kk=0;kk<ns;kk++)
{
if(idx[kk]+noos==hierarchy[nu+nos][2])
{
	hierarchy[nu+nos][nn]=kk+1+noos;
	nn++;
}

}
}
free (idx);
for (int ii = 0; ii < argDTTemp.size(); ii += 1){
			        I4 = argDTTemp[ii];
			        if (I4!=NULL) delete I4;
			        I4=NULL;
			 }
	    argDTTemp.resize(1);
}
}

dataTemp.resize(row,10,false);
dataTemp.saveMatrix(filename0,dataTemp,false);

for (int i = 0; i < mergeTemp.size()-1; i += 1){
//for (int i = 0; i < floor(mergeTemp.size()/10); i += 1){
	  I3=new vpImage<vpRGBa>;
	  I3->resize(height,width);
	  //vpTRACE("ok hierarchy1");
	  *I3=*mergeTemp[i+1];
	  argDTTemp.push_back(I3);
}
for (int i = 0; i < mergeTemp.size(); i += 1){
      mv0 = mergeTemp[i];
      if (mv0!=NULL) delete mv0;
      mv0=NULL;
	  //delete I2;
}
mergeTemp.resize(1);
level=1;
nos0=0;
std::cout << "First level built - Number of model views : " << argDTTemp.size() << std::endl;

while ( argDTTemp.size()>10)
{
ns= argDTTemp.size()-1;
nos=nrow;
idx=(unsigned long *)calloc(ns+1,sizeof(unsigned long));
std::cout << "Build clusters level " << level +1 << std::endl;
buildClusters(idx,1000,50,0.5);
mm=1;
//getchar();
for (int n = 0;n<ns;n++)
{
	if(n==0)
	{
		I0=argDTTemp[(int)idx[n]];
		I2 = new vpImage<vpRGBa>;
		I2->resize(height,width);
		*I2=*I0;
		mergeTemp.push_back(I2);
		hierarchy[n+nos][0]=level+1;
		hierarchy[n+nos][1]=mm;
		hierarchy[n+nos][2]=idx[n];
		dataTemp1[n+nos][0] = dataTemp1[idx[n]+nos0-1][0];
		dataTemp1[n+nos][1] = dataTemp1[idx[n]+nos0-1][1];
		dataTemp1[n+nos][2] = dataTemp1[idx[n]+nos0-1][2];
		dataTemp1[n+nos][3] = dataTemp1[idx[n]+nos0-1][3];
		dataTemp1[n+nos][4] = dataTemp1[idx[n]+nos0-1][4];
		dataTemp1[n+nos][5] = dataTemp1[idx[n]+nos0-1][5];
		dataTemp1[n+nos][6] = dataTemp1[idx[n]+nos0-1][6];
		dataTemp1[n+nos][7] = dataTemp1[idx[n]+nos0-1][7];
		dataTemp1[n+nos][8] = dataTemp1[idx[n]+nos0-1][8];
		dataTemp1[n+nos][9] = dataTemp1[idx[n]+nos0-1][9];
		nrow++;
		I.resize(I0->getHeight(),I0->getWidth());
		for (int y=0; y<I0->getHeight();y++)
				{
					for (int x=0;x<I0->getWidth();x++)
					{
				I[y][x]=(*I0)[y][x].A;
					}
				}
				sprintf(buf0, opath.c_str(), (level+1)*10000);
				std::string filename0(buf0);
				vpImageIo::writePNG(I, filename0);

	}
	else{
		flag=false;
	for (int m=0;m<mm+nos;m++)
	{
 if (idx[n]==hierarchy[m+nos][2])
 {
	 flag=true;
 }
	}
	if(flag==false)
	{
		//vpTRACE("");
		I0=argDTTemp[idx[n]];
		I5 = new vpImage<vpRGBa>;
		I5->resize(height,width);
		*I5=*I0;
		mergeTemp.push_back(I5);
		hierarchy[mm+nos][0]=level+1;
		hierarchy[mm+nos][1]=mm+1;
		hierarchy[mm+nos][2]=idx[n];
		dataTemp1[mm+nos][0] = dataTemp1[idx[n]+nos0-1][0];
		dataTemp1[mm+nos][1] = dataTemp1[idx[n]+nos0-1][1];
		dataTemp1[mm+nos][2] = dataTemp1[idx[n]+nos0-1][2];
		dataTemp1[mm+nos][3] = dataTemp1[idx[n]+nos0-1][3];
		dataTemp1[mm+nos][4] = dataTemp1[idx[n]+nos0-1][4];
		dataTemp1[mm+nos][5] = dataTemp1[idx[n]+nos0-1][5];
		dataTemp1[mm+nos][6] = dataTemp1[idx[n]+nos0-1][6];
		dataTemp1[mm+nos][7] = dataTemp1[idx[n]+nos0-1][7];
		dataTemp1[mm+nos][8] = dataTemp1[idx[n]+nos0-1][8];
		dataTemp1[mm+nos][9] = dataTemp1[idx[n]+nos0-1][9];
		//I.resize(I0->getHeight(),I0->getWidth());
		for (int y=0; y<I0->getHeight();y++)
		{
			for (int x=0;x<I0->getWidth();x++)
			{
		I[y][x]=(*I0)[y][x].A;
			}
		}
		sprintf(buf0, opath.c_str(), (level+1)*10000 + mm);
		std::string filename0(buf0);
		vpImageIo::writePNG(I, filename0);

		mm++;
		nrow++;
	}
}
}

for (int nu = 0;nu<mm;nu++)
{
int nn=3;
for (int kk=0;kk<ns;kk++)
{
	if(idx[kk]==hierarchy[nu+nos][2])
	{
		hierarchy[nu+nos][nn]=kk+1;
		nn++;
	}

}
}

free (idx);


for (int i = 0; i < argDTTemp.size(); i += 1){
			        I4 = argDTTemp[i];
			        if (I4!=NULL) delete I4;
			        I4=NULL;
			 }
	    argDTTemp.resize(1);

	  for (int i = 0; i < mergeTemp.size()-1; i += 1){
	  //for (int i = 0; i < floor(mergeTemp.size()/10); i += 1){
		  I3=new vpImage<vpRGBa>;
		  I3->resize(height,width);
		  *I3=*mergeTemp[i+1];
		  argDTTemp.push_back(I3);

	  }
	  cout << " size " <<argDTTemp.size()<< endl;
	  vpTRACE("ok hierarchy1");
	 for (int i = 0; i < mergeTemp.size(); i += 1){
		 vpTRACE("ok hierarchy1");
	        mv0 = mergeTemp[i];
	        if (mv0!=NULL) delete mv0;
	        mv0=NULL;
		  //delete I2;
	 }
	  mergeTemp.resize(1);

nos0=nos;
//if(level ==1)
level++;
std::cout << "Level " << level << " built - Number of model views : " << argDTTemp.size() << std::endl;
}
std::cout << "Model view graph built " << std::endl;

// Write data files
dataTemp1.resize(nrow,10,false);
dataTemp1.saveMatrix(filename1,dataTemp1,false);
transProb.saveMatrix(filenameTP,transProb,false);
hierarchy.resize(nrow,30,false);
hierarchy.saveMatrix(filename,hierarchy,false);
//file.close();



}


vpImage<vpRGBa>* apViews::downScale(int i, vpImage<vpRGBa> *_I)
{
    //vpImageIo::writePPM(*_I, "Iin.pgm");
    unsigned int cScale = static_cast<unsigned int>(pow(2., (int)(i)));
    //vpImage<vpRGBa>* I = new vpImage<vpRGBa>(_I->getHeight() / cScale, _I->getWidth() / cScale);
    vpImage<vpRGBa>* I = new vpImage<vpRGBa>(_I->getHeight()/2, _I->getWidth()/2);
    /*IplImage* vpI0 = cvCreateImageHeader(cvSize(_I->getWidth(), _I->getHeight()), IPL_DEPTH_8U, 4);
vpI0->imageData = (char*)(_I->bitmap);
IplImage* vpI = cvCreateImage(cvSize(_I->getWidth() / cScale, _I->getHeight() / cScale), IPL_DEPTH_8U, 4);
cvResize(vpI0, vpI, CV_INTER_NN);
vpImageConvert::convert(vpI, *I);
cvReleaseImage(&vpI);
vpI0->imageData = NULL;
cvReleaseImageHeader(&vpI0);
vpImage<unsigned char> I1(I->getHeight(),I->getWidth());
for (int y=0; y<I->getHeight();y++)
            {
                for (int x=0;x<I->getWidth();x++)
                {
            I1[y][x]=(*I)[y][x].A;
                }
            }*/

    for (unsigned int k = 0, ii = 0; k < I->getHeight(); k += 1, ii += 2){
        for (unsigned int l = 0, jj = 0; l < I->getWidth(); l += 1, jj += 2){
            //std::cout<< " ookii " << ii <<" ookjj " << jj << std::endl;
            (*I)[k][l].A = (*_I)[ii][jj].A;
            (*I)[k][l].R = (int)(((*_I)[ii][jj].R)/2);
            (*I)[k][l].G = (int)(((*_I)[ii][jj].G)/2);
            (*I)[k][l].B = (int)(((*_I)[ii][jj].B)/2);
        }
    }

    return I;
}

void apViews::downScale(vpImage<unsigned char> &_I, int i)
{
    //vpImageIo::writePPM(*_I, "Iin.pgm");
    unsigned int cScale = static_cast<unsigned int>(pow(2., (int)(i)));
    //vpImage<vpRGBa>* I = new vpImage<vpRGBa>(_I->getHeight() / cScale, _I->getWidth() / cScale);
    vpImage<unsigned char> I0(_I.getHeight()/cScale, _I.getWidth()/cScale);

    for (unsigned int k = 0, ii = 0; k < I0.getHeight(); k += 1, ii += cScale){
        for (unsigned int l = 0, jj = 0; l < I0.getWidth(); l += 1, jj += cScale){
            //std::cout<< " ookii " << ii <<" ookjj " << jj << std::endl;
            I0[k][l] = _I[ii][jj];
        }
    }
    _I.resize(I0.getHeight(), I0.getWidth());
    _I=I0;
    std::cout << " ok "<< std::endl;
    //vpImageIo::writePPM((*I), "Iout.pgm");
}

/*double apViews::computeSimilarityISC(vpImage<unsigned char> &I0, vpImage<unsigned char> &I0)
{
    //shape context
}*/

/*double apViews::computeSimilarityHint(vpImage<vpRGBa> *I0, vpImage<vpRGBa> *I1)
{
    //hinterstoisser
}*/



double apViews::computeSimilarity(vpImage<vpRGBa> *I0, vpImage<vpRGBa> *I1)
{

int width = I0->getWidth();
int height = I0->getHeight();
int m,mo,yy,xx, yarg, xarg, adtIy,adtIx;
double cor,t0,t1,dist, distrat, delta,oriT,oriI,dtI;
int offs =127;
int h2 = height/2;
int w2 = width/2;
t0= vpTime::measureTimeMs();
dist=0;
distrat = 0;
m=0;
mo=0;
for (int y = 0; y < height; y++) {
  for (int x = 0; x < width; x++) {
yarg = y-(((*I1)[y][x]).R-offs)*2;
xarg =  x-(((*I1)[y][x]).G-offs)*2;
	  if (((*I0)[y][x].A<100 || (*I0)[y][x].A>100) && yarg > 2 && yarg < height-2 &&  xarg > 2 &&  xarg < width-2)
    {
    	m++;
    	oriI = (double)(*I1)[yarg][xarg].A;
    	yy=-1;
    	xx=-1;
    	while (oriI==100 && yy<2)
    		{
    		oriI=(double)(*I1)[yarg+yy][xarg+xx].A;
    		yy ++;
			while ( oriI==100 && xx<2)
			{
				oriI=(double)(*I1)[yarg+yy][xarg+xx].A;
				xx ++;
			}
    		}
    	//(*I1)[y-(((*I1)[y][x]).R-offs)*2][x-(((*I1)[y][x]).G-offs)*2].A= oriI;
    	if (oriI>100 || oriI<100)
    		{
    		mo++;
    		delta=abs((*I0)[y][x].A-oriI);
    		//delta=abs((double)(*argIDT)[y][x].A-oriT);
    		if (delta>255-delta) delta=255-delta;
	   	//cout  << y << " " << coo << " " << orientT << " " << (double)(*I10)[y][x] << " " << x << endl;
    		//cout  <<" " << delta << endl;
    	//dist = dist + dtI + 0.1*delta;
    		dist = dist + (double)(*I1)[y][x].B + 0*delta;
    		//distrat = distrat + 100*(double)(*I1)[y][x].B/(sqrt((y-h2)*(y-h2)+(x-w2)*(x-w2))+2);
    		//distrat = distrat + abs(((y-h2)*(yarg-h2)+(x-w2)*(xarg-w2))) / (sqrt((yarg-h2)*(yarg-h2)+(xarg-w2)*(xarg-w2))*sqrt((y-h2)*(y-h2)+(x-w2)*(x-w2)));
    	//dist = dist  + 0.1*delta;
    		}
    	/*else
    	dist = dist + (ImDT)[y][x];*/
    }
  }
}

//cout << " distrat " << distrat/mo << endl;
dist=dist/mo;
//distrat=distrat/mo;
//distrat = 1-distrat / mo;
t1= vpTime::measureTimeMs();
//cout << " timem " << t1-t0 << endl;
return dist;
//return distrat;
}

double apViews::computeSimilarityScale(vpImage<vpRGBa> *I0, vpImage<vpRGBa> *I1, int scale)
{

int width = I0->getWidth();
int height = I0->getHeight();
int m,mo,yy,xx,adtIy,adtIx;
double cor,t0,t1,dist,delta,oriT,oriI,dtI;
unsigned int cScale = static_cast<unsigned int>(pow(2., (int)(scale)));
int offs = (int)(127/cScale);
//std::cout << "offs " << offs << std::endl;
t0= vpTime::measureTimeMs();
dist=0;
for (int y = 0; y < height; y++) {
  for (int x = 0; x < width; x++) {

	  if ((*I0)[y][x].A<100 || (*I0)[y][x].A>100)
    {
    	m++;
    	//oriI = (double)(*Im)[adtIy][adtIx];
    	oriI = (double)(*I1)[y-(((*I1)[y][x]).R-offs)*2][x-(((*I1)[y][x]).G-offs)*2].A;
    	yy=-1;
    	xx=-1;
    	while (oriI==100 && yy<2)
    		{
    		oriI=(double)(*I1)[y-(((*I1)[y][x]).R-offs)*2+yy][x-(((*I1)[y][x]).G-offs)*2 + xx].A;
    		yy ++;
			while ( oriI==100 && xx<2)
			{
				oriI=(double)(*I1)[y-(((*I1)[y][x]).R-offs)*2+yy][x-(((*I1)[y][x]).G-offs)*2 + xx].A;
				xx ++;
			}
    		}
    	(*I1)[y-(((*I1)[y][x]).R-offs)*2][x-(((*I1)[y][x]).G-offs)*2].A= oriI;
    	if (oriI>100 || oriI<100)
    		{
    		mo++;
    		delta=abs((*I0)[y][x].A-oriI);
    		//delta=abs((double)(*argIDT)[y][x].A-oriT);
    		if (delta>255-delta) delta=255-delta;
	   	//cout  << y << " " << coo << " " << orientT << " " << (double)(*I10)[y][x] << " " << x << endl;
    		//cout  <<" " << delta << endl;
    	//dist = dist + dtI + 0.1*delta;
    		dist = dist + (double)(*I1)[y][x].B + 0.05*delta;
    	//dist = dist  + 0.1*delta;
    		}
    	/*else
    	dist = dist + (ImDT)[y][x];*/
    }
  }
}

//cout << dist/mo << endl;
dist=dist/mo;
t1= vpTime::measureTimeMs();
//cout << " timem " << t1-t0 << endl;

return dist;
}


double apViews::computeSimilarity(vpImage<vpRGBa> *Im, vpImage<unsigned char> *IT)
{
int width = Im->getWidth();
int height = Im->getHeight();
int m,mo,yy,xx,adtIy,adtIx;
double cor,t0,t1,dist,delta,oriT,oriI,dtI;
int offs =(int) 127*((double)width/640);
//int offs =(int) 127;
t0= vpTime::measureTimeMs();
dist=0;
mo=0;
m=0;
for (int y = 0; y < height; y++) {
  for (int x = 0; x < width; x++) {

	  if ((*IT)[y][x]<100 || (*IT)[y][x]>100)
    {
		  //if ((double)(*Im)[y][x].B<255)


    	m++;
    	//oriI = (double)(*Im)[adtIy][adtIx];
    	oriI = (double)(*Im)[y-(((*Im)[y][x]).R-offs)*2][x-(((*Im)[y][x]).G-offs)*2].A;
    	yy=-1;
    	xx=-1;
    	while (oriI==100 && yy<2)
    		{
    		oriI=(double)(*Im)[y-(((*Im)[y][x]).R-offs)*2+yy][x-(((*Im)[y][x]).G-offs)*2 + xx].A;
    		yy ++;
			while ( oriI==100 && xx<2)
			{
				oriI=(double)(*Im)[y-(((*Im)[y][x]).R-offs)*2+yy][x-(((*Im)[y][x]).G-offs)*2 + xx].A;
				xx ++;
			}
    		}
    	(*Im)[y-(((*Im)[y][x]).R-offs)*2][x-(((*Im)[y][x]).G-offs)*2].A = oriI;
    	if (oriI>100 || oriI<100)
    		{
    		mo++;
    		delta=abs((*IT)[y][x]-oriI);
    		//delta=abs((double)(*argIDT)[y][x].A-oriT);
    		if (delta>255-delta) delta=255-delta;
	   	//cout  << y << " " << coo << " " << orientT << " " << (double)(*I10)[y][x] << " " << x << endl;
    		//cout  <<" " << delta << endl;
    	//dist = dist + dtI + 0.1*delta;
    		dist = dist + (double)(*Im)[y][x].B + 0*delta;
    		//cout << mo << endl;
    	//dist = dist  + 0.1*delta;
    		}
    	/*else
    	dist = dist + (ImDT)[y][x];*/

    }
  }
}

dist=dist/mo;
t1= vpTime::measureTimeMs();
//cout << " timem " << t1-t0 << endl;

return dist;
}

void apViews::computeSimilarityMatrix(vpMatrix &simmatrix, vpMatrix &preferences){

preferences.resize(argDTTemp.size()-1,1);
int nsize=(argDTTemp.size())*(argDTTemp.size()-1);
simmatrix.resize(300000,3);
vpImage<vpRGBa> *I0;
vpImage<vpRGBa> *I1;
vpImage<vpRGBa> *I2;
double sim0,sim1,sim;
int kk=0;
double med;
std::vector<double> simvect;
std::vector<double>::iterator first;
std::vector<double>::iterator last;
std::vector<double>::iterator middle;
cout << " ok " << endl;
for (int n = 0;n<argDTTemp.size()-1;n++)
{
	I0=argDTTemp[n+1];
	simvect.resize(0);
    //mm++;
	for (int k=0 ; k<argDTTemp.size()-1;k++)
	{
		//cout << " ok0 " << endl;
		if(k!=n){
			//cout << " ok1 " << endl;
		I1=argDTTemp[k+1];
		//cout << " ok2 " << I1->getHeight() << " "<<I1->getWidth() << endl;
		sim0 = computeSimilarity(I0,I1);
		sim1 = computeSimilarity(I1,I0);
		sim = 0.5*(sim0+sim1);
		//cout << " ok3 " << I1->getHeight() << " "<<I1->getWidth() << endl;
		simmatrix[kk][0] = n+1;
		simmatrix[kk][1] = k+1;
		//simmatrix[kk][2] = 150-sim;
		//simvect.push_back(150-sim);
		simmatrix[kk][2] = 50-sim;
		simvect.push_back(50-sim);
		if (50-sim<0){
			simmatrix[kk][2] = 50-sim;
			simvect.push_back(50-sim);
		}
		kk++;
		}
	}
    first=simvect.begin();
    last=simvect.end();
	std::sort(first,last);
    first=simvect.begin();
    last=simvect.end();
    middle = first + (last-first)/2;
    //std::nth_element(first, middle, last);
    med = (double)*middle;
    preferences[n][0]=med;
}
	simmatrix.resize(kk,3,false);
//simmatrix.saveMatrix(filename0,simmatrix,false);
//preferences.saveMatrix(filename1,preferences,false);
/*for (int i = 0; i < argDTTemp.size(); i += 1){
		        I2 = argDTTemp[i];
		        if (I2!=NULL) delete I2;
		        I2=NULL;
		 }*/
}

void apViews::loadViews(std::string opath)
{
const char *filename = "hierarchySoyuz.txt";
vpMatrix hierarchy;
hierarchy.loadMatrix(filename,hierarchy,false);
int level;
int i=hierarchy.getRows()-1;
level = hierarchy[i][0];
//views.resize(1);
vpImage<unsigned char> IT;
int j=1;
while(level>0)
{
std::vector<vpImage<unsigned char>*>* views = new std::vector<vpImage<unsigned char>*>;
views->resize(1);
while(hierarchy[i-j+1][0]==level){
	//while(j<64){
char buf[FILENAME_MAX];
sprintf(buf, opath.c_str(),level*1000+j-1);
std::string filename(buf);
vpImageIo::readPPM(IT, filename);
vpImage<unsigned char> *I00 = new vpImage<unsigned char>;
I00->resize(480,640);
*I00=IT;
views->push_back(I00);
j--;
delete I00;
}
Hviews.push_back(views);
delete views;
level--;
}
}

vpImage<vpRGBa>* apViews::dTO(vpImage<unsigned char> &I0)
{
vpImage<vpRGBa> *argImDT;
edgeOrientMap(I0);
vpImage<unsigned char> I1(I0.getHeight(),I0.getWidth());
/*  for (int x = 0; x < 640; x++) {
    for (int y = 0; y < 480; y++) {
    	if(x>=5 && x<635 && y>=5 && y<475-120)
    		I1[y][x] = I0[y][x];
    	else
    		I1[y][x] = 100;
    }
  }*/
for (int x = 0; x < I0.getWidth(); x++) {
    for (int y = 0; y < I0.getHeight(); y++) {
    	if(x>=5 && x<I0.getWidth()-5 && y>=5 && y<I0.getHeight()-5)
    		I1[y][x] = I0[y][x];
    	else
    		I1[y][x] = 100;
    }
  }
I0=I1;
//vpImageIo::writePNG(I0,"Iedg0.png");

argImDT=dt0(&I0);

for (int x = 0; x < I0.getWidth(); x++) {
    for (int y = 0; y < I0.getHeight(); y++) {
    	if(I0[y][x]!=100)
    		(*argImDT)[y][x].A = I0[y][x];
    	else
    		(*argImDT)[y][x].A = 100;
    }
  }


return argImDT;
}


void
apViews::initPyramid(vpImage<vpRGBa>* _I, std::vector< vpImage<vpRGBa>* >& _pyramid)
{
  _pyramid.resize(7);

  /*if(scales[0]){
    _pyramid[0] = &_I;
  }
  else{
    _pyramid[0] = NULL;
  }*/

  for(unsigned int i=1; i<_pyramid.size(); i += 1){
    //if(scales[i]){
      unsigned int cScale = static_cast<unsigned int>(pow(2., (int)i-1));
      vpImage<vpRGBa>* I = new vpImage<vpRGBa>(_I->getHeight() / cScale, _I->getWidth() / cScale);

      for (unsigned int k = 0, ii = 0; k < I->getHeight(); k += 1, ii += cScale){
        for (unsigned int l = 0, jj = 0; l < I->getWidth(); l += 1, jj += cScale){
          (*I)[k][l].A = (*_I)[ii][jj].A;
          (*I)[k][l].R = (int)(*_I)[ii][jj].R/cScale;
          (*I)[k][l].G = (int)(*_I)[ii][jj].G/cScale;
          (*I)[k][l].B = (int)(*_I)[ii][jj].B/cScale;

        }
      }
      _pyramid[i] = I;
    }
    /*else{
      _pyramid[i] = NULL;
    }*/
  //}
}

void apViews::rotateView(vpImage<unsigned char> &IT, vpImage<unsigned char> &Irot, double angle)
	{
	int height = IT.getHeight();
	int width = IT.getWidth();
	Irot.resize(height,width);
	double rho,theta;
	int xrot,yrot;
	double pi=3.1416;
	for (int y = 0; y < height; y++) {
		  for (int x = 0; x < width; x++) {
			  (Irot)[y][x]=100;
		  }
	}
	for (int y = 0; y < height; y++) {
	  for (int x = 0; x < width; x++) {
		  //(Irot)[y][x]=(IT)[y][x] - (int)255*(angle/M_PI);
		  if (((IT)[y][x]<100 || (IT)[y][x]>100) && x!=(int)width/2)
	    {
		rho = sqrt((y-(int)height/2)*(y-(int)height/2)+(x-(int)width/2)*(x-(int)width/2));
		theta = atan2(-(y-(int)height/2),(x-(int)width/2));
		theta = theta+angle;
		xrot = (int)(rho*cos(theta)+(int)width/2);
		yrot = (int)(-rho*sin(theta)+(int)height/2);
		  //cout << " xrot " << xrot << " yrot " << yrot << endl;
		if(xrot<width && xrot>0 && yrot>0 && yrot<height)
		{
			if((IT)[y][x]-255*(angle/M_PI)>0  && (IT)[y][x]-255*(angle/M_PI)<255 )
			{
		(Irot)[yrot][xrot]=(IT)[y][x] - (int)255*(angle/M_PI);
			}
			else if ((IT)[y][x]-255*(angle/M_PI)<0)
			{
		(Irot)[yrot][xrot]=	255 + (IT)[y][x] - (int)255*(angle/M_PI);
			}
			else if ((IT)[y][x]-255*(angle/M_PI)>255)
			{
		(Irot)[yrot][xrot]=	-255 + (IT)[y][x] - 255*(angle/M_PI);
			}
		}
	    }
	  }
	}
	}

void apViews::invert(vpImage<unsigned char> &IT, vpImage<unsigned char> &Irot)
	{
	int height = IT.getHeight();
	int width = IT.getWidth();
	Irot.resize(height,width);
	double rho,theta;
	int xrot,yrot;
	for (int y = 0; y < height; y++) {
		  for (int x = 0; x < width; x++) {
			  (Irot)[y][x]=IT[height-1-y][width-1-x];
		  }
	}
}

void apViews::invert90(vpImage<unsigned char> &IT, vpImage<unsigned char> &Irot)
	{
	int height = IT.getHeight();
	int width = IT.getWidth();
	Irot.resize(height,width);
	double rho,theta;
	int xrot,yrot;
	for (int y = 0; y < height; y++) {
		  for (int x = 0; x < width; x++) {
			  (Irot)[y][x]=IT[height-1-x][y];
		  }
	}
}

double apViews::computeSimilarityPos(vpImage<vpRGBa> *Im, vpImage<unsigned char> *IT, vpImagePoint pI)
{
		int width = Im->getWidth();
		int height = Im->getHeight();
		int m,mo,yy,xx,adtIy,adtIx;
		double cor,t0,t1,dist,delta,oriT,oriI,dtI;
		int offs =(int) 127*((double)width/640);
		//int offs =(int) 127;
		t0= vpTime::measureTimeMs();
		dist=0;
		mo=0;
		m=0;
		for (int y = 0; y < height; y++) {
		  for (int x = 0; x < width; x++) {

			  if (((*IT)[y][x]!=100) && y+(int)(pI.get_i()-height/2)>0 && y+(int)(pI.get_i()-height/2)<480 &&  x+(int)(pI.get_j()-width/2)>0 && x+(pI.get_j()-width/2)<640)
		    {
				  if(y-(((*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)]).R-offs)*2 >0 && y-(((*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)]).R-offs)*2 <480 && x-(int)(((*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)]).G-offs)*2 > 0 && x-(int)(((*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)]).G-offs)*2 <640)
				  {
				  /*cout << " h " <<pI.get_i()-height/2 << endl;
				  cout << " w " <<pI.get_j()-width/2 << endl;
				  getchar();*/
				  //if ((double)(*Im)[y][x].B<255)

		    	m++;
		    	//oriI = (double)(*Im)[adtIy][adtIx];
		    	oriI = (double)(*Im)[y-(((*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)]).R-offs)*2][x-(int)(((*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)]).G-offs)*2].A;
		    	yy=-1;
		    	xx=-1;
		    	while (oriI==100 && yy<2)
		    		{
		    		oriI=(double)(*Im)[y-(((*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)]).R-offs)*2+yy][x-(((*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)]).G-offs)*2 + xx].A;
		    		yy ++;
					while ( oriI==100 && xx<2)
					{
						oriI=(double)(*Im)[y-(((*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)]).R-offs)*2+yy][x-(((*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)]).G-offs)*2 + xx].A;
						xx ++;
					}
		    		}
		    	(*Im)[y-(((*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)]).R-offs)*2][x-(((*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)]).G-offs)*2].A = oriI;
		    	if (oriI>100 || oriI<100)
		    		{
		    		mo++;
		    		delta=abs((*IT)[y][x]-oriI);
		    		//delta=abs((double)(*argIDT)[y][x].A-oriT);
		    		if (delta>255-delta) delta=255-delta;
			   	//cout  << y << " " << coo << " " << orientT << " " << (double)(*I10)[y][x] << " " << x << endl;
		    		//cout  <<" " << delta << endl;
		    	//dist = dist + dtI + 0.1*delta;
		    		dist = dist + (double)(*Im)[y+(int)(pI.get_i()-height/2)][x+(int)(pI.get_j()-width/2)].B + 0.4*delta;
		    		//cout << mo << endl;
		    	//dist = dist  + 0.1*delta;
		    		}
		    	/*else
		    	dist = dist + (ImDT)[y][x];*/
		    }

		    }
		  }
		}

		dist=dist/mo;
		t1= vpTime::measureTimeMs();
		//cout << " timem " << t1-t0 << endl;

		return dist;
}

/*double apViews::computeSimilarityPosRot(vpImage<vpRGBa> *Im, vpImage<unsigned char> *IT, vpImagePoint &pI, const double Rot)
{
		int width = Im->getWidth();
		int height = Im->getHeight();
		int m,mo,yy,xx,adtIy,adtIx,xp,yp,xpim,ypim;
		double cor,t0,t1,dist,delta,oriT,oriI,dtI;
		int offs =(int) 127*((double)width/640);
		//int offs =(int) 127;
		t0= vpTime::measureTimeMs();
		dist=0;
		mo=0;
		m=0;

		for (int y = 0; y < height; y++) {
		  for (int x = 0; x < width; x++) {
			  yp = y+(int)(pI.get_i()-height/2);
			  xp = x+(int)(pI.get_j()-width/2);

			  if (((*IT)[y][x]!=100) && yp>0 && yp<480 && xp>0 && xp<640)
		    {
				  ypim = y-(int)(((*Im)[yp][xp]).R-offs)*2;
				  xpim = x-(int)(((*Im)[yp][xp]).G-offs)*2
				  if( ypim >0 && ypim <480 && xpim > 0 && xpim <640)
				  {

				  //if ((double)(*Im)[y][x].B<255)
		    	m++;
		    	//oriI = (double)(*Im)[adtIy][adtIx];
		    	oriI = (double)(*Im)[ypim][xpim].A;
		    	yy=-1;
		    	xx=-1;
		    	while (oriI==100 && yy<2)
		    		{
		    		oriI=(double)(*Im)[ypim + yy][xpim + xx].A;
		    		yy ++;
					while ( oriI==100 && xx<2)
					{
						oriI=(double)(*Im)[ypim + yy][xpim + xx].A;
						xx ++;
					}
		    		}
		    	(*Im)[ypim][xpim].A = oriI;
		    	if (oriI>100 || oriI<100)
		    		{
		    		mo++;
		    		delta=abs((*IT)[y][x]-oriI);
		    		//delta=abs((double)(*argIDT)[y][x].A-oriT);
		    		if (delta>255-delta) delta=255-delta;
			   	//cout  << y << " " << coo << " " << orientT << " " << (double)(*I10)[y][x] << " " << x << endl;
		    		//cout  <<" " << delta << endl;
		    	//dist = dist + dtI + 0.1*delta;
		    		dist = dist + (double)(*Im)[yp][xp].B + 0.4*delta;
		    		//cout << mo << endl;
		    	//dist = dist  + 0.1*delta;
		    		}
		    }

		    }
		  }
		}

		dist=dist/mo;
		t1= vpTime::measureTimeMs();
		//cout << " timem " << t1-t0 << endl;

		return dist;
}*/

double apViews::computeSimilarityPosRot(vpImage<vpRGBa> *Im, vpImage<unsigned char> *IT, vpImagePoint &pI, const double Rot)
{
		int width = Im->getWidth();
		int height = Im->getHeight();
		int m,mo,yy,xx,adtIy,adtIx,xp,yp,xpim,ypim,xrot,yrot;
		double cor,t0,t1,dist,delta,oriT,oriI,dtI,rho,theta;
		//int offs =(int) 127*((double)width/512);
		int offs =(int) 127;
		t0= vpTime::measureTimeMs();
		dist=0;
		mo=0;
		m=0;
		double ang;


		/*for (int y = 0; y < height; y++) {
			  for (int x = 0; x < width; x++) {
				  (Irot)[y][x]=100;
			  }
		}
		for (int y = 0; y < height; y++) {
		  for (int x = 0; x < width; x++) {
			  //(Irot)[y][x]=(IT)[y][x] - (int)255*(angle/M_PI);
			  if (((IT)[y][x]<100 || (IT)[y][x]>100) && x!=320)
		    {
			rho = sqrt((y-240)*(y-240)+(x-320)*(x-320));
			theta = atan2(-(y-240),(x-320));
			theta = theta+angle;
			xrot = (int)(rho*cos(theta)+320);
			yrot = (int)(-rho*sin(theta)+240);
			  //cout << " xrot " << xrot << " yrot " << yrot << endl;
			if(xrot<640 && xrot>0 && yrot>0 && yrot<480)
			{
				if((IT)[y][x]-255*(angle/M_PI)>0  && (IT)[y][x]-255*(angle/M_PI)<255 )
				{
			(Irot)[yrot][xrot]=(IT)[y][x] - (int)255*(angle/M_PI);
				}
				else if ((IT)[y][x]-255*(angle/M_PI)<0)
				{
			(Irot)[yrot][xrot]=	255 + (IT)[y][x] - (int)255*(angle/M_PI);
				}
				else if ((IT)[y][x]-255*(angle/M_PI)>255)
				{
			(Irot)[yrot][xrot]=	-255 + (IT)[y][x] - 255*(angle/M_PI);
				}
			}
		    }
		  }
		}*/


		for (int y = 0; y < height; y++) {
		  for (int x = 0; x < width; x++) {
			  if(((*IT)[y][x]!=100) && x!=(int)width/2)
			  {
			  rho = sqrt((y-(int)height/2)*(y-(int)height/2)+(x-(int)width/2)*(x-(int)width/2));
			  theta = atan2(-(y-(int)height/2),(x-(int)width/2));
		      theta = theta + Rot;
			  xrot = (int)(rho*cos(theta)+(int)width/2);
			  yrot = (int)(-rho*sin(theta)+(int)height/2);
			  yp = yrot+(int)(pI.get_i()-(int)height/2);
			  xp = xrot+(int)(pI.get_j()-(int)width/2);

			  if (yp>0 && yp<height && xp>0 && xp<width)
		    {
				  ypim = yrot-(int)(((*Im)[yp][xp]).R-offs)*2;
				  xpim = xrot-(int)(((*Im)[yp][xp]).G-offs)*2;
				  if( ypim >0 && ypim <height && xpim > 0 && xpim <width)
				  {
				  //if ((double)(*Im)[y][x].B<255)
		    	m++;
		    	//oriI = (double)(*Im)[adtIy][adtIx];

				if((*IT)[y][x]-255*(Rot/3.1416-floor(Rot/3.1416))>0  && (*IT)[y][x]-255*(Rot/3.1416-floor(Rot/3.1416))<255)
				{
			     ang = - (int)255*(Rot/3.1416-floor(Rot/3.1416));
				}
				else if ((*IT)[y][x]-255*(Rot/3.1416-floor(Rot/3.1416))<0)
				{
			     ang = 255 - (int)255*(Rot/3.1416-floor(Rot/3.1416));
				}
				else if ((*IT)[y][x]-255*(Rot/3.1416-floor(Rot/3.1416))>255)
				{
			     ang = -255 - 255*(Rot/3.1416-floor(Rot/3.1416));
				}
		    	oriI = (double)(*Im)[ypim][xpim].A;
		    	yy=-1;
		    	xx=-1;
		    	while (oriI==100 && yy<2)
		    		{


		    		oriI=(double)(*Im)[ypim + yy][xpim + xx].A + ang ;
		    		yy ++;
					while ( oriI==100 && xx<2)
					{
						oriI=(double)(*Im)[ypim + yy][xpim + xx].A + ang ;
						xx ++;
					}
		    		}
		    	(*Im)[ypim][xpim].A = oriI;
		    	if (oriI>100 || oriI<100)
		    		{
		    		mo++;
		    		delta=abs((*IT)[y][x]+ang-oriI);
		    		//delta=abs((double)(*argIDT)[y][x].A-oriT);
		    		if (delta>255-delta) delta=255-delta;
			   	//cout  << y << " " << coo << " " << orientT << " " << (double)(*I10)[y][x] << " " << x << endl;
		    	//dist = dist + dtI + 0.1*delta;
		    		dist = dist + (double)(*Im)[yp][xp].B + 0*delta;
		    	//dist = dist  + 0.1*delta;
		    		}
		    }
		    }
		  }
		  }
		}

		dist=dist/mo;
		t1= vpTime::measureTimeMs();

		return dist;
}

double apViews::computeSimilarityPosRotScale(vpImage<vpRGBa> *Im, vpImage<unsigned char> *IT, vpImagePoint &pI, const double Rot)
{
		int width = Im->getWidth();
		int height = Im->getHeight();
		int m,mo,yy,xx,adtIy,adtIx,xp,yp,xpim,ypim,xrot,yrot;
		double cor,t0,t1,dist,delta,oriT,oriI,dtI,rho,theta;
		//int offs =(int) 127*((double)width/512);
		int offs =(int) 127;
		t0= vpTime::measureTimeMs();
		dist=0;
		mo=0;
		m=0;
		double ang;
		double distrat = 0;
		int step = 10;
//#pragma omp parallel
		{
//#pragma omp	for private(x,y) shared(distrat)
		for (int y = 0; y < height; y++) {
		  for (int x = 0; x < width; x++) {
			  if(((*IT)[y][x]!=100) && x!=(int)width/2)// && ((double)y/step-(double)floor(y/step)<0.0005 || (double)x/step-(double)floor(x/step)<0.0005))
			  {
			  rho = sqrt((y-(int)height/2)*(y-(int)height/2)+(x-(int)width/2)*(x-(int)width/2));
			  theta = atan2(-(y-(int)height/2),(x-(int)width/2));
		      theta = theta + Rot;
			  xrot = (int)(rho*cos(theta)+(int)width/2);
			  yrot = (int)(-rho*sin(theta)+(int)height/2);
			  yp = yrot+(int)(pI.get_i()-(int)height/2);
			  xp = xrot+(int)(pI.get_j()-(int)width/2);

			  if (yp>0 && yp<height && xp>0 && xp<width)
		    {
				  ypim = yp-(int)(((*Im)[yp][xp]).R-offs)*2;
				  xpim = xp-(int)(((*Im)[yp][xp]).G-offs)*2;
				  if( ypim >0 && ypim <height && xpim > 0 && xpim <width)
				  {
				  //if ((double)(*Im)[y][x].B<255)
		    	//m++;
		    	//oriI = (double)(*Im)[adtIy][adtIx];

				if((*IT)[y][x]-255*(Rot/3.1416-floor(Rot/3.1416))>0  && (*IT)[y][x]-255*(Rot/3.1416-floor(Rot/3.1416))<255)
				{
			     ang = - (int)255*(Rot/3.1416-floor(Rot/3.1416));
				}
				else if ((*IT)[y][x]-255*(Rot/3.1416-floor(Rot/3.1416))<0)
				{
			     ang = 255 - (int)255*(Rot/3.1416-floor(Rot/3.1416));
				}
				else if ((*IT)[y][x]-255*(Rot/3.1416-floor(Rot/3.1416))>255)
				{
			     ang = -255 - 255*(Rot/3.1416-floor(Rot/3.1416));
				}

		    	oriI = (double)(*Im)[ypim][xpim].A;//+ang;
		    	yy=-1;
		    	xx=-1;
		    	while (oriI==100 && yy<2)
		    		{


		    		oriI=(double)(*Im)[ypim + yy][xpim + xx].A;// + ang ;
		    		yy ++;
					while ( oriI==100 && xx<2)
					{
						oriI=(double)(*Im)[ypim + yy][xpim + xx].A;// + ang ;
						xx ++;
					}
		    		}
		    	//(*Im)[ypim][xpim].A = oriI;
		    	if (oriI>100 || oriI<100)
		    		{
		    		mo++;
		    		//delta=abs((*IT)[y][x]+ang-oriI);

		    		delta=abs(-(*IT)[y][x]+ang-oriI+255);
			    		//delta=abs((double)(*argIDT)[y][x].A-oriT);
			    		if (delta>abs(255-delta)) delta=abs(255-delta);
		    		//delta=abs((double)(*argIDT)[y][x].A-oriT);
		    		//if (delta>255-delta) delta=255-delta;
			   	//cout  << y << " " << coo << " " << orientT << " " << (double)(*I10)[y][x] << " " << x << endl;
		    		//cout << " dist " << ((double)(*Im)[yp][xp].B) << " dist 2 " << sqrt((yp-pI.get_i())*(yp-pI.get_i())+(xp-pI.get_j())*(xp-pI.get_j())) << endl;
 		    	//dist = dist + dtI + 0.1*delta;
		    		//cout  << " dist  " << (double)(*Im)[yp][xp].B << " xp " << xp << " yp " << yp << " dist 2 " << sqrt(vpMath::sqr((int)(((*Im)[yp][xp]).R-offs)*2) + vpMath::sqr((int)(((*Im)[yp][xp]).G-offs)*2))  << endl;
                    //distrat = distrat + 100*abs(1-(sqrt((yp-pI.get_i())*(yp-pI.get_i())+(xp-pI.get_j())*(xp-pI.get_j())) / sqrt((ypim-pI.get_i())*(ypim-pI.get_i())+(xpim-pI.get_j())*(xpim-pI.get_j()))));
		    		distrat = distrat + muD*((double)(*Im)[yp][xp].B)/sqrt((yp-pI.get_i())*(yp-pI.get_i())+(xp-pI.get_j())*(xp-pI.get_j())) + lambdaO*0.1*delta;
		    		//distrat = distrat + 1000*abs(((yp-pI.get_i())*(ypim-pI.get_i())+(xp-pI.get_j())*(xpim-pI.get_j())) / (sqrt((ypim-pI.get_i())*(ypim-pI.get_i())+(xpim-pI.get_j())*(xpim-pI.get_j()))*sqrt((yp-pI.get_i())*(yp-pI.get_i())+(xp-pI.get_j())*(xp-pI.get_j()))));
		    		//dist = dist + (double)(*Im)[yp][xp].B + 0*delta;
		    	//dist = dist  + 0.1*delta;
		    		}
		    }
		    }
		  }
		  }
		}

}

		//distrat=1000-distrat/mo;
		distrat = distrat/mo;
		t1= vpTime::measureTimeMs();

		return distrat;
}

double apViews::computeSimilarityCP(vpImage<vpRGBa> *Im, std::vector<apContourPoint*> &ContourPoints, vpImagePoint &pI, const double Rot)
{
		int width = Im->getWidth();
		int height = Im->getHeight();
		int m,mo,y,x,yy,xx,adtIy,adtIx,xp,yp,xpim,ypim,xrot,yrot;
		double cor,t0,t1,dist,delta,oriT,oriI,dtI,rho,theta;
		//int offs =(int) 127*((double)width/512);
		int offs =(int) 127;
		t0= vpTime::measureTimeMs();
		dist=0;
		mo=0;
		m=0;
		double ang;
		double distrat = 0;
		int step = 1;
		apContourPoint *cpoint;
		//for (int cp=0;cp<floor((ContourPoints.size()-1)/step);cp++)
		for (int cp=0;cp<ContourPoints.size()-1;cp++)
		{
		//cpoint = ContourPoints[step*cp+1];
		cpoint = ContourPoints[cp+1];
		x = (int)cpoint->get_u();
		y = (int)cpoint->get_v();
		oriT = cpoint->get_ori();
			  rho = sqrt((y-(int)height/2)*(y-(int)height/2)+(x-(int)width/2)*(x-(int)width/2));
			  theta = atan2(-(y-(int)height/2),(x-(int)width/2))+Rot;
			  xrot = (int)(rho*cos(theta)+(int)width/2);
			  yrot = (int)(-rho*sin(theta)+(int)height/2);
			  yp = yrot+(int)(pI.get_i()-(int)height/2);
			  xp = xrot+(int)(pI.get_j()-(int)width/2);

			  if (yp>0 && yp<height && xp>0 && xp<width)
		    {
				  ypim = yp-(int)(((*Im)[yp][xp]).R-offs)*2;
				  xpim = xp-(int)(((*Im)[yp][xp]).G-offs)*2;
				  if( ypim >0 && ypim <height && xpim > 0 && xpim <width)
				  {
				  //if ((double)(*Im)[y][x].B<255)
		    	//m++;
		    	//oriI = (double)(*Im)[adtIy][adtIx];

				if(oriT-255*(Rot/3.1416-floor(Rot/3.1416))>0  && oriT-255*(Rot/3.1416-floor(Rot/3.1416))<255)
				{
			     ang = - (int)255*(Rot/3.1416-floor(Rot/3.1416));
				}
				else if (oriT-255*(Rot/3.1416-floor(Rot/3.1416))<0)
				{
			     ang = 255 - (int)255*(Rot/3.1416-floor(Rot/3.1416));
				}
				else if (oriT-255*(Rot/3.1416-floor(Rot/3.1416))>255)
				{
			     ang = -255 - 255*(Rot/3.1416-floor(Rot/3.1416));
				}

		    	oriI = (double)(*Im)[ypim][xpim].A;//+ang;
		    	yy=-1;
		    	xx=-1;
		    	while (oriI==100 && yy<2)
		    		{


		    		oriI=(double)(*Im)[ypim + yy][xpim + xx].A;// + ang ;
		    		yy ++;
					while ( oriI==100 && xx<2)
					{
						oriI=(double)(*Im)[ypim + yy][xpim + xx].A;// + ang ;
						xx ++;
					}
		    		}
		    	//(*Im)[ypim][xpim].A = oriI;
		    	if (oriI>100 || oriI<100)
		    		{
		    		mo++;
		    		delta=abs(oriT+ang-oriI);
		    		//delta=abs((double)(*argIDT)[y][x].A-oriT);
		    		if (delta>255-delta) delta=255-delta;
		    		distrat = distrat + muD*((double)(*Im)[yp][xp].B)/sqrt((yp-pI.get_i())*(yp-pI.get_i())+(xp-pI.get_j())*(xp-pI.get_j())) + lambdaO*delta;
		    		}
		    }
		    }

}
		distrat = distrat/mo;
		t1= vpTime::measureTimeMs();

		return distrat;
}

double apViews::computeSimilarityCPOpt(vpImage<vpRGBa> *Im, std::vector<apContourPoint*> &ContourPoints, vpImagePoint &pI, const double Rot)
{
		int width = Im->getWidth();
		int height = Im->getHeight();
		int hwidth = (int)width/2;
		int hheight = (int)height/2;
		int mo,y,x,yy,xx,xp,yp,xpim,ypim,xrot,yrot;
		double delta,oriT,oriI,rho,theta;
		int offs =(int) 127;
		mo=0;
		double ang;
		double distrat = 0;
		double Rot1 = 255*(Rot/3.1416-floor(Rot/3.1416));
		/*int xpI = pI.get_j();
		int ypI = pI.get_i();*/
		apContourPoint *cpoint;

		//for (int cp=0;cp<floor((ContourPoints.size()-1)/step);cp++)
		for (int cp=0;cp<ContourPoints.size()-1;cp++)
		{
		cpoint = ContourPoints[cp+1];
		x = (int)cpoint->get_u()-hwidth;
		y = (int)cpoint->get_v()-hheight;
		oriT = cpoint->get_ori();
			  rho = sqrt(y*y + x*x);
			  theta = atan2(-y,x)+Rot;
			  xrot = (int)(rho*cos(theta)+hwidth);
			  yrot = (int)(-rho*sin(theta)+hheight);
			  yp = yrot+(int)(pI.get_i()-hheight);
			  xp = xrot+(int)(pI.get_j()-hwidth);

			  if (yp>0 && yp<height && xp>0 && xp<width)
		    {
				  ypim = yp-(int)(((*Im)[yp][xp]).R-offs)*2;
				  xpim = xp-(int)(((*Im)[yp][xp]).G-offs)*2;
				  if( ypim >0 && ypim <height && xpim > 0 && xpim <width)
				  {
				  //if ((double)(*Im)[y][x].B<255)
		    	//m++;
		    	//oriI = (double)(*Im)[adtIy][adtIx];

				if(oriT-Rot1>0  && oriT-Rot1<255)
				{
			     ang = - Rot1;
				}
				else if (oriT-Rot1<0)
				{
			     ang = 255 - Rot1;
				}
				else if (oriT-Rot1>255)
				{
			     ang = -255 - Rot1;
				}

		    	oriI = (double)(*Im)[ypim][xpim].A;//+ang;
		    	yy=-1;
		    	xx=-1;
		    	while (oriI==100 && yy<2)
		    		{


		    		oriI=(double)(*Im)[ypim + yy][xpim + xx].A;// + ang ;
		    		yy ++;
					while ( oriI==100 && xx<2)
					{
						oriI=(double)(*Im)[ypim + yy][xpim + xx].A;// + ang ;
						xx ++;
					}
		    		}
		    	//(*Im)[ypim][xpim].A = oriI;
		    	if (oriI>100 || oriI<100)
		    		{
		    		mo++;
		    		delta=abs(oriT+ang-oriI);
		    		//delta=abs((double)(*argIDT)[y][x].A-oriT);
		    		if (delta>255-delta) delta=255-delta;
		    		distrat = distrat + muD*((double)(*Im)[yp][xp].B)/sqrt((yp-pI.get_i())*(yp-pI.get_i())+(xp-pI.get_j())*(xp-pI.get_j())) + lambdaO*delta;
		    		}
		    }
		    }

}
		distrat = distrat/mo;

		return distrat;
}


double apViews::computeSimilarityCPSOpt(vpImage<vpRGBa> *Im, std::vector<apContourPoint*> &ContourPoints, const int xpI, const int ypI, const double Rot, const double scale)
{
		int width = Im->getWidth();
		int height = Im->getHeight();
		int hwidth = (int)width/2;
		int hheight = (int)height/2;
		double y,x;
		int mo,yy,xx,xp,yp,xpim,ypim,xrot,yrot;
		double delta,oriT,oriI,rho,theta;
		//int offs =(int) 127*((double)width/512);
		int offs =(int) 127;
		mo=0;
		double ang;
		double distrat = 0;
		double Rot1 = 255*(Rot/3.1416-floor(Rot/3.1416));
		/*int xpI = pI.get_j();
		int ypI = pI.get_i();*/
		apContourPoint *cpoint;
		vpImage<unsigned char> Irot(height,width);

		//for (int cp=0;cp<floor((ContourPoints.size()-1)/step);cp++)
		for (int cp=0;cp<ContourPoints.size()-1;cp++)
		{
		cpoint = ContourPoints[cp+1];
		x = scale*(cpoint->get_u()-(double)hwidth);
		y = scale*(cpoint->get_v()-(double)hheight);
		oriT = cpoint->get_ori();
			  rho = sqrt(y*y + x*x);
			  theta = atan2(-y,x)+Rot;
			  xrot = (int)(rho*cos(theta)+(double)hwidth);
			  yrot = (int)(-rho*sin(theta)+(double)hheight);
			  yp = yrot+((int)ypI-hheight);
			  xp = xrot+((int)xpI-hwidth);
		    	/*if(yp == ypI && xp == xpI)
		    	{
		    	std::cout << " mo " << yp << " " << ((int)ypI-hheight) << " " << yrot << " " << theta << " " << rho << " " << y << " " << cpoint->get_v() << " " << cp << std::endl;
		    	}*/

			  if (yp>0 && yp<height && xp>0 && xp<width && yp!=ypI && xp!=xpI)
		    {
				  ypim = yp-(int)(((*Im)[yp][xp]).R-offs)*2;
				  xpim = xp-(int)(((*Im)[yp][xp]).G-offs)*2;
				  if( ypim >0 && ypim <height && xpim > 0 && xpim <width)
				  {
				  //if ((double)(*Im)[y][x].B<255)
		    	//m++;
		    	//oriI = (double)(*Im)[adtIy][adtIx];

				if(oriT-Rot1>0  && oriT-Rot1<255)
				{
			     ang = - Rot1;
				}
				else if (oriT-Rot1<0)
				{
			     ang = 255 - Rot1;
				}
				else if (oriT-Rot1>255)
				{
			     ang = -255 - Rot1;
				}

		    	oriI = (double)(*Im)[ypim][xpim].A;//+ang;

		    	//std::cout << " ypim " << ypim << " xpim " << xpim << " ori " << oriI << std::endl;
		    	yy=-1;
		    	xx=-1;
		    	while (oriI==100 && yy<2)
		    		{


		    		oriI=(double)(*Im)[ypim + yy][xpim + xx].A;// + ang ;
		    		yy ++;
					while ( oriI==100 && xx<2)
					{
						oriI=(double)(*Im)[ypim + yy][xpim + xx].A;// + ang ;
						xx ++;
					}
		    		}

		    	//Irot[yp][xp] = -oriT+ang;
		    	//(*Im)[ypim][xpim].A = oriI;
		    	if (oriI!=100)
		    		{
		    		mo++;
		    		//std::cout << " orit " << -oriT+ang << " ang " << ang << std::endl;
		    		delta=abs(-oriT+ang-oriI+255);
		    		//delta=abs((double)(*argIDT)[y][x].A-oriT);
		    		if (delta>abs(255-delta)) delta=abs(255-delta);
		    		distrat = distrat + muD*((double)(*Im)[yp][xp].B)/sqrt((yp-ypI)*(yp-ypI)+(xp-xpI)*(xp-xpI)) + lambdaO*delta;
		    		}
		    }
		    }

}

		//std::cout << " mo " << mo << " " << (double)(*Im)[100][100].B << std::endl;
		distrat = distrat/mo;

		/*if (xpI>180 && ypI > 140)
				{
				vpImageIo::writePNG(Irot, "Irot01.png");
				getchar();
				}*/

		return distrat;
}


double apViews::computeSimilarityCPSOptPartial(vpImage<vpRGBa> *Im, std::vector<apContourPoint*> &ContourPoints, const int xpI, const int ypI, const double Rot, const double scale)
{
		int width = Im->getWidth();
		int height = Im->getHeight();
		int hwidth = (int)width/2;
		int hheight = (int)height/2;
		double y,x;
		int mo,yy,xx,xp,yp,xpim,ypim,xrot,yrot;
		double delta,oriT,oriI,rho,theta;
		//int offs =(int) 127*((double)width/512);
		int offs =(int) 127;
		mo=0;
		double ang;
		double distrat = 0;
		double Rot1 = 255*(Rot/3.1416-floor(Rot/3.1416));
		/*int xpI = pI.get_j();
		int ypI = pI.get_i();*/
		apContourPoint *cpoint;
		std::vector<double> distances;
		distances.resize(0);

		//for (int cp=0;cp<floor((ContourPoints.size()-1)/step);cp++)
		for (int cp=0;cp<ContourPoints.size()-1;cp++)
		{
		cpoint = ContourPoints[cp+1];
		x = scale*(cpoint->get_u()-(double)hwidth);
		y = scale*(cpoint->get_v()-(double)hheight);
		oriT = cpoint->get_ori();
			  rho = sqrt(y*y + x*x);
			  theta = atan2(-y,x)+Rot;
			  xrot = (int)(rho*cos(theta)+(double)hwidth);
			  yrot = (int)(-rho*sin(theta)+(double)hheight);
			  yp = yrot+((int)ypI-hheight);
			  xp = xrot+((int)xpI-hwidth);
		    	/*if(yp == ypI && xp == xpI)
		    	{
		    	std::cout << " mo " << yp << " " << ((int)ypI-hheight) << " " << yrot << " " << theta << " " << rho << " " << y << " " << cpoint->get_v() << " " << cp << std::endl;
		    	}*/

			  if (yp>0 && yp<height && xp>0 && xp<width && yp!=ypI && xp!=xpI)
		    {
				  ypim = yp-(int)(((*Im)[yp][xp]).R-offs)*2;
				  xpim = xp-(int)(((*Im)[yp][xp]).G-offs)*2;
				  if( ypim >0 && ypim <height && xpim > 0 && xpim <width)
				  {
				  //if ((double)(*Im)[y][x].B<255)
		    	//m++;
		    	//oriI = (double)(*Im)[adtIy][adtIx];

				if(oriT-Rot1>0  && oriT-Rot1<255)
				{
			     ang = - Rot1;
				}
				else if (oriT-Rot1<0)
				{
			     ang = 255 - Rot1;
				}
				else if (oriT-Rot1>255)
				{
			     ang = -255 - Rot1;
				}

		    	oriI = (double)(*Im)[ypim][xpim].A;//+ang;
		    	yy=-1;
		    	xx=-1;
		    	while (oriI==100 && yy<2)
		    		{


		    		oriI=(double)(*Im)[ypim + yy][xpim + xx].A;// + ang ;
		    		yy ++;
					while ( oriI==100 && xx<2)
					{
						oriI=(double)(*Im)[ypim + yy][xpim + xx].A;// + ang ;
						xx ++;
					}
		    		}
		    	//(*Im)[ypim][xpim].A = oriI;
		    	if (oriI>100 || oriI<100)
		    		{
		    		mo++;
		    		delta=abs(oriT+ang-oriI);
		    		//delta=abs((double)(*argIDT)[y][x].A-oriT);
		    		if (delta>255-delta) delta=255-delta;

		    		//distrat = distrat + muD*((double)(*Im)[yp][xp].B)/sqrt((yp-ypI)*(yp-ypI)+(xp-xpI)*(xp-xpI)) + lambdaO*delta;
		    		distances.push_back(muD*((double)(*Im)[yp][xp].B)/sqrt((yp-ypI)*(yp-ypI)+(xp-xpI)*(xp-xpI)) + lambdaO*delta);


		    		}
		    }
		    }

}
		std::sort(distances.begin(), distances.begin()+(int)ContourPoints.size()/2);
		for (std::vector<double>::iterator it=distances.begin(); it!=distances.begin()+(int)ContourPoints.size()/2; ++it)
			distrat+= *it;
		//std::cout << " mo " << mo << " " << (double)(*Im)[100][100].B << std::endl;
		distrat = distrat/mo;

		return distrat;
}


double apViews::computeSimilaritySteger(vpImage<unsigned char> &Igrad, std::vector<apContourPoint*> &ContourPoints, const int xpI, const int ypI, const double Rot, const double scale)
{
		int width = Igrad.getWidth();
		int height = Igrad.getHeight();
		int hwidth = (int)width/2;
		int hheight = (int)height/2;
		double y,x;
		int mo,yy,xx,xp,yp,xpim,ypim,xrot,yrot;
		double delta,oriT,oriI,rho,theta;
		//int offs =(int) 127*((double)width/512);
		int offs =(int) 127;
		mo=0;
		double ang;
		double distrat = 0;
		double Rot1 = 255*(Rot/3.1416-floor(Rot/3.1416));
		/*int xpI = pI.get_j();
		int ypI = pI.get_i();*/
		apContourPoint *cpoint;
		vpImage<unsigned char> Irot(height,width);

		//for (int cp=0;cp<floor((ContourPoints.size()-1)/step);cp++)
		for (int cp=0;cp<ContourPoints.size()-1;cp++)
		{
		cpoint = ContourPoints[cp+1];
		x = scale*(cpoint->get_u()-(double)hwidth);
		y = scale*(cpoint->get_v()-(double)hheight);
		oriT = cpoint->get_ori();
			  rho = sqrt(y*y + x*x);
			  theta = atan2(-y,x)+Rot;
			  xrot = (int)(rho*cos(theta)+(double)hwidth);
			  yrot = (int)(-rho*sin(theta)+(double)hheight);
			  yp = yrot+((int)ypI-hheight);
			  xp = xrot+((int)xpI-hwidth);
		    	/*if(yp == ypI && xp == xpI)
		    	{
		    	std::cout << " mo " << yp << " " << ((int)ypI-hheight) << " " << yrot << " " << theta << " " << rho << " " << y << " " << cpoint->get_v() << " " << cp << std::endl;
		    	}*/

			  if (yp>0 && yp<height && xp>0 && xp<width && yp!=ypI && xp!=xpI)
		    {
				  //ypim = yp-(int)(((*Im)[yp][xp]).R-offs)*2;
				  //xpim = xp-(int)(((*Im)[yp][xp]).G-offs)*2;
				  //if( ypim >0 && ypim <height && xpim > 0 && xpim <width)
				  {
				  //if ((double)(*Im)[y][x].B<255)
		    	//m++;
		    	//oriI = (double)(*Im)[adtIy][adtIx];

				if(oriT-Rot1>0  && oriT-Rot1<255)
				{
			     ang = - Rot1;
				}
				else if (oriT-Rot1<0)
				{
			     ang = 255 - Rot1;
				}
				else if (oriT-Rot1>255)
				{
			     ang = -255 - Rot1;
				}

		    	oriI = (double)Igrad[yp][xp];//+ang;
		    	Irot[yp][xp] = (-oriT + ang);
		    	yy=-1;
		    	xx=-1;
		    	/*while (oriI==100 && yy<2)
		    		{


		    		oriI=(double)(*Im)[ypim + yy][xpim + xx].A;// + ang ;
		    		yy ++;
					while ( oriI==100 && xx<2)
					{
						oriI=(double)(*Im)[ypim + yy][xpim + xx].A;// + ang ;
						xx ++;
					}
		    		}*/
		    	//(*Im)[ypim][xpim].A = oriI;
		    	if (oriI!= 100)
		    		{
		    		mo++;
		    		delta=abs(255+(-oriT+ang-oriI));
		    		if (delta>abs(255-delta)) delta=abs(255-delta);
		    		//std::cout << " delta 1 " << abs(oriT+ang-oriI) << " delta 2 " << delta << std::endl;
		    		//delta=abs((double)(*argIDT)[y][x].A-oriT);
		    		//if (delta>abs(255-delta)) delta=abs(255-delta);
		    		distrat += lambdaO*delta;
		    		//distrat += lambdaO*cos((delta/255.0)*3.1416);
		    		}
		    }
		    }

}

		//std::cout << " mo " << mo << " " << (double)(*Im)[100][100].B << std::endl;
		distrat = distrat/mo;
		/*if (xpI>170 && ypI > 140)
		{
		vpImageIo::writePNG(Irot, "Irot00.png");
		getchar();
		}*/

		return distrat;
}



std::vector<double> apViews::computeSC(vpImage<unsigned char> &Iseg, vpImage<unsigned char> &IsegEdge, vpImagePoint &cog, double &angle, int &surface, int nr, int nw)
		{
	vpImage<vpRGBa> Ioverlay;
	vpImage<unsigned char> Idot(Iseg.getHeight(),Iseg.getWidth());
	vpImage<unsigned char> I1(Iseg.getHeight(),Iseg.getWidth());
	std::vector<double> logHist;
	apLogPolarHist lgpHist;
        //vpDisplayX display1;
	/*display1.init(I1, 800, 10, "Dots");
	vpDisplay::display(I1);*/
	vpImagePoint ip;
	for (int n=0; n < Iseg.getHeight() ; n++)
	{
	for (int m=0 ; m < Iseg.getWidth(); m++)
	  {
		if(Iseg[n][m]>0)
	    Idot[n][m]=255;
		else{
			Idot[n][m]=0;
		}
	  }
	}
	edgeOrientMap(Iseg);
    //vpImageIo::writePNG(Iseg, "Iseg0.png");
	double opt_sizePrecision = 0;
	double opt_grayLevelPrecision = 0.9;
	double opt_ellipsoidShapePrecision = 0;
	vpDot2 d;
	d.setGraphics(true);
	d.setWidth(50.0);
	d.setHeight(50.0);
        d.setArea(5000);
	d.setGrayLevelMin(200);
	d.setGrayLevelMax(255);
	d.setGrayLevelPrecision(opt_grayLevelPrecision);
	d.setSizePrecision(opt_sizePrecision);
	d.setEllipsoidShapePrecision(opt_ellipsoidShapePrecision);
	d.setComputeMoments(true);


	int surface0,sumsurface;
	vpImagePoint cog0,cogmean;
	double angle0,anglemean, mu11mean,mu20mean, mu02mean,m11s,m20s,m02s;
	surface = 0;
	angle = 0;
	sumsurface = 0;
	cogmean.set_u(0);
	cogmean.set_v(0);
	anglemean = 0;
	mu11mean = 0;
	mu20mean = 0;
	mu02mean = 0;
	m11s = 0;
	m20s = 0;
	m02s = 0;


        //vpDisplay::display(Idot);
	vpList<vpDot2>* list_d;
	vpDot2 dotB;
	dotB.setGraphics(true);
	std::vector<vpImagePoint> dotegdes;
	vpImagePoint topLeft;
	vpImagePoint bottomRight;
	std::vector<double> xx;
	std::vector<double> yy;
	vpRect rectdot;
	//list_d = d.searchDotsInArea(Idot, 0, 0, Iseg.getWidth(), Iseg.getHeight());

	    if( list_d->nbElement() == 0 )  {
	      std::cout << "Dot auto detection did not work." << std::endl;
	      //return ;
	    }
	    else {
	      std::cout << std::endl << list_d->nbElement() << " dots are detected" << std::endl;
	  {
		int i=0;
	        // Parse all founded dots for display
        list_d->front();
        while (!list_d->outside()) {
          vpDot2 tmp_d;
          tmp_d.setComputeMoments(true);
          tmp_d.setGraphics(true);
          tmp_d = list_d->value();
          tmp_d.track(Idot);
          //tmp_d.display(Idot,vpColor::red,2);
          double theta;
          surface0 = tmp_d.getArea();
	  if(surface0>500)
	  sumsurface = sumsurface + surface0;

          list_d->next();
        }

	        list_d->front();
	        while (!list_d->outside()) {
	          vpDot2 tmp_d;
	          tmp_d.setComputeMoments(true);
	          tmp_d.setGraphics(true);
	          tmp_d = list_d->value();
	          //tmp_d.track(Idot);
	          double theta;
                  surface0 = tmp_d.getArea();
		  cog0 = tmp_d.getCog();
		  angle0 = 0.5*atan2(2*tmp_d.mu11,(tmp_d.mu20-tmp_d.mu02));
		  if(surface0>500)
		  {
		  cout << " Dot of the segmented image : "  <<  " COG : " << cog0.get_u() << " " << cog0.get_v() << " Angle : " << angle0 << " Surface : " << surface0 << endl;
		  m11s = m11s + tmp_d.m11;//*((double)surface0/sumsurface);
		  m20s = m20s + tmp_d.m20;//*((double)surface0/sumsurface);
		  m02s = m02s + tmp_d.m02;//*((double)surface0/sumsurface);
		  cogmean.set_u(cogmean.get_u()+cog0.get_u()*((double)surface0/sumsurface));
		  cogmean.set_v(cogmean.get_v()+(double)cog0.get_v()*((double)surface0/sumsurface));
		  //anglemean = anglemean + angle0*((double)surface0/sumsurface);
			  rectdot = tmp_d.getBBox();
			  topLeft = rectdot.getTopLeft();
			  bottomRight = rectdot.getBottomRight();
			  //cout << " TopLeft : "  <<  topLeft << " Rightbottom " << bottomRight  << endl;
				for (int n=0; n < Iseg.getHeight() ; n++)
				{
				for (int m=0 ; m < Iseg.getWidth(); m++)
				  {
					if(Iseg[n][m]!=100)
						if(n>topLeft.get_i()-2 && n<bottomRight.get_i()+2 && m<bottomRight.get_j()+2 && m>topLeft.get_j()-2){
							xx.push_back((double)m);
							yy.push_back((double)n);
							/*ip.set_j(m);
							ip.set_i(n);
							vpDisplay::displayCross(I1,ip,2,vpColor::green,2);*/
						}
				  }
				}
				//vpImageIo::writePNG(Iseg, "Iseg0.pgm");
				/*vpDisplay::flush(I1);
				getchar();*/

		  }

	          list_d->next();
	          surface = sumsurface;
	          mu11mean = (double)m11s/sumsurface-cogmean.get_u()*cogmean.get_v();
	          mu20mean = (double)m20s/sumsurface-cogmean.get_u()*cogmean.get_u();
	          mu02mean = (double)m02s/sumsurface-cogmean.get_v()*cogmean.get_v();
	          angle = 0.5*atan2(2*mu11mean,(mu20mean-mu02mean));
	          //angle = 1.45;
	          cog = cogmean;
	        }
	      }
	    }
	    /*vpDisplay::displayCross(Idot, cog, 20, vpColor::green) ;
	    dotB.display(Idot,vpColor::green,2);*/
	    //angle = angle;
	    //cout << " Dot of the segmented image : "  <<  " COG : " << cog0.get_u() << " " << cog0.get_v() << " Angle : " << angle << " Surface : " << surface << endl;
	    logHist = lgpHist.RunCenter(xx,yy,cog,nr,nw);
	    lgpHist.SaveToFile("Iseghist.text", logHist);
	    // free memory allocated for the list of dots found in d.searchDotsInArea()
	    list_d->kill();
	    delete list_d;
            //vpDisplay::flush(Idot);
	    /*vpDisplay::getImage(I1,Ioverlay);
	    vpImageIo::writePPM(I1, "Iseg0.pgm");*/
	    return logHist;
		}

vpImage<vpRGBa>* apViews::dtOSeg(vpImage<unsigned char> &Iseg)
		{
	vpImage<vpRGBa> Ioverlay;
	vpImage<vpRGBa>* argImDT;
	vpImage<unsigned char> Idot(Iseg.getHeight(),Iseg.getWidth());
	vpImage<unsigned char> I1(Iseg.getHeight(),Iseg.getWidth());
	std::vector<double> logHist;
	apLogPolarHist lgpHist;
        //vpDisplayX display1;
	/*display1.init(I1, 800, 10, "Dots");
	vpDisplay::display(I1);*/
	vpImagePoint ip;
	for (int n=0; n < Iseg.getHeight() ; n++)
	{
	for (int m=0 ; m < Iseg.getWidth(); m++)
	  {
		I1[n][m] = 100;
		if(Iseg[n][m]>0)
	    Idot[n][m]=255;
		else{
			Idot[n][m]=0;
			Iseg[n][m] = 255;
		}
	  }
	}
	vpImageIo::writePNG(Iseg, "Iseg0.png");
	edgeOrientMap(Iseg);
	double opt_sizePrecision = 0;
	double opt_grayLevelPrecision = 0.9;
	double opt_ellipsoidShapePrecision = 0;
	vpDot2 d;
	d.setGraphics(true);
	d.setWidth(50.0);
	d.setHeight(50.0);
        d.setArea(5000);
	d.setGrayLevelMin(200);
	d.setGrayLevelMax(255);
	d.setGrayLevelPrecision(opt_grayLevelPrecision);
	d.setSizePrecision(opt_sizePrecision);
	d.setEllipsoidShapePrecision(opt_ellipsoidShapePrecision);
	d.setComputeMoments(true);


	int surface0,sumsurface,surface;
	vpImagePoint cog0,cogmean;
	double angle0,angle,anglemean, mu11mean,mu20mean, mu02mean,m11s,m20s,m02s;
	surface = 0;
	angle = 0;
	sumsurface = 0;
	cogmean.set_u(0);
	cogmean.set_v(0);
	anglemean = 0;
	mu11mean = 0;
	mu20mean = 0;
	mu02mean = 0;
	m11s = 0;
	m20s = 0;
	m02s = 0;


        //vpDisplay::display(Idot);
	vpList<vpDot2>* list_d;
	vpDot2 dotB;
	dotB.setGraphics(true);
	std::vector<vpImagePoint> dotegdes;
	vpImagePoint topLeft;
	vpImagePoint bottomRight;
	std::vector<double> xx;
	std::vector<double> yy;
	vpRect rectdot;
	//list_d = d.searchDotsInArea(Idot, 0, 0, Iseg.getWidth(), Iseg.getHeight());

	    if( list_d->nbElement() == 0 )  {
	      std::cout << "Dot auto detection did not work." << std::endl;
	      //return ;
	    }
	    else {
	      std::cout << std::endl << list_d->nbElement() << " dots are detected " << std::endl;
	  {
		int i=0;
	        // Parse all founded dots for display
        list_d->front();
        while (!list_d->outside()) {
          vpDot2 tmp_d;
          tmp_d.setComputeMoments(true);
          tmp_d.setGraphics(true);
          tmp_d = list_d->value();
          tmp_d.track(Idot);
          //tmp_d.display(Idot,vpColor::red,2);
          double theta;
          surface0 = tmp_d.getArea();
	  if(surface0>100)
	  sumsurface = sumsurface + surface0;

          list_d->next();
        }

	        list_d->front();
	        while (!list_d->outside()) {
	          vpDot2 tmp_d;
	          tmp_d.setComputeMoments(true);
	          tmp_d.setGraphics(true);
	          tmp_d = list_d->value();
	          //tmp_d.track(Idot);
	          double theta;
                  surface0 = tmp_d.getArea();
		  cog0 = tmp_d.getCog();
		  //tmp_d.track(Idot,cog0);
		  angle0 = 0.5*atan2(2*tmp_d.mu11,(tmp_d.mu20-tmp_d.mu02));
		  if(surface0>100)
		  {
		  cout << " Dot of the segmented image aa: "  <<  " COG : " << cog0.get_u() << " " << cog0.get_v() << " Angle : " << angle0 << " Surface : " << surface0 << endl;
		  m11s = m11s + tmp_d.m11;//*((double)surface0/sumsurface);
		  m20s = m20s + tmp_d.m20;//*((double)surface0/sumsurface);
		  m02s = m02s + tmp_d.m02;//*((double)surface0/sumsurface);
		  cogmean.set_u(cogmean.get_u()+cog0.get_u()*((double)surface0/sumsurface));
		  cogmean.set_v(cogmean.get_v()+(double)cog0.get_v()*((double)surface0/sumsurface));
		  //anglemean = anglemean + angle0*((double)surface0/sumsurface);
			  rectdot = tmp_d.getBBox();
			  topLeft = rectdot.getTopLeft();
			  bottomRight = rectdot.getBottomRight();
			  //cout << " TopLeft : "  <<  topLeft << " Rightbottom " << bottomRight  << endl;
				for (int n=0; n < Iseg.getHeight() ; n++)
				{
				for (int m=0 ; m < Iseg.getWidth(); m++)
				  {
					if(Iseg[n][m]!=100)
						if(n>topLeft.get_i()-10 && n<bottomRight.get_i()+10 && m<bottomRight.get_j()+10 && m>topLeft.get_j()-10){
							I1[n][m] = Iseg[n][m];
							/*ip.set_j(m);
							ip.set_i(n);
							vpDisplay::displayCross(I1,ip,2,vpColor::green,2);*/
						}
						/*else{
							I1[n][m] = 100;}*/
					/*else
					{I1[n][m] = 100;}*/
				  }
				}
				//vpImageIo::writePNG(Iseg, "Iseg0.pgm");
				/*vpDisplay::flush(I1);
				getchar();*/

		  }

	          list_d->next();
	          /*surface = sumsurface;
	          mu11mean = (double)m11s/sumsurface-cogmean.get_u()*cogmean.get_v();
	          mu20mean = (double)m20s/sumsurface-cogmean.get_u()*cogmean.get_u();
	          mu02mean = (double)m02s/sumsurface-cogmean.get_v()*cogmean.get_v();
	          angle = 0.5*atan2(2*mu11mean,(mu20mean-mu02mean));
	          //angle = 1.45;
	          cog = cogmean;*/
	        }
	      }
	    }
	    //vpImageIo::writePNG(I1,"IedgSeg.png");
	    argImDT=dt0(&I1);
	    return argImDT;
		}

vpImage<vpRGBa>* apViews::dtOSeg2(vpImage<unsigned char> &Iseg)
		{
	vpImage<vpRGBa> Ioverlay;
	vpImage<vpRGBa>* argImDT;
	vpImage<unsigned char> Idot(Iseg.getHeight(),Iseg.getWidth());
	//vpImage<unsigned char> I1(Iseg.getHeight(),Iseg.getWidth());
	vpImagePoint ip;
	/*for (int n=0; n < Iseg.getHeight() ; n++)
	{
	for (int m=0 ; m < Iseg.getWidth(); m++)
	  {
		I1[n][m] = 100;
		if(Iseg[n][m]>0)
	    Idot[n][m]=255;
		else{
			Idot[n][m]=0;
			Iseg[n][m] = 255;
		}
	  }
	}*/
	for (int n=0; n < Iseg.getHeight() ; n++)
		{
		for (int m=0 ; m < Iseg.getWidth(); m++)
		  {
			if(Iseg[n][m] == 0)
			{
				Iseg[n][m] = 255;
			}
		  }
		}
	edgeOrientMap(Iseg);

	/*for (int n=0; n < Iseg.getHeight() ; n++)
		{
			if(Iseg[n][m] != 100)
			{
				niseg++;
			}
		}*/

	    //vpImageIo::writePNG(Iseg,"IedgSeg.png");
	    argImDT=dt0(&Iseg);
	    return argImDT;
		}

vpImage<vpRGBa>* apViews::dtOSeg3(vpImage<unsigned char> &Iseg, int &nedge)
		{
	vpImage<vpRGBa> Ioverlay;
	vpImage<vpRGBa>* argImDT;
	vpImage<unsigned char> Idot(Iseg.getHeight(),Iseg.getWidth());
	//vpImage<unsigned char> I1(Iseg.getHeight(),Iseg.getWidth());
	vpImagePoint ip;
	/*for (int n=0; n < Iseg.getHeight() ; n++)
	{
	for (int m=0 ; m < Iseg.getWidth(); m++)
	  {
		I1[n][m] = 100;
		if(Iseg[n][m]>0)
	    Idot[n][m]=255;
		else{
			Idot[n][m]=0;
			Iseg[n][m] = 255;
		}
	  }
	}*/
	for (int n=0; n < Iseg.getHeight() ; n++)
		{
		for (int m=0 ; m < Iseg.getWidth(); m++)
		  {
			if(Iseg[n][m] == 0)
			{
				Iseg[n][m] = 255;
			}
		  }
		}
	nedge = edgeOrientMapN(Iseg);

	std::cout << " nedge " << nedge << std::endl;

	/*for (int n=0; n < Iseg.getHeight() ; n++)
		{
			if(Iseg[n][m] != 100)
			{
				niseg++;
			}
		}*/

	    //vpImageIo::writePNG(Iseg,"IedgSeg.png");
	    argImDT=dt0(&Iseg);

	    for (int x = 0; x < Iseg.getWidth(); x++) {
	        for (int y = 0; y < Iseg.getHeight(); y++) {
	        	if(Iseg[y][x]!=100)
	        		(*argImDT)[y][x].A = Iseg[y][x];
	        	else
	        		(*argImDT)[y][x].A = 100;
	        }
	      }
	    return argImDT;
		}

double apViews::computeSimilaritySC(std::vector<double> &CP0, std::vector<double> &CP1)
{
	//shape context
	double simi =0;
	for (int i =0 ; i<CP0.size(); i++)
	{
		simi+= (CP0[i]- CP1[i])*(CP0[i]- CP1[i]);
	}
	//std::cout << " simi " << simi/300 << std::endl;
	simi = simi/300;
	return simi;

}

double apViews::computeSimilaritySCPerm(std::vector<double> &CP0, std::vector<double> &CP1, int perm, int nr, int nw)
{
	//shape context
	/*std::vector<double> cp_;
	cp_.resize(nr*nw);
	std::string filename;
	filename = "histrot.txt";*/
	double simi =0;
	double cperm;
	int i =0;
	for(int rr = 0; rr < nr; rr++)
		for (int ww =0 ; ww<nw; ww++)
	{

			if(ww>=perm && ww-perm<nw)
				cperm = CP1[rr*nw + ww-perm];
			else if(ww<perm)
				cperm = CP1[rr*nw + nw-1-ww+perm];
			else if(ww-perm>=nw)
				{cperm = CP1[rr*nw + ww-perm-nw];}

			//cp_[i] = cperm;

		simi+= (CP0[i]- cperm)*(CP0[i]- cperm);
		i++;
	}
	simi = simi/300;

	/*ofstream cfile (filename.c_str(),ios::out);
	if (cfile.is_open())
	{
		int size = cp_.size();
		for(int i = 0; i < size; i++){
				cfile << cp_[i] <<"\t";
			}
		cfile.close();
	}*/
	return simi;


}



double apViews::computeSimilarityPosRotScaleSeg(vpImage<vpRGBa> *Im, vpImage<vpRGBa> *Iseg, vpImage<unsigned char> *IT, vpImagePoint &pI, const double Rot)
{
		int width = Im->getWidth();
		int height = Im->getHeight();
		int m,mo,yy,xx,adtIy,adtIx,xp,yp,xpim,ypim,xrot,yrot;
		double cor,t0,t1,dist,delta,oriT,oriI,dtI,rho,theta;
		//int offs =(int) 127*((double)width/512);
		int offs =(int) 127;
		t0= vpTime::measureTimeMs();
		dist=0;
		mo=0;
		m=0;
		double ang;
		double distrat = 0;
		for (int y = 0; y < height; y++) {
		  for (int x = 0; x < width; x++) {
			  if(((*IT)[y][x]!=100) && x!=(int)width/2)
			  {
			  rho = sqrt((y-(int)height/2)*(y-(int)height/2)+(x-(int)width/2)*(x-(int)width/2));
			  theta = atan2(-(y-(int)height/2),(x-(int)width/2));
		      theta = theta + Rot;
			  xrot = (int)(rho*cos(theta)+(int)width/2);
			  yrot = (int)(-rho*sin(theta)+(int)height/2);
			  yp = yrot+(int)(pI.get_i()-(int)height/2);
			  xp = xrot+(int)(pI.get_j()-(int)width/2);

			  if (yp>0 && yp<height && xp>0 && xp<width)
		    {
				  ypim = yp-(int)(((*Im)[yp][xp]).R-offs)*2;
				  xpim = xp-(int)(((*Im)[yp][xp]).G-offs)*2;
				  if( ypim >0 && ypim <height && xpim > 0 && xpim <width)
				  {
				  //if ((double)(*Im)[y][x].B<255)
		    	//m++;
		    	//oriI = (double)(*Im)[adtIy][adtIx];

				if((*IT)[y][x]-255*(Rot/3.1416-floor(Rot/3.1416))>0  && (*IT)[y][x]-255*(Rot/3.1416-floor(Rot/3.1416))<255)
				{
			     ang = - (int)255*(Rot/3.1416-floor(Rot/3.1416));
				}
				else if ((*IT)[y][x]-255*(Rot/3.1416-floor(Rot/3.1416))<0)
				{
			     ang = 255 - (int)255*(Rot/3.1416-floor(Rot/3.1416));
				}
				else if ((*IT)[y][x]-255*(Rot/3.1416-floor(Rot/3.1416))>255)
				{
			     ang = -255 - 255*(Rot/3.1416-floor(Rot/3.1416));
				}

		    	oriI = (double)(*Im)[ypim][xpim].A;//+ang;
		    	yy=-1;
		    	xx=-1;
		    	while (oriI==100 && yy<2)
		    		{


		    		oriI=(double)(*Im)[ypim + yy][xpim + xx].A;// + ang ;
		    		yy ++;
					while ( oriI==100 && xx<2)
					{
						oriI=(double)(*Im)[ypim + yy][xpim + xx].A;// + ang ;
						xx ++;
					}
		    		}
		    	//(*Im)[ypim][xpim].A = oriI;
		    	if (oriI>100 || oriI<100)
		    		{
		    		mo++;
		    		delta=abs((*IT)[y][x]+ang-oriI);
		    		//delta=abs((double)(*argIDT)[y][x].A-oriT);
		    		if (delta>255-delta) delta=255-delta;
			   	//cout  << y << " " << coo << " " << orientT << " " << (double)(*I10)[y][x] << " " << x << endl;
		    		//cout << " dist " << ((double)(*Im)[yp][xp].B) << " dist 2 " << sqrt((yp-pI.get_i())*(yp-pI.get_i())+(xp-pI.get_j())*(xp-pI.get_j())) << endl;
 		    	//dist = dist + dtI + 0.1*delta;
		    		//cout  << " dist  " << (double)(*Im)[yp][xp].B << " xp " << xp << " yp " << yp << " dist 2 " << sqrt(vpMath::sqr((int)(((*Im)[yp][xp]).R-offs)*2) + vpMath::sqr((int)(((*Im)[yp][xp]).G-offs)*2))  << endl;
                    //distrat = distrat + 100*abs(1-(sqrt((yp-pI.get_i())*(yp-pI.get_i())+(xp-pI.get_j())*(xp-pI.get_j())) / sqrt((ypim-pI.get_i())*(ypim-pI.get_i())+(xpim-pI.get_j())*(xpim-pI.get_j()))));
		    		distrat = distrat + 300*((double)(*Im)[yp][xp].B + 0.5*(double)(*Iseg)[yp][xp].B)/sqrt((yp-pI.get_i())*(yp-pI.get_i())+(xp-pI.get_j())*(xp-pI.get_j())) + 0.1*delta;
		    		//distrat = distrat + 1000*abs(((yp-pI.get_i())*(ypim-pI.get_i())+(xp-pI.get_j())*(xpim-pI.get_j())) / (sqrt((ypim-pI.get_i())*(ypim-pI.get_i())+(xpim-pI.get_j())*(xpim-pI.get_j()))*sqrt((yp-pI.get_i())*(yp-pI.get_i())+(xp-pI.get_j())*(xp-pI.get_j()))));
		    		//dist = dist + (double)(*Im)[yp][xp].B + 0*delta;
		    	//dist = dist  + 0.1*delta;
		    		}
		    }
		    }
		  }
		  }
		}

		//distrat=1000-distrat/mo;
		distrat = distrat/mo;
		t1= vpTime::measureTimeMs();

		return distrat;
}









/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
