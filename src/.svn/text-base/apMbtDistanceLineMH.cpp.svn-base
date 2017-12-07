#include "apMbtDistanceLineMH.h"
#include <visp/vpPlane.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpFeatureBuilder.h>
#include <stdlib.h>

/*!
  Basic constructor
*/

void
apMbtDistanceLineMH::init()
{
	MHmeline = NULL;
		  meline_ = NULL ;
		lost = false;
		index = 0;
			wmean = 1;
			cam = NULL;
			  nbFeature =0 ;
			  Reinit = false;
			  isvisible = true;

			  name = "";
			  p1 = NULL ;
			  p2 = NULL ;
			  line = NULL ;
			  meline = NULL ;
			  //hiddenface = NULL ;
			  idx = 0;
			  line = NULL;
			  pointsvect.resize(0);
}

apMbtDistanceLineMH::apMbtDistanceLineMH()//: vpMbtDistanceLine()
{
	init();
}


apMbtDistanceLineMH& apMbtDistanceLineMH::operator =(const apMbtDistanceLineMH& l)
{


	if (l.me!=NULL)
	{
		if (me==NULL)
			me = new vpMe;
		*me = *l.me;
	}
	else if ((me!=NULL)&&(l.me==NULL))
	{
		delete me;
		me=NULL;
	}

	if (l.line!=NULL)
	{
		if (line==NULL)
			line = new vpLine;
		*line = *l.line;
	}
	else if ((line!=NULL)&&(l.line==NULL))
	{
		delete line;
		line = NULL;
	}

	if (l.MHmeline!=NULL)
	{
		if (MHmeline==NULL)
			MHmeline = new apMHMeLine;
		*MHmeline = *l.MHmeline;
	}
	else if ((MHmeline!=NULL)&&(l.MHmeline==NULL))
	{
		delete MHmeline;
		MHmeline = NULL;
	}

	if (l.meline_!=NULL)
	{
		if (meline_==NULL)
			meline_ = new apMeLine;
		*meline_ = *l.meline_;
	}
	else if ((meline_!=NULL)&&(l.meline_==NULL))
	{
		delete meline_;
		meline_ = NULL;
	}

	if (l.meline!=NULL)
	{
		if (meline==NULL)
			meline = new vpMbtMeLine;
		*meline = *l.meline;
	}
	else if ((meline!=NULL)&&(l.meline==NULL))
	{
		delete meline;
		meline = NULL;
	}

	if (l.cam!=NULL)
	{
		if (cam==NULL)
			cam = new vpCameraParameters;
		*cam = *l.cam;
	}
	else if ((cam!=NULL)&&(l.cam==NULL))
	{
		delete cam;
		cam = NULL;
	}




	return *this;
}


/*apMbtDistanceLineMH& apMbtDistanceLineMH::operator =(const apMbtDistanceLineMH& l)
{
	wmean = l.wmean ;
	nbFeature = l.nbFeature ;
	isvisible = l.isvisible ;
	//useMH = l.useMH ;
	lost = l.lost;
	index = l.index;
	error = l.error;
	L = l.L;
	Lindex_polygon = l.Lindex_polygon;

	hiddenface = l.hiddenface;
	featureline = l.featureline;

	if (l.p1!=NULL)
	{
		if (p1==NULL)
			p1 = new vpPoint;
		*p1 = *l.p1;
	}
	else if ((p1!=NULL)&&(l.p1==NULL))
	{
		delete p1;
		p1=NULL;
	}

	if (l.p2!=NULL)
	{
		if (p2==NULL)
			p2 = new vpPoint;
		*p2 = *l.p2;
	}
	else if ((p2!=NULL)&&(l.p2==NULL))
	{
		delete p2;
		p2=NULL;
	}

	if (l.me!=NULL)
	{
		if (me==NULL)
			me = new vpMe;
		*me = *l.me;
	}
	else if ((me!=NULL)&&(l.me==NULL))
	{
		delete me;
		me=NULL;
	}

	if (l.line!=NULL)
	{
		if (line==NULL)
			line = new vpLine;
		*line = *l.line;
	}
	else if ((line!=NULL)&&(l.line==NULL))
	{
		delete line;
		line = NULL;
	}

	if (l.MHmeline!=NULL)
	{
		if (MHmeline==NULL)
			MHmeline = new apMHMeLine;
		//*MHmeline = *l.MHmeline;
	}
	else if ((MHmeline!=NULL)&&(l.MHmeline==NULL))
	{
		delete MHmeline;
		MHmeline = NULL;
	}

	if (l.meline_!=NULL)
	{
		if (meline_==NULL)
			meline_ = new apMeLine;
		*meline_ = *l.meline_;
	}
	else if ((meline_!=NULL)&&(l.meline_==NULL))
	{
		delete meline_;
		meline_ = NULL;
	}

	if (l.cam!=NULL)
	{
		if (cam==NULL)
			cam = new vpCameraParameters;
		*cam = *l.cam;
	}
	else if ((cam!=NULL)&&(l.cam==NULL))
	{
		delete cam;
		cam = NULL;
	}


	return *this;
}*/

/*!
  Basic destructor useful to deallocate the memory.
*/
apMbtDistanceLineMH::~apMbtDistanceLineMH()
{
  if (MHmeline != NULL) delete MHmeline ;
  if (meline_ != NULL) delete meline_ ;
  if (meline != NULL) delete meline ;
	if (line != NULL) delete line ;
	if (p1 != NULL) delete p1 ;
	if (p2 != NULL) delete p2 ;


	for (int i = 0 ; i<pointsvect.size(); i++)
	{
		apControlPoint *pt = pointsvect[i];
		delete pt;
	}
	pointsvect.resize(0);

}

/*!
  Project the line and the two points corresponding to its extremities into the image.

  \param cMo : The pose of the camera used to project the line into the image.
*/
void
apMbtDistanceLineMH::project(const vpHomogeneousMatrix &cMo)
{
  line->project(cMo) ;
  p1->project(cMo) ;
  p2->project(cMo) ;
}

/*!
  Build a 3D plane thanks to 3 points and stores it in \f$ plane \f$.

  \param P : The first point to define the plane
  \param Q : The second point to define the plane
  \param R : The third point to define the plane
  \param plane : The vpPlane instance used to store the computed plane equation.
*/
void
apMbtDistanceLineMH::buildPlane(vpPoint &P, vpPoint &Q, vpPoint &R, vpPlane &plane)
{
  vpColVector a(3);
  vpColVector b(3);
  vpColVector n(3);
  //Calculate vector corresponding to PQ
  a[0]=P.get_oX()-Q.get_oX();
  a[1]=P.get_oY()-Q.get_oY();
  a[2]=P.get_oZ()-Q.get_oZ();

  //Calculate vector corresponding to PR
  b[0]=P.get_oX()-R.get_oX();
  b[1]=P.get_oY()-R.get_oY();
  b[2]=P.get_oZ()-R.get_oZ();

  //Calculate normal vector to plane PQ x PR
  n=vpColVector::cross(a,b);

  //Equation of the plane is given by:
  double A = n[0];
  double B = n[1];
  double C = n[2];
  double D=-(A*P.get_oX()+B*P.get_oY()+C*P.get_oZ());

  double norm =  sqrt(A*A+B*B+C*C) ;
  plane.setA(A/norm) ;
  plane.setB(B/norm) ;
  plane.setC(C/norm) ;
  plane.setD(D/norm) ;
}


/*!
  Build a line thanks to 4 points.

  The method is the following : Two plane are computed thanks to (P1,P2,P3) and (P1,P2,P4) (see the buildPlane method). Then the line equation is computed thanks to the intersection between the two planes.

  \param P1 : The first point to compute the line.
  \param P2 : The second point to compute the line.
  \param P3 : The third point to compute the line.
  \param P4 : The fourth point to compute the line.
  \param L : The instance of vpLine to store the computed line equation.
*/
void
apMbtDistanceLineMH::buildLine(vpPoint &P1, vpPoint &P2, vpPoint &P3, vpPoint &P4, vpLine &L)
{
  vpPlane plane1;
  vpPlane plane2 ;
  buildPlane(P1,P2,P3,plane1) ;
  buildPlane(P1,P2,P4,plane2) ;

  L.setWorldCoordinates(plane1.getA(),plane1.getB(), plane1.getC(),plane1.getD(),
			plane2.getA(),plane2.getB(), plane2.getC(),plane2.getD()) ;
}


/*!
  Build a vpMbtDistanceLine thanks to two points corresponding to the extremities.

  \param _p1 : The first extremity.
  \param _p2 : The second extremity.
*/
void
apMbtDistanceLineMH::buildFrom(vpPoint &_p1, vpPoint &_p2)
{
  line = new vpLine ;
  p1 = new vpPoint ;
  p2 = new vpPoint ;

  *p1 = _p1 ;
  *p2 = _p2 ;

  vpColVector V1(3);
  vpColVector V2(3);
  vpColVector V3(3);
  vpColVector V4(3);

  V1[0] = p1->get_oX();
  V1[1] = p1->get_oY();
  V1[2] = p1->get_oZ();
  V2[0] = p2->get_oX();
  V2[1] = p2->get_oY();
  V2[2] = p2->get_oZ();


  if((V1-V2).sumSquare()!=0)
  {
    {
      V3[0]=double(rand()%1000)/100;
      V3[1]=double(rand()%1000)/100;
      V3[2]=double(rand()%1000)/100;


      vpColVector v_tmp1,v_tmp2;
      v_tmp1 = V2-V1;
      v_tmp2 = V3-V1;
      V4=vpColVector::cross(v_tmp1,v_tmp2);
    }

    vpPoint P3;
    P3.setWorldCoordinates(V3[0],V3[1],V3[2]);
    vpPoint P4;
    P4.setWorldCoordinates(V4[0],V4[1],V4[2]);
    buildLine(*p1,*p2, P3,P4, *line) ;
  }
  else
  {
    vpPoint P3;
    P3.setWorldCoordinates(V1[0],V1[1],V1[2]);
    vpPoint P4;
    P4.setWorldCoordinates(V2[0],V2[1],V2[2]);
    buildLine(*p1,*p2,P3,P4,*line) ;
  }
}


/*!
  Set the moving edge parameters.

  \param _me : an instance of vpMe containing all the desired parameters
*/
void
apMbtDistanceLineMH::setMovingEdgeMH(vpMe *_me)
{
  me = _me ;
  if (MHmeline != NULL)
  {
    MHmeline->setMe(me);
    //MHmeline->setCameraParameters(cam);
    meline->setMe(me) ;
  }
}
/*!
  Initialize the moving edge thanks to a given pose of the camera.
  The 3D model is projected into the image to create moving edges along the line.

  \param I : The image.
  \param cMo : The pose of the camera used to initialize the moving edges.
*/

void
apMbtDistanceLineMH::initMovingEdgeMHP(const vpImage<unsigned char> &I,const vpImage<unsigned char> &gradMap, const vpHomogeneousMatrix &cMo)
{
  //if(isvisible)
  {
    MHmeline = new apMHMeLine;
    MHmeline->setMe(me);
    MHmeline->setCameraParameters(cam);
    MHmeline->init_range = 0;
    int marge = /*10*/5; //ou 5 normalement
    //if (ip1.get_j()<ip2.get_j()) { MHmeline->jmin = (int)ip1.get_j()-marge ; MHmeline->jmax = (int)ip2.get_j()+marge ; } else{ MHmeline->jmin = (int)ip2.get_j()-marge ; MHmeline->jmax = (int)ip1.get_j()+marge ; }
    //if (ip1.get_i()<ip2.get_i()) { MHmeline->imin = (int)ip1.get_i()-marge ; MHmeline->imax = (int)ip2.get_i()+marge ; } else{ MHmeline->imin = (int)ip2.get_i()-marge ; MHmeline->imax = (int)ip1.get_i()+marge ; }
    try
    {
    if(pointsvect.size()>0)
      MHmeline->initTrackingP(I,pointsvect);
    }
    catch(...)
    {
      //vpTRACE("the line can't be initialized");
    }
  }
//	trackMovingEdge(I,cMo)  ;
}



/*!
  Track the moving edges in the image.

  \param I : the image.
  \param cMo : The pose of the camera.
*/
void
apMbtDistanceLineMH::trackMovingEdgeMHP(const vpImage<unsigned char> &I, const vpImage<unsigned char> &gradMap, const vpHomogeneousMatrix & /*cMo*/)
{
  {
    try
    {
    	if(pointsvect.size()>0)
          MHmeline->trackG(I, gradMap) ;
		//std::cout <<  "points-vect size " <<MHmeline->points_vect.size() << std::endl;

    }
    catch(...)
    {
      Reinit = true;
    }
    nbFeature = MHmeline->list.size();
  }
}

/*!
    Enable to display the points along the line with a color corresponding to their state.

    - If green : The vpMeSite is a good point.
    - If blue : The point is removed because of the vpMeSite tracking phase (constrast problem).
    - If purple : The point is removed because of the vpMeSite tracking phase (threshold problem).
    - If blue : The point is removed because of the robust method in the virtual visual servoing.

    \param I : The image.
*/
/*void
vpMbtDistanceLine::displayMovingEdges(const vpImage<unsigned char> &I)
{
  if (meline != NULL)
  {
    meline->list.front();
    while (!meline->list.outside())
    {
      vpMeSite pix = meline->list.value();
      if (pix.suppress == 0)
	vpDisplay::displayCross(I,vpImagePoint(pix.ifloat,pix.jfloat),3,vpColor::green,1);
      if (pix.suppress == 1)
	vpDisplay::displayCross(I,vpImagePoint(pix.ifloat,pix.jfloat),3,vpColor::blue,1);
      if (pix.suppress == 2)
	vpDisplay::displayCross(I,vpImagePoint(pix.ifloat,pix.jfloat),3,vpColor::purple,1);
      if (pix.suppress == 4)
	vpDisplay::displayCross(I,vpImagePoint(pix.ifloat,pix.jfloat),3,vpColor::red,1);
      meline->list.next();
    }
  }
}*/


/*!
  Display the line. The 3D line is projected into the image.

  \param I : The image.
  \param cMo : Pose used to project the 3D model into the image.
  \param cam : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the line.
*/
void
apMbtDistanceLineMH::display(const vpImage<unsigned char>&I, const vpHomogeneousMatrix &cMo, const vpCameraParameters&cam, const vpColor col, const unsigned int thickness)
{
  //if (isvisible ==true)
  {
    p1->changeFrame(cMo) ;
    p2->changeFrame(cMo) ;
    line->changeFrame(cMo) ;

    p1->projection() ;
    p2->projection() ;
    line->projection() ;

    vpImagePoint ip1, ip2;

    vpMeterPixelConversion::convertPoint(cam,p1->get_x(),p1->get_y(),ip1) ;
    vpMeterPixelConversion::convertPoint(cam,p2->get_x(),p2->get_y(),ip2) ;

    vpDisplay::displayLine(I,ip1,ip2,col, thickness);
  }
}


/*!
  Display the line. The 3D line is projected into the image.

  \param I : The image.
  \param cMo : Pose used to project the 3D model into the image.
  \param cam : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the line.
*/
void
apMbtDistanceLineMH::display(const vpImage<vpRGBa>&I, const vpHomogeneousMatrix &cMo, const vpCameraParameters&cam, const vpColor col, const unsigned int thickness)
{
  //if (isvisible ==true)
  {
    p1->changeFrame(cMo) ;
    p2->changeFrame(cMo) ;
    line->changeFrame(cMo) ;

    p1->projection() ;
    p2->projection() ;
    line->projection() ;

    vpImagePoint ip1, ip2;

    vpMeterPixelConversion::convertPoint(cam,p1->get_x(),p1->get_y(),ip1) ;
    vpMeterPixelConversion::convertPoint(cam,p2->get_x(),p2->get_y(),ip2) ;

    vpDisplay::displayLine(I,ip1,ip2,col, thickness);
  }
}


/*!
    Enable to display the points along the line with a color corresponding to their state.

    - If green : The vpMeSite is a good point.
    - If blue : The point is removed because of the vpMeSite tracking phase (constrast problem).
    - If purple : The point is removed because of the vpMeSite tracking phase (threshold problem).
    - If blue : The point is removed because of the robust method in the virtual visual servoing.

    \param I : The image.
*/
void
apMbtDistanceLineMH::displayMovingEdges(const vpImage<unsigned char> &I)
{
  if (meline != NULL)
  {
	for(std::list<vpMeSite>::iterator it=meline->list.begin(); it!=meline->list.end(); ++it)
	  {
	    vpMeSite pix = *it;
      /*if (pix.suppress == 0)
	vpDisplay::displayCross(I,vpImagePoint(pix.ifloat,pix.jfloat),3,vpColor::green,1);
      if (pix.suppress == 1)
	vpDisplay::displayCross(I,vpImagePoint(pix.ifloat,pix.jfloat),3,vpColor::blue,1);
      if (pix.suppress == 2)
	vpDisplay::displayCross(I,vpImagePoint(pix.ifloat,pix.jfloat),3,vpColor::purple,1);
      if (pix.suppress == 4)
	vpDisplay::displayCross(I,vpImagePoint(pix.ifloat,pix.jfloat),3,vpColor::red,1);*/
    }
  }
}


/*!
  Initialize the size of the interaction matrix and the error vector.
*/

void
apMbtDistanceLineMH::initInteractionMatrixErrorMH()
{
	//if(MHmeline->list.size()>0)
	{
    L.resize(MHmeline->list.size(),6) ;
    error.resize(MHmeline->list.size()) ;
    weight.resize(MHmeline->list.size()) ;
    nbFeature = MHmeline->list.size() ;
    }
}

/*!
  Compute the interaction matrix and the error vector corresponding to the line.
*/
void
apMbtDistanceLineMH::computeInteractionMatrixErrorMH(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo)
{

  //if (isvisible)
	{
int j =0 ;
apControlPoint *pt;
bool valid;
double x,y, xmin, ymin ;
int i0,j0,imin,jmin,kmin,jjmin;
jmin = 0;
apMHMeSite p;
std::vector< std::vector<apMHMeSite> > sites_cand = MHmeline->points_vect;
double errormin;
vpLine linept;
//std::cout << "ok line 0 "  << pointsvect.size() << std::endl ;
	for(std::vector<apMHMeSite>::iterator itr=MHmeline->list.begin(); itr!=MHmeline->list.end(); ++itr)
	  {
    // feature projection
	//std::cout << "ok line 0 "  << MHmeline->points_vect.size() << std::endl ;
    pt = pointsvect[j];
    linept = pt->getLine();
    linept.changeFrame(cMo);
    double a,b,c,s;
    a = linept.cP[4]*linept.cP[3] - linept.cP[0]*linept.cP[7];
    b = linept.cP[5]*linept.cP[3] - linept.cP[1]*linept.cP[7];
    c = linept.cP[6]*linept.cP[3] - linept.cP[2]*linept.cP[7];
    s = a*a + b*b;
    if (s>1e-8)
    {
    linept.projection();

    vpFeatureLine featl;
    vpFeatureBuilder::create(featl,linept) ;
    double rho = featl.getRho();
    double theta = featl.getTheta();
    //std::cout << "ok line 1 " << rho  << std::endl ;
    double co = cos(theta);
    double si = sin(theta);

    double mx = 1.0/cam->get_px() ;
    double my = 1.0/cam->get_py() ;
    double xc = cam->get_u0() ;
    double yc = cam->get_v0() ;

    double alpha ;
    vpMatrix H;
    H = featl.interaction();

    //std::cout << "ok line 3 "  << MHmeline->points_vect.size() << std::endl ;

    errormin = 2 ;
    valid = false;
    jjmin = 0;
    double wght = 0;
    int nsite = 0;
	for (int jj = 0; jj < sites_cand.size(); jj++) //for each candidate of P
	{
		//vpPointSite Pk = (list.value());
		//vpPointSite Pk0 = (list.value());
		apMHMeSite Pk = sites_cand[jj][j];

        if (jj == 0)
        {
            xmin = (Pk.j - xc) * mx;
            ymin = (Pk.i - yc) * my;
        }

		//apMHMeSite Pk =(MHmeline->list.value()).candidateList[jj];
		//std::cout << " ok o  " << Pk.i  << " " << Pk.j << " " << std::endl;

	      if((Pk.suppress == 0) )
	      {
//wght += sites_cand[jj][j].getWeight();
nsite ++;
x = (double)Pk.j;
y = (double)Pk.i;
j0=Pk.j;
i0=Pk.i;
//vpDisplay::displayCross(I,i0, j0, 3,vpColor::red);
x = (x-xc)*mx ;
y = (y-yc)*my ;
error[j] = rho - ( x*co + y*si) ;
//std::cout << " errorc " << fabs(error[j]) << " " << x << " " << y << std::endl;
//std::cout << " weight " << sites_cand[jj][ii].getWeight() << std::endl;
//if (abs(err)>errormin)
if (fabs(error[j])<=errormin)
{
//errormin = err;
errormin = fabs(error[j]);
xmin = x;
ymin = y;
imin=i0;
jmin=j0;
jjmin = jj;
valid = true;
}
	      }
	}
	//std::cout << "ok line 2 "  << jjmin << " j " << j << std::endl ;
	if(valid)
	{
  //error[j]=rho - ( xmin*co + ymin*si);
  if (MHmeline->list.size() > 10)// && sites_cand[jjmin][j].getWeight()>0.3 && sites_cand[jjmin][j].getWeight()<1.9)
  {
  weight[j] = (sites_cand[jjmin][j].getWeight())/2.0;
  if (weight[j]>2)
	  weight[j] = 0.5;
  //error[j]=sites_cand[jjmin][j].getWeight()*(rho - ( xmin*co + ymin*si));
  //weight[j] = 1;
  }
  else {
	  weight[j] = 0.5;
	  }
  error[j]=(rho - ( xmin*co + ymin*si));
  //weight[j] = 1;
  //std::cout << "ok line 4 "  << jjmin << " j " << j << std::endl ;
  //weight[j] = 1;
  if(weight[j]>0.5)
  vpDisplay::displayCross(I, sites_cand[jjmin][j].i,sites_cand[jjmin][j].j, 2, vpColor::green);
  if(weight[j]<0.5)
  vpDisplay::displayCross(I, sites_cand[jjmin][j].i,sites_cand[jjmin][j].j, 2, vpColor::yellow);
  //std::cout << " weightmin " << weight[ii] << std::endl;

  alpha = xmin*si - ymin*co;

  double *Lrho = H[0] ;
  double *Ltheta = H[1] ;
  // Calculate interaction matrix for a distance
  for (int k=0 ; k < 6 ; k++)
  {
    L[j][k] = (Lrho[k] + alpha*Ltheta[k]);
	//std::cout << " error " << error[j] << " L " << L[j][k]  << std::endl;
  }
    }
	else
	{
		error[j] = 0;
		  for (int k=0 ; k < 6 ; k++)
		  {
		    L[j][k] = 0;
			//std::cout << " error " << error[j] << " L " << L[j][k]  << std::endl;
		  }
	}
    }
        j++;
    	//std::cout << "ok line 4 "  << std::endl ;
    }
	//std::cout << "ok line 5 "  << std::endl ;
    //if (MHmeline != NULL) delete MHmeline ;
    //if (meline_ != NULL) delete meline_ ;
  	//if (line != NULL) delete line ;
	}
}


/*!
  Test wether the line is close to the border of the image (at a given threshold)

  \param I : the input image (to know its size)
  \param threshold : the threshold in pixel
  \return true if the line is near the border of the image
*/
bool
apMbtDistanceLineMH::closeToImageBorder(const vpImage<unsigned char>& I, const unsigned int threshold)
{
  if(threshold > I.getWidth() || threshold > I.getHeight()){
    return true;
  }
  //if (isvisible)
  {

	for(std::vector<apMHMeSite>::iterator it=MHmeline->list.begin(); it!=MHmeline->list.end(); ++it)
	{
      unsigned int i = it->i ;
      unsigned int j = it->j ;

      if( (i > (I.getHeight()- threshold) ) || i < threshold ||
          (j > (I.getWidth ()- threshold) ) || j < threshold ) {
        return true;
      }
    }
  }
  return false;
}

/*int apMbtDistanceLineMH::rndWghtDraw(double rand)
{
	try{
		//vpMbtMeLine * meline_tmp;
		//srand ( time(NULL) );
		unsigned nb_cand = getNumberOfCandidateLines();
		if (nb_cand == 0) throw nb_cand;
		int k = 0 ;

	// 	//with equal probabilities
	// 	int idx = (int)(nd_cand*((double)rand()/((double)RAND_MAX + 1)));
	// 	distLine = *features_vect[idx];

		//!using weights
	// 	vpColVector max_weight(nb_cand);
	// 	for ( int id = 0 ; id < nb_cand ; id++)
	// 	{
	// 		max_weight[id] = features_vect[id].weight_cumul;
	// 	}
	// 	cout << max_weight<<endl;
		double tirage = ((double)rand)/((double)RAND_MAX + 1)*(MHmeline->weight_cumul[nb_cand-1]);//Nombre aléatoire entre 0 et la somme des poids
		for ( k = 0 ; k < nb_cand ; k++)
		{
			if (MHmeline->weight_cumul[k] > tirage) {
				break;
			}
		}
		return k;

	}
	catch(...)
	{
		cout <<"## Error thrown in mbtDistanceLine::rndWghtDraw"<<endl;
	}
}*/


/*!---------------------------------------------------------------------------------
//! Build l corresponding to the hypothesis number index
//!---------------------------------------------------------------------------------
*/
void apMbtDistanceLineMH::buildDraw(apMbtDistanceLineMH &distLine, int idx)
{


	try{
		//vpMeLine * meline_tmp;
		srand ( time(NULL) );
		//unsigned nb_cand = getNumberOfCandidateLines();
		unsigned nb_cand =0;
		if (nb_cand == 0) throw nb_cand;
		int k = 0 ;
		if (idx >= nb_cand) throw idx;

		//meline_tmp = &(MHmeline->lines_vect.value());
		if (distLine.line ==NULL)
			distLine.line = new vpLine ;
		*(distLine.line) =  *(this->line);
		/*if (distLine.meline_ ==NULL)
			distLine.meline_ = new apMeLine;
		*distLine.meline_ = (MHmeline->lines_vect[idx]);*/
		if (distLine.cam ==NULL)
			distLine.cam = new vpCameraParameters ;
		*distLine.cam = *cam;

		distLine.index = index;
		//distLine.useMH = false;
	}
	catch(...)
	{
		cout <<"## Error thrown in mbtDistanceLine::rndWghtDraw"<<endl;
	}
}

/*apMbtDistanceLineMH* apMbtDistanceLineMH::bestWghtDraw(const vpImage<unsigned char> &I)
{
	//vpMeLine * meline_tmp;
	apMbtDistanceLineMH* distLine;
	distLine = new apMbtDistanceLineMH;
	int idx0 = 0;
	try{
		unsigned nb_cand = getNumberOfCandidateLines();
		std::cout << " nb cand " << nb_cand << std::endl;
		if (nb_cand == 0) throw nb_cand;
		int k = 0 ;

		double best_weight = 0;
		//int idx = 0;

		for ( k = 0 ; k < nb_cand ; k++)
		{
			if (MHmeline->weights[k] > best_weight) {
				best_weight = MHmeline->weights[k];
				idx0 = k;
			}
		}

		distLine->idx = idx0;
		//std::cout << " idx ap " << idx << std::endl;

		  vpImagePoint ip;

		for(std::vector<apMHMeSite>::iterator it=MHmeline->lines_vect[idx0].list.begin(); it!=MHmeline->lines_vect[idx0].list.end(); ++it)
			{
		    apMHMeSite p = *it;

		    if(p.suppress == 1) {
		      ip.set_i( p.i );
		      ip.set_j( p.j);
		      //vpDisplay::displayCross(I, ip, 2, vpColor::white) ; // Contrast
		    }
		    else if(p.suppress == 2) {
		      ip.set_i( p.i );
		      ip.set_j( p.j);
		      //vpDisplay::displayCross(I, ip, 2,vpColor::blue) ; // Threshold
		    }
		    else if(p.suppress == 3) {
		      ip.set_i( p.i );
		      ip.set_j( p.j);
		      //vpDisplay::displayCross(I, ip, 3, vpColor::red) ; // M-estimator
		    }
		    else if(p.suppress == 0) {
		      ip.set_i( p.i );
		      ip.set_j( p.j);
		      //vpDisplay::displayCross(I, ip, 2, vpColor::green) ; // OK
		    }
		  }

		//meline_tmp = &(MHmeline->lines_vect.value());
		//if (distLine.line ==NULL)
			//distLine.line = new vpLine ;
		(distLine->line) =  (line);

		//if (distLine.meline_ ==NULL)
			//distLine.meline_ = new apMeLine ;
		//*(distLine.meline_) = MHmeline->lines_vect.value();
		(distLine->meline_) = &(MHmeline->lines_vect[idx0]);
		//std::cout << " nb el " << distLine->meline_->list.size() << std::endl;


		//if (distLine.cam ==NULL)
			//distLine.cam = new vpCameraParameters ;
		// *distLine.cam = *cam;
		distLine->cam = cam;
		distLine->index = index;


		return distLine;
	}
	catch(...)
	{
		cout <<"## Error thrown in mbtDistanceLine::bestWghtDraw"<<endl;
	}

}
*/

/*!---------------------------------------------------------------------------------
//!
//! Build the feature with the points corresponding to classic VVS
//!
//!---------------------------------------------------------------------------------
*/
void apMbtDistanceLineMH::MaxLikeDraw(apMbtDistanceLineMH &distLine)
{
	apMeLine * meline_tmp = new apMeLine;

	try{

		for(std::vector<apMHMeSite>::iterator it=MHmeline->list.begin(); it!=MHmeline->list.end(); ++it)
		  {
			meline_tmp->list.push_back((apMHMeSite)*it);
		}

		if (distLine.line ==NULL)
			distLine.line = new vpLine ;
		*(distLine.line) =  *(this->line);
		if (distLine.meline_ ==NULL)
			distLine.meline_ = new apMeLine ;
		*distLine.meline_ = *meline_tmp;
		if (distLine.cam ==NULL)
			distLine.cam = new vpCameraParameters ;
		*distLine.cam = *cam;
		distLine.index = index;
		//distLine.useMH = false;
	}
	catch(...)
	{
		cout <<"## Error thrown in mbtDistanceLine::MaxLikeDraw"<<endl;
	}
}

void apMbtDistanceLineMH::initEKF(vpPoint &P1, vpPoint &P2, const vpHomogeneousMatrix &cMo)
{
/*P10 = P1;
P20 = P2;
c0Mo = cMo;
xEstL.resize(2);
xPredL.resize(2);
HxPredL.resize(4);
PEstL.resize(2,2);
PPredL.resize(2,2);
K.resize(2,4);
xEstL[0] = 1/P10[2];
xEstL[1] = 1/P20[2];
Q.resize(2,2);
R.resize(4,4);
R.setIdentity();
H.resize(4,2);
Q[0][0] = 0.1;
Q[1][1] = 0.1;*/
}


void apMbtDistanceLineMH::filterEKF(std::vector<Vec4i> &lines, vpImage<vpRGBa> &Inormd, const vpHomogeneousMatrix &cMo)
{
/*vpHomogeneousMatrix cMc0 = cMo*(c0Mo.inverse());
vpColVector P1c(3),P2c(3);
vpPoint P1c_,P2c_;
P10.changeFrame(cMc0,P1c);
P20.changeFrame(cMc0,P2c);
P1c_[0] = P1c[0];
P1c_[1] = P1c[1];
P1c_[2] = P1c[2];
P2c_[0] = P2c[0];
P2c_[1] = P2c[1];
P2c_[2] = P2c[2];
P1c_.projection();
P2c_.projection();
double x1 = P1c_.get_x();
double y1 = P1c_.get_y();
double x2 = P2c_.get_x();
double y2 = P2c_.get_y();
double u1,v1,u2,v2;
vpMeterPixelConversion::convertPoint(cam,x1,y1,u1,v1);
vpMeterPixelConversion::convertPoint(cam,x2,y2,u2,v2);
xPredL[0] = xEstL[0];
xPredL[1] = xEstL[1];
PPredL = PEstL + Q;
HxPredL[0] = u1;
HxPredL[1] = v1;
HxPredL[2] = u2;
HxPredL[3] = v2;
double distmin = 1000;
double uM1 = 0;
double vM1 = 0;
double uM2 = 0;
double vM2 = 0;
for (size_t i = 0; i < lines.size(); i++) {
	if (Inormd[lines[i][1]][lines[i][0]].A==255 && Inormd[lines[i][3]][lines[i][2]].A==255)
	{
//for (size_t i = 0; i < 1; i++) {
		double cHu = (u1 + u2)/2;
		double cHv = (v1 + v2)/2;
		double cMu = (lines[i][0]+lines[i][2])/2;
		double cMv = (lines[i][1]+lines[i][3])/2;
		double thetaH = atan((v1-v2)/(u1-u2));
		double thetaM = atan((lines[i][1]-lines[i][3])/(lines[i][0]-lines[i][2]));
		double distC = ((cHu - cMu)*(cHu - cMu) + (cHv-cMv)*(cHv-cMv));
		double distTheta = (thetaH-thetaM)*(thetaH-thetaM)/vpMath::sqr(M_PI/180);
		if (sqrt(distC) <20)
		{
			if (distC + distTheta < distmin)
			{
				distmin = distC + distTheta;
				if(u1-lines[i][0] < u1-lines[i][2])
				{
				xMesL[0] = lines[i][0];
				xMesL[1] = lines[i][1];
				xMesL[2] = lines[i][2];
				xMesL[3] = lines[i][3];
				}
				else
				{
					xMesL[2] = lines[i][0];
					xMesL[3] = lines[i][1];
					xMesL[0] = lines[i][2];
					xMesL[1] = lines[i][3];
				}
			}
		}

	}
}

H[0][0] = cMc0[0][3];
H[1][0] = cMc0[1][3];
H[2][1] = cMc0[0][3];
H[2][1] = cMc0[1][3];
PInnovL = H*PPredL*H.transpose() + R;
K = PPredL*H.transpose()*(PInnovL.pseudoInverse());
xEstL = XPredL + K*(xMesL - HxPredL);
vpMatrix I(2,2);
I.setIdentity();
PEstL = (I-K*H)*PPredL;
P10[2] = 1/xEstL[0];
P10[2] = 1/xEstL[1];*/
}
void apMbtDistanceLineMH::updateLine()
{

}
