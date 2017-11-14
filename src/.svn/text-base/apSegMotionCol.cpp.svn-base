#include "apSegMotionCol.h"
#include <visp/vpImage.h>
#include <visp/vpDisplayX.h>
#include <omp.h>

#include <cv.h>
using namespace cv ;

apSegMotionCol::apSegMotionCol(){

EdataF = NULL;
EdataB = NULL;
EdataHF = NULL;
EdataHB = NULL;
EdataGMMF = NULL;
EdataGMMB = NULL;
EdataHistF = NULL;
EdataHistB = NULL;
label = NULL;
labelPts = NULL;
//trajectory = NULL;
nb_trajectory = NULL;
ID_trajectory = NULL;
}

apSegMotionCol::~apSegMotionCol(){
/*	  if (nb_trajectory != NULL)
delete [] nb_trajectory;
	  if (ID_trajectory != NULL)
delete [] ID_trajectory;*/
	  if (EdataF != NULL)
delete [] EdataF;
	  if (EdataB != NULL)
delete [] EdataB;
	  if (label != NULL)
delete [] label;
	  if (EdataHF != NULL)
delete [] EdataHF;
	  if (EdataHB != NULL)
delete [] EdataHB;
	  if (EdataGMMF != NULL)
delete [] EdataGMMF;
	  if (EdataGMMB != NULL)
delete [] EdataGMMB;
	  if (EdataHistF != NULL)
delete [] EdataHistF;
	  if (EdataHistB != NULL)
delete [] EdataHistB;
}

void apSegMotionCol::init(){
	nPoints = KLTParameters.nbPoints;
	colFgd.resize(0);
	colBgd.resize(0);
	bi = 2;
	labelPts = new int[nPoints];

	//nb_trajectory = (int*)malloc (nPoints * sizeof(int));
	//ID_trajectory = (int*)malloc (nPoints * sizeof(int));
	nb_trajectory = new int[nPoints];
	ID_trajectory = new int[nPoints];
	//trajectory = new CvPoint2D32f*[nPoints];
	//trajectory  = (CvPoint2D32f**) malloc (nPoints * sizeof(CvPoint2D32f*));
	//for (int i = 0; i < nPoints; i++)
		//trajectory[i] = new CvPoint2D32f[1000];
		//trajectory[i]= (CvPoint2D32f*) malloc (1000 * sizeof(CvPoint2D32f));

	trajectory_.resize(nPoints);
	for (int i = 0; i < nPoints; i++)
			trajectory_[i].resize(1000);

}

void apSegMotionCol::initTracker(){
tracker.setTrackerId(1);				//1
tracker.setMaxFeatures(nPoints);		//nbPoint
tracker.setWindowSize(KLTParameters.window_size);				//6//10
tracker.setQuality(KLTParameters.quality);				//0.00000001//0.01
tracker.setMinDistance(KLTParameters.min_dist);				//10//15
tracker.setHarrisFreeParameter(KLTParameters.harris_free);	//0.1//0.04
tracker.setBlockSize(KLTParameters.block_size);				//9
tracker.setUseHarris(KLTParameters.use_Harris);				//1
tracker.setPyramidLevels(KLTParameters.level_pyr);			//3
}

void apSegMotionCol::clear()
{
	  if (nb_trajectory != NULL)
delete [] nb_trajectory;
	  if (ID_trajectory != NULL)
delete [] ID_trajectory;
	  if (EdataF != NULL)
delete [] EdataF;
	  if (EdataB != NULL)
delete [] EdataB;
	  if (label != NULL)
delete [] label;
	  if (EdataHF != NULL)
delete [] EdataHF;
	  if (EdataHB != NULL)
delete [] EdataHB;
	  if (EdataGMMF != NULL)
delete [] EdataGMMF;
	  if (EdataGMMB != NULL)
delete [] EdataGMMB;
	  if (EdataHistF != NULL)
delete [] EdataHistF;
	  if (EdataHistB != NULL)
delete [] EdataHistB;
	  if (labelPts != NULL)
delete [] labelPts;
	  /*if (trajectory!=NULL)
	  {
			for (int i = 0; i < nPoints; i++)
				if (trajectory[i] != NULL)
				delete [] trajectory[i];

			delete [] trajectory;
	  }*/
}

void apSegMotionCol::initTrajectories(IplImage *Img){

	tracker.initTracking(Img);
	nbPointToTrack = 0;
	for (int i = 0; i< tracker.getNbFeatures(); i++)
	{
		float x,y;int id;
		tracker.getFeature(i, id, x, y);
		ID_trajectory[nbPointToTrack] = id;
		nb_trajectory[nbPointToTrack] = 1;
		//trajectory [nbPointToTrack][0].x = x;
		//trajectory [nbPointToTrack][0].y = y;
		trajectory_[nbPointToTrack][0].x = x;
		trajectory_[nbPointToTrack][0].y = y;
		nbPointToTrack++;
	}
	nPoints = nbPointToTrack;
}

void apSegMotionCol::initEnergy(vpImage<unsigned char> &_I)
{
	resolution = _I.getHeight()*_I.getWidth();
	EdataF = new double[resolution];
	EdataB = new double[resolution];
	EdataHF = new double[resolution];
	EdataHB = new double[resolution];
	EdataGMMF = new double[resolution];
	EdataGMMB = new double[resolution];
	EdataHistF = new double[resolution];
	EdataHistB = new double[resolution];
	EsmoothH = new double[resolution];
	EsmoothV = new double[resolution];
	label = new int[resolution];
    minEnergy.init(resolution);
    nfg = 0;
    nbg = 0;
	histFg.resize(3);
	histBg.resize(3);
	histFgN.resize(3);
	histBgN.resize(3);
	histFgPts.resize(3);
	histBgPts.resize(3);
	//m_foregroundGMM = new apGMM(5);
	//m_backgroundGMM = new apGMM(5);
}

void apSegMotionCol::setRANSACParameters(apRANSACParameters &_ransac)
{
	ransac.BACKGROUND_THRESHOLD = _ransac.backgroundthresh;
	ransac.CONSENSUS = _ransac.consensus;
	ransac.MAX_RANSAC_ITER = _ransac.nmaxiterations;
	ransac.MINIMAL_SIZE_MODEL = _ransac.minimalsizemodel;
	ransac.PointsToBuildP = _ransac.pointstobuildP;
	ransac.HISTORYOFPOSITION = KLTParameters.history;
	ransac.NBPOINTSTOTRACK = KLTParameters.nbPoints;
}

void apSegMotionCol::setSegmentationParameters(apSegmentationParameters &_seg)
{
	setKLTParameters(_seg.KLTParams);
	setEnergyParameters(_seg.energyParams);
	setKernelParameters(_seg.kernelParams);
	setRANSACParameters(_seg.RANSACParams);
	setDeltaHomography(_seg.deltaHomography);
	setNGaussians(_seg.nGaussians);
	setNBins(_seg.nbins);
}

void apSegMotionCol::updateTrajectories(){

	bool *lostTrajectory = tracker.getListOfLostFeature();
	int nb_lost = 0;

	for(int i = 0; i < tracker.getNbPrevFeatures(); i++)
	{
		if(lostTrajectory[i])
		{
			for (int j = i - nb_lost; j < nbPointToTrack - 1; j++)
			{
				int size = nb_trajectory[j + 1];
				ID_trajectory[j] = ID_trajectory[j + 1];
				nb_trajectory[j] = nb_trajectory[j + 1];

				for (int k = 0; k < size; k++)
					{
					//trajectory[j][k] = trajectory[j+1][k];
					trajectory_[j][k] = trajectory_[j+1][k];
					}
			}
			nbPointToTrack--;
			nb_lost++;
		}
	}

	for(int i = 0; i < nbPointToTrack; i++)
	{
		float x,y;int id;
		tracker.getFeature(i, id, x, y);
		//tracker.g

		if( ID_trajectory[i] == id)
		{
			//trajectory [i][nb_trajectory[i]].x = x;
			//trajectory [i][nb_trajectory[i]].y = y;
			trajectory_[i][nb_trajectory[i]].x = x;
			trajectory_[i][nb_trajectory[i]].y = y;
			nb_trajectory[i]++;
		}
		else 
			printf("ID correspondant pas a la trajectoire\n");
	}


}

void apSegMotionCol::fusionTrajectories(apSegMotionCol tracker_tmp){

	int nbFeatureAdded = 0;
	for (int i = 0; i < tracker_tmp.tracker.getNbFeatures(); i++)
	{
		float x1,y1;
		int id1, max_id;
		tracker_tmp.tracker.getFeature(i, id1, x1, y1);

		bool isNewFeature = true;
		max_id = 0;
		int trackerCount = tracker.getNbFeatures();
		for(int j = 0; j < trackerCount && isNewFeature; j++)
		{
			float x2,y2;
			int id2;

			//printf(" %d,  %d \n", i, j );
			tracker.getFeature(j, id2, x2, y2);
			if (id2 > max_id) 
				max_id = id2;

			int x_1 = x1; int x_2 = x2; int y_1 = y1; int y_2 = y2; 
			if ((x_1 == x_2) && (y_1 == y_2))
				isNewFeature = false;
		}
		if(isNewFeature && tracker.getNbFeatures() < tracker.getMaxFeatures() )
		{
			nbFeatureAdded ++;
			tracker.addFeature(max_id+1, x1, y1);
			ID_trajectory[nbPointToTrack] = max_id + 1;
			//trajectory [nbPointToTrack][0].x = x1;
			//trajectory [nbPointToTrack][0].y = y1;
			trajectory_ [nbPointToTrack][0].x = x1;
			trajectory_ [nbPointToTrack][0].y = y1;
			nb_trajectory[nbPointToTrack] = 1;
			nbPointToTrack++;
		}
	}

}


void apSegMotionCol::displayTrajectories( vpImage<unsigned char> &I){
		
	for (int   j = 0 ; j < nbPointToTrack ; j++)
	{
		int   i;
		int min_born =  (nb_trajectory[j] - KLTParameters.history > 0)? (nb_trajectory[j]-KLTParameters.history): (0);

		int idx_j = j;
		int idx_i = 0;
		int nb_idx = 0;
		int min_born2;

		for (i = nb_trajectory[j]-1  ; i > min_born ; i--)
		{
			nb_idx++;
			/*int x2 = trajectory[j][i].x;
			int y2 = trajectory[j][i].y;
			int x1 = trajectory[j][i-1].x;
			int y1 = trajectory[j][i-1].y;*/
			int x2 = trajectory_[j][i].x;
			int y2 = trajectory_[j][i].y;
			int x1 = trajectory_[j][i-1].x;
			int y1 = trajectory_[j][i-1].y;

			/*int x1 = trajectory[j][min_born2].x;
		   int y1 = trajectory[j][min_born2].y;*/

			vpImagePoint p2 (y1, x1);
			vpImagePoint p1 (y2, x2);



			//switch (labelMotion[j])
			//{
			//case 0 :vpDisplay::displayLine(I,p2,p1,vpColor::red, 1); break;
			//case 1	:vpDisplay::displayLine(I,p2,p1,vpColor::red, 1); break;
			/*default :*/
			vpDisplay::displayLine(I,p2,p1,vpColor::red, 1);//break;
			//}

		}
		/*vpImagePoint ip (trajectory[j][nb_trajectory[j]-1].y, trajectory[j][nb_trajectory[j]-1].x);

		vpImagePoint ip1 (trajectory[j][min_born2].y, trajectory[j][min_born2].x);*/

		vpImagePoint ip (trajectory_[j][nb_trajectory[j]-1].y, trajectory_[j][nb_trajectory[j]-1].x);

		vpImagePoint ip1 (trajectory_[j][min_born2].y, trajectory_[j][min_born2].x);


		//switch (labelMotion[j])
		//{
		//case 0 :vpDisplay::displayCross(I, ip, 5, vpColor::red);   break;
		//case 1	:vpDisplay::displayCross(I, ip, 5, vpColor::red); break;
		/*default :*/
		vpDisplay::displayCross(I, ip, 5, vpColor::red);  //break;
		//vpDisplay::displayCross(I, ip1, 5, vpColor::yellow);
		//}
	}
	vpDisplay::flush(I);


}

void apSegMotionCol::labelPoints(vpImage<unsigned char> &I,vpImage<vpRGBa> &Icol)//std::vector<vpImagePoint> points[2])
{

ransac.RANSACFunction(trajectory_,nb_trajectory,nbPointToTrack,0,labelPts,I.getHeight(),I.getWidth());

int nbData = 5;
int* data;
colBgd.resize(0);
colFgd.resize(0);
for (int i = 0; i<nPoints; i++)
{
	//labelPts[i] = labelMotion0[i];
	//std::cout << " ok1 " << i << std::endl;
	//std::cout << " i " << labelMotion[i] << std::endl;
	int x1 = trajectory_[i][nb_trajectory[i]-1].x;
	int y1 = trajectory_[i][nb_trajectory[i]-1].y;
	if( x1<I.getWidth() && y1 < I.getHeight() )
	{
	vpImagePoint p1 (y1, x1);
	if(labelPts[i]==0){
		data = new int[nbData];
		vpDisplay::displayCross(I, p1, 5, vpColor::blue,5);
		data[0] = Icol[y1][x1].R;
		data[1] = Icol[y1][x1].G;
		data[2] = Icol[y1][x1].B;
		data[3] = y1;
		data[4] = x1;
		colBgd.push_back(data);
	}
	else
	{
		data = new int[nbData];
		vpDisplay::displayCross(I, p1, 5, vpColor::green,5);
		data[0] = Icol[y1][x1].R;
		data[1] = Icol[y1][x1].G;
		data[2] = Icol[y1][x1].B;
		data[3] = y1;
		data[4] = x1;
		colFgd.push_back(data);
	}
	}
}
}

vpRGBa apSegMotionCol::meanTemplate(vpImage<vpRGBa> &Icol,int x,int y)
{
	vpRGBa mean;
	mean.R = 0;
	mean.G = 0;
	mean.B = 0;
	bi = 4;
	bj = 4;
	int a = (2*bi+1)*(2*bj+1);
	for (int k = -bi ; k<=bi; k++)
		for (int l = -bj ; l<=bj; l++)
			if(x>=bj && x <Icol.getWidth()-bj && y>=bi && y <Icol.getHeight()-bi)
		{
			mean.R += Icol[y+k][x+l].R/a;
			mean.G += Icol[y+k][x+l].G/a;
			mean.B += Icol[y+k][x+l].B/a;
		}

	return mean;
}

void apSegMotionCol::labelMeanPoints(vpImage<unsigned char> &I,vpImage<vpRGBa> &Icol)
{

ransac.RANSACFunction(trajectory_,nb_trajectory,nbPointToTrack,0,labelPts,I.getHeight(),I.getWidth());
int nbData = 5;
//vpColVector data(nbData);
int *data;
colBgd.resize(0);
colFgd.resize(0);
vpRGBa mean;
for (int i = 0; i<nPoints; i++)
{
	//labelPts[i] = labelMotion[i];
	int x1 = trajectory_[i][nb_trajectory[i]-1].x;
	int y1 = trajectory_[i][nb_trajectory[i]-1].y;
	if( x1<I.getWidth() && y1 < I.getHeight() )
	{
	vpImagePoint p1 (y1, x1);
	if(labelPts[i]==0){
		vpDisplay::displayCross(I, p1, 5, vpColor::blue);
		mean = meanTemplate(Icol,x1,y1);
		data = new int[nbData];
		data[0] = mean.R;
		data[1] = mean.G;
		data[2] = mean.B;
		data[3] = y1;
		data[4] = x1;
		/*colP.col = Icol[y1][x1];
		colP.pt = p1;*/
		colBgd.push_back(data);
	}
	else
	{
		vpDisplay::displayCross(I, p1, 5, vpColor::green);
		mean = meanTemplate(Icol,x1,y1);
		data = new int[nbData];
		data[0] = mean.R;
		data[1] = mean.G;
		data[2] = mean.B;
		data[3] = y1;
		data[4] = x1;
		colFgd.push_back(data);
	}
	}
}
}

void apSegMotionCol::labelSelectPoints(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol)
{
	/*for (int i = 0; i < nbPointToTrack; i++)
		for (int j = 0; j < nb_trajectory[i]; j++)
			trajectory_[i][j] = trajectory[i][j];*/

	ransac.RANSACFunction(trajectory_,nb_trajectory,nbPointToTrack,0,labelPts,I.getHeight(),I.getWidth());
	int nbData = 5;
	int *data;
	vpRGBa mean;
	vpRGBa col;
	colBgd.resize(0);
	colFgd.resize(0);
	int cgrid = KLTParameters.grid_bg;
	int rgrid = KLTParameters.grid_bg;
	int cgridf = KLTParameters.grid_fg;
	int rgridf = KLTParameters.grid_fg;
	int imin,xminb,yminb,xminf,yminf;
	double distmin =50000;
	double dst;
	for(int ck = 0; ck<cgrid ; ck++)
		for(int vk = 0; vk<rgrid ; vk++)
		{
			distmin =500000;
	for (int i = 0; i<nPoints; i++)
	{
		int x1 = trajectory_[i][nb_trajectory[i]-1].x;
		int y1 = trajectory_[i][nb_trajectory[i]-1].y;

		if( x1<I.getWidth() && y1 < I.getHeight() )
		{
			if(labelPts[i]==0){
		dst = (x1-ck*(I.getWidth()/cgrid)-I.getWidth()/(2*cgrid))*(x1-ck*(I.getWidth()/cgrid)-I.getWidth()/(2*cgrid)) + (y1-vk*(I.getHeight()/cgrid)-I.getHeight()/(2*cgrid))*(y1-vk*(I.getHeight()/cgrid)-I.getHeight()/(2*cgrid));
			if(dst < distmin)
			{
				xminb = x1;
				yminb = y1;
				distmin=dst;
			}
		}
		}
	}
	//if(distmin < sqrt(2)*I.getWidth()/cgrid)
	if(distmin < sqrt(2)*I.getWidth()/cgrid)// && (Icol[yminb][xminb].R > 20 || Icol[yminb][xminb].G > 20 || Icol[yminb][xminb].B > 20 ))
	{
	vpImagePoint p1 (yminb, xminb);
	vpDisplay::displayCross(I, p1, 5, vpColor::blue,5);
	/*mean = meanTemplate(Icol,xminb,yminb);
	data[0] = mean.R;
	data[1] = mean.G;
	data[2] = mean.B;*/
	col = Icol[yminb][xminb];
	data = new int[nbData];
	data[0] = col.R;
	data[1] = col.G;
	data[2] = col.B;
	data[3] = yminb;
	data[4] = xminb;
	colBgd.push_back(data);
	}

		}

	for(int ck = 0; ck<cgridf ; ck++)
		for(int vk = 0; vk<rgridf ; vk++)
		{
			distmin =500000;
	for (int i = 0; i<nPoints; i++)
	{
		int x1 = trajectory_[i][nb_trajectory[i]-1].x;
		int y1 = trajectory_[i][nb_trajectory[i]-1].y;
		if( x1<I.getWidth() && y1 < I.getHeight() )
		{
		//vpImagePoint p1 (y1, x1);

		/*if(labelPts[i]!=0){
		vpDisplay::displayCross(I, p1, 5, vpColor::green,5);
		data[0] = Icol[y1][x1].R;
		data[1] = Icol[y1][x1].G;
		data[2] = Icol[y1][x1].B;
		data[3] = y1;
		data[4] = x1;
		colFgd.push_back(data);
		}*/

		if (labelPts[i]!=0)
		{
		dst = (x1-ck*(I.getWidth()/cgridf)-I.getWidth()/(2*cgridf))*(x1-ck*(I.getWidth()/cgridf)-I.getWidth()/(2*cgridf)) + (y1-vk*(I.getHeight()/cgridf)-I.getHeight()/(2*cgridf))*(y1-vk*(I.getHeight()/cgridf)-I.getHeight()/(2*cgridf));

			if(dst < distmin)
			{
				xminb = x1;
				yminb = y1;
				distmin=dst;
			}
		}

		}
	}
	if(distmin < sqrt(2)*I.getWidth()/cgridf)// && (Icol[yminb][xminb].R > 20 || Icol[yminb][xminb].G > 20 || Icol[yminb][xminb].B > 20 ))
	{
	vpImagePoint p2 (yminb, xminb);
	vpDisplay::displayCross(I, p2, 5, vpColor::green,5);
		col = Icol[yminb][xminb];
		data = new int[nbData];
	data[0] = col.R;
	data[1] = col.G;
	data[2] = col.B;
	data[3] = yminb;
	data[4] = xminb;
	//std::cout << " Fg " << (int)col.R << " " << (int)col.G << " " << (int)col.B << std::endl;
	colFgd.push_back(data);
	}
}
	std::cout << " sizeBg " << colBgd.size() << " sizeFg "<< colFgd.size() << std::endl;
}

void apSegMotionCol::computeInverseDenominateur(int &i, int &j, vpColVector &p){
    inverseDenominateur = (1./(p[2]*j+p[5]*i+1.));
}

void apSegMotionCol::computeInverseDenominateur(vpColVector &X, vpColVector &p){
    inverseDenominateur = (1./(p[2]*X[0]+p[5]*X[1]+1.));
}

void apSegMotionCol::Warp(int i_y, int j_x, double &i_res, double &j_res, vpColVector &p)
{
    computeInverseDenominateur(i_y,j_x,p);
    i_res = (p[1]*j_x + (1.0+p[4])*i_y + p[7])*inverseDenominateur;
    j_res = ((1.0+p[0])*j_x + p[3]*i_y + p[6])*inverseDenominateur;
}

void apSegMotionCol::getParameters(vpHomography &H, vpColVector &p)
{
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
        {
            if(i+3*j!=8)
            {
            	p[i+3*j] = H[i][j];
                if(i==j)p[i+3*j]--;
            }
            else
            	p[i+3*j]=1.;
        }

}



/*void apSegMotionCol::computeLikelihoodHomography(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol)
{
	//for (int i = 0; i < nbPointToTrack; i++)
		//for (int j = 0; j < nb_trajectory[i]; j++)
			//trajectory_[i][j] = trajectory[i][j];

	apRANSAC ransac;
	vpColVector p(9);
	vpHomography H;
	vpHomography Hinv;
	vpMatrix U(3,3);
	int nFrame = 0;
	vpImage<vpRGBa> Iprec;
	vpImage<vpRGBa> Idiff(I.getHeight(),I.getWidth());
	vpImage<vpRGBa> Idiff2(I.getHeight(),I.getWidth());
	histIm.push_back(Icol);
	ransac.RANSACFunctionHomography(trajectory_, nb_trajectory, nbPointToTrack, 0, H, nFrame, labelPts);
	Hinv = H.inverse();
	getParameters(H,p);

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
        {
        	U[i][j] = H[i][j];
        }
    std::cout << " i " << (int)nFrame << std::endl;
    Iprec = histIm[(int)nFrame];

    int nbData = 5;
	vpColVector errorPix(nbData);
	//vpColVector data(5);
	int *data;
	apKernel density(5,0);
	double probaBgd = 0;
	double probaFgd = 0;
	double dsty;
	int l;
	double eta = 0;
	vpColVector _p(3);
	vpColVector p_(3);
	double i_,j_;
	double t0 = vpTime::measureTimeMs();
	vpImage<vpRGBa> Icol_(I.getHeight(),I.getWidth());

	colBgd.resize(0);
	colFgd.resize(0);

	for (int i = 0; i < nPoints ; i ++)
	{
		//std::cout << " ok1 " << i << std::endl;
		//std::cout << " i " << labelMotion[i] << std::endl;
		int x1 = trajectory_[i][nb_trajectory[i]-1].x;
		int y1 = trajectory_[i][nb_trajectory[i]-1].y;
		if( x1<I.getWidth() && y1 < I.getHeight() )
		{
		//std::cout << " ok2 " << x1 << " " << y1 << std::endl;
		vpImagePoint p1 (y1, x1);
		if(labelPts[i]==0){
			vpDisplay::displayCross(I, p1, 5, vpColor::blue);
			data = new int[nbData];
			data[0] = Icol[y1][x1].R;
			data[1] = Icol[y1][x1].G;
			data[2] = Icol[y1][x1].B;
			data[3] = y1;
			data[4] = x1;
			//colP.col = Icol[y1][x1];
			//colP.pt = p1;
			colBgd.push_back(data);
		}
		else //if (labelPts[i]==-1)
		{
			vpDisplay::displayCross(I, p1, 5, vpColor::green);
			data = new int[nbData];
			data[0] = Icol[y1][x1].R;
			data[1] = Icol[y1][x1].G;
			data[2] = Icol[y1][x1].B;
			data[3] = y1;
			data[4] = x1;
			colFgd.push_back(data);
		}
		}
	}


	for (int i = 0; i<Icol.getHeight(); i++)
		for (int j =0 ; j < Icol.getWidth() ; j++)
		{
			//_p[0] = j;
			//_p[1] = i;
			//_p[2] = 1;
			//p_ = U * _p;
			//p_/=p_[2];
			//if(p_[1] > 0 && p_[1] < 512 && p_[0]>0 && p_[0]<512)
			//{Icol_[(int)p_[1]][(int)p_[0]] = Icol[i][j];
			//}
			//std::cout << " i " << i << " j " << j << " i_ " << p_[1]<< " j_ " << p_[0] << std::endl;}
			//Warp(i,j,i_,j_,p);
		    //std::cout << " i " << i << " j " << j << " i_ " << i_ << " j_ " << j_ << std::endl;
			//if(i_ > 0 && i_ < 512 && j_>0 && j_<512)
			//Icol_[(int)i_][(int)j_] = Icol[i][j];

		    double inv_z = 1. / (Hinv[2][0] * j + Hinv[2][1] * i + Hinv[2][2]);

		    j_ = (Hinv[0][0] * j + Hinv[0][1] * i + Hinv[0][2]) * inv_z;
		    i_ = (Hinv[1][0] * j + Hinv[1][1] * i + Hinv[1][2]) * inv_z;

		    if(i_ > 0 && i_ < Icol.getHeight() && j_>0 && j_<Icol.getWidth())
		    			Icol_[(int)i_][(int)j_] = Icol[i][j];

		}
	vpImageIo::writePNG(Icol,"Icol.png");
	vpImageIo::writePNG(Icol_,"Iwarp.png");
	vpImageIo::writePNG(Iprec,"Iprec.png");
	for (int i = 0; i<Icol.getHeight(); i++)
		for (int j =0 ; j < Icol.getWidth() ; j++)
		{
			Idiff[i][j].R = (int)(Icol_[i][j].R + Iprec[i][j].R)/2;
			Idiff[i][j].G = (int)(Icol_[i][j].G + Iprec[i][j].G)/2;
			Idiff[i][j].B = (int)(Icol_[i][j].B + Iprec[i][j].B)/2;

			Idiff2[i][j].R = (int)(Icol[i][j].R + Iprec[i][j].R)/2;
			Idiff2[i][j].G = (int)(Icol[i][j].G + Iprec[i][j].G)/2;
			Idiff2[i][j].B = (int)(Icol[i][j].B + Iprec[i][j].B)/2;
		}

	vpImageIo::writePNG(Idiff,"Idiff.png");
	vpImageIo::writePNG(Idiff2,"Idiff2.png");

	for (int i = 0; i<Icol.getHeight(); i++)
		for (int j =0 ; j < Icol.getWidth() ; j++)
		{
			probaBgd = 0;
			probaFgd = 0;
			vpImagePoint p1(i,j);

			EdataHistB[i*Icol.getWidth() + j] = abs(eta*(log(1-(double)(histFgPts[0].get((int)Icol[i][j].R))/nfg) +
					log(1-(double)(histFgPts[1].get((int)Icol[i][j].G))/nfg) +
					log(1-(double)(histFgPts[2].get((int)Icol[i][j].B))/nfg)));

			EdataHistF[i*Icol.getWidth() + j] = abs(eta*(log(1-(double)(histBgPts[0].get((int)Icol[i][j].R))/nbg) +
					log(1-(double)(histBgPts[1].get((int)Icol[i][j].G))/nbg) +
					log(1-(double)(histBgPts[2].get((int)Icol[i][j].B))/nbg)));

			if(Icol_[i][j].R>0 && Icol_[i][j].G > 0 && Icol_[i][j].B > 0)
			{
			errorPix[0] = ((double)Icol_[i][j].R - (double)Iprec[i][j].R);
			errorPix[1]= ((double)Icol_[i][j].G - (double)Iprec[i][j].G);
			errorPix[2] = ((double)Icol_[i][j].B - (double)Iprec[i][j].B);

			   double density = 0;
			   for (int k=0;k<3;k++)
				   density += errorPix[k]*errorPix[k]/2000;
			   if(density < 1)
			   {
				   density =  0.75*(1-density);}
			   else
				   density = 0;

			   probaBgd += density;

				//probaBgd+=density.KernelDensity(errorPix);
				//std::cout << probaBgd  << std::endl;
				if(probaBgd > 0.1)
				{
					//std::cout << EdataF[i*Icol.getWidth() + j]  << std::endl;
					//vpDisplay::displayCross(I, p1, 5, vpColor::black);
				}
				if (probaBgd>0)
				{
					//std::cout << -log(probaBgd) << " en "<< EdataHistB[i*Icol.getWidth() + j]  << std::endl;
				EdataHB[i*Icol.getWidth() + j] =  -log(probaBgd);// + EdataHistB[i*Icol.getWidth() + j];
				EdataHF[i*Icol.getWidth() + j] =  -log(1 - probaBgd);// + EdataHistF[i*Icol.getWidth() + j];
				}
				else {
					EdataHB[i*Icol.getWidth() + j] = 1 ;//+ EdataHistB[i*Icol.getWidth() + j];
					EdataHF[i*Icol.getWidth() + j] = 0 ;//+ EdataHistF[i*Icol.getWidth() + j];
				}
			}
			else{
				EdataHB[i*Icol.getWidth() + j] = 0;// + EdataHistB[i*Icol.getWidth() + j];
				EdataHF[i*Icol.getWidth() + j] = 0;// + EdataHistF[i*Icol.getWidth() + j];
			}

		}
}*/

void apSegMotionCol::GMM()
{
	double t0 = vpTime::measureTimeMs();
    int i, j;
    int nsamplesBgd = colBgd.size();
    int nsamplesFgd = colFgd.size();
    //CvRNG rng_state = cvRNG(-1);s
    /*CvMat* samplesFgd = cvCreateMat( nsamplesFgd, 3, CV_32FC1 );
    CvMat* labelsBgd = cvCreateMat( nsamplesBgd, 1, CV_32SC1 );
    CvMat* samplesBgd = cvCreateMat( nsamplesBgd, 3, CV_32FC1 );
    CvMat* labelsFgd = cvCreateMat( nsamplesFgd, 1, CV_32SC1 );*/

    cv::Mat samplesFgd( nsamplesFgd, 3,  CV_32FC1);
    cv::Mat labelsBgd( nsamplesBgd, 1, CV_32SC1);
    cv::Mat samplesBgd( nsamplesBgd, 3, CV_32FC1 );
    cv::Mat labelsFgd( nsamplesFgd, 1, CV_32SC1 );
    CvEMParams params;
    int step;
    float *data;
    float *data1;
    //vpColVector _data;
    int *_data;

    /*CvMat* means = cvCreateMat( 10, 3, CV_64FC1 );
    for(int i = 0; i<10;i++)
    {
    step = means->step/sizeof(float);
    data = means->data.fl;
    (data+i*step)[0] = 127;
    (data+i*step)[1] = 127;
    (data+i*step)[2] = 127;
    }*/

    params.covs      = NULL;
    params.means     = NULL;
    params.weights   = NULL;
    params.probs     = NULL;
    params.nclusters = nGaussians;
    params.cov_mat_type       = CvEM::COV_MAT_SPHERICAL;
    params.start_step         = CvEM::START_AUTO_STEP;
    params.term_crit.max_iter = 100;
    params.term_crit.epsilon  = 0.1;
    params.term_crit.type     = CV_TERMCRIT_ITER|CV_TERMCRIT_EPS;


    for(int i = 0; i<nsamplesBgd;i++)
    {
     /*step = samplesBgd->step/sizeof(float);
     data = samplesBgd->data.fl;*/
    _data = colBgd[i];
    /*(samplesBgd->data.fl+i*step)[0] = _data[0];
    (samplesBgd->data.fl+i*step)[1] = _data[1];
    (samplesBgd->data.fl+i*step)[2] = _data[2];*/
    samplesBgd.at<float>(i,0)=(float)_data[0];
    samplesBgd.at<float>(i,1)=(float)_data[1];
    samplesBgd.at<float>(i,2)=(float)_data[2];
    }

    for(int i = 0; i<nsamplesFgd;i++)
    {
    /*step = samplesFgd->step/sizeof(float);
    data1 = samplesFgd->data.fl;*/
    _data = colFgd[i];
    /*(samplesFgd->data.fl+i*step)[0] = _data[0];
    (samplesFgd->data.fl+i*step)[1] = _data[1];
    (samplesFgd->data.fl+i*step)[2] = _data[2];*/

        samplesFgd.at<float>(i,0)=(float)_data[0];
        samplesFgd.at<float>(i,1)=(float)_data[1];
        samplesFgd.at<float>(i,2)=(float)_data[2];


    }
    em_modelFgd.clear();
    em_modelBgd.clear();
    em_modelFgd.train( samplesFgd, cv::Mat(), params, &labelsFgd );
    em_modelBgd.train( samplesBgd, cv::Mat(), params, &labelsBgd );
	//const CvMat* m = em_modelFgd.get_means();
    cv::Mat m = em_modelFgd.get_means();
	for(int g=0;g<nGaussians;g++) {
		//for(int c=0;c<3;c++) cout << m->data.db[g*3 + c] <<", ";
		/*for(int c=0;c<3;c++) cout << m.at<double>(g,c) <<", ";
		cout << endl;*/
	}
	m = em_modelBgd.get_means();
	for(int g=0;g<nGaussians;g++) {
		//for(int c=0;c<3;c++) cout << m->data.db[g*3 + c] <<", ";
		/*for(int c=0;c<3;c++) cout << m.at<double>(g,c) <<", ";
		cout << endl;*/

	}

	double t1 = vpTime::measureTimeMs();

	std::cout << " time set gmm " << t1 - t0 << std::endl;

}

/*void apSegMotionCol::GMM2()
{
	double t0 = vpTime::measureTimeMs();
    int i, j;
    int nsamplesBgd = colBgd.size();
    for(i = 0; i < nPoints)
    int nsamplesFgd = colFgd.size();
    //CvRNG rng_state = cvRNG(-1);s
    CvMat* samplesFgd = cvCreateMat( nsamplesFgd, 3, CV_32FC1 );
    CvMat* labelsBgd = cvCreateMat( nsamplesBgd, 1, CV_32SC1 );
    CvMat* samplesBgd = cvCreateMat( nsamplesBgd, 3, CV_32FC1 );
    CvMat* labelsFgd = cvCreateMat( nsamplesFgd, 1, CV_32SC1 );
    CvEMParams params;
    int step;
    float *data;
    float *data1;
    vpColVector _data;
    nGaussians = 5;

    params.covs      = NULL;
    params.means     = NULL;
    params.weights   = NULL;
    params.probs     = NULL;
    params.nclusters = nGaussians;
    params.cov_mat_type       = CvEM::COV_MAT_SPHERICAL;
    params.start_step         = CvEM::START_AUTO_STEP;
    params.term_crit.max_iter = 200;
    params.term_crit.epsilon  = 0.1;
    params.term_crit.type     = CV_TERMCRIT_ITER|CV_TERMCRIT_EPS;


    for(int i = 0; i<nsamplesBgd;i++)
    {
     step = samplesBgd->step/sizeof(float);
     data = samplesBgd->data.fl;
    _data = colBgd[i];
    (data+i*step)[0] = _data[0];
    (data+i*step)[1] = _data[1];
    (data+i*step)[2] = _data[2];
    }

    for(int i = 0; i<nsamplesFgd;i++)
    {
    step = samplesFgd->step/sizeof(float);
    _data = colFgd[i];
    data = samplesFgd->data.fl;
    (data+i*step)[0] = _data[0];
    (data+i*step)[1] = _data[1];
    (data+i*step)[2] = _data[2];
    }
    em_modelFgd.clear();
    em_modelBgd.clear();
    em_modelFgd.train( samplesFgd, 0, params, labelsFgd );
    em_modelBgd.train( samplesBgd, 0, params, labelsBgd );
	const CvMat* m = em_modelFgd.get_means();
	for(int g=0;g<nGaussians;g++) {
		for(int c=0;c<3;c++) cout << m->data.db[g*3 + c] <<", ";
		cout << endl;
	}
	m = em_modelBgd.get_means();
	for(int g=0;g<nGaussians;g++) {
		for(int c=0;c<3;c++) cout << m->data.db[g*3 + c] <<", ";
		cout << endl;

	}

	double t1 = vpTime::measureTimeMs();

	std::cout << " time set gmm " << t1 - t0 << std::endl;

}*/

void apSegMotionCol::buildGMMs()
{
apGMM gmm(5);
componentFgd.resize(colFgd.size());
componentBgd.resize(colBgd.size());
//gmm.buildGMMs(*m_backgroundGMM,*m_foregroundGMM,colBgd,colFgd,componentBgd,componentFgd);

//std::cout << " ok " << (double)m_backgroundGMM->m_gaussians->mu.R << " ok1 " << (double)m_backgroundGMM->m_gaussians->mu.G << " ok 2 " << (double)m_backgroundGMM->m_gaussians->mu.B << std::endl;

}


void apSegMotionCol::computeDataEnergyGMM(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol)
{
	double t0 = vpTime::measureTimeMs();
	/*CvMat* sample = cvCreateMat( 1, 3, CV_32FC1);
	CvMat* probsFgd = cvCreateMat( 1, nGaussians, CV_64FC1 );
	CvMat* probsBgd = cvCreateMat( 1, nGaussians, CV_64FC1 );*/
	cv::Mat sample( 1, 3, CV_32FC1);
		cv::Mat probsFgd( 1, nGaussians, CV_64FC1 );
		cv::Mat probsBgd( 1, nGaussians, CV_64FC1 );
	double probaFgd, probaBgd;
	float _s;
	float ps[3];
    for( int i = 0; i < Icol.getHeight(); i++ )
    {
        for( int j = 0; j < Icol.getWidth(); j++ )
        {
        	probaFgd = 0;
        	probaBgd = 0;
            /*sample->data.fl[0] = (float)Icol[i][j].R;
            sample->data.fl[1] = (float)Icol[i][j].G;
            sample->data.fl[2] = (float)Icol[i][j].B;*/
        	sample.at<float>(0,0) = (float)Icol[i][j].R;
        	sample.at<float>(0,1) = (float)Icol[i][j].G;
        	sample.at<float>(0,2) = (float)Icol[i][j].B;
            em_modelFgd.predict(sample, &probsFgd );
            em_modelBgd.predict(sample, &probsBgd );
            //double* wFgd = em_modelFgd.get_weights()->data.db;
            //double* wBgd = em_modelBgd.get_weights()->data.db;
            double norm = (double)1/nGaussians;
            for(int k=0; k<nGaussians;k++)
            {
            	//probaFgd += wFgd[k]*(double)(((float*)probsFgd->data.db)[k]);
            	//probaBgd += wBgd[k]*(double)(((float*)probsBgd->data.db)[k]);
            	//probaFgd = max(probaFgd, probsFgd->data.db[k]);
            	//probaBgd = max(probaBgd, probsBgd->data.db[k]);
            	probaFgd = max(probaFgd, probsFgd.at<double>(0,k));
            	probaBgd = max(probaBgd, probsBgd.at<double>(0,k));
                //std::cout << " proba " << norm*probsFgd->data.db[k] << std::endl;

            	/*for(int c=0;c<3;c++) {
                _s = sample->data.fl[c];
            	double mu = em_modelFgd.get_means()->data.db[k*3 + c];
            	double x = (double)_s;
            	double sigma_sq = em_modelFgd.get_covs()->data.db[0];
            	double _p = (1/sqrt(2*M_PI*sigma_sq))*exp(-((x-mu)*(x-mu))/(2*sigma_sq));
            	ps[c] = (float)_p; //((double*)_tmp.data)[0];
            							}
            	probaFgd += (double)ps[0]*ps[1]*ps[2];


                for(int c=0;c<3;c++) {
            	   _s = sample->data.fl[c];
            			         		double mu = em_modelBgd.get_means()->data.db[k*3 + c];
            			            	double x = (double)_s;
            			            	double sigma_sq = em_modelBgd.get_covs()->data.db[k*3 + c];
            			            	double _p = (1/sqrt(2*M_PI*sigma_sq))*exp(-((x-mu)*(x-mu))/(2*sigma_sq));
            			            	ps[c] = (float)_p; //((double*)_tmp.data)[0];
            			            							}
            			            	probaFgd += (double)ps[0]*ps[1]*ps[2];*/

            }
            EdataGMMB[i*Icol.getWidth() + j] = -log(probaBgd);
            EdataGMMF[i*Icol.getWidth() + j] = -log(probaFgd);
            std::cout << " proba1 " << probaBgd << " fgd " << probaFgd << std::endl;
        }
    }

	double t1 = vpTime::measureTimeMs();

	std::cout << " time gmm " << t1 - t0 << std::endl;
}

void apSegMotionCol::computeEnergy(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol)
{
	vpColVector errorPix(5);
	//vpColVector data;
	int *data;
	apKernel density(5,0);
	double probaBgd = 0;
	double probaFgd = 0;
	double dsty;
	int l;

	double eta = 2 ;
	double t0 = vpTime::measureTimeMs();

	std::cout << " hist " << (double)colBgd.size()  << " size fgd " << (double)colFgd.size() << std::endl;

	for (int i = 0; i<Icol.getHeight(); i++)
		for (int j =0 ; j < Icol.getWidth() ; j++)
		{
			probaBgd = 0;
			probaFgd = 0;
			vpImagePoint p1(i,j);
			for (int k = 0 ; k < colBgd.size() ; k++)
			{
				data = colBgd[k];
				errorPix[0] = Icol[i][j].R - data[0];
				errorPix[1] = Icol[i][j].G - data[1];
				errorPix[2] = Icol[i][j].B - data[2];
				errorPix[3] = i - data[3];
				errorPix[4] = j - data[4];
				probaBgd+=density.KernelDensity(errorPix);

			}
			probaBgd*=1;
			probaBgd/=colBgd.size();
			EdataHistB[i*Icol.getWidth() + j] = abs(eta*(log(1-(double)(histFg[0].get((int)Icol[i][j].R))/nfg) +
					log(1-(double)(histFg[1].get((int)Icol[i][j].G))/nfg) +
					log(1-(double)(histFg[2].get((int)Icol[i][j].B))/nfg)));
			//std::cout << -log(probaBgd) << std::endl;
			if (probaBgd>0)
			{
				EdataB[i*Icol.getWidth() + j] = -log(probaBgd) + EdataHistB[i*Icol.getWidth() + j];

			//EdataF[i*Icol.getWidth() + j] = -log(1-probaBgd);
			}
			else
			{
				EdataB[i*Icol.getWidth() + j] = 0 + EdataHistB[i*Icol.getWidth() + j];
				//EdataF[i*Icol.getWidth() + j] = 0;
			}
			if(probaBgd > 0.3)
				label[i*Icol.getWidth() + j] = 0;
			else label[i*Icol.getWidth() + j] = 1;

			for (int k = 0 ; k < colFgd.size() ; k++)
			{
				data = colFgd[k];
				errorPix[0] = Icol[i][j].R - data[0];
				errorPix[1] = Icol[i][j].G - data[1];
				errorPix[2] = Icol[i][j].B - data[2];
				errorPix[3] = i - data[3];
				errorPix[4] = j - data[4];
				//errorPix[3] = 0;
				//errorPix[4] = 0;
				probaFgd+=density.KernelDensity(errorPix);
			}
			probaFgd*=1;
			probaFgd/=colFgd.size();

			EdataHistF[i*Icol.getWidth() + j] = abs(eta*(log(1-(double)(histBg[0].get((int)Icol[i][j].R))/nbg) +
					log(1-(double)(histBg[1].get((int)Icol[i][j].G))/nbg) +
					log(1-(double)(histBg[2].get((int)Icol[i][j].B))/nbg)));

			//std::cout << " hist " << (double)EdataHF[i*Icol.getWidth() + j] << std::endl;

			if (probaFgd>0)
			{EdataF[i*Icol.getWidth() + j] = -log(probaFgd) +  EdataHistF[i*Icol.getWidth() + j] ;
			}
			else
			{
				EdataF[i*Icol.getWidth() + j] = 0 + EdataHistF[i*Icol.getWidth() + j];
			}
			if(EdataHistB[i*Icol.getWidth() + j] > 0.05)
			{
				//std::cout << EdataF[i*Icol.getWidth() + j]  << std::endl;
				//vpDisplay::displayCross(I, p1, 5, vpColor::blue);
			}

		}
	double t1 = vpTime::measureTimeMs();
}


void apSegMotionCol::computeDataEnergyKernelLimb(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol)
{
	vpHomography H;
	vpHomography Hinv;
	int nFrame = 0;
	double ratio = (double)nbins/256;
	vpImage<vpRGBa> Iprec;
	vpImage<vpRGBa> Idiff(I.getHeight(),I.getWidth());
	vpImage<vpRGBa> Idiff2(I.getHeight(),I.getWidth());
	vpRGBa col0,col1;
	ransac.RANSACFunctionH(trajectory_, nb_trajectory, nbPointToTrack, 0, H, nFrame, labelPts, deltaHomography);
    int frame = max(0,(int)histIm.size()- deltaHomography - 1);
    Iprec = histIm[frame];
	Hinv = H.inverse();
	double i_,j_;
	vpImage<vpRGBa> Icol_(I.getHeight(),I.getWidth());
	vpColVector errorPix(5);
	int *data;
	double probaBgd = 0;
	double probaFgd = 0;

	for (int i = 0; i<Icol.getHeight(); i++)
		for (int j =0 ; j < Icol.getWidth() ; j++)
		{

		    double inv_z = 1. / (Hinv[2][0] * j + Hinv[2][1] * i + Hinv[2][2]);

		    j_ = (Hinv[0][0] * j + Hinv[0][1] * i + Hinv[0][2]) * inv_z;
		    i_ = (Hinv[1][0] * j + Hinv[1][1] * i + Hinv[1][2]) * inv_z;

		    if(i_ > 0 && i_ < Icol.getHeight() && j_>0 && j_<Icol.getWidth())
		    			Icol_[(int)i_][(int)j_] = Icol[i][j];

		}
	/*vpImageIo::writePNG(Icol,"Icol.png");
	vpImageIo::writePNG(Icol_,"Iwarp.png");
	vpImageIo::writePNG(Iprec,"Iprec.png");*/
	/*for (int i = 0; i<Icol.getHeight(); i++)
		for (int j =0 ; j < Icol.getWidth() ; j++)
		{
			Idiff[i][j].R = (int)(Icol_[i][j].R + Iprec[i][j].R)/2;
			Idiff[i][j].G = (int)(Icol_[i][j].G + Iprec[i][j].G)/2;
			Idiff[i][j].B = (int)(Icol_[i][j].B + Iprec[i][j].B)/2;

			Idiff2[i][j].R = (int)(Icol[i][j].R + Iprec[i][j].R)/2;
			Idiff2[i][j].G = (int)(Icol[i][j].G + Iprec[i][j].G)/2;
			Idiff2[i][j].B = (int)(Icol[i][j].B + Iprec[i][j].B)/2;
		}*/

	//vpImageIo::writePNG(Idiff,"Idiff.png");
	//vpImageIo::writePNG(Idiff2,"Idiff2.png");

	double density;
	double bwidthH = kernelParameters.bwidth_col_1;

	for (int i = 0; i<Icol.getHeight(); i++)
		for (int j =0 ; j < Icol.getWidth() ; j++)
		{
			probaBgd = 0;
			probaFgd = 0;
			col0 = Icol_[i][j];
			col1 = Iprec[i][j];

			if(col0.R>0 && col0.G > 0 && col0.B > 0)
			{
			errorPix[0] = ((double)col0.R - (double)col1.R);
			errorPix[1]= ((double)col0.G - (double)col1.G);
			errorPix[2] = ((double)col0.B - (double)col1.B);

			   density = 0;
			   for (int k=0;k<3;k++)
				   density += errorPix[k]*errorPix[k]/bwidthH;
			   /*if(density < 1)
			   {
				   density =  0.75*(1-density);}
			   else
				   density = 0;

			   probaBgd += density;*/

			   probaBgd += (1.5/(sqrt(2*M_PI)))*exp(-0.5*density);

				//if(probaBgd > 0.1)
				//{
					//std::cout << EdataF[i*Icol.getWidth() + j]  << std::endl;
					//vpDisplay::displayCross(I, p1, 5, vpColor::black);
				//}
				if (probaBgd>0)
				{
				EdataHB[i*Icol.getWidth() + j] =  -log(probaBgd);
				EdataHF[i*Icol.getWidth() + j] =  -log(1 - probaBgd);
				}
				else {
					EdataHB[i*Icol.getWidth() + j] = 1 ;
					EdataHF[i*Icol.getWidth() + j] = 0 ;
				}
			}
			else{
				EdataHB[i*Icol.getWidth() + j] = 0;
				EdataHF[i*Icol.getWidth() + j] = 1;
			}

		}


	double dsty;
	int l;

	//double eta = 0.08 ;
	//double eta = 0.3 ;
	double alpha = energyParameters.alpha_0;
	double beta = energyParameters.beta_0;
	//double eta1 = 0.1;
	//double eta1 = 20;
	//double eta1 = 1;
	double bwidth = kernelParameters.bwidth_col_0*kernelParameters.bwidth_col_0;
	double bwidth2 = kernelParameters.bwidth_spat*kernelParameters.bwidth_spat;
	vpColVector *data0;

	double t0 = vpTime::measureTimeMs();

	for (int i = 0; i<Icol.getHeight(); i++)
		for (int j =0 ; j < Icol.getWidth() ; j++)
		{
			probaBgd = 0;
			probaFgd = 0;
			vpImagePoint p1(i,j);
			col0 = Icol[i][j];

            //#pragma omp parallel for private(data)
			for (int k = 0 ; k < colBgd.size() ; k++)
			{
				data = colBgd[k];
				dsty = ((col0.R - data[0])*(col0.R - data[0]) + (col0.G - data[1])*(col0.G - data[1]) +
						   (col0.B - data[2])*(col0.B - data[2]))/bwidth;

				   if(dsty < 1)
				   {
					   probaBgd +=  0.75*(1-dsty);}

				   //probaBgd +=dsty;
				   //probaBgd += (1/(sqrt(2*M_PI)))*exp(-0.5*dsty);

			}

			probaBgd*=1;
			probaBgd/=colBgd.size();
			/*probaBgd+=(KLTParameters.grid_bg*KLTParameters.grid_bg-colBgd.size())*0.75*(1-((col0.R)*(col0.R) + (col0.G)*(col0.G) +
					   (col0.B)*(col0.B))/bwidth);
			probaBgd/=KLTParameters.grid_bg*KLTParameters.grid_bg;*/

			if (probaBgd>0)
			{
				EdataB[i*Icol.getWidth() + j] = -alpha*log(probaBgd) + beta*EdataHB[i*Icol.getWidth() + j];
			}
			else
			{
				EdataB[i*Icol.getWidth() + j] = 0 + beta*EdataHB[i*Icol.getWidth() + j];
			}
			/*if(probaBgd > 0.3)
				label[i*Icol.getWidth() + j] = 0;
			else label[i*Icol.getWidth() + j] = 1;*/

            //#pragma omp parallel for private(data)
			for (int k = 0 ; k < colFgd.size() ; k++)
			{
				data = colFgd[k];
				   dsty = ((col0.R - data[0])*(col0.R - data[0]) + (col0.G - data[1])*(col0.G - data[1]) +
						   (col0.B - data[2])*(col0.B - data[2]))/bwidth  + ((i - data[3])*(i - data[3]) + (j - data[4])*(j - data[4]))/bwidth2;
				   if(dsty < 1)
				   {
					   probaFgd +=  0.75*(1-dsty);}
				   //probaFgd +=dsty;
			}
			probaFgd*=1;
			probaFgd/=colFgd.size();

			if (probaFgd>0)
			{EdataF[i*Icol.getWidth() + j] = -alpha*log(probaFgd) +  beta*EdataHF[i*Icol.getWidth() + j];
			}
			else
			{
				EdataF[i*Icol.getWidth() + j] = 0 + beta*EdataHF[i*Icol.getWidth() + j];
			}

		}
	double t1 = vpTime::measureTimeMs();

	std::cout << " time kernel " << t1 - t0 << std::endl;
}

void apSegMotionCol::computeDataEnergyKernelHistLimb(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol)
{
	//apRANSAC ransac;
	vpHomography H;
	vpHomography Hinv;
	int nFrame = 0;
	double ratio = (double)nbins/256;
	vpImage<vpRGBa> Iprec;
	vpImage<vpRGBa> Idiff(I.getHeight(),I.getWidth());
	vpImage<vpRGBa> Idiff2(I.getHeight(),I.getWidth());
	vpRGBa col0,col1;
	ransac.RANSACFunctionH(trajectory_, nb_trajectory, nbPointToTrack, 0, H, nFrame, labelPts, deltaHomography);
    int frame = max(0,(int)histIm.size()- deltaHomography - 1);
    Iprec = histIm[frame];
	Hinv = H.inverse();
	double i_,j_;
	vpImage<vpRGBa> Icol_(I.getHeight(),I.getWidth());
	vpColVector errorPix(5);
	int *data;
	double probaBgd = 0;
	double probaFgd = 0;
	double probaBgdLimb = 0;
	double probaFgdLimb = 0;
	vpImagePoint ip;

	for (int i = 0; i<Icol.getHeight(); i++)
		for (int j =0 ; j < Icol.getWidth() ; j++)
		{

		    double inv_z = 1. / (Hinv[2][0] * j + Hinv[2][1] * i + Hinv[2][2]);

		    j_ = (Hinv[0][0] * j + Hinv[0][1] * i + Hinv[0][2]) * inv_z;
		    i_ = (Hinv[1][0] * j + Hinv[1][1] * i + Hinv[1][2]) * inv_z;

		    if(i_ > 0 && i_ < Icol.getHeight() && j_>0 && j_<Icol.getWidth())
		    			Icol_[(int)i_][(int)j_] = Icol[i][j];

		}
	/*vpImageIo::writePNG(Icol,"Icol.png");
	vpImageIo::writePNG(Icol_,"Iwarp.png");
	vpImageIo::writePNG(Iprec,"Iprec.png");*/
	for (int i = 0; i<Icol.getHeight(); i++)
		for (int j =0 ; j < Icol.getWidth() ; j++)
		{
			Idiff[i][j].R = (int)(Icol_[i][j].R + Iprec[i][j].R)/2;
			Idiff[i][j].G = (int)(Icol_[i][j].G + Iprec[i][j].G)/2;
			Idiff[i][j].B = (int)(Icol_[i][j].B + Iprec[i][j].B)/2;

			Idiff2[i][j].R = (int)(Icol[i][j].R + Iprec[i][j].R)/2;
			Idiff2[i][j].G = (int)(Icol[i][j].G + Iprec[i][j].G)/2;
			Idiff2[i][j].B = (int)(Icol[i][j].B + Iprec[i][j].B)/2;
		}

	//vpImageIo::writePNG(Idiff,"Idiff.png");
	//vpImageIo::writePNG(Idiff2,"Idiff2.png");

	double density;
	double bwidthH = kernelParameters.bwidth_col_1;

	for (int i = 0; i<Icol.getHeight(); i++)
		for (int j =0 ; j < Icol.getWidth() ; j++)
		{
			probaBgd = 0;
			probaFgd = 0;
			col0 = Icol_[i][j];
			col1 = Iprec[i][j];

			if(col0.R>0 && col0.G > 0 && col0.B > 0)
			{
			errorPix[0] = ((double)col0.R - (double)col1.R);
			errorPix[1]= ((double)col0.G - (double)col1.G);
			errorPix[2] = ((double)col0.B - (double)col1.B);

			   density = 0;
			   for (int k=0;k<3;k++)
				   density += errorPix[k]*errorPix[k]/bwidthH;
			   /*if(density < 1)
			   {
				   density =  0.75*(1-density);}
			   else
				   density = 0;

			   probaBgd += density;*/

			   probaBgd += (1.5/(sqrt(2*M_PI)))*exp(-0.5*density);

				//if(probaBgd > 0.1)
				//{
					//std::cout << EdataF[i*Icol.getWidth() + j]  << std::endl;
					//vpDisplay::displayCross(I, p1, 5, vpColor::black);
				//}
				if (probaBgd>0)
				{
				EdataHB[i*Icol.getWidth() + j] =  -log(probaBgd);
				EdataHF[i*Icol.getWidth() + j] =  -log(1 - probaBgd);
				}
				else {
					EdataHB[i*Icol.getWidth() + j] = 1 ;
					EdataHF[i*Icol.getWidth() + j] = 0 ;
				}
			}
			else{
				EdataHB[i*Icol.getWidth() + j] = 0;
				EdataHF[i*Icol.getWidth() + j] = 1;
			}

		}


	double dsty;
	int l;

	//double eta = 0.08 ;
	//double eta = 0.3 ;
	double alpha = energyParameters.alpha_0;
	double beta = energyParameters.beta_0;
	//double eta1 = 0.1;
	//double eta1 = 20;
	//double eta1 = 1;
	double bwidth = kernelParameters.bwidth_col_0*kernelParameters.bwidth_col_0;
	double bwidth2 = kernelParameters.bwidth_spat*kernelParameters.bwidth_spat;
	vpColVector *data0;

	double t0 = vpTime::measureTimeMs();
	double histfg,histbg;
	int cR,cG,cB;

	double thetaLimb = limb.getTheta();
	double rhoLimb = limb.getRho();
	double rho,rh0,rhoO;

	std::cout << " rhoLimb " << rhoLimb << std::endl;

	for (int i = 0; i<Icol.getHeight(); i++)
		for (int j =0 ; j < Icol.getWidth() ; j++)
		{
			probaBgd = 0;
			probaFgd = 0;
			vpImagePoint p1(i,j);
			col0 = Icol[i][j];

			cR = (int)(col0.R*ratio);
			cG = (int)(col0.G*ratio);
			cB = (int)(col0.B*ratio);

			histbg = (double)(histBgN[0][cR]*histBgN[1][cG]*histBgN[2][cB]);
			histfg = (double)(histFgN[0][cR]*histFgN[1][cG]*histFgN[2][cB]);

            //#pragma omp parallel for private(data)
			for (int k = 0 ; k < colBgd.size() ; k++)
			{
				data = colBgd[k];
				dsty = ((col0.R - data[0])*(col0.R - data[0]) + (col0.G - data[1])*(col0.G - data[1]) +
						   (col0.B - data[2])*(col0.B - data[2]))/bwidth;

				   if(dsty < 1)
				   {
					   probaBgd +=  0.75*(1-dsty);}

				   //probaBgd +=dsty;
				   //probaBgd += (1/(sqrt(2*M_PI)))*exp(-0.5*dsty);

			}

			probaBgd*=1;
			probaBgd/=colBgd.size();
			/*probaBgd+=(KLTParameters.grid_bg*KLTParameters.grid_bg-colBgd.size())*0.75*(1-((col0.R)*(col0.R) + (col0.G)*(col0.G) +
					   (col0.B)*(col0.B))/bwidth);
			probaBgd/=KLTParameters.grid_bg*KLTParameters.grid_bg;*/

			rho = j*cos(thetaLimb) + i*sin(thetaLimb);
			rhoO = j*cos(thetaLimb+M_PI/2) + i*sin(thetaLimb+M_PI/2);
			dsty = (rho-rhoLimb)*(rho-rhoLimb)/300.0;
			double fact=0;
			   if(dsty < 1)
			   {
				if(rhoO > limbo1.getRho() && rhoO < limbo2.getRho())
				{
				   probaFgdLimb = 0.75*(1-dsty);
				   //probaBgdLimb = 1-probaFgdLimb;
				   fact = (1-dsty);
					//probaFgdLimb =  1-(1/(sqrt(2*1000*M_PI)))*exp(-0.5*dsty);
				   if(-log(probaFgdLimb)>2)
				   			{
				   				ip.set_i(i);
				   				ip.set_j(j);
				   				vpDisplay::displayCross(I, ip, 5, vpColor::red);
				   			}
				}
				else
				{
					   probaBgdLimb =  0.75*(1-dsty);
					   //probaFgdLimb = 1-probaBgdLimb;
					   fact = -3*(1+dsty);
					//probaBgdLimb =  1-(1/(sqrt(2*1000*M_PI)))*exp(-0.5*dsty);
				}
			   }
			//probaBgdLimb =  (1/(sqrt(2*1000*M_PI)))*exp(-0.5*dsty);
			/*if(abs(rho-rhoLimb)<30)
			{
				ip.set_i(i);
				ip.set_j(j);
				//vpDisplay::displayCross(I, ip, 5, vpColor::red);
			}*/

			if (probaBgd>0)
			{
				if (probaBgdLimb>0){
				EdataB[i*Icol.getWidth() + j] = -0*alpha*log(probaBgd) + beta*EdataHB[i*Icol.getWidth() + j] -  1*alpha*log(histbg) + fact;// - 1*log(probaBgdLimb);
				//EdataB[i*Icol.getWidth() + j] = probaBgdLimb;
				}
				else{
					EdataB[i*Icol.getWidth() + j] = -0*alpha*log(probaBgd) + beta*EdataHB[i*Icol.getWidth() + j] -  1*alpha*log(histbg);
				}

			}
			else
			{
				if (probaBgdLimb>0){
				EdataB[i*Icol.getWidth() + j] = 0 + beta*EdataHB[i*Icol.getWidth() + j] - alpha*log(histbg) + fact;// - 1*log(probaBgdLimb);
				//EdataB[i*Icol.getWidth() + j] = probaBgdLimb;
				}
				else EdataB[i*Icol.getWidth() + j] = beta*EdataHB[i*Icol.getWidth() + j] - alpha*log(histbg);


			}
			/*if(probaBgd > 0.3)
				label[i*Icol.getWidth() + j] = 0;
			else label[i*Icol.getWidth() + j] = 1;*/

            //#pragma omp parallel for private(data)
			for (int k = 0 ; k < colFgd.size() ; k++)
			{
				data = colFgd[k];
				   dsty = ((col0.R - data[0])*(col0.R - data[0]) + (col0.G - data[1])*(col0.G - data[1]) +
						   (col0.B - data[2])*(col0.B - data[2]))/bwidth  + ((i - data[3])*(i - data[3]) + (j - data[4])*(j - data[4]))/bwidth2;
				   if(dsty < 1)
				   {
					   probaFgd +=  0.75*(1-dsty);}
				   //probaFgd +=dsty;
			}
			probaFgd*=1;
			probaFgd/=colFgd.size();

			if (probaFgd>0)
			{
				if (probaFgdLimb>0){
				EdataF[i*Icol.getWidth() + j] = -0*alpha*log(probaFgd) +  beta*EdataHF[i*Icol.getWidth() + j] - alpha*log(histfg)-fact;// - log(probaFgdLimb);
				}
				else {
					EdataF[i*Icol.getWidth() + j] = -0*alpha*log(probaFgd) +  beta*EdataHF[i*Icol.getWidth() + j] - alpha*log(histfg);
				}

			}
			else
			{
				if (probaFgdLimb>0){
				EdataF[i*Icol.getWidth() + j] = 0 + beta*EdataHF[i*Icol.getWidth() + j] - alpha*log(histfg)-fact;// - log(probaFgdLimb);
				}
				else{
					EdataF[i*Icol.getWidth() + j] = 0 + beta*EdataHF[i*Icol.getWidth() + j] - alpha*log(histfg);
				}
			}

		}
	double t1 = vpTime::measureTimeMs();

	std::cout << " time kernel " << t1 - t0 << std::endl;
}

void apSegMotionCol::computeDataEnergyKernel(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol)
{
	//apRANSAC ransac;
	vpHomography H;
	vpHomography Hinv;
	int nFrame = 0;
	double ratio = (double)nbins/256;
	vpImage<vpRGBa> Iprec;
	vpImage<vpRGBa> Idiff(I.getHeight(),I.getWidth());
	vpImage<vpRGBa> Idiff2(I.getHeight(),I.getWidth());
	vpRGBa col0,col1;
	ransac.RANSACFunctionH(trajectory_, nb_trajectory, nbPointToTrack, 0, H, nFrame, labelPts, deltaHomography);
    int frame = max(0,(int)histIm.size()- deltaHomography - 1);
    Iprec = histIm[frame];
	Hinv = H.inverse();
	double i_,j_;
	vpImage<vpRGBa> Icol_(I.getHeight(),I.getWidth());
	Icol_ = Iprec;
	vpColVector errorPix(5);
	int *data;
	double probaBgd = 0;
	double probaFgd = 0;

	for (int i = 0; i<Icol.getHeight(); i++)
		for (int j =0 ; j < Icol.getWidth() ; j++)
		{

		    double inv_z = 1. / (Hinv[2][0] * j + Hinv[2][1] * i + Hinv[2][2]);

		    j_ = (Hinv[0][0] * j + Hinv[0][1] * i + Hinv[0][2]) * inv_z;
		    i_ = (Hinv[1][0] * j + Hinv[1][1] * i + Hinv[1][2]) * inv_z;

		    if(i_ > 0 && i_ < Icol.getHeight() && j_>0 && j_<Icol.getWidth())
		    			Icol_[(int)i_][(int)j_] = Icol[i][j];

		}
	/*vpImageIo::writePNG(Icol,"Icol.png");
	vpImageIo::writePNG(Icol_,"Iwarp.png");
	vpImageIo::writePNG(Iprec,"Iprec.png");*/
	/*for (int i = 0; i<Icol.getHeight(); i++)
		for (int j =0 ; j < Icol.getWidth() ; j++)
		{
			Idiff[i][j].R = (int)(Icol_[i][j].R + Iprec[i][j].R)/2;
			Idiff[i][j].G = (int)(Icol_[i][j].G + Iprec[i][j].G)/2;
			Idiff[i][j].B = (int)(Icol_[i][j].B + Iprec[i][j].B)/2;

			Idiff2[i][j].R = (int)(Icol[i][j].R + Iprec[i][j].R)/2;
			Idiff2[i][j].G = (int)(Icol[i][j].G + Iprec[i][j].G)/2;
			Idiff2[i][j].B = (int)(Icol[i][j].B + Iprec[i][j].B)/2;
		}

	vpImageIo::writePNG(Idiff,"Idiff1.png");
	vpImageIo::writePNG(Idiff2,"Idiff2.png");*/

	double density;
	double bwidthH = kernelParameters.bwidth_col_1;

	for (int i = 0; i<Icol.getHeight(); i++)
		for (int j =0 ; j < Icol.getWidth() ; j++)
		{
			probaBgd = 0;
			probaFgd = 0;
			col0 = Icol_[i][j];
			col1 = Iprec[i][j];

			if(col0.R>0 && col0.G > 0 && col0.B > 0)// && i>20 && i < Icol.getHeight()-20 && j>20 && j < Icol.getHeight()-20)
			{
			errorPix[0] = ((double)col0.R - (double)col1.R);
			errorPix[1]= ((double)col0.G - (double)col1.G);
			errorPix[2] = ((double)col0.B - (double)col1.B);

			   density = 0;
			   for (int k=0;k<3;k++)
				   density += errorPix[k]*errorPix[k]/bwidthH;
				   //density += errorPix[k]*errorPix[k]/1;
			   /*if(density < 1)
			   {
				   density =  0.75*(1-density);}
			   else
				   density = 0;

			   probaBgd += density;*/


			    //if(density > 0.5)
			   	   //density = 0;

				   probaBgd += (1.5/(sqrt(2*M_PI)))*exp(-0.5*density);


				   /*if(probaBgd > 0.3)
				   {
				    Idiff2[i][j].R = 0;
					Idiff2[i][j].G = 0;
					Idiff2[i][j].B = 0;
				   }
				   else
				   {
					Idiff2[i][j].R = Icol[i][j].R;
					Idiff2[i][j].G = Icol[i][j].G;
					Idiff2[i][j].B = Icol[i][j].B;
				   }*/

				if (probaBgd>0)
				{
				EdataHB[i*Icol.getWidth() + j] =  -log(probaBgd);
				EdataHF[i*Icol.getWidth() + j] =  -log(1 - probaBgd);
				}
				else {
					EdataHB[i*Icol.getWidth() + j] = 1 ;
					EdataHF[i*Icol.getWidth() + j] = 0 ;
				}
			}
			else{
				EdataHB[i*Icol.getWidth() + j] = 0;
				EdataHF[i*Icol.getWidth() + j] = 1;
			}

		}

	//vpImageIo::writePNG(Idiff2,"Idiff3.png");


	double dsty;
	int l;

	//double eta = 0.08 ;
	//double eta = 0.3 ;
	double alpha = energyParameters.alpha_0;
	double beta = energyParameters.beta_0;
	//double eta1 = 0.1;
	//double eta1 = 20;
	//double eta1 = 1;
	double bwidth = kernelParameters.bwidth_col_0*kernelParameters.bwidth_col_0;
	double bwidth2 = kernelParameters.bwidth_spat*kernelParameters.bwidth_spat;
	vpColVector *data0;

	double t0 = vpTime::measureTimeMs();

	for (int i = 0; i<Icol.getHeight(); i++)
		for (int j =0 ; j < Icol.getWidth() ; j++)
		{
			probaBgd = 0;
			probaFgd = 0;
			vpImagePoint p1(i,j);
			col0 = Icol[i][j];

            //#pragma omp parallel for private(data)
			for (int k = 0 ; k < colBgd.size() ; k++)
			{
				data = colBgd[k];
				dsty = ((col0.R - data[0])*(col0.R - data[0]) + (col0.G - data[1])*(col0.G - data[1]) +
						   (col0.B - data[2])*(col0.B - data[2]))/bwidth;//  + ((i - data[3])*(i - data[3]) + (j - data[4])*(j - data[4]))/bwidth2;

				   if(dsty < 1)
				   {
					   probaBgd +=  0.75*(1-dsty);
				   }

				   //probaBgd +=dsty;
				   //probaBgd += (2/(sqrt(2*M_PI)))*exp(-0.5*dsty);

			}
			probaBgd*=1;
			probaBgd/=colBgd.size();

			if (probaBgd>0)
			{
				EdataB[i*Icol.getWidth() + j] = -alpha*log(probaBgd) + beta*EdataHB[i*Icol.getWidth() + j];
				//std::cout << " " << -alpha*log(probaBgd) << " H "<< beta*EdataHB[i*Icol.getWidth() + j] << std::endl;

			}
			else
			{
				EdataB[i*Icol.getWidth() + j] = 0 + beta*EdataHB[i*Icol.getWidth() + j];
			}
			/*if(probaBgd > 0.3)
				label[i*Icol.getWidth() + j] = 0;
			else label[i*Icol.getWidth() + j] = 1;*/

            //#pragma omp parallel for private(data)
			for (int k = 0 ; k < colFgd.size() ; k++)
			{
				data = colFgd[k];
				   dsty = ((col0.R - data[0])*(col0.R - data[0]) + (col0.G - data[1])*(col0.G - data[1]) +
						   (col0.B - data[2])*(col0.B - data[2]))/bwidth  + ((i - data[3])*(i - data[3]) + (j - data[4])*(j - data[4]))/bwidth2;
				   if(dsty < 1)
				   {
					   probaFgd +=  0.75*(1-dsty);}
				   //probaFgd +=dsty;
				   //probaFgd += (1/(sqrt(2*M_PI)))*exp(-0.5*dsty);

			}
			probaFgd*=1;
			probaFgd/=colFgd.size();

			   if(probaBgd > 0.5)
			   {
			    Idiff2[i][j].R = 0;
				Idiff2[i][j].G = 0;
				Idiff2[i][j].B = 0;
			   }
			   else
			   {
				Idiff2[i][j].R = Icol[i][j].R;
				Idiff2[i][j].G = Icol[i][j].G;
				Idiff2[i][j].B = Icol[i][j].B;
			   }

			   if(probaFgd < 0.5)
			   {
			    Idiff[i][j].R = 0;
				Idiff[i][j].G = 0;
				Idiff[i][j].B = 0;
			   }
			   else
			   {
				Idiff[i][j].R = Icol[i][j].R;
				Idiff[i][j].G = Icol[i][j].G;
				Idiff[i][j].B = Icol[i][j].B;
			   }

			if (probaFgd>0)
			{EdataF[i*Icol.getWidth() + j] = -alpha*log(probaFgd) +  beta*EdataHF[i*Icol.getWidth() + j];
			}
			else
			{
				EdataF[i*Icol.getWidth() + j] = 0 + beta*EdataHF[i*Icol.getWidth() + j];
			}

		}
/*
	for (int k = 0 ; k < colBgd.size() ; k++)
	{
		data = colBgd[k];
		std::cout << "Fg "<< data[0] << " " << data[1] << " " << data[2] << std::endl;
	}
	for (int k = 0 ; k < colFgd.size() ; k++)
	{
		data = colFgd[k];
		std::cout << "Bg " << data[0] << " " << data[1] << " " << data[2] << std::endl;

	}*/

	vpImageIo::writePNG(Idiff2,"Idiff4.png");
	vpImageIo::writePNG(Idiff,"Idiff5.png");


	double t1 = vpTime::measureTimeMs();

	std::cout << " time kernel " << t1 - t0 << std::endl;
}

void apSegMotionCol::computeDataEnergyKernelHist(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol)
{
	//apRANSAC ransac;
	vpHomography H;
	vpHomography Hinv;
	int nFrame = 0;
	double ratio = (double)nbins/256;
	vpImage<vpRGBa> Iprec;
	vpImage<vpRGBa> Idiff(I.getHeight(),I.getWidth());
	vpImage<vpRGBa> Idiff2(I.getHeight(),I.getWidth());
	vpRGBa col0,col1;
	ransac.RANSACFunctionH(trajectory_, nb_trajectory, nbPointToTrack, 0, H, nFrame, labelPts, deltaHomography);
    int frame = max(0,(int)histIm.size()- deltaHomography - 1);
    Iprec = histIm[frame];
	Hinv = H.inverse();
	double i_,j_;
	vpImage<vpRGBa> Icol_(I.getHeight(),I.getWidth());
	Icol_ = Iprec;
	vpColVector errorPix(5);
	int *data;
	double probaBgd = 0;
	double probaFgd = 0;

	double histfg,histbg;


	int cR,cG,cB;


	for (int i = 0; i<Icol.getHeight(); i++)
		for (int j =0 ; j < Icol.getWidth() ; j++)
		{

		    double inv_z = 1. / (Hinv[2][0] * j + Hinv[2][1] * i + Hinv[2][2]);

		    j_ = (Hinv[0][0] * j + Hinv[0][1] * i + Hinv[0][2]) * inv_z;
		    i_ = (Hinv[1][0] * j + Hinv[1][1] * i + Hinv[1][2]) * inv_z;

		    if(i_ > 0 && i_ < Icol.getHeight() && j_>0 && j_<Icol.getWidth())
		    			Icol_[(int)i_][(int)j_] = Icol[i][j];

		}
	/*vpImageIo::writePNG(Icol,"Icol.png");
	vpImageIo::writePNG(Icol_,"Iwarp.png");
	vpImageIo::writePNG(Iprec,"Iprec.png");*/
	/*for (int i = 0; i<Icol.getHeight(); i++)
		for (int j =0 ; j < Icol.getWidth() ; j++)
		{
			Idiff[i][j].R = (int)(Icol_[i][j].R + Iprec[i][j].R)/2;
			Idiff[i][j].G = (int)(Icol_[i][j].G + Iprec[i][j].G)/2;
			Idiff[i][j].B = (int)(Icol_[i][j].B + Iprec[i][j].B)/2;

			Idiff2[i][j].R = (int)(Icol[i][j].R + Iprec[i][j].R)/2;
			Idiff2[i][j].G = (int)(Icol[i][j].G + Iprec[i][j].G)/2;
			Idiff2[i][j].B = (int)(Icol[i][j].B + Iprec[i][j].B)/2;
		}

	vpImageIo::writePNG(Idiff,"Idiff1.png");
	vpImageIo::writePNG(Idiff2,"Idiff2.png");*/

	double density;
	double bwidthH = kernelParameters.bwidth_col_1;

	for (int i = 0; i<Icol.getHeight(); i++)
		for (int j =0 ; j < Icol.getWidth() ; j++)
		{
			probaBgd = 0;
			probaFgd = 0;
			col0 = Icol_[i][j];
			col1 = Iprec[i][j];

			if(col0.R>0 && col0.G > 0 && col0.B > 0)// && i>20 && i < Icol.getHeight()-20 && j>20 && j < Icol.getHeight()-20)
			{
			errorPix[0] = ((double)col0.R - (double)col1.R);
			errorPix[1]= ((double)col0.G - (double)col1.G);
			errorPix[2] = ((double)col0.B - (double)col1.B);

			   density = 0;
			   for (int k=0;k<3;k++)
				   density += errorPix[k]*errorPix[k]/bwidthH;
				   //density += errorPix[k]*errorPix[k]/1;
			   /*if(density < 1)
			   {
				   density =  0.75*(1-density);}
			   else
				   density = 0;

			   probaBgd += density;*/


			    //if(density > 0.5)
			   	   //density = 0;

				   probaBgd += (1.5/(sqrt(2*M_PI)))*exp(-0.5*density);


				   /*if(probaBgd > 0.3)
				   {
				    Idiff2[i][j].R = 0;
					Idiff2[i][j].G = 0;
					Idiff2[i][j].B = 0;
				   }
				   else
				   {
					Idiff2[i][j].R = Icol[i][j].R;
					Idiff2[i][j].G = Icol[i][j].G;
					Idiff2[i][j].B = Icol[i][j].B;
				   }*/

				if (probaBgd>0)
				{
				EdataHB[i*Icol.getWidth() + j] =  -log(probaBgd);
				EdataHF[i*Icol.getWidth() + j] =  -log(1 - probaBgd);
				}
				else {
					EdataHB[i*Icol.getWidth() + j] = 1 ;
					EdataHF[i*Icol.getWidth() + j] = 0 ;
				}
			}
			else{
				EdataHB[i*Icol.getWidth() + j] = 0;
				EdataHF[i*Icol.getWidth() + j] = 1;
			}

		}

	//vpImageIo::writePNG(Idiff2,"Idiff3.png");


	double dsty;
	int l;

	//double eta = 0.08 ;
	//double eta = 0.3 ;
	double alpha = energyParameters.alpha_0;
	double beta = energyParameters.beta_0;
	//double eta1 = 0.1;
	//double eta1 = 20;
	//double eta1 = 1;
	double bwidth = kernelParameters.bwidth_col_0*kernelParameters.bwidth_col_0;
	double bwidth2 = kernelParameters.bwidth_spat*kernelParameters.bwidth_spat;
	vpColVector *data0;

	double t0 = vpTime::measureTimeMs();

	for (int i = 0; i<Icol.getHeight(); i++)
		for (int j =0 ; j < Icol.getWidth() ; j++)
		{
			probaBgd = 0;
			probaFgd = 0;
			vpImagePoint p1(i,j);
			col0 = Icol[i][j];

			cR = (int)(col0.R*ratio);
			cG = (int)(col0.G*ratio);
			cB = (int)(col0.B*ratio);

			histbg = (double)(histBgN[0][cR]*histBgN[1][cG]*histBgN[2][cB]);
			histfg = (double)(histFgN[0][cR]*histFgN[1][cG]*histFgN[2][cB]);

            //#pragma omp parallel for private(data)
			for (int k = 0 ; k < colBgd.size() ; k++)
			{
				data = colBgd[k];
				dsty = ((col0.R - data[0])*(col0.R - data[0]) + (col0.G - data[1])*(col0.G - data[1]) +
						   (col0.B - data[2])*(col0.B - data[2]))/bwidth;//  + ((i - data[3])*(i - data[3]) + (j - data[4])*(j - data[4]))/bwidth2;

				   if(dsty < 1)
				   {
					   probaBgd +=  0.75*(1-dsty);
				   }

				   //probaBgd +=dsty;
				   //probaBgd += (2/(sqrt(2*M_PI)))*exp(-0.5*dsty);

			}
			probaBgd*=1;
			probaBgd/=colBgd.size();

			if (probaBgd>0)
			{
				EdataB[i*Icol.getWidth() + j] = -alpha*log(probaBgd) + beta*EdataHB[i*Icol.getWidth() + j]  - 0.2*log(histbg);
				//std::cout << " " << -alpha*log(probaBgd) << " H "<< beta*EdataHB[i*Icol.getWidth() + j] << std::endl;

			}
			else
			{
				EdataB[i*Icol.getWidth() + j] = 0 + beta*EdataHB[i*Icol.getWidth() + j] - 0.2*log(histbg);
			}
			/*if(probaBgd > 0.3)
				label[i*Icol.getWidth() + j] = 0;
			else label[i*Icol.getWidth() + j] = 1;*/

            //#pragma omp parallel for private(data)
			for (int k = 0 ; k < colFgd.size() ; k++)
			{
				data = colFgd[k];
				   dsty = ((col0.R - data[0])*(col0.R - data[0]) + (col0.G - data[1])*(col0.G - data[1]) +
						   (col0.B - data[2])*(col0.B - data[2]))/bwidth  + ((i - data[3])*(i - data[3]) + (j - data[4])*(j - data[4]))/bwidth2;
				   if(dsty < 1)
				   {
					   probaFgd +=  0.75*(1-dsty);}
				   //probaFgd +=dsty;
				   //probaFgd += (1/(sqrt(2*M_PI)))*exp(-0.5*dsty);

			}
			probaFgd*=1;
			probaFgd/=colFgd.size();

			   if(probaBgd > 0.5)
			   {
			    Idiff2[i][j].R = 0;
				Idiff2[i][j].G = 0;
				Idiff2[i][j].B = 0;
			   }
			   else
			   {
				Idiff2[i][j].R = Icol[i][j].R;
				Idiff2[i][j].G = Icol[i][j].G;
				Idiff2[i][j].B = Icol[i][j].B;
			   }

			   if(probaFgd < 0.5)
			   {
			    Idiff[i][j].R = 0;
				Idiff[i][j].G = 0;
				Idiff[i][j].B = 0;
			   }
			   else
			   {
				Idiff[i][j].R = Icol[i][j].R;
				Idiff[i][j].G = Icol[i][j].G;
				Idiff[i][j].B = Icol[i][j].B;
			   }

			if (probaFgd>0)
			{EdataF[i*Icol.getWidth() + j] = -alpha*log(probaFgd) +  beta*EdataHF[i*Icol.getWidth() + j] - 0.2*log(histfg);
			}
			else
			{
				EdataF[i*Icol.getWidth() + j] = 0 + beta*EdataHF[i*Icol.getWidth() + j] - 0.2*log(histfg);
			}

		}
/*
	for (int k = 0 ; k < colBgd.size() ; k++)
	{
		data = colBgd[k];
		std::cout << "Fg "<< data[0] << " " << data[1] << " " << data[2] << std::endl;
	}
	for (int k = 0 ; k < colFgd.size() ; k++)
	{
		data = colFgd[k];
		std::cout << "Bg " << data[0] << " " << data[1] << " " << data[2] << std::endl;

	}*/

	vpImageIo::writePNG(Idiff2,"Idiff4.png");
	vpImageIo::writePNG(Idiff,"Idiff5.png");


	double t1 = vpTime::measureTimeMs();

	std::cout << " time kernel " << t1 - t0 << std::endl;
}




void apSegMotionCol::computeDataEnergyHist(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol)
{
	double t0 = vpTime::measureTimeMs();
	//apRANSAC ransac;
	vpHomography H;
	vpHomography Hinv;
	int nFrame = 0;
	vpImage<vpRGBa> Iprec;
	vpImage<vpRGBa> Idiff2(I.getHeight(),I.getWidth());
	vpRGBa col0,col1;
	int height = I.getHeight();
	int width = I.getWidth();
	double ratio = (double)nbins/256;
	ransac.RANSACFunctionH(trajectory_, nb_trajectory, nbPointToTrack, 0, H, nFrame, labelPts, deltaHomography);

    int frame = max(0,(int)histIm.size()- deltaHomography - 1);
    Iprec = histIm[frame];
	Hinv = H.inverse();
	double i_,j_;
	vpImage<vpRGBa> Icol_(width,height);
	Icol_ = Iprec;
	int errorPix[3];
	double probaBgd = 0;
	double probaFgd = 0;

	double alpha = energyParameters.alpha;
	double beta = energyParameters.beta;
	double bwidth = kernelParameters.bwidth_col_0*kernelParameters.bwidth_col_0;
	double bwidthH = kernelParameters.bwidth_col_1;
	double histfg,histbg;
	int cR,cG,cB;
	for (int i = 0; i<height; i++)
		for (int j =0 ; j < width ; j++)
		{

		    double inv_z = 1. / (Hinv[2][0] * j + Hinv[2][1] * i + Hinv[2][2]);

		    j_ = (Hinv[0][0] * j + Hinv[0][1] * i + Hinv[0][2]) * inv_z;
		    i_ = (Hinv[1][0] * j + Hinv[1][1] * i + Hinv[1][2]) * inv_z;

		    if(i_ > 0 && i_ < height && j_>0 && j_<width)
		    			Icol_[(int)i_][(int)j_] = Icol[i][j];
		}

	for (int i = 0; i<height; i++)
		for (int j =0 ; j < width ; j++)
		{
			probaBgd = 0;
			col0 = Icol_[i][j];
			col1 = Iprec[i][j];

			if(col0.R>0 && col0.G > 0 && col0.B > 0)
			{
			errorPix[0] = col0.R - col1.R;
			errorPix[1]= col0.G - col1.G;
			errorPix[2] = col0.B - col1.B;

			   double density = 0;

			   for (int k=0;k<3;k++)
				   density += errorPix[k]*errorPix[k]/bwidthH;
				   //density += errorPix[k]*errorPix[k]/1;
			   /*if(density < 1)
			   {
				   density =  0.75*(1-density);}
			   else
				   density = 0;

			   probaBgd += density;*/


			    //if(density > 0.5)
			   	   //density = 0;

				   probaBgd += (1.5/(sqrt(2*M_PI)))*exp(-0.5*density);


				   if(probaBgd > 0.3)
				   {
				    Idiff2[i][j].R = 0;
					Idiff2[i][j].G = 0;
					Idiff2[i][j].B = 0;
				   }
				   else
				   {
					Idiff2[i][j].R = Icol[i][j].R;
					Idiff2[i][j].G = Icol[i][j].G;
					Idiff2[i][j].B = Icol[i][j].B;
				   }

				if (probaBgd>0)
				{

				EdataHB[i*width + j] =  -log(probaBgd);
				EdataHF[i*width + j] =  -log(1 - probaBgd);
				}
				else {
					EdataHB[i*width + j] = 1 ;
					EdataHF[i*width + j] = 0 ;
				}
			}
			else{
				EdataHB[i*width + j] = 0;
				EdataHF[i*width + j] = 1;
			}

			col0 = Icol[i][j];
			cR = (int)(col0.R*ratio);
			cG = (int)(col0.G*ratio);
			cB = (int)(col0.B*ratio);

			histbg = (double)(histBgN[0][cR]*histBgN[1][cG]*histBgN[2][cB]);
			histfg = (double)(histFgN[0][cR]*histFgN[1][cG]*histFgN[2][cB]);

			EdataB[i*width + j] = - alpha*log(histbg) + beta*EdataHB[i*width + j];
			EdataF[i*width + j] = - alpha*log(histfg) + beta*EdataHF[i*width + j];
		}

	vpImageIo::writePNG(Idiff2,"Idiff3.png");


	double t1 = vpTime::measureTimeMs();
	std::cout << " time kernel " << t1 - t0 << std::endl;
}

void apSegMotionCol::computeDataEnergyHistInitDeepSpace(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol)
{

	double ratio = (double)nbins/256;

    for(int l =0; l<3; l++)
    {
    	histFgN[l].resize(nbins);
    	histBgN[l].resize(nbins);
    }

    for(int l =0; l<3; l++)
    {
        for (int k = 0; k<nbins; k++)
        {
       	 histBg[l].set(k,10);
        }
	 histBg[l].set(0,100000);
	 /*histBg[l].set(1,5000);
	 histBg[l].set(2,3000);
	 histBg[l].set(3,2000);
	 histBg[l].set(4,1500);
	 histBg[l].set(5,1000);*/
     histBg[l].smooth(5);
     for (int k = 0; k<nbins; k++)
     {
    	 histBgN[l][k] = (double)(histBg[l].get(k)/100000.0);
	     //std::cout << " height " << histBgN[l][k] << " width " << histBgN[l][k] << std::endl;
     }

    }

	double t0 = vpTime::measureTimeMs();

	vpRGBa col0,col1;
	int height = I.getHeight();
	int width = I.getWidth();

	int errorPix[3];
	double probaBgd = 0;
	double probaFgd = 0;

	double alpha = energyParameters.alpha;
	double beta = energyParameters.beta;
	double bwidth = kernelParameters.bwidth_col_0*kernelParameters.bwidth_col_0;

	double histfg,histbg;
	int cR,cG,cB;

	int frame = max(0,(int)histIm.size()- deltaHomography - 1);
	vpImage<vpRGBa> Iprec = histIm[frame];

	vpImage<vpRGBa> Ithresh;
	Ithresh = Icol;


	for (int i = 0; i<height; i++)
		for (int j =0 ; j < width ; j++)
		{

			probaBgd = 0;
			col0 = Icol[i][j];
			col1 = Iprec[i][j];

			if(col0.R>0 && col0.G > 0 && col0.B > 0)
			{
			errorPix[0] = col0.R - col1.R;
			errorPix[1]= col0.G - col1.G;
			errorPix[2] = col0.B - col1.B;

			   double density = 0;
			   for (int k=0;k<3;k++)
				   density += (double)errorPix[k]*errorPix[k]/2000;
			   if(density < 1)
			   {
				   density =  0.75*(1-density);}
			   else
				   density = 0;

			   probaBgd += density;

				if (probaBgd>0)
				{

				EdataHB[i*width + j] =  -log(probaBgd);
				EdataHF[i*width + j] =  -log(1 - probaBgd);
				}
				else {
					EdataHB[i*width + j] = 1 ;
					EdataHF[i*width + j] = 0 ;
				}
			}
			else{
				EdataHB[i*width + j] = 0;
				EdataHF[i*width + j] = 1;
			}

			col0 = Icol[i][j];
			cR = (int)(col0.R*ratio);
			cG = (int)(col0.G*ratio);
			cB = (int)(col0.B*ratio);

			histbg = (double)(histBgN[0][cR]*histBgN[1][cG]*histBgN[2][cB]);


			//EdataB[i*width + j] = - 0.3*log(1-histbg);// + beta*EdataHB[i*width + j];
			//EdataF[i*width + j] = - 0.3*log(histbg);// + beta*EdataHF[i*width + j];




			//std::cout << " edb " << EdataB[i*width + j] << " edf " << EdataF[i*width + j] << std::endl;

			if(Icol[i][j].R < 30 && Icol[i][j].G < 30 && Icol[i][j].B < 30)
			{
				Ithresh[i][j].R = 255;
				Ithresh[i][j].G = 255;
				Ithresh[i][j].B = 255;
				EdataB[i*width + j] = 0.1;
				EdataF[i*width + j] = 2;
			}
			else
			{
				EdataB[i*width + j] = 2;
				EdataF[i*width + j] = 0.1;
			}
		}

	 vpImageIo::writePNG(Ithresh, "ithresh.png");

	double t1 = vpTime::measureTimeMs();
	std::cout << " time kernel " << t1 - t0 << std::endl;
}


void apSegMotionCol::computeDataEnergyHistDeepSpace(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol)
{
	double t0 = vpTime::measureTimeMs();

	int nFrame = 0;
	vpImage<vpRGBa> Iprec;
	vpRGBa col0,col1;
	int height = I.getHeight();
	int width = I.getWidth();
	double ratio = (double)nbins/256;

	double alpha = energyParameters.alpha;
	double beta = energyParameters.beta;
	double bwidth = kernelParameters.bwidth_col_0*kernelParameters.bwidth_col_0;
	double histfg,histbg;
	int cR,cG,cB;

	for (int i = 0; i<height; i++)
		for (int j =0 ; j < width ; j++)
		{


			col0 = Icol[i][j];
			cR = (int)(col0.R*ratio);
			cG = (int)(col0.G*ratio);
			cB = (int)(col0.B*ratio);

			histbg = (double)(histBgN[0][cR]*histBgN[1][cG]*histBgN[2][cB]);
			histfg = (double)(histFgN[0][cR]*histFgN[1][cG]*histFgN[2][cB]);

			EdataB[i*width + j] = - alpha*log(histbg);
			EdataF[i*width + j] = - alpha*log(histfg);
		}

	double t1 = vpTime::measureTimeMs();
	std::cout << " time kernel " << t1 - t0 << std::endl;
}


void apSegMotionCol::computeEnergyHist(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol)
{
	vpColVector errorPix(5);
	int *data;
	double probaBgd = 0;
	double probaFgd = 0;
	double dsty;
	int l;

	double eta = 2;

	double t0 = vpTime::measureTimeMs();
	for (int i = 0; i<Icol.getHeight(); i++)
		for (int j =0 ; j < Icol.getWidth() ; j++)
		{

			EdataB[i*Icol.getWidth() + j] = abs(eta*(log(1-(double)(histBg[0].get((int)Icol[i][j].R))/nbg) +
					log(1-(double)(histBg[1].get((int)Icol[i][j].G))/nbg) +
					log(1-(double)(histBg[2].get((int)Icol[i][j].B))/nbg)));

			EdataF[i*Icol.getWidth() + j] = abs(eta*(log(1-(double)(histFg[0].get((int)Icol[i][j].R))/nfg) +
					log(1-(double)(histFg[1].get((int)Icol[i][j].G))/nfg) +
					log(1-(double)(histFg[2].get((int)Icol[i][j].B))/nfg)));

		}
	double t1 = vpTime::measureTimeMs();
}


void apSegMotionCol::computeSpatialEnergy(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol)
{
	double esmooth,esmoothH,esmoothV;
	double gamma;
	double mu;
	double dist[6];
	dist[0] = 1;
	dist[1] = sqrt(2);
	dist[2] = 1;
	dist[3] = 1;
	dist[4] = sqrt(2);
	dist[5] = 1;
	int distColV[6];
	int distColH[6];
	double meandistH,meandistV;
	int meandisth,meandistv;
	int count;
	vpRGBa col0,col1;
	int height = I.getHeight();
	int width = I.getWidth();
	double esmoothh[width*height];
	double esmoothv[width*height];
	int k,l;

    #pragma omp parallel for private(esmoothH,esmoothV,meandistv,count,k,l,meandisth,col0,col1,distColV,distColH,meandistV,meandistH) shared(esmoothh,esmoothv)
	for (int i = 0; i < height; i++)
	{
		for (int j =0 ; j < width ; j++)
		{
			if(j>1 && j<width-2 && i>1 && i<height-2 )
		{
		esmoothH = 0;
		esmoothV = 0;
		//meandistV = 0;
		//meandistH = 0;
		meandistv = 0;
		meandisth = 0;
		count = 0;
		//meandist = 0;
		for ( k = -1 ; k < 2 ; k++)
			for ( l = -1 ; l < 2 ; l++)
				{
				if(l!=k)
					{
						//dist[count] = sqrt((k-l)*(k-l));
						col0 = Icol[i+k][j];
						col1 = Icol[i+l][j];
						distColV[count] = (col0.R - col1.R)*(col0.R - col1.R) +
								(col0.G - col1.G)*(col0.G - col1.G)
								+ (col0.B - col1.B)*(col0.B - col1.B);
						col0 = Icol[i][j+k];
						col1 = Icol[i][j+l];
						distColH[count] = (col0.R - col1.R)*(col0.R -col1.R) +
								(col0.G - col1.G)*(col0.G - col1.G)
								+ (col0.B -col1.B)*(col0.B -col1.B);
						meandistv += distColV[count];
						meandisth += distColH[count];
						count++;
					}
				}

		meandistV = (double)meandistv/3;
		meandistH = (double)meandisth/3;


		for (int l = 0;l < 6 ; l++)
		{
			esmoothV += (exp(-(double)distColV[l]/meandistV))/dist[l];
			esmoothH += (exp(-(double)distColH[l]/meandistH))/dist[l];
		}
		esmoothh[i*width + j] = 0.8*esmoothH;
		esmoothv[i*width + j] = 0.8*esmoothV;
		}
			else
			{
				esmoothh[i*width + j] = 0;
				esmoothv[i*width + j] = 0;
			}

		}
	}

	for (int i = 0; i < height; i++)
		for (int j =0 ; j < width ; j++)
		{
			EsmoothH[i*width + j] = esmoothh[i*width + j];
			EsmoothV[i*width + j] = esmoothv[i*width + j];
		}
	/*delete [] dist;
	delete [] distColV;
	delete [] distColH;*/
}

/*void apSegMotionCol::minimizeEnergy(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol)
{
	//minEnergy.minEnergy(EdataB,EdataF,EsmoothH,EsmoothV,I);
	MRF::CostVal *Def = new MRF::CostVal[resolution];
	MRF::CostVal *Deb = new MRF::CostVal[resolution];
	unsigned char* R = new unsigned char[resolution];
	unsigned char* G = new unsigned char[resolution];
	unsigned char* B = new unsigned char[resolution];
	unsigned char* mask = new unsigned char[resolution];
	for (int k =0; k<I.getHeight() ; k++)
		for (int l =0; l<I.getWidth() ; l++)
	{
		Def[k*I.getWidth() + l] = (double)(EdataF[k*I.getWidth() + l] + EdataHistF[k*I.getWidth() + l]);
		Deb[k*I.getWidth() + l] = (double)(EdataB[k*I.getWidth() + l] + EdataHistB[k*I.getWidth() + l]);
		R[k*I.getWidth() + l] = Icol[k][l].R;
		G[k*I.getWidth() + l] = Icol[k][l].G;
		B[k*I.getWidth() + l] = Icol[k][l].B;
		//std::cout << (int)R[k*I.getWidth() + l] << " Deb " << (int)G[k*I.getWidth() + l] << std::endl;
		mask[k*I.getWidth() + l] = (unsigned char)label[k*I.getWidth() + l] ;
	}
	minEnergy.minEnergy(Def,Deb,R,G,B,mask,I.getHeight(),I.getWidth(), I);
	for (int l =0; l<resolution ; l++)
{
		label[l] = (int)mask[l];
}
	vpImagePoint p1;*/

	/*int *mask_;
	mask_ = minEnergy.mask;*/
	/*for (int i = 0; i<I.getHeight(); i++)
		for (int j =0 ; j < I.getWidth() ; j++)
		{
			p1.set_i(i);
			p1.set_j(j);
			//std::cout << minEnergy.mask[i*I.getWidth() + j] << std::endl;
			if(EdataHistB[i*Icol.getWidth() + j] > 2000)
				vpDisplay::displayCross(I, p1, 5, vpColor::red);
		}*/
//}

void apSegMotionCol::minimizeEnergy(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol)
{
	unsigned char* mask = new unsigned char[resolution];
	minEnergy.minEnergy(EdataB,EdataF,EsmoothH,EsmoothV, mask, I);
	for (int l =0; l<resolution ; l++)
	{
		label[l] = (int)mask[l];
	}

	vpImagePoint p1;

	/*int *mask_;
	mask_ = minEnergy.mask;*/
	/*for (int i = 0; i<I.getHeight(); i++)
		for (int j =0 ; j < I.getWidth() ; j++)
		{
			p1.set_i(i);
			p1.set_j(j);
			//std::cout << minEnergy.mask[i*I.getWidth() + j] << std::endl;
			if(label[i*I.getWidth() + j] == 0)
				vpDisplay::displayCross(I, p1, 5, vpColor::red);
		}*/
}

void apSegMotionCol::computeColorHist(vpImage<vpRGBa> &_I)
{
	double ratio = (double)nbins/256;
	double eta = 0;
    std::vector<unsigned int *>  histfg;
    std::vector<unsigned int *>  histbg;
    histfg.resize(3);
    histbg.resize(3);

    /*histFg.resize(3);
    histBg.resize(3);*/

    for(int l =0; l<3; l++)
    {
    	histfg[l] = new unsigned int[nbins];
    	histbg[l] = new unsigned int[nbins];
    	memset(histfg[l], 0, nbins * sizeof(unsigned int));
    	memset(histbg[l], 0, nbins * sizeof(unsigned int));
    	histFgN[l].resize(nbins);
    	histBgN[l].resize(nbins);
    }


    std::vector<std::vector<int> > dataFg;
    dataFg.resize(3);

    std::vector<std::vector<int> > dataBg;
    dataBg.resize(3);
    /*nfg = 0;
    nbg = 0;*/

    vpRGBa col;

    for (unsigned int i=0; i < _I.getHeight(); i ++) {
    	for (unsigned int j=0; j < _I.getWidth(); j ++) {
    		col = _I[i][j];
    		if (label[i*_I.getWidth() + j] == 0)
    		{
    			histbg[0][ (int)((double)col.R*ratio) ] ++;
    			histbg[1][ (int)((double)col.G*ratio) ] ++;
    			histbg[2][ (int)((double)col.B*ratio) ] ++;
    			nbg ++;
    	}
    		else
    		{
    			histfg[0][ (int)((double)col.R*ratio) ] ++;
    			histfg[1][ (int)((double)col.G*ratio) ] ++;
    			histfg[2][ (int)((double)col.B*ratio) ] ++;
    			nfg ++;

    		}
    	}
    }


    for(int l =0; l<3; l++)
    {
     for (unsigned k=0; k < nbins ; k ++) {
     /*histFg[l].set(k,histfg[l][k]);
     histBg[l].set(k,histbg[l][k]);*/
	 histFg[l].set(k,histFg[l].get(k) + histfg[l][k]);
	 histBg[l].set(k,histBg[l].get(k) + histbg[l][k]);
     }
     histFg[l].smooth(3);
     histBg[l].smooth(3);

     for (int k = 0; k<nbins; k++)
     {
    	 //histFg[l].set(k,histFg[l].get(k));
    	 //histBg[l].set(k,histBg[l].get(k));
    	 histFgN[l][k] = (double)(histFg[l].get(k))/nfg;
    	 histBgN[l][k] = (double)(histBg[l].get(k))/nbg;
    	 //std::cout << " hist " << (double)histBgN[l][k] << std::endl;
     }

    }

    /*for (unsigned int i=0; i < _I.getHeight(); i ++) {
    	for (unsigned int j=0; j < _I.getWidth(); j ++) {
EdataHistF[j + i*_I.getWidth()] = eta*(log((double)(histFg[0].get((int)_I[i][j].R))/nfg) +
		log((double)(histFg[1].get((int)_I[i][j].G))/nfg) +
		log((double)(histFg[2].get((int)_I[i][j].B))/nfg));
EdataHistB[j + i*_I.getWidth()] = eta*(log((double)(histBg[0].get((int)_I[i][j].R))/nbg) +
		log((double)(histBg[1].get((int)_I[i][j].G))/nbg) +
		log((double)(histBg[2].get((int)_I[i][j].B))/nbg));
    	}
    	}*/


    /*std::vector<vpImage<unsigned char> > Is;
    Is.resize(3);
    for(int l =0; l<3; l++)
    Is[l].resize( _I.getHeight(), _I.getWidth());

	for (unsigned i=0; i < _I.getHeight(); i ++) {
    	for (unsigned j=0; j < _I.getWidth(); j ++) {
    Is[0][i][j] = _I[i][j].R;
    Is[1][i][j] = _I[i][j].G;
    Is[2][i][j] = _I[i][j].B;
    	}
    }

    for(int l =0; l<3; l++)
    {
     histFg[l].calculate(Is[l]);
     histBg[l].calculate(Is[l]);
     histFg[l].smooth(3);
     histBg[l].smooth(3);
     for (int k = 0; k<nbins; k++)
     {
    	 histFg[l].set(k,-eta*log(histFg[l].get(k)));
    	 histBg[l].set(k,-eta*log(histFg[l].get(k)));
    	 std::cout << " hist " << histFg[l].get(k) << std::endl;
     }
    }*/

    for(int l =0; l<3; l++)
    {
    	delete [] histfg[l];
    	delete [] histbg[l];
    }

}

void apSegMotionCol::computeColorHistLimb(vpImage<vpRGBa> &_I)
{
	double ratio = (double)nbins/256;
	double eta = 0;
    std::vector<unsigned int *>  histfg;
    std::vector<unsigned int *>  histbg;
    histfg.resize(3);
    histbg.resize(3);

    /*histFg.resize(3);
    histBg.resize(3);*/

    for(int l =0; l<3; l++)
    {
    	histfg[l] = new unsigned int[nbins];
    	histbg[l] = new unsigned int[nbins];
    	memset(histfg[l], 0, nbins * sizeof(unsigned int));
    	memset(histbg[l], 0, nbins * sizeof(unsigned int));
    	histFgN[l].resize(nbins);
    	histBgN[l].resize(nbins);
    }


    std::vector<std::vector<int> > dataFg;
    dataFg.resize(3);

    std::vector<std::vector<int> > dataBg;
    dataBg.resize(3);
    /*nfg = 0;
    nbg = 0;*/

    vpRGBa col;

    for (unsigned int i=0; i < _I.getHeight(); i ++) {
    	for (unsigned int j=0; j < _I.getWidth(); j ++) {
    		col = _I[i][j];
    		if (label[i*_I.getWidth() + j] == 0 || (col.R<20 && col.G<20 && col.B<20))
    		{
    			histbg[0][ (int)((double)col.R*ratio) ] ++;
    			histbg[1][ (int)((double)col.G*ratio) ] ++;
    			histbg[2][ (int)((double)col.B*ratio) ] ++;
    			nbg ++;
    	}
    		else
    		{
    			histfg[0][ (int)((double)col.R*ratio) ] ++;
    			histfg[1][ (int)((double)col.G*ratio) ] ++;
    			histfg[2][ (int)((double)col.B*ratio) ] ++;
    			nfg ++;

    		}
    	}
    }


    for(int l =0; l<3; l++)
    {
     for (unsigned k=0; k < nbins ; k ++) {
     /*histFg[l].set(k,histfg[l][k]);
     histBg[l].set(k,histbg[l][k]);*/
	 histFg[l].set(k,histFg[l].get(k) + histfg[l][k]);
	 histBg[l].set(k,histBg[l].get(k) + histbg[l][k]);
     }
     histFg[l].smooth(3);
     histBg[l].smooth(3);

     for (int k = 0; k<nbins; k++)
     {
    	 //histFg[l].set(k,histFg[l].get(k));
    	 //histBg[l].set(k,histBg[l].get(k));
    	 histFgN[l][k] = (double)(histFg[l].get(k))/nfg;
    	 histBgN[l][k] = (double)(histBg[l].get(k))/nbg;
    	 //std::cout << " hist " << (double)histFg[l].get(k) << std::endl;
     }

    }

    /*for (unsigned int i=0; i < _I.getHeight(); i ++) {
    	for (unsigned int j=0; j < _I.getWidth(); j ++) {
EdataHistF[j + i*_I.getWidth()] = eta*(log((double)(histFg[0].get((int)_I[i][j].R))/nfg) +
		log((double)(histFg[1].get((int)_I[i][j].G))/nfg) +
		log((double)(histFg[2].get((int)_I[i][j].B))/nfg));
EdataHistB[j + i*_I.getWidth()] = eta*(log((double)(histBg[0].get((int)_I[i][j].R))/nbg) +
		log((double)(histBg[1].get((int)_I[i][j].G))/nbg) +
		log((double)(histBg[2].get((int)_I[i][j].B))/nbg));
    	}
    	}*/


    /*std::vector<vpImage<unsigned char> > Is;
    Is.resize(3);
    for(int l =0; l<3; l++)
    Is[l].resize( _I.getHeight(), _I.getWidth());

	for (unsigned i=0; i < _I.getHeight(); i ++) {
    	for (unsigned j=0; j < _I.getWidth(); j ++) {
    Is[0][i][j] = _I[i][j].R;
    Is[1][i][j] = _I[i][j].G;
    Is[2][i][j] = _I[i][j].B;
    	}
    }

    for(int l =0; l<3; l++)
    {
     histFg[l].calculate(Is[l]);
     histBg[l].calculate(Is[l]);
     histFg[l].smooth(3);
     histBg[l].smooth(3);
     for (int k = 0; k<nbins; k++)
     {
    	 histFg[l].set(k,-eta*log(histFg[l].get(k)));
    	 histBg[l].set(k,-eta*log(histFg[l].get(k)));
    	 std::cout << " hist " << histFg[l].get(k) << std::endl;
     }
    }*/

    for(int l =0; l<3; l++)
    {
    	delete [] histfg[l];
    	delete [] histbg[l];
    }

}

void apSegMotionCol::computeHistKLTPoints(vpImage<vpRGBa> &_I)
{
	double ratio = (double)nbins/256;
	double eta = 0;
    std::vector<unsigned int *>  histfg;
    std::vector<unsigned int *>  histbg;
    histfg.resize(3);
    histbg.resize(3);

    histFg.resize(3);
    histBg.resize(3);

    /*nfg = 0;
    nbg = 0;*/


    for(int l =0; l<3; l++)
    {
    	histfg[l] = new unsigned int[nbins];
    	histbg[l] = new unsigned int[nbins];
    	memset(histfg[l], 0, nbins * sizeof(unsigned int));
    	memset(histbg[l], 0, nbins * sizeof(unsigned int));
    	histFgN[l].resize(nbins);
    	histBgN[l].resize(nbins);
    }

    std::vector<std::vector<int> > dataFg;
    dataFg.resize(3);

    std::vector<std::vector<int> > dataBg;
    dataBg.resize(3);
    int *data;

    nbg = 0;
    nfg = 0;

    nbg += colBgd.size();
    nfg += colFgd.size();
    for (unsigned int i=0; i < colBgd.size(); i ++) {
    	data = colBgd[i];
    			histbg[0][ (int)(data[0]*ratio) ] ++;
    			histbg[1][ (int)(data[1]*ratio) ] ++;
    			histbg[2][ (int)(data[2]*ratio) ] ++;
    			//nbg++;
    	}

    for (unsigned int i=0; i < colFgd.size(); i ++) {
    	data = colFgd[i];
    			histfg[0][ (int)(data[0]*ratio)] ++;
    			histfg[1][ (int)(data[1]*ratio)] ++;
    			histfg[2][ (int)(data[2]*ratio)] ++;
    			//nfg++;
    }

    for(int l =0; l<3; l++)
    {
     for (unsigned k = 0; k < nbins ; k ++) {
     histFg[l].set(k,histfg[l][k]);
     histBg[l].set(k,histbg[l][k]);
    	 /*histFg[l].set(k,histFg[l].get(k) + histfg[l][k]);
    	 histBg[l].set(k,histBg[l].get(k) + histbg[l][k]);*/
     }
     histFg[l].smooth(3);
     histBg[l].smooth(3);

     for (int k = 0; k<nbins; k++)
     {
    	 //histFg[l].set(k,histFg[l].get(k));
    	 //histBg[l].set(k,histBg[l].get(k));
    	 histFgN[l][k] = (double)(histFg[l].get(k))/nfg;
    	 histBgN[l][k] = (double)(histBg[l].get(k))/nbg;
    	 //std::cout << " hist " << (double)histBgN[l][k] << std::endl;
     }

     /*for (int k = 0; k<nbins; k++)
     {
    	 histFg[l].set(k,histFg[l].get(k));
    	 histBg[l].set(k,histBg[l].get(k));
    	 //histFg[l].set(k,-eta*log(1 - histFg[l].get(k))/nfg);
    	 //histBg[l].set(k,-eta*log(1 - histBg[l].get(k))/nfg);
    	 //std::cout << " hist " << (double)histFg[l].get(k) << std::endl;
     }*/

    }

    /*for (unsigned int i=0; i < _I.getHeight(); i ++) {
    	for (unsigned int j=0; j < _I.getWidth(); j ++) {
EdataHistF[j + i*_I.getWidth()] = eta*(log((double)(histFg[0].get((int)_I[i][j].R))/nfg) +
		log((double)(histFg[1].get((int)_I[i][j].G))/nfg) +
		log((double)(histFg[2].get((int)_I[i][j].B))/nfg));
EdataHistB[j + i*_I.getWidth()] = eta*(log((double)(histBg[0].get((int)_I[i][j].R))/nbg) +
		log((double)(histBg[1].get((int)_I[i][j].G))/nbg) +
		log((double)(histBg[2].get((int)_I[i][j].B))/nbg));
    	}
    	}*/


    /*std::vector<vpImage<unsigned char> > Is;
    Is.resize(3);
    for(int l =0; l<3; l++)
    Is[l].resize( _I.getHeight(), _I.getWidth());

	for (unsigned i=0; i < _I.getHeight(); i ++) {
    	for (unsigned j=0; j < _I.getWidth(); j ++) {
    Is[0][i][j] = _I[i][j].R;
    Is[1][i][j] = _I[i][j].G;
    Is[2][i][j] = _I[i][j].B;
    	}
    }

    for(int l =0; l<3; l++)
    {
     histFg[l].calculate(Is[l]);
     histBg[l].calculate(Is[l]);
     histFg[l].smooth(3);
     histBg[l].smooth(3);
     for (int k = 0; k<nbins; k++)
     {
    	 histFg[l].set(k,-eta*log(histFg[l].get(k)));
    	 histBg[l].set(k,-eta*log(histFg[l].get(k)));
    	 std::cout << " hist " << histFg[l].get(k) << std::endl;
     }
    }*/

    for(int l =0; l<3; l++)
    {
    	delete [] histfg[l];
    	delete [] histbg[l];
    }

}

void apSegMotionCol::getIseg(vpImage<unsigned char> &_I, vpImage<unsigned char> &I_,vpImage<vpRGBa> &Icol_)
{

	vpImagePoint p1;
    I_ = _I;
	for (int i = 0; i<_I.getHeight(); i++)
		for (int j =0 ; j < _I.getWidth() ; j++)
		{
			p1.set_i(i);
			p1.set_j(j);
			if(i>2 && i<_I.getHeight()-3 && j<_I.getWidth()-3 && j>2)
			{if(label[i*_I.getWidth() + j] == 0)
			{
				I_[i][j] = 0;
				I_[i+1][j] = 0;
				I_[i+2][j] = 0;
				I_[i+3][j] = 0;
				I_[i][j+1] = 0;
				I_[i][j+2] = 0;
				I_[i][j+3] = 0;
				I_[i-1][j] = 0;
				I_[i-2][j] = 0;
				I_[i-3][j] = 0;
				I_[i][j-1] = 0;
				I_[i][j-2] = 0;
				I_[i][j-3] = 0;

				Icol_[i][j] = 0;
				Icol_[i+1][j] = 0;
				Icol_[i+2][j] = 0;
				Icol_[i+3][j] = 0;
				Icol_[i][j+1] = 0;
				Icol_[i][j+2] = 0;
				Icol_[i][j+3] = 0;
				Icol_[i-1][j] = 0;
				Icol_[i-2][j] = 0;
				Icol_[i-3][j] = 0;
				Icol_[i][j-1] = 0;
				Icol_[i][j-2] = 0;
				Icol_[i][j-3] = 0;

			}
			}
			else {
				I_[i][j] = 0;
				Icol_[i][j] = 0;
			}
		}

}

void apSegMotionCol::rgb2yuv(vpImage<vpRGBa> &_I,vpImage<vpRGBa> &I_)
{
 I_.resize(_I.getHeight(), _I.getWidth());
	for (unsigned i=0; i < _I.getHeight(); i ++) {
    	for (unsigned j=0; j < _I.getWidth(); j ++) {
    I_[i][j].R = 0.299*_I[i][j].R + 0.587*_I[i][j].G + 0.114*_I[i][j].B;
    I_[i][j].G = 0.492*(_I[i][j].B - I_[i][j].R);
    I_[i][j].B = 0.877*(_I[i][j].R - I_[i][j].R);
    	}
    }
}

void edgeOrientMap(vpImage<unsigned char> &I0)
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
    I0[n][m] =   (int)imG[n][m];
  }
}

double cannyTh1=4;
double cannyTh2=8;
IplImage* Ip = NULL;
Mat dst0;
vpImageConvert::convert(I0, Ip);
Mat src0=Mat(Ip,true);
IplImage* dst = cvCreateImage( cvSize(I0.getWidth(),I0.getHeight()), 8, 1);
cvCanny( Ip, dst, cannyTh1, cannyTh2, 3 );
vpImageConvert::convert(dst, I1);
double a,b;
for (int i=0;i<height-0;i++)
		{
			for (int j=0;j<width-0;j++)
			{
				if(i>3 && i < height-3 && j>3 && j < width-3){
				a=(apImageFilter::sobelFilterX(I0,i,j));
				b=(apImageFilter::sobelFilterY(I0,i,j));
			    if ( I1[i][j]==255) //
				{

				 I0[i][j]=255*(-(atan(a/b))/M_PI+1/2);

				}
				 else {I0[i][j]=100;}
				}
				else{
					I0[i][j]=100;
				}
			}
	}


}


void
apSegMotionCol::setParameterSpace(int n, int m, vpImage<unsigned char> &I)
{
//V.init(n,m);
//Theta.resize(n);
cU.resize(n);
cV.resize(n);
Rho.resize(m);
int h,w;
double r,ct;
//h=I.getHeight();
//w=I.getWidth();
//r=sqrt((double)h*h/(cam.get_py()*cam.get_py())+(double)w*w/(cam.get_px()*cam.get_px()));
ct = 5000;
r = (int)sqrt(2)*ct/2.0;
//Rho.resize(r);
//LineVote.init(n,m);
//Rho[0]=0;
int i,j;
for (i=0;i<n;i++)
{
//cU[i]=i*M_PI/n-M_PI/2;
cU[i] = -ct + 2*i*ct/n;
cV[i] = -ct + 2*i*ct/n;
}
for (j=0;j<m;j++)
{
Rho[j]=j*2000/m + 1000;
}
}

void apSegMotionCol::detectLimbC(vpImage<unsigned char> &I, vpImage<vpRGBa> &_I)
{
	vpImage<unsigned char> Igs0,Igs, I_;
	I_.resize(_I.getHeight(), _I.getWidth());
	Igs0.resize(_I.getHeight(), _I.getWidth());
	vpImageConvert::convert(_I,Igs);
	Igs0 = Igs;
	edgeOrientMap(Igs);

	std::vector<triple> triples;

	for (unsigned i=0; i < _I.getHeight(); i ++) {
    	for (unsigned j=0; j < _I.getWidth(); j ++) {
    		I_[i][j] = 0;
    	}
    	}
	for (unsigned i=30; i < _I.getHeight()-30; i ++) {
    	for (unsigned j=30; j < _I.getWidth()-30; j ++) {
    		if(Igs[i][j]!=100)
    		{
    		  double delta = 3.1416 * (double) ((double) Igs[i][j] / 255 - 0.5);
    		  delta = M_PI/2;
    		  vpPointSite s;
    		  vpImagePoint pbg;
    		  int k=0;
    		  int k1=0;
    		  vpPointSite  *list_query_pixels;
    		  vpPointSite sq;
    		  int n0;
    		  s.init((double)i, (double)j, delta, 0, 1);
    		  list_query_pixels = s.getQueryList(Igs0, 25);
    		  for (int n=0; n<51; n++)
    		  {
    			  sq = list_query_pixels[n];

    			  if(_I[sq.i][sq.j].R < 30 && _I[sq.i][sq.j].G < 30 && _I[sq.i][sq.j].B < 30){
    				 if(n > 4)
    				 {
    	              //n0 = n;
    					 k1++;
    				 }
    				 k++;
    			  }
    		  }
    		  if(k>3)
    		  {
    			  I_[i][j] = 255;
    				 //if(n0>4)
    			  //std::cout << " k1 " << k1 << std::endl;

    		  }
    		  delete [] list_query_pixels;
    		}
    	}
    	}

	                setParameterSpace(2000,200,I);

	                circleVote.init(2000,2000,200);
                    double theta;
                    int u,v, indu, indv;

	                for (unsigned i=0; i < _I.getHeight(); i ++) {
	                    	for (unsigned j=0; j < _I.getWidth(); j ++) {

	                        if (I_[i][j] != 0)
	                        {
	                        	//std::cout << " ok " << std::endl;
	        	    //vpPixelMeterConversion::convertPoint(*cam,j,i,xx,yy);
	                        	for (int iT = 0; iT<90; iT++)
	                        		for (int iR = 0; iR < 200; iR++)
	                        		{
	                        		    //theta = M_PI*iT/180.0;
	                        			u = j - Rho[iR]*cos(M_PI*iT/180.0);
	                        			v = i - Rho[iR]*sin(M_PI*iT/180.0);
	                //rho = xx*cos(Theta[indTheta])+yy*sin(Theta[indTheta]) - Rho[0];
	                indu = floor((u-cU[0])/(-2*cU[0]/2000));
	                indv = floor((v-cV[0])/(-2*cV[0]/2000));
	                circleVote.VoteCenter[indu][indv]=circleVote.VoteCenter[indu][indv]+1;
	                circleVote.VoteRad[iR] = circleVote.VoteRad[iR]+ 1;
	                        		}
	                        }

	                    	}
	                }



	                triples = circleVote.getBestVotes(1);
	                std::cout << " triples " << triples[0].i1 << " " << triples[0].j1 << " " << triples[0].k1 << std::endl;


	                IplImage* Ip = NULL;
					IplImage* Ip1;
					//vpImageConvert::convert(I_, Ip);
					vpImageConvert::convert(I, Ip);
					Mat color_dst;
					vector<Vec2f> linesV;
					Mat dst(Ip);
					vpImage<unsigned char> I3;
					cvtColor(dst, color_dst, CV_GRAY2BGR);
/*					HoughLines(dst, linesV, 3, CV_PI / 60, 150, 0, 0);
					  for( size_t i = 0; i < linesV.size(); i++ )
					  {
					     float rho = linesV[i][0], theta = linesV[i][1];
					     Point pt1, pt2;
					     double a = cos(theta), b = sin(theta);
					     double x0 = a*rho, y0 = b*rho;
					     pt1.x = cvRound(x0 + 1000*(-b));
					     pt1.y = cvRound(y0 + 1000*(a));
					     pt2.x = cvRound(x0 - 1000*(-b));
					     pt2.y = cvRound(y0 - 1000*(a));
					     line( color_dst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
						 limb.setRho(rho);
						 limb.setTheta(theta);
					  }*/

					 vector<Vec3f> circles;

					   /// Apply the Hough Transform to find the circles
					   HoughCircles( dst, circles, CV_HOUGH_GRADIENT, 1, dst.rows/8, 10, 20, 1000, 4000 );

					   /// Draw the circles detected
					   for( size_t i = 0; i < circles.size(); i++ )
					   {
					       Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
					       int radius = cvRound(circles[i][2]);
					       // circle center
					       circle( color_dst, center, 3, Scalar(0,255,0), -1, 8, 0 );
					       // circle outline
					       circle( color_dst, center, radius, Scalar(0,0,255), 3, 8, 0 );
					    }

						vpImage<vpRGBa> Ioverlay;
						 Ip1 = new IplImage(color_dst);
						 vpImageConvert::convert(Ip1, I3);
						 delete Ip1;
						vpDisplayX display;
						 display.init(I3, 1500, 10, "Hough");
						 vpDisplay::display(I3);
						 vpDisplay::flush(I3);
						 vpDisplay::getImage(I3,Ioverlay);
						 std::cout << " ok " << std::endl;
						 vpImageIo::writePNG(Ioverlay, "ihough0.png");

						 vpImagePoint p0,p1;
						 p1 = p0;
						 int imin,imax, jmin, jmax;
						 imax = 0;
						 imin = _I.getHeight();
						 jmin = _I.getWidth();
						 jmax = 0;
						 limbo1.setRho(0);
						 limbo2.setRho(0);

						 for(int i = 25; i<_I.getHeight()-25; i++)
						 {
							 int j = (int)(limb.getRho()-i*sin(limb.getTheta()))/cos(limb.getTheta());
							 if(j>25 && j<_I.getWidth()-25)
							 {
					    		  double delta = limb.getTheta();
					    		  vpPointSite s;
					    		  vpImagePoint pbg;
					    		  int k=0;
					    		  int k1=0;
					    		  vpPointSite  *list_query_pixels;
					    		  vpPointSite sq;
					    		  int n0;
					    		  s.init((double)i, (double)j, delta, 0, 1);
					    		  list_query_pixels = s.getQueryList(Igs0, 25);
					    		  for (int n=0; n<51; n++)
					    		  {
					    			  sq = list_query_pixels[n];

					    			  if(_I[sq.i][sq.j].R < 30 && _I[sq.i][sq.j].G < 30 && _I[sq.i][sq.j].B < 30){
					    				 if(n > 4)
					    				 {
					    					 k1++;
					    				 }
					    				 k++;
					    			  }
					    		  }
					    		  if(k<3)
					    		  {
					    			  if(i<imin)
					    			  {
					    			imin = i;
					    			jmin = j;
					    			  }
					    			  if(i>imax)
					    			  {
					    				  imax = i;
					    				  jmax = j;
					    			  }
					    		  }
					    		  delete [] list_query_pixels;
							 }
						 }
						 if(imin<_I.getHeight())
						 {
							 double thetao1 = limb.getTheta() + M_PI/2;
							 double rhoo1 = jmin*cos(thetao1) + imin*sin(thetao1);
							 limbo1.setRho(rhoo1);
							 limbo1.setTheta(thetao1);
						 }
						 if(imax<_I.getHeight()-1)
						 {
							 double thetao2 = limb.getTheta() + M_PI/2;
							 double rhoo2 = jmax*cos(thetao2) + imax*sin(thetao2);
							 limbo2.setRho(rhoo2);
							 limbo2.setTheta(thetao2);
						 }
						 vpLine limTemp;
						 if(limbo2.getRho()<limbo1.getRho())
						 {
							 limTemp = limbo2;
							 limbo2 = limbo1;
							 limbo1 = limTemp;
						 }
						/*std::cout << " imax " << imin << " jmax " << jmin << std::endl;
						 vpDisplay::displayLine(I,imin,jmin,imax,jmax,vpColor::blue,2);
						 vpDisplay::displayCross(I,imin,jmin,5,vpColor::white,4);
						 vpDisplay::displayCross(I,imax,jmax,5,vpColor::yellow,4);*/


					 //getchar();
	vpImageIo::writePNG(Igs, "Iseg4.png");
	vpImageIo::writePNG(I_, "Iseg5.png");
}


void apSegMotionCol::detectLimb(vpImage<unsigned char> &I, vpImage<vpRGBa> &_I)
{
	vpImage<unsigned char> Igs0,Igs, I_;
	I_.resize(_I.getHeight(), _I.getWidth());
	Igs0.resize(_I.getHeight(), _I.getWidth());
	vpImageConvert::convert(_I,Igs);
	Igs0 = Igs;
	edgeOrientMap(Igs);
	for (unsigned i=0; i < _I.getHeight(); i ++) {
    	for (unsigned j=0; j < _I.getWidth(); j ++) {
    		I_[i][j] = 0;
    	}
    	}
	for (unsigned i=30; i < _I.getHeight()-30; i ++) {
    	for (unsigned j=30; j < _I.getWidth()-30; j ++) {
    		if(Igs[i][j]!=100)
    		{
    		  double delta = 3.1416 * (double) ((double) Igs[i][j] / 255 - 0.5);
    		  delta = M_PI/2;
    		  vpPointSite s;
    		  vpImagePoint pbg;
    		  int k=0;
    		  int k1=0;
    		  vpPointSite  *list_query_pixels;
    		  vpPointSite sq;
    		  int n0;
    		  s.init((double)i, (double)j, delta, 0, 1);
    		  list_query_pixels = s.getQueryList(Igs0, 25);
    		  for (int n=0; n<51; n++)
    		  {
    			  sq = list_query_pixels[n];

    			  if(_I[sq.i][sq.j].R < 30 && _I[sq.i][sq.j].G < 30 && _I[sq.i][sq.j].B < 30){
    				 if(n > 4)
    				 {
    	              //n0 = n;
    					 k1++;
    				 }
    				 k++;
    			  }
    		  }
    		  if(k>3)
    		  {
    			  I_[i][j] = 255;
    				 //if(n0>4)
    			  //std::cout << " k1 " << k1 << std::endl;

    		  }
    		  delete [] list_query_pixels;
    		}
    	}
    	}

	                IplImage* Ip = NULL;
					IplImage* Ip1;
					vpImageConvert::convert(I_, Ip);
					//vpImageConvert::convert(I, Ip);
					Mat color_dst;
					vector<Vec2f> linesV;
					Mat dst(Ip);
					vpImage<unsigned char> I3;
					cvtColor(dst, color_dst, CV_GRAY2BGR);
					HoughLines(dst, linesV, 3, CV_PI / 60, 150, 0, 0);
					  for( size_t i = 0; i < linesV.size(); i++ )
					  {
					     float rho = linesV[i][0], theta = linesV[i][1];
					     Point pt1, pt2;
					     double a = cos(theta), b = sin(theta);
					     double x0 = a*rho, y0 = b*rho;
					     pt1.x = cvRound(x0 + 1000*(-b));
					     pt1.y = cvRound(y0 + 1000*(a));
					     pt2.x = cvRound(x0 - 1000*(-b));
					     pt2.y = cvRound(y0 - 1000*(a));
					     line( color_dst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
						 limb.setRho(rho);
						 limb.setTheta(theta);
					  }

					 /*vector<Vec3f> circles;

					   /// Apply the Hough Transform to find the circles
					   HoughCircles( dst, circles, CV_HOUGH_GRADIENT, 1, dst.rows/8, 10, 20, 1000, 4000 );

					   /// Draw the circles detected
					   for( size_t i = 0; i < circles.size(); i++ )
					   {
					       Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
					       int radius = cvRound(circles[i][2]);
					       // circle center
					       circle( color_dst, center, 3, Scalar(0,255,0), -1, 8, 0 );
					       // circle outline
					       circle( color_dst, center, radius, Scalar(0,0,255), 3, 8, 0 );
					    }*/

						vpImage<vpRGBa> Ioverlay;
						 Ip1 = new IplImage(color_dst);
						 vpImageConvert::convert(Ip1, I3);
						 delete Ip1;
						vpDisplayX display;
						 display.init(I3, 1500, 10, "Hough");
						 vpDisplay::display(I3);
						 vpDisplay::flush(I3);
						 vpDisplay::getImage(I3,Ioverlay);
						 std::cout << " ok " << std::endl;
						 vpImageIo::writePNG(Ioverlay, "ihough0.png");

						 vpImagePoint p0,p1;
						 p1 = p0;
						 int imin,imax, jmin, jmax;
						 imax = 0;
						 imin = _I.getHeight();
						 jmin = _I.getWidth();
						 jmax = 0;
						 limbo1.setRho(0);
						 limbo2.setRho(0);

						 for(int i = 25; i<_I.getHeight()-25; i++)
						 {
							 int j = (int)(limb.getRho()-i*sin(limb.getTheta()))/cos(limb.getTheta());
							 if(j>25 && j<_I.getWidth()-25)
							 {
					    		  double delta = limb.getTheta();
					    		  vpPointSite s;
					    		  vpImagePoint pbg;
					    		  int k=0;
					    		  int k1=0;
					    		  vpPointSite  *list_query_pixels;
					    		  vpPointSite sq;
					    		  int n0;
					    		  s.init((double)i, (double)j, delta, 0, 1);
					    		  list_query_pixels = s.getQueryList(Igs0, 25);
					    		  for (int n=0; n<51; n++)
					    		  {
					    			  sq = list_query_pixels[n];

					    			  if(_I[sq.i][sq.j].R < 30 && _I[sq.i][sq.j].G < 30 && _I[sq.i][sq.j].B < 30){
					    				 if(n > 4)
					    				 {
					    					 k1++;
					    				 }
					    				 k++;
					    			  }
					    		  }
					    		  if(k<3)
					    		  {
					    			  if(i<imin)
					    			  {
					    			imin = i;
					    			jmin = j;
					    			  }
					    			  if(i>imax)
					    			  {
					    				  imax = i;
					    				  jmax = j;
					    			  }
					    		  }
					    		  delete [] list_query_pixels;
							 }
						 }
						 if(imin<_I.getHeight())
						 {
							 double thetao1 = limb.getTheta() + M_PI/2;
							 double rhoo1 = jmin*cos(thetao1) + imin*sin(thetao1);
							 limbo1.setRho(rhoo1);
							 limbo1.setTheta(thetao1);
						 }
						 if(imax<_I.getHeight()-1)
						 {
							 double thetao2 = limb.getTheta() + M_PI/2;
							 double rhoo2 = jmax*cos(thetao2) + imax*sin(thetao2);
							 limbo2.setRho(rhoo2);
							 limbo2.setTheta(thetao2);
						 }
						 vpLine limTemp;
						 if(limbo2.getRho()<limbo1.getRho())
						 {
							 limTemp = limbo2;
							 limbo2 = limbo1;
							 limbo1 = limTemp;
						 }
						/*std::cout << " imax " << imin << " jmax " << jmin << std::endl;
						 vpDisplay::displayLine(I,imin,jmin,imax,jmax,vpColor::blue,2);
						 vpDisplay::displayCross(I,imin,jmin,5,vpColor::white,4);
						 vpDisplay::displayCross(I,imax,jmax,5,vpColor::yellow,4);*/


					 //getchar();
	vpImageIo::writePNG(Igs, "Iseg4.png");
	vpImageIo::writePNG(I_, "Iseg5.png");
}



