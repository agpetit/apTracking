#include "tracker.h"
#include <visp/vpImage.h>
#include <visp/vpDisplayX.h>



SegmentMotion::SegmentMotion(){

	nb_trajectory = (int*)malloc (NBPOINTSTOTRACK * sizeof(int));
	ID_trajectory = (int*)malloc (NBPOINTSTOTRACK * sizeof(int));
	trajectory  = (CvPoint2D32f**) malloc (NBPOINTSTOTRACK * sizeof(CvPoint2D32f*));
	for (int i = 0; i < NBPOINTSTOTRACK; i++)
		trajectory[i]= (CvPoint2D32f*) malloc (1000 * sizeof(CvPoint2D32f));
	trajectory_.resize(NBPOINTSTOTRACK);
	for (int i = 0; i < NBPOINTSTOTRACK; i++)
			trajectory_[i].resize(1000);
	colFgd.resize(0);
	colBgd.resize(0);
	bi = 2;
	bj = 2;

}



SegmentMotion::SegmentMotion(int nbPoints){
	init(nbPoints);
}

void SegmentMotion::init(int nbPoints){

	tracker.setTrackerId(1);				//1
	tracker.setMaxFeatures(nbPoints);		//nbPoint
	tracker.setWindowSize(6);				//10
	tracker.setQuality(0.000001);				//0.01
	tracker.setMinDistance(10);				//15
	tracker.setHarrisFreeParameter(0.1);	//0.04
	tracker.setBlockSize(9);				//9
	tracker.setUseHarris(1);				//1
	tracker.setPyramidLevels(1);			//3

}

void SegmentMotion::initTrajectories(IplImage *Img){

	tracker.initTracking(Img);
	nbPointToTrack = 0;
	for (int i = 0; i< tracker.getNbFeatures(); i++)
	{
		float x,y;int id;
		tracker.getFeature(i, id, x, y);
		ID_trajectory[nbPointToTrack] = id;
		nb_trajectory[nbPointToTrack] = 1;
		trajectory [nbPointToTrack][0].x = x;
		trajectory [nbPointToTrack][0].y = y;
		nbPointToTrack++;
	}
}

void SegmentMotion::initEnergy(vpImage<unsigned char> &_I)
{
	resolution = _I.getHeight()*_I.getWidth();
	EdataF = new double[resolution];
	EdataB = new double[resolution];
	EdataHistF = new double[resolution];
	EdataHistB = new double[resolution];
	EsmoothH = new double[resolution];
	EsmoothV = new double[resolution];
	label = new int[resolution];
    minEnergy.init(resolution);
    nfg = 1;
    nbg =1;

	histFg.resize(3);
	histBg.resize(3);
}

void SegmentMotion::updateTrajectories(){

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
					trajectory[j][k] = trajectory[j+1][k];
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
			trajectory [i][nb_trajectory[i]].x = x;
			trajectory [i][nb_trajectory[i]].y = y;
			nb_trajectory[i]++;
		}
		else 
			printf("ID correspondant pas a la trajectoire\n");
	}


}

void SegmentMotion::fusionTrajectories(SegmentMotion tracker_tmp){

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
			trajectory [nbPointToTrack][0].x = x1;
			trajectory [nbPointToTrack][0].y = y1;
			nb_trajectory[nbPointToTrack] = 1;
			nbPointToTrack++;
		}
	}

}


void SegmentMotion::displayTrajectories( vpImage<unsigned char> &I){
		

	for (int   j = 0 ; j < nbPointToTrack ; j++)
	{
		int   i;
		int min_born =  (nb_trajectory[j] - HISTORYOFPOSITION > 0)? (nb_trajectory[j]-HISTORYOFPOSITION): (0);

		int idx_j = j;
		int idx_i = 0;
		int nb_idx = 0;
		for (i = nb_trajectory[j]-1  ; i > min_born ; i--)
		{
			nb_idx++;
			int x2 = trajectory[j][i].x;
			int y2 = trajectory[j][i].y;
			int x1 = trajectory[j][i-1].x;
			int y1 = trajectory[j][i-1].y; 
			vpImagePoint p2 (y1, x1);
			vpImagePoint p1 (y2, x2);

			//switch (labelMotion[j])
			//{
			//case 0 :vpDisplay::displayLine(I,p2,p1,vpColor::red, 1); break;
			//case 1	:vpDisplay::displayLine(I,p2,p1,vpColor::red, 1); break;
			/*default :*/
			vpDisplay::displayLine(I,p2,p1,vpColor::red, 2);//break;
			//}

		}
		vpImagePoint ip (trajectory[j][nb_trajectory[j]-1].y, trajectory[j][nb_trajectory[j]-1].x);
		//switch (labelMotion[j])
		//{
		//case 0 :vpDisplay::displayCross(I, ip, 5, vpColor::red);   break;
		//case 1	:vpDisplay::displayCross(I, ip, 5, vpColor::red); break;
		/*default :*/
		vpDisplay::displayCross(I, ip, 5, vpColor::red);  //break;
		//}
	}
	vpDisplay::flush(I);


}

void SegmentMotion::labelPoints(vpImage<unsigned char> &I,vpImage<vpRGBa> &Icol)//std::vector<vpImagePoint> points[2])
{
for (int i = 0; i < nbPointToTrack; i++)
	for (int j = 0; j < 1000; j++)
		trajectory_[i][j] = trajectory[i][j];

int labelMotion[600];
RANSAC ransac;
ransac.RANSACFunction(trajectory_,nb_trajectory,nbPointToTrack,0,labelMotion);
int nbData = 5;
vpColVector data(nbData);
colBgd.resize(0);
colFgd.resize(0);
for (int i = 0; i<600; i++)
{
	//std::cout << " ok1 " << i << std::endl;
	//std::cout << " i " << labelMotion[i] << std::endl;
	int x1 = trajectory_[i][nb_trajectory[i]-1].x;
	int y1 = trajectory_[i][nb_trajectory[i]-1].y;
	if( x1<I.getWidth() && y1 < I.getHeight() )
	{
	//std::cout << " ok2 " << x1 << " " << y1 << std::endl;
	vpImagePoint p1 (y1, x1);
	if(labelMotion[i]==0){
		vpDisplay::displayCross(I, p1, 5, vpColor::blue);
		data[0] = Icol[y1][x1].R;
		data[1] = Icol[y1][x1].G;
		data[2] = Icol[y1][x1].B;
		data[3] = y1;
		data[4] = x1;
		/*colP.col = Icol[y1][x1];
		colP.pt = p1;*/
		colBgd.push_back(data);
	}
	else //if (labelMotion[i]==-1)
	{
		vpDisplay::displayCross(I, p1, 5, vpColor::green);
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

vpRGBa SegmentMotion::meanTemplate(vpImage<vpRGBa> &Icol,int x,int y)
{
	vpRGBa mean;
	mean.R = 0;
	mean.G = 0;
	mean.B = 0;
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

void SegmentMotion::labelMeanPoints(vpImage<unsigned char> &I,vpImage<vpRGBa> &Icol)
{
for (int i = 0; i < nbPointToTrack; i++)
	for (int j = 0; j < 1000; j++)
		trajectory_[i][j] = trajectory[i][j];

int labelMotion[600];
RANSAC ransac;
ransac.RANSACFunction(trajectory_,nb_trajectory,nbPointToTrack,0,labelMotion);
int nbData = 5;
vpColVector data(nbData);
colBgd.resize(0);
colFgd.resize(0);
vpRGBa mean;
for (int i = 0; i<600; i++)
{
	//std::cout << " ok1 " << i << std::endl;
	//std::cout << " i " << labelMotion[i] << std::endl;
	int x1 = trajectory_[i][nb_trajectory[i]-1].x;
	int y1 = trajectory_[i][nb_trajectory[i]-1].y;
	if( x1<I.getWidth() && y1 < I.getHeight() )
	{
	//std::cout << " ok2 " << x1 << " " << y1 << std::endl;
	vpImagePoint p1 (y1, x1);
	if(labelMotion[i]==0){
		vpDisplay::displayCross(I, p1, 5, vpColor::blue);
		mean = meanTemplate(Icol,x1,y1);
		data[0] = mean.R;
		data[1] = mean.G;
		data[2] = mean.B;
		data[3] = y1;
		data[4] = x1;
		/*colP.col = Icol[y1][x1];
		colP.pt = p1;*/
		colBgd.push_back(data);
	}
	else //if (labelMotion[i]==-1)
	{
		vpDisplay::displayCross(I, p1, 5, vpColor::green);
		mean = meanTemplate(Icol,x1,y1);
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

void SegmentMotion::labelSelectPoints(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol)
{
	for (int i = 0; i < nbPointToTrack; i++)
		for (int j = 0; j < 1000; j++)
			trajectory_[i][j] = trajectory[i][j];

	int labelMotion[600];
	RANSAC ransac;
	ransac.RANSACFunction(trajectory_,nb_trajectory,nbPointToTrack,0,labelMotion);
	int nbData = 5;
	vpColVector data(nbData);
	vpRGBa mean;
	colBgd.resize(0);
	colFgd.resize(0);
	int cgrid = 12;
	int rgrid = 12;
	int cgridf = 48;
	int rgridf = 48;
	int imin,xminb,yminb,xminf,yminf;
	double distmin =500;
	double dst;
	for(int ck = 0; ck<cgrid ; ck++)
		for(int vk = 0; vk<rgrid ; vk++)
		{
			distmin =50000;
	for (int i = 0; i<600; i++)
	{
		//std::cout << " ok1 " << i << std::endl;
		//std::cout << " i " << labelMotion[i] << std::endl;
		int x1 = trajectory_[i][nb_trajectory[i]-1].x;
		int y1 = trajectory_[i][nb_trajectory[i]-1].y;
		if( x1<I.getWidth() && y1 < I.getHeight() )
		{
		//std::cout << " ok2 " << x1 << " " << y1 << std::endl;
			if(labelMotion[i]==0){
		dst = (x1-ck*(I.getWidth()/cgrid)-I.getWidth()/(2*cgrid))*(x1-ck*(I.getWidth()/cgrid)-I.getWidth()/(2*cgrid)) + (y1-vk*(I.getWidth()/cgrid)-I.getWidth()/(2*cgrid))*(y1-vk*(I.getWidth()/cgrid)-I.getWidth()/(2*cgrid));
			if(dst < distmin)
			{
				xminb = x1;
				yminb = y1;
				distmin=dst;
			}
		}
		}
	}
	//if(distmin < sqrt(2)*I.getWidth()/8)
	{
	vpImagePoint p1 (yminb, xminb);
	vpDisplay::displayCross(I, p1, 5, vpColor::blue,5);
	/*p1.set_j(ck*(I.getWidth()/6)-(I.getWidth()/12));
	p1.set_j(ck*(I.getWidth()/6)-(I.getWidth()/12));*/
	/*mean = meanTemplate(Icol,xminb,yminb);
	data[0] = mean.R;
	data[1] = mean.G;
	data[2] = mean.B;*/
	data[0] = Icol[yminb][xminb].R;
	data[1] = Icol[yminb][xminb].G;
	data[2] = Icol[yminb][xminb].B;
	data[3] = yminb;
	data[4] = xminb;
	/*colP.col = Icol[y1][x1];
	colP.pt = p1;*/
	colBgd.push_back(data);
	}

		}

	//for(int ck = 0; ck<cgridf ; ck++)
		//for(int vk = 0; vk<rgridf ; vk++)
		{
			distmin =500000;
	for (int i = 0; i<600; i++)
	{
		//std::cout << " ok1 " << i << std::endl;
		//std::cout << " i " << labelMotion[i] << std::endl;
		int x1 = trajectory_[i][nb_trajectory[i]-1].x;
		int y1 = trajectory_[i][nb_trajectory[i]-1].y;
		if( x1<I.getWidth() && y1 < I.getHeight() )
		{
		//std::cout << " ok2 " << x1 << " " << y1 << std::endl;
		vpImagePoint p1 (y1, x1);

		if(labelMotion[i]!=0){
		vpDisplay::displayCross(I, p1, 5, vpColor::green,5);
		data[0] = Icol[y1][x1].R;
		data[1] = Icol[y1][x1].G;
		data[2] = Icol[y1][x1].B;
		data[3] = y1;
		data[4] = x1;
		colFgd.push_back(data);
		}

		/*if (labelMotion[i]!=0)
		{
		dst = (x1-ck*(I.getWidth()/cgridf)-I.getWidth()/(2*cgridf))*(x1-ck*(I.getWidth()/cgridf)-I.getWidth()/(2*cgridf)) + (y1-vk*(I.getWidth()/cgridf)-I.getWidth()/(2*cgridf))*(y1-vk*(I.getWidth()/cgridf)-I.getWidth()/(2*cgridf));

			if(dst < distmin)
			{
				xminb = x1;
				yminb = y1;
				distmin=dst;
			}
		}*/
		}
	}
	/*if(distmin < sqrt(2)*I.getWidth()/cgridf)
	{
	vpImagePoint p2 (yminb, xminb);
	vpDisplay::displayCross(I, p2, 5, vpColor::green);
	data[0] = Icol[yminb][xminb].R;
	data[1] = Icol[yminb][xminb].G;
	data[2] = Icol[yminb][xminb].B;
	data[3] = yminb;
	data[4] = xminb;
	colFgd.push_back(data);
	}*/
}

	std::cout << " size " << colBgd.size() << " "<< colFgd.size() << std::endl;
}

void SegmentMotion::GMM()
{
    int i, j;
    int nsamplesBgd = colBgd.size();
    int nsamplesFgd = colFgd.size();
    //CvRNG rng_state = cvRNG(-1);
    CvMat* samplesFgd = cvCreateMat( nsamplesFgd, 3, CV_32FC1 );
    CvMat* labelsBgd = cvCreateMat( nsamplesBgd, 1, CV_32SC1 );
    CvMat* samplesBgd = cvCreateMat( nsamplesBgd, 3, CV_32FC1 );
    CvMat* labelsFgd = cvCreateMat( nsamplesFgd, 1, CV_32SC1 );
    CvEMParams params;
    int step;
    float *data;
    float *data1;
    vpColVector _data;

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
    params.nclusters = 10;
    params.cov_mat_type       = CvEM::COV_MAT_SPHERICAL;
    params.start_step         = CvEM::START_AUTO_STEP;
    params.term_crit.max_iter = 10;
    params.term_crit.epsilon  = 0.1;
    params.term_crit.type     = CV_TERMCRIT_ITER|CV_TERMCRIT_EPS;

    for(int i = 0; i<nsamplesBgd;i++)
    {
    step = samplesBgd->step/sizeof(float);
    _data = colBgd[i];
    data = samplesBgd->data.fl;
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

    em_modelFgd.train( samplesFgd, 0, params, labelsFgd );
    em_modelBgd.train( samplesBgd, 0, params, labelsBgd );
}


void SegmentMotion::computeLikelihoodGMM(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol)
{
	CvMat* sample = cvCreateMat( 1, 3, CV_32FC1);
	CvMat* probsFgd = cvCreateMat( 1, 10, CV_64FC1 );
	CvMat* probsBgd = cvCreateMat( 1, 10, CV_64FC1 );
	double probaFgd, probaBgd;
    for( int i = 0; i < Icol.getHeight(); i++ )
    {
        for( int j = 0; j < Icol.getWidth(); j++ )
        {
        	probaFgd = 0;
        	probaBgd = 0;
            sample->data.fl[0] = (float)Icol[i][j].R;
            sample->data.fl[1] = (float)Icol[i][j].G;
            sample->data.fl[2] = (float)Icol[i][j].B;
            int responseFgd = cvRound(em_modelFgd.predict( sample, probsFgd ));
            int responseBgd = cvRound(em_modelBgd.predict( sample, probsBgd ));
            for(int k=0; k<10;k++)
            {
            	probaFgd += probsFgd->data.fl[k]/10;
            	probaBgd += probsFgd->data.fl[k]/10;
            }
            EdataB[i*Icol.getWidth() + j] = -log(probaBgd);
            EdataF[i*Icol.getWidth() + j] = -log(probaFgd);
        }
    }
}

void SegmentMotion::computeLikelihood(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol)
{
	vpColVector errorPix(5);
	vpColVector data;
	apKernel density(5,0);
	double probaBgd = 0;
	double probaFgd = 0;
	double dsty;
	int l;

	double eta = 2 ;

	double t0 = vpTime::measureTimeMs();
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

				   /*dsty = (Icol[i][j].R - colBgd[k*30][0])*(Icol[i][j].R - colBgd[k*30][0])/bwidth[0] + (Icol[i][j].G - colBgd[k*30][1])*(Icol[i][j].G - colBgd[k*30][1])/bwidth[1] +
						   (Icol[i][j].B - colBgd[k*30][2])*(Icol[i][j].B - colBgd[k*30][2])/bwidth[2] + (i - colBgd[k*30][3])*(i - colBgd[k*30][3])/bwidth[3] + (j - colBgd[k*30][4])*(j - colBgd[k*30][4])/bwidth[4];
				   if(dsty < 1)
				   {
					   dsty =  0.75*(1-dsty);}
				   else
					   dsty = 0;

				   proba +=dsty;*/

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

			for (int k = 0 ; k < colFgd.size()/1 ; k++)
			{
				data = colFgd[k*1];
				errorPix[0] = Icol[i][j].R - data[0];
				errorPix[1] = Icol[i][j].G - data[1];
				errorPix[2] = Icol[i][j].B - data[2];
				errorPix[3] = i - data[3];
				errorPix[4] = j - data[4];
				/*errorPix[3] = 0;
				errorPix[4] = 0;*/
				probaFgd+=density.KernelDensity(errorPix);
			}
			probaFgd*=1;
			probaFgd/=colFgd.size();

			EdataHistF[i*Icol.getWidth() + j] = abs(eta*(log(1-(double)(histBg[0].get((int)Icol[i][j].R))/nbg) +
					log(1-(double)(histBg[1].get((int)Icol[i][j].G))/nbg) +
					log(1-(double)(histBg[2].get((int)Icol[i][j].B))/nbg)));

			//std::cout << " hist " << (double)EdataHistF[i*Icol.getWidth() + j] << std::endl;

			if (probaFgd>0)
			{EdataF[i*Icol.getWidth() + j] = -log(probaFgd) + EdataHistF[i*Icol.getWidth() + j];
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

void SegmentMotion::computeLikelihoodHist(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol)
{
	vpColVector errorPix(5);
	vpColVector data;
	apKernel density(5,0);
	double probaBgd = 0;
	double probaFgd = 0;
	double dsty;
	int l;

	double eta = 2;

	double t0 = vpTime::measureTimeMs();
	for (int i = 0; i<Icol.getHeight(); i++)
		for (int j =0 ; j < Icol.getWidth() ; j++)
		{

			EdataF[i*Icol.getWidth() + j] = abs(eta*(log(1-(double)(histBg[0].get((int)Icol[i][j].R))/nbg) +
					log(1-(double)(histBg[1].get((int)Icol[i][j].G))/nbg) +
					log(1-(double)(histBg[2].get((int)Icol[i][j].B))/nbg)));

			EdataB[i*Icol.getWidth() + j] = abs(eta*(log(1-(double)(histFg[0].get((int)Icol[i][j].R))/nfg) +
					log(1-(double)(histFg[1].get((int)Icol[i][j].G))/nfg) +
					log(1-(double)(histFg[2].get((int)Icol[i][j].B))/nfg)));

		}
	double t1 = vpTime::measureTimeMs();
}


void SegmentMotion::computeSpatialEnergy(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol)
{
	double esmooth;
	double gamma;
	double mu;
	std::vector<double> dist;
	std::vector<double> distColV,distColH;
	double d;
	double dColV,dColH;
	double meandistH,meandistV;
	double energy = 0;
	vpImagePoint p1,p2;
	for (int i = 0; i<Icol.getHeight(); i++)
		for (int j =0 ; j < Icol.getWidth() ; j++)
		{
			if(j>1 && j<Icol.getWidth()-2 && i>1 && i<Icol.getHeight()-2 )
		{
		dist.resize(0);
		distColV.resize(0);
		distColH.resize(0);
		for (int k = -1 ; k < 2 ; k++)
			for (int l = -1 ; l < 2 ; l++)
				{
				if(l!=k)
					{
						d = sqrt((k-l)*(k-l));
						dist.push_back(d);
						dColV = (Icol[i+k][j].R - Icol[i+l][j].R)*(Icol[i+k][j].R - Icol[i+l][j].R) +
								(Icol[i+k][j].G - Icol[i+l][j].G)*(Icol[i+k][j].G - Icol[i+l][j].G)
								+ (Icol[i+k][j].B - Icol[i+l][j].B)*(Icol[i+k][j].B - Icol[i+l][j].B);
						distColV.push_back(dColV);
						meandistV += dColV;
						dColH = (Icol[i][j+k].R - Icol[i][j+l].R)*(Icol[i][j+k].R - Icol[i][j+l].R) +
								(Icol[i][j+k].G - Icol[i][j+l].G)*(Icol[i][j+k].G - Icol[i][j+l].G)
								+ (Icol[i][j+k].B - Icol[i][j+l].B)*(Icol[i][j+k].B - Icol[i][j+l].B);
						distColH.push_back(dColH);
						meandistH += dColH;
					}
				}

		if (distColV.size()>0)
		{
		meandistV/=distColV.size();
		esmooth = 0;
		for (int ii = 0;ii < distColV.size() ; ii++)
		{
			esmooth += 0.3*(exp(-distColV[ii]/(2*meandistV)))/dist[ii];
		}

		EsmoothV[i*Icol.getWidth() + j] = esmooth;
		}
		else EsmoothV[i*Icol.getWidth() + j] = 0;

		if (distColH.size()>0)
			{
			meandistH/=distColH.size();
			esmooth = 0;
			for (int ii = 0;ii < distColH.size() ; ii++)
			{
			esmooth += 0.3*(exp(-distColH[ii]/(2*meandistH)))/dist[ii];
			}

			EsmoothH[i*Icol.getWidth() + j] = esmooth;
			}
			else EsmoothH[i*Icol.getWidth() + j] = 0;
		}
			else
			{
				EsmoothH[i*Icol.getWidth() + j] = 0;
			}

		}
}

/*void SegmentMotion::minimizeEnergy(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol)
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

void SegmentMotion::minimizeEnergy(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol, vpImage<vpRGBa> &Iseg)
{
	unsigned char* mask = new unsigned char[resolution];
	minEnergy.minEnergy(EdataB,EdataF,EsmoothH,EsmoothV, mask, I);
	for (int l =0; l<resolution ; l++)
	{
		label[l] = (int)mask[l];
	}

	vpImagePoint p1;
	Iseg = Icol;

	/*int *mask_;
	mask_ = minEnergy.mask;*/
	for (int i = 0; i<I.getHeight(); i++)
		for (int j =0 ; j < I.getWidth() ; j++)
		{
			p1.set_i(i);
			p1.set_j(j);
			//std::cout << minEnergy.mask[i*I.getWidth() + j] << std::endl;
			if(label[i*I.getWidth() + j] == 0)
				{Iseg[i][j].R = 0;
				Iseg[i][j].G = 0;
				Iseg[i][j].B = 0;

				}
		}
}

void SegmentMotion::RGB2YUV(vpImage<vpRGBa> &_I,vpImage<vpRGBa> &I_)
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

void SegmentMotion::computeColorLikelihoodHist(vpImage<vpRGBa> &_I)
{
	int nbins = 256;

	double eta = 0;

    std::vector<unsigned int *>  histfg;
    std::vector<unsigned int *>  histbg;
    histfg.resize(3);
    histbg.resize(3);

    histFg.resize(3);
    histBg.resize(3);

    for(int l =0; l<3; l++)
    {
    	histfg[l] = new unsigned int[nbins];
    	histbg[l] = new unsigned int[nbins];
    	memset(histfg[l], 0, nbins * sizeof(unsigned int));
    	memset(histbg[l], 0, nbins * sizeof(unsigned int));
    }


    std::vector<std::vector<int> > dataFg;
    dataFg.resize(3);

    std::vector<std::vector<int> > dataBg;
    dataBg.resize(3);
    nfg = 0;
    nbg = 0;

    for (unsigned int i=0; i < _I.getHeight(); i ++) {
    	for (unsigned int j=0; j < _I.getWidth(); j ++) {
    		if (label[i*_I.getWidth() + j] == 0)
    		{
    			histbg[0][ (int)_I[i][j].R ] ++;
    			histbg[1][ (int)_I[i][j].G ] ++;
    			histbg[2][ (int)_I[i][j].B ] ++;
    			nbg ++;
    	}
    		else
    		{
    			histfg[0][ (int)_I[i][j].R ] ++;
    			histfg[1][ (int)_I[i][j].G ] ++;
    			histfg[2][ (int)_I[i][j].B ] ++;
    			nfg ++;

    		}
    	}
    }


    for(int l =0; l<3; l++)
    {
     for (unsigned k=0; k < nbins ; k ++) {
     histFg[l].set(k,histfg[l][k]);
     histBg[l].set(k,histbg[l][k]);
     }
     /*histFg[l].smooth(3);
     histBg[l].smooth(3);

     for (int k = 0; k<nbins; k++)
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



