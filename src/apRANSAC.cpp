#include "apRANSAC.h"


void apRANSAC::RANSACFunction (std::vector<std::vector<CvPoint2D32f> > &trajectory, int *nb_trajectory, int nbPointToTrack, int idx_frame, int *best_label_motion, int height, int width){


	int nbFrame = HISTORYOFPOSITION;
	int nb_inlier = 0;
	int nb_computed_point;


	CvRNG rng;
	int nbInlierThreshold  = 50;
	int nbIteration = 100;
	int nb_iter_rech_model = 200;
	int idx = 0;
	int nbPoint = nbPointToTrack;

	srand ( time(NULL) );
	int threshold_inlier = nbPointToTrack * CONSENSUS;
	int max_inlier = 0;
	int max_point_computed = 0;
	int ransac_iter = 0;


	while ( nb_inlier < threshold_inlier && ransac_iter < MAX_RANSAC_ITER)
	{
		int label_motion[NBPOINTSTOTRACK];
		int size = 0; int nb_iter = 0;
		int idxMaxPointToBuildP[PointsToBuildP];
		int sizePointToBuildP[PointsToBuildP];
		int size_max = 0;
		nbFrame = HISTORYOFPOSITION ;

		// get subset points with the minimal size of information, rectangular matrix 2*size x nbPointToBuildP 

		while (size < 10 && nb_iter < nb_iter_rech_model)
			{
				int idxPointToBuildP[PointsToBuildP];
				for (int k = 0; k < PointsToBuildP; k++) idxPointToBuildP[k] = -1;

				int size_temp = 999;
				for (int k = 0; k < PointsToBuildP; k++)
				{
					int temp = cvRandInt( &rng ) % nbPointToTrack;
					while (isTaken(temp, idxPointToBuildP, k))
							temp = cvRandInt( &rng ) % nbPointToTrack;
					idxPointToBuildP[k] = temp;
					sizePointToBuildP[k] = BORN_MAX(nb_trajectory[idxPointToBuildP[k]], nbFrame);
					size_temp = min (size_temp, sizePointToBuildP[k]);
				}
				size = size_temp;
				if (size > size_max)
				{
					size_max = size;
					for (int k = 0; k < PointsToBuildP; k++)
						idxMaxPointToBuildP[k] = idxPointToBuildP[k];

				}
				nb_iter++;
			}

		if (size_max > 0 )
		{
			if (size_max <= 15)
			{
				size = size_max;
				//for (int k = 0; k < PointsToBuildP; k++)
				//	size = min(size , nb_trajectory[idxMaxPointToBuildP[k]]);
			}

			if (size_max > 0)
			{
				nbFrame = size_max;
				
				vpMatrix a(nbFrame * 2, PointsToBuildP );
				vpMatrix a_t( PointsToBuildP , nbFrame * 2);
				vpMatrix a_t_a( nbFrame * 2,  nbFrame * 2);
				vpMatrix a_t_a_1( nbFrame * 2, nbFrame * 2);
				vpMatrix a_t_a_1_a_t( nbFrame * 2, nbFrame * 2);
				vpMatrix a_p(PointsToBuildP ,  nbFrame * 2);
				vpMatrix a_p_t(nbFrame * 2, PointsToBuildP );
				vpMatrix P (nbFrame * 2,nbFrame * 2);
	
				for (int i = 0; i < PointsToBuildP; i++)
				{
					int min_born = nb_trajectory[idxMaxPointToBuildP[i]] - size;
					int ligne = 0;
					for (int j = nb_trajectory[idxMaxPointToBuildP[i]]-1  ; j >= min_born ; j--)
					{
						a	[2*ligne][i]	= trajectory[idxMaxPointToBuildP[i]][j].x;
						a	[2*ligne+1][i]	= trajectory[idxMaxPointToBuildP[i]][j].y;
						a_t	[i][2*ligne]	= trajectory[idxMaxPointToBuildP[i]][j].x;
						a_t	[i][2*ligne+1]	= trajectory[idxMaxPointToBuildP[i]][j].y;
						ligne++;
					}
				}
	
			a_t_a = a.AtA();
			a_t_a_1 = a_t_a.inverseByLU();
			vpMatrix::mult2Matrices(a_t_a_1, a_t, a_t_a_1_a_t);
			vpMatrix::mult2Matrices(a, a_t_a_1_a_t, P);

			nb_inlier = 0;
			nb_computed_point = 0;
			for (int i = 0; i < nbPointToTrack; i++)
			{

				vpMatrix t1 (nbFrame * 2, 1);
				vpMatrix t1_temp (nbFrame * 2, 1);
				vpMatrix t1_temp_2 (nbFrame * 2, 1);
				int ligne = 0;
				int min_born =  (nb_trajectory[i] - size >= 0)? (nb_trajectory[i] - size): (-1);
				if (min_born >=0)
				{
					nb_computed_point ++;
					for (int j = nb_trajectory[i]-1  ; j >= min_born ; j--)
					{
						t1	[2*ligne ][0]	= trajectory[i][j].x;
						t1	[2*ligne +1][0]	= trajectory[i][j].y;
						ligne++;
					}
					vpMatrix::mult2Matrices(P, t1, t1_temp);
					vpMatrix::add2WeightedMatrices(t1_temp,1.0, t1, -1.0, t1_temp_2);
					double t1_norme = t1_temp_2.euclideanNorm();
					//printf("%f \n", t1_norme);
					if (t1_norme < BACKGROUND_THRESHOLD)
					{
						nb_inlier++;
						label_motion[i] = 0;
					}
					else 
						label_motion[i] = 1;
				}
				else 
					label_motion[i] = -1;
			}

			}
		}

		int xp,yp;
		int n_fg = 0;
		int n_bg = 0;

		for (int i = 0; i < nbPointToTrack; i++)
		{
			xp = trajectory[i][nb_trajectory[i]-1].x;
			yp = trajectory[i][nb_trajectory[i]-1].y;
			if (xp < width/10 || xp > width - (int)width/10 || yp < height/10 || yp > height - (int)height/10)
				if(label_motion[i] == 0)
					n_bg++;
				else
					n_fg++;
		}

			
		if (nb_inlier > max_inlier)
		{
			max_inlier =  nb_inlier;
			max_point_computed = nb_computed_point;
			if (n_bg > n_fg)
			{
			for (int n = 0; n < nbPointToTrack; n++)
				best_label_motion[n] = label_motion[n];
			}
			else
			{
			for (int n = 0; n < nbPointToTrack; n++)
				best_label_motion[n] = 1-label_motion[n];
			}
			//memcpy(best_label_motion, label_motion,NBPOINTSTOTRACK * sizeof(int));
		}

		//printf("all %d , inlier %d \n", nb_computed_point, nb_inlier);
		ransac_iter++;
	}

	printf("iter = %3d | total %3d | nbPointUsed %3d | nbInlier %3d ",ransac_iter, nbPoint,  max_point_computed, max_inlier);

	
}

void apRANSAC::RANSACFunctionHomography (std::vector<std::vector<CvPoint2D32f> > &trajectory, int *nb_trajectory, int nbPointToTrack,int idx_frame, vpHomography &H_, int Frame, int best_label_motion[800]){


	int nbFrame = HISTORYOFPOSITION;
	int nb_inlier = 0;
	int nb_computed_point = 0;
    vpHomography H, aHb;

    double residual = 1;

    std::vector<double> xav;
    std::vector<double> yav;
    std::vector<double> xbv;
    std::vector<double> ybv;

	int size = 3;
	int nb_iter = 0;
	int idxMaxPointToBuildP[PointsToBuildP];
	int sizePointToBuildP[PointsToBuildP];
	int size_max = 0;
	CvRNG rng;


	/*while (size < 10 )
		{
			int idxPointToBuildP[PointsToBuildP];
			for (int k = 0; k < PointsToBuildP; k++) idxPointToBuildP[k] = -1;

			int size_temp = 999;
			for (int k = 0; k < PointsToBuildP; k++)
			{
				int temp = cvRandInt( &rng ) % nbPointToTrack;
				while (isTaken(temp, idxPointToBuildP, k))
						temp = cvRandInt( &rng ) % nbPointToTrack;
				idxPointToBuildP[k] = temp;
				sizePointToBuildP[k] = BORN_MAX(nb_trajectory[idxPointToBuildP[k]], nbFrame);
				size_temp = min (size_temp, sizePointToBuildP[k]);
			}
			size = size_temp;
			if (size > size_max)
			{
				size_max = size;
				for (int k = 0; k < PointsToBuildP; k++)
					idxMaxPointToBuildP[k] = idxPointToBuildP[k];

			}
			nb_iter++;
		}*/

	std::cout << " size " << size << std::endl;
	int ntrajmin = 1000;

	for (int i = 0; i < 800; i++)
	{
		if(nb_trajectory[i] < ntrajmin)
		ntrajmin = nb_trajectory[i];
	}

	int min_born = max (0,ntrajmin - size);

	for (int i = 0; i < 800; i++)
	{

		int ligne = 0;
		//int min_born =  (nb_trajectory[i] - size >= 0)? (nb_trajectory[i] - size): (-1);
		if (min_born >=0)
		{
			nb_computed_point ++;
			/*vpPixelMeterConversion::convertPoint(cam,trajectory[i][nb_trajectory[i]-1 ].x,trajectory[i][nb_trajectory[i]-1 ].y,xa_,ya_);
			vpPixelMeterConversion::convertPoint(cam,trajectory[i][min_born].x,trajectory[i][min_born].y,xb_,yb_);
			xav.push_back(xa_);
			yav.push_back(ya_);
			xbv.push_back(xb_);
			ybv.push_back(yb_);*/

				xav.push_back(trajectory[i][nb_trajectory[i]-1 ].x);
				yav.push_back(trajectory[i][nb_trajectory[i]-1 ].y);
				xbv.push_back(trajectory[i][min_born].x);
				ybv.push_back(trajectory[i][min_born].y);
		}
	}

    double *xa = new double[nb_computed_point];
    double *xb = new double[nb_computed_point];
    double *ya = new double[nb_computed_point];
    double *yb = new double[nb_computed_point];

	for (int i = 0; i < nb_computed_point; i++)
	{
				xa[i] = xav[i];
				ya[i] = yav[i];
				xb[i] = xbv[i];
				yb[i] = ybv[i];
				std::cout << " nb points " << xa[i] << " ya " << ya[i] << " xb " << xb[i] << " yb " << yb[i] << std::endl;

	}

	vpColVector inliers(nb_computed_point);
	int consensus = 200;
	double threshold = 2;
	int area = 50;

    Frame = min_born;
    std::cout << " nb points " << Frame << std::endl;
    H.ransac(nb_computed_point, xb, yb, xa, ya, aHb, inliers, residual, consensus, threshold, area);


	for (int i = 0; i < nb_computed_point ; i++)
	{
		best_label_motion[i] = inliers[i];
	}
    std::cout << " rows " << inliers.getRows() << std::endl;
    H_ = aHb;

}

void apRANSAC::RANSACFunctionH0 (std::vector<std::vector<CvPoint2D32f> > &trajectory, int *nb_trajectory, int nbPointToTrack,int idx_frame, vpHomography &H_, int Frame, int *best_label_motion){


	//int nbFrame = HISTORYOFPOSITION;
	int nb_inlier = 0;
	int nb_computed_point = 0;
    vpHomography H, aHb;

    double residual = 1;

    std::vector<double> xav;
    std::vector<double> yav;
    std::vector<double> xbv;
    std::vector<double> ybv;

	int size = 6;
	//int nb_iter = 0;
	//int idxMaxPointToBuildP[PointsToBuildP];
	//int sizePointToBuildP[PointsToBuildP];
	//int size_max = 0;
	//CvRNG rng;

	/*while (size < 10 )
		{
			int idxPointToBuildP[PointsToBuildP];
			for (int k = 0; k < PointsToBuildP; k++) idxPointToBuildP[k] = -1;

			int size_temp = 999;
			for (int k = 0; k < PointsToBuildP; k++)
			{
				int temp = cvRandInt( &rng ) % nbPointToTrack;
				while (isTaken(temp, idxPointToBuildP, k))
						temp = cvRandInt( &rng ) % nbPointToTrack;
				idxPointToBuildP[k] = temp;
				sizePointToBuildP[k] = BORN_MAX(nb_trajectory[idxPointToBuildP[k]], nbFrame);
				size_temp = min (size_temp, sizePointToBuildP[k]);
			}
			size = size_temp;
			if (size > size_max)
			{
				size_max = size;
				for (int k = 0; k < PointsToBuildP; k++)
					idxMaxPointToBuildP[k] = idxPointToBuildP[k];

			}
			nb_iter++;
		}*/

	int ntrajmin = 1000;

	/*for (int i = 0; i < 800; i++)
	{
		if(nb_trajectory[i] < ntrajmin)
		ntrajmin = nb_trajectory[i];
	}*/

	//std::cout << " ntrajmin " << ntrajmin<< std::endl;
	//int size_ = min(ntrajmin)
	//int min_born = max (0,ntrajmin - size);
	int min_born;

	for (int i = 0; i < NBPOINTSTOTRACK; i++)
	{

		int ligne = 0;
		//int min_born =  (nb_trajectory[i] - size >= 0)? (nb_trajectory[i] - size): (-1);
		min_born = max (0,nb_trajectory[i] - size);
		//std::cout << " ntrajmin " <<nb_trajectory[i] << " " << min_born << std::endl;
		if (min_born >=0)
		{
			nb_computed_point ++;
			/*vpPixelMeterConversion::convertPoint(cam,trajectory[i][nb_trajectory[i]-1 ].x,trajectory[i][nb_trajectory[i]-1 ].y,xa_,ya_);
			vpPixelMeterConversion::convertPoint(cam,trajectory[i][min_born].x,trajectory[i][min_born].y,xb_,yb_);
			xav.push_back(xa_);
			yav.push_back(ya_);
			xbv.push_back(xb_);
			ybv.push_back(yb_);*/
				xav.push_back(trajectory[i][nb_trajectory[i]-1 ].x);
				yav.push_back(trajectory[i][nb_trajectory[i]-1 ].y);
				xbv.push_back(trajectory[i][min_born].x);
				ybv.push_back(trajectory[i][min_born].y);
				//std::cout << " nb points " <<nb_computed_point << " " << trajectory[i][nb_trajectory[i]-1 ].x << " ya " << trajectory[i][nb_trajectory[i]-1 ].y << " xb " << trajectory[i][min_born].x << " yb " << trajectory[i][min_born].y << std::endl;
		}
	}

    double *xa = new double[nb_computed_point];
    double *xb = new double[nb_computed_point];
    double *ya = new double[nb_computed_point];
    double *yb = new double[nb_computed_point];

	for (int i = 0; i < nb_computed_point; i++)
	{
				xa[i] = xav[i];
				ya[i] = yav[i];
				xb[i] = xbv[i];
				yb[i] = ybv[i];
				//std::cout << " nb points " <<nb_computed_point << " " << xa[i] << " ya " << ya[i] << " xb " << xb[i] << " yb " << yb[i] << std::endl;

	}

	vpColVector inliers(nb_computed_point);
	int consensus = 200;
	double threshold = 1;
	int area = 0;

    //Frame = min_born;
	Frame = size;
    //std::cout << " nb points " << Frame << std::endl;
    H.ransac(nb_computed_point, xb, yb, xa, ya, aHb, inliers, residual, consensus, threshold, area);


	for (int i = 0; i < nb_computed_point ; i++)
	{
		best_label_motion[i] = inliers[i];
	}
    std::cout << " rows " << inliers.getRows() << std::endl;
    H_ = aHb;

    delete [] xa;
    delete [] xb;
    delete [] ya;
    delete [] yb;


}


void apRANSAC::RANSACFunctionH (std::vector<std::vector<CvPoint2D32f> > &trajectory, int *nb_trajectory, int nbPointToTrack,int idx_frame, vpHomography &H_, int Frame, int *best_label_motion, int deltaH){


	//int nbFrame = HISTORYOFPOSITION;
	int nb_inlier = 0;
	int nb_computed_point = 0;
    vpHomography H, aHb;

    double residual = 1;

    std::vector<double> xav;
    std::vector<double> yav;
    std::vector<double> xbv;
    std::vector<double> ybv;

	int size = deltaH;
	//int nb_iter = 0;
	//int idxMaxPointToBuildP[PointsToBuildP];
	//int sizePointToBuildP[PointsToBuildP];
	//int size_max = 0;
	//CvRNG rng;

	/*while (size < 10 )
		{
			int idxPointToBuildP[PointsToBuildP];
			for (int k = 0; k < PointsToBuildP; k++) idxPointToBuildP[k] = -1;

			int size_temp = 999;
			for (int k = 0; k < PointsToBuildP; k++)
			{
				int temp = cvRandInt( &rng ) % nbPointToTrack;
				while (isTaken(temp, idxPointToBuildP, k))
						temp = cvRandInt( &rng ) % nbPointToTrack;
				idxPointToBuildP[k] = temp;
				sizePointToBuildP[k] = BORN_MAX(nb_trajectory[idxPointToBuildP[k]], nbFrame);
				size_temp = min (size_temp, sizePointToBuildP[k]);
			}
			size = size_temp;
			if (size > size_max)
			{
				size_max = size;
				for (int k = 0; k < PointsToBuildP; k++)
					idxMaxPointToBuildP[k] = idxPointToBuildP[k];

			}
			nb_iter++;
		}*/

	int ntrajmin = 1000;

	/*for (int i = 0; i < 800; i++)
	{
		if(nb_trajectory[i] < ntrajmin)
		ntrajmin = nb_trajectory[i];
	}*/

	//std::cout << " ntrajmin " << ntrajmin<< std::endl;
	//int size_ = min(ntrajmin)
	//int min_born = max (0,ntrajmin - size);
	int min_born;

	for (int i = 0; i < NBPOINTSTOTRACK; i++)
	{
		if(best_label_motion[i] == 0)
		{

		int ligne = 0;
		//int min_born =  (nb_trajectory[i] - size >= 0)? (nb_trajectory[i] - size): (-1);
		min_born = max (0,nb_trajectory[i] - size);
		//std::cout << " ntrajmin " <<nb_trajectory[i] << " " << min_born << std::endl;
		if (min_born >=0)
		{
			nb_computed_point ++;
			/*vpPixelMeterConversion::convertPoint(cam,trajectory[i][nb_trajectory[i]-1 ].x,trajectory[i][nb_trajectory[i]-1 ].y,xa_,ya_);
			vpPixelMeterConversion::convertPoint(cam,trajectory[i][min_born].x,trajectory[i][min_born].y,xb_,yb_);
			xav.push_back(xa_);
			yav.push_back(ya_);
			xbv.push_back(xb_);
			ybv.push_back(yb_);*/
				xav.push_back(trajectory[i][nb_trajectory[i]-1 ].x);
				yav.push_back(trajectory[i][nb_trajectory[i]-1 ].y);
				xbv.push_back(trajectory[i][min_born].x);
				ybv.push_back(trajectory[i][min_born].y);
				//std::cout << " nb points " <<nb_computed_point << " " << trajectory[i][nb_trajectory[i]-1 ].x << " ya " << trajectory[i][nb_trajectory[i]-1 ].y << " xb " << trajectory[i][min_born].x << " yb " << trajectory[i][min_born].y << std::endl;
		}
		}
	}

    double *xa = new double[nb_computed_point];
    double *xb = new double[nb_computed_point];
    double *ya = new double[nb_computed_point];
    double *yb = new double[nb_computed_point];

	for (int i = 0; i < nb_computed_point; i++)
	{
				xa[i] = xav[i];
				ya[i] = yav[i];
				xb[i] = xbv[i];
				yb[i] = ybv[i];
				//std::cout << " nb points " <<nb_computed_point << " " << xa[i] << " ya " << ya[i] << " xb " << xb[i] << " yb " << yb[i] << std::endl;

	}

	vpColVector inliers(nb_computed_point);
	int consensus = 200;
	double threshold = 1;
	int area = 0;

    //Frame = min_born;
	Frame = size;
    //std::cout << " nb points " << Frame << std::endl;
    H.ransac(nb_computed_point, xb, yb, xa, ya, aHb, inliers, residual, consensus, threshold, area);


	/*for (int i = 0; i < nb_computed_point ; i++)
	{
		//best_label_motion[i] = inliers[i];
	}*/
    std::cout << " rows " << inliers.getRows() << std::endl;
    H_ = aHb;

    delete [] xa;
    delete [] xb;
    delete [] ya;
    delete [] yb;


}


void apRANSAC::RANSACFunction_V2( std::vector<std::vector<CvPoint2D32f> > &trajectory, int *nb_trajectory, int idx_frame,
				int best_label_motion[800]){

	int nbFrame = HISTORYOFPOSITION ;
	int nb_inlier = 0;
	int nb_computed_point;

	int nbInlierThreshold  = 10;
	int nbIteration = 100;
	int threshold_inlier = 15;
	int ransac_iter = 0;
	

	while ( nb_inlier < threshold_inlier && ransac_iter < MAX_RANSAC_ITER)
	{
		int idxModel [PointsToBuildP];
		int hist[1];
		getSubSet(nb_trajectory, idxModel, hist);
		vpMatrix *P;
		if (hist[0] > 0)
		{

			computePMatrix( idxModel, trajectory, nb_trajectory, hist[0], P);
			nb_inlier  = computeInlier(trajectory, nb_trajectory, hist[0], *P);
		}
		ransac_iter++;
	}
}

char apRANSAC::isTaken(int temp, int *idxPointToBuildP, int k){
	
	for (int i = 0; i < k; i++)
		if(idxPointToBuildP[i] == temp)
			return 1;

	return 0;
}


void apRANSAC::getSubSet ( int *nb_trajectory, int *idxModel , int *size ){

	CvRNG rng;
	int nb_iter_rech_model = 50;
	int nb_iter = 0;
	int idxMaxPointToBuildP[PointsToBuildP];
	int sizePointToBuildP[PointsToBuildP];
	int size_max = 0;

	while (size_max < MINIMAL_SIZE_MODEL && nb_iter < nb_iter_rech_model)
	{
		int idxPointToBuildP[PointsToBuildP];
		int size_temp = 999;

		for (int k = 0; k < PointsToBuildP; k++) 
			idxPointToBuildP[k] = -1;

		for (int k = 0; k < PointsToBuildP; k++)
		
		{
			int temp = cvRandInt( &rng ) % NBPOINTSTOTRACK;
			while (isTaken(temp, idxPointToBuildP, k))
					temp = cvRandInt( &rng ) % NBPOINTSTOTRACK;
			idxPointToBuildP[k] = temp;
			sizePointToBuildP[k] = BORN_MAX(nb_trajectory[idxPointToBuildP[k]], HISTORYOFPOSITION);
			size_temp = min (size_temp, sizePointToBuildP[k]);
		}

		if (size_temp > size_max)
		{
			size_max = size_temp;
			for (int k = 0; k < PointsToBuildP; k++)
				idxModel[k] = idxPointToBuildP[k];
		}
		nb_iter++;
	}
		
	printf("%d\tfrom ", size_max);
	for (int k = 0; k < PointsToBuildP; k++)
		printf("%d\t", nb_trajectory[idxModel[k]]);
	printf("\n");
		
	size[0] = size_max;
}


void apRANSAC::computePMatrix(int *idx_model, std::vector<std::vector<CvPoint2D32f> > &trajectory, int *nb_trajectory, int hist, vpMatrix *P){

		
	int nbFrame = hist;

	vpMatrix a(nbFrame * 2, PointsToBuildP );
	vpMatrix a_t( PointsToBuildP , nbFrame * 2);
	vpMatrix a_t_a( nbFrame * 2,  nbFrame * 2);
	vpMatrix a_t_a_1( nbFrame * 2, nbFrame * 2);
	vpMatrix a_t_a_1_a_t( nbFrame * 2, nbFrame * 2);
	vpMatrix a_p(PointsToBuildP ,  nbFrame * 2);
	vpMatrix a_p_t(nbFrame * 2, PointsToBuildP );
	vpMatrix P_temp (nbFrame * 2,nbFrame * 2);

	for (int i = 0; i < PointsToBuildP; i++)
	{
		int min_born = nb_trajectory[idx_model[i]] - nbFrame;
		int ligne = 0;
		for (int j = nb_trajectory[idx_model[i]]-1  ; j >= min_born ; j--)
		{
			a	[2*ligne][i]	= trajectory[idx_model[i]][j].x;
			a	[2*ligne+1][i]	= trajectory[idx_model[i]][j].y;
			a_t	[i][2*ligne]	= trajectory[idx_model[i]][j].x;
			a_t	[i][2*ligne+1]	= trajectory[idx_model[i]][j].y;
			ligne++;
		}
	}

	a_t_a = a.AtA();
	a_t_a_1 = a_t_a.inverseByLU();
	vpMatrix::mult2Matrices(a_t_a_1, a_t, a_t_a_1_a_t);
	vpMatrix::mult2Matrices(a, a_t_a_1_a_t, P_temp);
	P = new vpMatrix(P_temp);
}

int  apRANSAC::computeInlier(std::vector<std::vector<CvPoint2D32f> > &trajectory, int *nb_trajectory, int hist, vpMatrix P){

	int nbFrame = hist;
	int nb_inlier = 0;
	int nb_computed_point = 0;
	for (int i = 0; i < NBPOINTSTOTRACK; i++)
	{

		vpMatrix t1 (nbFrame * 2, 1);
		vpMatrix t1_temp (nbFrame * 2, 1);
		vpMatrix t1_temp_2 (nbFrame * 2, 1);
		int ligne = 0;
		int min_born =  (nb_trajectory[i] - nbFrame >= 0)? (nb_trajectory[i] - nbFrame): (-1);
		if (min_born >=0)
		{
			nb_computed_point ++;
			for (int j = nb_trajectory[i]-1  ; j >= min_born ; j--)
			{
				t1	[2*ligne ][0]	= trajectory[i][j].x;
				t1	[2*ligne +1][0]	= trajectory[i][j].y;
				ligne++;
			}
			vpMatrix::mult2Matrices(P, t1, t1_temp);
			vpMatrix::add2WeightedMatrices(t1_temp,1.0, t1, -1.0, t1_temp_2);
			double t1_norme = t1_temp_2.euclideanNorm();
			//printf("%f \n", t1_norme);
			if (t1_norme < BACKGROUND_THRESHOLD)
			{
				nb_inlier++;
				//label_motion[i] = 0;
			}
			//else
				//label_motion[i] = 1;
		}
		//else
			//label_motion[i] = -1;
	}

	return nb_inlier;
}

