
#include "opencv2/core/types_c.h"
#include "opencv2/imgproc/imgproc.hpp"


#include "lib_motion.h"
#include "RANSAC.h"
//#include "D:\Tools\openCV2\opencv\modules\calib3d\src\modelest.cpp"
//#include "opencv2/calib3d/_modelest.h"
#include <stdlib.h>
#include <math.h>
#include <cv.h>





using namespace std;
using namespace cv;



void FusionPreviousFeatures(CvPoint2D32f *frame1_features , CvPoint2D32f *frame2_features, int *number_of_features, 
							char *optical_flow_found_feature, CvPoint2D32f *temp_feat, int nb_temp_feat)
							
{
	int i;
	int cpt = 0;
	int k = 0;
	CvPoint2D32f feat_resul [NBFEATURESMAX];
	for (i = 0; i<number_of_features[0]; i++)
	{
		if ( optical_flow_found_feature[i] == 1)
		{
			feat_resul [k].x = frame2_features[i].x ;
			feat_resul [k].y = frame2_features[i].y	;
			k++;
		}
	}

	if (k < NBFEATURESMAX)
	{
		for (i = 0; i<nb_temp_feat && k < NBFEATURESMAX; i++)
		{
			feat_resul [k].x = temp_feat[i].x ;
			feat_resul [k].y = temp_feat[i].y ;
			k++;

		}
	}

	for (i = 0; i< k; i++)
	{
			frame1_features[i].x = feat_resul [i].x ;
			frame1_features[i].y = feat_resul [i].y ;
	}
	*number_of_features = k;
}



void allocateOnDemand( IplImage **img, CvSize size, int depth, int channels )
{
	if ( *img != NULL )	return;
	*img = cvCreateImage( size, depth, channels );
	if ( *img == NULL )
	{
		fprintf(stderr, "Error: Couldn't allocate image.  Out of memory?\n");
		exit(-1);
	}
}

double square(int a)
{
	return a * a;
}

//void LabelingMotionRegions (MarkerCpt *featuresCpt, int *nbFeaturesCpt)
//{
//	int dist_ang;
//	int dist_hypo;
//	printf("label \t \size \t angle \t hypo \n");
//	for (int i = 0; i < nbFeaturesCpt[0]; i++)
//	{
//		MarkerCpt motionRegion = featuresCpt[i];
//		printf("%d \t %d \t %d \t %d \n", motionRegion.label, motionRegion.nb, motionRegion.ang, motionRegion.hyp);
//	}
//
//}

