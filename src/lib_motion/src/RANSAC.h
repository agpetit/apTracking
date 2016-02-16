#include "opencv2/core/types_c.h"
#include "opencv2/imgproc/imgproc.hpp"
//#include "d:\Tools\openCV2\opencv_Build\install\include\opencv2\core\types_c.h"
//#include "/usr/local/include/opencv2/core/types_c.h"
#include "lib_motion.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <cv.h>


#define BORN_MAX(val,max) ((val)>(max)?(max):(val))
#define BORN_MIN(val,min) ((val)<(min)?(min):(val))
#define PointsToBuildP  5
#define BACKGROUND_THRESHOLD 0.5
#define MAX_RANSAC_ITER 100
#define CONSENSUS 0.5
#define MINIMAL_SIZE_MODEL 100


class RANSAC
{
public :
void RANSACFunction_V2( std::vector<std::vector<CvPoint2D32f> > trajectory, int *nb_trajectory, int idx_frame,
			int best_label_motion[600]);

void RANSACFunction (std::vector<std::vector<CvPoint2D32f> > trajectory, int *nb_trajectory, int nbPointToTrack,int idx_frame, int best_label_motion[600]);

void getSubSet ( int *nb_trajectory, int *idxModel , int *size );

char isTaken(int temp, int *idxPointToBuildP, int k);

void computePMatrix(int *idx_model, std::vector<std::vector<CvPoint2D32f> > trajectory,
					int *nb_trajectory, int hist, vpMatrix *P);

int  computeInlier(std::vector<std::vector<CvPoint2D32f> > trajectory, int *nb_trajectory, int hist, vpMatrix P);
};
