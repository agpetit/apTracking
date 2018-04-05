//#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <cv.h>

#include <visp/vpHomography.h>
#include <visp/vpPixelMeterConversion.h>


#define BORN_MAX(val,max) ((val)>(max)?(max):(val))
#define BORN_MIN(val,min) ((val)<(min)?(min):(val))

using namespace std;

class apRANSAC
{
public :

int PointsToBuildP;
double BACKGROUND_THRESHOLD;
int MAX_RANSAC_ITER;
double CONSENSUS;
int MINIMAL_SIZE_MODEL;
int HISTORYOFPOSITION;
int NBPOINTSTOTRACK;

void RANSACFunction_V2( std::vector<std::vector<CvPoint2D32f> > &trajectory, int *nb_trajectory, int idx_frame,
			int best_label_motion[800]);

void RANSACFunctionHomography (std::vector<std::vector<CvPoint2D32f> > &trajectory, int *nb_trajectory, int nbPointToTrack,int idx_frame, vpHomography &H_, int Frame, int best_label_motion[800]);

void RANSACFunctionH0 (std::vector<std::vector<CvPoint2D32f> > &trajectory, int *nb_trajectory, int nbPointToTrack,int idx_frame, vpHomography &H_, int Frame, int *best_label_motion);


void RANSACFunctionH (std::vector<std::vector<CvPoint2D32f> > &trajectory, int *nb_trajectory, int nbPointToTrack,int idx_frame, vpHomography &H_, int Frame,  int *best_label_motion, int deltaH);


void RANSACFunction (std::vector<std::vector<CvPoint2D32f> > &trajectory, int *nb_trajectory, int nbPointToTrack,int idx_frame, int *best_label_motion, int height, int width);

void getSubSet ( int *nb_trajectory, int *idxModel , int *size );

char isTaken(int temp, int *idxPointToBuildP, int k);

void computePMatrix(int *idx_model, std::vector<std::vector<CvPoint2D32f> > &trajectory,
					int *nb_trajectory, int hist, vpMatrix *P);

int  computeInlier(std::vector<std::vector<CvPoint2D32f> > &trajectory, int *nb_trajectory, int hist, vpMatrix P);
};
