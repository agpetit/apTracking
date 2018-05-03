#include <string.h>
#include <visp/vpImage.h>
//#include <visp/vpDisplayGTK.h> 
#include <visp/vpDisplayX.h> 
#include <visp/vpConfig.h>
#include <visp/vpVideoReader.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>
#include <visp/vpFeatureVanishingPoint.h>
#include <visp/vpRansac.h>
#include <visp/vpKltOpencv.h>
#include <visp/vpColVector.h>

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


#define NBFEATURESMAX 200

using namespace std;

/*class Tools
{

public:
	
	//static void GetFluxFromBitMap(LAR *LarParam, vpImage<vpRGBa> I, unsigned char *Read_YUV_Y, 
	//				   unsigned char *Read_YUV_U, unsigned char *Read_YUV_V,
	//				    short *Read_YUV_Y_short, short *Read_YUV_U_short, short *Read_YUV_V_short);

};*/


//void MarkerSegmentation(LAR *LarParams, CvPoint2D32f *frame1_features, 
//						CvPoint2D32f *frame2_features, char *optical_flow_found_feature, int number_of_features );

//void GetBorderToTrack(REGION ***RAG, int *nbRAG, CvPoint2D32f *frame_features, int *number_of_features);

void ComputeVanishingPoint (IplImage* src,  CvPoint *vanishingPoint1, CvPoint *vanishingPoint2  );


void FusionPreviousFeatures(CvPoint2D32f *frame1_features , CvPoint2D32f *frame2_features, int *number_of_features, 
							char *optical_flow_found_feature, CvPoint2D32f *temp_feat, int nb_temp_feat);

void allocateOnDemand( IplImage **img, CvSize size, int depth, int channels );

double square(int a);

void RANSACFunction (CvPoint2D32f  trajectory[100][1000], int *nb_trajectory, int nbPointToTrack,int idx_frame, int label_motion[100]);
//#endif


