#include <stdio.h>
#include <stdlib.h>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
//#include "RANSAC.h"
#include "tracker.h"
#include <time.h>
#include <visp/vpConfig.h>
#include <visp/vpVideoReader.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageConvert.h>
#include <visp/vpFeatureVanishingPoint.h>
#include <visp/vpRansac.h>
#include <visp/vpKltOpencv.h>
#include <visp/vpColVector.h>
#include <visp/vpMeLine.h>
#include <visp/vpIoTools.h>

#include "lib_motion.h"

#include <math.h>
#include <cv.h>

 
/*

*/

using namespace std;
using namespace cv;


#define		JHMSwithMotion 1
#ifdef		JHMSwithMotion

#define WITH_DISPLAY 1
#define _WITH_MOTION 1
#define _WITH_SEGMENTATION 1
#define _WITH_VANISHINGPOINT 1
#define _WITH_OPTICAL_FLOW 1
#define _WITH_SCALABILITY  1
#define LEVEL 0
 

int main (int argc, char *argv[]){

	// to compute fps ------
	//clock_t  start, end;
	//double fpsRun;
	//int counter = 0;
	//double sec;
	int xpic = 512;
	int ypic = 512;
	vpVideoReader reader;
	vpImage<vpRGBa> I_col (ypic ,xpic);
	vpImage<unsigned char> I_uchar (ypic ,xpic);
	reader.setFileName(argv[1]);//"D:\\Projets\\APASH\\data\\videos\\sequence_floor0_undist\\seq_4\\image.%04d.pgm");
	reader.setFirstFrameIndex(1);
	try	{reader.open(I_col);} catch(...) {return -1; }

	vpImageConvert::convert(I_col,I_uchar);
	CvSize frame_size;	frame_size.height = I_uchar.getHeight();	frame_size.width =  I_uchar.getWidth();
	vpDisplayX display (I_uchar, frame_size.height ,frame_size.width ,"__1");

	IplImage *frame_1c_orig = NULL;
	allocateOnDemand( &frame_1c_orig, frame_size, IPL_DEPTH_8U, 1 );
	unsigned int nbFrames = reader.getLastFrameIndex();

	vpImage<vpRGBa> Ioverlay;

	SegmentMotion segMot; 
	reader.acquire(I_col);
	vpImageConvert::convert(I_col,I_uchar);
	vpImageConvert::convert(I_uchar, frame_1c_orig) ;
	segMot.init(600);
	segMot.initTrajectories(frame_1c_orig);
	segMot.initEnergy(I_uchar);
	int k=0;
	unsigned int current_frame = 1;
    vpImage<vpRGBa> Iseg;
	while(current_frame < nbFrames)
	{

		try {reader.acquire(I_col);
		vpImageConvert::convert(I_col,I_uchar);}
		catch(...)
		{
			cout << "End of sequence"<< endl;
			system("pause");
			exit(1);
		}
		vpImageConvert::convert(I_uchar, frame_1c_orig) ;
		vpDisplay::display(I_uchar) ;


		//add new features at each 25 frames

		if ((current_frame % 25) ==  1)
		{
			SegmentMotion segTemp;
			segTemp.initTrajectories(frame_1c_orig);
			segMot.fusionTrajectories(segTemp);
			cout << segMot.tracker.getNbFeatures()<< endl;
		}
		segMot.tracker.track(frame_1c_orig);
		segMot.updateTrajectories();
		segMot.displayTrajectories(I_uchar);
		//int labelMotion[100];
		//vpImagePoint points[100][2];
		if (k>0)
		segMot.labelSelectPoints(I_uchar, I_col);

		vpDisplay::flush(I_uchar);
		vpDisplay::getImage(I_uchar,Ioverlay);
		std::string opath = vpIoTools::path("itr%06d.png");
        char buf4[FILENAME_MAX];
        sprintf(buf4, opath.c_str(), k);
        std::string filename4(buf4);
        //std::cout << "Write: " << filename4 << std::endl;
            vpImageIo::write(Ioverlay, filename4);
		 if(k>10)
		{
		/*segMot.GMM();
		segMot.computeLikelihoodGMM(I_uchar, I_col);*/
			 if(k == 11)
	    segMot.computeLikelihood(I_uchar, I_col);
			 else segMot.computeLikelihoodHist(I_uchar, I_col);

		segMot.computeSpatialEnergy(I_uchar, I_col);
		segMot.minimizeEnergy(I_uchar, I_col,Iseg);

		std::string opath1 = vpIoTools::path("isegy%06d.png");
        char buf5[FILENAME_MAX];
        sprintf(buf5, opath1.c_str(), k);
        std::string filename5(buf5);
        //std::cout << "Write: " << filename4 << std::endl;
            vpImageIo::write(Iseg, filename5);

		if(k == 11)
		segMot.computeColorLikelihoodHist(I_col);
		}

		vpDisplay::flush(I_uchar);
		vpDisplay::getClick(I_uchar);


		k++;

	}



	return 1;
}
















/*int main (int argc, char *argv[])
{
	uchar *ImageY_filtred =	NULL;
	uchar *ImageY_filtred_downSampled =	NULL;
	uchar *ImageU_filtred =	NULL;
	uchar *ImageV_filtred = NULL;
	
	int xpic = 352	;
	int ypic = 256;

	int ratio = 1 << LEVEL;
	vpVideoReader reader;
	vpImage<vpRGBa> I_1 (ypic ,xpic) ;	
	vpImage<unsigned char> I_1_uchar (ypic ,xpic);
	vpImage<vpRGBa> I_1_downSample_f1 (ypic/ratio ,xpic/ratio ) ;	
	vpImage<vpRGBa> I_2_downSample_f2 (ypic/ratio ,xpic/ratio ) ;	
	vpImage<vpRGBa> I_2 (ypic,xpic);
	vpImage<vpRGBa> I_SegMap (ypic,xpic);
	vpImage<vpRGBa> I_hi (ypic,xpic);
	//120426-111514
	//120323-111800
	//D:\\Projets\\APASH\\data\\videos\\sequence_floor1\\120323-111800.avi
	//"D:\\Projets\\APASH\\data\\ViSP-images\\mire-2\\image.%04d.pgm"
	
	reader.setFileName("D:\\Projets\\APASH\\data\\videos\\sequence_floor0_undist\\seq_5\\image%4d.pgm");
	reader.setFirstFrameIndex(2);
	try	{reader.open(I_1_uchar);} catch(...) {return -1; }
	//vpDisplayGDI display (I_1, ypic,xpic,"SegmentationMap");
	vpDisplayGDI display2 (I_1_uchar, ypic/ratio ,xpic/ratio ,"__1");
	//vpDisplayGDI display3 (I_2_downSample_f2, ypic/ratio ,xpic/ratio ,"__2");

	// to compute fps ------
	clock_t  start, end;
	double fpsRun;
	int counter = 0;
	double sec;

	CvSize frame_size;	frame_size.height = I_1.getHeight();	frame_size.width =  I_1.getWidth();
	CvSize frame_size_down;	
	frame_size_down.height = I_1.getHeight()/ (ratio);	
	frame_size_down.width =  I_1.getWidth() / (ratio);
	unsigned int nbFrames = reader.getLastFrameIndex();

	int FullSize = frame_size.height * frame_size.width;

	IplImage *frame = NULL;
	IplImage *frame_3c_orig = NULL;
	IplImage *frame_1c_orig = NULL;
	IplImage *frame_2c_orig = NULL;


	IplImage *frame1_1C_subsample = NULL;
	IplImage *frame2_1C_subsample = NULL;
	IplImage *eig_image = NULL;
	IplImage *temp_image = NULL;
	IplImage *pyramid1 = NULL;
	IplImage *pyramid2 = NULL;	
	IplImage *ImageDownSample= NULL;	
		
	Mat matOutput_1(frame_size_down,IPL_DEPTH_8U);
	Mat matOutput_2(frame_size_down,IPL_DEPTH_8U);
	Mat matOutput_3(frame_size_down,IPL_DEPTH_8U);
	
	//cvCreateMat(frame_size.height, frame_size.height,IPL_DEPTH_32F);
	
	IplImage *velx = NULL;
	IplImage *vely = NULL;
	allocateOnDemand( &velx, frame_size_down, IPL_DEPTH_32F, 1 );
	allocateOnDemand( &vely, frame_size_down, IPL_DEPTH_32F, 1 );

	
	allocateOnDemand( &frame_3c_orig, frame_size, IPL_DEPTH_8U, 3 );
	allocateOnDemand( &frame_1c_orig, frame_size, IPL_DEPTH_8U, 1 );
	allocateOnDemand( &frame_2c_orig, frame_size, IPL_DEPTH_8U, 1 );

	allocateOnDemand( &frame1_1C_subsample, frame_size_down, IPL_DEPTH_8U, 1 );
	allocateOnDemand( &frame2_1C_subsample, frame_size_down, IPL_DEPTH_8U, 1 );
	allocateOnDemand( &pyramid1,			frame_size_down, IPL_DEPTH_8U, 1 );
	allocateOnDemand( &pyramid2,			frame_size_down, IPL_DEPTH_8U, 1 );
	allocateOnDemand( &eig_image,			frame_size_down, IPL_DEPTH_32F, 1 );
	allocateOnDemand( &temp_image,			frame_size_down, IPL_DEPTH_32F, 1 );
	IplImage *I_1_cv = cvCreateImage( frame_size_down, 8, 1 );

	unsigned int current_frame = 1;
	int number_of_features = NBFEATURESMAX ;
	CvPoint2D32f frame1_features[NBFEATURESMAX];
	
	//vpMatrix featMatrix ( 2 * HISTORYOFPOSITION, NBPOINTSTOTRACK);
	//vpColVector featVect (featMatrix);
	//int nbFeatureIndex[NBPOINTSTOTRACK];

	CvPoint2D32f frame2_features[NBFEATURESMAX];
	CvPoint2D32f trajectory [NBPOINTSTOTRACK][1000];
	int nb_trajectory [NBPOINTSTOTRACK];
	int ID_trajectory [NBPOINTSTOTRACK];
	int nbPointToTrack  = 0;
	int labelMotion [NBPOINTSTOTRACK];
	

	char optical_flow_found_feature[NBFEATURESMAX];
	float optical_flow_feature_error[NBFEATURESMAX];

	 
	vpKltOpencv tracker;
	SegmentMotion segMot; 
		
	tracker.setTrackerId(1);
	  //tracker.setOnMeasureFeature(&modifyFeature);
	tracker.setMaxFeatures(NBPOINTSTOTRACK);
	tracker.setWindowSize(10);
	tracker.setQuality(0.01);
	tracker.setMinDistance(15);
	tracker.setHarrisFreeParameter(0.04);
	tracker.setBlockSize(9);
	tracker.setUseHarris(1);
	tracker.setPyramidLevels(3);

	start = clock ();
	while(current_frame < nbFrames)
	{
		if ( current_frame == 1)
		{
			reader.acquire(I_1_uchar);
			vpImageConvert::convert(I_1_uchar, frame_1c_orig) ;
			segMot.initTrajectories(frame_1c_orig);
		}
		else 
		{
			reader.acquire(I_1_uchar);
			vpImageConvert::convert(I_1_uchar, frame_1c_orig) ;
		}
	  
		vpDisplay::display(I_1_uchar) ;

		if ((current_frame % 25) ==  1)
		{
			SegmentMotion segTemp;
			segTemp.initTrajectories(frame_1c_orig);
			segMot.fusionTrajectorie(segTemp);
		}

		tracker.track(frame_1c_orig);
		//tracker.display(I_1_uchar, vpColor::red);
		//vpDisplay::flush(I_1_uchar) ;

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


		//RANSACFunction (trajectory, nb_trajectory, nbPointToTrack, current_frame, labelMotion);

		//int min_size = 9999;
		//for (int i = 0; i < nbPointToTrack; i ++)
		//	if (nb_trajectory[i] < min_size)
		//		min_size = nb_trajectory[i];

		//min_size = MIN(min_size, HISTORYOFPOSITION);

		//vpColVector vect ();
		//vpColVector model();
		//
		//


		//for (int i = 0; i < nbPointToTrack; i++)
		//{
		//	int _j = 0;
		//	for (int  j = min_size - 1; j >=  0; j++)
		//	{
		//		vect[i][2*_j]		= trajectory[i][j].x;
		//		vect[i][2*_j + 1]	= trajectory[i][j].y;
		//		_j++;
		//	}
		//}


		//vpRansac::ransac(nbPointToTrack, 

		//printf("lost %d\n ", nb_lost);
		//for(int i = 0; i < tracker.getNbFeatures(); i++)
		//{
		//	float x1,y1;
		//	int id1, 
		//	tracker_tmp.getFeature(i, id1, x1, y1);
		//	int idx = getIdxFromID
		//	int size = nb_trajectory [
		//	trajectory[
		//}



#ifdef	WITH_SCALABILITY
		Mat mat_1 = cvarrToMat(frame_1c_orig, true, true, 0);
		Mat mat_2 = cvarrToMat(frame_2c_orig, true, true, 0);
		CvSize tempSize; 
		int l = 1;
		while (l <= LEVEL)
		{
			tempSize.height = frame_1c_orig->height/ (1 << l);
			tempSize.width  = frame_1c_orig->width / (1 << l);

			pyrDown(mat_1, matOutput_1, tempSize);
			pyrDown(mat_2, matOutput_2, tempSize);
			mat_1 = matOutput_1;
			mat_2 = matOutput_2;
			l++;
		}
		*frame2_1C_subsample = matOutput_2;
		*frame1_1C_subsample = matOutput_1;
		// faut s�parer les data de chaque buffer pour pouvoir calculer les vanishing points
#else 
		//frame1_1C_subsample = frame_1c_orig;
		//frame2_1C_subsample = frame_2c_orig;
#endif

		//vpImageConvert::convert(frame1_1C_subsample, I_1_downSample_f1);
		//vpDisplay::display(I_1_downSample_f1);

#ifdef WITH_MOTION
		if (current_frame == 1)
		{
			cvGoodFeaturesToTrack(frame1_1C_subsample, eig_image, temp_image, frame1_features, 
				&number_of_features, .01, 15, NULL, 9, 1, 0.04);
			for (int i = 0; i < NBPOINTSTOTRACK; i++)
			{
				nb_trajectory[i] = 0;
				trajectory[i][nb_trajectory[i]].x = frame1_features[i].x;
				trajectory[i][nb_trajectory[i]].y = frame1_features[i].y;
				nb_trajectory[i]++;
			  }
		}
		else 
		{
			int nb_temp_feat = NBFEATURESMAX ;
			CvPoint2D32f temp_feat[NBFEATURESMAX];
			cvGoodFeaturesToTrack(frame1_1C_subsample, eig_image, temp_image, temp_feat,
				&nb_temp_feat, .5, .5, NULL);
			FusionPreviousFeatures(frame1_features, frame2_features, &number_of_features, optical_flow_found_feature,
				temp_feat, nb_temp_feat);
		}

		CvSize optical_flow_window = cvSize(3,3);
		CvTermCriteria optical_flow_termination_criteria = cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20,0.03);

		cvCalcOpticalFlowPyrLK(frame1_1C_subsample, frame2_1C_subsample, pyramid1, pyramid2, frame1_features, frame2_features, number_of_features, 
			optical_flow_window, 3, optical_flow_found_feature, optical_flow_feature_error,optical_flow_termination_criteria, 0 );

		if (current_frame > HISTORYOFPOSITION)
			RANSACFunction (trajectory, nb_trajectory, current_frame, labelMotion);
	
#endif

#ifdef WITH_OPTICAL_FLOW

		cvCalcOpticalFlowLK(frame1_1C_subsample, frame2_1C_subsample, cvSize(3,3), velx, vely);
		cvConvertScaleAbs(velx,frame1_1C_subsample,64,0);
		cvConvertScaleAbs(vely,frame2_1C_subsample,64,0);
		vpImageConvert::convert(frame1_1C_subsample, I_1_downSample_f1);
		vpImageConvert::convert(frame2_1C_subsample, I_2_downSample_f2);

#endif

#ifdef WITH_VANISHINGPOINT
		//Observation: la detection des points de fuites est quasi impossible dans des r�solutions r�duites
		I_1_cv = frame_1c_orig;
		CvPoint  vanishingPoint1;
		CvPoint  vanishingPoint2;
		ComputeVanishingPoint (I_1_cv,  &vanishingPoint1, &vanishingPoint2);
#endif

#ifdef WITH_DISPLAY	
		vpDisplay::display(I_1_uchar);
		//vpDisplay::flush(I_1_uchar) ;
		//vpDisplay::display(I_2_downSample_f2);
		//vpDisplay::flush(I_2_downSample_f2) ;
#endif


//#ifdef WITH_MOTION

		//for (int i = 0; i < NBPOINTSTOTRACK; i++)
		//	{
		//		if (optical_flow_found_feature[i] == 1)
		//		{
		//			if (nb_trajectory[i] > 0)
		//			{
		//				int x1 = frame1_features[i].x;
		//				int y1 = frame1_features[i].y;
		//				int x2 = trajectory[i][nb_trajectory[i]-1].x;
		//				int y2 = trajectory[i][nb_trajectory[i]-1].y; 
		//				double hypotenuse;	
		//				hypotenuse = sqrt( square(x1 - x2) + square(y1 - y2) );
		//				
		//				if (hypotenuse  < MOTION_FILTER)
		//				{
		//					trajectory[i][nb_trajectory[i]].x = frame1_features[i].x;
		//					trajectory[i][nb_trajectory[i]].y = frame1_features[i].y;
		//					
		//					nb_trajectory[i]++;
		//				}
		//				else 
		//					nb_trajectory[i] = 0;
		//			}
		//			else 
		//			{
		//				trajectory[i][nb_trajectory[i]].x = frame1_features[i].x;
		//				trajectory[i][nb_trajectory[i]].y = frame1_features[i].y;
		//				nb_trajectory[i]++;
		//			}
		//		}
		//		else
		//			nb_trajectory[i] = 0;
		//	}


#ifdef WITH_DISPLAY	
	
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
			
				double hypotenuse;	
				hypotenuse = sqrt( square(x1 - x2) + square(y1 - y2) );
				//featMatrix[nbFeatureIndex[i]][i]
				switch (labelMotion[j])
				{
					case 0 :vpDisplay::displayLine(I_1_uchar,p2,p1,vpColor::red, 1); break;
					case 1	:vpDisplay::displayLine(I_1_uchar,p2,p1,vpColor::red, 1); break;
					default :vpDisplay::displayLine(I_1_uchar,p2,p1,vpColor::red, 1);break;
				}
				
			}
			vpImagePoint ip (trajectory[j][nb_trajectory[j]-1].y, trajectory[j][nb_trajectory[j]-1].x);
			switch (labelMotion[j])
			{
				case 0 :vpDisplay::displayCross(I_1_uchar, ip, 5, vpColor::red);   break;
				case 1	:vpDisplay::displayCross(I_1_uchar, ip, 5, vpColor::red); break;
				default :vpDisplay::displayCross(I_1_uchar, ip, 5, vpColor::red);  break;
			}
		}
		//vpImage<vpRGBa> i_temp;
		//display2.getImage( i_temp);
		//vpImageIo::writePNG(i_temp, "test.png");
		vpDisplay::flush(I_1_uchar) ;
#endif
//#endif


#ifdef WITH_VANISHINGPOINT
	#ifdef WITH_DISPLAY	
		vpImagePoint center1 (vanishingPoint1.x / ratio, vanishingPoint1.y / ratio);	
		vpImagePoint center2 (vanishingPoint2.x / ratio, vanishingPoint2.y / ratio);	
		vpDisplay::displayCircle(I_1_downSample_f1, center1, 20/ratio, vpColor::green, 0, 2);
		vpDisplay::displayCircle(I_1_downSample_f1, center2, 10/ratio, vpColor::blue, 0, 2);
		vpDisplay::flush(I_1_downSample_f1) ;
	#endif	
#endif

		end = clock();
		++counter;       
	    sec = (double)(end - start) / CLOCKS_PER_SEC ; 
		fpsRun = (double)counter / sec;
	    printf("%d FPS = %.2f\n ",current_frame, fpsRun );
		//printf("number_of_features = %d \n", number_of_features);
		current_frame++;
	}
}

#endif
/*
Calcul des lignes droites dans une image
#define HoughLines_ 1
#ifdef HoughLines 
		IplImage *I_1_cv;
		vpImageConvert::convert(I_1, I_1_cv);
		Mat src (I_1_cv,true);
		Mat dst(288, 352, CV_8UC1);
		Canny(src , dst, 50, 200, 3);
		vector<Vec4i> lines;
		HoughLinesP(dst, lines, 1, CV_PI/180, 50, 5, 6 );
		for( size_t i = 0; i < lines.size(); i++ )
		{
			Vec4i l = lines[i];
			Point p1(l[0], l[1]);
			Point p2(l[2], l[3]);
			vpImagePoint p11(p1.x,p1.y);
			vpImagePoint p22(p2.x,p2.y);

			vpDisplay::displayLine( I_1, p11, p22, vpColor::green, 1);
		}
*/		
#endif



/*
Motion vector display
for ( int i = 0; i < number_of_features; i++)
		{
			if (optical_flow_found_feature[i])
			{
				int x1 = frame1_features[i].x;
				int y1 = frame1_features[i].y;
				int x2 = frame2_features[i].x;
				int y2 = frame2_features[i].y;
				
				vpImagePoint p2 (y1, x1);
				vpImagePoint p1 (y2, x2);
				double hypotenuse;	
				hypotenuse = sqrt( square(x1 - x2) + square(y1 - y2) );
				if (hypotenuse > 0)
				{
					//vpDisplay::displayArrow(I_1,p2,p1,vpColor::red,2,2,1);	
					vpDisplay::displayLine(I_1,p2,p1,vpColor::red, 1);	
					//vpDisplay::getImage(I_1, I_1);
				}
				else 
					vpDisplay::displayPoint(I_1, p2, vpColor::red);
				
			}
			
		}
*/
