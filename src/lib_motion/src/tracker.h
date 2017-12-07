
#ifndef SEGMENTMOTION_H
#define SEGMENTMOTION_H


#include <visp/vpKltOpencv.h>
#include "lib_motion.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <visp/vpHistogram.h>
#include "RANSAC.h"
#include "apKernel.h"

#include "minE.h"
#include "ml.h"

class SegmentMotion {

public:
	vpKltOpencv tracker;
	int nbPointToTrack ;
	int *nb_trajectory ;
	int *ID_trajectory ;
	std::vector<std::vector<CvPoint2D32f> > trajectory_;
	CvPoint2D32f **trajectory;

	double *EdataF;
	double *EdataB;
	double *EdataHistF;
	double *EdataHistB;
	double *EsmoothV;
	double *EsmoothH;
	int* label;

	std::vector<vpColVector> colFgd;
	std::vector<vpColVector> colBgd;

    CvEM em_modelFgd;
    CvEM em_modelBgd;

	minE minEnergy;

	int bj,bi;

	int resolution;

	int nfg,nbg;

	std::vector<vpHistogram> histFg;
	std::vector<vpHistogram> histBg;

	SegmentMotion ();
	SegmentMotion (int nbPoint);

	void init(int nbPoints);
	void initTrajectories(IplImage *Img);
	void initEnergy(vpImage<unsigned char> &_I);
	void updateTrajectories();
	void fusionTrajectories(SegmentMotion tracker_tmp);
	void displayTrajectories(vpImage<unsigned char> &I);
	void labelPoints( vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol);
	vpRGBa meanTemplate(vpImage<vpRGBa> &Icol,int x,int y);
	void labelMeanPoints(vpImage<unsigned char> &I,vpImage<vpRGBa> &Icol);
	void labelSelectPoints(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol);
	void GMM();
	void computeLikelihoodGMM(vpImage<unsigned char> &I,vpImage<vpRGBa> &Icol);
	void computeLikelihood(vpImage<unsigned char> &I,vpImage<vpRGBa> &Icol);
	void computeSpatialEnergy(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol);
	void minimizeEnergy(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol,vpImage<vpRGBa> &Iseg);
	void computeColorLikelihoodHist(vpImage<vpRGBa> &_I);
	void RGB2YUV(vpImage<vpRGBa> &_I,vpImage<vpRGBa> &I_);
	void computeLikelihoodHist(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol);



};



#endif
