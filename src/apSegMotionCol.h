
#ifndef APSEGMENTMOTION_H
#define APSEGMENTMOTION_H

#include <visp/vpKltOpencv.h>
#include "opencv2/imgproc.hpp"
#include <visp/vpHistogram.h>
#include "apRANSAC.h"
#include "apKernel.h"
#include "vpPointSite.h"
#include "apImageFilter.h"
#include <visp/vpLine.h>
#include "apGMM.h"
#include "apHoughVoteCircle.h"

//#include "apLibMotion.h"
#include "minE.h"
#include "ml.h"

#if defined(WIN32)
#include <visp/vpDisplayGDI.h>
typedef vpDisplayGDI vpDisplayX;
#endif

struct apEnergyParameters
{
  apEnergyParameters(): alpha_0(0.3), beta_0(1.0), gamma_0(0.8), alpha(0.3), beta(0.1), gamma(0.8)
  {
  }
  apEnergyParameters(double p1,
            double p2,
            double p3,
            double p4,
            double p5,
            double p6
            )
  {
	alpha_0 = p1;
	beta_0 = p2;
    gamma_0 = p3;
    alpha = p4;
    beta = p5;
    gamma = p6;
  }
  ~apEnergyParameters()
  {
  }

  double alpha_0;
  double beta_0;
  double gamma_0;
  double alpha;
  double beta;
  double gamma;
};

struct apKernelParameters
{
  apKernelParameters(): bwidth_col_0(255), bwidth_col_1(2000), bwidth_spat(0.3)
  {
  }
  apKernelParameters(double p1,
            double p2,
            double p3
            )
  {
	bwidth_col_0 = p1;
	bwidth_col_1 = p2;
    bwidth_spat = p3;
  }
  ~apKernelParameters()
  {
  }
  double bwidth_col_0;
  double bwidth_col_1;
  double bwidth_spat;
};


struct apKLTParameters
  {
    apKLTParameters(): nbPoints(600), history(25), window_size(6), quality(0.00000001), min_dist(10),
    		harris_free(0.1), block_size(9), use_Harris(1),level_pyr(1),grid_bg(4),grid_fg(24)
    {
    }

    apKLTParameters(int p1,
              int p2,
              int p3,
              double p4,
              int p5,
              double p6,
              int p7,
              int p8,
              int p9,
              int p10,
              int p11
              )
    {
  	nbPoints = p1;
  	history = p2;
    window_size = p3;
    quality = p4;
    min_dist = p5;
    harris_free = p6;
    block_size = p7;
    use_Harris = p8,
    level_pyr =  p9;
    grid_bg = p10;
    grid_fg = p11;
    }

  ~apKLTParameters()
  {
  }
  int nbPoints;
  int history;
  int window_size;
  double quality;
  int min_dist;
  double harris_free;
  int block_size;
  int use_Harris;
  int level_pyr;
  int grid_bg;
  int grid_fg;
};

struct apRANSACParameters
  {
    apRANSACParameters(): nmaxiterations(500), pointstobuildP(5), minimalsizemodel(100), backgroundthresh(0.3), consensus(0.2)
    {
    }

    apRANSACParameters(int p1,
              int p2,
              int p3,
              double p4,
              double p5
              )
    {
    nmaxiterations = p1;
    pointstobuildP = p2;
    minimalsizemodel = p3;
    backgroundthresh = p4;
    consensus = p5;
    }

  ~apRANSACParameters()
  {
  }
  int nmaxiterations;
  int pointstobuildP;
  int minimalsizemodel;
  double backgroundthresh;
  double consensus;
};

struct apSegmentationParameters
{
  apSegmentationParameters()
  {
  }
  apSegmentationParameters(apEnergyParameters &p1,
		  apKernelParameters &p2,
		  apKLTParameters &p3, apRANSACParameters &p4, int p5, int p6
            )
  {
	energyParams = p1;
	kernelParams = p2;
	KLTParams = p3;
	RANSACParams = p4;
	startFrame = p5;
	nGaussians = p6;
  }
  ~apSegmentationParameters()
  {
  }

  apEnergyParameters energyParams;
  apKernelParameters kernelParams;
  apKLTParameters KLTParams;
  apRANSACParameters RANSACParams;

  int startFrame;
  int deltaHomography;
  int nGaussians;
  int nbins;
  typedef enum
	{
	EARTH,
	DEEPSPACE,
	LIMB
	}backgroundType;
	backgroundType bType;
};

class apSegMotionCol {

public:
	vpKltOpencv tracker;
	int nbPointToTrack ;
	int *nb_trajectory ;
	int *ID_trajectory ;
	std::vector<std::vector<CvPoint2D32f> > trajectory_;
	//CvPoint2D32f **trajectory;

	double *EdataF;
	double *EdataB;
	double *EdataHF;
	double *EdataHB;
	double *EdataHistF;
	double *EdataHistB;
	double *EdataGMMF;
	double *EdataGMMB;
	double *EsmoothV;
	double *EsmoothH;
	int* label;
	int* labelPts;

	int* labelMotion0;
	int* labelMotion1;

    apEnergyParameters energyParameters;
    apKernelParameters kernelParameters;
    apKLTParameters KLTParameters;
    apRANSACParameters RANSACParameters;


	int nGaussians;
	int deltaHomography;
	int nbins;

	std::vector<int> componentFgd;
	std::vector<int> componentBgd;

	std::vector<int*> colFgd;
	std::vector<int*> colBgd;

	std::vector<vpImage<vpRGBa> > histIm;

    //cv::EM em_modelFgd;
    //cv::EM em_modelBgd;

	minE minEnergy;

	int bj,bi;

	int resolution;

	int nPoints;

	int nfg,nbg;

	double inverseDenominateur;

	std::vector<vpHistogram> histFg;
	std::vector<vpHistogram> histBg;

	std::vector<std::vector<double> > histFgN;
	std::vector<std::vector<double> > histBgN;

	std::vector<vpHistogram> histFgPts;
	std::vector<vpHistogram> histBgPts;

	apGMM *m_backgroundGMM;
	apGMM *m_foregroundGMM;

	vpLine limb,limbo1,limbo2;

	apHoughVoteCircle circleVote;
	std::vector<double> cU;
	std::vector<double> cV;
	std::vector<double> Rho;

	apRANSAC ransac;

	apSegMotionCol ();
	virtual ~apSegMotionCol();
	void init();
	void initTracker();
	void clear();
	void setKLTParameters(apKLTParameters &_klt){KLTParameters = _klt;}
	void setEnergyParameters(apEnergyParameters &_energy){energyParameters = _energy;}
	void setKernelParameters(apKernelParameters &_kernel){kernelParameters = _kernel;}
	void setRANSACParameters(apRANSACParameters &_ransac);
	void setSegmentationParameters(apSegmentationParameters &_seg);
	void setDeltaHomography(int _deltaH){deltaHomography = _deltaH;}
	void setNGaussians(int _ngaussians){nGaussians = _ngaussians;}
	void setNBins(int _nbins){nbins = _nbins;}
        void initTrajectories(cv::Mat Img);
	void initEnergy(vpImage<unsigned char> &_I);
	void updateTrajectories();
	void fusionTrajectories(apSegMotionCol tracker_tmp);
	void displayTrajectories(vpImage<unsigned char> &I);
	void labelPoints( vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol);
	vpRGBa meanTemplate(vpImage<vpRGBa> &Icol,int x,int y);
	void labelMeanPoints(vpImage<unsigned char> &I,vpImage<vpRGBa> &Icol);
	void labelSelectPoints(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol);
	//void computeLikelihoodHomography(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol);
	void computeDataEnergyKernelLimb(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol);
	void computeDataEnergyKernelHistLimb(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol);
	void computeDataEnergyHistInitDeepSpace(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol);
	void computeDataEnergyHistDeepSpace(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol);
	void computeDataEnergyKernel(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol);
	void computeDataEnergyHist(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol);
	void computeDataEnergyKernelHist(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol);
	void GMM();
	void buildGMMs();
	void learnGMMs(){;}
	void computeDataEnergyGMM(vpImage<unsigned char> &I,vpImage<vpRGBa> &Icol);
	void computeEnergy(vpImage<unsigned char> &I,vpImage<vpRGBa> &Icol);
	void computeSpatialEnergy(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol);
	void minimizeEnergy(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol);
	void computeColorHist(vpImage<vpRGBa> &_I);
	void computeColorHistLimb(vpImage<vpRGBa> &_I);
	void rgb2yuv(vpImage<vpRGBa> &_I,vpImage<vpRGBa> &I_);
	void computeEnergyHist(vpImage<unsigned char> &I, vpImage<vpRGBa> &Icol);
	void computeHistKLTPoints(vpImage<vpRGBa> &_I);
	void getIseg(vpImage<unsigned char> &_I, vpImage<unsigned char> &I_,vpImage<vpRGBa> &Icol_);
	void computeInverseDenominateur(int &i, int &j, vpColVector &p);
	void computeInverseDenominateur(vpColVector &X, vpColVector &p);
	void Warp(int i_y, int j_x, double &i_res, double &j_res, vpColVector &p);
	void getParameters(vpHomography &H, vpColVector &p);
	void setParameterSpace(int n, int m, vpImage<unsigned char> &I);
	void detectLimbC(vpImage<unsigned char> &I,vpImage<vpRGBa> &_I);
	void detectLimb(vpImage<unsigned char> &I,vpImage<vpRGBa> &_I);

};



#endif
