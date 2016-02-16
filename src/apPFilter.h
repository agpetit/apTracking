/*
 * apPFilter.h
 *
 *  Created on: Mar 1, 2012
 *      Author: agpetit
 */

#ifndef APPFILTER_H_
#define APPFILTER_H_

#include "ctPFilter.h"
#include <visp/vpImage.h>
#include "ctParticle.h"
#include "apContourPoint.h"
#include <visp/vpPixelMeterConversion.h>
#include "apLogPolarHist.h"
#include "apDetection.h"

#ifdef VISP_HAVE_OPENCV
#  if VISP_HAVE_OPENCV_VERSION >= 0x020101
#    include <opencv2/core/core.hpp>
#    include <opencv2/imgproc/imgproc.hpp>
#    include <opencv2/imgproc/imgproc_c.h>
#  else
#    include <cv.h>
#  endif
#endif

using namespace cv;



class apPFilter : public ctPFilter
{
private :
	vpImage<unsigned char> ModelView;
	std::vector<double> scales;
	std::vector<vpImage<unsigned char>*> ModelViewSc;
	std::vector< apContourPoint*> ContourPoints;
	std::vector<std::vector<apContourPoint*>> ContourPointsSc;
	std::vector<std::vector<double>> logPolarHistV;
	vpImagePoint cog;
	int surface;
	double orientation;
	vpMatrix hrchy;
	vpMatrix data0;
	vpMatrix data1;
	int ind_min;
	vpPoseVector pose;
	vpPoseVector poseOpt_;
	std::vector< std::vector<vpImage<unsigned char>*>*> Hviews;
	vpCameraParameters cam;
	double probaFilt;
	std::vector<double> logPolarHist;

	int sample_x;
	int sample_y;
	int sample_theta;

	int nr_;
	int nw_;

	ctParticle prclOpt_;


public:
	apPFilter();
	virtual ~apPFilter();
	void setCameraParameters(vpCameraParameters &_cam){cam = _cam;}
	void setDetectionParameters(apDetection &_detect){detect = _detect;}
	void setModel(vpImage<unsigned char> &MView, vpImagePoint &cg, int surf, double ori, vpPoseVector &_pose);
	void setProba(double Proba){probaFilt = Proba;}
	void initSC();
	void computeSC(std::string filename, int nfilt);
	double getProba(){return probaFilt;}
	vpPoseVector getPose(){return pose;}
	void likelihoodViewOC(vpImage<vpRGBa> *dTOI, vpImage<vpRGBa> *dTOIseg, bool weights);
	void likelihoodViewOCFast0(vpImage<vpRGBa> *dTOI, vpImage<vpRGBa> *dTOIseg, bool weights,int niseg);
	void likelihoodViewOCFast(vpImage<vpRGBa> *dTOI, vpImage<vpRGBa> *dTOIseg, bool weights);
	double computeSimErrorCPV(ctParticle *prcl, vpImage<vpRGBa> *I, bool weights);
	void likelihoodViewOCOpt(vpImage<vpRGBa> *dTOI, vpImage<vpRGBa> *dTOIseg, bool weights);
	void likelihoodViewSteger(vpImage<unsigned char> &Igrad, vpImage<vpRGBa> *dTOIseg, bool weights);
	void likelihoodViewSC(std::vector<double> &logPolarHistIseg, vpImagePoint &cogIseg, double angleIseg, bool weights);
	void reWeightP(ctParticle &prclEst);
	//void init(const unsigned int pNumber, const ctRect & location, const MotionModel &Model, std::vector< std::vector<vpImage<unsigned char>*>*> Views, vpMatrix &hierarchy, vpMatrix &dataTemp0, vpMatrix &dataTemp1);
	/*void likelihoodView(vpImage<vpRGBa> *dTOI, bool weights);
	void likelihoodViewInit(vpImage<vpRGBa> *dTOI, bool weights);
	void likelihoodViewInitCP(vpImage<vpRGBa> *dTOI, bool weights);
	void likelihoodViewInitSC(vpImage<unsigned char> &Iseg, bool weights);
	void likelihoodViewInitSC(std::vector<double> &logPolarHistIseg, vpImagePoint &cogIseg, bool weights);
	void likelihoodViewSegInit(vpImage<vpRGBa> *dTOI, vpImage<vpRGBa> *dTOIseg, bool weights);*/
	void likelihoodStruct(vpImage<vpRGBa> *dTOI, bool weights, int indFilt, std::string hpath);
	void likelihoodStruct2(vpImage<vpRGBa> *dTOI, bool weights, int indFilt, std::string hpath);
	void likelihoodStruct3(vpImage<vpRGBa> *dTOI, vpImage<vpRGBa> *dTOIseg, bool weights, int indFilt, int startingLevel);
	void likelihoodStruct4(vpImage<vpRGBa> *dTOI, bool weights, int indFilt, int startingLevel);
	void reWeight(vpImage<unsigned char> &Iseg);
	void weightsUpdate2(vpImage<unsigned char> &Iseg);//, const float & min_threshold, const float & lost_prcl_ratio);
	void resize(const vpImage<unsigned char>& _I, vpImage<unsigned char>&  Io, double scale);
	vpHomogeneousMatrix getTemplatePose(vpImage<unsigned char> &I);
	vpHomogeneousMatrix getBestPose();
	void displayTemp(vpImage<unsigned char> &I);
	void displayBestTemp(vpImage<unsigned char> &I);
	void setStructure(vpMatrix &hier, vpMatrix &dat0, vpMatrix &dat1, std::vector< std::vector<vpImage<unsigned char>*>*> &HV);

};

#endif /* APPFILTER_H_ */
