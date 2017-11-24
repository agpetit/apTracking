/*
 * apDetector.h
 *
 *  Created on: Mar 2, 2012
 *      Author: agpetit
 */

#ifndef APDETECTOR_H_
#define APDETECTOR_H_


#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpBasicFeature.h>
#include <visp/vpImage.h>
#include <visp/vpFeatureLine.h>
#include <visp/vpList.h>
#include <visp/vpVideoReader.h>
#include "apViews.h"
#include "apPFilter.h"
#include "ctParticle.h"
#include "apMbTracker.h"
#include "apSegmentation.h"


class apDetector
{

public:
typedef enum
{
TOP,
TOPDOWN,
} SearchStrategy;

typedef enum
{
BEST,
MEAN,
} PFEstimateType;

protected:
vpMatrix hierarchy;
vpMatrix dataViews0;
vpMatrix dataViews1;
std::vector< std::vector<vpImage<unsigned char>*>* > Hviews;
std::vector< apPFilter* > filters;
apMbTracker tracker;
vpHomogeneousMatrix cMo;
ctParticle estimateIMM;
vpMatrix filtTransProb;
vpMatrix filtProb;
//std::vector <double> normFact;
vpMatrix normFact;
int startingLevel;
int fr;
apDetection detection;
vpMatrix likelihoods;
vpMatrix probasfilt;
std::string object;


apSegmentation segment;

public:
	apDetector();
	virtual ~apDetector();
	void init(apDetection &_detect, std::string _object);
	void setSegmentationParameters(apSegmentationParameters &_seg){segment.segParam = _seg;}
	void setStartingLevel();
	void loadViews(std::string viewspath);
	void setFilters(std::string hpath, vpCameraParameters &cam);
    //void setTracker(apMbTracker &_tracker){tracker = _tracker;}
	void detect(std::string ipath, std::string isegpath, std::string hpath, std::string scpath, vpCameraParameters &cam, int fframe, double thld, const SearchStrategy &searchStrategy, const PFEstimateType &PFEType);
	void computeTransProba(vpMatrix &simmatrix);
	void computeInteractionMixing(int nbParticles);
	void computeCRF(int nbParticles, double &ProbaMax, int &filtMax, ctParticle &pComb,int k);
	ctParticle computeModeProb(int nbParticles);
	void computeInteractionMixingBest(int nbParticles);
	void computeModeProbBest(int nbParticles, double &ProbaMax, int &filtMax, ctParticle &pComb);
	void computeTransitionProba(const char *filenameTP);
	void computeTransitionProbV(const char *filenameTP);
	vpHomogeneousMatrix getPose(){return cMo;}
	int getFrame(){return fr;}

};

#endif /* APDETECTOR_H_ */
