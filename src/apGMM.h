/*
 * GrabCut implementation source code 
 * by Justin Talbot, jtalbot@stanford.edu
 * Placed in the Public Domain, 2010
 * 
 */

#ifndef apGMM_H
#define apGMM_H

/*#include "Color.h"
#include "Image.h"*/

#include <visp/vpConfig.h>
#include <visp/vpMath.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include "vibe-background.h"
#include <stdio.h>
#include <stdlib.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <time.h>
#include <visp/vpVideoReader.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpFeatureVanishingPoint.h>
#include <visp/vpRansac.h>
#include <visp/vpKltOpencv.h>
#include <visp/vpColVector.h>
#include <visp/vpMeLine.h>

#include "cv.h"

struct Gaussian
{
	vpRGBa mu;					// mean of the gaussian
	double covariance[3][3];		// covariance matrix of the gaussian
	double determinant;			// determinant of the covariance matrix
	double inverse[3][3];			// inverse of the covariance matrix
	double pi;					// weighting of this gaussian in the GMM.

	// These are only needed during Orchard and Bouman clustering.
	double eigenvalues[3];		// eigenvalues of covariance matrix
	double eigenvectors[3][3];	// eigenvectors of   "          "
};

class apGMM
{
public:

	// Initialize GMM with number of gaussians desired.
	apGMM(unsigned int K);
	~apGMM();

	unsigned int K() const { return m_K; }

	// Returns the probability density of color c in this GMM
	double p(vpRGBa c);

	// Returns the probability density of color c in just Gaussian k
	double p(unsigned int i, vpRGBa c);

public:

	unsigned int m_K;		// number of gaussians
	Gaussian* m_gaussians;	// an array of K gaussians

	void buildGMMs(apGMM& backgroundGMM, apGMM& foregroundGMM, std::vector<vpColVector> &colBgd, std::vector<vpColVector> &colFgd, std::vector<int> &componentBack, std::vector<int> &componentFore);
	void learnGMMs(apGMM& backgroundGMM, apGMM& foregroundGMM, int *components, const vpImage<vpRGBa>& image, int *label);
};
/*
// Build the initial GMMs using the Orchard and Bouman color clustering algorithm
void buildGMMs(apGMM& backgroundGMM, apGMM& foregroundGMM, int components, const vpImage<vpRGBa>& image, const Image<SegmentationValue>& hardSegmentation);

// Iteratively learn GMMs using GrabCut updating algorithm
void learnGMMs(apGMM& backgroundGMM, apGMM& foregroundGMM, int components, const vpImage<vpRGBa>& image, const Image<SegmentationValue>& hardSegmentation);*/


// Helper class that fits a single Gaussian to color samples
class GaussianFitter
{
public:
	GaussianFitter();
	
	// Add a color sample
	void add(vpRGBa c);
	
	// Build the gaussian out of all the added color samples
	void finalize(Gaussian& g, unsigned int totalCount, bool computeEigens = false) const;
	
private:

	//vpRGBa s;			// sum of r,g, and b
	double s[3];
	vpMatrix  p;		// matrix of products (i.e. r*r, r*g, r*b), some values are duplicated.

	unsigned int count;	// count of color samples added to the gaussian
};

#endif //GMM_H
