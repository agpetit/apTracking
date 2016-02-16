/*
 * apSegmentation.h
 *
 *  Created on: Nov 15, 2012
 *      Author: agpetit
 */

#ifndef APSEGMENTATION_H_
#define APSEGMENTATION_H_

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

#include "apLibMotion.h"
#include "apSegMotionCol.h"


#include <math.h>
#include <cv.h>

using namespace std;
using namespace cv;


class VISP_EXPORT apSegmentation
{
public :
	typedef enum
	{
	VIBE,
	OPENCV_GMM_ZIVKOVIC,
	OPENCV_GMM_BAF,
	CRIMINISI,
	PETIT
	}segmentationType;

	segmentationType type;
	apSegmentationParameters segParam;

    vibeModel_t *model;// = libvibeModelNew();
	int32_t width;
	int32_t height;
	int32_t stride;

	apSegMotionCol segMot;
	IplImage *frame_1c_orig;

	vpCameraParameters cam;

	bool okseg;

	vpImage <vpRGBa> Iclust;
	vpImage <unsigned char> I_uchar;

	vpDisplayX display;


public:
	apSegmentation();
	virtual ~apSegmentation();
	void init(vpImage<vpRGBa> &_I);
	void clear();
	void setSegmentationParameters(apSegmentationParameters &_segParam){segParam = _segParam;}
	void segmentFgdBgd(vpImage<vpRGBa> &_I, vpImage<vpRGBa> &I_, int frame);
	void clean();




};
//#endif
#endif /* APSEGMENTATION_H_ */
