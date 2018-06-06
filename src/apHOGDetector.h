/*
 * apHOGDetector.h
 *
 *  Created on: May 23, 2018
 *      Author: Antoine Petit
 */

#ifndef apHOGDetector_h
#define apHOGDetector_h

#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <visp/vpOpenCVGrabber.h>
#include <visp/vpVideoReader.h>
#include <cstdlib>
#include <visp/vpDebug.h>
#include <visp/vpConfig.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpPose.h>
#include <visp/vpPoint.h>
#include <visp/vpImagePoint.h>
#include <visp/vpDot2.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpImageConvert.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpVideoWriter.h>

#include <fstream>
#include <math.h>
#include <string.h>
#include <highgui.h>

#include <visp/vpException.h>
#include <visp/vpImageException.h>
#include <visp/vpRGBa.h>
#include <visp/vpIoTools.h>
#include <visp/vpImageIo.h>
#include <visp/vpParseArgv.h>
#include <visp/vpImage.h>
#include <visp/vpImageFilter.h>
#include <visp/vpImageConvert.h>
#include <visp/vpTime.h>
#include <visp/vpFeatureLuminance.h>
#include <visp/vpPlot.h>


#include <iostream>
#include <sstream>

#include "luaconfig.h"
#include <time.h>

using namespace cv;
using namespace cv::ml;
using namespace std;

class apHOGDetector
{
     public :

    apHOGDetector();
    virtual ~apHOGDetector();

void get_svm_detector(const Ptr<SVM>& svm, vector< float > & hog_detector );
void convert_to_ml(const std::vector< cv::Mat > & train_samples, cv::Mat& trainData );
void load_images( const string & prefix, const string & filename, vector< Mat > & img_lst);
void load_imagesL( const string & prefix, const string & filename, vector< Mat > & img_lst, int i );
void sample_neg( const vector< Mat > & full_neg_lst, vector< Mat > & neg_lst, const Size & size );
Mat get_hogdescriptor_visu(const Mat& color_origImg, vector<float>& descriptorValues, const Size & size );
void compute_hog( const vector< Mat > & img_lst, vector< Mat > & gradient_lst, const Size & size );
void compute_one_hog( const Mat & img, Mat & gradient_lst, const Size & size );
void train_svm( const vector< Mat > & gradient_lst, const vector< int > & labels );
void train_svmL( const vector< Mat > & gradient_lst, const vector< int > & labels, int i );
void draw_locations( Mat & img, const vector< Rect > & locations, const Scalar & color );
void test_it( const Size & size );
void test_itL( cv::Mat &img, const Size & size, int i , vector< Rect > & locations, vector< double > &weights);
vpHomogeneousMatrix getTemplatePose(double scale, int i, int j, vpCameraParameters &cam, vpHomogeneousMatrix &cMo);
void train(const string & pos_dir, const string & pos, const string & neg_dir, const string & neg);
void detect(vpImage<unsigned char> &_I, std::string &_path, vpHomogeneousMatrix &cMo_);


};


#endif
