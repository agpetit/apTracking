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
#include "p_helper.h"

#include <time.h>

using namespace cv;
using namespace cv::ml;
using namespace std;

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
void test_itL( const Size & size, int i, vector< Rect > & locations_ );

void get_svm_detector(const Ptr<SVM>& svm, vector< float > & hog_detector )
{
    // get the support vectors
    Mat sv = svm->getSupportVectors();
    const int sv_total = sv.rows;
    // get the decision function
    Mat alpha, svidx;
    double rho = svm->getDecisionFunction(0, alpha, svidx);

    CV_Assert( alpha.total() == 1 && svidx.total() == 1 && sv_total == 1 );
    CV_Assert( (alpha.type() == CV_64F && alpha.at<double>(0) == 1.) ||
               (alpha.type() == CV_32F && alpha.at<float>(0) == 1.f) );
    CV_Assert( sv.type() == CV_32F );
    hog_detector.clear();

    hog_detector.resize(sv.cols + 1);
    memcpy(&hog_detector[0], sv.ptr(), sv.cols*sizeof(hog_detector[0]));
    hog_detector[sv.cols] = (float)-rho;
}


/*
* Convert training/testing set to be used by OpenCV Machine Learning algorithms.
* TrainData is a matrix of size (#samples x max(#cols,#rows) per samples), in 32FC1.
* Transposition of samples are made if needed.
*/
void convert_to_ml(const std::vector< cv::Mat > & train_samples, cv::Mat& trainData )
{
    //--Convert data
    const int rows = (int)train_samples.size();
    const int cols = (int)std::max( train_samples[0].cols, train_samples[0].rows );
    cv::Mat tmp(1, cols, CV_32FC1); //< used for transposition if needed
    trainData = cv::Mat(rows, cols, CV_32FC1 );
    vector< Mat >::const_iterator itr = train_samples.begin();
    vector< Mat >::const_iterator end = train_samples.end();
    for( int i = 0 ; itr != end ; ++itr, ++i )
    {
        CV_Assert( itr->cols == 1 ||
            itr->rows == 1 );
        if( itr->cols == 1 )
        {
            transpose( *(itr), tmp );
            tmp.copyTo( trainData.row( i ) );
        }
        else if( itr->rows == 1 )
        {
            itr->copyTo( trainData.row( i ) );
        }
    }
}

void load_images( const string & prefix, const string & filename, vector< Mat > & img_lst )
{
    string line;
    ifstream file;

    file.open( (prefix+filename).c_str() );
    if( !file.is_open() )
    {
        cerr << "Unable to open the list of images from " << filename << " filename." << endl;
        exit( -1 );
    }

    bool end_of_parsing = false;
    while( !end_of_parsing )
    {
        getline( file, line );
        if( line.empty() ) // no more file to read
        {
            end_of_parsing = true;
            break;
        }
        Mat img = imread( (prefix+line).c_str() ); // load the image
        if( img.empty() ) // invalid image, just skip it.
            continue;
#ifdef _DEBUG
        imshow( "image", img );
        waitKey( 10 );
#endif
        img_lst.push_back( img.clone() );
    }
}

void load_imagesL( const string & prefix, const string & filename, vector< Mat > & img_lst, int i )
{
    string line;
    ifstream file;

    string imn="%03d_test.png";

    file.open( (prefix+filename).c_str() );
    if( !file.is_open() )
    {
        cerr << "Unable to open the list of images from " << filename << " filename." << endl;
        exit( -1 );
    }

    bool end_of_parsing = false;
    while( !end_of_parsing )
    {
        getline( file, line );
        if( line.empty() ) // no more file to read
        {
            end_of_parsing = true;
            break;
        }

            char buf4[FILENAME_MAX];
            sprintf(buf4, (prefix+line + imn).c_str(), i);
            std::string filename4(buf4);

            std::cout << "filename 4 " << filename4 << std::endl;

        cv::Mat img = cv::imread(filename4); // load the image

        if( img.empty() ) // invalid image, just skip it.
            continue;

#ifdef _DEBUG
        imshow( "image", img );
        waitKey( 10 );
#endif
        img_lst.push_back( img.clone() );


    }

}

void sample_neg( const vector< Mat > & full_neg_lst, vector< Mat > & neg_lst, const Size & size )
{
    Rect box;
    box.width = size.width;
    box.height = size.height;

    const int size_x = box.width;
    const int size_y = box.height;

    srand( (unsigned int)time( NULL ) );

    vector< Mat >::const_iterator img = full_neg_lst.begin();
    vector< Mat >::const_iterator end = full_neg_lst.end();
    for( ; img != end ; ++img )
    {
        box.x = rand() % (img->cols - size_x);
        box.y = rand() % (img->rows - size_y);
        Mat roi = (*img)(box);
        neg_lst.push_back( roi.clone() );
#ifdef _DEBUG
        imshow( "img", roi.clone() );
        waitKey( 10 );
#endif
    }
}

// From http://www.juergenwiki.de/work/wiki/doku.php?id=public:hog_descriptor_computation_and_visualization
Mat get_hogdescriptor_visu(const Mat& color_origImg, vector<float>& descriptorValues, const Size & size )
{
    const int DIMX = size.width;
    const int DIMY = size.height;
    float zoomFac = 3;
    Mat visu;
    resize(color_origImg, visu, Size( (int)(color_origImg.cols*zoomFac), (int)(color_origImg.rows*zoomFac) ) );

    int cellSize        = 8;
    int gradientBinSize = 9;
    float radRangeForOneBin = (float)(CV_PI/(float)gradientBinSize); // dividing 180 into 9 bins, how large (in rad) is one bin?

    // prepare data structure: 9 orientation / gradient strenghts for each cell
    int cells_in_x_dir = DIMX / cellSize;
    int cells_in_y_dir = DIMY / cellSize;
    float*** gradientStrengths = new float**[cells_in_y_dir];
    int** cellUpdateCounter   = new int*[cells_in_y_dir];
    for (int y=0; y<cells_in_y_dir; y++)
    {
        gradientStrengths[y] = new float*[cells_in_x_dir];
        cellUpdateCounter[y] = new int[cells_in_x_dir];
        for (int x=0; x<cells_in_x_dir; x++)
        {
            gradientStrengths[y][x] = new float[gradientBinSize];
            cellUpdateCounter[y][x] = 0;

            for (int bin=0; bin<gradientBinSize; bin++)
                gradientStrengths[y][x][bin] = 0.0;
        }
    }

    // nr of blocks = nr of cells - 1
    // since there is a new block on each cell (overlapping blocks!) but the last one
    int blocks_in_x_dir = cells_in_x_dir - 1;
    int blocks_in_y_dir = cells_in_y_dir - 1;

    // compute gradient strengths per cell
    int descriptorDataIdx = 0;
    int cellx = 0;
    int celly = 0;

    for (int blockx=0; blockx<blocks_in_x_dir; blockx++)
    {
        for (int blocky=0; blocky<blocks_in_y_dir; blocky++)
        {
            // 4 cells per block ...
            for (int cellNr=0; cellNr<4; cellNr++)
            {
                // compute corresponding cell nr
                cellx = blockx;
                celly = blocky;
                if (cellNr==1) celly++;
                if (cellNr==2) cellx++;
                if (cellNr==3)
                {
                    cellx++;
                    celly++;
                }

                for (int bin=0; bin<gradientBinSize; bin++)
                {
                    float gradientStrength = descriptorValues[ descriptorDataIdx ];
                    descriptorDataIdx++;

                    gradientStrengths[celly][cellx][bin] += gradientStrength;

                } // for (all bins)


                // note: overlapping blocks lead to multiple updates of this sum!
                // we therefore keep track how often a cell was updated,
                // to compute average gradient strengths
                cellUpdateCounter[celly][cellx]++;

            } // for (all cells)


        } // for (all block x pos)
    } // for (all block y pos)


    // compute average gradient strengths
    for (celly=0; celly<cells_in_y_dir; celly++)
    {
        for (cellx=0; cellx<cells_in_x_dir; cellx++)
        {

            float NrUpdatesForThisCell = (float)cellUpdateCounter[celly][cellx];

            // compute average gradient strenghts for each gradient bin direction
            for (int bin=0; bin<gradientBinSize; bin++)
            {
                gradientStrengths[celly][cellx][bin] /= NrUpdatesForThisCell;
            }
        }
    }

    // draw cells
    for (celly=0; celly<cells_in_y_dir; celly++)
    {
        for (cellx=0; cellx<cells_in_x_dir; cellx++)
        {
            int drawX = cellx * cellSize;
            int drawY = celly * cellSize;

            int mx = drawX + cellSize/2;
            int my = drawY + cellSize/2;

            rectangle(visu, Point((int)(drawX*zoomFac), (int)(drawY*zoomFac)), Point((int)((drawX+cellSize)*zoomFac), (int)((drawY+cellSize)*zoomFac)), Scalar(100,100,100), 1);

            // draw in each cell all 9 gradient strengths
            for (int bin=0; bin<gradientBinSize; bin++)
            {
                float currentGradStrength = gradientStrengths[celly][cellx][bin];

                // no line to draw?
                if (currentGradStrength==0)
                    continue;

                float currRad = bin * radRangeForOneBin + radRangeForOneBin/2;

                float dirVecX = cos( currRad );
                float dirVecY = sin( currRad );
                float maxVecLen = (float)(cellSize/2.f);
                float scale = 2.5; // just a visualization scale, to see the lines better

                // compute line coordinates
                float x1 = mx - dirVecX * currentGradStrength * maxVecLen * scale;
                float y1 = my - dirVecY * currentGradStrength * maxVecLen * scale;
                float x2 = mx + dirVecX * currentGradStrength * maxVecLen * scale;
                float y2 = my + dirVecY * currentGradStrength * maxVecLen * scale;

                // draw gradient visualization
                cv::line(visu, Point((int)(x1*zoomFac),(int)(y1*zoomFac)), Point((int)(x2*zoomFac),(int)(y2*zoomFac)), Scalar(0,255,0), 1);

            } // for (all bins)

        } // for (cellx)
    } // for (celly)


    // don't forget to free memory allocated by helper data structures!
    for (int y=0; y<cells_in_y_dir; y++)
    {
        for (int x=0; x<cells_in_x_dir; x++)
        {
            delete[] gradientStrengths[y][x];
        }
        delete[] gradientStrengths[y];
        delete[] cellUpdateCounter[y];
    }
    delete[] gradientStrengths;
    delete[] cellUpdateCounter;

    return visu;

} // get_hogdescriptor_visu

void compute_hog( const vector< Mat > & img_lst, vector< Mat > & gradient_lst, const Size & size )
{
    HOGDescriptor hog;
    hog.winSize = size;
    Mat gray;
    vector< Point > location;
    vector< float > descriptors;

    vector< Mat >::const_iterator img = img_lst.begin();
    vector< Mat >::const_iterator end = img_lst.end();
    for( ; img != end ; ++img )
    {
        cvtColor( *img, gray, COLOR_BGR2GRAY );
        hog.compute( gray, descriptors, Size( 16, 16 ), Size( 0, 0 ), location );
        gradient_lst.push_back( Mat( descriptors ).clone() );
#ifdef _DEBUG
        imshow( "gradient", get_hogdescriptor_visu( img->clone(), descriptors, size ) );
        waitKey( 10 );
        imwrite("hog.png", get_hogdescriptor_visu( img->clone(), descriptors, size ) );
#endif
    }
}

void compute_one_hog( const Mat & img, Mat & gradient, const Size & size )
{
    HOGDescriptor hog;
    hog.winSize = size;
    Mat gray;
    vector< Point > location;
    vector< float > descriptors;

    {
        cvtColor( img, gray, COLOR_BGR2GRAY );
        hog.compute( gray, descriptors, Size( 16, 16 ), Size( 0, 0 ), location );
        gradient = Mat( descriptors ).clone();
#ifdef _DEBUG
        imshow( "gradient", get_hogdescriptor_visu( img.clone(), descriptors, size ) );
        waitKey( 10 );
        imwrite("hogV1.png", get_hogdescriptor_visu( img.clone(), descriptors, size ) );
#endif
    }
}

void train_svm( const vector< Mat > & gradient_lst, const vector< int > & labels )
{

    Mat train_data;
    convert_to_ml( gradient_lst, train_data );

    clog << "Start training...";
    Ptr<SVM> svm = SVM::create();
    /* Default values to train SVM */
    svm->setCoef0(0.0);
    svm->setDegree(3);
    svm->setTermCriteria(TermCriteria( CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, 1e-3 ));
    svm->setGamma(0);
    svm->setKernel(SVM::LINEAR);
    svm->setNu(0.5);
    svm->setP(0.1); // for EPSILON_SVR, epsilon in loss function?
    svm->setC(0.01); // From paper, soft classifier
    svm->setType(SVM::EPS_SVR); // C_SVC; // EPSILON_SVR; // may be also NU_SVR; // do regression task
    svm->train(train_data, ROW_SAMPLE, Mat(labels));
    clog << "...[done]" << endl;

    svm->save( "my_people_detector.yml" );

    delete svm;
}

void train_svmL( const vector< Mat > & gradient_lst, const vector< int > & labels, int i )
{

    Mat train_data;
    convert_to_ml( gradient_lst, train_data );

    string imn="%03d.yml";
            char buf4[FILENAME_MAX];
            sprintf(buf4, ("my_liver_detector" + imn).c_str(), i);
            std::string filename4(buf4);

    clog << "Start training...";
    Ptr<SVM> svm = SVM::create();
    /* Default values to train SVM */
    svm->setCoef0(0.0);
    svm->setDegree(3);
    svm->setTermCriteria(TermCriteria( CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, 1e-3 ));
    svm->setGamma(0);
    svm->setKernel(SVM::LINEAR);
    svm->setNu(0.5);
    svm->setP(0.1); // for EPSILON_SVR, epsilon in loss function?
    svm->setC(0.01); // From paper, soft classifier
    svm->setType(SVM::EPS_SVR); // C_SVC; // EPSILON_SVR; // may be also NU_SVR; // do regression task
    svm->train(train_data, ROW_SAMPLE, Mat(labels));
    clog << "...[done]" << endl;

    svm->save( filename4 );
}

void draw_locations( Mat & img, const vector< Rect > & locations, const Scalar & color )
{
    if( !locations.empty() )
    {
        vector< Rect >::const_iterator loc = locations.begin();
        vector< Rect >::const_iterator end = locations.end();
        for( ; loc != end ; ++loc )
        {
            rectangle( img, *loc, color, 2 );
        }
    }
}

void test_itL( cv::Mat &img, const Size & size, int i , vector< Rect > & locations, vector< double > &weights)
{
    char key = 27;
    Scalar reference( 0, 255, 0 );
    Scalar trained( 0, 0, 255 );
    Mat draw;
    Ptr<SVM> svm;
    HOGDescriptor hog;
    HOGDescriptor my_hog;
    my_hog.winSize = size;
    VideoCapture video;

    string imn="%03d.yml";
            char buf4[FILENAME_MAX];
            sprintf(buf4, ("my_liver_detector" + imn).c_str(), i);
            std::string filename4(buf4);
    // Load the trained SVM.
    svm = StatModel::load<SVM>( filename4 );
    // Set the trained svm to my_hog
    vector< float > hog_detector;
    get_svm_detector( svm, hog_detector );
    my_hog.setSVMDetector( hog_detector );


    // Set the people detector.
    //hog.setSVMDetector( hog.getDefaultPeopleDetector() );
    // Open the camera.
    /*video.open(0);
    if( !video.isOpened() )
    {
        cerr << "Unable to open the device 0" << endl;
        exit( -1 );
    }*/

    bool end_of_process = false;
    //while( !end_of_process )
    {
        //video >> img;
        //img = imread("/home/antoine/soft/seeing3Dchairs/lung01.png");
        /*if( img.empty() )
            break;*/
        /*Mat gradient;
        compute_one_hog( img1, gradient, Size( 160, 96 ) );*/

        draw = img.clone();
        locations.clear();
        weights.clear();
        //hog.detectMultiScale( img, locations );
        //draw_locations( draw, locations, reference );

        //locations.clear();
        int t0 = cv::getTickCount();
        my_hog.detectMultiScale( img, locations, weights, 0.3, Size(2,2), Size(16,16), 1.1, true);

        int t1 = cv::getTickCount();
        double secs = (t1-t0)/cv::getTickFrequency();

        //std::cout << secs << " " << std::endl;

        for (int kk = 0; kk <weights.size(); kk++)
        std::cout << weights[kk] << " " << std::endl;

        draw_locations( draw, locations, trained );

        cv::imwrite("draw930.png", draw);

        imshow( "Video", draw );
        key = (char)waitKey( 1 );
        if( 27 == key )
            end_of_process = true;
    }
}


void test_it( const Size & size )
{
    char key = 27;
    Scalar reference( 0, 255, 0 );
    Scalar trained( 0, 0, 255 );
    Mat img, draw;
    Ptr<SVM> svm;
    HOGDescriptor hog;
    HOGDescriptor my_hog;
    my_hog.winSize = size;
    VideoCapture video;
    vector< Rect > locations;
    vector<double> foundWeights;

    // Load the trained SVM.
    svm = StatModel::load<SVM>( "my_people_detector.yml" );
    // Set the trained svm to my_hog
    vector< float > hog_detector;
    get_svm_detector( svm, hog_detector );
    my_hog.setSVMDetector( hog_detector );
    // Set the people detector.
    //hog.setSVMDetector( hog.getDefaultPeopleDetector() );
    // Open the camera.
    video.open(0);
    if( !video.isOpened() )
    {
        cerr << "Unable to open the device 0" << endl;
        exit( -1 );
    }

    bool end_of_process = false;
    while( !end_of_process )
    {
        //video >> img;
        img = imread("/home/antoine/soft/seeing3Dchairs/lung10.png");
        if( img.empty() )
            break;

        draw = img.clone();

        //locations.clear();
        //hog.detectMultiScale( img, locations, foundWeights );
        //draw_locations( draw, locations, reference );

        locations.clear();
        my_hog.detectMultiScale( img, locations);
        draw_locations( draw, locations, trained );

        imshow( "Video", draw );
        key = (char)waitKey( 10 );
        if( 27 == key )
            end_of_process = true;
    }
}

vpHomogeneousMatrix getTemplatePose(double scale, int i, int j, vpCameraParameters &cam, vpHomogeneousMatrix &cMo)
{
//double angle = orientation-orieff-prcl.get_angle();
vpRotationMatrix R;
vpTranslationVector tr;
vpRotationMatrix Rz = vpRotationMatrix(vpRxyzVector(0,0,0));

vpHomogeneousMatrix cMoTrans;
cMo.extract(R);
cMo.extract(tr);
double xx,yy;
tr[2] = tr[2]*scale;
xx = tr[0];
yy = tr[1];
tr[0] = xx+tr[2]*(j-cam.get_u0())/cam.get_px();
tr[1] = yy+tr[2]*(i-cam.get_v0())/cam.get_py();

cMoTrans.buildFrom(tr,R);
return cMoTrans;
}

int main( int argc, char** argv )
{

    std::cout << " ok " << std::endl;
    cv::CommandLineParser parser(argc, argv, "{help h|| show help message}"
            "{pd||pos_dir}{p||pos.lst}{nd||neg_dir}{n||neg.lst}");
    if (parser.has("help"))
    {
        parser.printMessage();
        exit(0);
    }
    vector< vector< Mat > > pos_lst;
    vector< vector< Mat > > full_neg_lst;
    vector< vector< Mat > > neg_lst;
    vector< vector< Mat > > gradient_lst;
    vector< vector< int > > labels;
    string pos_dir = parser.get<string>("pd");
    string pos = parser.get<string>("p");
    string neg_dir = parser.get<string>("nd");
    string neg = parser.get<string>("n");    if( pos_dir.empty() || pos.empty() || neg_dir.empty() || neg.empty() )
    {
        cout << "Wrong number of parameters." << endl
            << "Usage: " << argv[0] << " --pd=pos_dir -p=pos.lst --nd=neg_dir -n=neg.lst" << endl
            << "example: " << argv[0] << " --pd=/INRIA_dataset/ -p=Train/pos.lst --nd=/INRIA_dataset/ -n=Train/neg.lst" << endl;
        exit( -1 );
    }

    int nexamplars = 480;
    pos_lst.resize(nexamplars);
    neg_lst.resize(nexamplars);
    full_neg_lst.resize(nexamplars);
    gradient_lst.resize(nexamplars);
    labels.resize(nexamplars);

    std::ifstream file;

    std::string path = "/home/antoine/soft/Liver/Train/posLiver/poses.txt";

    file.open(path.c_str(), std::ifstream::in);
    double dvalue0,dvalue1,dvalue2,dvalue3,dvalue4,dvalue5,dvalue6,dvalue7,dvalue8,dvalue9,dvalue10,dvalue11;

    //Mat img = imread("/home/antoine/soft/Lung/Test/lung934.png");
    //Mat img = imread("/home/antoine/Documents/SlicerCapture/image_00000_testS.png");
    //Mat img = imread("test_liver.png");
    //Mat img = imread("/home/antoine/soft/Liver/Test/liver1.png");

    Mat img = imread("/home/antoine/Documents/SlicerCapture2/initliver2.png");
    //Mat img = imread("022_test.png");

    vpCameraParameters camparam;
    camparam.initPersProjWithoutDistortion(540,540,img.rows/2,img.cols/2);

    int widthT = 160;
    int heightT = 96;

    for (int i = 0; i < nexamplars; i++){
    load_imagesL( pos_dir, pos, pos_lst[i], i );
    labels[i].assign( pos_lst[i].size(), +1);
    std::cout << " ok assign "  << std::endl;
    const unsigned int old = (unsigned int)labels[i].size();
    load_imagesL( neg_dir, neg, full_neg_lst[i], 0);
    sample_neg( full_neg_lst[i], neg_lst[i], Size( widthT, heightT ) );
    labels[i].insert( labels[i].end(), neg_lst[i].size(), -1 );
    CV_Assert( old < labels[i].size() );

    compute_hog( pos_lst[i], gradient_lst[i], Size( widthT, heightT ) );
    std::cout << " ok compute hog 1 " << std::endl;
    compute_hog( neg_lst[i], gradient_lst[i], Size( widthT, heightT ) );
    std::cout << " ok compute hog 2 " << std::endl;
    train_svmL( gradient_lst[i], labels[i], i );
    }


    vector< Rect > locations;
    vector< double > weights;

    std::string line;

    for (int i = 0; i < nexamplars; i++)
    {

    test_itL( img, Size( widthT, heightT ),i, locations, weights);
    std::cout << " ok detect " << i << std::endl;

        std::string filename4;
        getline(file, line);

        std::cout << line << std::endl;

        file >> filename4;
        file >> dvalue0;
        file >> dvalue1;
        file >> dvalue2;
        file >> dvalue3;
        file >> dvalue4;
        file >> dvalue5;
        file >> dvalue6;
        file >> dvalue7;
        file >> dvalue8;
        file >> dvalue9;
        file >> dvalue10;
        file >> dvalue11;


        vpTranslationVector tr;
        vpQuaternionVector qt;
        vpRotationMatrix Rot;

        tr[0] = dvalue0;
        tr[1] = dvalue1;
        tr[2] = dvalue2;

        Rot[0][0] = dvalue3;
        Rot[0][1] = dvalue4;
        Rot[0][2] = dvalue5;
        Rot[1][0] = dvalue6;
        Rot[1][1] = dvalue7;
        Rot[1][2] = dvalue8;
        Rot[2][0] = dvalue9;
        Rot[2][1] = dvalue10;
        Rot[2][2] = dvalue11;
        //std::cout << " values " << filename4 << " " << dvalue1 << " " << dvalue2 << " " << dvalue3 << " " << dvalue4 << " " << dvalue5 << " " << dvalue6 << std::endl;

        vpHomogeneousMatrix cMo, cMoTrans;
        cMo.buildFrom(tr,Rot);

        int pos_i, pos_j;
        double scale;

        if (weights.size() > 0)
        {

        pos_i = locations[0].y + (locations[0].height)/2;
        pos_j = locations[0].x + (locations[0].width)/2;

        scale = (double)locations[0].width/widthT;

        //if (scale > 1.02)
        //std::cout << " scale " << scale << " pos_i " << pos_i << " pos_j " << pos_j << std::endl;

        cMoTrans = getTemplatePose(scale, pos_i, pos_j , camparam, cMo);

      std::cout << " cMo " << cMo << " cMoTrans " << cMoTrans << std::endl;

        }

    }


    //test_it( Size( 96, 160 ) ); // change with your parameters

    return 0;
}
