/*
 * apMbtDistanceLineMH.h
 *
 *  Created on: November 10, 2012
 *      Author: Antoine Petit
 */
#ifndef APMBTDISTANCELINEMH_HH
#define APMBTDISTANCELINEMH_HH

#include "apMHMeLine.h"
#include "apMeLine.h"
#include "apControlPoint.h"

#include <visp/vpPoint.h>
#include <visp/vpMbtMeLine.h>
#include <visp/vpLine.h>
#include <visp/vpPlane.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeatureLine.h>
//#include <visp/vpMbtHiddenFace.h>

//#include <visp/vpMbtDistanceLine.h>


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


class VISP_EXPORT apMbtDistanceLineMH
{
  
  public:
    std::string name;
	bool lost;
	vpCameraParameters *cam;
	vpMe *me;
	vpFeatureLine featureline;

  public:
    apMHMeLine* MHmeline;
    apMeLine* meline_;
    double wmean ;
    int index ;
    int idx;

  public:
    //! The moving edge container
    vpMbtMeLine *meline;
    //! The 3D line
    vpLine *line;
    //! The first extremity
    vpPoint *p1;
    //! The second extremity
    vpPoint *p2;
    //! The interaction matrix
    vpMatrix L;
    //! The error vector
    vpColVector error;

    vpColVector weight;
    //! The number of moving edges
    int nbFeature;
    //! Indicates if the line has to be reinitialized
    bool Reinit;
    //! Pointer to the list of faces
    //vpMbtHiddenFaces *hiddenface;
    //! Index of the faces which contain the line
    std::vector<int> Lindex_polygon;
    //! Indicates if the line is visible or not
    bool isvisible;

    std::vector<apControlPoint*> pointsvect;

  public:
    void init();
    apMbtDistanceLineMH();
    apMbtDistanceLineMH& operator =(const apMbtDistanceLineMH& l);
    ~apMbtDistanceLineMH();

    /*!
      Get the camera paramters.

      \param cam : The vpCameraParameters used to store the camera parameters.
     */
     inline void getCameraParameters(vpCameraParameters *cam) {cam = this->cam;}


     void setMovingEdgeMH(vpMe *_me);
     inline void setCameraParameters(vpCameraParameters *_cam){this->cam = _cam;}

     /*!
      Get the mean weight of the line. The mean weight is computed thanks to the weight of each moving edge.
      Those weights are computed by the robust estimation method used during the virtual visual servoing.

      \return The mean weight of the line.

     */
     inline double getMeanWeight() const {return wmean;}

     /*!
      Set the mean weight of the line.

      \param wmean : The mean weight of the line.
     */
     inline void setMeanWeight(const double wmean) {this->wmean = wmean;}

     /*!
       Set a boolean parameter to indicates if the line is visible in the image or not.

       \param _isvisible : Set to true if the line is visible
     */
     inline void setVisible(bool _isvisible) {isvisible = _isvisible ;}

     /*!
       Check if the line is visible in the image or not.

       \return Return true if the line is visible
     */
     inline bool isVisible() {return isvisible; }

     /*!
       Get the name of the line.

       \return Return the name of the line
     */
     inline std::string getName() const {return name;}

     /*!
       Set the name of the line.

       \param name : The name of the line.
     */
     inline void setName(const std::string name) {this->name = name;}

     /*!
       Set the name of the line.

       \param name : The name of the line.
     */
     inline void setName(const char* name) {this->name = name;}

     //void setMovingEdge(vpMe *Me);
     void buildPlane(vpPoint &P, vpPoint &Q, vpPoint &R, vpPlane &plane);
     void buildLine(vpPoint &P1, vpPoint &P2, vpPoint &P3, vpPoint &P4, vpLine &L);
     void buildFrom(vpPoint &_p1, vpPoint &_p2);

	void setLost(bool _Lost) {lost = _Lost ;}
	bool getLost() const{return lost ;}

    void initMovingEdgeMHP(const vpImage<unsigned char> &I, const vpImage<unsigned char> &gradMap, const vpHomogeneousMatrix &cMo);
    void trackMovingEdgeMHP(const vpImage<unsigned char> &I, const vpImage<unsigned char> &gradMap, const vpHomogeneousMatrix &cMo);

    void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam, const vpColor col, const unsigned int thickness = 1);
    void display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam, const vpColor col, const unsigned int thickness = 1);
    void displayMovingEdges(const vpImage<unsigned char> &I);

    bool closeToImageBorder(const vpImage<unsigned char>& I, const unsigned int threshold);

    void initInteractionMatrixErrorMH();
    void computeInteractionMatrixErrorMH(const vpImage<unsigned char> &I,const vpHomogeneousMatrix &cMo);

    /*int getNumberOfCandidateLines() {if(MHmeline == NULL)
    							return 0;
    							 else
    							return MHmeline->getNbOfCandidateLines();}*/
    //int rndWghtDraw(double rand);
    void buildDraw(apMbtDistanceLineMH &distLine, int idx);
    //apMbtDistanceLineMH* bestWghtDraw(const vpImage<unsigned char> &I);

    void MaxLikeDraw(apMbtDistanceLineMH &distLine);

    void setIndex(int i) {index = i;}
    int getIndex() const{return index ;}

    //SLAM - Line landmarks

    vpHomogeneousMatrix c0Mo;
    vpPoint P10,P20;
    vpColVector xEstL,xPredL, xMesL, HxPredL;
    vpMatrix PEstL,PPredL, PInnovL;
    vpMatrix K;
    vpMatrix Q;
    vpMatrix R;
    vpMatrix H;


    void initEKF(vpPoint &P1, vpPoint &P2, const vpHomogeneousMatrix &cMo);
    void filterEKF(std::vector<Vec4i> &lines, vpImage<vpRGBa> &Inormd, const vpHomogeneousMatrix &cMo);
    void updateLine();

  private:
    void project(const vpHomogeneousMatrix &cMo);
    //void setFace( vpMbtHiddenFaces *_hiddenface) { hiddenface = _hiddenface ; }
    void belongToPolygon(int index) { Lindex_polygon.push_back(index) ; }



} ;

#endif
//#endif

