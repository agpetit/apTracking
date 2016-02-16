/*
 * apControlPoint.h
 *
 *  Created on: March 10, 2011
 *      Author: Antoine Petit
 */


#ifndef DOXYGEN_SHOULD_SKIP_THIS

/*!
 \file apControlPoint.h
 \brief Make the complete tracking of an object by using its complete CAD model.
*/

#ifndef apControlPoint_HH
#define apControlPoint_HH

#include <visp/vpConfig.h>

#include <visp/vpPoint.h>
#include <visp/vpPlane.h>
#include <visp/vpLine.h>
#include <visp/vpMe.h>
#include "apControlPointTracker.h"
#include <visp/vpFeatureLine.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpImagePoint.h>
#include <visp/vpCameraParameters.h>


class VISP_EXPORT apControlPoint : public apControlPointTracker
{
  private:
    //vpPointSite PExt[2] ;

    double rho, theta, theta_1;
    double delta ,delta_1;
    int sign;
    //double a,b,c;
    vpMe *me;
    vpCameraParameters *cam;
    vpFeatureLine featureline ;
    vpFeatureLine featureline1;
    vpFeaturePoint featurepoint;
    vpLine line;
    double profile1[11];

  
  public: 
    //int imin, imax;
    //int jmin, jmax;
    //double expecteddensity;
    // the image point
    int nbFeature;

    vpImagePoint *icpoint;

    //vpPointSite s;

    // the 3D point
    vpPoint *cpoint;
    vpPoint cpointo;

    //! The moving edge container associated to the control point
    //vpMbtMeLine *meline;
    //! The 3D line associated to the control point
    //vpLine *line;

    //! Normale to surface where the control point lies
    vpColVector norm;
    vpColVector normw;

    //! Gradient vector associated to the control point
    vpColVector grad;

    //! Gradient profile associated to the control Points
    //vpColVector profile;

    double error;
    vpColVector errorD;

    vpColVector *profile;

    vpColVector L;
    vpColVector L1;
    vpColVector L2;
    vpMatrix LD;

    double xs,ys,nxs,nys,Zs,xs_old,ys_old,nxs_old,nys_old;

    bool isSilhouette;
    bool invnormal;

  
  public:  
    apControlPoint();
    ~apControlPoint();
    void buildPlane(vpPoint &pointn, vpColVector &normal,vpPlane &plane);
    vpFeatureLine getFeatureLine();
    vpLine getLine(){return line;}
    void buildFeatureLine();
    void buildPoint();
    double getTheta(){return theta;};
    //vpPoint getPoint(){return *cpointo;};
    void setMovingEdge(vpMe *_me);
    void setCameraParameters(vpCameraParameters *_cam);
    void initControlPoint(const vpImage<unsigned char>& I, double cvlt);
    void setProfile(const vpImage<unsigned char>& Iprec);
    void detectSilhouette(const vpImage<unsigned char>& I, const vpImage<unsigned char>& I0, double cvlt);
    void update(const vpHomogeneousMatrix &_cMo);
    void updateSilhouettePoint(const vpHomogeneousMatrix &_cMo);
    void track(const vpImage<unsigned char> &I, const vpImage<unsigned char> &Iprec);
    void trackPred(const vpImage<unsigned char>& I, const vpImage<unsigned char>& Iprec);
    void trackMH(const vpImage<unsigned char>& I, const vpImage<unsigned char>& Iprec);
    void buildPoint(int n, int m, double &Z, double orient, vpColVector &normo, vpHomogeneousMatrix &cMo);
    void buildSilhouettePoint(int n, int m, double &Z, double orient, vpColVector &normo, vpHomogeneousMatrix &cMo);
    void buildPLine(vpHomogeneousMatrix &oMc);
    //void updateParameters(const vpImage<unsigned char> &I, double rho, double theta);
    //void updateParameters(const vpImage<unsigned char> &I, vpImagePoint ip1, vpImagePoint ip2, double rho, double theta);
    //void display(const vpImage<unsigned char>& /*I*/, vpColor /*col*/) {;}
    
     /*!
     Get the a coefficient of the line corresponding to \f$ i \; cos(\theta) + j \; sin(\theta) - \rho = 0 \f$
   
     \return : The a coefficient of the moving edge  
    */
    //inline double get_a() const { return this->a;}
    
     /*!
     Get the a coefficient of the line corresponding to \f$ i \; cos(\theta) + j \; sin(\theta) - \rho = 0 \f$
   
     \return : The b coefficient of the moving edge  
    */
    //inline double get_b() const { return this->b;}
    
     /*!
     Get the a coefficient of the line corresponding to \f$ i \; cos(\theta) + j \; sin(\theta) - \rho = 0 \f$
   
     \return : The c coefficient of the moving edge  
    */
    //inline double get_c() const { return this->c;}

    inline double get_theta() const { return this->theta;}

    void display(const vpImage<unsigned char>&I, const vpColor col, const unsigned int thickness);

    void display(const vpImage<vpRGBa>&I, vpColor col, unsigned int thickness);

    void display(const vpImage<unsigned char>&I, vpColor col){};

    void initInteractionMatrixError();
    void initInteractionMatrixError30();
    void computeInteractionMatrixError(const vpHomogeneousMatrix &cMo, const vpImage<unsigned char> &I);
    void computeInteractionMatrixErrorMH(const vpHomogeneousMatrix &cMo,const vpImage<unsigned char> &I);
    void computeInteractionMatrixError2(const vpHomogeneousMatrix &cMo, const vpImage<unsigned char> &I);
    void computeInteractionMatrixError3(const vpHomogeneousMatrix &cMo,const vpImage<unsigned char>& I, const vpImage<double> &Igrad, const vpImage<double> &Igradx, const vpImage<double> &Igrady);
    void computeInteractionMatrixError30(const vpHomogeneousMatrix &cMo, const vpImage<unsigned char> &I, const vpImage<double> &Igrad, const vpImage<double> &Igradx, const vpImage<double> &Igrady);
    void computeInteractionMatrixError4(const vpHomogeneousMatrix &cMo, const vpImage<unsigned char> &I, const vpImage<double> &Igrad, const vpImage<double> &Igradx, const vpImage<double> &Igrady);
  
  private:
    void sample(const vpImage<unsigned char>& I){};
    //void reSample(const vpImage<unsigned char>&image);
    //void reSample(const vpImage<unsigned char>&image, vpImagePoint ip1, vpImagePoint ip2);
    //void updateDelta();
    //void bubbleSortI();
    //void bubbleSortJ();
    //void suppressPoints(const vpImage<unsigned char> &I);
    //void setExtremities();
    //void seekExtremities(const vpImage<unsigned char> &I);
    //void findSignal(const vpImage<unsigned char>& I, const vpMe *me, double *conv);


} ;

#endif
#endif

