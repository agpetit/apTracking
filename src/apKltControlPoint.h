/*
 * apKltControlPoint.h
 *
 *  Created on: Apr 24, 2013
 *      Author: agpetit
 */

#ifndef APKLTCONTROLPOINT_H_
#define APKLTCONTROLPOINT_H_

#include <visp/vpConfig.h>
#include <visp/vpMath.h>
#include <visp/vpPoint.h>
#include <visp/vpPlane.h>
#include <visp/vpLine.h>
#include <visp/vpMe.h>
#include <visp/vpFeatureLine.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpImagePoint.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpGEMM.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpHomography.h>

#include <ostream>
#include <cmath>    // std::fabs
#include <limits>   // numeric_limits

class apKltControlPoint
{
public:


    //vpMe me;
    vpCameraParameters cam;

    vpImagePoint icpoint;
    vpImagePoint icpoint_curr;
    vpImagePoint icpoint0;
    vpImagePoint icpointi;
    vpImagePoint icpoint_old;

    //vpPointSite s;

    // the 3D point
    vpPoint cpoint;
    vpPoint cpointo;

    //! The moving edge container associated to the control point
    //vpMbtMeLine *meline;
    //! The 3D line associated to the control point
    //vpLine *line;

    //! Normale to surface where the control point lies
    vpColVector norm;
    vpColVector normw;

    //! Gradient profile associated to the control Points
    //vpColVector profile;

    vpColVector error;
    vpMatrix L;

    //! current camera to plane distance to speed up the computation
    double dt;
    //! distance between the plane and the camera at the initial position
    double d0;

    vpMatrix H;
    //! normal to the initial plane
    vpColVector N;
    //! current normal
    vpColVector N_cur;
    //! inverse of the distance between the plane and the camera at the initial position (speed up computation)
    double invd0;
    //! cRc0_0n (temporary variable to speed up the computation)
    vpColVector cRc0_0n;

    vpRotationMatrix R0;
    double Z0;
    bool valid ;


	apKltControlPoint();
	virtual ~apKltControlPoint();
    void setMovingEdge(vpMe _me);
    void setCameraParameters(vpCameraParameters _cam);
    void initPoint(int y, int x, double &Z, vpColVector &normo, vpHomogeneousMatrix &cMo);
    void buildPlane(vpPoint &pointn, vpColVector &normal,vpPlane &plane);
    void buildPoint(int y, int x, double &Z, vpColVector &normo, vpHomogeneousMatrix &cMo);
    void computeInteractionMatrixErrorH(const vpImage<unsigned char> &_I);
    double compute_1_over_Z(const double x, const double y);
    inline void computeP_mu_t(const double x_in, const double y_in, double& x_out, double& y_out, const vpMatrix& _cHc0);
    void computeHomography(const vpHomogeneousMatrix& _cTc0, vpHomography& _cHc0);
    void computeInteractionMatrixError(const vpHomogeneousMatrix &cMo, const vpImage<unsigned char> &I);

};

#endif /* APKLTCONTROLPOINT_H_ */
