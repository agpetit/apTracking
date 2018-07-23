/*
 * apCCDTracker.cpp
 *
 *  Created on: Nov 23, 2012
 *      Author: agpetit
 */

#include "apCCDTracker.h"
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGDI.h>


apCCDTracker::apCCDTracker() {
	pointsCCD.resize(1);
    scaleLevel = 0;
	pointsCCD[0].resize(0);
	npointsCCD = 0;
}

apCCDTracker::~apCCDTracker() {
	// TODO Auto-generated destructor stub
#pragma omp parallel for
    for (int i = 0; i < pointsCCD.size(); ++i)
    {
        for (int k = 0; k < pointsCCD[i].size(); k++)
        {
            apControlPoint *p = pointsCCD[i][k];
            if (p != NULL)
                delete p;
            p = NULL;
        }
        pointsCCD[i].clear();
    }
    pointsCCD.resize(0);
}

void apCCDTracker::init(apCCDParameters &_ccdParameters, vpCameraParameters &_cam)
{
	ccdParameters = _ccdParameters;
	cam = _cam;
	Phi = cv::Mat::zeros(ccdParameters.phi_dim,1, CV_64F);
	Sigma_Phi = cv::Mat::zeros(ccdParameters.phi_dim,ccdParameters.phi_dim, CV_64F);
	sigmaF.resize(6,6);
	sigmaP.resize(6,6);
	delta_Phi = cv::Mat::zeros(ccdParameters.phi_dim,1, CV_64F);
    tol = 0.0;
    tol_old = 0.0;
    convergence = false;
    norm = 0.0;
	resolution = pointsCCD[scaleLevel].size();
    nv = cv::Mat::zeros(resolution, 2, CV_64F);
    mean_vic = cv::Mat::zeros(resolution, 6, CV_64F);
    cov_vic = cv::Mat::zeros(resolution, 18, CV_64F);

    nv_prev = cv::Mat::zeros(resolution, 2, CV_64F);
    mean_vic_prev = cv::Mat::zeros(resolution, 6, CV_64F);
    cov_vic_prev = cv::Mat::zeros(resolution, 18, CV_64F);
    vic_prev = cv::Mat::zeros(resolution, 20*floor(ccdParameters.h/ccdParameters.delta_h), CV_64F);

    nabla_E = cv::Mat::zeros(ccdParameters.phi_dim,1, CV_64F);
    hessian_E = cv::Mat::zeros(ccdParameters.phi_dim,ccdParameters.phi_dim, CV_64F);
    hessian_E_partial = cv::Mat::zeros(ccdParameters.phi_dim,ccdParameters.phi_dim, CV_64F);
    vic = cv::Mat::zeros(resolution, 20*floor(ccdParameters.h/ccdParameters.delta_h), CV_64F);
    p_old = cv::Mat::zeros(resolution, 4, CV_64F);
	int normal_points_number = floor(ccdParameters.h/ccdParameters.delta_h);
	nerror_ccd = 2*normal_points_number*3*resolution;
    //w_ccd.resize(nerror_ccd);
	w_ccd.resize(resolution);
    w_ccd=1;
}

void apCCDTracker::setImage(const vpImage<vpRGBa> &_I)
{
    vpImageConvert::convert(_I, image);
}

void apCCDTracker::setPrevImage(const vpImage<vpRGBa> &_I)
{
    vpImageConvert::convert(_I, image_prev);
}

void apCCDTracker::clearCCDTracker()
{
    vic.release();
    mean_vic.release();
    cov_vic.release();
    nv.release();

    vic_prev.release();
    mean_vic_prev.release();
    cov_vic_prev.release();
    nv_prev.release();

    Phi.release();
    Sigma_Phi.release();
    delta_Phi.release();
    p_old.release();
    nabla_E.release();
    hessian_E.release();
    hessian_E_partial.release();
    image.release();
    image_prev.release();

#pragma omp parallel for
    for (int i = 0; i < pointsCCD.size(); ++i)
    {
        for (int k = 0; k < pointsCCD[i].size(); k++)
        {
            apControlPoint *p = pointsCCD[i][k];
            if (p != NULL)
                delete p;
            p = NULL;
        }
        pointsCCD[i].clear();
    }
    pointsCCD.resize(1);
}

void
computeJTR(const vpMatrix& _interaction, const vpColVector& _error, vpMatrix& _JTR)
{
   if(_interaction.getRows() != _error.getRows() || _interaction.getCols() != 6 ){
     throw vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
               "Incorrect matrices size in computeJTR.");
   }

   _JTR.resize(6, 1);
   const unsigned int N = _interaction.getRows();

   for (unsigned int i = 0; i < 6; i += 1){
     double ssum = 0;
     for (unsigned int j = 0; j < N; j += 1){
       ssum += _interaction[j][i] * _error[j];
     }
     _JTR[i][0] = ssum;
   }
}

inline double logistic(double x)
{
  return 1.0/(1.0+exp(-x));
}

inline double probit(double x)
{
  return 0.5*(1+1/sqrt(2)*erf(x));
}

inline cv::Scalar random_color(CvRNG* rng)
{
  int color = cvRandInt(rng);
  return CV_RGB(color&255, (color>>8)&255, (color>>16)&255);
}

void apCCDTracker::updateCCDPoints(vpHomogeneousMatrix &cMo)
{
	tol = 0;
#pragma omp parallel
    {
        double local_tol = 0.0;
#pragma omp for nowait
        for(int k = 0; k < pointsCCD[scaleLevel].size(); k++)
        {
            apControlPoint* p = pointsCCD[scaleLevel][k];
            p->updateSilhouettePoint(cMo);
            local_tol += pow((p->xs - p_old.at<double>(k, 0))*p_old.at<double>(k, 2) +
                             (p->ys - p_old.at<double>(k, 1))*p_old.at<double>(k, 3), 2);
        }
#pragma omp critical
        tol += local_tol;
    }
	tol_old = tol;

    nv = cv::Mat::zeros(resolution, 2, CV_64F);
    mean_vic = cv::Mat::zeros(resolution, 6, CV_64F);
    cov_vic = cv::Mat::zeros(resolution, 18, CV_64F);

    nabla_E = cv::Mat::zeros(ccdParameters.phi_dim,1, CV_64F);
    hessian_E = cv::Mat::zeros(ccdParameters.phi_dim,ccdParameters.phi_dim, CV_64F);
    hessian_E_partial = cv::Mat::zeros(ccdParameters.phi_dim,ccdParameters.phi_dim, CV_64F);

}

void apCCDTracker::computeLocalStatistics()
{
  const cv::Mat_<cv::Vec3b>& img = (cv::Mat_<cv::Vec3b>&)image;

  const double sigma = ccdParameters.h/(ccdParameters.alpha*ccdParameters.gamma_3);
  // sigma_hat = gamma_3 * sigma

  //  double sigma_hat = max(h/sqrt(2*gamma_2), gamma_4);
  const double sigma_hat = ccdParameters.gamma_3*sigma + ccdParameters.gamma_4;
  // to save the normalized parameters of vic[i,8]
  // dimension: resolution x 2
  // the first column save the normalized coefficient outside the curve
  // the second column store the one inside the curve
  cv::Mat normalized_param = cv::Mat::zeros(resolution, 2, CV_64F);

  vic = cv::Mat::zeros(resolution, 20*floor(ccdParameters.h/ccdParameters.delta_h), CV_64F);

//  CvRNG rng;
//  cv::Scalar color = random_color(&rng);
  /*for(int i=0; i < ccdParameters.resolution;i++)
  {*/
//  int i=0;
//#pragma omp parallel for
  for(int kk = 0 ; kk < pointsCCD[scaleLevel].size(); kk ++)
  {
    // temporary points used to store those points in the
    // normal direction as well as negative normal direction
    cv::Point3d tmp1, tmp2;

    // store the distance from a point in normal(negative norml) direction
    // to the point on the curve
    cv::Point3d tmp_dis1, tmp_dis2;

    const int i = kk;
    apControlPoint *p = pointsCCD[scaleLevel][kk];
    double *nv_ptr = nv.ptr<double>(i);
    // normal vector (n_x, n_y)
    // tagent vector (nv.at<double>(i,1), -n_x)
    int invnorm =0;

    if(p->invnormal)
    {
    nv_ptr[0] = -p->nxs;
    nv_ptr[1] = -p->nys;
    }
    else
    {
     nv_ptr[0] = p->nxs;
     nv_ptr[1] = p->nys;
    }

    // dimension = 4
    // 0 - x value
    // 1 - y value
    // 2 - normal vector x
    // 3 - normal vector y
    cv::Mat p_old = cv::Mat::zeros(resolution, 4, CV_64F);
    double *p_old_ptr = p_old.ptr<double>(i);

    // old value of the control points
    p_old_ptr[0] = p->icpoint->get_u();
    p_old_ptr[1] = p->icpoint->get_v();

    // old normal vector of the control points
    p_old_ptr[2] =p->icpoint->get_u();
    p_old_ptr[3] = p->icpoint->get_v();

    int k = 0;
    double alpha = 0.5;
    double *vic_ptr = vic.ptr<double>(i);
    for (int j = ccdParameters.delta_h; j <= ccdParameters.h; j += ccdParameters.delta_h, k++)
    {
      ///////////////////////////////////////////////////////////////////////////////////////////
      // calculate in the direction +n: (n_x, n_y)
      /////////////////////////////////////////////////////////////////////////////////////////
      // x_{k,l}
      tmp1.x = round(p->icpoint->get_u() + j*nv_ptr[0]);
      // y_{k,l}
      tmp1.y = round(p->icpoint->get_v() + j*nv_ptr[1]);

      // distance between x_{k,l} and x_{k,0} in the normal direction
      // appoximately it is l*h, l = {1,2,3,.....}
      tmp_dis1.x = (tmp1.x-p->icpoint->get_u())*nv_ptr[0] + (tmp1.y-p->icpoint->get_v())*nv_ptr[1];

      // distance between y_{k,l} and y_{k,0} along the curve
      // it approximates 0
      tmp_dis1.y = (tmp1.x-p->icpoint->get_u())*nv_ptr[1] - (tmp1.y-p->icpoint->get_v())*nv_ptr[0];

      vic_ptr[10*k + 0] = tmp1.y;
      vic_ptr[10*k + 1] = tmp1.x;
      vic_ptr[10*k + 2] = tmp_dis1.x;
      vic_ptr[10*k + 3] = tmp_dis1.y;

      // fuzzy assignment a(d_{k,l}) = 1/2*(erf(d_{kl})/\sqrt(2)*sigma) + 1/2
      vic_ptr[10*k + 4] = 0.5*(erf((tmp_dis1.x)/(sqrt(2)*sigma)) + 1);
      //vic_ptr[10*k + 4] = logistic(tmp_dis1.x/(sqrt(2)*sigma));
      //double  wp1 = (a_{d,l} - gamm_1) /(1-gamma_1)
      double wp1 = (vic_ptr[10*k + 4] - ccdParameters.gamma_1)/(1-ccdParameters.gamma_1);

      // wp1^4, why? if a_{d,l} \approx 0.5, do not count the point
      vic_ptr[10*k + 5] = wp1*wp1*wp1*wp1;

      // wp1 = (1-a_{d,l} - gamm_1) /(1-gamma_1)
      // double wp2 = (1-vic_ptr[10*k + 4] - gamma_1)/(1-gamma_1);
      double wp2 = (1-vic_ptr[10*k + 4] - 0.25);
      vic_ptr[10*k + 6] = -64*wp2*wp2*wp2*wp2 + 0.25;
      // W_p(d_p, simga_p) = c*max[0, exp(-d_p^2/2*sigma_p'^2) - exp(-gamma_2))]
      vic_ptr[10*k + 7] = std::max((exp(-0.5*tmp_dis1.x*tmp_dis1.x/(sigma_hat*sigma_hat)) - exp(-ccdParameters.gamma_2)), 0.0);
      // W' = 0.5*exp(-|d_v= - d_p=|/alpha)/alpha
      vic_ptr[ 10*k + 8] = 0.5*exp(-abs(tmp_dis1.y)/alpha)/alpha;
      // the derivative of col_5: 1/(sqrt(2*PI)*sigma)*exp{-d_{k,l}^2/(2*sigma*sigma)}
      vic_ptr[ 10*k + 9] = exp(-tmp_dis1.x*tmp_dis1.x/(2*sigma*sigma))/(sqrt(2*CV_PI)*sigma);

      // calculate the normalization parameter c
      normalized_param.at<double>(i, 0) += vic_ptr[ 10*k + 7];


#ifdef DEBUG
      if(i == 0)
        std::cout << "tmp1 " << tmp1.x  << " " << tmp1.y << std::endl;
#endif

      ///////////////////////////////////////////////////////////////////////////////////////////
      // calculate in the direction -n: (-n_x, -n_y)
      /////////////////////////////////////////////////////////////////////////////////////////
      tmp2.x = round(p->icpoint->get_u() - j*nv_ptr[0]);
      tmp2.y = round(p->icpoint->get_v() - j*nv_ptr[1]);
      // cv::circle(canvas_tmp, cv::Point2d(tmp2.x, tmp2.y), 1, CV_RGB(255,0,0), 1);
#ifdef DEBUG
      if(i == 0)
        std::cout << "tmp2 " << tmp2.x  << " " << tmp2.y << std::endl;
#endif

      // start compute the size in the direction of -(n_x, n_y)
      tmp_dis2.x = (tmp2.x-p->icpoint->get_u())*nv_ptr[0] + (tmp2.y-p->icpoint->get_v())*nv_ptr[1];
      tmp_dis2.y = (tmp2.x-p->icpoint->get_u())*nv_ptr[1] - (tmp2.y-p->icpoint->get_v())*nv_ptr[0];
      int negative_normal = k + (int)floor(ccdParameters.h/ccdParameters.delta_h);
      vic_ptr[10*negative_normal + 0] = tmp2.y;
      vic_ptr[10*negative_normal + 1] = tmp2.x;
      vic_ptr[10*negative_normal + 2] = tmp_dis2.x;
      vic_ptr[10*negative_normal + 3] = tmp_dis2.y;
      //std::cout << " u " <<  p->icpoint->get_u() <<  " v " << p->icpoint->get_v() << " dist " << tmp_dis2.x << " nx " << nv_ptr[0] << " ny "<< nv_ptr[1] << " theta " << p->get_theta() << std::endl;
       vic_ptr[10*negative_normal + 4] = 0.5*(erf(tmp_dis2.x/(sqrt(2)*sigma)) + 1);
      //vic_ptr[10*negative_normal + 4] = logistic(tmp_dis2.x/(sqrt(2)*sigma));
      // vic_ptr[10*negative_normal + 4] = 0.5;
      wp1 = (vic_ptr[10*negative_normal + 4] - 0.25);
      vic_ptr[10*negative_normal + 5] = -64*wp1*wp1*wp1*wp1 + 0.25;
      wp2 = (1 - vic_ptr[10*negative_normal + 4] - ccdParameters.gamma_1)/(1-ccdParameters.gamma_1);
      vic_ptr[10*negative_normal + 6] = wp2*wp2*wp2*wp2;
      vic_ptr[10*negative_normal + 7] = std::max((exp(-0.5*tmp_dis2.x*tmp_dis2.x/(sigma_hat*sigma_hat)) - exp(-ccdParameters.gamma_2)), 0.0);
      vic_ptr[ 10*negative_normal + 8] = 0.5*exp(-abs(tmp_dis2.x)/alpha)/alpha;
      vic_ptr[ 10*negative_normal + 9] = exp(-tmp_dis2.x*tmp_dis2.x/(2*sigma*sigma))/(sqrt(2*CV_PI)*sigma);
      normalized_param.at<double>(i, 1) += vic_ptr[ 10*negative_normal + 7];
    }
//    i++;
  }


#ifdef DEBUG
  printf("%-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s\n",
         "x", "y", "dist_x", "dist_y", "a", "w1^4", "w2^4", "prox", "edf", "erf'"
         );
  for (int  i = 0; i < 20*floor(ccdParameters.h/ccdParameters.delta_h); ++i)
  {
    // std::cout << vic.at<double>(0,i) << "    ";
    printf("%-5f   ", vic.at<double>(0,i));
    if((i+1)%10 == 0)
      std::cout << std::endl;
  }
#endif

//#pragma omp parallel for
  for (int i = 0; i < resolution; ++i)
  {
    int k = 0;
    // w1 = \sum wp_1, w2 = \sum wp_2
    double w1 =0.0 , w2 = 0.0;

    // store mean value near the curve
    std::vector<double> m1(3,0.0), m2(3,0.0);

    // store the second mean value near the curve
    std::vector<double> m1_o2(9,0.0), m2_o2(9,0.0);

    // compute local statistics

    // start search the points in the +n direction as well as -n direction
    double wp1 = 0.0, wp2 = 0.0;

    double *vic_ptr = vic.ptr<double>(i);
    double *mean_vic_ptr = mean_vic.ptr<double>(i);
    double *cov_vic_ptr = cov_vic.ptr<double>(i);
    for (int j = ccdParameters.delta_h; j <= ccdParameters.h; j += ccdParameters.delta_h, k++)
    {
      wp1 = 0.0, wp2 = 0.0;
      int negative_normal = k + (int)floor(ccdParameters.h/ccdParameters.delta_h);

      // wp1 = w(a_{k,l})*w(d_{k,l})*w(d)
      wp1 = vic_ptr[ 10*k+ 5]*vic_ptr[ 10*k+ 7]/normalized_param.at<double>(i,0);

      // wp2 = w(a_{k,l})*w(d_{k,l})*w(d)
      wp2 = vic_ptr[ 10*k+ 6]*vic_ptr[ 10*k+ 7]/normalized_param.at<double>(i,1);

      //std::cout << " wp20 " << wp2 << " wp10 " << wp1 << std::endl;

      //wp2 = 0.001;

      //w1 = \sum{wp1}
      w1 += wp1;

      //w2 = \sum{wp2}

      w2 += wp2;

      // compute the mean value in the vicinity of a point
      // m_{ks} = I{k}^{s} = \sum_{l} w_{kls}{I_{kl}} : s = 1 or 2

      m1[0] += wp1*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[0];
      m1[1] += wp1*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[1];
      m1[2] += wp1*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[2];

      m2[0] += wp2*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[0];
      m2[1] += wp2*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[1];
      m2[2] += wp2*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[2];

      /*m2[0] += 0;
      m2[1] += 0;
      m2[2] += 0;*/

      // compute second order local statistics
      // m_{k,s} = \sum_{l} w_{kls} I_{kl}*I_{kl}^T

      for (int m = 0; m < 3; ++m)
      {
        for (int n =0; n < 3; ++n)
        {
          m1_o2[m*3+n] += wp1*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[m]
                          *img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[n];
          m2_o2[m*3+n] += wp2*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[m]
                          *img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[n];
        }
      }

      wp1 = vic_ptr[ 10*negative_normal+ 5]*vic_ptr[ 10*negative_normal+ 7]/normalized_param.at<double>(i,0);
      wp2 = vic_ptr[ 10*negative_normal+ 6]*vic_ptr[ 10*negative_normal+ 7]/normalized_param.at<double>(i,1);

      //std::cout << " wp1 " << wp1 << " wp2 " << wp2 << std::endl;
      //wp1 = 0.001;

      w1 += wp1;
      w2 += wp2;

      m1[0] += wp1*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[0];
      m1[1] += wp1*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[1];
      m1[2] += wp1*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[2];

      /*m1[0] += 0;
      m1[1] += 0;
      m1[2] += 0;*/

      m2[0] += wp2*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[0];
      m2[1] += wp2*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[1];
      m2[2] += wp2*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[2];

      for (int m = 0; m < 3; ++m)
      {
        for (int n =0; n < 3; ++n)
        {
          m1_o2[m*3+n] += wp1*img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[m]
                          *img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[n];
          m2_o2[m*3+n] += wp2*img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[m]
                          *img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[n];
        }
      }
    }
    // std::cout << "w1: " << "              w2:" << std::endl;
    // std::cout << "w1 == " << w1 << "  w2== " << w2 << std::endl;
    mean_vic_ptr[0] = m1[0]/w1;
    mean_vic_ptr[1] = m1[1]/w1;
    mean_vic_ptr[2] = m1[2]/w1;
    mean_vic_ptr[3] = m2[0]/w2;
    mean_vic_ptr[4] = m2[1]/w2;
    mean_vic_ptr[5] = m2[2]/w2;

    for (int m = 0; m < 3; ++m)
    {
      for (int n = 0 ; n < 3; ++n)
      {
        cov_vic_ptr[ m*3+n] = m1_o2[m*3+n]/w1 -m1[m]*m1[n]/(w1*w1);
        cov_vic_ptr[ 9+m*3+n] = m2_o2[m*3+n]/w2 -m2[m]*m2[n]/(w2*w2);
        if(m == n)
        {
          cov_vic_ptr[ m*3+n] += ccdParameters.kappa;
          cov_vic_ptr[ 9+m*3+n] += ccdParameters.kappa;
        }
      }
    }
  }
  normalized_param.release();
}


void apCCDTracker::computeLocalStatisticsPrev1()
{
  const cv::Mat_<cv::Vec3b>& img = (cv::Mat_<cv::Vec3b>&)image;
  const cv::Mat_<cv::Vec3b>& img_prev = (cv::Mat_<cv::Vec3b>&)image_prev;

  double alph = 0.8;
  double bet = 0.2;
  const double sigma = ccdParameters.h/(ccdParameters.alpha*ccdParameters.gamma_3);
  // sigma_hat = gamma_3 * sigma

  //  double sigma_hat = max(h/sqrt(2*gamma_2), gamma_4);
  const double sigma_hat = ccdParameters.gamma_3*sigma + ccdParameters.gamma_4;
  // to save the normalized parameters of vic[i,8]
  // dimension: resolution x 2
  // the first column save the normalized coefficient outside the curve
  // the second column store the one inside the curve
  cv::Mat normalized_param = cv::Mat::zeros(resolution, 2, CV_64F);

  vic = cv::Mat::zeros(resolution, 20*floor(ccdParameters.h/ccdParameters.delta_h), CV_64F);

//  CvRNG rng;
//  cv::Scalar color = random_color(&rng);
  /*for(int i=0; i < ccdParameters.resolution;i++)
  {*/
//  int i=0;
//#pragma omp parallel for
  for(int kk = 0 ; kk < pointsCCD[scaleLevel].size(); kk ++)
  {
    // temporary points used to store those points in the
    // normal direction as well as negative normal direction
    cv::Point3d tmp1, tmp2;

    // store the distance from a point in normal(negative norml) direction
    // to the point on the curve
    cv::Point3d tmp_dis1, tmp_dis2;

    const int i = kk;
    apControlPoint *p = pointsCCD[scaleLevel][kk];
    double *nv_ptr = nv.ptr<double>(i);
    // normal vector (n_x, n_y)
    // tagent vector (nv.at<double>(i,1), -n_x)
    int invnorm =0;

    if(p->invnormal)
    {
    nv_ptr[0] = -p->nxs;
    nv_ptr[1] = -p->nys;
    }
    else
    {
     nv_ptr[0] = p->nxs;
     nv_ptr[1] = p->nys;
    }

    // dimension = 4
    // 0 - x value
    // 1 - y value
    // 2 - normal vector x
    // 3 - normal vector y
    cv::Mat p_old = cv::Mat::zeros(resolution, 4, CV_64F);
    double *p_old_ptr = p_old.ptr<double>(i);

    // old value of the control points
    p_old_ptr[0] = p->icpoint->get_u();
    p_old_ptr[1] = p->icpoint->get_v();

    // old normal vector of the control points
    p_old_ptr[2] =p->icpoint->get_u();
    p_old_ptr[3] = p->icpoint->get_v();

    int k = 0;
    double alpha = 0.5;
    double *vic_ptr = vic.ptr<double>(i);
    double *vic_ptr_prev = vic_prev.ptr<double>(i);

    for (int j = ccdParameters.delta_h; j <= ccdParameters.h; j += ccdParameters.delta_h, k++)
    {
      ///////////////////////////////////////////////////////////////////////////////////////////
      // calculate in the direction +n: (n_x, n_y)
      /////////////////////////////////////////////////////////////////////////////////////////
      // x_{k,l}
      tmp1.x = round(p->icpoint->get_u() + j*nv_ptr[0]);
      // y_{k,l}
      tmp1.y = round(p->icpoint->get_v() + j*nv_ptr[1]);

      // distance between x_{k,l} and x_{k,0} in the normal direction
      // appoximately it is l*h, l = {1,2,3,.....}
      tmp_dis1.x = (tmp1.x-p->icpoint->get_u())*nv_ptr[0] + (tmp1.y-p->icpoint->get_v())*nv_ptr[1];

      // distance between y_{k,l} and y_{k,0} along the curve
      // it approximates 0
      tmp_dis1.y = (tmp1.x-p->icpoint->get_u())*nv_ptr[1] - (tmp1.y-p->icpoint->get_v())*nv_ptr[0];

      vic_ptr[10*k + 0] = tmp1.y;
      vic_ptr[10*k + 1] = tmp1.x;
      vic_ptr[10*k + 2] = tmp_dis1.x;
      vic_ptr[10*k + 3] = tmp_dis1.y;

      // fuzzy assignment a(d_{k,l}) = 1/2*(erf(d_{kl})/\sqrt(2)*sigma) + 1/2
      vic_ptr[10*k + 4] = 0.5*(erf((tmp_dis1.x)/(sqrt(2)*sigma)) + 1);
      //vic_ptr[10*k + 4] = logistic(tmp_dis1.x/(sqrt(2)*sigma));
      //double  wp1 = (a_{d,l} - gamm_1) /(1-gamma_1)
      double wp1 = (vic_ptr[10*k + 4] - ccdParameters.gamma_1)/(1-ccdParameters.gamma_1);

      // wp1^4, why? if a_{d,l} \approx 0.5, do not count the point
      vic_ptr[10*k + 5] = wp1*wp1*wp1*wp1;

      // wp1 = (1-a_{d,l} - gamm_1) /(1-gamma_1)
      // double wp2 = (1-vic_ptr[10*k + 4] - gamma_1)/(1-gamma_1);
      double wp2 = (1-vic_ptr[10*k + 4] - 0.25);
      vic_ptr[10*k + 6] = -64*wp2*wp2*wp2*wp2 + 0.25;
      // W_p(d_p, simga_p) = c*max[0, exp(-d_p^2/2*sigma_p'^2) - exp(-gamma_2))]
      vic_ptr[10*k + 7] = std::max((exp(-0.5*tmp_dis1.x*tmp_dis1.x/(sigma_hat*sigma_hat)) - exp(-ccdParameters.gamma_2)), 0.0);
      // W' = 0.5*exp(-|d_v= - d_p=|/alpha)/alpha
      vic_ptr[ 10*k + 8] = 0.5*exp(-abs(tmp_dis1.y)/alpha)/alpha;
      // the derivative of col_5: 1/(sqrt(2*PI)*sigma)*exp{-d_{k,l}^2/(2*sigma*sigma)}
      vic_ptr[ 10*k + 9] = exp(-tmp_dis1.x*tmp_dis1.x/(2*sigma*sigma))/(sqrt(2*CV_PI)*sigma);

      // calculate the normalization parameter c
      normalized_param.at<double>(i, 0) += vic_ptr[ 10*k + 7];


#ifdef DEBUG
      if(i == 0)
        std::cout << "tmp1 " << tmp1.x  << " " << tmp1.y << std::endl;
#endif

      ///////////////////////////////////////////////////////////////////////////////////////////
      // calculate in the direction -n: (-n_x, -n_y)
      /////////////////////////////////////////////////////////////////////////////////////////
      tmp2.x = round(p->icpoint->get_u() - j*nv_ptr[0]);
      tmp2.y = round(p->icpoint->get_v() - j*nv_ptr[1]);
      // cv::circle(canvas_tmp, cv::Point2d(tmp2.x, tmp2.y), 1, CV_RGB(255,0,0), 1);
#ifdef DEBUG
      if(i == 0)
        std::cout << "tmp2 " << tmp2.x  << " " << tmp2.y << std::endl;
#endif

      // start compute the size in the direction of -(n_x, n_y)
      tmp_dis2.x = (tmp2.x-p->icpoint->get_u())*nv_ptr[0] + (tmp2.y-p->icpoint->get_v())*nv_ptr[1];
      tmp_dis2.y = (tmp2.x-p->icpoint->get_u())*nv_ptr[1] - (tmp2.y-p->icpoint->get_v())*nv_ptr[0];
      int negative_normal = k + (int)floor(ccdParameters.h/ccdParameters.delta_h);
      vic_ptr[10*negative_normal + 0] = tmp2.y;
      vic_ptr[10*negative_normal + 1] = tmp2.x;
      vic_ptr[10*negative_normal + 2] = tmp_dis2.x;
      vic_ptr[10*negative_normal + 3] = tmp_dis2.y;
      //std::cout << " u " <<  p->icpoint->get_u() <<  " v " << p->icpoint->get_v() << " dist " << tmp_dis2.x << " nx " << nv_ptr[0] << " ny "<< nv_ptr[1] << " theta " << p->get_theta() << std::endl;
       vic_ptr[10*negative_normal + 4] = 0.5*(erf(tmp_dis2.x/(sqrt(2)*sigma)) + 1);
      //vic_ptr[10*negative_normal + 4] = logistic(tmp_dis2.x/(sqrt(2)*sigma));
      // vic_ptr[10*negative_normal + 4] = 0.5;
      wp1 = (vic_ptr[10*negative_normal + 4] - 0.25);
      vic_ptr[10*negative_normal + 5] = -64*wp1*wp1*wp1*wp1 + 0.25;
      wp2 = (1 - vic_ptr[10*negative_normal + 4] - ccdParameters.gamma_1)/(1-ccdParameters.gamma_1);
      vic_ptr[10*negative_normal + 6] = wp2*wp2*wp2*wp2;
      vic_ptr[10*negative_normal + 7] = std::max((exp(-0.5*tmp_dis2.x*tmp_dis2.x/(sigma_hat*sigma_hat)) - exp(-ccdParameters.gamma_2)), 0.0);
      vic_ptr[ 10*negative_normal + 8] = 0.5*exp(-abs(tmp_dis2.x)/alpha)/alpha;
      vic_ptr[ 10*negative_normal + 9] = exp(-tmp_dis2.x*tmp_dis2.x/(2*sigma*sigma))/(sqrt(2*CV_PI)*sigma);
      normalized_param.at<double>(i, 1) += vic_ptr[ 10*negative_normal + 7];
    }
//    i++;
  }


#ifdef DEBUG
  printf("%-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s\n",
         "x", "y", "dist_x", "dist_y", "a", "w1^4", "w2^4", "prox", "edf", "erf'"
         );
  for (int  i = 0; i < 20*floor(ccdParameters.h/ccdParameters.delta_h); ++i)
  {
    // std::cout << vic.at<double>(0,i) << "    ";
    printf("%-5f   ", vic.at<double>(0,i));
    if((i+1)%10 == 0)
      std::cout << std::endl;
  }
#endif

//#pragma omp parallel for
  for (int i = 0; i < resolution; ++i)
  {
    int k = 0;
    // w1 = \sum wp_1, w2 = \sum wp_2
    double w1 =0.0 , w2 = 0.0;

    // store mean value near the curve
    std::vector<double> m1(3,0.0), m2(3,0.0);

    // store the second mean value near the curve
    std::vector<double> m1_o2(9,0.0), m2_o2(9,0.0);

    // compute local statistics

    // start search the points in the +n direction as well as -n direction
    double wp1 = 0.0, wp2 = 0.0;

    double *vic_ptr = vic.ptr<double>(i);
    double *vic_ptr_prev = vic_prev.ptr<double>(i);
    double *mean_vic_ptr = mean_vic.ptr<double>(i);
    double *cov_vic_ptr = cov_vic.ptr<double>(i);
    for (int j = ccdParameters.delta_h; j <= ccdParameters.h; j += ccdParameters.delta_h, k++)
    {
      wp1 = 0.0, wp2 = 0.0;
      int negative_normal = k + (int)floor(ccdParameters.h/ccdParameters.delta_h);

      // wp1 = w(a_{k,l})*w(d_{k,l})*w(d)
      wp1 = vic_ptr[ 10*k+ 5]*vic_ptr[ 10*k+ 7]/normalized_param.at<double>(i,0);

      // wp2 = w(a_{k,l})*w(d_{k,l})*w(d)
      wp2 = vic_ptr[ 10*k+ 6]*vic_ptr[ 10*k+ 7]/normalized_param.at<double>(i,1);

      //std::cout << " wp20 " << wp2 << " wp10 " << wp1 << std::endl;

      //wp2 = 0.001;

      //w1 = \sum{wp1}
      w1 += wp1;

      //w2 = \sum{wp2}

      w2 += wp2;

      // compute the mean value in the vicinity of a point
      // m_{ks} = I{k}^{s} = \sum_{l} w_{kls}{I_{kl}} : s = 1 or 2

      m1[0] += wp1*(alph*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[0] + bet*img_prev(vic_ptr_prev[ 10*k + 0 ], vic_ptr_prev[ 10*k + 1 ])[0]);
      m1[1] += wp1*(alph*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[1] + bet*img_prev(vic_ptr_prev[ 10*k + 0 ], vic_ptr_prev[ 10*k + 1 ])[1]);
      m1[2] +=  wp1*(alph*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[2] + bet*img_prev(vic_ptr_prev[ 10*k + 0 ], vic_ptr_prev[ 10*k + 1 ])[2]);
      m2[0] +=  wp2*(alph*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[0] + bet*img_prev(vic_ptr_prev[ 10*k + 0 ], vic_ptr_prev[ 10*k + 1 ])[0]);
      m2[1] +=  wp2*(alph*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[1] + bet*img_prev(vic_ptr_prev[ 10*k + 0 ], vic_ptr_prev[ 10*k + 1 ])[1]);
      m2[2] +=  wp2*(alph*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[2] + bet*img_prev(vic_ptr_prev[ 10*k + 0 ], vic_ptr_prev[ 10*k + 1 ])[2]);

      /*m2[0] += 0;
      m2[1] += 0;
      m2[2] += 0;*/

      // compute second order local statistics
      // m_{k,s} = \sum_{l} w_{kls} I_{kl}*I_{kl}^T

      for (int m = 0; m < 3; ++m)
      {
        for (int n =0; n < 3; ++n)
        {
          m1_o2[m*3+n] += wp1*(alph*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[m] + bet*img_prev(vic_ptr_prev[ 10*k + 0 ], vic_ptr_prev[ 10*k + 1 ])[m])
                          *(alph*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[n] + bet*img_prev(vic_ptr_prev[ 10*k + 0 ], vic_ptr_prev[ 10*k + 1 ])[n]);
          m2_o2[m*3+n] += wp2*(alph*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[m] + bet*img_prev(vic_ptr_prev[ 10*k + 0 ], vic_ptr_prev[ 10*k + 1 ])[m])
                                  *(alph*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[n] + bet*img_prev(vic_ptr_prev[ 10*k + 0 ], vic_ptr_prev[ 10*k + 1 ])[n]);
        }
      }

      wp1 = vic_ptr[ 10*negative_normal+ 5]*vic_ptr[ 10*negative_normal+ 7]/normalized_param.at<double>(i,0);
      wp2 = vic_ptr[ 10*negative_normal+ 6]*vic_ptr[ 10*negative_normal+ 7]/normalized_param.at<double>(i,1);

      //std::cout << " wp1 " << wp1 << " wp2 " << wp2 << std::endl;
      //wp1 = 0.001;

      w1 += wp1;
      w2 += wp2;

      m1[0] += wp1*(alph*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[0] + bet*img_prev(vic_ptr_prev[10*negative_normal+0], vic_ptr_prev[10*negative_normal+1])[0]);
      m1[1] += wp1*(alph*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[1] + bet*img_prev(vic_ptr_prev[10*negative_normal+0], vic_ptr_prev[10*negative_normal+1])[1]);
      m1[2] += wp1*(alph*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[2] + bet*img_prev(vic_ptr_prev[10*negative_normal+0], vic_ptr_prev[10*negative_normal+1])[2]);

      /*m1[0] += 0;
      m1[1] += 0;
      m1[2] += 0;*/

      m2[0] += wp2*(alph*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[0] + bet*img_prev(vic_ptr_prev[10*negative_normal+0], vic_ptr_prev[10*negative_normal+1])[0]);
      m2[1] += wp2*(alph*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[1] + bet*img_prev(vic_ptr_prev[10*negative_normal+0], vic_ptr_prev[10*negative_normal+1])[1]);
      m2[2] += wp2*(alph*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[2] + bet*img_prev(vic_ptr_prev[10*negative_normal+0], vic_ptr_prev[10*negative_normal+1])[2]);

      for (int m = 0; m < 3; ++m)
      {
        for (int n =0; n < 3; ++n)
        {
          m1_o2[m*3+n] += wp1*(alph*img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[m] + bet*img_prev(vic_ptr_prev[ 10*negative_normal + 0 ], vic_ptr_prev[ 10*negative_normal + 1 ])[m])
                          *(alph*img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[n] + bet*img_prev(vic_ptr_prev[ 10*negative_normal + 0 ], vic_ptr_prev[ 10*negative_normal + 1 ])[n]);
          m2_o2[m*3+n] += wp2*(alph*img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[m] + bet*img_prev(vic_ptr_prev[ 10*negative_normal + 0 ], vic_ptr_prev[ 10*negative_normal + 1 ])[m])
                                  *(alph*img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[n] + bet*img_prev(vic_ptr_prev[ 10*negative_normal + 0 ], vic_ptr_prev[ 10*negative_normal + 1 ])[n]);
        }
      }
    }
    // std::cout << "w1: " << "              w2:" << std::endl;
    // std::cout << "w1 == " << w1 << "  w2== " << w2 << std::endl;
    mean_vic_ptr[0] = m1[0]/w1;
    mean_vic_ptr[1] = m1[1]/w1;
    mean_vic_ptr[2] = m1[2]/w1;
    mean_vic_ptr[3] = m2[0]/w2;
    mean_vic_ptr[4] = m2[1]/w2;
    mean_vic_ptr[5] = m2[2]/w2;

    for (int m = 0; m < 3; ++m)
    {
      for (int n = 0 ; n < 3; ++n)
      {
        cov_vic_ptr[ m*3+n] = m1_o2[m*3+n]/w1 -m1[m]*m1[n]/(w1*w1);
        cov_vic_ptr[ 9+m*3+n] = m2_o2[m*3+n]/w2 -m2[m]*m2[n]/(w2*w2);
        if(m == n)
        {
          cov_vic_ptr[ m*3+n] += ccdParameters.kappa;
          cov_vic_ptr[ 9+m*3+n] += ccdParameters.kappa;
        }
      }
    }
  }
  normalized_param.release();
}



void apCCDTracker::setOld()
{
  const cv::Mat_<cv::Vec3b>& img = (cv::Mat_<cv::Vec3b>&)image_prev;
  const double sigma = ccdParameters.h/(ccdParameters.alpha*ccdParameters.gamma_3);
  // sigma_hat = gamma_3 * sigma

  //  double sigma_hat = max(h/sqrt(2*gamma_2), gamma_4);
  const double sigma_hat = ccdParameters.gamma_3*sigma + ccdParameters.gamma_4;
  // to save the normalized parameters of vic[i,8]
  // dimension: resolution x 2
  // the first column save the normalized coefficient outside the curve
  // the second column store the one inside the curve
  cv::Mat normalized_param = cv::Mat::zeros(resolution, 2, CV_64F);

  vic_prev = cv::Mat::zeros(resolution, 20*floor(ccdParameters.h/ccdParameters.delta_h), CV_64F);

//  CvRNG rng;
//  cv::Scalar color = random_color(&rng);
  /*for(int i=0; i < ccdParameters.resolution;i++)
  {*/
//  int i=0;
//#pragma omp parallel for
  for(int kk = 0 ; kk < pointsCCD[scaleLevel].size(); kk ++)
  {
    // temporary points used to store those points in the
    // normal direction as well as negative normal direction
    cv::Point3d tmp1, tmp2;

    // store the distance from a point in normal(negative norml) direction
    // to the point on the curve
    cv::Point3d tmp_dis1, tmp_dis2;

    const int i = kk;
    apControlPoint *p = pointsCCD[scaleLevel][kk];
    double *nv_ptr = nv.ptr<double>(i);
    // normal vector (n_x, n_y)
    // tagent vector (nv.at<double>(i,1), -n_x)
    int invnorm =0;

    if(p->invnormal)
    {
    nv_ptr[0] = -p->nxs;
    nv_ptr[1] = -p->nys;
    }
    else
    {
     nv_ptr[0] = p->nxs;
     nv_ptr[1] = p->nys;
    }

    p->nxs_old = nv_ptr[0];
    p->nys_old = nv_ptr[1];

    p->xs_old = p->icpoint->get_u();
    p->ys_old = p->icpoint->get_v();
  }
}

void apCCDTracker::computeLocalStatisticsPrev0()
{
  const cv::Mat_<cv::Vec3b>& img = (cv::Mat_<cv::Vec3b>&)image;
  const cv::Mat_<cv::Vec3b>& imgprev = (cv::Mat_<cv::Vec3b>&)image_prev;

  const double sigma = ccdParameters.h/(ccdParameters.alpha*ccdParameters.gamma_3);
  // sigma_hat = gamma_3 * sigma

  //  double sigma_hat = max(h/sqrt(2*gamma_2), gamma_4);
  const double sigma_hat = ccdParameters.gamma_3*sigma + ccdParameters.gamma_4;
  // to save the normalized parameters of vic[i,8]
  // dimension: resolution x 2
  // the first column save the normalized coefficient outside the curve
  // the second column store the one inside the curve
  cv::Mat normalized_param = cv::Mat::zeros(resolution, 2, CV_64F);
  cv::Mat normalized_param_old = cv::Mat::zeros(resolution, 2, CV_64F);


  //vic = cv::Mat::zeros(resolution, 20*floor(ccdParameters.h/ccdParameters.delta_h), CV_64F);
  vic_old = cv::Mat::zeros(resolution, 20*floor(ccdParameters.h/ccdParameters.delta_h), CV_64F);


//  CvRNG rng;
//  cv::Scalar color = random_color(&rng);
  /*for(int i=0; i < ccdParameters.resolution;i++)
  {*/
//  int i=0;
//#pragma omp parallel for
  for(int kk = 0 ; kk < pointsCCD[scaleLevel].size(); kk ++)
  {
    // temporary points used to store those points in the
    // normal direction as well as negative normal direction
    cv::Point3d tmp1, tmp2, tmp1_old, tmp2_old;

    // store the distance from a point in normal(negative norml) direction
    // to the point on the curve
    cv::Point3d tmp_dis1, tmp_dis2, tmp_dis1_old, tmp_dis2_old;;

    const int i = kk;
    apControlPoint *p = pointsCCD[scaleLevel][kk];
    double *nv_ptr = nv.ptr<double>(i);
    // normal vector (n_x, n_y)
    // tagent vector (nv.at<double>(i,1), -n_x)
    int invnorm =0;

    if(p->invnormal)
    {
    nv_ptr[0] = -p->nxs;
    nv_ptr[1] = -p->nys;
    }
    else
    {
     nv_ptr[0] = p->nxs;
     nv_ptr[1] = p->nys;
    }

    // dimension = 4
    // 0 - x value
    // 1 - y value
    // 2 - normal vector x
    // 3 - normal vector y
    cv::Mat p_old = cv::Mat::zeros(resolution, 4, CV_64F);
    double *p_old_ptr = p_old.ptr<double>(i);

    // old value of the control points
    p_old_ptr[0] = p->icpoint->get_u();
    p_old_ptr[1] = p->icpoint->get_v();

    // old normal vector of the control points
    p_old_ptr[2] =p->icpoint->get_u();
    p_old_ptr[3] = p->icpoint->get_v();

    int k = 0;
    double alpha = 0.5;
    double *vic_ptr = vic.ptr<double>(i);
    double *vic_ptr_old = vic_old.ptr<double>(i);

    for (int j = ccdParameters.delta_h; j <= ccdParameters.h; j += ccdParameters.delta_h, k++)
    {
      ///////////////////////////////////////////////////////////////////////////////////////////
      // calculate in the direction +n: (n_x, n_y)
      /////////////////////////////////////////////////////////////////////////////////////////
      // x_{k,l}
      tmp1.x = round(p->icpoint->get_u() + j*nv_ptr[0]);
      // y_{k,l}
      tmp1.y = round(p->icpoint->get_v() + j*nv_ptr[1]);

      tmp1_old.x = round(p->xs_old + j*p->nxs_old);
            // y_{k,l}
      tmp1_old.y = round(p->ys_old + j*p->nys_old);

      // distance between x_{k,l} and x_{k,0} in the normal direction
      // appoximately it is l*h, l = {1,2,3,.....}
      tmp_dis1.x = (tmp1.x-p->icpoint->get_u())*nv_ptr[0] + (tmp1.y-p->icpoint->get_v())*nv_ptr[1];

      // distance between y_{k,l} and y_{k,0} along the curve
      // it approximates 0
      tmp_dis1.y = (tmp1.x-p->icpoint->get_u())*nv_ptr[1] - (tmp1.y-p->icpoint->get_v())*nv_ptr[0];

      tmp_dis1_old.x = (tmp1_old.x-p->xs_old)*p->nxs_old + (tmp1_old.y-p->nys_old)*p->nys_old;

       // distance between y_{k,l} and y_{k,0} along the curve
       // it approximates 0
       tmp_dis1_old.y = (tmp1_old.x-p->xs_old)*p->nys_old - (tmp1.y-p->ys_old)*p->nxs_old;


      vic_ptr_old[10*k + 0] = tmp1.y;
      vic_ptr_old[10*k + 1] = tmp1.x;
      vic_ptr_old[10*k + 2] = tmp_dis1.x;
      vic_ptr_old[10*k + 3] = tmp_dis1.y;

      // fuzzy assignment a(d_{k,l}) = 1/2*(erf(d_{kl})/\sqrt(2)*sigma) + 1/2
      vic_ptr_old[10*k + 4] = 0.5*(erf((tmp_dis1.x)/(sqrt(2)*sigma)) + 1);
      //vic_ptr[10*k + 4] = logistic(tmp_dis1.x/(sqrt(2)*sigma));
      //double  wp1 = (a_{d,l} - gamm_1) /(1-gamma_1)
      double wp1 = (vic_ptr_old[10*k + 4] - ccdParameters.gamma_1)/(1-ccdParameters.gamma_1);

      // wp1^4, why? if a_{d,l} \approx 0.5, do not count the point
      vic_ptr[10*k + 5] = wp1*wp1*wp1*wp1;

      // wp1 = (1-a_{d,l} - gamm_1) /(1-gamma_1)
      // double wp2 = (1-vic_ptr[10*k + 4] - gamma_1)/(1-gamma_1);
      double wp2 = (1-vic_ptr[10*k + 4] - 0.25);
      vic_ptr[10*k + 6] = -64*wp2*wp2*wp2*wp2 + 0.25;
      // W_p(d_p, simga_p) = c*max[0, exp(-d_p^2/2*sigma_p'^2) - exp(-gamma_2))]
      vic_ptr[10*k + 7] = std::max((exp(-0.5*tmp_dis1.x*tmp_dis1.x/(sigma_hat*sigma_hat)) - exp(-ccdParameters.gamma_2)), 0.0);
      // W' = 0.5*exp(-|d_v= - d_p=|/alpha)/alpha
      vic_ptr[ 10*k + 8] = 0.5*exp(-abs(tmp_dis1.y)/alpha)/alpha;
      // the derivative of col_5: 1/(sqrt(2*PI)*sigma)*exp{-d_{k,l}^2/(2*sigma*sigma)}
      vic_ptr[ 10*k + 9] = exp(-tmp_dis1.x*tmp_dis1.x/(2*sigma*sigma))/(sqrt(2*CV_PI)*sigma);

      // calculate the normalization parameter c
      normalized_param.at<double>(i, 0) += vic_ptr[ 10*k + 7];


#ifdef DEBUG
      if(i == 0)
        std::cout << "tmp1 " << tmp1.x  << " " << tmp1.y << std::endl;
#endif

      ///////////////////////////////////////////////////////////////////////////////////////////
      // calculate in the direction -n: (-n_x, -n_y)
      /////////////////////////////////////////////////////////////////////////////////////////
      tmp2.x = round(p->icpoint->get_u() - j*nv_ptr[0]);
      tmp2.y = round(p->icpoint->get_v() - j*nv_ptr[1]);
      // cv::circle(canvas_tmp, cv::Point2d(tmp2.x, tmp2.y), 1, CV_RGB(255,0,0), 1);
#ifdef DEBUG
      if(i == 0)
        std::cout << "tmp2 " << tmp2.x  << " " << tmp2.y << std::endl;
#endif

      // start compute the size in the direction of -(n_x, n_y)
      tmp_dis2.x = (tmp2.x-p->icpoint->get_u())*nv_ptr[0] + (tmp2.y-p->icpoint->get_v())*nv_ptr[1];
      tmp_dis2.y = (tmp2.x-p->icpoint->get_u())*nv_ptr[1] - (tmp2.y-p->icpoint->get_v())*nv_ptr[0];
      int negative_normal = k + (int)floor(ccdParameters.h/ccdParameters.delta_h);
      vic_ptr[10*negative_normal + 0] = tmp2.y;
      vic_ptr[10*negative_normal + 1] = tmp2.x;
      vic_ptr[10*negative_normal + 2] = tmp_dis2.x;
      vic_ptr[10*negative_normal + 3] = tmp_dis2.y;
      //std::cout << " u " <<  p->icpoint->get_u() <<  " v " << p->icpoint->get_v() << " dist " << tmp_dis2.x << " nx " << nv_ptr[0] << " ny "<< nv_ptr[1] << " theta " << p->get_theta() << std::endl;
       vic_ptr[10*negative_normal + 4] = 0.5*(erf(tmp_dis2.x/(sqrt(2)*sigma)) + 1);
      //vic_ptr[10*negative_normal + 4] = logistic(tmp_dis2.x/(sqrt(2)*sigma));
      // vic_ptr[10*negative_normal + 4] = 0.5;
      wp1 = (vic_ptr[10*negative_normal + 4] - 0.25);
      vic_ptr[10*negative_normal + 5] = -64*wp1*wp1*wp1*wp1 + 0.25;
      wp2 = (1 - vic_ptr[10*negative_normal + 4] - ccdParameters.gamma_1)/(1-ccdParameters.gamma_1);
      vic_ptr[10*negative_normal + 6] = wp2*wp2*wp2*wp2;
      vic_ptr[10*negative_normal + 7] = std::max((exp(-0.5*tmp_dis2.x*tmp_dis2.x/(sigma_hat*sigma_hat)) - exp(-ccdParameters.gamma_2)), 0.0);
      vic_ptr[ 10*negative_normal + 8] = 0.5*exp(-abs(tmp_dis2.x)/alpha)/alpha;
      vic_ptr[ 10*negative_normal + 9] = exp(-tmp_dis2.x*tmp_dis2.x/(2*sigma*sigma))/(sqrt(2*CV_PI)*sigma);
      normalized_param.at<double>(i, 1) += vic_ptr[ 10*negative_normal + 7];



      vic_ptr_old[10*k + 0] = tmp1.y;
      vic_ptr_old[10*k + 1] = tmp1.x;
      vic_ptr_old[10*k + 2] = tmp_dis1.x;
      vic_ptr_old[10*k + 3] = tmp_dis1.y;

      // fuzzy assignment a(d_{k,l}) = 1/2*(erf(d_{kl})/\sqrt(2)*sigma) + 1/2
      vic_ptr_old[10*k + 4] = 0.5*(erf((tmp_dis1_old.x)/(sqrt(2)*sigma)) + 1);
      //vic_ptr[10*k + 4] = logistic(tmp_dis1.x/(sqrt(2)*sigma));
      //double  wp1 = (a_{d,l} - gamm_1) /(1-gamma_1)
      double wp1_old = (vic_ptr_old[10*k + 4] - ccdParameters.gamma_1)/(1-ccdParameters.gamma_1);

      // wp1^4, why? if a_{d,l} \approx 0.5, do not count the point
      vic_ptr_old[10*k + 5] = wp1_old*wp1_old*wp1_old*wp1_old;

      // wp1 = (1-a_{d,l} - gamm_1) /(1-gamma_1)
      // double wp2 = (1-vic_ptr[10*k + 4] - gamma_1)/(1-gamma_1);
      double wp2_old = (1-vic_ptr_old[10*k + 4] - 0.25);
      vic_ptr_old[10*k + 6] = -64*wp2_old*wp2_old*wp2_old*wp2_old + 0.25;
      // W_p(d_p, simga_p) = c*max[0, exp(-d_p^2/2*sigma_p'^2) - exp(-gamma_2))]
      vic_ptr_old[10*k + 7] = std::max((exp(-0.5*tmp_dis1_old.x*tmp_dis1_old.x/(sigma_hat*sigma_hat)) - exp(-ccdParameters.gamma_2)), 0.0);
      // W' = 0.5*exp(-|d_v= - d_p=|/alpha)/alpha
      vic_ptr_old[ 10*k + 8] = 0.5*exp(-abs(tmp_dis1_old.y)/alpha)/alpha;
      // the derivative of col_5: 1/(sqrt(2*PI)*sigma)*exp{-d_{k,l}^2/(2*sigma*sigma)}
      vic_ptr_old[ 10*k + 9] = exp(-tmp_dis1_old.x*tmp_dis1_old.x/(2*sigma*sigma))/(sqrt(2*CV_PI)*sigma);

      // calculate the normalization parameter c
      normalized_param_old.at<double>(i, 0) += vic_ptr_old[ 10*k + 7];


#ifdef DEBUG
      if(i == 0)
        std::cout << "tmp1 " << tmp1.x  << " " << tmp1.y << std::endl;
#endif

      ///////////////////////////////////////////////////////////////////////////////////////////
      // calculate in the direction -n: (-n_x, -n_y)
      /////////////////////////////////////////////////////////////////////////////////////////
      tmp2_old.x = round(p->xs_old - j*p->nxs_old);
      tmp2_old.y = round(p->ys_old - j*p->nys_old);
      // cv::circle(canvas_tmp, cv::Point2d(tmp2.x, tmp2.y), 1, CV_RGB(255,0,0), 1);
#ifdef DEBUG
      if(i == 0)
        std::cout << "tmp2 " << tmp2.x  << " " << tmp2.y << std::endl;
#endif

      // start compute the size in the direction of -(n_x, n_y)
      tmp_dis2_old.x = (tmp2_old.x-p->xs)*nv_ptr[0] + (tmp2_old.y-p->icpoint->get_v())*nv_ptr[1];
      tmp_dis2_old.y = (tmp2_old.x-p->xs)*nv_ptr[1] - (tmp2_old.y-p->icpoint->get_v())*nv_ptr[0];
      vic_ptr[10*negative_normal + 0] = tmp2.y;
      vic_ptr[10*negative_normal + 1] = tmp2.x;
      vic_ptr[10*negative_normal + 2] = tmp_dis2.x;
      vic_ptr[10*negative_normal + 3] = tmp_dis2.y;
      //std::cout << " u " <<  p->icpoint->get_u() <<  " v " << p->icpoint->get_v() << " dist " << tmp_dis2.x << " nx " << nv_ptr[0] << " ny "<< nv_ptr[1] << " theta " << p->get_theta() << std::endl;
       vic_ptr[10*negative_normal + 4] = 0.5*(erf(tmp_dis2.x/(sqrt(2)*sigma)) + 1);
      //vic_ptr[10*negative_normal + 4] = logistic(tmp_dis2.x/(sqrt(2)*sigma));
      // vic_ptr[10*negative_normal + 4] = 0.5;
      wp1 = (vic_ptr[10*negative_normal + 4] - 0.25);
      vic_ptr[10*negative_normal + 5] = -64*wp1*wp1*wp1*wp1 + 0.25;
      wp2 = (1 - vic_ptr[10*negative_normal + 4] - ccdParameters.gamma_1)/(1-ccdParameters.gamma_1);
      vic_ptr[10*negative_normal + 6] = wp2*wp2*wp2*wp2;
      vic_ptr[10*negative_normal + 7] = std::max((exp(-0.5*tmp_dis2.x*tmp_dis2.x/(sigma_hat*sigma_hat)) - exp(-ccdParameters.gamma_2)), 0.0);
      vic_ptr[ 10*negative_normal + 8] = 0.5*exp(-abs(tmp_dis2.x)/alpha)/alpha;
      vic_ptr[ 10*negative_normal + 9] = exp(-tmp_dis2.x*tmp_dis2.x/(2*sigma*sigma))/(sqrt(2*CV_PI)*sigma);
      normalized_param.at<double>(i, 1) += vic_ptr[ 10*negative_normal + 7];
    }
//    i++;
  }


#ifdef DEBUG
  printf("%-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s\n",
         "x", "y", "dist_x", "dist_y", "a", "w1^4", "w2^4", "prox", "edf", "erf'"
         );
  for (int  i = 0; i < 20*floor(ccdParameters.h/ccdParameters.delta_h); ++i)
  {
    // std::cout << vic.at<double>(0,i) << "    ";
    printf("%-5f   ", vic.at<double>(0,i));
    if((i+1)%10 == 0)
      std::cout << std::endl;
  }
#endif

//#pragma omp parallel for
  for (int i = 0; i < resolution; ++i)
  {
    int k = 0;
    // w1 = \sum wp_1, w2 = \sum wp_2
    double w1 =0.0 , w2 = 0.0;

    // store mean value near the curve
    std::vector<double> m1(3,0.0), m2(3,0.0);

    // store the second mean value near the curve
    std::vector<double> m1_o2(9,0.0), m2_o2(9,0.0);

    // compute local statistics

    // start search the points in the +n direction as well as -n direction
    double wp1 = 0.0, wp2 = 0.0;

    double *vic_ptr = vic.ptr<double>(i);
    double *mean_vic_ptr = mean_vic.ptr<double>(i);
    double *cov_vic_ptr = cov_vic.ptr<double>(i);
    for (int j = ccdParameters.delta_h; j <= ccdParameters.h; j += ccdParameters.delta_h, k++)
    {
      wp1 = 0.0, wp2 = 0.0;
      int negative_normal = k + (int)floor(ccdParameters.h/ccdParameters.delta_h);

      // wp1 = w(a_{k,l})*w(d_{k,l})*w(d)
      wp1 = vic_ptr[ 10*k+ 5]*vic_ptr[ 10*k+ 7]/normalized_param.at<double>(i,0);

      // wp2 = w(a_{k,l})*w(d_{k,l})*w(d)
      wp2 = vic_ptr[ 10*k+ 6]*vic_ptr[ 10*k+ 7]/normalized_param.at<double>(i,1);

      //std::cout << " wp20 " << wp2 << " wp10 " << wp1 << std::endl;

      //wp2 = 0.001;

      //w1 = \sum{wp1}
      w1 += wp1;

      //w2 = \sum{wp2}

      w2 += wp2;

      // compute the mean value in the vicinity of a point
      // m_{ks} = I{k}^{s} = \sum_{l} w_{kls}{I_{kl}} : s = 1 or 2

      m1[0] += wp1*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[0];
      m1[1] += wp1*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[1];
      m1[2] += wp1*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[2];

      m2[0] += wp2*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[0];
      m2[1] += wp2*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[1];
      m2[2] += wp2*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[2];

      /*m2[0] += 0;
      m2[1] += 0;
      m2[2] += 0;*/

      // compute second order local statistics
      // m_{k,s} = \sum_{l} w_{kls} I_{kl}*I_{kl}^T

      for (int m = 0; m < 3; ++m)
      {
        for (int n =0; n < 3; ++n)
        {
          m1_o2[m*3+n] += wp1*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[m]
                          *img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[n];
          m2_o2[m*3+n] += wp2*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[m]
                          *img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[n];
        }
      }

      wp1 = vic_ptr[ 10*negative_normal+ 5]*vic_ptr[ 10*negative_normal+ 7]/normalized_param.at<double>(i,0);
      wp2 = vic_ptr[ 10*negative_normal+ 6]*vic_ptr[ 10*negative_normal+ 7]/normalized_param.at<double>(i,1);

      //std::cout << " wp1 " << wp1 << " wp2 " << wp2 << std::endl;
      //wp1 = 0.001;

      w1 += wp1;
      w2 += wp2;

      m1[0] += wp1*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[0];
      m1[1] += wp1*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[1];
      m1[2] += wp1*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[2];

      /*m1[0] += 0;
      m1[1] += 0;
      m1[2] += 0;*/

      m2[0] += wp2*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[0];
      m2[1] += wp2*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[1];
      m2[2] += wp2*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[2];

      for (int m = 0; m < 3; ++m)
      {
        for (int n =0; n < 3; ++n)
        {
          m1_o2[m*3+n] += wp1*img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[m]
                          *img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[n];
          m2_o2[m*3+n] += wp2*img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[m]
                          *img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[n];
        }
      }
    }
    // std::cout << "w1: " << "              w2:" << std::endl;
    // std::cout << "w1 == " << w1 << "  w2== " << w2 << std::endl;
    mean_vic_ptr[0] = m1[0]/w1;
    mean_vic_ptr[1] = m1[1]/w1;
    mean_vic_ptr[2] = m1[2]/w1;
    mean_vic_ptr[3] = m2[0]/w2;
    mean_vic_ptr[4] = m2[1]/w2;
    mean_vic_ptr[5] = m2[2]/w2;

    for (int m = 0; m < 3; ++m)
    {
      for (int n = 0 ; n < 3; ++n)
      {
        cov_vic_ptr[ m*3+n] = m1_o2[m*3+n]/w1 -m1[m]*m1[n]/(w1*w1);
        cov_vic_ptr[ 9+m*3+n] = m2_o2[m*3+n]/w2 -m2[m]*m2[n]/(w2*w2);
        if(m == n)
        {
          cov_vic_ptr[ m*3+n] += ccdParameters.kappa;
          cov_vic_ptr[ 9+m*3+n] += ccdParameters.kappa;
        }
      }
    }
  }
  normalized_param.release();
}




void apCCDTracker::computeLocalStatisticsPrev(const vpImage<unsigned char> &I)
{
  const cv::Mat_<cv::Vec3b>& img = (cv::Mat_<cv::Vec3b>&)image_prev;
  const double sigma = ccdParameters.h/(ccdParameters.alpha*ccdParameters.gamma_3);
  // sigma_hat = gamma_3 * sigma

  //  double sigma_hat = max(h/sqrt(2*gamma_2), gamma_4);
  const double sigma_hat = ccdParameters.gamma_3*sigma + ccdParameters.gamma_4;
  // to save the normalized parameters of vic[i,8]
  // dimension: resolution x 2
  // the first column save the normalized coefficient outside the curve
  // the second column store the one inside the curve
  cv::Mat normalized_param = cv::Mat::zeros(resolution, 2, CV_64F);

  vic_prev = cv::Mat::zeros(resolution, 20*floor(ccdParameters.h/ccdParameters.delta_h), CV_64F);

//  CvRNG rng;
//  cv::Scalar color = random_color(&rng);
  /*for(int i=0; i < ccdParameters.resolution;i++)
  {*/
//  int i=0;
//#pragma omp parallel for
  for(int kk = 0 ; kk < pointsCCD[scaleLevel].size(); kk ++)
  {
    // temporary points used to store those points in the
    // normal direction as well as negative normal direction
    cv::Point3d tmp1, tmp2;

    // store the distance from a point in normal(negative norml) direction
    // to the point on the curve
    cv::Point3d tmp_dis1, tmp_dis2;

    const int i = kk;
    apControlPoint *p = pointsCCD[scaleLevel][kk];
    double *nv_ptr = nv.ptr<double>(i);
    // normal vector (n_x, n_y)
    // tagent vector (nv.at<double>(i,1), -n_x)
    int invnorm =0;

    if(p->invnormal)
    {
    nv_ptr[0] = -p->nxs;
    nv_ptr[1] = -p->nys;
    }
    else
    {
     nv_ptr[0] = p->nxs;
     nv_ptr[1] = p->nys;
    }

    // dimension = 4
    // 0 - x value
    // 1 - y value
    // 2 - normal vector x
    // 3 - normal vector y
    cv::Mat p_old = cv::Mat::zeros(resolution, 4, CV_64F);
    double *p_old_ptr = p_old.ptr<double>(i);

    // old value of the control points
    p_old_ptr[0] = p->icpoint->get_u();
    p_old_ptr[1] = p->icpoint->get_v();

    // old normal vector of the control points
    p_old_ptr[2] =p->icpoint->get_u();
    p_old_ptr[3] = p->icpoint->get_v();

    int k = 0;
    double alpha = 0.5;
    double *vic_ptr = vic_prev.ptr<double>(i);
    for (int j = ccdParameters.delta_h; j <= ccdParameters.h; j += ccdParameters.delta_h, k++)
    {
      ///////////////////////////////////////////////////////////////////////////////////////////
      // calculate in the direction +n: (n_x, n_y)
      /////////////////////////////////////////////////////////////////////////////////////////
      // x_{k,l}
      tmp1.x = round(p->icpoint->get_u() + j*nv_ptr[0]);
      // y_{k,l}
      tmp1.y = round(p->icpoint->get_v() + j*nv_ptr[1]);

   	 //vpDisplay::displayCross(I,vpImagePoint((int)tmp1.y,(int)tmp1.x),2,vpColor::red,2);

      // distance between x_{k,l} and x_{k,0} in the normal direction
      // appoximately it is l*h, l = {1,2,3,.....}
      tmp_dis1.x = (tmp1.x-p->icpoint->get_u())*nv_ptr[0] + (tmp1.y-p->icpoint->get_v())*nv_ptr[1];

      // distance between y_{k,l} and y_{k,0} along the curve
      // it approximates 0
      tmp_dis1.y = (tmp1.x-p->icpoint->get_u())*nv_ptr[1] - (tmp1.y-p->icpoint->get_v())*nv_ptr[0];

      vic_ptr[10*k + 0] = tmp1.y;
      vic_ptr[10*k + 1] = tmp1.x;
      vic_ptr[10*k + 2] = tmp_dis1.x;
      vic_ptr[10*k + 3] = tmp_dis1.y;

      // fuzzy assignment a(d_{k,l}) = 1/2*(erf(d_{kl})/\sqrt(2)*sigma) + 1/2
      vic_ptr[10*k + 4] = 0.5*(erf((tmp_dis1.x)/(sqrt(2)*sigma)) + 1);
      //vic_ptr[10*k + 4] = logistic(tmp_dis1.x/(sqrt(2)*sigma));
      //double  wp1 = (a_{d,l} - gamm_1) /(1-gamma_1)
      double wp1 = (vic_ptr[10*k + 4] - ccdParameters.gamma_1)/(1-ccdParameters.gamma_1);

      // wp1^4, why? if a_{d,l} \approx 0.5, do not count the point
      vic_ptr[10*k + 5] = wp1*wp1*wp1*wp1;

      // wp1 = (1-a_{d,l} - gamm_1) /(1-gamma_1)
      // double wp2 = (1-vic_ptr[10*k + 4] - gamma_1)/(1-gamma_1);
      double wp2 = (1-vic_ptr[10*k + 4] - 0.25);
      vic_ptr[10*k + 6] = -64*wp2*wp2*wp2*wp2 + 0.25;
      // W_p(d_p, simga_p) = c*max[0, exp(-d_p^2/2*sigma_p'^2) - exp(-gamma_2))]
      vic_ptr[10*k + 7] = std::max((exp(-0.5*tmp_dis1.x*tmp_dis1.x/(sigma_hat*sigma_hat)) - exp(-ccdParameters.gamma_2)), 0.0);
      // W' = 0.5*exp(-|d_v= - d_p=|/alpha)/alpha
      vic_ptr[ 10*k + 8] = 0.5*exp(-abs(tmp_dis1.y)/alpha)/alpha;
      // the derivative of col_5: 1/(sqrt(2*PI)*sigma)*exp{-d_{k,l}^2/(2*sigma*sigma)}
      vic_ptr[ 10*k + 9] = exp(-tmp_dis1.x*tmp_dis1.x/(2*sigma*sigma))/(sqrt(2*CV_PI)*sigma);

      // calculate the normalization parameter c
      normalized_param.at<double>(i, 0) += vic_ptr[ 10*k + 7];


#ifdef DEBUG
      if(i == 0)
        std::cout << "tmp1 " << tmp1.x  << " " << tmp1.y << std::endl;
#endif

      ///////////////////////////////////////////////////////////////////////////////////////////
      // calculate in the direction -n: (-n_x, -n_y)
      /////////////////////////////////////////////////////////////////////////////////////////
      tmp2.x = round(p->icpoint->get_u() - j*nv_ptr[0]);
      tmp2.y = round(p->icpoint->get_v() - j*nv_ptr[1]);

    	 //vpDisplay::displayCross(I,vpImagePoint((int)tmp2.y,(int)tmp2.x),2,vpColor::blue,2);

      // cv::circle(canvas_tmp, cv::Point2d(tmp2.x, tmp2.y), 1, CV_RGB(255,0,0), 1);
#ifdef DEBUG
      if(i == 0)
        std::cout << "tmp2 " << tmp2.x  << " " << tmp2.y << std::endl;
#endif

      // start compute the size in the direction of -(n_x, n_y)
      tmp_dis2.x = (tmp2.x-p->icpoint->get_u())*nv_ptr[0] + (tmp2.y-p->icpoint->get_v())*nv_ptr[1];
      tmp_dis2.y = (tmp2.x-p->icpoint->get_u())*nv_ptr[1] - (tmp2.y-p->icpoint->get_v())*nv_ptr[0];
      int negative_normal = k + (int)floor(ccdParameters.h/ccdParameters.delta_h);
      vic_ptr[10*negative_normal + 0] = tmp2.y;
      vic_ptr[10*negative_normal + 1] = tmp2.x;
      vic_ptr[10*negative_normal + 2] = tmp_dis2.x;
      vic_ptr[10*negative_normal + 3] = tmp_dis2.y;
      //std::cout << " u " <<  p->icpoint->get_u() <<  " v " << p->icpoint->get_v() << " dist " << tmp_dis2.x << " nx " << nv_ptr[0] << " ny "<< nv_ptr[1] << " theta " << p->get_theta() << std::endl;
       vic_ptr[10*negative_normal + 4] = 0.5*(erf(tmp_dis2.x/(sqrt(2)*sigma)) + 1);
      //vic_ptr[10*negative_normal + 4] = logistic(tmp_dis2.x/(sqrt(2)*sigma));
      // vic_ptr[10*negative_normal + 4] = 0.5;
      wp1 = (vic_ptr[10*negative_normal + 4] - 0.25);
      vic_ptr[10*negative_normal + 5] = -64*wp1*wp1*wp1*wp1 + 0.25;
      wp2 = (1 - vic_ptr[10*negative_normal + 4] - ccdParameters.gamma_1)/(1-ccdParameters.gamma_1);
      vic_ptr[10*negative_normal + 6] = wp2*wp2*wp2*wp2;
      vic_ptr[10*negative_normal + 7] = std::max((exp(-0.5*tmp_dis2.x*tmp_dis2.x/(sigma_hat*sigma_hat)) - exp(-ccdParameters.gamma_2)), 0.0);
      vic_ptr[ 10*negative_normal + 8] = 0.5*exp(-abs(tmp_dis2.x)/alpha)/alpha;
      vic_ptr[ 10*negative_normal + 9] = exp(-tmp_dis2.x*tmp_dis2.x/(2*sigma*sigma))/(sqrt(2*CV_PI)*sigma);
      normalized_param.at<double>(i, 1) += vic_ptr[ 10*negative_normal + 7];
    }
//    i++;
  }


#ifdef DEBUG
  printf("%-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s\n",
         "x", "y", "dist_x", "dist_y", "a", "w1^4", "w2^4", "prox", "edf", "erf'"
         );
  for (int  i = 0; i < 20*floor(ccdParameters.h/ccdParameters.delta_h); ++i)
  {
    // std::cout << vic.at<double>(0,i) << "    ";
    printf("%-5f   ", vic.at<double>(0,i));
    if((i+1)%10 == 0)
      std::cout << std::endl;
  }
#endif

//#pragma omp parallel for
  for (int i = 0; i < resolution; ++i)
  {
    int k = 0;
    // w1 = \sum wp_1, w2 = \sum wp_2
    double w1 =0.0 , w2 = 0.0;

    // store mean value near the curve
    std::vector<double> m1(3,0.0), m2(3,0.0);

    // store the second mean value near the curve
    std::vector<double> m1_o2(9,0.0), m2_o2(9,0.0);

    // compute local statistics

    // start search the points in the +n direction as well as -n direction
    double wp1 = 0.0, wp2 = 0.0;

    double *vic_ptr = vic_prev.ptr<double>(i);
    double *mean_vic_ptr = mean_vic_prev.ptr<double>(i);
    double *cov_vic_ptr = cov_vic_prev.ptr<double>(i);
    for (int j = ccdParameters.delta_h; j <= ccdParameters.h; j += ccdParameters.delta_h, k++)
    {
      wp1 = 0.0, wp2 = 0.0;
      int negative_normal = k + (int)floor(ccdParameters.h/ccdParameters.delta_h);

      // wp1 = w(a_{k,l})*w(d_{k,l})*w(d)
      wp1 = vic_ptr[ 10*k+ 5]*vic_ptr[ 10*k+ 7]/normalized_param.at<double>(i,0);
      // wp2 = w(a_{k,l})*w(d_{k,l})*w(d)
      wp2 = vic_ptr[ 10*k+ 6]*vic_ptr[ 10*k+ 7]/normalized_param.at<double>(i,1);

      //wp2 = 0.001;

      //w1 = \sum{wp1}
      w1 += wp1;

      //w2 = \sum{wp2}
      w2 += wp2;

      // compute the mean value in the vicinity of a point
      // m_{ks} = I{k}^{s} = \sum_{l} w_{kls}{I_{kl}} : s = 1 or 2

      m1[0] += wp1*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[0];
      m1[1] += wp1*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[1];
      m1[2] += wp1*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[2];


      m2[0] += wp2*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[0];
      m2[1] += wp2*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[1];
      m2[2] += wp2*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[2];

      /*m2[0] += 0;
      m2[1] += 0;
      m2[2] += 0;*/

      // compute second order local statistics
      // m_{k,s} = \sum_{l} w_{kls} I_{kl}*I_{kl}^T

      for (int m = 0; m < 3; ++m)
      {
        for (int n =0; n < 3; ++n)
        {
          m1_o2[m*3+n] += wp1*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[m]
                          *img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[n];
          m2_o2[m*3+n] += wp2*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[m]
                          *img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[n];
        }
      }

      wp1 = vic_ptr[ 10*negative_normal+ 5]*vic_ptr[ 10*negative_normal+ 7]/normalized_param.at<double>(i,0);
      wp2 = vic_ptr[ 10*negative_normal+ 6]*vic_ptr[ 10*negative_normal+ 7]/normalized_param.at<double>(i,1);

      //wp1 = 0.001;

      w1 += wp1;
      w2 += wp2;

      m1[0] += wp1*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[0];
      m1[1] += wp1*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[1];
      m1[2] += wp1*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[2];

      /*m1[0] += 0;
      m1[1] += 0;
      m1[2] += 0;*/

      m2[0] += wp2*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[0];
      m2[1] += wp2*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[1];
      m2[2] += wp2*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[2];

      for (int m = 0; m < 3; ++m)
      {
        for (int n =0; n < 3; ++n)
        {
          m1_o2[m*3+n] += wp1*img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[m]
                          *img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[n];
          m2_o2[m*3+n] += wp2*img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[m]
                          *img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[n];
        }
      }
    }
    // std::cout << "w1: " << "              w2:" << std::endl;
    // std::cout << "w1 == " << w1 << "  w2== " << w2 << std::endl;
    mean_vic_ptr[0] = m1[0]/w1;
    mean_vic_ptr[1] = m1[1]/w1;
    mean_vic_ptr[2] = m1[2]/w1;
    mean_vic_ptr[3] = m2[0]/w2;
    mean_vic_ptr[4] = m2[1]/w2;
    mean_vic_ptr[5] = m2[2]/w2;
    //std::cout << " w1 == " << mean_vic_ptr[0]  << "  w2== " << mean_vic_ptr[1] << "  w3== " << mean_vic_ptr[2] << " w4 == " << mean_vic_ptr[3]  << "  w5== " << mean_vic_ptr[4] << "  w6== " << mean_vic_ptr[5] <<std::endl;

    for (int m = 0; m < 3; ++m)
    {
      for (int n = 0 ; n < 3; ++n)
      {
        cov_vic_ptr[ m*3+n] = m1_o2[m*3+n]/w1 -m1[m]*m1[n]/(w1*w1);
        cov_vic_ptr[ 9+m*3+n] = m2_o2[m*3+n]/w2 -m2[m]*m2[n]/(w2*w2);
        if(m == n)
        {
          cov_vic_ptr[ m*3+n] += ccdParameters.kappa;
          cov_vic_ptr[ 9+m*3+n] += ccdParameters.kappa;
        }
      }
    }
  }
  normalized_param.release();
}


void apCCDTracker::computeLocalStatisticsPrevSpace()
{
  const cv::Mat_<cv::Vec3b>& img = (cv::Mat_<cv::Vec3b>&)image_prev;
  const double sigma = ccdParameters.h/(ccdParameters.alpha*ccdParameters.gamma_3);
  // sigma_hat = gamma_3 * sigma

  //  double sigma_hat = max(h/sqrt(2*gamma_2), gamma_4);
  const double sigma_hat = ccdParameters.gamma_3*sigma + ccdParameters.gamma_4;
  // to save the normalized parameters of vic[i,8]
  // dimension: resolution x 2
  // the first column save the normalized coefficient outside the curve
  // the second column store the one inside the curve
  cv::Mat normalized_param = cv::Mat::zeros(resolution, 2, CV_64F);

  vic_prev = cv::Mat::zeros(resolution, 20*floor(ccdParameters.h/ccdParameters.delta_h), CV_64F);

//  CvRNG rng;
//  cv::Scalar color = random_color(&rng);
  /*for(int i=0; i < ccdParameters.resolution;i++)
  {*/
//  int i=0;
#pragma omp parallel for
  for(int kk = 0 ; kk < pointsCCD[scaleLevel].size(); kk ++)
  {
    // temporary points used to store those points in the
    // normal direction as well as negative normal direction
    cv::Point3d tmp1, tmp2;

    // store the distance from a point in normal(negative norml) direction
    // to the point on the curve
    cv::Point3d tmp_dis1, tmp_dis2;

    const int i = kk;
    apControlPoint *p = pointsCCD[scaleLevel][kk];
    double *nv_ptr = nv.ptr<double>(i);
    // normal vector (n_x, n_y)
    // tagent vector (nv.at<double>(i,1), -n_x)
    nv_ptr[0] = p->nxs;
    nv_ptr[1] = p->nys;

    // dimension = 4
    // 0 - x value
    // 1 - y value
    // 2 - normal vector x
    // 3 - normal vector y
    cv::Mat p_old = cv::Mat::zeros(resolution, 4, CV_64F);
    double *p_old_ptr = p_old.ptr<double>(i);

    // old value of the control points
    p_old_ptr[0] = p->icpoint->get_u();
    p_old_ptr[1] = p->icpoint->get_v();

    // old normal vector of the control points
    p_old_ptr[2] =p->icpoint->get_u();
    p_old_ptr[3] = p->icpoint->get_v();

    int k = 0;
    double alpha = 0.5;
    double *vic_ptr = vic_prev.ptr<double>(i);
    for (int j = ccdParameters.delta_h; j <= ccdParameters.h; j += ccdParameters.delta_h, k++)
    {
      ///////////////////////////////////////////////////////////////////////////////////////////
      // calculate in the direction +n: (n_x, n_y)
      /////////////////////////////////////////////////////////////////////////////////////////
      // x_{k,l}
      tmp1.x = round(p->icpoint->get_u() + j*nv_ptr[0]);
      // y_{k,l}
      tmp1.y = round(p->icpoint->get_v() + j*nv_ptr[1]);

      // distance between x_{k,l} and x_{k,0} in the normal direction
      // appoximately it is l*h, l = {1,2,3,.....}
      tmp_dis1.x = (tmp1.x-p->icpoint->get_u())*nv_ptr[0] + (tmp1.y-p->icpoint->get_v())*nv_ptr[1];

      // distance between y_{k,l} and y_{k,0} along the curve
      // it approximates 0
      tmp_dis1.y = (tmp1.x-p->icpoint->get_u())*nv_ptr[1] - (tmp1.y-p->icpoint->get_v())*nv_ptr[0];

      vic_ptr[10*k + 0] = tmp1.y;
      vic_ptr[10*k + 1] = tmp1.x;
      vic_ptr[10*k + 2] = tmp_dis1.x;
      vic_ptr[10*k + 3] = tmp_dis1.y;

      // fuzzy assignment a(d_{k,l}) = 1/2*(erf(d_{kl})/\sqrt(2)*sigma) + 1/2
      vic_ptr[10*k + 4] = 0.5*(erf((tmp_dis1.x)/(sqrt(2)*sigma)) + 1);
      //vic_ptr[10*k + 4] = logistic(tmp_dis1.x/(sqrt(2)*sigma));
      //double  wp1 = (a_{d,l} - gamm_1) /(1-gamma_1)
      double wp1 = (vic_ptr[10*k + 4] - ccdParameters.gamma_1)/(1-ccdParameters.gamma_1);

      // wp1^4, why? if a_{d,l} \approx 0.5, do not count the point
      vic_ptr[10*k + 5] = wp1*wp1*wp1*wp1;

      // wp1 = (1-a_{d,l} - gamm_1) /(1-gamma_1)
      // double wp2 = (1-vic_ptr[10*k + 4] - gamma_1)/(1-gamma_1);
      double wp2 = (1-vic_ptr[10*k + 4] - 0.25);
      vic_ptr[10*k + 6] = -64*wp2*wp2*wp2*wp2 + 0.25;
      // W_p(d_p, simga_p) = c*max[0, exp(-d_p^2/2*sigma_p'^2) - exp(-gamma_2))]
      vic_ptr[10*k + 7] = std::max((exp(-0.5*tmp_dis1.x*tmp_dis1.x/(sigma_hat*sigma_hat)) - exp(-ccdParameters.gamma_2)), 0.0);
      // W' = 0.5*exp(-|d_v= - d_p=|/alpha)/alpha
      vic_ptr[ 10*k + 8] = 0.5*exp(-abs(tmp_dis1.y)/alpha)/alpha;
      // the derivative of col_5: 1/(sqrt(2*PI)*sigma)*exp{-d_{k,l}^2/(2*sigma*sigma)}
      vic_ptr[ 10*k + 9] = exp(-tmp_dis1.x*tmp_dis1.x/(2*sigma*sigma))/(sqrt(2*CV_PI)*sigma);

      // calculate the normalization parameter c
      normalized_param.at<double>(i, 0) += vic_ptr[ 10*k + 7];


#ifdef DEBUG
      if(i == 0)
        std::cout << "tmp1 " << tmp1.x  << " " << tmp1.y << std::endl;
#endif

      ///////////////////////////////////////////////////////////////////////////////////////////
      // calculate in the direction -n: (-n_x, -n_y)
      /////////////////////////////////////////////////////////////////////////////////////////
      tmp2.x = round(p->icpoint->get_u() - j*nv_ptr[0]);
      tmp2.y = round(p->icpoint->get_v() - j*nv_ptr[1]);
      // cv::circle(canvas_tmp, cv::Point2d(tmp2.x, tmp2.y), 1, CV_RGB(255,0,0), 1);
#ifdef DEBUG
      if(i == 0)
        std::cout << "tmp2 " << tmp2.x  << " " << tmp2.y << std::endl;
#endif

      // start compute the size in the direction of -(n_x, n_y)
      tmp_dis2.x = (tmp2.x-p->icpoint->get_u())*nv_ptr[0] + (tmp2.y-p->icpoint->get_v())*nv_ptr[1];
      tmp_dis2.y = (tmp2.x-p->icpoint->get_u())*nv_ptr[1] - (tmp2.y-p->icpoint->get_v())*nv_ptr[0];
      int negative_normal = k + (int)floor(ccdParameters.h/ccdParameters.delta_h);
      vic_ptr[10*negative_normal + 0] = tmp2.y;
      vic_ptr[10*negative_normal + 1] = tmp2.x;
      vic_ptr[10*negative_normal + 2] = tmp_dis2.x;
      vic_ptr[10*negative_normal + 3] = tmp_dis2.y;
      //std::cout << " u " <<  p->icpoint->get_u() <<  " v " << p->icpoint->get_v() << " dist " << tmp_dis2.x << " nx " << nv_ptr[0] << " ny "<< nv_ptr[1] << " theta " << p->get_theta() << std::endl;
       vic_ptr[10*negative_normal + 4] = 0.5*(erf(tmp_dis2.x/(sqrt(2)*sigma)) + 1);
      //vic_ptr[10*negative_normal + 4] = logistic(tmp_dis2.x/(sqrt(2)*sigma));
      // vic_ptr[10*negative_normal + 4] = 0.5;
      wp1 = (vic_ptr[10*negative_normal + 4] - 0.25);
      vic_ptr[10*negative_normal + 5] = -64*wp1*wp1*wp1*wp1 + 0.25;
      wp2 = (1 - vic_ptr[10*negative_normal + 4] - ccdParameters.gamma_1)/(1-ccdParameters.gamma_1);
      vic_ptr[10*negative_normal + 6] = wp2*wp2*wp2*wp2;
      vic_ptr[10*negative_normal + 7] = std::max((exp(-0.5*tmp_dis2.x*tmp_dis2.x/(sigma_hat*sigma_hat)) - exp(-ccdParameters.gamma_2)), 0.0);
      vic_ptr[ 10*negative_normal + 8] = 0.5*exp(-abs(tmp_dis2.x)/alpha)/alpha;
      vic_ptr[ 10*negative_normal + 9] = exp(-tmp_dis2.x*tmp_dis2.x/(2*sigma*sigma))/(sqrt(2*CV_PI)*sigma);
      normalized_param.at<double>(i, 1) += vic_ptr[ 10*negative_normal + 7];
    }
//    i++;
  }


#ifdef DEBUG
  printf("%-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s\n",
         "x", "y", "dist_x", "dist_y", "a", "w1^4", "w2^4", "prox", "edf", "erf'"
         );
  for (int  i = 0; i < 20*floor(ccdParameters.h/ccdParameters.delta_h); ++i)
  {
    // std::cout << vic.at<double>(0,i) << "    ";
    printf("%-5f   ", vic.at<double>(0,i));
    if((i+1)%10 == 0)
      std::cout << std::endl;
  }
#endif

#pragma omp parallel for
  for (int i = 0; i < resolution; ++i)
  {
    int k = 0;
    // w1 = \sum wp_1, w2 = \sum wp_2
    double w1 =0.0 , w2 = 0.0;

    // store mean value near the curve
    std::vector<double> m1(3,0.0), m2(3,0.0);

    // store the second mean value near the curve
    std::vector<double> m1_o2(9,0.0), m2_o2(9,0.0);

    // compute local statistics

    // start search the points in the +n direction as well as -n direction
    double wp1 = 0.0, wp2 = 0.0;

    double *vic_ptr = vic_prev.ptr<double>(i);
    double *mean_vic_ptr = mean_vic_prev.ptr<double>(i);
    double *cov_vic_ptr = cov_vic_prev.ptr<double>(i);
    for (int j = ccdParameters.delta_h; j <= ccdParameters.h; j += ccdParameters.delta_h, k++)
    {
      wp1 = 0.0, wp2 = 0.0;
      int negative_normal = k + (int)floor(ccdParameters.h/ccdParameters.delta_h);

      // wp1 = w(a_{k,l})*w(d_{k,l})*w(d)
      wp1 = vic_ptr[ 10*k+ 5]*vic_ptr[ 10*k+ 7]/normalized_param.at<double>(i,0);

      // wp2 = w(a_{k,l})*w(d_{k,l})*w(d)
      wp2 = vic_ptr[ 10*k+ 6]*vic_ptr[ 10*k+ 7]/normalized_param.at<double>(i,1);;

      //w1 = \sum{wp1}
      w1 += wp1;

      //w2 = \sum{wp2}
      w2 += wp2;

      // compute the mean value in the vicinity of a point
      // m_{ks} = I{k}^{s} = \sum_{l} w_{kls}{I_{kl}} : s = 1 or 2

      m1[0] += wp1*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[0];
      m1[1] += wp1*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[1];
      m1[2] += wp1*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[2];
      m2[0] += wp2*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[0];
      m2[1] += wp2*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[1];
      m2[2] += wp2*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[2];

      /*m1[0] += 0;
      m1[1] += 0;
      m1[2] += 0;
      m2[0] += 0;
      m2[1] += 0;
      m2[2] += 0;*/

      // compute second order local statistics
      // m_{k,s} = \sum_{l} w_{kls} I_{kl}*I_{kl}^T

      for (int m = 0; m < 3; ++m)
      {
        for (int n =0; n < 3; ++n)
        {
          m1_o2[m*3+n] += wp1*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[m]
                          *img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[n];
          m2_o2[m*3+n] += wp2*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[m]
                          *img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[n];
        }
      }

      wp1 = vic_ptr[ 10*negative_normal+ 5]*vic_ptr[ 10*negative_normal+ 7]/normalized_param.at<double>(i,0);
      wp2 = vic_ptr[ 10*negative_normal+ 6]*vic_ptr[ 10*negative_normal+ 7]/normalized_param.at<double>(i,1);

      w1 += wp1;
      w2 += wp2;

      m1[0] += 0;
      m1[1] += 0;
      m1[2] += 0;
      m2[0] += 0;
      m2[1] += 0;
      m2[2] += 0;

      /*m1[0] += wp1*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[0];
      m1[1] += wp1*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[1];
      m1[2] += wp1*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[2];
      m2[0] += wp2*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[0];
      m2[1] += wp2*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[1];
      m2[2] += wp2*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[2];*/

      for (int m = 0; m < 3; ++m)
      {
        for (int n =0; n < 3; ++n)
        {
          m1_o2[m*3+n] += wp1*img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[m]
                          *img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[n];
          m2_o2[m*3+n] += wp2*img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[m]
                          *img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[n];
        }
      }

      /*for (int m = 0; m < 3; ++m)
      {
        for (int n =0; n < 3; ++n)
        {
          m1_o2[m*3+n] += 0;
          m2_o2[m*3+n] += 0;
        }
      }*/

    }
    // std::cout << "w1: " << "              w2:" << std::endl;
    // std::cout << "w1 == " << w1 << "  w2== " << w2 << std::endl;
    mean_vic_ptr[0] = m1[0]/w1;
    mean_vic_ptr[1] = m1[1]/w1;
    mean_vic_ptr[2] = m1[2]/w1;

    /*mean_vic_ptr[0] = 0;
    mean_vic_ptr[1] = 0;
    mean_vic_ptr[2] = 0;*/

    mean_vic_ptr[3] = m2[0]/w2;
    mean_vic_ptr[4] = m2[1]/w2;
    mean_vic_ptr[5] = m2[2]/w2;

    mean_vic_ptr[3] = 0;
    mean_vic_ptr[4] = 0;
    mean_vic_ptr[5] = 0;

    for (int m = 0; m < 3; ++m)
    {
      for (int n = 0 ; n < 3; ++n)
      {
        cov_vic_ptr[ m*3+n] = m1_o2[m*3+n]/w1 -m1[m]*m1[n]/(w1*w1);
        cov_vic_ptr[ 9+m*3+n] = m2_o2[m*3+n]/w2 -m2[m]*m2[n]/(w2*w2);
        if(m == n)
        {
          cov_vic_ptr[ m*3+n] += ccdParameters.kappa;
          cov_vic_ptr[ 9+m*3+n] += ccdParameters.kappa;
        }
      }
    }
  }
  normalized_param.release();
}

void apCCDTracker::computeLocalStatisticsSpace()
{
  const cv::Mat_<cv::Vec3b>& img = (cv::Mat_<cv::Vec3b>&)image;
  const double sigma = ccdParameters.h/(ccdParameters.alpha*ccdParameters.gamma_3);
  // sigma_hat = gamma_3 * sigma

  //  double sigma_hat = max(h/sqrt(2*gamma_2), gamma_4);
  const double sigma_hat = ccdParameters.gamma_3*sigma + ccdParameters.gamma_4;
  // to save the normalized parameters of vic[i,8]
  // dimension: resolution x 2
  // the first column save the normalized coefficient outside the curve
  // the second column store the one inside the curve
  cv::Mat normalized_param = cv::Mat::zeros(resolution, 2, CV_64F);

  vic = cv::Mat::zeros(resolution, 20*floor(ccdParameters.h/ccdParameters.delta_h), CV_64F);

//  CvRNG rng;
//  cv::Scalar color = random_color(&rng);
  /*for(int i=0; i < ccdParameters.resolution;i++)
  {*/
//  int i=0;
#pragma omp parallel for
  for(int kk = 0 ; kk < pointsCCD[scaleLevel].size(); kk ++)
  {
    // temporary points used to store those points in the
    // normal direction as well as negative normal direction
    cv::Point3d tmp1, tmp2;

    // store the distance from a point in normal(negative norml) direction
    // to the point on the curve
    cv::Point3d tmp_dis1, tmp_dis2;

    const int i = kk;
    apControlPoint *p = pointsCCD[scaleLevel][kk];
    double *nv_ptr = nv.ptr<double>(i);
    // normal vector (n_x, n_y)
    // tagent vector (nv.at<double>(i,1), -n_x)
    nv_ptr[0] = p->nxs;
    nv_ptr[1] = p->nys;

    // dimension = 4
    // 0 - x value
    // 1 - y value
    // 2 - normal vector x
    // 3 - normal vector y
    cv::Mat p_old = cv::Mat::zeros(resolution, 4, CV_64F);
    double *p_old_ptr = p_old.ptr<double>(i);

    // old value of the control points
    p_old_ptr[0] = p->icpoint->get_u();
    p_old_ptr[1] = p->icpoint->get_v();

    // old normal vector of the control points
    p_old_ptr[2] =p->icpoint->get_u();
    p_old_ptr[3] = p->icpoint->get_v();

    int k = 0;
    double alpha = 0.5;
    double *vic_ptr = vic.ptr<double>(i);
    for (int j = ccdParameters.delta_h; j <= ccdParameters.h; j += ccdParameters.delta_h, k++)
    {
      ///////////////////////////////////////////////////////////////////////////////////////////
      // calculate in the direction +n: (n_x, n_y)
      /////////////////////////////////////////////////////////////////////////////////////////
      // x_{k,l}
      tmp1.x = round(p->icpoint->get_u() + j*nv_ptr[0]);
      // y_{k,l}
      tmp1.y = round(p->icpoint->get_v() + j*nv_ptr[1]);

      // distance between x_{k,l} and x_{k,0} in the normal direction
      // appoximately it is l*h, l = {1,2,3,.....}
      tmp_dis1.x = (tmp1.x-p->icpoint->get_u())*nv_ptr[0] + (tmp1.y-p->icpoint->get_v())*nv_ptr[1];

      // distance between y_{k,l} and y_{k,0} along the curve
      // it approximates 0
      tmp_dis1.y = (tmp1.x-p->icpoint->get_u())*nv_ptr[1] - (tmp1.y-p->icpoint->get_v())*nv_ptr[0];

      vic_ptr[10*k + 0] = tmp1.y;
      vic_ptr[10*k + 1] = tmp1.x;
      vic_ptr[10*k + 2] = tmp_dis1.x;
      vic_ptr[10*k + 3] = tmp_dis1.y;

      // fuzzy assignment a(d_{k,l}) = 1/2*(erf(d_{kl})/\sqrt(2)*sigma) + 1/2
      vic_ptr[10*k + 4] = 0.5*(erf((tmp_dis1.x)/(sqrt(2)*sigma)) + 1);
      //vic_ptr[10*k + 4] = logistic(tmp_dis1.x/(sqrt(2)*sigma));
      //double  wp1 = (a_{d,l} - gamm_1) /(1-gamma_1)
      double wp1 = (vic_ptr[10*k + 4] - ccdParameters.gamma_1)/(1-ccdParameters.gamma_1);

      // wp1^4, why? if a_{d,l} \approx 0.5, do not count the point
      vic_ptr[10*k + 5] = wp1*wp1*wp1*wp1;

      // wp1 = (1-a_{d,l} - gamm_1) /(1-gamma_1)
      // double wp2 = (1-vic_ptr[10*k + 4] - gamma_1)/(1-gamma_1);
      double wp2 = (1-vic_ptr[10*k + 4] - 0.25);
      vic_ptr[10*k + 6] = -64*wp2*wp2*wp2*wp2 + 0.25;
      // W_p(d_p, simga_p) = c*max[0, exp(-d_p^2/2*sigma_p'^2) - exp(-gamma_2))]
      vic_ptr[10*k + 7] = std::max((exp(-0.5*tmp_dis1.x*tmp_dis1.x/(sigma_hat*sigma_hat)) - exp(-ccdParameters.gamma_2)), 0.0);
      // W' = 0.5*exp(-|d_v= - d_p=|/alpha)/alpha
      vic_ptr[ 10*k + 8] = 0.5*exp(-abs(tmp_dis1.y)/alpha)/alpha;
      // the derivative of col_5: 1/(sqrt(2*PI)*sigma)*exp{-d_{k,l}^2/(2*sigma*sigma)}
      vic_ptr[ 10*k + 9] = exp(-tmp_dis1.x*tmp_dis1.x/(2*sigma*sigma))/(sqrt(2*CV_PI)*sigma);

      // calculate the normalization parameter c
      normalized_param.at<double>(i, 0) += vic_ptr[ 10*k + 7];


#ifdef DEBUG
      if(i == 0)
        std::cout << "tmp1 " << tmp1.x  << " " << tmp1.y << std::endl;
#endif

      ///////////////////////////////////////////////////////////////////////////////////////////
      // calculate in the direction -n: (-n_x, -n_y)
      /////////////////////////////////////////////////////////////////////////////////////////
      tmp2.x = round(p->icpoint->get_u() - j*nv_ptr[0]);
      tmp2.y = round(p->icpoint->get_v() - j*nv_ptr[1]);
      // cv::circle(canvas_tmp, cv::Point2d(tmp2.x, tmp2.y), 1, CV_RGB(255,0,0), 1);
#ifdef DEBUG
      if(i == 0)
        std::cout << "tmp2 " << tmp2.x  << " " << tmp2.y << std::endl;
#endif

      // start compute the size in the direction of -(n_x, n_y)
      tmp_dis2.x = (tmp2.x-p->icpoint->get_u())*nv_ptr[0] + (tmp2.y-p->icpoint->get_v())*nv_ptr[1];
      tmp_dis2.y = (tmp2.x-p->icpoint->get_u())*nv_ptr[1] - (tmp2.y-p->icpoint->get_v())*nv_ptr[0];
      int negative_normal = k + (int)floor(ccdParameters.h/ccdParameters.delta_h);
      vic_ptr[10*negative_normal + 0] = tmp2.y;
      vic_ptr[10*negative_normal + 1] = tmp2.x;
      vic_ptr[10*negative_normal + 2] = tmp_dis2.x;
      vic_ptr[10*negative_normal + 3] = tmp_dis2.y;
      //std::cout << " u " <<  p->icpoint->get_u() <<  " v " << p->icpoint->get_v() << " dist " << tmp_dis2.x << " nx " << nv_ptr[0] << " ny "<< nv_ptr[1] << " theta " << p->get_theta() << std::endl;
       vic_ptr[10*negative_normal + 4] = 0.5*(erf(tmp_dis2.x/(sqrt(2)*sigma)) + 1);
      //vic_ptr[10*negative_normal + 4] = logistic(tmp_dis2.x/(sqrt(2)*sigma));
      // vic_ptr[10*negative_normal + 4] = 0.5;
      wp1 = (vic_ptr[10*negative_normal + 4] - 0.25);
      vic_ptr[10*negative_normal + 5] = -64*wp1*wp1*wp1*wp1 + 0.25;
      wp2 = (1 - vic_ptr[10*negative_normal + 4] - ccdParameters.gamma_1)/(1-ccdParameters.gamma_1);
      vic_ptr[10*negative_normal + 6] = wp2*wp2*wp2*wp2;
      vic_ptr[10*negative_normal + 7] = std::max((exp(-0.5*tmp_dis2.x*tmp_dis2.x/(sigma_hat*sigma_hat)) - exp(-ccdParameters.gamma_2)), 0.0);
      vic_ptr[ 10*negative_normal + 8] = 0.5*exp(-abs(tmp_dis2.x)/alpha)/alpha;
      vic_ptr[ 10*negative_normal + 9] = exp(-tmp_dis2.x*tmp_dis2.x/(2*sigma*sigma))/(sqrt(2*CV_PI)*sigma);
      normalized_param.at<double>(i, 1) += vic_ptr[ 10*negative_normal + 7];
    }
//    i++;
  }


#ifdef DEBUG
  printf("%-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s  %-5s\n",
         "x", "y", "dist_x", "dist_y", "a", "w1^4", "w2^4", "prox", "edf", "erf'"
         );
  for (int  i = 0; i < 20*floor(ccdParameters.h/ccdParameters.delta_h); ++i)
  {
    // std::cout << vic.at<double>(0,i) << "    ";
    printf("%-5f   ", vic.at<double>(0,i));
    if((i+1)%10 == 0)
      std::cout << std::endl;
  }
#endif

#pragma omp parallel for
  for (int i = 0; i < resolution; ++i)
  {
    int k = 0;
    // w1 = \sum wp_1, w2 = \sum wp_2
    double w1 =0.0 , w2 = 0.0;

    // store mean value near the curve
    std::vector<double> m1(3,0.0), m2(3,0.0);

    // store the second mean value near the curve
    std::vector<double> m1_o2(9,0.0), m2_o2(9,0.0);

    // compute local statistics

    // start search the points in the +n direction as well as -n direction
    double wp1 = 0.0, wp2 = 0.0;

    double *vic_ptr = vic.ptr<double>(i);
    double *mean_vic_ptr = mean_vic.ptr<double>(i);
    double *cov_vic_ptr = cov_vic.ptr<double>(i);
    for (int j = ccdParameters.delta_h; j <= ccdParameters.h; j += ccdParameters.delta_h, k++)
    {
      wp1 = 0.0, wp2 = 0.0;
      int negative_normal = k + (int)floor(ccdParameters.h/ccdParameters.delta_h);

      // wp1 = w(a_{k,l})*w(d_{k,l})*w(d)
      wp1 = vic_ptr[ 10*k+ 5]*vic_ptr[ 10*k+ 7]/normalized_param.at<double>(i,0);

      // wp2 = w(a_{k,l})*w(d_{k,l})*w(d)
      wp2 = vic_ptr[ 10*k+ 6]*vic_ptr[ 10*k+ 7]/normalized_param.at<double>(i,1);;

      //w1 = \sum{wp1}
      w1 += wp1;

      //w2 = \sum{wp2}
      w2 += wp2;

      // compute the mean value in the vicinity of a point
      // m_{ks} = I{k}^{s} = \sum_{l} w_{kls}{I_{kl}} : s = 1 or 2

      m1[0] += wp1*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[0];
      m1[1] += wp1*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[1];
      m1[2] += wp1*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[2];
      m2[0] += wp2*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[0];
      m2[1] += wp2*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[1];
      m2[2] += wp2*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[2];

      /*m1[0] += 0;
      m1[1] += 0;
      m1[2] += 0;
      m2[0] += 0;
      m2[1] += 0;
      m2[2] += 0;*/

      // compute second order local statistics
      // m_{k,s} = \sum_{l} w_{kls} I_{kl}*I_{kl}^T

      for (int m = 0; m < 3; ++m)
      {
        for (int n =0; n < 3; ++n)
        {
          m1_o2[m*3+n] += wp1*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[m]
                          *img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[n];
          m2_o2[m*3+n] += wp2*img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[m]
                          *img(vic_ptr[ 10*k + 0 ], vic_ptr[ 10*k + 1 ])[n];
        }
      }

      wp1 = vic_ptr[ 10*negative_normal+ 5]*vic_ptr[ 10*negative_normal+ 7]/normalized_param.at<double>(i,0);
      wp2 = vic_ptr[ 10*negative_normal+ 6]*vic_ptr[ 10*negative_normal+ 7]/normalized_param.at<double>(i,1);

      w1 += wp1;
      w2 += wp2;

      m1[0] += 0;
      m1[1] += 0;
      m1[2] += 0;
      m2[0] += 0;
      m2[1] += 0;
      m2[2] += 0;

      /*m1[0] += wp1*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[0];
      m1[1] += wp1*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[1];
      m1[2] += wp1*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[2];
      m2[0] += wp2*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[0];
      m2[1] += wp2*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[1];
      m2[2] += wp2*img(vic_ptr[10*negative_normal+0], vic_ptr[10*negative_normal+1])[2];*/

      for (int m = 0; m < 3; ++m)
      {
        for (int n =0; n < 3; ++n)
        {
          m1_o2[m*3+n] += wp1*img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[m]
                          *img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[n];
          m2_o2[m*3+n] += wp2*img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[m]
                          *img(vic_ptr[ 10*negative_normal + 0 ], vic_ptr[ 10*negative_normal + 1 ])[n];
        }
      }

      /*for (int m = 0; m < 3; ++m)
      {
        for (int n =0; n < 3; ++n)
        {
          m1_o2[m*3+n] += 0;
          m2_o2[m*3+n] += 0;
        }
      }*/

    }
    // std::cout << "w1: " << "              w2:" << std::endl;
    // std::cout << "w1 == " << w1 << "  w2== " << w2 << std::endl;
    mean_vic_ptr[0] = m1[0]/w1;
    mean_vic_ptr[1] = m1[1]/w1;
    mean_vic_ptr[2] = m1[2]/w1;

    /*mean_vic_ptr[0] = 0;
    mean_vic_ptr[1] = 0;
    mean_vic_ptr[2] = 0;*/

    mean_vic_ptr[3] = m2[0]/w2;
    mean_vic_ptr[4] = m2[1]/w2;
    mean_vic_ptr[5] = m2[2]/w2;

    mean_vic_ptr[3] = 0;
    mean_vic_ptr[4] = 0;
    mean_vic_ptr[5] = 0;

    for (int m = 0; m < 3; ++m)
    {
      for (int n = 0 ; n < 3; ++n)
      {
        cov_vic_ptr[ m*3+n] = m1_o2[m*3+n]/w1 -m1[m]*m1[n]/(w1*w1);
        cov_vic_ptr[ 9+m*3+n] = m2_o2[m*3+n]/w2 -m2[m]*m2[n]/(w2*w2);
        if(m == n)
        {
          cov_vic_ptr[ m*3+n] += ccdParameters.kappa;
          cov_vic_ptr[ 9+m*3+n] += ccdParameters.kappa;
        }
      }
    }
  }
  normalized_param.release();
}

void apCCDTracker::updateParameters(vpMatrix &LTCIL, vpColVector &LTCIR)
{
    cv::Mat_<cv::Vec3b>& img = (cv::Mat_<cv::Vec3b>&)image;

    int npointsccd = pointsCCD[scaleLevel].size();
    int normal_points_number = floor(ccdParameters.h/ccdParameters.delta_h);
    int nerror_ccd = 2*normal_points_number*3*npointsccd;
    error_ccd.resize(nerror_ccd);
    error0_ccd.resize(nerror_ccd);
    weighted_error_ccd.resize(nerror_ccd);

#pragma omp parallel
    {
        double _tmp_cov_data[9];
        cv::Mat tmp_cov(3,3,CV_64F,_tmp_cov_data);// = cv::Mat::zeros(3,3,CV_64F);
        double _tmp_cov_inv_data[9];
        cv::Mat tmp_cov_inv(3,3,CV_64F,_tmp_cov_inv_data);// = cv::Mat::zeros(3,3,CV_64F);
        std::vector<double> _tmp_jacobian_data(3 * ccdParameters.phi_dim);
        cv::Mat tmp_jacobian(ccdParameters.phi_dim,3,CV_64F,_tmp_jacobian_data.data());// = cv::Mat::zeros(ccdParameters.phi_dim,3,CV_64F);
        double _tmp_pixel_diff[3];
        cv::Mat tmp_pixel_diff(3,1,CV_64F,_tmp_pixel_diff);// = cv::Mat::zeros(3, 1, CV_64F);

        cv::Mat local_nabla_E = cv::Mat::zeros(nabla_E.rows, nabla_E.cols, CV_64F);
        cv::Mat local_hessian_E = cv::Mat::zeros(hessian_E.rows, hessian_E.cols, CV_64F);


        // std::cout << "dimension: " << Sigma_Phi.cols << " " << Sigma_Phi.rows << std::endl;
    //    int i =0;
#pragma omp for nowait
        for(int kk = 0 ; kk < pointsCCD[scaleLevel].size(); kk ++)
        {
            const int i = kk;
            apControlPoint *p = pointsCCD[scaleLevel][kk];

            double *vic_ptr = vic.ptr<double>(i);
            double *nv_ptr = nv.ptr<double>(i);
            double *mean_vic_ptr = mean_vic.ptr<double>(i);
            double *cov_vic_ptr = cov_vic.ptr<double>(i);
            double normal_points_number = floor(ccdParameters.h/ccdParameters.delta_h);


            for (int j = 0; j < 2*normal_points_number; ++j)
            {
                memset(tmp_cov.data, 0, 8 * sizeof(double));
                memset(tmp_cov_inv.data, 0, 9 * sizeof(double));
    //            tmp_cov = cv::Mat::zeros(3,3,CV_64F);
    //            tmp_cov_inv = cv::Mat::zeros(3,3,CV_64F);

                for (int m = 0; m < 3; ++m)
                {
                    double *tmp_cov_ptr = tmp_cov.ptr<double>(m);
                    for (int n = 0; n < 3; ++n)
                    {
                        tmp_cov_ptr[n] = vic_ptr[10*j+4] * cov_vic_ptr[m*3+n] +(1-vic_ptr[10*j+4])* cov_vic_ptr[m*3+n+9];
                    }
                }
                //if (j==0)
                tmp_cov_inv = tmp_cov.inv(cv::DECOMP_CHOLESKY);

                memset(tmp_pixel_diff.data, 0, 3 * sizeof(double));
    //            tmp_pixel_diff = cv::Mat::zeros(3, 1, CV_64F);

                //compute the difference between I_{kl} and \hat{I_{kl}}
                for (int m = 0; m < 3; ++m)
                {
                    tmp_pixel_diff.at<double>(m,0) = img(vic_ptr[10*j+0], vic_ptr[10*j+1])[m]- vic_ptr[10*j+4] * mean_vic_ptr[m]- (1-vic_ptr[10*j+4])* mean_vic_ptr[m+3];
                    //error_ccd[i*2*normal_points_number*3 + j*3 + m] = img(vic_ptr[10*j+0], vic_ptr[10*j+1])[m]- vic_ptr[10*j+4] * mean_vic_ptr[m]- (1-vic_ptr[10*j+4])* mean_vic_ptr[m+3];
                }

                //compute jacobian matrix
                memset(tmp_jacobian.data, 0, 3 * ccdParameters.phi_dim * sizeof(double));
    //            tmp_jacobian = cv::Mat::zeros(ccdParameters.phi_dim,3,CV_64F);

                for (int n = 0; n < 3; ++n)
                {
                    //tmp_pixel_diff.at<double>(n,0) = img(vic_ptr[10*j+0], vic_ptr[10*j+1])[n]- vic_ptr[10*j+4] * mean_vic_ptr[n]- (1-vic_ptr[10*j+4])* mean_vic_ptr[n+3];
                    tmp_jacobian.at<double>(0,n) = -cam.get_px()*vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*(-nv_ptr[0]/p->Zs);
                    tmp_jacobian.at<double>(1,n) = -cam.get_px()*vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*(-nv_ptr[1]/p->Zs);
                    tmp_jacobian.at<double>(2,n) = -cam.get_px()*vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*((nv_ptr[0]*p->xs+nv_ptr[1]*p->ys)/p->Zs);
                    tmp_jacobian.at<double>(3,n) = -cam.get_px()*vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*(nv_ptr[0]*p->xs*p->ys+nv_ptr[1]*(1+p->ys*p->ys));
                    tmp_jacobian.at<double>(4,n) = -cam.get_px()*vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*(-nv_ptr[1]*p->xs*p->ys-nv_ptr[0]*(1+p->xs*p->xs));
                    tmp_jacobian.at<double>(5,n) = -cam.get_px()*vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*(nv_ptr[0]*p->ys-nv_ptr[1]*p->xs);

                    if (ccdParameters.fixedrotationx == 1) tmp_jacobian.at<double>(3,n) = 0;
                    if (ccdParameters.fixedrotationy == 1) tmp_jacobian.at<double>(4,n) = 0;
                    if (ccdParameters.fixedrotationz == 1) tmp_jacobian.at<double>(5,n) = 0;

//tmp_jacobian.at<double>(5,n) = 0;
//tmp_jacobian.at<double>(4,n) = 0;
//tmp_jacobian.at<double>(3,n) = 0;

                    /*L_ccd[i*2*normal_points_number*3 + j*3 + n][0] = -955*vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*(-nv_ptr[0]/p->Zs);
              L_ccd[i*2*normal_points_number*3 + j*3 + n][1] = -955*vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*(-nv_ptr[1]/p->Zs);
              L_ccd[i*2*normal_points_number*3 + j*3 + n][2] = -955*vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*((nv_ptr[0]*p->xs+nv_ptr[1]*p->ys)/p->Zs);
              L_ccd[i*2*normal_points_number*3 + j*3 + n][3] = -955*vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*(nv_ptr[0]*p->xs*p->ys+nv_ptr[1]*(1+p->ys*p->ys));
              L_ccd[i*2*normal_points_number*3 + j*3 + n][4] = -955*vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*(-nv_ptr[1]*p->xs*p->ys-nv_ptr[0]*(1+p->xs*p->xs));
              L_ccd[i*2*normal_points_number*3 + j*3 + n][5] = -955*vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*(nv_ptr[0]*p->ys-nv_ptr[1]*p->xs);*
              /*if(ccdParameters.phi_dim == 8)
              {
                tmp_jacobian.at<double>(6,n) = vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*nv_ptr[0]*p->Zs;
                tmp_jacobian.at<double>(7,n) = vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*nv_ptr[1]*p->Zs;
              }*/
                    // std::cout << mean_vic_ptr[n] << " " << mean_vic_ptr[n+3] << std::endl;
                }
                const cv::Mat &tmp_jacobian_x_tmp_cov_inv = tmp_jacobian*tmp_cov_inv;
                //\nabla{E_2} = \sum J * \Sigma_{kl}^{-1} * (I_{kl} - \hat{I_{kl}})
                local_nabla_E += tmp_jacobian_x_tmp_cov_inv*tmp_pixel_diff;
                //Hessian{E_2} = \sum J * \Sigma_{kl}^{-1} * J
                local_hessian_E += tmp_jacobian_x_tmp_cov_inv*tmp_jacobian.t();
                /*double t1 = vpTime::measureTimeMs();
            std::cout << " time refine " << t1 - t0 << std::endl;*/
            }

    //        i++;
        }
#pragma omp critical
        {
            //\nabla{E_2} = \sum J * \Sigma_{kl}^{-1} * (I_{kl} - \hat{I_{kl}})
            nabla_E += local_nabla_E;
            //Hessian{E_2} = \sum J * \Sigma_{kl}^{-1} * J
            hessian_E += local_hessian_E;
        }
    }

    for(int ii = 0; ii<6; ii++)
  	  for(int jj = 0; jj <6; jj++)
  		  LTCIL[ii][jj] = hessian_E.at<double>(ii,jj);
        for(int jj = 0; jj <6; jj++)
        	LTCIR[jj] = nabla_E.at<double>(jj,0);

//    double t0= vpTime::measureTimeMs();
//    cv::Mat Sigma_Phi_inv = Sigma_Phi.inv(cv::DECOMP_CHOLESKY);
//    double t1= vpTime::measureTimeMs();
    //std::cout << " time chol " << t1 - t0 << std::endl;
    //hessian_E += Sigma_Phi_inv;
    //nabla_E += 2*Sigma_Phi_inv*Phi;
    cv::Mat hessian_E_inv = hessian_E.inv(cv::DECOMP_CHOLESKY);
    /*cv::Mat hessian_E_inv = hessian_E.inv(cv::DECOMP_SVD);
    for(int ii = 0; ii<6; ii++)
  	  for(int jj = 0; jj <6; jj++)
  		  Hessian_inv[6][6] = hessian_E_inv.at<double>(ii,jj);

    for(int jj = 0; jj <6; jj++)
    	      Nabla[jj] = nabla_E.at<double>(jj,0);*/


    //delta_Phi = hessian_E_inv*nabla_E;
    // #ifdef DEBUG
    // std::cout << delta_Phi.at<double>(0,0) << " "
    //           << delta_Phi.at<double>(1,0) << " "
    //           << delta_Phi.at<double>(2,0) << " "
    //           << delta_Phi.at<double>(3,0) << " "
    //           << delta_Phi.at<double>(4,0) << " "
    //           << delta_Phi.at<double>(5,0) << " ";
    // if(ccdParameters.phi_dim == 8)
    // {
    //   std::cout<< delta_Phi.at<double>(6,0) << " "
    //            << delta_Phi.at<double>(7,0) << " ";
    // }
    // std::cout<< std::endl;
    // #endif
    // cv::norm(delta_Phi);
    //std::cout << " phi1 " << delta_Phi.at<double>(0,0) << " phi2 " << delta_Phi.at<double>(1,0) << " phi3 " << delta_Phi.at<double>(2,0) << std::endl;
    //Phi -= delta_Phi;
    //Sigma_Phi = ccdParameters.c*Sigma_Phi + 2*(1-ccdParameters.c)*hessian_E_inv;

    Sigma_Phi = ccdParameters.c*Sigma_Phi + 2*(1-ccdParameters.c)*hessian_E_inv;

    //Sigma_Phi = /*Sigma_Phi +*/ hessian_E_inv;
    sigmaP.resize(6,6);
    for(int ii = 0; ii<6; ii++)
  	  for(int jj = 0; jj <6; jj++)
  		  sigmaP[ii][jj] = Sigma_Phi.at<double>(ii,jj);


//    Sigma_Phi_inv.release();
//    hessian_E_inv.release();
//    tmp_cov.release();
//    tmp_cov_inv.release();
//    tmp_jacobian.release();
//    tmp_pixel_diff.release();

}

void apCCDTracker::updateParametersPrev(vpMatrix &LTCIL, vpColVector &LTCIR)
{
    cv::Mat_<cv::Vec3b>& img = (cv::Mat_<cv::Vec3b>&)image;

    //vpColVector error_ccd;
    //vpColVector error0_ccd;
    //vpColVector weighted_error_ccd;
    //vpColVector w_ccd;
//    vpMatrix covariance;
//    vpMatrix covariance_inv;
    //vpMatrix L_ccd;
    int npointsccd = pointsCCD[scaleLevel].size();
    int normal_points_number = floor(ccdParameters.h/ccdParameters.delta_h);
    int nerror_ccd = 2*normal_points_number*3*npointsccd;
    error_ccd.resize(nerror_ccd);
    error0_ccd.resize(nerror_ccd);
    weighted_error_ccd.resize(nerror_ccd);
    std::vector<vpMatrix> cov_inv_vect;
    cov_inv_vect.resize(pointsCCD[scaleLevel].size() * 2 * normal_points_number);
    L_ccd.resize(nerror_ccd,6);
#pragma omp parallel
    {
        double _tmp_cov_data[9];
        cv::Mat tmp_cov(3,3,CV_64F,_tmp_cov_data);// = cv::Mat::zeros(3,3,CV_64F);
        double _tmp_cov_inv_data[9];
        cv::Mat tmp_cov_inv(3,3,CV_64F,_tmp_cov_inv_data);// = cv::Mat::zeros(3,3,CV_64F);
        std::vector<double> _tmp_jacobian_data(3 * ccdParameters.phi_dim);
        cv::Mat tmp_jacobian(ccdParameters.phi_dim,3,CV_64F,_tmp_jacobian_data.data());// = cv::Mat::zeros(ccdParameters.phi_dim,3,CV_64F);
        double _tmp_pixel_diff[3];
        cv::Mat tmp_pixel_diff(3,1,CV_64F,_tmp_pixel_diff);// = cv::Mat::zeros(3, 1, CV_64F);
        double _residu1[1];
        cv::Mat residu1(1,1,CV_64F,_residu1);// = cv::Mat::zeros(1,1,CV_64F);

        cv::Mat local_nabla_E = cv::Mat::zeros(nabla_E.rows, nabla_E.cols, CV_64F);
        cv::Mat local_hessian_E = cv::Mat::zeros(hessian_E.rows, hessian_E.cols, CV_64F);
        cv::Mat local_hessian_E_partial = cv::Mat::zeros(hessian_E_partial.rows, hessian_E_partial.cols, CV_64F);

        double px = cam.get_px();

            double normal_points_number = floor(ccdParameters.h/ccdParameters.delta_h);

            double lambda_prev = 0.0;
        // std::cout << "dimension: " << Sigma_Phi.cols << " " << Sigma_Phi.rows << std::endl;
    //    int i =0;
#pragma omp for nowait
        for(int kk = 0 ; kk < pointsCCD[scaleLevel].size(); kk ++)
        {
            const int i = kk;
            apControlPoint *p = pointsCCD[scaleLevel][kk];

            double *vic_ptr = vic.ptr<double>(i);
            double *nv_ptr = nv.ptr<double>(i);
            double *mean_vic_ptr = mean_vic.ptr<double>(i);
            double *cov_vic_ptr = cov_vic.ptr<double>(i);

            double *mean_vic_ptr_prev = mean_vic_prev.ptr<double>(i);
            double *cov_vic_ptr_prev = cov_vic_prev.ptr<double>(i);

	    double Zs = p->Zs;
	    double xs = p->xs;
	    double ys = p->ys;
	    double nvzs0 = nv_ptr[0]/Zs;
	    double nvzs1 = nv_ptr[1]/Zs;
	    double nvzs01 = (nv_ptr[0]*xs+nv_ptr[1]*ys)/Zs;

            for (int j = 0; j < 2*normal_points_number; ++j)
            {
                memset(tmp_cov.data, 0, 8 * sizeof(double));
                memset(tmp_cov_inv.data, 0, 9 * sizeof(double));
    //            tmp_cov = cv::Mat::zeros(3,3,CV_64F);
    //            tmp_cov_inv = cv::Mat::zeros(3,3,CV_64F);


                for (int m = 0; m < 3; ++m)
                {
                    double *tmp_cov_ptr = tmp_cov.ptr<double>(m);
                    for (int n = 0; n < 3; ++n)
                    {
                        tmp_cov_ptr[n] = vic_ptr[10*j+4] * cov_vic_ptr[m*3+n] +(1-vic_ptr[10*j+4])* cov_vic_ptr[m*3+n+9] + lambda_prev*lambda_prev*(vic_ptr[10*j+4] * cov_vic_ptr_prev[m*3+n] +(1-vic_ptr[10*j+4])* cov_vic_ptr_prev[m*3+n+9]);
                    }
                }
                //if (j==0)
                tmp_cov_inv = tmp_cov.inv(cv::DECOMP_CHOLESKY);

                memset(tmp_pixel_diff.data, 0, 3 * sizeof(double));
    //            tmp_pixel_diff = cv::Mat::zeros(3, 1, CV_64F);

                //compute the difference between I_{kl} and \hat{I_{kl}}
                for (int m = 0; m < 3; ++m)
                {
                    tmp_pixel_diff.at<double>(m,0) = img(vic_ptr[10*j+0], vic_ptr[10*j+1])[m]- vic_ptr[10*j+4] * mean_vic_ptr[m]- (1-vic_ptr[10*j+4])* mean_vic_ptr[m+3] + lambda_prev*(img(vic_ptr[10*j+0], vic_ptr[10*j+1])[m]- vic_ptr[10*j+4] * mean_vic_ptr_prev[m]- (1-vic_ptr[10*j+4])* mean_vic_ptr_prev[m+3]);
                    //error_ccd[i*2*normal_points_number*3 + j*3 + m] = img(vic_ptr[10*j+0], vic_ptr[10*j+1])[m]- vic_ptr[10*j+4] * mean_vic_ptr[m]- (1-vic_ptr[10*j+4])* mean_vic_ptr[m+3];
                }

                //compute jacobian matrix
                memset(tmp_jacobian.data, 0, 3 * ccdParameters.phi_dim * sizeof(double));
    //            tmp_jacobian = cv::Mat::zeros(ccdParameters.phi_dim,3,CV_64F);

                //residu1 = tmp_pixel_diff.t()*tmp_cov_inv*tmp_pixel_diff;
                //const double sqrt_residu1_00 = sqrt(residu1.at<double>(0,0));

                /*for (int m = 0; m < 3; ++m)
                {
                    //error_ccd[i*2*normal_points_number*3 + j*3 + m] = sqrt_residu1_00;
                    error_ccd[i*2*normal_points_number*3 + j*3 + m]	= tmp_pixel_diff.at<double>(m,0);
                }*/

                for (int n = 0; n < 3; ++n)
                {
                    //tmp_pixel_diff.at<double>(n,0) = img(vic_ptr[10*j+0], vic_ptr[10*j+1])[n]- vic_ptr[10*j+4] * mean_vic_ptr[n]- (1-vic_ptr[10*j+4])* mean_vic_ptr[n+3];
                    tmp_jacobian.at<double>(0,n) = -px*(vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*(-nvzs0) + lambda_prev*(vic_ptr[10*j + 9]*(mean_vic_ptr_prev[n] - mean_vic_ptr_prev[n+3])*(nvzs0)));
                    tmp_jacobian.at<double>(1,n) = -px*(vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*(-nvzs1) + lambda_prev*(vic_ptr[10*j + 9]*(mean_vic_ptr_prev[n] - mean_vic_ptr_prev[n+3])*(nvzs1)));
                    tmp_jacobian.at<double>(2,n) = -px*(vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*(nvzs01) + lambda_prev*(vic_ptr[10*j + 9]*(mean_vic_ptr_prev[n] - mean_vic_ptr_prev[n+3])*(nvzs01)));
                    tmp_jacobian.at<double>(3,n) = -cam.get_px()*(vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*(nv_ptr[0]*p->xs*p->ys+nv_ptr[1]*(1+p->ys*p->ys)) + lambda_prev*(vic_ptr[10*j + 9]*(mean_vic_ptr_prev[n] - mean_vic_ptr_prev[n+3])*(nv_ptr[0]*p->xs*p->ys+nv_ptr[1]*(1+p->ys*p->ys))));
                    tmp_jacobian.at<double>(4,n) = -cam.get_px()*(vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*(-nv_ptr[1]*p->xs*p->ys-nv_ptr[0]*(1+p->xs*p->xs)) + lambda_prev*(vic_ptr[10*j + 9]*(mean_vic_ptr_prev[n] - mean_vic_ptr_prev[n+3])*(-nv_ptr[1]*p->xs*p->ys-nv_ptr[0]*(1+p->xs*p->xs))) );
                    tmp_jacobian.at<double>(5,n) = -cam.get_px()*(vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*(nv_ptr[0]*p->ys-nv_ptr[1]*p->xs) + lambda_prev*(vic_ptr[10*j + 9]*(mean_vic_ptr_prev[n] - mean_vic_ptr_prev[n+3])*(nv_ptr[0]*p->ys-nv_ptr[1]*p->xs)));

/*tmp_jacobian.at<double>(5,n) = 0;
tmp_jacobian.at<double>(4,n) = 0;
tmp_jacobian.at<double>(3,n) = 0;*/


              /*L_ccd[i*2*normal_points_number*3 + j*3 + n][0] = tmp_jacobian.at<double>(0,n);
              L_ccd[i*2*normal_points_number*3 + j*3 + n][1] = tmp_jacobian.at<double>(1,n);
              L_ccd[i*2*normal_points_number*3 + j*3 + n][2] = tmp_jacobian.at<double>(2,n);
              L_ccd[i*2*normal_points_number*3 + j*3 + n][3] = tmp_jacobian.at<double>(3,n);
              L_ccd[i*2*normal_points_number*3 + j*3 + n][4] = tmp_jacobian.at<double>(4,n);
              L_ccd[i*2*normal_points_number*3 + j*3 + n][5] = tmp_jacobian.at<double>(5,n);*/
              /*if(ccdParameters.phi_dim == 8)
              {
                tmp_jacobian.at<double>(6,n) = vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*nv_ptr[0]*p->Zs;
                tmp_jacobian.at<double>(7,n) = vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3])*nv_ptr[1]*p->Zs;
              }*/
                    // std::cout << mean_vic_ptr[n] << " " << mean_vic_ptr[n+3] << std::endl;
                }
                const cv::Mat &tmp_jacobian_x_tmp_cov_inv = tmp_jacobian*tmp_cov_inv;
                //\nabla{E_2} = \sum J * \Sigma_{kl}^{-1} * (I_{kl} - \hat{I_{kl}})
                local_nabla_E += tmp_jacobian_x_tmp_cov_inv*tmp_pixel_diff;
                //Hessian{E_2} = \sum J * \Sigma_{kl}^{-1} * J
                local_hessian_E += tmp_jacobian_x_tmp_cov_inv*tmp_jacobian.t();
                /*double t1 = vpTime::measureTimeMs();
            std::cout << " time refine " << t1 - t0 << std::endl;*/
            }

    //        i++;
        }
#pragma omp critical
        {
            //\nabla{E_2} = \sum J * \Sigma_{kl}^{-1} * (I_{kl} - \hat{I_{kl}})
            nabla_E += local_nabla_E;
            //Hessian{E_2} = \sum J * \Sigma_{kl}^{-1} * J
            hessian_E += local_hessian_E;
        }
    }

    vpColVector v(6);
    //sigmaF = 0.2*sigmaF + 0.8*computeCovarianceMatrix(L_ccd,v,error_ccd);
    //std::cout << " sigmaF " <<  sigmaF << std::endl;


    for(int ii = 0; ii<6; ii++)
  	  for(int jj = 0; jj <6; jj++)
  		  LTCIL[ii][jj] = hessian_E.at<double>(ii,jj);
        for(int jj = 0; jj <6; jj++)
        	LTCIR[jj] = nabla_E.at<double>(jj,0);

    double t0= vpTime::measureTimeMs();
    cv::Mat Sigma_Phi_inv = Sigma_Phi.inv(cv::DECOMP_CHOLESKY);
    double t1= vpTime::measureTimeMs();
    std::cout << " time chol " << t1 - t0 << std::endl;
    hessian_E += Sigma_Phi_inv;
    nabla_E += 2*Sigma_Phi_inv*Phi;
    //cv::Mat hessian_E_inv = hessian_E.inv(cv::DECOMP_CHOLESKY);

    cv::Mat hessian_E_inv = hessian_E.inv(cv::DECOMP_SVD);


    /*for(int ii = 0; ii<6; ii++)
  	  for(int jj = 0; jj <6; jj++)
  		  Hessian_inv[6][6] = hessian_E_inv.at<double>(ii,jj);

    for(int jj = 0; jj <6; jj++)
    	      Nabla[jj] = nabla_E.at<double>(jj,0);*/


    //delta_Phi = hessian_E_inv*nabla_E;
    // #ifdef DEBUG
    // std::cout << delta_Phi.at<double>(0,0) << " "
    //           << delta_Phi.at<double>(1,0) << " "
    //           << delta_Phi.at<double>(2,0) << " "
    //           << delta_Phi.at<double>(3,0) << " "
    //           << delta_Phi.at<double>(4,0) << " "
    //           << delta_Phi.at<double>(5,0) << " ";
    // if(ccdParameters.phi_dim == 8)
    // {
    //   std::cout<< delta_Phi.at<double>(6,0) << " "
    //            << delta_Phi.at<double>(7,0) << " ";
    // }
    // std::cout<< std::endl;
    // #endif
    // cv::norm(delta_Phi);
    //std::cout << " phi1 " << delta_Phi.at<double>(0,0) << " phi2 " << delta_Phi.at<double>(1,0) << " phi3 " << delta_Phi.at<double>(2,0) << std::endl;
    //Phi -= delta_Phi;

    //Sigma_Phi = ccdParameters.c*Sigma_Phi + 2*(1-ccdParameters.c)*hessian_E_inv;

    //Sigma_Phi = /*Sigma_Phi +*/ 2*hessian_E_inv;
    /*sigmaP.resize(6,6);
    for(int ii = 0; ii<6; ii++)
  	  for(int jj = 0; jj <6; jj++)
  		  sigmaP[ii][jj] = Sigma_Phi.at<double>(ii,jj);*/


//    Sigma_Phi_inv.release();
//    hessian_E_inv.release();
//    tmp_cov.release();
//    tmp_cov_inv.release();
//    tmp_jacobian.release();
//    tmp_pixel_diff.release();

}

void apCCDTracker::updateParametersRobust(vpMatrix &LTCIL, vpColVector &LTCIR, vpRobust &robust)
{
    cv::Mat_<cv::Vec3b>& img = (cv::Mat_<cv::Vec3b>&)image;
//    cv::Mat tmp_cov_inv0 = cv::Mat::zeros(3,3,CV_64F);
    //cv::Mat tmp_jacobian = cv::Mat::zeros(ccdParameters.phi_dim,3,CV_64F);
    //vpColVector error_ccd;
    //vpColVector error0_ccd;
    //vpColVector weighted_error_ccd;
    //vpColVector w_ccd;
//    vpMatrix covariance;
//    vpMatrix covariance_inv;
    //vpMatrix L_ccd;
    int npointsccd = pointsCCD[scaleLevel].size();
    int normal_points_number = floor(ccdParameters.h/ccdParameters.delta_h);
    int nerror_ccd = 2*normal_points_number*3*npointsccd;
    error_ccd.resize(nerror_ccd);
    error0_ccd.resize(nerror_ccd);
    weighted_error_ccd.resize(nerror_ccd);
    //covariance.resize(nerror_ccd,nerror_ccd);
    //covariance_inv.resize(nerror_ccd,nerror_ccd);
    std::vector<vpMatrix> cov_inv_vect;
//    std::vector<double> double_inv_vect;
//    cov_inv_vect.resize(0);
    cov_inv_vect.resize(pointsCCD[scaleLevel].size() * 2 * normal_points_number);
//    double_inv_vect.resize(nerror_ccd/3);
    L_ccd.resize(nerror_ccd,6);
    //w_ccd.resize(nerror_ccd);

//    double t0 = vpTime::measureTimeMicros();

    // std::cout << "dimension: " << Sigma_Phi.cols << " " << Sigma_Phi.rows << std::endl;
//    int i =0;
//    int uu = 0;
#pragma omp parallel
    {
        double _tmp_pixel_diff[3];
        cv::Mat tmp_pixel_diff(3,1,CV_64F,_tmp_pixel_diff);// = cv::Mat::zeros(3, 1, CV_64F);
        double _residu1[1];
        cv::Mat residu1(1,1,CV_64F,_residu1);// = cv::Mat::zeros(1,1,CV_64F);
        double _tmp_cov[9];
        cv::Mat tmp_cov(3,3,CV_64F,_tmp_cov);// = cv::Mat::zeros(3,3,CV_64F);
        double _tmp_cov_inv[9];
        cv::Mat tmp_cov_inv(3,3,CV_64F,_tmp_cov_inv);// = cv::Mat::zeros(3,3,CV_64F);
        double _tmp_chol_cov_inv[9];
        cv::Mat tmp_chol_cov_inv(3,3,CV_64F,_tmp_chol_cov_inv);// = cv::Mat::zeros(3,3,CV_64F);
        double _residu[3];
        cv::Mat residu(3,1,CV_64F,_residu);// = cv::Mat::zeros(3, 1, CV_64F);
        vpMatrix cov_inv_(3,3);
#pragma omp for nowait
        for(int kk = 0 ; kk < pointsCCD[scaleLevel].size(); kk ++)
        {
            const int i = kk;
            apControlPoint *p = pointsCCD[scaleLevel][kk];

            double *vic_ptr = vic.ptr<double>(i);
            double *nv_ptr = nv.ptr<double>(i);
            double *mean_vic_ptr = mean_vic.ptr<double>(i);
            double *cov_vic_ptr = cov_vic.ptr<double>(i);

            for (int j = 0; j < 2*normal_points_number; ++j)
            {
                memset(tmp_cov.data, 0, 9 * sizeof(double));
                memset(tmp_cov_inv.data, 0, 9 * sizeof(double));
                memset(tmp_chol_cov_inv.data, 0, 9 * sizeof(double));
                memset(residu.data, 0, 3 * sizeof(double));

                const double vp10jp4 = vic_ptr[10*j+4];
                for (int m = 0; m < 3; ++m)
                {
                    double *tmp_cov_ptr = tmp_cov.ptr<double>(m);
                    for (int n = 0; n < 3; ++n)
                    {
                        tmp_cov_ptr[n] = vp10jp4 * cov_vic_ptr[m*3+n] + (1.0 - vp10jp4)* cov_vic_ptr[m*3+n+9];
                    }
                }
                tmp_cov_inv = tmp_cov.inv(cv::DECOMP_CHOLESKY);
                memset(tmp_pixel_diff.data, 0, 3 * sizeof(double));
//                tmp_pixel_diff = cv::Mat::zeros(3, 1, CV_64F);

                //compute the difference between I_{kl} and \hat{I_{kl}}
                for (int m = 0; m < 3; ++m)
                {
                    tmp_pixel_diff.at<double>(m,0) = img(vic_ptr[10*j+0], vic_ptr[10*j+1])[m]- vp10jp4 * mean_vic_ptr[m]- (1.0-vp10jp4)* mean_vic_ptr[m+3];
                    error0_ccd[i*2*normal_points_number*3 + j*3 + m] = img(vic_ptr[10*j+0], vic_ptr[10*j+1])[m]- vp10jp4 * mean_vic_ptr[m]- (1.0 - vp10jp4)* mean_vic_ptr[m+3];
                }

                residu1 = tmp_pixel_diff.t()*tmp_cov_inv*tmp_pixel_diff;
                const double sqrt_residu1_00 = sqrt(residu1.at<double>(0,0));
                for (int m = 0; m < 3; ++m)
                {
                    error_ccd[i*2*normal_points_number*3 + j*3 + m] = sqrt_residu1_00;
                    for (int n = 0; n < 3; ++n)
                        cov_inv_[m][n] = tmp_cov_inv.at<double>(m,n);
                }
                cov_inv_vect[i*2*normal_points_number + j] = cov_inv_;

                for (int n = 0; n < 3; ++n)
                {
                    double *ptr = L_ccd[i*2*normal_points_number*3 + j*3 + n];
                    const double f = -cam.get_px()*vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3]);
                    ptr[0] = f*(-nv_ptr[0]/p->Zs);
                    ptr[1] = f*(-nv_ptr[1]/p->Zs);
                    ptr[2] = f*((nv_ptr[0]*p->xs+nv_ptr[1]*p->ys)/p->Zs);
                    ptr[3] = f*(nv_ptr[0]*p->xs*p->ys+nv_ptr[1]*(1+p->ys*p->ys));
                    ptr[4] = f*(-nv_ptr[1]*p->xs*p->ys-nv_ptr[0]*(1+p->xs*p->xs));
                    ptr[5] = f*(nv_ptr[0]*p->ys-nv_ptr[1]*p->xs);
                }
            }
        }
    }

//    std::cout << "delta_t(0) = " << vpTime::measureTimeMicros() - t0 << " µs" << std::endl;
//    t0 = vpTime::measureTimeMicros();

    robust.MEstimator(vpRobust::TUKEY, error_ccd, w_ccd);
    //std::cout << error_ccd << std::endl;
    //w_ccd =1;
#pragma omp parallel for
    for (int k = 0; k < L_ccd.getRows() ; k++)
    {
    	for(int l =0; l < 6 ; l++)
    	{
    		L_ccd[l][k] = w_ccd[k]*L_ccd[l][k];
    	}

    	weighted_error_ccd[k] = w_ccd[k]*error0_ccd[k];
    }

//    std::cout << "delta_t(1) = " << vpTime::measureTimeMicros() - t0 << " µs" << std::endl;
//    t0 = vpTime::measureTimeMicros();
    vpMatrix CIL(L_ccd.getRows(),6);

//    double sum;
#pragma omp parallel for
    for (int k = 0; k<CIL.getRows(); k++)
    {
        const int k_3 = floor(k/3);
        const vpMatrix &cov_inv_ = cov_inv_vect[k_3];
    	for (int l = 0; l < 6 ; l++)
    	{
            double sum = 0;
    		for(int u=0; u < 3; u++){
         //{sum += tmp_cov_inv.at<double>(k-3*floor(k/3),u)*L_ccd[3*floor(k/3)+u][l];}
                sum += cov_inv_[k-3*k_3][u]*L_ccd[3*k_3+u][l];}
    		CIL[k][l] = sum;
    	}
    }
//    std::cout << "delta_t(2) = " << vpTime::measureTimeMicros() - t0 << " µs" << std::endl;
//    t0 = vpTime::measureTimeMicros();

    LTCIL.resize(L_ccd.getCols(), CIL.getCols(), false);
    const int ij_max= LTCIL.getRows() * LTCIL.getCols();
#pragma omp parallel for
    for(int ij = 0 ; ij < ij_max ; ++ij)
    {
        const int j = ij / LTCIL.getRows();
        const int i = ij - j * LTCIL.getRows();
        double v(0.0);
        const double *p0 = L_ccd[0] + i;
        const double *p1 = CIL[0] + j;
        for(int k = 0 ; k < CIL.getRows() ; ++k, p0 += L_ccd.getCols(), p1 += CIL.getCols())
            v += *p0 * *p1;
        LTCIL[i][j] = v;
    }
//    LTCIL = L_ccd.transpose()*CIL;
    //LTCIL = L_ccd.transpose()*L_ccd;
//    std::cout << "delta_t(3) = " << vpTime::measureTimeMicros() - t0 << " µs" << std::endl;
//    t0 = vpTime::measureTimeMicros();
    const vpMatrix _CIL = CIL;
    const vpColVector _weighted_error_ccd = weighted_error_ccd;

    vpMatrix LTCIR_;
    computeJTR(_CIL,_weighted_error_ccd,LTCIR_);
    LTCIR = LTCIR_;
//    std::cout << "delta_t(4) = " << vpTime::measureTimeMicros() - t0 << " µs" << std::endl;

    //LTCIL = L_ccd.transpose()*covariance_inv*L_ccd;
    //std::cout << " LTCIR "  << LTCIR << std::endl;
//    double t0= vpTime::measureTimeMs();
//    cv::Mat Sigma_Phi_inv;// = Sigma_Phi.inv(cv::DECOMP_CHOLESKY);
//    double t1= vpTime::measureTimeMs();
    //hessian_E += Sigma_Phi_inv;
    //nabla_E += 2*Sigma_Phi_inv*Phi;
//    cv::Mat hessian_E_inv;// = hessian_E.inv(cv::DECOMP_CHOLESKY);
    /*cv::Mat hessian_E_inv = hessian_E.inv(cv::DECOMP_SVD);
    for(int ii = 0; ii<6; ii++)
  	  for(int jj = 0; jj <6; jj++)
  		  Hessian_inv[6][6] = hessian_E_inv.at<double>(ii,jj);

    for(int jj = 0; jj <6; jj++)
    	      Nabla[jj] = nabla_E.at<double>(jj,0);*/


    //delta_Phi = hessian_E_inv*nabla_E;
    // #ifdef DEBUG
    // std::cout << delta_Phi.at<double>(0,0) << " "
    //           << delta_Phi.at<double>(1,0) << " "
    //           << delta_Phi.at<double>(2,0) << " "
    //           << delta_Phi.at<double>(3,0) << " "
    //           << delta_Phi.at<double>(4,0) << " "
    //           << delta_Phi.at<double>(5,0) << " ";
    // if(ccdParameters.phi_dim == 8)
    // {
    //   std::cout<< delta_Phi.at<double>(6,0) << " "
    //            << delta_Phi.at<double>(7,0) << " ";
    // }
    // std::cout<< std::endl;
    // #endif
    // cv::norm(delta_Phi);
    //std::cout << " phi1 " << delta_Phi.at<double>(0,0) << " phi2 " << delta_Phi.at<double>(1,0) << " phi3 " << delta_Phi.at<double>(2,0) << std::endl;
    //Phi -= delta_Phi;
    //Sigma_Phi = ccdParameters.c*Sigma_Phi + 2*(1-ccdParameters.c)*hessian_E_inv;

//    Sigma_Phi_inv.release();
//    hessian_E_inv.release();
//    tmp_cov.release();
//    tmp_cov_inv.release();
//    //tmp_jacobian.release();
//    tmp_pixel_diff.release();
//    residu1.release();
//    residu.release();
//    tmp_cov_inv0.release();
//    tmp_chol_cov_inv.release();

}


void apCCDTracker::updateParametersRobustPrev(vpMatrix &LTCIL, vpColVector &LTCIR, vpRobust &robust)
{
    cv::Mat_<cv::Vec3b>& img = (cv::Mat_<cv::Vec3b>&)image;
//    cv::Mat tmp_cov_inv0 = cv::Mat::zeros(3,3,CV_64F);
    //cv::Mat tmp_jacobian = cv::Mat::zeros(ccdParameters.phi_dim,3,CV_64F);
    //vpColVector error_ccd;
    //vpColVector error0_ccd;
    //vpColVector weighted_error_ccd;
    //vpColVector w_ccd;
//    vpMatrix covariance;
//    vpMatrix covariance_inv;
    //vpMatrix L_ccd;
    int npointsccd = pointsCCD[scaleLevel].size();
    int normal_points_number = floor(ccdParameters.h/ccdParameters.delta_h);
    int nerror_ccd = 2*normal_points_number*3*npointsccd;
    error_ccd.resize(nerror_ccd);
    vpColVector error2(npointsccd);
    error2 = 0;
    error0_ccd.resize(nerror_ccd);
    weighted_error_ccd.resize(nerror_ccd);
    //covariance.resize(nerror_ccd,nerror_ccd);
    //covariance_inv.resize(nerror_ccd,nerror_ccd);
    std::vector<vpMatrix> cov_inv_vect;
    cov_inv_vect.resize(pointsCCD[scaleLevel].size() * 2 * normal_points_number);
//    std::vector<double> double_inv_vect;
//    cov_inv_vect.resize(0);

//    double_inv_vect.resize(nerror_ccd/3);
    L_ccd.resize(nerror_ccd,6);
    w_ccd.resize(resolution);
    w_ccd = 1;

    int npointsccd_p = (int)npointsccd/10;
    int nerror_ccd_p = 2*normal_points_number*3*npointsccd_p;

    L_ccd_partial.resize(nerror_ccd_p,6);
    std::vector<int> error_ccd_vect;
    std::vector<int> idx;
    idx.resize(npointsccd);
    error_ccd_vect.resize(npointsccd);
    for (int i = 0; i != idx.size(); ++i) idx[i] = i;

    std::vector<vpMatrix> cov_inv_vect_partial;
    cov_inv_vect_partial.resize(nerror_ccd_p * 2 * normal_points_number);


//    double t0 = vpTime::measureTimeMicros();

    // std::cout << "dimension: " << Sigma_Phi.cols << " " << Sigma_Phi.rows << std::endl;
//    int i =0;
//    int uu = 0;
//#pragma omp parallel
    {
        double _tmp_pixel_diff[3];
        cv::Mat tmp_pixel_diff(3,1,CV_64F,_tmp_pixel_diff);// = cv::Mat::zeros(3, 1, CV_64F);
        double _residu1[1];
        cv::Mat residu1(1,1,CV_64F,_residu1);// = cv::Mat::zeros(1,1,CV_64F);
        double _tmp_cov[9];
        cv::Mat tmp_cov(3,3,CV_64F,_tmp_cov);// = cv::Mat::zeros(3,3,CV_64F);
        double _tmp_cov_inv[9];
        cv::Mat tmp_cov_inv(3,3,CV_64F,_tmp_cov_inv);// = cv::Mat::zeros(3,3,CV_64F);
        double _tmp_chol_cov_inv[9];
        cv::Mat tmp_chol_cov_inv(3,3,CV_64F,_tmp_chol_cov_inv);// = cv::Mat::zeros(3,3,CV_64F);
        double _residu[3];
        cv::Mat residu(3,1,CV_64F,_residu);// = cv::Mat::zeros(3, 1, CV_64F);
        vpMatrix cov_inv_(3,3);
        double lambda_prev = 0.1;
//#pragma omp for nowait
        for(int kk = 0 ; kk < pointsCCD[scaleLevel].size(); kk ++)
        {
            const int i = kk;
            apControlPoint *p = pointsCCD[scaleLevel][kk];

            double *vic_ptr = vic.ptr<double>(i);
            double *nv_ptr = nv.ptr<double>(i);
            double *mean_vic_ptr = mean_vic.ptr<double>(i);
            double *cov_vic_ptr = cov_vic.ptr<double>(i);

            double *mean_vic_ptr_prev = mean_vic_prev.ptr<double>(i);
            double *cov_vic_ptr_prev = cov_vic_prev.ptr<double>(i);

            for (int j = 0; j < 2*normal_points_number; ++j)
            {
                memset(tmp_cov.data, 0, 9 * sizeof(double));
                memset(tmp_cov_inv.data, 0, 9 * sizeof(double));
                memset(tmp_chol_cov_inv.data, 0, 9 * sizeof(double));
                memset(residu.data, 0, 3 * sizeof(double));

                const double vp10jp4 = vic_ptr[10*j+4];

                for (int m = 0; m < 3; ++m)
                {
                    double *tmp_cov_ptr = tmp_cov.ptr<double>(m);
                    for (int n = 0; n < 3; ++n)
                    {
                        tmp_cov_ptr[n] = vp10jp4 * cov_vic_ptr[m*3+n] + (1.0 - vp10jp4)* cov_vic_ptr[m*3+n+9] + lambda_prev*lambda_prev*(vp10jp4 * cov_vic_ptr_prev[m*3+n] + (1.0 - vp10jp4)* cov_vic_ptr_prev[m*3+n+9]);
                    }
                }
                tmp_cov_inv = tmp_cov.inv(cv::DECOMP_CHOLESKY);
                memset(tmp_pixel_diff.data, 0, 3 * sizeof(double));
//                tmp_pixel_diff = cv::Mat::zeros(3, 1, CV_64F);

                //compute the difference between I_{kl} and \hat{I_{kl}}
                for (int m = 0; m < 3; ++m)
                {
                    tmp_pixel_diff.at<double>(m,0) = img(vic_ptr[10*j+0], vic_ptr[10*j+1])[m]- vp10jp4 * mean_vic_ptr[m]- (1.0-vp10jp4)* mean_vic_ptr[m+3] + lambda_prev*(img(vic_ptr[10*j+0], vic_ptr[10*j+1])[m]- vp10jp4 * mean_vic_ptr_prev[m]- (1.0-vp10jp4)* mean_vic_ptr_prev[m+3]);
                    //std::cout << " current " << img(vic_ptr[10*j+0], vic_ptr[10*j+1])[m]- vp10jp4 * mean_vic_ptr[m]- (1.0-vp10jp4)* mean_vic_ptr[m+3] << " prev " << img(vic_ptr[10*j+0], vic_ptr[10*j+1])[m]- vp10jp4 * mean_vic_ptr_prev[m]- (1.0-vp10jp4)* mean_vic_ptr_prev[m+3] << std::endl;
                    error0_ccd[i*2*normal_points_number*3 + j*3 + m] = tmp_pixel_diff.at<double>(m,0);
                }

                residu1 = tmp_pixel_diff.t()*tmp_cov_inv*tmp_pixel_diff;
                const double sqrt_residu1_00 = sqrt(residu1.at<double>(0,0));
                for (int m = 0; m < 3; ++m)
                {
                    error_ccd[i*2*normal_points_number*3 + j*3 + m] = sqrt_residu1_00;
                    //error2[i] += sqrt_residu1_00;
                    //error_ccd_vect[i*2*normal_points_number*3 + j*3 + m] = sqrt_residu1_00;
                    error_ccd_vect[i] += sqrt_residu1_00;
                    error2[i] += residu1.at<double>(0,0);
                    for (int n = 0; n < 3; ++n)
                        cov_inv_[m][n] = tmp_cov_inv.at<double>(m,n);
                }
                cov_inv_vect[i*2*normal_points_number + j] = cov_inv_;

                for (int n = 0; n < 3; ++n)
                {
                    double *ptr = L_ccd[i*2*normal_points_number*3 + j*3 + n];
                    const double f = -cam.get_px()*vic_ptr[10*j + 9]*(mean_vic_ptr[n] - mean_vic_ptr[n+3]);
                    const double f_prev = -lambda_prev*cam.get_px()*vic_ptr[10*j + 9]*(mean_vic_ptr_prev[n] - mean_vic_ptr_prev[n+3]);
                    ptr[0] = f*(-nv_ptr[0]/p->Zs) + f_prev*(-nv_ptr[0]/p->Zs);
                    ptr[1] = f*(-nv_ptr[1]/p->Zs) +  f_prev*(-nv_ptr[1]/p->Zs);
                    ptr[2] = f*((nv_ptr[0]*p->xs+nv_ptr[1]*p->ys)/p->Zs) + f_prev*((nv_ptr[0]*p->xs+nv_ptr[1]*p->ys)/p->Zs);
                    ptr[3] = f*(nv_ptr[0]*p->xs*p->ys+nv_ptr[1]*(1+p->ys*p->ys)) + f_prev*(nv_ptr[0]*p->xs*p->ys+nv_ptr[1]*(1+p->ys*p->ys));
                    ptr[4] = f*(-nv_ptr[1]*p->xs*p->ys-nv_ptr[0]*(1+p->xs*p->xs)) + f_prev*(-nv_ptr[1]*p->xs*p->ys-nv_ptr[0]*(1+p->xs*p->xs));
                    ptr[5] = f*(nv_ptr[0]*p->ys-nv_ptr[1]*p->xs) + f_prev*(nv_ptr[0]*p->ys-nv_ptr[1]*p->xs);
                }

            }
            error2[i] = sqrt(error2[i]);
        }
    }


    // sort indexes based on comparing values in v
    std::sort(idx.begin(), idx.end(),[&error_ccd_vect](int i1, int i2) {return error_ccd_vect[i1] > error_ccd_vect[i2];});


//    std::cout << "delta_t(0) = " << vpTime::measureTimeMicros() - t0 << " µs" << std::endl;
//    t0 = vpTime::measureTimeMicros();

    //error2 = error_ccd/10.0;
    //robust.MEstimator(vpRobust::TUKEY, error_ccd, w_ccd);
    robust.MEstimator(vpRobust::TUKEY, error2, w_ccd);
    //std::cout << " w_ccd " << w_ccd << std::endl;
    //w_ccd =1;
//#pragma omp parallel for
    for(int kk = 0 ; kk < pointsCCD[scaleLevel].size(); kk ++)
      {
    	for (int j = 0; j < 2*normal_points_number; ++j)
    	            {
    		for (int m = 0; m < 3; ++m)
    		                {
    			/*for(int l =0; l < 6 ; l++)
    			    	{
    			    		//if(w_ccd[k] ==0) printf("ok");
    				L_ccd[l][kk*2*normal_points_number*3 + j*3 + m] = L_ccd[l][kk*2*normal_points_number*3 + j*3 + m];
    			    	}*/
    		        weighted_error_ccd[kk*2*normal_points_number*3 + j*3 + m] = (w_ccd[kk])*(w_ccd[kk])*error0_ccd[kk*2*normal_points_number*3 + j*3 + m];
    		                }
    	            }
      }

    /*for(int kk = 0 ; kk < npointsccd_p; kk ++)
      {
    	for (int j = 0; j < 2*normal_points_number; ++j)
    	            {
    		for (int m = 0; m < 3; ++m)
    		                {
    			for(int l =0; l < 6 ; l++)
		         {
    				L_ccd_partial[l][kk*2*normal_points_number*3 + j*3 + m] = L_ccd[l][idx[kk]*2*normal_points_number*3 + j*3 + m];
		         }
    		                }
    		cov_inv_vect_partial[kk*2*normal_points_number + j] = cov_inv_vect[idx[kk]*2*normal_points_number + j];
      }
      }*/

    /*for (int k = 0; k < L_ccd.getRows() ; k++)
    {
    	for(int l =0; l < 6 ; l++)
    	{
    		if(w_ccd[k] ==0) printf("ok");
    		L_ccd[l][k] = w_ccd[k]*L_ccd[l][k];
    	}

    	weighted_error_ccd[k] = w_ccd[k]*error0_ccd[k];
    }*/

//    std::cout << "delta_t(1) = " << vpTime::measureTimeMicros() - t0 << " µs" << std::endl;
//    t0 = vpTime::measureTimeMicros();
    vpMatrix CIL(L_ccd.getRows(),6);

//    double sum;
//#pragma omp parallel for
    for (int k = 0; k<CIL.getRows(); k++)
    {
        const int k_3 = floor(k/3);
        const vpMatrix &cov_inv_ = cov_inv_vect[k_3];
    	for (int l = 0; l < 6 ; l++)
    	{
            double sum = 0;
    		for(int u=0; u < 3; u++){
         //{sum += tmp_cov_inv.at<double>(k-3*floor(k/3),u)*L_ccd[3*floor(k/3)+u][l];}
                sum += cov_inv_[k-3*k_3][u]*L_ccd[3*k_3+u][l];}
    		CIL[k][l] = sum;
    	}
    }
//    std::cout << "delta_t(2) = " << vpTime::measureTimeMicros() - t0 << " µs" << std::endl;
//    t0 = vpTime::measureTimeMicros();

    LTCIL.resize(L_ccd.getCols(), CIL.getCols(), false);
    const int ij_max= LTCIL.getRows() * LTCIL.getCols();
//#pragma omp parallel for
    for(int ij = 0 ; ij < ij_max ; ++ij)
    {
        const int j = ij / LTCIL.getRows();
        const int i = ij - j * LTCIL.getRows();
        double v(0.0);
        const double *p0 = L_ccd[0] + i;
        const double *p1 = CIL[0] + j;
        for(int k = 0 ; k < CIL.getRows() ; ++k, p0 += L_ccd.getCols(), p1 += CIL.getCols())
            v += *p0 * *p1;
        LTCIL[i][j] = v;
    }

/*
    vpMatrix CIL_partial(L_ccd_partial.getRows(),6);

 //    double sum;
 //#pragma omp parallel for
     for (int k = 0; k<CIL_partial.getRows(); k++)
     {
         const int k_3 = floor(k/3);
         const vpMatrix &cov_inv_ = cov_inv_vect_partial[k_3];
     	for (int l = 0; l < 6 ; l++)
     	{
             double sum = 0;
     		for(int u=0; u < 3; u++){
          //{sum += tmp_cov_inv.at<double>(k-3*floor(k/3),u)*L_ccd[3*floor(k/3)+u][l];}
                 sum += cov_inv_[k-3*k_3][u]*L_ccd_partial[3*k_3+u][l];}
     		CIL_partial[k][l] = sum;
     	}
     }
 //    std::cout << "delta_t(2) = " << vpTime::measureTimeMicros() - t0 << " µs" << std::endl;
 //    t0 = vpTime::measureTimeMicros();
     vpMatrix LTCIL_partial;

     LTCIL_partial.resize(L_ccd_partial.getCols(), CIL_partial.getCols(), false);
     const int ij_max_p= LTCIL_partial.getRows() * LTCIL_partial.getCols();
 //#pragma omp parallel for
     for(int ij = 0 ; ij < ij_max_p ; ++ij)
     {
         const int j = ij / LTCIL_partial.getRows();
         const int i = ij - j * LTCIL_partial.getRows();
         double v(0.0);
         const double *p0 = L_ccd_partial[0] + i;
         const double *p1 = CIL_partial[0] + j;
         for(int k = 0 ; k < CIL_partial.getRows() ; ++k, p0 += L_ccd_partial.getCols(), p1 += CIL_partial.getCols())
             v += *p0 * *p1;
         LTCIL_partial[i][j] = v;
     }
     */
//    LTCIL = L_ccd.transpose()*CIL;
    //LTCIL = L_ccd.transpose()*L_ccd;
//    std::cout << "delta_t(3) = " << vpTime::measureTimeMicros() - t0 << " µs" << std::endl;
//    t0 = vpTime::measureTimeMicros();
    const vpMatrix _CIL = CIL;
    const vpColVector _weighted_error_ccd = weighted_error_ccd;

    vpMatrix LTCIR_;
    computeJTR(_CIL,_weighted_error_ccd,LTCIR_);
    LTCIR = LTCIR_;
//    std::cout << "delta_t(4) = " << vpTime::measureTimeMicros() - t0 << " µs" << std::endl;

    //LTCIL = L_ccd.transpose()*covariance_inv*L_ccd;
    //std::cout << " LTCIR "  << LTCIR << std::endl;
//    double t0= vpTime::measureTimeMs();
//    cv::Mat Sigma_Phi_inv;// = Sigma_Phi.inv(cv::DECOMP_CHOLESKY);
//    double t1= vpTime::measureTimeMs();
    //hessian_E += Sigma_Phi_inv;
    //nabla_E += 2*Sigma_Phi_inv*Phi;
    //cv::Mat hessian_E_inv = hessian_E.inv(cv::DECOMP_CHOLESKY);
    /*cv::Mat hessian_E_inv = hessian_E.inv(cv::DECOMP_SVD);
    for(int ii = 0; ii<6; ii++)
  	  for(int jj = 0; jj <6; jj++)
  		  Hessian_inv[6][6] = hessian_E_inv.at<double>(ii,jj);

    for(int jj = 0; jj <6; jj++)
    	      Nabla[jj] = nabla_E.at<double>(jj,0);*/


    //delta_Phi = hessian_E_inv*nabla_E;
    // #ifdef DEBUG
    // std::cout << delta_Phi.at<double>(0,0) << " "
    //           << delta_Phi.at<double>(1,0) << " "
    //           << delta_Phi.at<double>(2,0) << " "
    //           << delta_Phi.at<double>(3,0) << " "
    //           << delta_Phi.at<double>(4,0) << " "
    //           << delta_Phi.at<double>(5,0) << " ";
    // if(ccdParameters.phi_dim == 8)
    // {
    //   std::cout<< delta_Phi.at<double>(6,0) << " "
    //            << delta_Phi.at<double>(7,0) << " ";
    // }
    // std::cout<< std::endl;
    // #endif
    // cv::norm(delta_Phi);
    //std::cout << " phi1 " << delta_Phi.at<double>(0,0) << " phi2 " << delta_Phi.at<double>(1,0) << " phi3 " << delta_Phi.at<double>(2,0) << std::endl;
    //Phi -= delta_Phi;
    //Sigma_Phi = ccdParameters.c*Sigma_Phi + 2*(1-ccdParameters.c)*hessian_E_inv;

    /*sigmaP.resize(6,6);
    for(int ii = 0; ii<6; ii++)
  	  for(int jj = 0; jj <6; jj++)
  		  sigmaP[ii][jj] = Sigma_Phi.at<double>(ii,jj);*/


    //sigmaP = ccdParameters.c*sigmaP + 2*(1-ccdParameters.c)*(LTCIL.pseudoInverse(LTCIL.getRows()* DBL_EPSILON));

    sigmaP = ccdParameters.c*sigmaP + 2*(1-ccdParameters.c)*(LTCIL.pseudoInverse(LTCIL.getRows()* DBL_EPSILON));



//    Sigma_Phi_inv.release();
//    hessian_E_inv.release();
//    tmp_cov.release();
//    tmp_cov_inv.release();
//    //tmp_jacobian.release();
//    tmp_pixel_diff.release();
//    residu1.release();
//    residu.release();
//    tmp_cov_inv0.release();
//    tmp_chol_cov_inv.release();

}

void apCCDTracker::checkCCDConvergence()
{
    if( tol < 1 && ccdParameters.h > 10)
    {
      ccdParameters.h = ccdParameters.h/sqrt(2);
    }

    if( tol < 0.001)
    {
      convergence = true;
    }
}

void apCCDTracker::choleskyDecomposition(cv::Mat &A, cv::Mat &L,int n)
{
    //int n = A.getRows();
    double sum1 = 0.0;
    double sum2 = 0.0;
    double sum3 = 0.0;
    //vector<vector<double> > l(n, vector<double> (n));
    L.at<double>(0,0) = sqrt(A.at<double>(0,0));
    for (int j = 1; j <= n-1; j++)
    L.at<double>(j,0) = A.at<double>(j,0)/L.at<double>(0,0);
    for (int i = 1; i <= (n-2); i++)
    {
    for (int k = 0; k <= (i-1); k++)
    sum1 += pow(L.at<double>(i,k), 2);
    L.at<double>(i,i)= sqrt(A.at<double>(i,i)-sum1);
    for (int j = (i+1); j <= (n-1); j++)
    {
    for (int k = 0; k <= (i-1); k++)
    sum2 += L.at<double>(j,k)*L.at<double>(i,k);
    L.at<double>(j,i) = (A.at<double>(j,i)-sum2)/L.at<double>(i,i);
    }
    }
    for (int k = 0; k <= (n-2); k++)
    sum3 += pow(L.at<double>(n-1,k), 2);
    L.at<double>(n-1,n-1) = sqrt(A.at<double>(n-1,n-1)-sum3);
}

vpMatrix apCCDTracker::computeCovarianceMatrix(const vpMatrix &A,
		const vpColVector &x, const vpColVector &b) {
	//double sigma2 = ((b.t()) * b)/((double)b.getRows()) ;//- ((b.t()) * A * x);
	double sigma2 = 70*70;
	//std::cout << " error mean CCD " << sqrt((((b).t()) * b)/((double)b.getRows())) << std::endl;
	//std::cout << " sigma 2 " << sigma2 << std::endl;
	return (A.t() * A).pseudoInverse() * sigma2;
}
