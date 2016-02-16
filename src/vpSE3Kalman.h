#ifndef __VP_SE3_KALMAN__
#define __VP_SE3_KALMAN__

#include <visp/vpSubMatrix.h>
#include <visp/vpSubColVector.h>
#include <visp/vpSubRowVector.h>
#include <visp/vpThetaUVector.h>

#define KDEBUG 1

/*!
  \file vpSE3Kalman.h

  \brief Definition of the vpSL3Kalman class
*/

/*!
  \class vpSE3Kalman
  \ingroup TrackingAlignment
  \brief Definition of the vpSE3Kalman class
  vpSL3Kalman class provides a Kalman filter in SE3 group where a 
  probabilistic camera position, orientation and kinematic torsor is estimated
  If he is associated with a SE3 alignment minimization algorithm
  The SE3 alignment minimization is initialized at each step 
  and provides a observation for Kalman filter

  \author Jean Laneurit (IRISA - INRIA Rennes)

  \sa vpImage, vpImagePoint, vpMatrix, vpColVector, vpRowVector, vpTrianglePatch, vpPolygonPatch, vpWarp, vpAlignmentMinimization
*/

class vpSE3Kalman{
 	protected:
	/** state */
	vpColVector _X;
	/** covariance */
	vpMatrix _Q;
	/** temporary state  */
	vpColVector _Xtmp;
	/** temporary covariance */
	vpMatrix _Qtmp;


	/** ThetaU sub-vector in X */
	vpSubColVector _P;
	/** ThetaU sub-vector in X */
	vpSubColVector _Ou;
	/** Velocities sub-vector in X */
	vpSubColVector _V;
	/** Rotation velovities sub-vector in X */
	vpSubColVector _W;
	/** Orientation  + P */
	vpSubColVector _POu;
	/** Covariance of position and orientation parameters, sub matrix in Q*/
	vpSubMatrix _QPOu;



	/** Prediction */
	/** Input covariance */
	vpMatrix _Qu;
	/** Input covariance */
	vpSubMatrix _Ju;
	/** Jacobian for dynamic equation */
	vpMatrix _Jx;

	/** R(w *dt) = exp(skew(w)*dt) */
	vpSubMatrix  _RwDt; 
	/** R(w *dt) = d exp(skew(w)*dt) /d w*dt */
	vpMatrix  _dRwDt_dwDt; 
	/** d Ou / d w*/
	vpSubMatrix _dOu_dw;

	/** R(Ou)=exp(skew(Ou)) */
	vpMatrix  _ROu;
	/** _dSk2Ou_dOu =  d(skew(Ou)*skew(Ou))/d Ou */
	vpMatrix  _dROu_dOu;
	/** _dOu_dOu = d Psi(exp(skew(w*dt))*exp(skew(Ou))) d Ou */
	vpSubMatrix  _dOu_dOu;

	/** S(w*dt)*dt=exp(skew(w*dt)) */
	vpMatrix  _SwDt;
	/** S(w*dt)*dt=exp(skew(w*dt)) *dt*/
	vpSubMatrix  _SwDtDt; 
	/** d exp(skew(w*dt)) / d wdt  */
	vpMatrix _dSwDt_dwDt;
	/** d exp(skew(w*dt)) / d wdt *Dt  */
	vpSubMatrix _dSwDt_dwDtDt;

	/** d P / d w*/
	vpMatrix _dP_dw;


	/** a vector */
	vpColVector _Vec;
	/**  SkOu = skew(Vec)  */
	vpMatrix  _SkVec; 
	/**  Sk2Ou = skew(Vec)skew(Vec)  */
	vpMatrix  _Sk2Vec; 
	/**  SSkV = stack(skew(Vec))  */
	vpColVector  _SSkVec; 
	/**  SSk2V = stack(skew(Vec)skew(Vec))  */
	vpColVector  _SSk2Vec; 
	/**  dSkV_dV = d skew(Vec) /dV  */
	vpMatrix  _dSSkVec_dVec; 
	/**  dSk2V_dV = d skew(Vec)skew(Vec) /dV  */
	vpMatrix  _dSSk2Vec_dVec;

	
	vpColVector _A;
	vpMatrix _dA_dSR;
	vpRowVector _dCs_dSR;
	vpMatrix _dPsi_dSR;
	vpRotationMatrix _RwDtROu;


	/** Observations */
	/** vision observation */
	vpColVector _Xv;
	/** vision covraiance */
	vpMatrix _Qv;
	/** jacobian for vision observation matrix */
	vpMatrix _Hv;
	/** Kalman gain matrix for vision information */
	vpMatrix _Kv;
	/** Innovation covariance for vision information */
	vpMatrix _IQv;
	/** Innovation vector for vision information */
	vpColVector _Iv;
	/** Ou vector incuded in vision observation */
	vpSubColVector _Ouv;	
	/** Mahalanobis distance */
	vpMatrix _D;


	/** buffers  */
 	vpColVector _buffer31;//(size 3x1)
 	vpRowVector _buffer13;//(size 1x3)
	vpMatrix _buffer39;//(size 3x9)
 	vpMatrix _buffer93a,_buffer93b;//(size 9x3)
 	vpMatrix _buffer99a,_buffer99b,_buffer99c;//(size 9x9)
 	vpMatrix _buffer612;//(size 6x12)
 	vpMatrix _buffer126;//(size 12x6)
 	vpColVector _buffer61;//(size 6x1)
 	vpMatrix _buffer66;//(size 6x6)
 	vpMatrix _buffer123;//(size 12x3)
 	vpMatrix _buffer33;//(size 3x3)
	vpMatrix _buffer1212a;//(size 12x12)
	vpMatrix _buffer1212b;//(size 12x12)

	vpMatrix _eye1212;//(size 12x12
	vpMatrix _eye99;//(size 9 x 9 )
	vpMatrix _eye33;//(size 3x3)


	/** some variables */
	const double epsilon;
	double norm,norm2,norm3,norm4;
	double alpha1,alpha2,alpha3,alpha4,alpha5,alpha6;

	/** variance on velocities */
	double _varv;
	/** variance on rotation velovities */
	double _varw;


	/** Compute state covariance */
	void ComputeQu();
	/** Prepare relative matrices of the vector V */
	void PrepareVec(const vpColVector  &V, const double & dt=1.0);

	/** Compute rotation matrix R(Vec) */ 
	void Compute_RVec(vpMatrix  &out);
	/** Compute derivative matrix d [R(Vec)]_v / d Vec */ 
	void Compute_dRVec_dVec(vpMatrix  &out);
	/** Compute  matrix S(Vec) */ 
	void Compute_SVec(vpMatrix  &out);
	/** Compute derivative matrix d [S(Vec)]_v / d Vec */ 
	void Compute_dSVec_dVec(vpMatrix  &out);
	
	/** Compute derivative matrix d logm(R) / d [R]_v */ 
	void Compute_dPsi_dR(const vpMatrix &R);
	/** Compute derivative matrix d P / d w */ 
	void Compute_dP_dw(const double & dt);
	/** Compute derivative matrix d Ou / d Ou */ 
	void Compute_dOu_dOu();
	/** Compute derivative matrix d Ou / d w */ 
	void Compute_dOu_dw(const double &dt);
	/** Vision correction + check mahalanobis distance*/
	bool visionMahaUpdate(); 
	
	public:
	/** Default constructor*/
	vpSE3Kalman();
	/** Default destructor */
	virtual ~vpSE3Kalman(){};

	/** Initialisation */
	virtual void init();
	/** Predict the state */
	virtual void predict(const double &dt);
	/** Vision correction */
	virtual void visionUpdate();

	


	/** Get/Set */
	/** Get vision data  */
	void setXv(const vpColVector &Xv){_Xv=Xv;}
	/** Get covariance on vision data */
	void setQv(const vpMatrix &covMes){_Qv=covMes;}
 
	/** Get displacement */
	vpColVector getDisplacement(){return _POu;};
	/** Get displacement covariance */
	vpMatrix getDisplacementCov(){return _QPOu;};
	/** Get current estimated state */
	vpColVector getState(){return _X;};
	/** Get current associated covariance */
	vpMatrix getStateCovariance(){return _Q;};

	vpMatrix getGain(){return _Kv;};


	/** Define the variance of the velocity data */
	void setVarV(double varv){_varv=varv;}
	/** Define the variance of the rotation velocity data */
	void setVarW(double varw){_varw=varw;}
	
	void setQu(vpMatrix &Qu){_Qu=Qu;}


};

#endif
