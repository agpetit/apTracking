
#ifndef ctPFilter_h
#define ctPFilter_h

//Visp
#include <visp/vpList.h>
// #include <visp/vpHistogram.h>
#include <visp/vpImage.h>
#include <visp/vpColor.h>
#include <visp/vpImageIo.h>

//
#include "ctParticle.h"
#include "apDetection.h"
//#include <ct_tracking/ctObject.h>
//#include <ct_tracking/ctObject9k.h>
// #include <ct_tracking/ctObject_mpart.h>
//#include <ct_tracking/ctTools.h>
//#include <ct_tracking/ctHist.h>

/*!
  \file ctPFilter.h
*/
class ctPFilter
{
public:
	typedef enum
	{
		RNDSTEP,
		GSTEP,
		GSTEP_ATTITUDE,
		CSTE_VEL
	} MotionModel;

	//! ---------------  constructor / Destructor ------------------
	ctPFilter();
	~ctPFilter();
	
	//
	void init(const unsigned int pNumber, const ctParticle & pIni, const MotionModel &model = ctPFilter::GSTEP);
	//void init(const unsigned int pNumber, const ctRect & poseIni, const MotionModel &model = ctPFilter::GSTEP );
	void init(const unsigned int pNumber, const float u, const float v, const float & angle, const float & size, const MotionModel &model = ctPFilter::GSTEP);
	
	//!---------------------- Prediction --------------------------
	void evolution(const MotionModel Model, const vpColVector &measure, float delta_t = 0.1);
	void evolution(const MotionModel Model, const vpCameraParameters &cam,const vpColVector &measure, float delta_t = 0.1);
	void rdmU_Prediction(const float & stepMax, const float & angleMax, const float & scaleMax);
	void Vel_Prediction(float * vel, const float & sigma_uv, const float & sigma_angle, const float & sigma_size);
	void cstVel_Prediction(const float & sigma_uv, const float & sigma_angle, const float & sigma_size);
	
	ctParticle compute_predMean();
	//!---------------------  Likelihood  -------------------------

	/*!
		\param img_hsv : image from which histogram is calculated
		\param sigma_ker : variance of the kernel used in histogram calculation
			WARNING:	if sigma<=0 no kernel is used (classic histogram is computed)
	*/
	//!Gray scale
		
	//void likelihood(const vpImage<unsigned char> & Igray, const ctObject9k<unsigned char> & object_ref, std::list<ctPoint<unsigned char> > *points2sub, bool weights);
	//void likelihood(const vpImage<unsigned char> & Igray, const ctObject<unsigned char> & object_ref, std::list<ctPoint<unsigned char> > &points2, bool weights);
	//void likelihood2(const vpImage<unsigned char> & Igray, const ctObject9k<unsigned char> & object_ref, std::list<ctPoint<unsigned char> > *points2sub,std::list<ctPoint<unsigned char> > points2,  bool weights);
	
	//!Color
	//void likelihood(const vpImage<vpRGBa> & Icolor, const ctObject9k<vpRGBa> & object_ref, std::list<ctPoint<vpRGBa> > *points2sub, bool weights);
	//void likelihood(const vpImage<vpRGBa> & Icolor, const ctObject9k<vpRGBa> & object_ref, std::vector <std::list<ctPoint<vpRGBa> > *> &pointsTab, bool weights);
		
	//void likelihood(const vpImage<vpRGBa> & Icolor, const ctObject<vpRGBa> & object_ref, std::list<ctPoint<vpRGBa> > &points2, bool weights);
	//void likelihood2(const vpImage<vpRGBa> & Icolor, const ctObject9k<vpRGBa> & object_ref, std::list<ctPoint<vpRGBa> > *points2sub, std::list<ctPoint<vpRGBa> > points2, bool weights);
	
	
	//! ------------------------ Weights update --------------------
	double compute_distVar();
	void weightsUpdate(const float & min_threshold, const float & lost_prcl_ratio);
	
	//!------------------------- Resample --------------------------
	void weightedDraw(const int & nb);
	void simpleDraw();//random draw (not weighted !)
	
	//!------------------------- Estimate --------------------------
	void estimateMean();
	void estimateBest();
// 	void estimateBest(int percentage);//Mean of the "percentage"% best particles
	void estimateMeanSimple();//Simple mean of all particles, without weights
	
	//!------------------------- Global step --------------------------
	//void run(const vpImage<unsigned char> & Igray, const ctObject9k<unsigned char> & object_ref, std::list<ctPoint<unsigned char> > *points2sub, bool weights);
// 	void run(const vpImage<vpRGBa> & Icolor, const ctObject9k<vpRGBa> & object_ref, std::list<ctPoint<vpRGBa> > *points2sub, bool weights);

	
	
	//!------------------------- Get/Set ---------------------------- 
	inline float      getWght_sum() const				  	{ return wght_sum ; }
	inline void       setWght_sum(const float & WeightS) 	{ wght_sum = WeightS ; }
	inline ctParticle getEstim() const           		  	{ return pEstimate ; }
	inline void       setEstim(const ctParticle & ptcl)  	{ pEstimate = ptcl ; }
	inline ctParticle getPrcl_ini() const           	  	{ return prcl_ini ; }
	inline void       setPrcl_ini(const ctParticle & ptcl) { prcl_ini = ptcl ; }
	inline ctParticle getBest_prcl() const           		{ return best_particle ; }
	inline void       setBest_prcl(const ctParticle & ptcl){ best_particle = ptcl ; }
	inline ctParticle getPredMean() const           		{ return predMean_particle ; }
	
	//Velocity
	inline float get_vel_u()     const { return vel_u ; }
	inline float get_vel_v()     const { return vel_v ; }
	inline float get_vel_size()  const { return vel_size ; }
	inline float get_vel_angle() const { return vel_angle ; }
	
	inline void set_vel (float V_u, float V_v, float V_angle, float V_size, float DeltaSize, float RatioSize){vel_u = V_u; vel_v = V_v;vel_size = V_size; vel_angle = V_angle;delta_size = DeltaSize;ratio_size = RatioSize;}	
	
	inline unsigned int   get_pNum()      const { return particleVect.size() ; }

	inline void set_lost(bool is_lost) {object_lost = is_lost;}

	inline void setSigmaUV(const double s) {sigmauv = s;}
	inline void setSigmaZ(const double s) {sigmaz = s;}
	inline void setSigmaR(const double s) {sigmar = s;}
	inline void setStepMax(double uv, double angle, double z) {tMax = uv;rMax = angle;scaleMax = z;}

	inline double getSigmaUV() const{return sigmauv;}
	inline double getSigmaR() const{return sigmar;}
	inline double getSigmaZ() const{return sigmaz;}
	inline double getStepMaxUV() const{return tMax;}
	inline double getStepMaxAngle() const{return rMax;}
	inline double getStepMaxZ() const{return scaleMax;}

	inline float get_err_min() const{return err_min;}

	inline MotionModel getMotionModel()const{return model;}
	
	//!Tools
	void show_weights();
	
	double compute_Neff();
	
	double compute_variance();



public:
	//! Liste des particules :
	std::vector<ctParticle*>  particleVect;
	
	//! Nombre de particules
	//   unsigned int pNumber;	
	
	float err_sum, err_min, err_max;
	
	//!nombre de particules pour lesquelles nb_part_in < object_ref.get_nbPart()/2
	int nb_prcl_out;
	
	bool object_lost;
	float best_weight;
	float orieff;
	ctParticle predMean_particle;
	ctParticle pEstimate;
	double sigmauv;
	double sigmar;
	double sigmaz;
	apDetection detect;
	
private:
	float wght_sum;
	//! Estimate
	// Vitesse
// 	float * vel_curr;
	float vel_u, vel_v, vel_angle, vel_size, delta_size, ratio_size;
	//! Estimate of the state
	//! Best particle
	ctParticle best_particle;
	//! Mean of pred particles
	//! Initial position estimate
	ctParticle prcl_ini; // Pas tr�s utile... d�j� m�moris� dans la classe objet

	double tMax, rMax, scaleMax;

	int pNum;

	MotionModel model;

};

#endif


