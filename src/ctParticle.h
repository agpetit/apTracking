#ifndef ctParticle_h
#define ctParticle_h

#include <visp/vpImage.h>
#include <visp/vpColor.h>
#include <visp/vpColVector.h>
#include <visp/vpNoise.h>
#include <visp/vpCameraParameters.h>
#include "apViews.h"
#include "apContourPoint.h"
#include "apLogPolarHist.h"
#include "apDetection.h"


#ifdef VISP_HAVE_OPENCV
#  if VISP_HAVE_OPENCV_VERSION >= 0x020101
#    include <opencv2/core/core.hpp>
#    include <opencv2/imgproc/imgproc.hpp>
#    include <opencv2/imgproc/imgproc_c.h>
#  else
#    include <cv.h>
#  endif
#endif
using namespace std;
using namespace cv;

//#include <ct_tracking/ctObject9k.h>
// #include <ct_tracking/ctObject.h>
//#include <ct_tracking/ctHist.h>
//#include <ct_tracking/ctRect.h>


/*!
  \file ctParticle.h
*/
class ctParticle
{
public:
	ctParticle();
	~ctParticle();
	
	void initLogPolarHist(int nr, int nw, int height, int width);
	void initDetection(apDetection &_detect);

	/*! 
		operator < :  a particle is set < to another if its weight is smaller
	*/
	bool operator<(const ctParticle &prcl){
		return (weight < prcl.getWeight());};
	bool operator>(const ctParticle &prcl){
		return (weight > prcl.getWeight());};
	
	//! Display
	void display(vpImage<vpRGBa> & I, float ratio,vpColor Col, int thickness = 2);
	void displayPred(vpImage<vpRGBa> & I, float ratio,vpColor Col, int thickness = 2);
	
	//!Set
	inline void set_u(float U) { u = U; }
	inline void set_v(float V) { v = V; }
	inline void set_size(float pSize) { size = pSize; }
	inline void set_angle(float pAngle) { angle = pAngle; }
	inline void set_ind(int index) { ind = index;}
	
	inline void set_vPred(float vPred) { v_pred = vPred;}
	inline void set_uPred(float uPred) { u_pred = uPred;}
	inline void set_sizePred(float pSizePred) { size_pred = pSizePred; }
	inline void set_anglePred(float pAnglePred) { angle_pred = pAnglePred; }
	
	inline void setWeight(float Weight) { weight = Weight; }
	inline void setWghtCumul(float WeightC) { wght_cumul = WeightC; }
	inline void setDist(float D) { distance = D ; }
	inline void setOri(float orip) { oripart = orip ; }
	
	inline void setProba(float Proba) { proba = Proba; }

	inline void set_sx(int sx){sample_x = sx;}
	inline void set_sy(int sy){sample_y = sy;}
	inline void set_stheta(int stheta){sample_theta = stheta;}
	inline void set_sigmauv(int suv){sig_uv = suv;}
	inline void set_sigmar(int sr){sig_r = sr;}

	inline void set_nr(int _nr){nr = _nr;}
	inline void set_nw(int _nw){nw = _nw;}

	inline void set_lambdaO(double _lambdaO){lambdaO = _lambdaO;}
	inline void set_muD(double _muD){muD = _muD;}

	
	//!Get
	inline float get_u() const { return u ; }
	inline float get_v() const { return v ; }
	inline float get_size() const { return size ; }
	inline float get_angle() const { return angle ; }
	inline int get_ind() const { return ind ; }
	
	inline float get_uPred() const { return u_pred ; }
	inline float get_vPred() const { return v_pred ; }
	inline float get_sizePred() const { return size_pred ; }
	inline float get_anglePred() const { return angle_pred ; }
	
	inline float getWeight() const { return weight ; }
	inline float getDist() const { return distance ; }
	inline float getOri() const { return oripart ; }
	inline float getWghtCumul() const { return wght_cumul ; }
	
	inline float getProba() const { return proba ; }

	inline float getWidth(float ratio)const { return (float)sqrt(size/ratio) ; }
	inline float getHeight(float ratio)const{ return (float)sqrt(size*ratio) ; }
	inline float getWidthPred(float ratio)const { return (float)sqrt(size_pred/ratio) ; }
	inline float getHeightPred(float ratio)const{ return (float)sqrt(size_pred*ratio) ; }
	
	inline unsigned get_nbPartOut()const{return nb_part_out;}
	
	//inline void buildFromRect(const ctRect &loc) { u=loc.get_uc(); v=loc.get_vc(); angle = loc.get_angle(); size = loc.get_size();}
	//inline void buildPredFromRect(const ctRect &loc) { u_pred=loc.get_uc(); v_pred=loc.get_vc(); angle_pred = loc.get_angle(); size_pred = loc.get_size();}

	//void buildRect(ctRect *loc, float ratio)const { loc->set_uc(u); loc->set_vc(v);loc->set_angle(angle);loc->set_size(size) ;loc->set_ratio(ratio);}
	//ctRect get_rect(float ratio)const { ctRect rect(u, v, angle, size, ratio); return rect;}
	//ctRect get_rect_pred(float ratio)const { ctRect rect(u_pred, v_pred, angle_pred, size_pred, ratio); return rect;}
	
	//!----------------------------------------------------------------------------
	//!--------------------Evolution : --------------------------------------------
	void compensate_attitude(const vpCameraParameters &Cam, const vpColVector & measure);
	
	//!Add gaussian noise on coordinates :
	void addGNoise(float sigma, float mean);//Ajout d'un bruit gaussien sur les coordonn�es u v des particules
	void addGNoise(vpGaussRand &GRand);//Ajout d'un bruit gaussien sur les coordonn�es u v des particules

	void addGNoisePred(vpGaussRand &GRanduv, vpGaussRand &GRandangle, vpGaussRand &GRandz);
		
	void pred_GNoise(vpGaussRand &GRanduv,vpGaussRand &GRandangle,vpGaussRand &GRandz);
	void pred_rdmUStep(const float & StepMax, const float & angleMax, const float & scaleMax);
	void pred_CsteVel(const vpColVector &v, vpGaussRand &GRanduv,vpGaussRand &GRandangle,vpGaussRand &GRandz,float dt = 0.1);
	void pred_vel(double * vel);
// 	void pred_vel(const vpColVector &dedt, float dt);//prediction utilisant la vitesse des features
	
	// Mesure de la distance entre les coordonn�es pr�dites et celles d'un point donn�
	// (Point track� par exemple)
	void computeDrift(float u_ref, float v_ref);

	void buildFrom(vpImage<unsigned char> &Iseg);
	//void initViews( std::vector< std::vector<vpImage<unsigned char>*>*> Views, vpMatrix &hierarchy, vpMatrix &dataTemp0, vpMatrix &dataTemp1);
	void resize(const vpImage<unsigned char>& _I, vpImage<unsigned char>&  Io, double scale);
	double computeSimError(vpImage<vpRGBa> *I, vpImage<unsigned char> &IView, const vpImagePoint cog, const int surface,const double orientation,bool weights);
	double computeSimError(vpImage<vpRGBa> *I, std::vector<vpImage<unsigned char>*> ModelViewSc, const vpImagePoint cog, const int surface,const double orientation,bool weights);
	double computeSimErrorCP(vpImage<vpRGBa> *I, std::vector<apContourPoint*> &ContourPoints, const int surface,const double orientation,bool weights);
	double computeSimErrorCPV(vpImage<vpRGBa> *I,  std::vector<std::vector<apContourPoint*>> &ContourPointsSc, const int surface, const double orientation,bool weights);
	double computeSimErrorCPVN(vpImage<vpRGBa> *I,  std::vector<std::vector<apContourPoint*>> &ContourPointsSc, const int surface, const double orientation,bool weights, int niseg, int nsample);
	double computeSimErrorSeg(vpImage<vpRGBa> *I, vpImage<vpRGBa> *Iseg, vpImage<unsigned char> &IView, const int surface, const double orientation,bool weights);
	double computeSimErrorC(vpImage<vpRGBa> *I, vpImage<unsigned char> &IView, const vpImagePoint cog, const int surface, const double orientation,bool weights);
	double computeSimErrorSC(std::vector<double> &logPolarHistIseg,  const vpImagePoint cog,  std::vector<vpImage<unsigned char>*> ModelViewSc, const int surface, const double orientation,bool weights);
	double computeSimErrorSC(std::vector<double> &logPolarHistIseg,  const vpImagePoint cog, std::vector<std::vector<apContourPoint*>> &ContourPointsSc, const int surface, const double orientation,bool weights);
	double computeSimErrorSCOpt(std::vector<double> &logPolarHistIseg,  const vpImagePoint cog, double angleIseg, std::vector<std::vector<double>> &logPolarHistV, const int surface, const double orientation,bool weights);
	double computeSimErrorSteger(vpImage<unsigned char> &Igrad,  std::vector<std::vector<apContourPoint*>> &ContourPointsSc, const int surface, const double orientation,bool weights);

	void reWeight(vpImage<unsigned char> &Iseg);
		
		
	//!-------------------------------------------------------------------
	//!------------ Comparison with a reference object -------------------
	
	/*!
		Compute error between particle and object_reference (divided into 9 parts) in terms of color histograms	
		WARNING! Utilise les valeurs pr�dites !!
		WARNING: s'assurer dans le code qu'on utilise le m�me mode de calcul (avec ou sans noyau) pour les histogrammes dans la particule et l'histogramme de r�f�rence
	*/
	
	//!-- Using transformation of each pixel ---
	//!
	//!Color object :
	//!Object ref: ctObject
	//float compute_histDist(const vpImage<vpRGBa> & Icolor, const ctObject<vpRGBa> & object_ref, std::list<ctPoint<vpRGBa> > &points2, bool weights);
	//!Object ref: ctObject9k
	//float compute_histDist(const vpImage<vpRGBa> & Icolor, const ctObject9k<vpRGBa> & object_ref, int i, std::list<ctPoint<vpRGBa> > *points2sub, bool weights);
	//float compute_histDist(const vpImage<vpRGBa> & Icolor, const ctObject9k<vpRGBa> & object_ref, std::list<ctPoint<vpRGBa> > *points2sub, bool weights);
	//float compute_globHistDist(const vpImage<vpRGBa> & Icolor, const ctObject9k<vpRGBa> & object_ref, std::list<ctPoint<vpRGBa> > &points2, bool weights);
// 	float compute_globHistDist2(const vpImage<vpRGBa> & Icolor, const ctObject9k<vpRGBa> & object_ref, bool weights);
	
	//!Gray scale object :
	//!Object ref: ctObject
	//float compute_histDist(const vpImage<unsigned char> & Igray, const ctObject<unsigned char>  & object_ref, std::list<ctPoint<unsigned char> > &points2, bool weights);
	//!Object ref: ctObject9k
	//float compute_histDist(const vpImage<unsigned char> & Igray, const ctObject9k<unsigned char> & object_ref, int i, std::list<ctPoint<unsigned char> > *points2sub, bool weights);
	//float compute_histDist(const vpImage<unsigned char>  & Igray, const ctObject9k<unsigned char>  & object_ref, std::list<ctPoint<unsigned char> > *points2sub, bool weights);
	//float compute_globHistDist(const vpImage<unsigned char>  & Igray, const ctObject9k<unsigned char>  & object_ref, std::list<ctPoint<unsigned char> > &points2, bool weights);


	// Coordonn�es dans l'image :
	float u , v; 
	// Taille
	float size;
	// Orientation
	float angle;

	int ind;
	float oripart;


private:

	apViews view;
	apDetection detectParam;
	double muD,lambdaO;
	apLogPolarHist logPolarHist;
	//! Etat de la particule :
	int sample_x;
	int sample_y;
	int sample_theta;
	
	int nr;
	int nw;

	int sig_uv;
	int sig_r;


	// Etat pr�dit :
	float u_pred, v_pred;//float
	float size_pred;
	float angle_pred;
	
	float ratio;
	
	
	//! Mesure distance 
	float distance;
	//! Poids
	float weight;

	float proba;;
	// Cumul des poids
	float wght_cumul;
	
	unsigned nb_part_out;//nombre de parties hors de l'image
	
	bool inside;
};

#endif



