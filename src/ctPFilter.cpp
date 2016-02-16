#include <omp.h>
#include <math.h>
#include <list>

#include <visp/vpDebug.h>
#include <visp/vpNoise.h>

#include <cv.h>

//#include <ct_tracking/ctPFilter.h>
#include "ctPFilter.h"

/*!

  Default constructor.

*/
ctPFilter::ctPFilter()
	: err_sum(0), err_min(0), err_max(0), nb_prcl_out(0), object_lost(false), wght_sum(0), vel_u(0), vel_v(0), vel_angle(0), vel_size(0), delta_size(0), ratio_size(1)
{
	//vpTRACE("Call the constructor of pFilter");

	//   pNumber = 0;


	/*sigmauv = 6;
	sigmar = 0.05;
	sigmaz = 0.7;*/
	sigmauv = 7;
	sigmar = 0.1;
	sigmaz = 8;
	sigmauv = 6;
	sigmar = 0.08;
	sigmaz = 5;
	tMax = 13;
	rMax = 0.6;
	scaleMax = 0.5;
	orieff = 0;
}

/*!

  Destructor.

*/
ctPFilter::~ctPFilter()
{
	//vpTRACE("Call the destructor of pFilter");

}


/*!
  Initialize particle filter
*/

void ctPFilter::init(const unsigned int pNumber, const ctParticle & pIni, const MotionModel &Model)
{
	//vpTRACE("Filter Initialisation");

}


/*!
  Initialize particle filter
*/

/*void ctPFilter::init(const unsigned int pNumber, const ctRect & location, const MotionModel &Model)
{
	vpTRACE("Filter Initialisation");


	prcl_ini.set_u(location.get_uc());
	prcl_ini.set_v(location.get_vc());
	prcl_ini.set_angle(location.get_angle());
	prcl_ini.set_size(location.get_size());
	prcl_ini.setWeight(1/pNumber);//Poids �gal pour toutes les particules

	vpGaussRand rand(4,0);

	ctParticle prcl;
	particleVect.clear();

	//Initialisation des particules dans la liste
	for (unsigned int i = 0; i < pNumber ; i++)
	{
		prcl = prcl_ini;
		prcl.addGNoise(rand);//bruitage des coordonn�es // A modifier �ventuellement !
				// Bruiter aussi angle et taille !

		//Initialisation coordonn�es pr�dites
		prcl.set_uPred(prcl.get_u());
		prcl.set_vPred(prcl.get_v());

		particleVect.push_back(prcl) ;//Ajout de la particule dans la liste
	}

	pEstimate = prcl_ini ;

	model = Model;
    object_lost = false;
	pNum = pNumber;
	vpTRACE("Initialisation ok");
}

*/

/*!
  Initialize particle filter
*/

void ctPFilter::init(const unsigned int partNum, const float u, const float v, const float & angle, const float & size, const MotionModel &Model)
{

}

//!******************************************************************************************************
//!---------------------------------------------------------------------------------
//!-------------------------------------Prediction----------------------------------
//!******************************************************************************************************
void ctPFilter::evolution(const MotionModel Model, const vpColVector &measure, float delta_t)
{

}

void ctPFilter::evolution(const MotionModel Model, const vpCameraParameters &Cam, const vpColVector &measure, float delta_t)
{

}

/*!
	Prediction : Random uniform step
*/

void ctPFilter::rdmU_Prediction(const float & stepMax, const float & angleMax, const float & scaleMax)
{
	ctParticle *prcl;


	for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
	{
		prcl = particleVect[i];
		prcl->pred_rdmUStep(stepMax, angleMax, scaleMax);
		//particleVect[i] = prcl;

	}
}

/*!
  Prediction : Constant Velocity + noise
  \param vel : array of 4 velocity components (du, dv, dangle, sqrt(S2/S1))
  WARNING : vitesse sur S � ajouter !!

*/

void ctPFilter::Vel_Prediction(float * vel, const float & sigma_uv, const float & sigma_angle, const float & sigma_size)
{

}

/*!
  Prediction : Constant Velocity + noise
  \param vel : array of 4 velocity components (du, dv, dangle, sqrt(S2/S1))

*/

void ctPFilter::cstVel_Prediction(const float & sigma_uv, const float & sigma_angle, const float & sigma_size)
{

}

/*!
  Computes and return the mean of predicted particles
*/

ctParticle ctPFilter::compute_predMean()
{
	float u_cum = 0; float v_cum = 0 ; float angle_cum = 0; float size_cum = 0;
	ctParticle *ptcl;


	for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
	{
		ptcl = particleVect[i];
		u_cum += ptcl->get_uPred();
		v_cum += ptcl->get_vPred();
		angle_cum += ptcl->get_anglePred();
		size_cum += ptcl->get_sizePred();

	}

	u_cum /= this->get_pNum();
	v_cum /= this->get_pNum();
	angle_cum /= this->get_pNum();
	size_cum /= this->get_pNum();
	predMean_particle.set_u((int)u_cum);
	predMean_particle.set_v((int)v_cum);
	predMean_particle.set_angle(angle_cum);
	predMean_particle.set_size(size_cum);

	//predMean_particle = *ptcl;

	return predMean_particle;
}


//!******************************************************************************************************
//!---------------------------------------------------------------------------------
//!-------------------------------------Likelihood----------------------------------
//!******************************************************************************************************
/*!
	Likelihood computed  by comparing histogram with color-part object (divided into 9 parts)
	(La distance entre les histogrammes est calcul�e dans la fonction compare � partir du coefficient de Battacharyya)
	WARNING : effets de bords -> on exclut les particules qui sortent de l'image de plus de moiti� ?
*/
/*void ctPFilter::likelihood(const vpImage<unsigned char> & Igray, const ctObject9k<unsigned char> & object_ref, std::list<ctPoint<unsigned char> > *points2sub, bool weights)
{
//  	vpTRACE("Start colLikelihood");
	err_sum = 0;
	err_min = 0;
	err_max = 0 ;

	float err_curr = 0;
	ctParticle prcl;

	nb_prcl_out = 0; //nb de particules pas enti�rement dans l'image

	//!-------------------------Parcours de la liste de particules-----------------------//


	for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
	{
		//!Distance of the current particle
		prcl = particleVect[i];
		err_curr = prcl.compute_histDist(Igray, object_ref, points2sub, weights);//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ! -> distance 1 lorsque la particule sort)

		//! On compte les particules qui ne sont pas enti�rement dans l'image (pour ensuite d�tecter si l'objet est perdu)

		if (prcl.get_nbPartOut() > 0) {//4) {// object_ref.get_nbPart()/2) {
			nb_prcl_out ++;
		}

		//!Associate error with particle (could be done in ctParticle.cpp ?)
		//!Note : Tester : set distance as a mean of err_curr and err_curr_global ?
		prcl.setDist(err_curr);


		//! err_min, err_max
		if (i == 0) {
			err_min = err_curr;
			err_max = err_curr;
		}
		else {
			if (err_curr < err_min) {
				err_min = err_curr;
			}
			if (err_curr > err_max) {
				err_max = err_curr;
			}
		}

		err_sum += err_curr;

		//! Update prcl
		particleVect[i] = prcl;


  }
}
*/


//!A tester:
/*!

	Likelihood computed  by comparing histogram of an object
	(La distance entre les histogrammes est calcul�e dans la fonction compare � partir du coefficient de Battacharyya)
	WARNING : effets de bords -> on exclut les particules qui sortent de l'image de plus de moiti� ?
*/
/*void ctPFilter::likelihood(const vpImage<unsigned char> & Igray, const ctObject<unsigned char> & object_ref, std::list<ctPoint<unsigned char> > &points2, bool weights)
{
	err_sum = 0;
	err_min = 0;
	err_max = 0 ;

	float err_curr = 0;

	ctParticle prcl;

	nb_prcl_out = 0; //nb de particules pas enti�rement dans l'image


	//!-------------------------Parcours de la liste de particules-----------------------//


	for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
	{
		prcl = particleVect[i];

		//! Compute error
		err_curr = prcl.compute_histDist(Igray, object_ref, points2, weights);//!WARNING : if the predicted particle is out, the 2nd histogram will be computed from a small number of points, and the comparison can be biased.


		//!Associate error with particle (could be done in ctParticle.cpp ?)
		//!Note : Tester : set distance as a mean of err_curr and err_curr_global ?
		prcl.setDist(err_curr);


		//! err_min, err_max
		if (i == 0) {
			err_min = err_curr;
			err_max = err_curr;
		}
		else {
			if (err_curr < err_min) {
				err_min = err_curr;
			}
			if (err_curr > err_max) {
				err_max = err_curr;
			}
		}

		err_sum += err_curr;

		//! Update prcl
		particleVect[i] = prcl;



	}//!End loop
}
*/


//!A tester:
/*!
	Likelihood computed  by comparing histogram of an object
	(La distance entre les histogrammes est calcul�e dans la fonction compare � partir du coefficient de Battacharyya)
	WARNING : effets de bords -> on exclut les particules qui sortent de l'image
*/
/*void ctPFilter::likelihood2(const vpImage<unsigned char> & Igray, const ctObject9k<unsigned char> & object_ref, std::list<ctPoint<unsigned char> > *points2sub, std::list<ctPoint<unsigned char> >  points2, bool weights)
{
	err_sum = 0;
	err_min = 0;
	err_max = 0 ;

	float err_curr = 0;
	float err_curr_global (0);
	ctParticle prcl;

	nb_prcl_out = 0; //nb de particules pas enti�rement dans l'image

	//!-------------------------Parcours de la liste de particules-----------------------//


	for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
	{
		//!Distance of the current particle
		prcl = particleVect[i];
		err_curr = prcl.compute_histDist(Igray, object_ref, 1, points2sub, weights) + prcl.compute_histDist( Igray, object_ref, 3, points2sub, weights) + prcl.compute_histDist(Igray, object_ref, 5, points2sub, weights) +prcl.compute_histDist(Igray, object_ref, 7, points2sub, weights);
		err_curr /= 4;

		//TEST !!!
		err_curr_global = prcl.compute_globHistDist(Igray, object_ref, points2, weights);//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ?!)
		//\TEST !


		err_curr = (err_curr + err_curr_global)/2 ;
		//! On compte les particules qui ne sont pas enti�rement dans l'image (pour ensuite d�tecter si l'objet est perdu)

		if (prcl.get_nbPartOut() > 0) {//4) {// object_ref.get_nbPart()/2) {
			nb_prcl_out ++;
		}

		//!Associate error with particle (could be done in ctParticle.cpp ?)
		//!Note : Tester : set distance as a mean of err_curr and err_curr_global ?
		prcl.setDist(err_curr);


		//! err_min, err_max
		if (i == 0) {
			err_min = err_curr;
			err_max = err_curr;
		}
		else {
			if (err_curr < err_min) {
				err_min = err_curr;
			}
			if (err_curr > err_max) {
				err_max = err_curr;
			}
		}

		err_sum += err_curr;

		//! Update prcl
		particleVect[i] = prcl;


	}
}
*/

//!Color

	/*!
	Likelihood computed  by comparing histogram with color-part object (divided into 9 parts)
	(La distance entre les histogrammes est calcul�e dans la fonction compare � partir du coefficient de Battacharyya)
	WARNING : effets de bords -> on exclut les particules qui sortent de l'image de plus de moiti� ?
*/
/*void ctPFilter::likelihood(const vpImage<vpRGBa> & Icolor, const ctObject9k<vpRGBa> & object_ref, std::list<ctPoint<vpRGBa> > *points2sub, bool weights)
{
//  	vpTRACE("Start colLikelihood");
	err_sum = 0;
	err_min = 0;
	err_max = 0 ;

	float err_curr = 0;
// 	float err_curr_global (0);
	ctParticle prcl;

	nb_prcl_out = 0; //nb de particules pas enti�rement dans l'image


// 	double t0,t1,t2,t3;
	//!-------------------------Parcours de la liste de particules-----------------------//

	//!WARNING !! d�finir des param�tres en private !!

	//#pragma omp parallel for
	for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
	{
		prcl = particleVect[i];

		err_curr = prcl.compute_histDist(Icolor, object_ref, points2sub, weights);//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ! -> distance 1 lorsque la particule sort)

		//!Associate error with particle (could be done in ctParticle.cpp ?)
		//!Note : Tester : set distance as a mean of err_curr and err_curr_global ?
		prcl.setDist(err_curr);

		//! Update prcl
		particleVect[i] = prcl;


	}


	for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
	{
		prcl = particleVect[i];
		err_curr = prcl.getDist();

		//! On compte les particules qui ne sont pas enti�rement dans l'image (pour ensuite d�tecter si l'objet est perdu)

		if (prcl.get_nbPartOut() > 0) {//4) {// object_ref.get_nbPart()/2) {
			nb_prcl_out ++;
		}

		err_sum += err_curr;

			//! err_min, err_max
		if (i == 0) {
			err_min = err_curr;
			err_max = err_curr;
		}
		else {
			if (err_curr < err_min) {
				err_min = err_curr;
			}
			if (err_curr > err_max) {
				err_max = err_curr;
			}
		}

	}


//	for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
//	{
//		//!Distance of the current particle
//		prcl = particleVect[i];
//// 		t0 = vpTime::measureTimeMs();
//		err_curr = prcl.compute_histDist(Icolor, object_ref, points2sub, weights);//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ! -> distance 1 lorsque la particule sort)
//// 		t1 = vpTime::measureTimeMs();
//// 		std::cout << "! Compute hist_dist time 1 prcl : " << t1-t0 << std::endl;
//		//TEST !!!
//// 		err_curr_global = prcl.compute_globHistDist(Icolor, object_ref, points2sub, weights);//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ?!)
//		//\TEST !
//
//		//! On compte les particules qui ne sont pas enti�rement dans l'image (pour ensuite d�tecter si l'objet est perdu)
//
//		if (prcl.get_nbPartOut() > 0) {//4) {// object_ref.get_nbPart()/2) {
//			nb_prcl_out ++;
//		}
//
//		//!Associate error with particle (could be done in ctParticle.cpp ?)
//		//!Note : Tester : set distance as a mean of err_curr and err_curr_global ?
//		prcl.setDist(err_curr);
//
//
//		//! err_min, err_max
//		if (i == 0) {
//			err_min = err_curr;
//			err_max = err_curr;
//		}
//		else {
//			if (err_curr < err_min) {
//				err_min = err_curr;
//			}
//			if (err_curr > err_max) {
//				err_max = err_curr;
//			}
//		}
//
//		err_sum += err_curr;
//
//		//! Update prcl
//		particleVect[i] = prcl;
//
//
//// 		t2 = vpTime::measureTimeMs();
//// 		std::cout << "minmax time : " << t2-t1 << std::endl;
//  }
}

*/
	/*!
	Likelihood computed  by comparing histogram with color-part object (divided into 9 parts)
	(La distance entre les histogrammes est calcul�e dans la fonction compare � partir du coefficient de Battacharyya)
	WARNING : effets de bords -> on exclut les particules qui sortent de l'image de plus de moiti� ?
*/
/*void ctPFilter::likelihood(const vpImage<vpRGBa> & Icolor, const ctObject9k<vpRGBa> & object_ref, std::vector<std::list<ctPoint<vpRGBa> > *> &pointsTab, bool weights)
{
//  	vpTRACE("Start colLikelihood");
	err_sum = 0;
	err_min = 0;
	err_max = 0 ;

	float err_curr = 0;
// 	float err_curr_global (0);
	ctParticle prcl;

	nb_prcl_out = 0; //nb de particules pas enti�rement dans l'image


// 	double t0,t1,t2,t3;
	//!-------------------------Parcours de la liste de particules-----------------------//

	//!WARNING !! d�finir des param�tres en private !!

	//double t0, t1, t2, t3, t4, t5, t6;
 //	t0 = vpTime::measureTimeMs();

	#pragma omp parallel for
	for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
	{

		err_curr = particleVect[i].compute_histDist(Icolor, object_ref, pointsTab[i], weights);//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ! -> distance 1 lorsque la particule sort)

		//!Associate error with particle (could be done in ctParticle.cpp ?)
		//!Note : Tester : set distance as a mean of err_curr and err_curr_global ?
		particleVect[i].setDist(err_curr);

		//! Update prcl
		//particleVect[i] = prcl;
	}

	//t1 = vpTime::measureTimeMs();
	//std::cout << "parallel time : " << t1-t0 << std::endl;

	for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
	{
		prcl = particleVect[i];
		err_curr = prcl.getDist();

		//! On compte les particules qui ne sont pas enti�rement dans l'image (pour ensuite d�tecter si l'objet est perdu)

		if (prcl.get_nbPartOut() > 0) {//4) {// object_ref.get_nbPart()/2) {
			nb_prcl_out ++;
		}

		err_sum += err_curr;

			//! err_min, err_max
		if (i == 0) {
			err_min = err_curr;
			err_max = err_curr;
		}
		else {
			if (err_curr < err_min) {
				err_min = err_curr;
			}
			if (err_curr > err_max) {
				err_max = err_curr;
			}
		}

	}

//	for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
//	{
//		//!Distance of the current particle
//		prcl = particleVect[i];
//// 		t0 = vpTime::measureTimeMs();
//		err_curr = prcl.compute_histDist(Icolor, object_ref, points2sub, weights);//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ! -> distance 1 lorsque la particule sort)
//// 		t1 = vpTime::measureTimeMs();
//// 		std::cout << "! Compute hist_dist time 1 prcl : " << t1-t0 << std::endl;
//		//TEST !!!
//// 		err_curr_global = prcl.compute_globHistDist(Icolor, object_ref, points2sub, weights);//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ?!)
//		//\TEST !
//
//		//! On compte les particules qui ne sont pas enti�rement dans l'image (pour ensuite d�tecter si l'objet est perdu)
//
//		if (prcl.get_nbPartOut() > 0) {//4) {// object_ref.get_nbPart()/2) {
//			nb_prcl_out ++;
//		}
//
//		//!Associate error with particle (could be done in ctParticle.cpp ?)
//		//!Note : Tester : set distance as a mean of err_curr and err_curr_global ?
//		prcl.setDist(err_curr);
//
//
//		//! err_min, err_max
//		if (i == 0) {
//			err_min = err_curr;
//			err_max = err_curr;
//		}
//		else {
//			if (err_curr < err_min) {
//				err_min = err_curr;
//			}
//			if (err_curr > err_max) {
//				err_max = err_curr;
//			}
//		}
//
//		err_sum += err_curr;
//
//		//! Update prcl
//		particleVect[i] = prcl;
//
//
//// 		t2 = vpTime::measureTimeMs();
//// 		std::cout << "minmax time : " << t2-t1 << std::endl;
//  }
}


*/

//!A tester:
/*!

	Likelihood computed  by comparing histogram of an object
	(La distance entre les histogrammes est calcul�e dans la fonction compare � partir du coefficient de Battacharyya)
	WARNING : effets de bords -> on exclut les particules qui sortent de l'image de plus de moiti� ?
*/
/*void ctPFilter::likelihood(const vpImage<vpRGBa> & Icolor, const ctObject<vpRGBa> & object_ref, std::list<ctPoint<vpRGBa> > &points2, bool weights)
{
	err_sum = 0;
	err_min = 0;
	err_max = 0 ;

	float err_curr = 0;
	ctParticle prcl;

	nb_prcl_out = 0; //nb de particules pas enti�rement dans l'image


	//!-------------------------Parcours de la liste de particules-----------------------//


	for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
	{
		//! Compute error
		prcl = particleVect[i];

		err_curr = prcl.compute_histDist(Icolor, object_ref, points2, weights);


		//! On compte les particules qui ne sont pas enti�rement dans l'image (pour ensuite d�tecter si l'objet est perdu)

		if (prcl.get_nbPartOut() > 0) {//4) {// object_ref.get_nbPart()/2) {
			nb_prcl_out ++;
		}

		//!Associate error with particle (could be done in ctParticle.cpp ?)
		//!Note : Tester : set distance as a mean of err_curr and err_curr_global ?
		prcl.setDist(err_curr);


		//! err_min, err_max
		if (i == 0) {
			err_min = err_curr;
			err_max = err_curr;
		}
		else {
			if (err_curr < err_min) {
				err_min = err_curr;
			}
			if (err_curr > err_max) {
				err_max = err_curr;
			}
		}

		err_sum += err_curr;

		//! Update prcl
		particleVect[i] = prcl;



	}//!End loop
}

*/

//!A tester:
/*!
	Likelihood computed  by comparing histogram of an object
	(La distance entre les histogrammes est calcul�e dans la fonction compare � partir du coefficient de Battacharyya)
	WARNING : effets de bords -> on exclut les particules qui sortent de l'image
*/
/*void ctPFilter::likelihood2(const vpImage<vpRGBa> & Icolor, const ctObject9k<vpRGBa> & object_ref, std::list<ctPoint<vpRGBa> > *points2sub, std::list<ctPoint<vpRGBa> > points2 , bool weights)
{
	err_sum = 0;
	err_min = 0;
	err_max = 0 ;

	float err_curr = 0;
	float err_curr_global (0);
	ctParticle prcl;

	nb_prcl_out = 0; //nb de particules pas enti�rement dans l'image

	double t0,t1,t2,t3;
	//!-------------------------Parcours de la liste de particules-----------------------//


	for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
	{
		//!Distance of the current particle
		prcl = particleVect[i];
		t0 = vpTime::measureTimeMs();
		err_curr = prcl.compute_histDist(Icolor, object_ref, points2sub, weights);//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ! -> distance 1 lorsque la particule sort)
		t1 = vpTime::measureTimeMs();
		std::cout << "compute hist_dist time : " << t1-t0 << std::endl;
		//TEST !!!
		err_curr_global = prcl.compute_globHistDist(Icolor, object_ref, points2, weights);//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ?!)
		//\TEST !
		t2 = vpTime::measureTimeMs();
		std::cout << "compute globHist_dist time : " << t2-t1 << std::endl;
		//! On compte les particules qui ne sont pas enti�rement dans l'image (pour ensuite d�tecter si l'objet est perdu)

		if (prcl.get_nbPartOut() > 0) {//4) {// object_ref.get_nbPart()/2) {
			nb_prcl_out ++;
		}

		//!Associate error with particle (could be done in ctParticle.cpp ?)
		//!Note : Tester : set distance as a mean of err_curr and err_curr_global ?
		prcl.setDist(err_curr);

		//! err_min, err_max
		if (i == 0) {
			err_min = err_curr;
			err_max = err_curr;
		}
		else {
			if (err_curr < err_min) {
				err_min = err_curr;
			}
			if (err_curr > err_max) {
				err_max = err_curr;
			}
		}

		err_sum += err_curr;

		//! Update prcl
		particleVect[i] = prcl;
		t3 = vpTime::measureTimeMs();
		std::cout << "min max time : " << t3-t2 << std::endl;
	}
}
//!******************************************************************************************************
//!------------------------------------------------------------------------------------------------------
//! --------------------------------------- Weights update ----------------------------------------------
//!******************************************************************************************************

/*!
	Computes and returns variance of distances
*/
double ctPFilter::compute_distVar()
{
	double dist_mean = err_sum/this->get_pNum();
	double distVar = 0;

	ctParticle *prcl;

	#pragma omp parallel for
	for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
	{
		distVar += (particleVect[i]->getDist()-dist_mean)*(particleVect[i]->getDist()-dist_mean);
	}
	distVar /= particleVect.size();
	return distVar;
}

/*!

	Mise � jour des poids � partir de la distance
	weight = exp(-20*distance�)
	Detect whether object is lost or not : if err_min > min_threshold, object is declared lost and all
	weights are set to 1;
	\param lost_prcl_ratio : pourcentage de particules sortant de l'image � partir duquel l'objet est d�clar� perdu

*/
void ctPFilter::weightsUpdate(const float & min_threshold, const float & lost_prcl_ratio)
{
	ctParticle *prcl;
	float wght = 0; float dist_curr = 0;
	best_weight = 0;

	float weight_cumul = 0;
	//! test if object is lost
	if ((err_min > min_threshold)||((float)nb_prcl_out/(float)this->get_pNum() > lost_prcl_ratio)) {
		std::cout << "erreur min : " << err_min << std::endl;
		std::cout << "min threshold was : " << min_threshold  << std::endl;
		std::cout << "ratio lost particles : " << (float)nb_prcl_out/(float)this->get_pNum() << std::endl;
		std::cout << "min threshold ratio was : " << lost_prcl_ratio << std::endl;
		object_lost = true;

		//#pragma omp parallel for
		for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
		{
			weight_cumul += 1;
			particleVect[i]->setWeight(1);
			particleVect[i]->setWghtCumul(weight_cumul);
		}
		best_weight = 1;// Memorise best weight
		this->setWght_sum(weight_cumul);
	}
	//! object is not lost
	else {
		object_lost = false;

		for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
		{
			prcl = particleVect[i];
			dist_curr = prcl->getDist();

		//Calcul nouveau poids
			if (err_max-err_min == 0)// Si toutes les distances sont les m�mes, ts les poids sont mis �gaux � 1
				wght = 1;
				//wght = 0;
			else
			{
// 				wght= exp(-3*(dist_curr-err_min)/(err_max-err_min)*(dist_curr-err_min)/(err_max-err_min)); // Somme ne vaut pas 1 !!
// 				wght = exp(-0.5*dist_curr*dist_curr);// Somme ne vaut pas 1 !!
				wght = exp(-0.001*dist_curr*dist_curr);// Somme ne vaut pas 1 !!
				//wght = 1 - ((dist_curr-err_min)/(err_max-err_min)*(dist_curr-err_min)/(err_max-err_min));// Somme ne vaut pas 1 !!
				//wght =  (err_sum-dist_curr)/(err_sum*(particleVect.size()-1));// mesure de vraisemblance lin�aire - Somme = 1
			}
			if (wght > best_weight)
				{best_weight = wght;// Memorise best weight
				//orieff = prcl->getOri();
				}
			weight_cumul += wght;
			prcl->setWeight(wght);
			//std::cout << " weights " << wght << std::endl;
			prcl->setWghtCumul(weight_cumul);
			//particleVect[i] = prcl;

		}
		//std::cout << " ok00 " << orieff << std::endl;
		//std::cout << "best weight : " << best_weight << std::endl;
		this->setWght_sum(weight_cumul);
	}
}


//!******************************************************************************************************
//!------------------------------------------------------------------------------------------------------
//! --------------------------------------- Resample ----------------------------------------------
//!******************************************************************************************************


/*!

	Tirage al�atoire pond�r� - Random weighted draw
	\param nb : Nombre de particules tir�es

*/
void ctPFilter::weightedDraw(const int & nb)//WARNING FUITE MEMOIRE
{
	ctParticle *prcl;
	ctParticle *prcl1;
	std::vector<ctParticle>  newParticleVect;
	int nb_tirage = 0;
	int count = 0;


	for (int i = 0 ; i < nb; i++)// Parcours de la liste de particules
	{
		nb_tirage = (int)(particleVect.size() * (particleVect[i]->getWeight())/(this->getWght_sum()));
		//std::cout << "nb_tirages " << nb_tirage << std::endl;
		for (int j = 0 ; j < nb_tirage ; j++)
		{
			prcl = particleVect[i];
			prcl->set_u(prcl->get_uPred());
			prcl->set_v(prcl->get_vPred());
			prcl->set_angle(prcl->get_anglePred());
			prcl->set_size(prcl->get_sizePred());
			prcl->setWeight(1);
    		newParticleVect.push_back(*prcl);

			count ++;
		}
	}
	//std::cout << "size parck vect " << newParticleVect.size() << std::endl;

// Compl�ment : tirage orient�
	double tirage;
	while (count < particleVect.size())
	{

		tirage = (double)rand()/((double)RAND_MAX + 1)*(this->getWght_sum());//Nombre al�atoire entre 0 et la somme des poids
		//std::cout << "tirage " <<tirage << std::endl;
		for (int k = 0 ; k < particleVect.size() ; k++)
		{
			prcl = particleVect[k];
			if (prcl->getWghtCumul() > tirage) {
				break;
			}

		}
		prcl->set_u(prcl->get_uPred());
		prcl->set_v(prcl->get_vPred());
		prcl->set_angle(prcl->get_anglePred());
		prcl->set_size(prcl->get_sizePred());
		prcl->setWeight(1);
		newParticleVect.push_back(*prcl);
		count++;
	}

	//particleVect = newParticleVect;
	for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
	{
	*particleVect[i] = newParticleVect[i];
	}
	//std::cout << "size parck vect " << particleVect.size() << " ok " << newParticleVect.size() << std::endl;
}



/*!

	Tirage al�atoire non pond�r� - Random draw (not weighted !)
	In fact, simply set new particle list as predicted particles list

*/
void ctPFilter::simpleDraw()
{
	ctParticle *prcl;



	for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
	{
		prcl = particleVect[i];
		prcl->set_u(prcl->get_uPred());
		prcl->set_v(prcl->get_vPred());
		prcl->set_angle(prcl->get_anglePred());
		prcl->set_size(prcl->get_sizePred());
		prcl->setWeight(1);

    	//particleVect[i] = prcl;


	}
}


//!******************************************************************************************************
//!------------------------------------------------------------------------------------------------------
//! --------------------------------------- Estimate ----------------------------------------------
//!******************************************************************************************************


/*!

	Estimation du vecteur d'�tat (moyenne pond�r�e sur les particules)
	Note : Update velocity in the same time

*/
void ctPFilter::estimateMean()// Sur les valeurs pr�dites ! (avant tirage)
{
	float u_cum = 0; float v_cum = 0 ; float angle_cum = 0; double size_cum = 0;
	ctParticle *ptcl, last_pEstim;

	last_pEstim = pEstimate; // Memorise last estimate before computing the new one


	for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
	{
		ptcl = particleVect[i];
		u_cum += ptcl->get_uPred()*ptcl->getWeight();
		v_cum += ptcl->get_vPred()*ptcl->getWeight();
		angle_cum += ptcl->get_anglePred()*ptcl->getWeight();
		size_cum += ptcl->get_sizePred()*ptcl->getWeight();
		;
	}

	u_cum /= this->getWght_sum();
	v_cum /= this->getWght_sum();
	angle_cum /= this->getWght_sum();
	size_cum /= this->getWght_sum();
	pEstimate.set_u((int)u_cum);
	pEstimate.set_v((int)v_cum);
	pEstimate.set_angle(angle_cum);
	pEstimate.set_size(size_cum);

	//pEstimate = *ptcl;// Update estimate
	//std::cout << "u_cum " << u_cum << " v_cum " << v_cum << " angle_cum " << angle_cum << " size_cum " << size_cum << std::endl;

	// Update  current velocity :
	this->set_vel(u_cum - last_pEstim.get_u(), v_cum - last_pEstim.get_v(), angle_cum - last_pEstim.get_angle(), (last_pEstim.get_size()-size_cum)/(2.*(last_pEstim.get_size()/prcl_ini.get_size()*sqrt(last_pEstim.get_size()/prcl_ini.get_size()))), size_cum-last_pEstim.get_size(),size_cum/last_pEstim.get_size());
}


/*!

	Estimation du vecteur d'�tat (Simple mean of all particles, without weights)
	Note : Update velocity in the same time

*/
void ctPFilter::estimateMeanSimple()
{
	float u_cum = 0; float v_cum = 0 ; float angle_cum = 0; float size_cum = 0;
	ctParticle *ptcl, last_pEstim;

	last_pEstim = pEstimate; // Memorise last estimate before computing the new one


	for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
	{
		ptcl = particleVect[i];
		u_cum += ptcl->get_uPred();
		v_cum += ptcl->get_vPred();
		angle_cum += ptcl->get_anglePred();
		size_cum += ptcl->get_sizePred();

	}

	u_cum /= this->get_pNum();
	v_cum /= this->get_pNum();
	angle_cum /= this->get_pNum();
	size_cum /= this->get_pNum();
	pEstimate.set_u((int)u_cum);
	pEstimate.set_v((int)v_cum);
	pEstimate.set_angle(angle_cum);
	pEstimate.set_size(size_cum);

	//pEstimate = ptcl;// Update estimate

	std::cout << "u_cum " << u_cum << " v_cum " << v_cum << " angle_cum " << angle_cum << " size_cum " << size_cum << std::endl;
	// Update  current velocity :
	this->set_vel(u_cum - last_pEstim.get_u(), v_cum - last_pEstim.get_v(), angle_cum - last_pEstim.get_angle(), (last_pEstim.get_size()-size_cum)/(2.*(last_pEstim.get_size()/prcl_ini.get_size()*sqrt(last_pEstim.get_size()/prcl_ini.get_size()))), size_cum-last_pEstim.get_size(),size_cum/last_pEstim.get_size());
}



/*!

  Estimation du vecteur d'�tat (meilleure particule)
  Compute best_particle (weightsUpdate function must have been called before that)
  Set pEstimate = best_particle
  Note : Update velocity in the same time

*/
void ctPFilter::estimateBest()// Sur les valeurs pr�dites pond�r�es ! (avant tirage)
{
	ctParticle *ptcl, last_pEstim;
	last_pEstim = pEstimate; // Memorize last estimate before computing the new one


	ptcl = particleVect[0];
	int i = 0;
	while ((ptcl->getWeight() != best_weight) && (i < particleVect.size()))// Parcours de la liste de particules
	{
		ptcl = particleVect[i];

		i++;
	}
	if (ptcl->getWeight() == best_weight)
	{
		best_particle.set_u(ptcl->get_uPred());
		best_particle.set_v(ptcl->get_vPred());
		best_particle.set_angle(ptcl->get_anglePred());
		best_particle.set_size(ptcl->get_sizePred());
		best_particle.setOri(orieff);
	}
	else std::cout << "Error : no particle of weight = best_weight" << std::endl;

	pEstimate = best_particle;// Update Estimate
	std::cout << "u_cum " << pEstimate.get_u() << " v_cum " << pEstimate.get_v() << " angle_cum " << pEstimate.get_angle() << " size_cum " << pEstimate.get_size() << std::endl;
	// Update  current velocity :
	this->set_vel((float)best_particle.get_u() - last_pEstim.get_u(), (float)best_particle.get_v() - last_pEstim.get_v(), (float)best_particle.get_angle() - last_pEstim.get_angle(), (last_pEstim.get_size()-best_particle.get_size())/(2.*(last_pEstim.get_size()/prcl_ini.get_size()*sqrt(last_pEstim.get_size()/prcl_ini.get_size()))), (float)best_particle.get_size()-last_pEstim.get_size(),(float)best_particle.get_size()/last_pEstim.get_size());
}

/*!

  Estimation du vecteur d'�tat (meilleure particule)
  Compute best_particle (weightsUpdate function must have been called before that)
  Set pEstimate = best_particle
  Note : Update velocity in the same time
  WARNING !! Faire moyenne pond�r�e !!

*/
// void ctPFilter::estimateBest(int percentage)// Sur les valeurs pr�dites pond�r�es ! (avant tirage)
// {
// 	std::cout << "WARNING !! Moyenne pond�r�e !! Pas encore test�"<< std::endl;
// 	if (percentage > 100)
// 		vpTRACE("percentage must be inferior to 100 !");
// 	int num_best = percentage * particleVect.size() /100;
// 	if (num_best == 0)
// 		num_best++;
//
// 	float wghtS = 0;
//
// 	ctParticle ptcl, last_pEstim;
// 	float u_cum = 0; float v_cum = 0 ; float angle_cum = 0; float size_cum = 0;
// 	last_pEstim = pEstimate; // Memorize last estimate before computing the new one
//
// 	std::list<ctParticle> particleVector = ctTools::triFusion(* particleList);
// 	for (int i =0; i < num_best; i++)
// 	{
// 		ptcl = particleVector[i];
// // 		u_cum += ptcl.get_uPred();
// // 		v_cum += ptcl.get_vPred();
// // 		angle_cum += ptcl.get_anglePred();
// // 		size_cum += ptcl.get_sizePred();
// 		u_cum += ptcl.get_uPred()*ptcl.getWeight();
// 		v_cum += ptcl.get_vPred()*ptcl.getWeight();
// 		angle_cum += ptcl.get_anglePred()*ptcl.getWeight();
// 		size_cum += ptcl.get_sizePred()*ptcl.getWeight();
// 		wghtS += ptcl.getWeight();
// 	}
//
// // 	u_cum /= num_best;
// // 	v_cum /= num_best;
// // 	angle_cum /= num_best;
// // 	size_cum /= num_best;
// 	u_cum /= wghtS;
// 	v_cum /= wghtS;
// 	angle_cum /= wghtS;
// 	size_cum /= wghtS;
//
// 	ptcl.set_u((int)u_cum);
// 	ptcl.set_v((int)v_cum);
// 	ptcl.set_angle(angle_cum);
// 	ptcl.set_size(size_cum);
//
// 	pEstimate = ptcl;// Update Estimate
//
// 	// Update  current velocity :
// 	this->set_vel(u_cum - last_pEstim.get_u(), v_cum - last_pEstim.get_v(), angle_cum - last_pEstim.get_angle(), sqrt(size_cum) - sqrt(last_pEstim.get_size()));
// }

/*void ctPFilter::run(const vpImage<unsigned char> & Igray, const ctObject9k<unsigned char> & object_ref, std::list<ctPoint<unsigned char> > *points2sub, bool weights)
{
	vpTRACE("A FINIR ET TESTER !!!!!!!!!");

	//!Init parameters
	err_sum = 0;
	err_min = 0;
	err_max = 0 ;

	float err_curr = 0;
// 	float err_curr_global (0);

	float stepmax = STEPMAXRECT;
	float anglemax = ANGLEMAXRECT;
	float scalemax = SCALEMAXRECT;
	nb_prcl_out = 0; //nb de particules pas enti�rement dans l'image

	bool outside = false;
	int it =0;

	//!Parcours de la liste de particules
	ctParticle prcl;



	for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
	{
		prcl = particleVect[i];//V�rifier l'op�rateur '=' !

		//!---------------------- Prediction --------------------------
		prcl.pred_rdmUStep(stepmax, anglemax, scalemax);


		//!---------------------- On complete si trop de particules sortent --------------------------
		while((outside == true)&&(it <5))
		{
			outside =((prcl.get_uPred() < 0)||(prcl.get_uPred()>Igray.getWidth())|| (prcl.get_vPred()<0) || (prcl.get_vPred()>Igray.getHeight()));
			it++;
		}

		//!Distance of the current particle
		err_curr = prcl.compute_histDist(Igray, object_ref, points2sub, weights);//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ! -> distance 1 lorsque la particule sort)

		//TEST !!!
// 		err_curr_global = prcl.compute_globHistDist(Igray, object_ref, points2, weights);//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ?!)
		//\TEST !

		//! On compte les particules qui ne sont pas enti�rement dans l'image (pour ensuite d�tecter si l'objet est perdu)

		if (prcl.get_nbPartOut() > 0) {//4) {// object_ref.get_nbPart()/2) {
			nb_prcl_out ++;
		}

		//!Associate error with particle (could be done in ctParticle.cpp ?)
		//!Note : Tester : set distance as a mean of err_curr and err_curr_global ?
		prcl.setDist(err_curr);


		//! err_min, err_max
		if (i == 0) {
			err_min = err_curr;
			err_max = err_curr;
		}
		else {
			if (err_curr < err_min) {
				err_min = err_curr;
			}
			if (err_curr > err_max) {
				err_max = err_curr;
			}
		}

		err_sum += err_curr;
		particleVect[i] = prcl;

	}

}

*/

// void ctPFilter::run(const vpImage<vpRGBa> & Icolor, const ctObject9k<vpRGBa> & object_ref, std::list<ctPoint<vpRGBa> > *points2sub, bool weights)
// {
// 	vpTRACE("A ECRIRE !!!!!!!!!");
// }


void ctPFilter::show_weights()
{
	//!Parcours de la liste de particules
	ctParticle *prcl;


	for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
	{
		prcl = particleVect[i];//V�rifier l'op�rateur '=' !
		std::cout << i << "  :" << prcl->getWeight() << std::endl;

	}

}

double ctPFilter::compute_Neff()
{
	double wght_sum = 0;
	double wght_squared = 0;

	//!Parcours de la liste de particules
	ctParticle *prcl;


	for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
	{
		prcl = particleVect[i];//V�rifier l'op�rateur '=' !
		wght_sum += prcl->getWeight();
		wght_squared += prcl->getWeight()*prcl->getWeight();

	}

	double Neff = wght_sum*wght_sum/wght_squared;
	return Neff;

}

double ctPFilter::compute_variance()
{
	double wght_mean = 0;
	double wght_var = 0;

	//!Parcours de la liste de particules
	ctParticle *prcl;

	for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
	{
		prcl = particleVect[i];//V�rifier l'op�rateur '=' !
		wght_mean += prcl->getWeight();

	}
	wght_mean /= particleVect.size();


	for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
	{
		prcl = particleVect[i];//V�rifier l'op�rateur '=' !
		wght_var += (prcl->getWeight()-wght_mean)*(prcl->getWeight()-wght_mean);

	}
	wght_var /= particleVect.size();
	return wght_var;
}


