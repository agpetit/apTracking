/*
 * apMbTracker.cpp
 *
 *  Created on: March 10, 2011
 *      Author: Antoine Petit
 */

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#include <visp/vpPose.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpImageIo.h>
#include <visp/vpRobust.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpMatrixException.h>
#include <visp/vpMatrix.h>
#include <visp/vpImagePoint.h>
#include <visp/vpFeatureBuilder.h>

#include <visp/vpException.h>
#include <visp/vpTrackingException.h>
#include <visp/vpVideoWriter.h>

#include "apMbTracker.h"
#include "apControlPoint.h"
#include "apKltControlPoint.h"
//#include "apViews.h"

#include <visp/vpImageTools.h>

#include <fstream>
#include <cstring>
#include <cstdio>
#include <iostream>

#include "luaconfig.h"

#include <string>
#include <sstream>

/*!
 Basic constructor
 */
apMbTracker::apMbTracker() {
	compute_interaction = 1;
	npoints = 0;
	lambda = 1;
	percentageGdPt = 0.1;
	frame = 0;
	frame0 = 0;
	computeCovariance = false;
	points.resize(1);
	scales.resize(1);
	scales[0] = true;
	points[0].resize(0);
	lines.resize(1);
	lines[0].resize(0);
	lines_cand.resize(1);
	lines_cand[0].resize(0);
	Ipyramid.resize(0);
	IRGBpyramid.resize(0);
	meantime = 0;
	nkltPoints = 0;
	kltPoints.resize(1);
	kltPoints[0] = std::map<int, apKltControlPoint>();
	predictKLT = false;
	covarianceMatrix.resize(6,6);
	covarianceMatrixME.resize(6,6);
	covarianceMatrixCCD.resize(6,6);
	covarianceMatrixKLT.resize(6,6);
	covarianceMatrixP.resize(6,6);
	covarianceMatrixL.resize(6,6);
	timetrack = 0;
	timetrackklt =0;
	timeextractklt = 0;
	timeextract = 0;
	timevvs =0;
	sigmag = 0.7;
	sigmap = 10;
        controlpoints.resize(0);


}

/*!
 Basic destructor useful to deallocate the memory.
 */
apMbTracker::~apMbTracker() {
	apControlPoint *p;

	for (unsigned int i = 0; i < points.size(); i += 1) {
		if (scales[i]) {
			for (int k = 0; k < points[i].size(); k++) {
				p = (points[i])[k];
				if (p != NULL)
					delete p;
				p = NULL;
			}
			points[i].resize(0);
		}
	}
	points.resize(0);
	cleanPyramid(Ipyramid);

	for (unsigned int i = 0; i < CCDTracker.pointsCCD.size(); i += 1) {
		if (scales[i]) {
			for (int k = 0; k < CCDTracker.pointsCCD[i].size(); k++) {
				p = (CCDTracker.pointsCCD[i])[k];
				if (p != NULL)
					delete p;
				p = NULL;
			}
			CCDTracker.pointsCCD[i].resize(0);
		}
	}
	CCDTracker.pointsCCD.resize(0);
	cleanPyramid(IRGBpyramid);

	apMbtDistanceLineMH *l;
	for (unsigned int i = 0; i < lines.size(); i += 1) {
		if (scales[i]) {
			for (int k = 0; k < points[i].size(); k++) {
				l = lines[i][k];
				for (int ii; ii < l->pointsvect.size(); ii++) {
					p = l->pointsvect[ii];
					delete p;
					p = NULL;
				}
				l->pointsvect.resize(0);
				if (l != NULL)
					delete l;
				l = NULL;
			}

			lines[i].clear();
		}
	}
	lines.resize(0);
#ifdef ENABLE_ZMQ
        delete m_socketPub;
        delete m_socketSub;
#endif // ENABLE_ZMQ
}


/*void
 apMbTracker::setOgre(const exampleVpAROgre &ogre_)
 {
 ogre = ogre_;
 }*/

/*!
 Set the moving edge parameters.

 \param _me : an instance of vpMe containing all the desired parameters
 */
void apMbTracker::setMovingEdge(const vpMe &_me) {
	this->me = _me;

	for (unsigned int i = 0; i < points.size(); i += 1) {
		if (scales[i]) {
			apControlPoint *p;
			for (int k = 0; k < points[i].size(); k++) {
				p = (points[i])[k];
				p->setMovingEdge(&me);
			}
		}
	}
}

/*!
 Compute the visual servoing loop to get the pose of the feature set.

 \exception vpTrackingException::notEnoughPointError if the number of detected
 feature is equal to zero.

 \param _I : The current image.
 */
void apMbTracker::computeVVS(const vpImage<unsigned char>& _I) {
	double residu_1 = 1e3;
	double r = 1e3 - 1;
	vpMatrix LTL;
	vpColVector LTR;

	// compute the interaction matrix and its pseudo inverse
	apControlPoint *p;

	vpColVector w;
	vpColVector weighted_error;
	vpColVector factor;

	unsigned int iter = 0;

	//Nombre de moving edges
	int nbrow = 0;
	vpFeatureLine fli;

	for (int k = 0; k < points[scaleLevel].size(); k++) {
		p = (points[scaleLevel])[k];
		nbrow += 1;
		p->initInteractionMatrixError();
		//
		//std::cout<<fli.getTheta()<<std::endl;
	}

	if (nbrow == 0) {
		vpERROR_TRACE(
				"\n\t\t Error-> not enough data in the interaction matrix...");
		throw vpTrackingException(vpTrackingException::notEnoughPointError,
				"\n\t\t Error-> not enough data in the interaction matrix...");
	}

	vpMatrix L(nbrow, 6), Lp;

	// compute the error vector
	vpColVector error(nbrow);
	int nerror = error.getRows();
	vpColVector v;

	double limite = 3; //Une limite de 3 pixels
	limite = limite / cam.get_px(); //Transformation limite pixel en limite metre.

	//Parametre pour la premiere phase d'asservissement
	double e_prev = 0, e_cur, e_next;
	bool reloop = true;
	double count = 0;

	/*** First phase ***/

	while (reloop == true && iter < 3) {
		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 0;
			factor.resize(nerror);
			factor = 1;
		}

		count = 0;
		//;
		int n = 0;
		//reloop = false;
		reloop = true;
//#pragma omp parallel for
		for (int k = 0; k < points[scaleLevel].size(); k++)
		//for (int k = 0; k<points[scaleLevel].size(); k++)
		{
			//p = (points[scaleLevel])[k];
			p = (points[scaleLevel])[k];
			p->computeInteractionMatrixError(cMo, _I);
			//p->computeInteractionMatrixError2(cMo,_I);
			//p->computeInteractionMatrixError4(cMo, I ,Igrad , Igradx, Igrady);

			double fac = 1;
			/*if (iter == 0)
			 {
			 fac = 0.2;
			 break;
			 }*/

			//std::cout << " fac " << fac << std::endl;

			if (iter == 0 && p != NULL)

				for (int j = 0; j < 6; j++) {
					L[n][j] = p->L[j]; //On remplit la matrice d'interaction globale
				}
			error[n] = p->error; //On remplit la matrice d'erreur

			if (error[n] <= limite)
				count = count + 1.0; //Si erreur proche de 0 on incremente cur

			w[n] = 1;

			if (iter == 0) {
				factor[n] = fac;
				vpPointSite site = p->s;
				//if (site.suppress != 0) factor[n] = 0;
				if (site.suppress != 0)
					factor[n] = 0.2;
			}
			/*
			 //else
			 //{
			 e_cur = p->error;
			 e_next = p->error;
			 if (fabs(e_cur - e_prev) < limite) {
			 w[n] += 0.5;
			 //w[n]=1;
			 }
			 if (fabs(e_cur - e_next) < limite) {
			 w[n] += 0.5;
			 //w[n]=1;
			 }
			 e_prev = e_cur;
			 //}
			 */
			n += 1;
			//
		}

		count = count / (double) nbrow;
		if (count < 0.85) {
			reloop = true;
		}

		double num = 0;
		double den = 0;

		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			eri = error[i];
			num += wi * vpMath::sqr(eri);
			den += wi;

			weighted_error[i] = wi * eri;
		}

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					L[i][j] = w[i] * factor[i] * L[i][j];
				}
			}
		}

		LTL = L.AtA();
		computeJTR(L, weighted_error, LTR);
		v = -0.7 * LTL.pseudoInverse(LTL.getRows() * DBL_EPSILON) * LTR;
		cMo = vpExponentialMap::direct(v).inverse() * cMo;

		//std::cout << " cmo iter " << cMo << std::endl;

		iter++;
	}
	std::cout << "\t First minimization in " << iter << " iteration "
			<< std::endl;

	/*** Second phase ***/
	vpColVector W_true;
	vpMatrix L_true;
	vpRobust robust(nerror);
	robust.setIteration(0);
	iter = 0;
	//vpColVector error_px(nerror);

	while ( (iter < 10)) {
		// ;
		int n = 0;
//#pragma omp parallel for
		for (int k = 0; k < points[scaleLevel].size(); k++)
		//for (int k = 0; k<points[scaleLevel].size(); k++)
		{
			//p = (points[scaleLevel])[k];
			p = (points[scaleLevel])[k];
			p->computeInteractionMatrixError(cMo, _I);
			//p->computeInteractionMatrixError2(cMo,_I);
			//p->computeInteractionMatrixError4(cMo, I ,Igrad , Igradx, Igrady);
			for (int j = 0; j < 6; j++) {
				L[n][j] = p->L[j];
				error[n] = p->error;
				//error_px[n+i] = l->error[i] * cam.get_px();
			}
			n += 1;
			//
		}

		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 1;

			robust.setThreshold(2 / cam.get_px()); // limite en metre
			robust.MEstimator(vpRobust::TUKEY, error, w);
			//robust.setThreshold(2); // limite en pixel
			//robust.MEstimator(vpRobust::TUKEY, error_px,w);
		} else {
			robust.setIteration(iter);
			robust.MEstimator(vpRobust::TUKEY, error, w);
			//robust.MEstimator(vpRobust::TUKEY, error_px,w);
		}

		residu_1 = r;

		L_true = L;
		W_true = vpColVector(nerror);

		double num = 0;
		double den = 0;
		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			eri = error[i];
			W_true[i] = wi * wi;
			num += wi * vpMath::sqr(eri);
			den += wi;

			weighted_error[i] = wi * eri;
		}

		r = sqrt(num / den); //Le critere d'arret prend en compte le poids

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					L[i][j] = w[i] * factor[i] * L[i][j];
				}
			}
		}

		LTL = L.AtA();
		computeJTR(L, weighted_error, LTR);
		v = -lambda * LTL.pseudoInverse(LTL.getRows() * DBL_EPSILON) * LTR;
		cMo = vpExponentialMap::direct(v).inverse() * cMo;

		//std::cout << " cmo iter " << cMo << std::endl;

		iter++;
	}
	if (computeCovariance) {
		vpMatrix D;
		D.diag(W_true);
		covarianceMatrixME = computeCovarianceMatrix(L_true, -v, lambda * error,
				D);
		covarianceMatrix = covarianceMatrixME;
	}

	int n = 0;
	for (int k = 0; k < points[scaleLevel].size(); k++) {
		p = (points[scaleLevel])[k];
		{
			double wmean = 0;
			/*if (p->nbFeature > 0)*///

			//for (int i=0 ; i < p->nbFeature ; i++)
			//{
			wmean += w[n];
			vpPointSite site = p->s;
			if (w[n] < 0.5) {
				site.suppress = 4;
				p->s = site;
			}

			//}
			n += 1;

			wmean = 1;

			//p->setMeanWeight(wmean);

		}

	}
	//   cout << "\t Robust minimization in " << iter << " iteration " << endl ;
	//    std::cout << "error: " << (residu_1 - r) << std::endl;
}

/*void
 apMbTracker::computeVVSHyb(const vpImage<unsigned char>& _I, 	apOgre ogre_)
 {
 double residu_1 =1e3;
 double r =1e3-1;
 vpMatrix LTL;
 vpColVector LTR;

 // compute the interaction matrix and its pseudo inverse
 apControlPoint *p ;

 vpColVector w;
 vpColVector weighted_error;
 vpColVector factor;

 vpTranslationVector tr;
 cMo.extract(tr);

 unsigned int iter = 0;

 //Nombre de moving edges
 int nbrow  = 0;
 vpFeatureLine fli;

 ;
 for (int k = 0; k<points[scaleLevel].size(); k++)
 {
 p = (points[scaleLevel])[k];
 nbrow += 1;
 p->initInteractionMatrixError();

 //std::cout<<fli.getTheta()<<std::endl;
 }

 if (nbrow==0)
 {
 vpERROR_TRACE("\n\t\t Error-> not enough data in the interaction matrix...") ;
 throw vpTrackingException(vpTrackingException::notEnoughPointError, "\n\t\t Error-> not enough data in the interaction matrix...");
 }

 vpMatrix L(nbrow,6), Lsd, LT;
 // matrice d'interaction a la position desiree
 vpMatrix Hsd;  // hessien a la position desiree
 vpMatrix H ; // Hessien utilise pour le levenberg-Marquartd

 vpColVector errorG ;

 // compute the error vector
 vpColVector error(nbrow);
 int nerror = error.getRows();
 int nerrorG;
 vpColVector v ;

 double limite = 3; //Une limite de 3 pixels
 limite = limite / cam.get_px(); //Transformation limite pixel en limite metre.

 //Parametre pour la premiere phase d'asservissement
 double e_prev = 0, e_cur, e_next;
 bool reloop = true;
 double count = 0;
 int nbr =480;
 int nbc = 640;
 vpImage<unsigned char> imG(480,640);
 vpImage<unsigned char> Igd(480,640);
 vpImage<unsigned char> Ig(480,640);
 vpImage<unsigned char> Idiff(480,640);
 vpColVector e;

 for (int i=3; i < nbr-3 ; i++)
 {
 //   cout << i << endl ;
 for (int j = 3 ; j < nbc-3; j++)
 {
 // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
 double Igf =   vpImageFilter::gaussianFilter(_I,i,j) ;
 imG[i][j] = Igf ;

 }
 }*/

/*  for (int i=3; i < nbr-3 ; i++)
 {
 for (int j = 3 ; j < nbc-3; j++)
 {*/
/*  for (int i=150; i < 350 ; i++)
 {
 for (int j = 150 ; j < 550; j++)
 {
 // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
 double Ix =   1 * vpImageFilter::sobelFilterX(imG,i,j);
 double Iy =   1 * vpImageFilter::sobelFilterY(imG,i,j);
 Igd[i][j]= (unsigned char)sqrt(vpMath::sqr(Ix)+vpMath::sqr(Iy));
 //I2[i][j] = (unsigned char)Igrad[i][j];
 }
 }

 sId.init(Igd.getHeight(), Igd.getWidth(), tr[2]);
 sI.init(Igd.getHeight(), Igd.getWidth(), tr[2]);
 sId.buildFrom(Igd);
 sId.interaction(Lsd);
 //Lsd=2*Lsd;
 nerrorG = Lsd.getRows();
 vpColVector errorT(nerror+nerrorG);
 vpDisplayX displayo;
 displayo.init(Idiff, 1500, 1000, "display");
 double mu = 0.003;
 vpMatrix diagHsd(6,6);
 vpMatrix diagLTL(6,6);


 Hsd = Lsd.AtA();
 diagHsd.eye(6);
 for(int i = 0 ; i < 6 ; i++) diagHsd[i][i] = Hsd[i][i];

 H = ((mu * diagHsd) + Hsd).pseudoInverse();

 /*** First phase ***/

/*  while ( reloop == true && iter<30)
 {

 cMo.extract(tr);
 ogre_.updateClipDistances0(tr[2],1);
 ogre_.updateRTTGrad(Ig,&cMo);
 sI.update(Ig.getHeight(), Ig.getWidth(),tr[2]);
 sI.buildFrom(Ig);
 sI.error(sId, errorG);

 vpImageTools::imageDifference(Ig,Igd,Idiff);
 vpDisplay::display(Idiff);
 vpDisplay::flush(Idiff);*/

/*H = ((mu * diagHsd) + Hsd).pseudoInverse();
 //	compute the control law
 e = H * Lsd.t() *errorG;
 v =  -0.5*e;
 cMo =  vpExponentialMap::direct(v).inverse() * cMo;*/

/*	  if(iter==0)
 {
 weighted_error.resize(nerror+nerrorG) ;
 w.resize(nerror+nerrorG);
 w = 0;
 factor.resize(nerror+nerrorG);
 factor = 1;
 }*/

/*  if(iter==0)
 {
 weighted_error.resize(nerror) ;
 w.resize(nerror);
 w = 0;
 factor.resize(nerror);
 factor = 1;
 }*/

/*count = 0;
 ;
 int n = 0;
 //reloop = false;
 reloop = true;

 for (int k = 0; k<points[scaleLevel].size(); k++)
 {
 p = (points[scaleLevel])[k];
 p->computeInteractionMatrixError(cMo,_I);
 //p->computeInteractionMatrixError2(cMo,_I);

 double fac = 1;
 if (iter == 0)
 {
 fac = 0.2;
 break;
 }

 //std::cout << " fac " << fac << std::endl;

 if (iter == 0 && p != NULL)
 p->list.front();

 //for (int i=0 ; i < 1 ; i++)
 //{
 for (int j=0; j < 6 ; j++)
 {
 L[n][j] = p->L[j]; //On remplit la matrice d'interaction globale
 }
 error[n] = p->error; //On remplit la matrice d'erreur

 if (error[n] <= limite) count = count+1.0; //Si erreur proche de 0 on incremente cur

 w[n] = 0;

 if (iter == 0)
 {
 factor[n] = fac;
 vpPointSite site = p->list.value();
 if (site.suppress != 0) factor[n] = 0.2;
 p->list.next();
 }

 e_cur = p->error;
 e_next = p->error;
 if ( fabs(e_cur - e_prev) < limite )
 {
 w[n] += 0.5;
 }
 if ( fabs(e_cur - e_next) < limite )
 {
 w[n] += 0.5;
 }
 e_prev = e_cur;


 n+= 1 ;

 }

 count = count / (double)nbrow;
 if (count < 0.85)
 {
 reloop = true;
 }

 double num=0;
 double den=0;

 double wi; double eri;
 for(int i = 0; i < nerror; i++)
 {
 wi = w[i]*factor[i];
 eri = error[i];
 num += wi*vpMath::sqr(eri);
 den += wi ;

 weighted_error[i] =50000* wi*eri;
 //weighted_error[i] =  wi*eri;
 //weighted_error[i] =  0*wi*eri;
 }
 for(int i = nerror; i < nerror+nerrorG; i++)
 {
 weighted_error[i] =  3*errorG[i-nerror];
 }


 if((iter==0) || compute_interaction)
 {
 for (int i=0 ; i < nerror ; i++)
 {
 for (int j=0 ; j < 6 ; j++)
 {
 L[i][j] = 50000*w[i]*factor[i]*L[i][j] ;
 //L[i][j] = w[i]*factor[i]*L[i][j] ;
 //L[i][j] = 0*w[i]*factor[i]*L[i][j] ;
 }
 }
 }


 vpMatrix::stackMatrices(L,Lsd,LT);*/
//LT=L;


/* H = ((mu * diagHsd) + Hsd).pseudoInverse();
 H1 = ((mu * diagHsd1) + Hsd1).pseudoInverse();
 //	compute the control law
 e = H * Lsd.t() *error;
 e1 = H1 * Lsd1.t() *error1;
 double normeError = (error1.sumSquare());
 //A.plot(0,0,iter,sqrt((normeError)/(I3.getHeight()*I3.getWidth())));
 //if (normeError < 0.5e7) lambda = 1 ;
 v =  -lambda*e;*/

/*double t0= vpTime::measureTimeMs();
 LTL = LT.AtA();
 double t1= vpTime::measureTimeMs();
 std::cout<<"timeiter"<< t1-t0<<std::endl;
 diagLTL.eye(6);
 for(int i = 0 ; i < 6 ; i++) diagLTL[i][i] = LTL[i][i];
 LTL=LTL + 0.003*diagLTL;
 //std::cout << " ltl " << LTL.pseudoInverse() << std::endl;
 //std::cout<<  " H " << H << std::endl;
 //computeJTR(LT, weighted_error, LTR);
 //v = -0.2*LTL.pseudoInverse()*LTR;
 v = -0.5*(LTL.pseudoInverse())*LT.t()*weighted_error;
 //std::cout << v << std::endl;
 cMo =  vpExponentialMap::direct(v).inverse() * cMo;


 iter++;
 }
 std::cout << "\t First minimization in " << iter << " iteration00 " << std::endl ;*/

//std::cout<<iter<<std::endl;
/*   ;
 int n =0 ;
 for (int k = 0; k<points[scaleLevel].size(); k++)
 {
 p = (points[scaleLevel])[k] ;
 {
 double wmean = 0 ;
 //if (p->nbFeature > 0)
 p->list.front();

 //for (int i=0 ; i < p->nbFeature ; i++)
 //{
 wmean += w[n] ;
 vpPointSite s = p->list.value() ;
 if (w[n] < 0.5)
 {
 s.suppress = 4 ;
 p->list.modify(s) ;
 }

 p->list.next();
 //}
 n+= 1 ;

 wmean = 1;

 //p->setMeanWeight(wmean);

 }

 }
 //   cout << "\t Robust minimization in " << iter << " iteration " << endl ;
 //    std::cout << "error: " << (residu_1 - r) << std::endl;
 }*/

void apMbTracker::computeVVSCorr(const vpImage<unsigned char>& I,
		const vpImage<double>& Igrad, const vpImage<double>& Igradx,
		const vpImage<double>& Igrady) {
	double residu_1 = 1e3;
	double r = 1e3 - 1;
	vpMatrix LTL;
	vpColVector LTR;

	// compute the interaction matrix and its pseudo inverse
	apControlPoint *p;

	vpColVector w;
	vpColVector weighted_error;
	vpColVector factor;

	unsigned int iter = 0;

	//Nombre de moving edges
	int nbrow = 0;
	vpFeatureLine fli;

	for (int k = 0; k < points[scaleLevel].size(); k++) {
		p = (points[scaleLevel])[k];
		nbrow += 1;
		//p->initInteractionMatrixError();
		p->initInteractionMatrixError30();
		//
		//std::cout<<fli.getTheta()<<std::endl;
	}

	std::cout << " nbrow " << nbrow << " size " << points[scaleLevel].size()
			<< " scale level " << scaleLevel << std::endl;

	if (nbrow == 0) {
		vpERROR_TRACE(
				"\n\t\t Error-> not enough data in the interaction matrix...");
		throw vpTrackingException(vpTrackingException::notEnoughPointError,
				"\n\t\t Error-> not enough data in the interaction matrix...");
	}

	vpMatrix L(nbrow * 11, 6), Lp;

	// compute the error vector
	vpColVector error(nbrow * 11);
	int nerror = error.getRows();
	vpColVector v;

	double limite = 3; //Une limite de 3 pixels
	limite = limite / cam.get_px(); //Transformation limite pixel en limite metre.

	//Parametre pour la premiere phase d'asservissement
	double e_prev = 0, e_cur, e_next;
	bool reloop = true;
	double count = 0;
	while (reloop == true && iter < 50) {
		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 0;
			factor.resize(nerror);
			factor = 1;
		}

		count = 0;
		//;
		int n = 0;
		//reloop = false;
		reloop = true;
		vpImage<unsigned char> _I;

		//for (int k = 0; k<points[scaleLevel].size(); k++)
		for (int k = 0; k < points[scaleLevel].size(); k++) {
			p = (points[scaleLevel])[k];
			//p->computeInteractionMatrixError3(cMo, I, Igrad, Igradx, Igrady);
			p->computeInteractionMatrixError30(cMo, I, Igrad, Igradx, Igrady);

			//std::cout << "autoIm00 "   << std::endl;
			//p->computeInteractionMatrixError2(cMo,_I);
			//getchar();


			double fac = 1;
			/*if (iter == 0) {
			 fac = 0.2;
			 break;
			 }*/

			//std::cout << " fac " << fac << std::endl;

			if (iter == 0 && p != NULL)
				p->s;

			/*for (int j = 0; j < 6; j++) {
			 L[n][j] = p->L[j]; //On remplit la matrice d'interaction globale
			 }
			 error[n] = p->error; //On remplit la matrice d'erreur
			 */
			for (int l = 0; l < 11; l++) {
				for (int j = 0; j < 6; j++)
					L[n * 11 + l][j] = p->LD[l][j]; //On remplit la matrice d'interaction globale

				error[n * 11 + l] = p->errorD[l];

				if (error[n * 11 + l] <= limite)
					count = count + 1.0; //Si erreur proche de 0 on incremente cur

				w[n * 11 + l] = 1;

				if (iter == 0) {
					factor[n * 11 + l] = fac;
					vpPointSite site = p->s;
					if (site.suppress != 0)
						factor[n * 11 + l] = 0.2;
				}

				//else
				//{
				/*e_cur = p->errorD[0];
				 e_next = p->errorD[0];
				 if (fabs(e_cur - e_prev) < limite) {
				 w[n*11+l] += 0.5;
				 }
				 if (fabs(e_cur - e_next) < limite) {
				 w[n*11 + l] += 0.5;
				 }
				 e_prev = e_cur;*/
				//}

			}
			n += 1;
			//
		}

		//vpDisplay::flush(I);

		count = count / (double) nbrow;
		if (count < 0.85) {
			reloop = true;
		}

		double num = 0;
		double den = 0;

		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			eri = error[i];
			num += wi * vpMath::sqr(eri);
			den += wi;

			//weighted_error[i] =  wi*eri ;
			weighted_error[i] = 1;
		}

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					L[i][j] = w[i] * factor[i] * L[i][j];
				}
			}
		}
		vpMatrix Hsd; // hessien a la position desiree
		vpMatrix H;

		LTL = L.AtA();
		//std::cout << weighted_error << std::endl;
		computeJTR(L, weighted_error, LTR);
		v = -0.1 * LTL.pseudoInverse() * LTR;
		cMo = vpExponentialMap::direct(v).inverse() * cMo;

		iter++;
	}
	std::cout << "\t First minimization in " << iter << " iteration00 "
			<< std::endl;

	//std::cout<<iter<<std::endl;
	// ;
	int n = 0;
	for (int k = 0; k < points[scaleLevel].size(); k++) {
		p = (points[scaleLevel])[k];
		{
			double wmean = 0;
			p->s;

			//for (int i=0 ; i < p->nbFeature ; i++)
			//{
			wmean += w[n];
			vpPointSite site = p->s;
			if (w[n] < 0.5) {
				site.suppress = 4;
				p->s = site;
				;
			}
			//}
			n += 1;

			wmean = 1;

			//p->setMeanWeight(wmean);

		}
		//
	}
	//   cout << "\t Robust minimization in " << iter << " iteration " << endl ;
	//    std::cout << "error: " << (residu_1 - r) << std::endl;
}

void apMbTracker::computeVVSHybrid(const vpImage<unsigned char>& I,
		const vpImage<double>& Igrad, const vpImage<double>& Igradx,
		const vpImage<double>& Igrady) {
	double residu_1 = 1e3;
	double r = 1e3 - 1;
	vpMatrix LTL;
	vpColVector LTR;

	vpMatrix LTLD;
	vpColVector LTRD;

	// compute the interaction matrix and its pseudo inverse
	apControlPoint *p;

	vpColVector w, wD;
	vpColVector weighted_error, weighted_errorD;
	vpColVector factor, factorD;

	unsigned int iter = 0;

	//Nombre de moving edges
	int nbrow = 0;
	vpFeatureLine fli;

	for (int k = 0; k < points[scaleLevel].size(); k++) {
		p = (points[scaleLevel])[k];
		nbrow += 1;
		p->initInteractionMatrixError();
		p->initInteractionMatrixError30();
		//
		//std::cout<<fli.getTheta()<<std::endl;
	}

	std::cout << " nbrow " << nbrow << " size " << points[scaleLevel].size()
			<< " scale level " << scaleLevel << std::endl;

	if (nbrow == 0) {
		vpERROR_TRACE(
				"\n\t\t Error-> not enough data in the interaction matrix...");
		throw vpTrackingException(vpTrackingException::notEnoughPointError,
				"\n\t\t Error-> not enough data in the interaction matrix...");
	}

	vpMatrix L(nbrow, 6), LD(nbrow * 11, 6), Lp;

	// compute the error vector
	vpColVector error(nbrow), errorD(nbrow * 11);
	int nerror = error.getRows();
	int nerrorD = errorD.getRows();
	vpColVector v;

	double limite = 3; //Une limite de 3 pixels
	limite = limite / cam.get_px(); //Transformation limite pixel en limite metre.

	//Parametre pour la premiere phase d'asservissement
	double e_prev = 0, e_cur, e_next;
	bool reloop = true;
	double count = 0;

	////////////////////////////////////////////////////////////:::

	while (reloop == true && iter < 15) {
		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 0;
			factor.resize(nerror);
			factor = 1;
		}

		if (iter == 0) {
			weighted_errorD.resize(nerrorD);
			wD.resize(nerrorD);
			wD = 0;
			factorD.resize(nerrorD);
			factorD = 1;
		}

		count = 0;
		//;
		int n = 0;
		//reloop = false;
		reloop = true;
		for (int k = 0; k < points[scaleLevel].size(); k++)
		//for (int k = 0; k<points[scaleLevel].size(); k++)
		{
			//p = (points[scaleLevel])[k];
			p = (points[scaleLevel])[k];
			p->computeInteractionMatrixError(cMo, I);
			//p->computeInteractionMatrixError2(cMo,_I);
			//p->computeInteractionMatrixError4(cMo, I ,Igrad , Igradx, Igrady);
			p->computeInteractionMatrixError30(cMo, I, Igrad, Igradx, Igrady);

			double fac = 1;
			if (iter == 0 && p != NULL)

				for (int j = 0; j < 6; j++) {
					L[n][j] = p->L[j]; //On remplit la matrice d'interaction globale
				}
			error[n] = p->error; //On remplit la matrice d'erreur

			if (error[n] <= limite)
				count = count + 1.0; //Si erreur proche de 0 on incremente cur

			w[n] = 0;

			if (iter == 0) {
				factor[n] = fac;
				vpPointSite site = p->s;
				//if (site.suppress != 0) factor[n] = 0;
				if (site.suppress != 0)
					factor[n] = 0.2;
			}

			//else
			//{
			e_cur = p->error;
			e_next = p->error;
			if (fabs(e_cur - e_prev) < limite) {
				w[n] += 0.5;
				//w[n]=1;
			}
			if (fabs(e_cur - e_next) < limite) {
				w[n] += 0.5;
				//w[n]=1;
			}
			e_prev = e_cur;
			//}



			for (int l = 0; l < 11; l++) {

				for (int j = 0; j < 6; j++)
					LD[n * 11 + l][j] = p->LD[l][j]; //On remplit la matrice d'interaction globale

				errorD[n * 11 + l] = p->errorD[l];

				if (errorD[n * 11 + l] <= limite)
					count = count + 1.0; //Si erreur proche de 0 on incremente cur

				wD[n * 11 + l] = 1;

				if (iter == 0) {
					factorD[n * 11 + l] = fac;
					vpPointSite site = p->s;
					if (site.suppress != 0)
						factorD[n * 11 + l] = 0.2;
				}

			}

			n += 1;
			//
		}

		count = count / (double) nbrow;
		if (count < 0.85) {
			reloop = true;
		}

		double num = 0;
		double den = 0;

		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			eri = error[i];
			num += wi * vpMath::sqr(eri);
			den += wi;

			weighted_error[i] = wi * eri;
		}

		for (int i = 0; i < nerrorD; i++) {
			wi = wD[i] * factorD[i];
			eri = errorD[i];
			num += wi * vpMath::sqr(eri);
			den += wi;

			//weighted_errorD[i] =  wi*eri ;
			weighted_errorD[i] = 1;
		}

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					L[i][j] = w[i] * factor[i] * L[i][j];
				}
			}

			for (int i = 0; i < nerrorD; i++) {
				for (int j = 0; j < 6; j++) {
					LD[i][j] = wD[i] * factorD[i] * LD[i][j];
				}
			}
		}

		double weight_d = (double) 1 / 5000000000;
		//double weight_d = 0;

		LTL = L.AtA();
		computeJTR(L, weighted_error, LTR);

		LTLD = LD.AtA();
		computeJTR(LD, weighted_errorD, LTRD);
		v = -0.7 * (LTL + weight_d * LTLD).pseudoInverse(LTL.getRows()
				* DBL_EPSILON) * (LTR + weight_d * LTRD);

		//v = -0.7 * LTL.pseudoInverse(LTL.getRows() * DBL_EPSILON) * LTR;
		cMo = vpExponentialMap::direct(v).inverse() * cMo;

		iter++;
	}

	///////////////////////////////////////////////////////////////////////

	std::cout << "\t First minimization in " << iter << " iteration00 "
			<< std::endl;

	//std::cout<<iter<<std::endl;
	int n = 0;
	for (int k = 0; k < points[scaleLevel].size(); k++) {
		p = (points[scaleLevel])[k];
		{
			double wmean = 0;
			p->s;

			wmean += w[n];
			vpPointSite site = p->s;
			if (w[n] < 0.5) {
				site.suppress = 4;
				p->s = site;
			}

			n += 1;

			wmean = 1;

			//p->setMeanWeight(wmean);

		}
		//
	}
	//   cout << "\t Robust minimization in " << iter << " iteration " << endl ;
	//    std::cout << "error: " << (residu_1 - r) << std::endl;
}

void apMbTracker::computeVVSMH(const vpImage<unsigned char>& _I) {
	double residu_1 = 1e3;
	double r = 1e3 - 1;
	vpMatrix LTL;
	vpColVector LTR;

	// compute the interaction matrix and its pseudo inverse
	vpColVector w;
	vpColVector weighted_error;
	vpColVector factor;


	covarianceMatrix.resize(6,6);
	covarianceMatrixME.resize(6,6);


	int iter = 0;

	//Nombre de moving edges
	int nbrow = 0;
	vpFeatureLine fli;

	const size_t scaleLevel_size = points[scaleLevel].size();
	for (int k = 0; k < scaleLevel_size; k++) {
		apControlPoint *p = (points[scaleLevel])[k];
		nbrow += 1;
		p->initInteractionMatrixError();

		//std::cout<<fli.getTheta()<<std::endl;
	}

	//std::cout << " size points " << points[scaleLevel].size() << std::endl;
	if (nbrow == 0) {
		vpERROR_TRACE(
				"\n\t\t Error-> not enough data in the interaction matrix...");
		throw vpTrackingException(vpTrackingException::notEnoughPointError,
				"\n\t\t Error-> not enough data in the interaction matrix...");
	}

	vpMatrix L(nbrow, 6), Lp;

	// compute the error vector
	vpColVector error(nbrow);
	int nerror = error.getRows();
	vpColVector v;
	double limite = 3; //Une limite de 3 pixels
	limite = limite / cam.get_px(); //Transformation limite pixel en limite metre.

	//Parametre pour la premiere phase d'asservissement
	double e_prev = 0, e_cur, e_next;
	bool reloop = true;
	double count = 0;
	/*** First phase ***/

	while (reloop == true && iter < 3) {
		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 1;
			factor.resize(nerror);
			factor = 1;
		}

		count = 0;
		int n = 0;
		//reloop = false;
		reloop = true;

		if (iter == 0)
			(points[scaleLevel])[0]->computeInteractionMatrixErrorMH(cMo, _I);
		else
		{
//#pragma omp parallel for
			for (int k = 0; k < points[scaleLevel].size(); ++k) {
				apControlPoint *p = (points[scaleLevel])[k];
				p->computeInteractionMatrixErrorMH(cMo, _I);
				//p->computeInteractionMatrixError2(cMo,_I);

				double fac = 1;

				//On remplit la matrice d'interaction globale
                                for (int j = 0; j < 6; j++) {
					L[n][j] = p->L[j];
				}
				error[n] = p->error; //On remplit la matrice d'erreur

				if (error[n] <= limite)
					count = count + 1.0; //Si erreur proche de 0 on incremente cur


				if (iter == 0) {
					factor[n] = fac;
					vpPointSite site = p->s;
					//if (site.suppress != 0) factor[n] = 0;
					if (site.suppress != 0)
						factor[n] = 0.2;
				}

				w[n] = 1;

//				e_next = e_cur = p->error;
//				if (fabs(e_cur - e_prev) < limite)
//					w[k] += 1.0;
//				else
//					w[k] += 0.5;
//				e_prev = e_cur;

				n++;
			}
		}

		count = count / (double) nbrow;
		if (count < 0.85) {
			reloop = true;
		}

		//double num = 0;
		//double den = 0;

		double wi;
		double eri;
//#pragma omp parallel for
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			eri = error[i];
			//num += wi * vpMath::sqr(eri);
			//den += wi;
			weighted_error[i] = wi * eri;
		}

		if ((iter == 0) || compute_interaction) {
//#pragma omp parallel for
			for (int i = 0; i < nerror; i++) {
                                for (int j = 0; j < 6; j++) {
					//L[i][j] = w[i]*factor[i]*L[i][j] ;
					L[i][j] = factor[i] * L[i][j];
				}
			}
		}

		//		LTL = L.AtA();
		L.AtA(LTL);
		computeJTR(L, weighted_error, LTR);
		v = -0.7 * LTL.pseudoInverse(LTL.getRows() * DBL_EPSILON) * LTR;
		cMo = vpExponentialMap::direct(v).inverse() * cMo;


		iter++;
	}
	std::cout << "\t First minimization in " << iter << " iteration00 "
			<< std::endl;

	/*** Second phase ***/

	vpColVector W_true;
	vpMatrix L_true;
	vpRobust robust(nerror);
	robust.setIteration(0);
	iter = 0;
	//vpColVector error_px(nerror);

	while ((iter < 10)) {
//#pragma omp parallel for
		for (int k = 0; k < points[scaleLevel].size(); k++) {
			apControlPoint *p = (points[scaleLevel])[k];
			p->computeInteractionMatrixErrorMH(cMo, _I);
                        for (int j = 0; j < 6; j++) {
				L[k][j] = p->L[j];
			}
			error[k] = p->error;
		}

		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 1;

			robust.setThreshold(2 / cam.get_px()); // limite en metre
			robust.MEstimator(vpRobust::TUKEY, error, w);
			//robust.setThreshold(2); // limite en pixel
			//robust.MEstimator(vpRobust::TUKEY, error_px,w);
		} else {
			robust.setIteration(iter);
			robust.MEstimator(vpRobust::TUKEY, error, w);
			//robust.MEstimator(vpRobust::TUKEY, error_px,w);
		}

		residu_1 = r;

		L_true = L;
		W_true = vpColVector(nerror);

		//double num = 0;
		//double den = 0;
		double wi;
		double eri;
//#pragma omp parallel for
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			W_true[i] = wi * wi;
			eri = error[i];
			//num += wi * vpMath::sqr(eri);
			//den += wi;

			weighted_error[i] = wi * eri;
		}

		//r = sqrt(num / den); //Le critere d'arret prend en compte le poids

		if ((iter == 0) || compute_interaction) {
//#pragma omp parallel for
			for (int i = 0; i < nerror; i++) {
                                for (int j = 0; j < 6; j++) {
					L[i][j] = w[i] * factor[i] * L[i][j];
				}
			}
		}

		//		LTL = L.AtA();
		L.AtA(LTL);
		computeJTR(L, weighted_error, LTR);


		//double stdME = sqrt((double)(weighted_error.t()*weighted_error)/(weighted_error.size()));

		//double wghtME = ((double)1/weighted_error.size())*(1/stdME);

		//douwghtME = 0.7;

		//std::cout << "t3 = " << vpTime::measureTimeMs() - t0 << std::endl;

		//v = -lambda * (weight_me * wghtME*LTL).pseudoInverse(LTL.getRows() * DBL_EPSILON) * (weight_me*wghtME*LTR);

		v = -lambda * LTL.pseudoInverse(LTL.getRows() * DBL_EPSILON) * LTR;
		cMo = vpExponentialMap::direct(v).inverse() * cMo;

		iter++;
	}
	if (computeCovariance) {
		vpMatrix D;
		D.diag(W_true);
		covarianceMatrix = computeCovarianceMatrix(L_true, -v, lambda * error,
				D);
		covarianceMatrixME = covarianceMatrix;
	}

}

void apMbTracker::computeVVSPointsLinesMH(const vpImage<unsigned char>& _I) {
	double residu_1 = 1e3;
	double r = 1e3 - 1;
	double residu_1L = 1e3;
	double rL = 1e3 - 1;
	vpMatrix LTL;
	vpColVector LTR;

	// compute the interaction matrix and its pseudo inverse
	apMbtDistanceLineMH *l, *l1, *l2;

	// compute the interaction matrix and its pseudo inverse
	vpColVector w;
	vpColVector weighted_error;
	vpColVector factor;

	vpColVector wL;
	vpColVector weighted_errorL;
	vpColVector factorL;

	vpColVector weighted_errorH;

	vpColVector errorH;

	int iter = 0;

	covarianceMatrix.resize(6,6);
	covarianceMatrixME.resize(6,6);
	covarianceMatrixL.resize(6,6);
	covarianceMatrixP.resize(6,6);

	//Nombre de moving edges
	int nbrow = 0;
	int nbrowL = 0;
	vpFeatureLine fli;

	lines[scaleLevel][0]->getCameraParameters(&cam);
	std::cout << " nb lines clusters " << lines[scaleLevel].size() << std::endl;
	for (int k = 0; k < lines[scaleLevel].size(); k++) {
		l = lines[scaleLevel][k];
		if (l->MHmeline->points_vect.size() > 0) {
			l->initInteractionMatrixErrorMH();
			nbrowL += l->nbFeature;
			//std::cout << " nb features " << l->nbFeature << std::endl;
		}
	}
	if (nbrowL == 0) {
		vpERROR_TRACE(
				"\n\t\t Error-> not enough data in the interaction matrix...");
		throw vpTrackingException(vpTrackingException::notEnoughPointError,
				"\n\t\t Error-> not enough data in the interaction matrix...");
	}
	//std::cout << " ok02 " << std::endl;
	const size_t scaleLevel_size = points[scaleLevel].size();
	for (int k = 0; k < scaleLevel_size; k++) {
		apControlPoint *p = (points[scaleLevel])[k];
		nbrow += 1;
		p->initInteractionMatrixError();

		//std::cout<<fli.getTheta()<<std::endl;
	}
	if (nbrow == 0) {
		vpERROR_TRACE(
				"\n\t\t Error-> not enough data in the interaction matrix...");
		throw vpTrackingException(vpTrackingException::notEnoughPointError,
				"\n\t\t Error-> not enough data in the interaction matrix...");
	}

	vpMatrix L(nbrow, 6), Lp;

	vpMatrix LL(nbrowL, 6), LpL;

	vpMatrix LH;

	// compute the error vector
	vpColVector error(nbrow);
	int nerror = error.getRows();

	vpColVector errorL(nbrowL);
	int nerrorL = errorL.getRows();


	vpColVector v;
	double limite = 3; //Une limite de 3 pixels
	limite = limite / cam.get_px(); //Transformation limite pixel en limite metre.

	//Parametre pour la premiere phase d'asservissement
	double e_prev = 0, e_cur, e_next;
	bool reloop = true;
	double count = 0;

	double e_prevL = 0, e_curL, e_nextL;
	bool reloopL = true;
	double countL = 0;
	/*** First phase ***/

	//std::vector<vpRobust> robline;
	//robline.resize(0);

	while (reloop == true && iter < 3) {

		if (iter == 0)
		{
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 1;
			factor.resize(nerror);
			factor = 1;

			weighted_errorL.resize(nerrorL);
			wL.resize(nerrorL);
			wL = 1;
			factorL.resize(nerrorL);
			factorL = 1;
		}

		countL = 0;
		int nL = 0;
		//reloop = false;
		reloopL = true;


		for (int k = 0; k < lines[scaleLevel].size(); k++) {
		 l = lines[scaleLevel][k];
		 /*if(l->MHmeline->points_vect.size()>0)
		 {
			 vpRobust robl;

		 }*/

		 if(l->MHmeline->points_vect.size()>0)
		 {
		 l->computeInteractionMatrixErrorMH(_I, cMo);
		 double fac = 1;
		 //std::cout << " ok0001 " << std::endl;
		 for (int i = 0; i < l->nbFeature; i++) {

		errorL[nL + i] = l->error[i];
		 for (int j = 0; j < 6; j++) {
		 LL[nL + i][j] = l->L[i][j];
		 //std::cout << " ok0001 " << errorL[nL + i]<< " LL " << LL[nL + i][j] << std::endl;
		 }
		 if (errorL[nL + i] <= limite)
		 countL++;


		 /*wL[nL + i] = 1;
		 if (i == 0) {
		 e_curL = l->error[0];
		 if (l->nbFeature > 1)
		 e_nextL = l->error[1];
		 if (fabs(e_curL - e_nextL) < limite && vpMath::sign(e_curL)
		 == vpMath::sign(e_nextL)) {
		 wL[nL + i] = 1;//0.5
		 }
		 e_prevL = e_curL;
		 }

		 //else w[n+i] = 1;
		 //}
		 else if (i == l->nbFeature - 1) {
		 e_curL = l->error[i];
		 if (fabs(e_curL - e_prevL) < limite && vpMath::sign(e_curL)
		 == vpMath::sign(e_prevL)) {
		 wL[nL + i] = 1;//0.5;
		 }
		 }
		 else {
		 e_curL = l->error[i];
		 e_nextL = l->error[i + 1];
		 if (fabs(e_curL - e_prevL) < limite) {
		 wL[nL + i] += 0.5;
		 }
		 if (fabs(e_cur - e_next) < limite) {
		 wL[nL + i] += 0.5;
		 }
		 e_prevL = e_curL;
		 }*/

		 wL[nL + i] = 1;

		 }

		 nL += l->nbFeature;

		 }
		}


		countL = countL / (double) nbrowL;
		if (countL < 0.85) {
			reloopL = true;
		}


		count = 0;
		int n = 0;
		//reloop = false;
		reloop = true;

		if (iter == 0)
			(points[scaleLevel])[0]->computeInteractionMatrixErrorMH(cMo, _I);
		else
		{
//#pragma omp parallel for
			for (int k = 0; k < points[scaleLevel].size(); ++k) {
				apControlPoint *p = (points[scaleLevel])[k];
				p->computeInteractionMatrixErrorMH(cMo, _I);
				//p->computeInteractionMatrixError2(cMo,_I);

				for(int j=0;j<6;j++)
					L[n][j] = p->L[j];

				error[n] = p->error; //On remplit la matrice d'erreur

				if (error[n] <= limite)
					count = count + 1.0; //Si erreur proche de 0 on incremente cur

				w[n] = 0;

				e_next = e_cur = p->error;
				if (fabs(e_cur - e_prev) < limite)
					w[n] += 1.0;
				else
					w[n] += 0.5;
				e_prev = e_cur;

				w[n] = 1;
				n++;
			}
		}

		count = count / (double) nbrow;
		if (count < 0.85) {
			reloop = true;
		}

		double num = 0;
		double den = 0;

		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			eri = error[i];
			num += wi * vpMath::sqr(eri);
			den += wi;
			weighted_error[i] = wi * eri;
		}

		double numL = 0;
		double denL = 0;

		double wiL;
		double eriL;
		for (int i = 0; i < nerrorL; i++) {
			wiL = wL[i] * factorL[i];
			eriL = errorL[i];
			numL += wiL * vpMath::sqr(eriL);
			denL += wiL;
			weighted_errorL[i] = wiL * eriL;
		}

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					//L[i][j] = w[i]*factor[i]*L[i][j] ;
					L[i][j] = factor[i] * L[i][j];
				}
			}

			for (int i = 0; i < nerrorL; i++) {
				for (int j = 0; j < 6; j++) {
					//L[i][j] = w[i]*factor[i]*L[i][j] ;
					//LL[i][j] = factorL[i] * LL[i][j];
				}
			}
		}

		 vpMatrix::stackMatrices(L, LL, LH);
		 vpMatrix::stackMatrices(weighted_error, weighted_errorL,
		 weighted_errorH);

		//		LTL = L.AtA();
		//LH.AtA(LTL);
		//std::cout << " LTL0 " << LTL << std::endl;
	     L.AtA(LTL);
	     //std::cout << " LTL1 " << LTL << std::endl;
		computeJTR(L, weighted_error, LTR);
		v = -0.7 * LTL.pseudoInverse(LTL.getRows() * DBL_EPSILON) * LTR;
		cMo = vpExponentialMap::direct(v).inverse() * cMo;

		//std::cout << " cMo " << cMo << std::endl;


		iter++;
	}
	std::cout << "\t First minimization in " << iter << " iteration00 "
			<< std::endl;

	/*** Second phase ***/

	vpColVector W_true;
	vpMatrix L_true;
	vpRobust robust(nerror);
	robust.setIteration(0);

	vpColVector WL_true;
    vpMatrix LL_true;
	vpRobust robustL(nerrorL);
	robustL.setIteration(0);

	vpColVector WH_true;
	vpMatrix LH_true;

	iter = 0;
	//vpColVector error_px(nerror);

	while (((int) ((residu_1 - r) * 1e8) != 0) && (iter < 12)) {

		int nL = 0;
		for (int k = 0; k < lines[scaleLevel].size(); k++) {
		 l = lines[scaleLevel][k];
		 if(l->MHmeline->points_vect.size()>0)
		 {
		 l->computeInteractionMatrixErrorMH(_I, cMo);
		 for (int i = 0; i < l->nbFeature; i++) {
		 for (int j = 0; j < 6; j++){
		 LL[nL + i][j] = l->L[i][j];
		 }
		 errorL[nL + i] = l->error[i];
		 }
		 nL += l->nbFeature;
		 }
		 }


//#pragma omp parallel for
		for (int k = 0; k < points[scaleLevel].size(); k++) {
			apControlPoint *p = (points[scaleLevel])[k];
			p->computeInteractionMatrixErrorMH(cMo, _I);
			for (int j = 0; j < 6; j++) {
				L[k][j] = p->L[j];

				error[k] = p->error;
			}
		}

		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 1;

			robust.setThreshold(2 / cam.get_px()); // limite en metre
			robust.MEstimator(vpRobust::TUKEY, error, w);
			//robust.setThreshold(2); // limite en pixel
			//robust.MEstimator(vpRobust::TUKEY, error_px,w);

			weighted_errorL.resize(nerrorL);
			wL.resize(nerrorL);
			wL = 1;

			robustL.setThreshold(2 / cam.get_px()); // limite en metre
			robustL.MEstimator(vpRobust::TUKEY, errorL, wL);

		} else {
			robust.setIteration(iter);
			robust.MEstimator(vpRobust::TUKEY, error, w);
			//robust.MEstimator(vpRobust::TUKEY, error_px,w);

			robustL.setIteration(iter);
			robustL.MEstimator(vpRobust::TUKEY, errorL, wL);
			//robust.MEstimator(vpRobust::TUKEY, error_px,w);
		}

		residu_1 = r;

		L_true = L;
		W_true = vpColVector(nerror);


		LL_true = LL;
		WL_true = vpColVector(nerrorL);



		//---------------------------------------
		// Compute the mean of the weight for a given line reinit if necessary
		nL = 0;
		for (int k = 0; k < lines[scaleLevel].size(); k++) {
		 l = lines[scaleLevel][k];
		 if(l->MHmeline->points_vect.size()>0)
		 {
		 double wmean = 0;
		 for (int i = 0; i < l->nbFeature; i++) {
		 if (iter == 11)
		 {
		 wL[nL+i] = l->weight[i]*wL[nL+i];
		 //std::cout << " weight " << w[n+i] <<std::endl;
		 }
		 wmean += wL[nL + i];

		 }
		 nL += l->nbFeature;
		 wmean /= l->nbFeature;
		 if (l->nbFeature == 0)
		 wmean = 1;
		 l->wmean = wmean;

		 }
		 }


		double num = 0;
		double den = 0;
		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			W_true = wi * wi;
			eri = error[i];
			num += wi * vpMath::sqr(eri);
			den += wi;

			weighted_error[i] = wi * eri;
		}

		r = sqrt(num / den); //Le critere d'arret prend en compte le poids

		double numL = 0;
		double denL = 0;
		double wiL;
		double eriL;
		for (int i = 0; i < nerrorL; i++) {
			wiL = wL[i] * factorL[i];
			WL_true = wiL * wiL;
			eriL = errorL[i];
			numL += wiL * vpMath::sqr(eriL);
			denL += wiL;

			weighted_errorL[i] = wiL * eriL;
		}

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					L[i][j] = w[i] * factor[i] * L[i][j];
				}
			}

			for (int i = 0; i < nerrorL; i++) {
				for (int j = 0; j < 6; j++) {
					LL[i][j] = wL[i] * factorL[i] * LL[i][j];
				}
			}
		}

		vpMatrix::stackMatrices(L, LL, LH);
		vpMatrix::stackMatrices(weighted_error, weighted_errorL,
		weighted_errorH);

		LH.AtA(LTL);
		//LTL = L.AtA();
		//L.AtA(LTL);
		computeJTR(LH, weighted_errorH, LTR);

		vpMatrix::stackMatrices(error, errorL, errorH);
		double stdME = sqrt((double)(weighted_errorH.t()*weighted_errorH)/(weighted_errorH.size()));
		double wghtME = ((double)1/weighted_errorH.size())*(1/stdME);

		wghtME = 1;

		//std::cout << "t3 = " << vpTime::measureTimeMs() - t0 << std::endl;

		v = -lambda * (weight_me * wghtME*LTL).pseudoInverse(LTL.getRows() * DBL_EPSILON) * (weight_me*wghtME*LTR);

		//v = -lambda * LTL.pseudoInverse(LTL.getRows() * DBL_EPSILON) * LTR;
		cMo = vpExponentialMap::direct(v).inverse() * cMo;

		iter++;
	}
	if (computeCovariance) {

		vpMatrix D;
		D.diag(W_true);
		covarianceMatrixP = computeCovarianceMatrix(L_true, -v, lambda * error,D);
		vpMatrix DL;
		DL.diag(WL_true);
		covarianceMatrixL = computeCovarianceMatrix(LL_true, -v, lambda * errorL,DL);

		vpMatrix::stackMatrices(W_true, WL_true, WH_true);
		vpMatrix::stackMatrices(error, errorL, errorH);
		vpMatrix::stackMatrices(L_true, LL_true, LH_true);
		WH_true = vpColVector(nerror+nerrorL);
		vpMatrix DH;
		DH.diag(WH_true);
		covarianceMatrixME = computeCovarianceMatrix(LH_true, -v, lambda * errorH,DH);
		covarianceMatrix = covarianceMatrixME;


	}

}

void apMbTracker::computeVVSPointsLinesRobustMH(const vpImage<unsigned char>& _I) {
	double residu_1 = 1e3;
	double r = 1e3 - 1;
	double residu_1L = 1e3;
	double rL = 1e3 - 1;
	vpMatrix LTL;
	vpColVector LTR;

	// compute the interaction matrix and its pseudo inverse
	apMbtDistanceLineMH *l, *l1, *l2;

	// compute the interaction matrix and its pseudo inverse
	vpColVector w;
	vpColVector weighted_error;
	vpColVector factor;

	vpColVector wL;
	vpColVector weighted_errorL;
	vpColVector factorL;

	vpColVector weighted_errorH;

	vpColVector errorH;

	int iter = 0;

	covarianceMatrix.resize(6,6);
	covarianceMatrixME.resize(6,6);
	covarianceMatrixL.resize(6,6);
	covarianceMatrixP.resize(6,6);

	//Nombre de moving edges
	int nbrow = 0;
	int nbrowL = 0;
	vpFeatureLine fli;

	int nblm = 2;
	int nblm2 = 0;
	lines[scaleLevel][0]->getCameraParameters(&cam);
	std::cout << " nb lines clusters " << lines[scaleLevel].size() << std::endl;
	for (int k = 0; k < lines[scaleLevel].size(); k++) {
		l = lines[scaleLevel][k];
		if (l->nbFeature > nblm) {
			l->initInteractionMatrixErrorMH();
			nbrowL += l->nbFeature;
			std::cout << " nb features " << l->nbFeature << std::endl;
		}
	}

	std::cout << " nb row " << nbrowL << std::endl;

	if (nbrowL == 0) {
		vpERROR_TRACE(
				"\n\t\t Error-> not enough data in the interaction matrix...");
		throw vpTrackingException(vpTrackingException::notEnoughPointError,
				"\n\t\t Error-> not enough data in the interaction matrix...");
	}
	//std::cout << " ok02 " << std::endl;
	const size_t scaleLevel_size = points[scaleLevel].size();
	for (int k = 0; k < scaleLevel_size; k++) {
		apControlPoint *p = (points[scaleLevel])[k];
		nbrow += 1;
		p->initInteractionMatrixError();

		//std::cout<<fli.getTheta()<<std::endl;
	}
	if (nbrow == 0) {
		vpERROR_TRACE(
				"\n\t\t Error-> not enough data in the interaction matrix...");
		throw vpTrackingException(vpTrackingException::notEnoughPointError,
				"\n\t\t Error-> not enough data in the interaction matrix...");
	}

	vpMatrix L(nbrow, 6), Lp;

	vpMatrix LL(nbrowL, 6), LpL;

	vpMatrix LH;

	// compute the error vector
	vpColVector error(nbrow);
	int nerror = error.getRows();

	vpColVector errorL(nbrowL);
	int nerrorL = errorL.getRows();


	vpColVector v;
	double limite = 3; //Une limite de 3 pixels
	limite = limite / cam.get_px(); //Transformation limite pixel en limite metre.

	//Parametre pour la premiere phase d'asservissement
	double e_prev = 0, e_cur, e_next;
	bool reloop = true;
	double count = 0;

	double e_prevL = 0, e_curL, e_nextL;
	bool reloopL = true;
	double countL = 0;
	/*** First phase ***/


	while (reloop == true && iter < 3) {

		if (iter == 0)
		{
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 1;
			factor.resize(nerror);
			factor = 1;

			weighted_errorL.resize(nerrorL);
			wL.resize(nerrorL);
			wL = 1;
			factorL.resize(nerrorL);
			factorL = 1;
		}

		countL = 0;
		int nL = 0;
		//reloop = false;
		reloopL = true;


		for (int k = 0; k < lines[scaleLevel].size(); k++) {
		 l = lines[scaleLevel][k];

		 if (l->nbFeature > nblm)
		 {
		 l->computeInteractionMatrixErrorMH(_I, cMo);
		 double fac = 1;
		 //std::cout << " ok0001 " << std::endl;
		 for (int i = 0; i < l->nbFeature; i++) {

		errorL[nL + i] = l->error[i];
		 for (int j = 0; j < 6; j++) {
		 LL[nL + i][j] = l->L[i][j];
		 //std::cout << " ok0001 " << errorL[nL + i]<< " LL " << LL[nL + i][j] << std::endl;
		 }
		 if (errorL[nL + i] <= limite)
		 countL++;


		 /*wL[nL + i] = 1;
		 if (i == 0) {
		 e_curL = l->error[0];
		 if (l->nbFeature > 1)
		 e_nextL = l->error[1];
		 if (fabs(e_curL - e_nextL) < limite && vpMath::sign(e_curL)
		 == vpMath::sign(e_nextL)) {
		 wL[nL + i] = 1;//0.5
		 }
		 e_prevL = e_curL;
		 }

		 //else w[n+i] = 1;
		 //}
		 else if (i == l->nbFeature - 1) {
		 e_curL = l->error[i];
		 if (fabs(e_curL - e_prevL) < limite && vpMath::sign(e_curL)
		 == vpMath::sign(e_prevL)) {
		 wL[nL + i] = 1;//0.5;
		 }
		 }
		 else {
		 e_curL = l->error[i];
		 e_nextL = l->error[i + 1];
		 if (fabs(e_curL - e_prevL) < limite) {
		 wL[nL + i] += 0.5;
		 }
		 if (fabs(e_cur - e_next) < limite) {
		 wL[nL + i] += 0.5;
		 }
		 e_prevL = e_curL;
		 }*/

		 wL[nL + i] = 1;

		 }

		 nL += l->nbFeature;

		 }
		}

		std::cout << " nl " << nL << std::endl;


		countL = countL / (double) nbrowL;
		if (countL < 0.85) {
			reloopL = true;
		}


		count = 0;
		int n = 0;
		//reloop = false;
		reloop = true;

		if (iter == 0)
			(points[scaleLevel])[0]->computeInteractionMatrixErrorMH(cMo, _I);
		else
		{
//#pragma omp parallel for
			for (int k = 0; k < points[scaleLevel].size(); ++k) {
				apControlPoint *p = (points[scaleLevel])[k];
				p->computeInteractionMatrixErrorMH(cMo, _I);
				//p->computeInteractionMatrixError2(cMo,_I);

				for(int j=0;j<6;j++)
					L[n][j] = p->L[j];

				error[n] = p->error; //On remplit la matrice d'erreur

				if (error[n] <= limite)
					count = count + 1.0; //Si erreur proche de 0 on incremente cur

				w[n] = 0;

				e_next = e_cur = p->error;
				if (fabs(e_cur - e_prev) < limite)
					w[n] += 1.0;
				else
					w[n] += 0.5;
				e_prev = e_cur;

				w[n] = 1;
				n++;
			}
		}

		count = count / (double) nbrow;
		if (count < 0.85) {
			reloop = true;
		}

		double num = 0;
		double den = 0;

		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			eri = error[i];
			num += wi * vpMath::sqr(eri);
			den += wi;
			weighted_error[i] = wi * eri;
		}

		double numL = 0;
		double denL = 0;

		double wiL;
		double eriL;
		for (int i = 0; i < nerrorL; i++) {
			wiL = wL[i] * factorL[i];
			eriL = errorL[i];
			numL += wiL * vpMath::sqr(eriL);
			denL += wiL;
			weighted_errorL[i] = wiL * eriL;
		}

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					//L[i][j] = w[i]*factor[i]*L[i][j] ;
					L[i][j] = factor[i] * L[i][j];
				}
			}

			for (int i = 0; i < nerrorL; i++) {
				for (int j = 0; j < 6; j++) {
					//L[i][j] = w[i]*factor[i]*L[i][j] ;
					//LL[i][j] = factorL[i] * LL[i][j];
				}
			}
		}

		 vpMatrix::stackMatrices(L, LL, LH);
		 vpMatrix::stackMatrices(weighted_error, weighted_errorL,
		 weighted_errorH);

		//		LTL = L.AtA();
		//LH.AtA(LTL);
		//std::cout << " LTL0 " << LTL << std::endl;
	     L.AtA(LTL);
	     //std::cout << " LTL1 " << LTL << std::endl;
		computeJTR(L, weighted_error, LTR);
		v = -0.7 * LTL.pseudoInverse(LTL.getRows() * DBL_EPSILON) * LTR;
		cMo = vpExponentialMap::direct(v).inverse() * cMo;

		//std::cout << " cMo " << cMo << std::endl;


		iter++;
	}
	//std::cout << "\t First minimization in " << iter << " iteration00 "
		//	<< std::endl;

	/*** Second phase ***/

	 //printf(" ok 2 ");

	vpColVector W_true;
	vpMatrix L_true;


	vpRobust robust(nerror);
	robust.setIteration(0);

	vpColVector WL_true;
    vpMatrix LL_true;
	//vpRobust robustL(nerrorL);
	//robustL.setIteration(0);

	vpColVector WH_true;
	vpMatrix LH_true;

	iter = 0;
	vpColVector errorl;
	vpColVector weighted_errorl;
	vpColVector wl;
	std::vector<vpColVector> wlines;
	wlines.resize(0);

	 //printf(" ok ");

		vpRobust *rob;
		for (unsigned int i = 0; i < robustlines.size(); i += 1) {
					rob = robustlines[i];
					if (rob != NULL)
						delete rob;
					rob = NULL;
		}
		robustlines.resize(0);

	//std::vector<vpRobust*> robustlines;
	//robustlines.resize(0);
	//vpColVector error_px(nerror);

	while (((int) ((residu_1 - r) * 1e8) != 0) && (iter < 12)) {

		if (iter == 0)
		{
		weighted_errorL.resize(nerrorL);
		wL.resize(nerrorL);
		wL = 1;
		}
		 //printf(" ok0 ");
		int nL = 0;
		int kl = 0;
		for (int k = 0; k < lines[scaleLevel].size(); k++) {

		 l = lines[scaleLevel][k];
	    if (l->nbFeature > nblm)
		 {
		errorl.resize(l->nbFeature);
		 l->computeInteractionMatrixErrorMH(_I, cMo);
		 for (int i = 0; i < l->nbFeature; i++) {
		 for (int j = 0; j < 6; j++){
		 LL[nL + i][j] = l->L[i][j];
		 }

		 //if (l->error[i] < 1)
		 {
		 errorL[nL + i] = l->error[i];
		 errorl[i] = l->error[i];
		 //std::cout  << " aka " << l->error[i] << std::endl;
		 }
		 /*else
		 {
			 errorL[nL + i] = 0.2;
			 errorl[i] =0.2;
		 }*/

		 }

		 		 vpRobust *robust;
		 		 vpRobust *robust0;

		 		 //printf(" ok1 ");

			if (iter == 0) {

				robust = new vpRobust(l->nbFeature);
				weighted_errorl.resize(l->nbFeature);
				wl.resize(l->nbFeature);
				wl = 1;
				wlines.push_back(wl);

				robust->setThreshold(2 / cam.get_px()); // limite en metre
				robust->MEstimator(vpRobust::TUKEY, errorl, wl);
				robustlines.push_back(robust);


			}
			else
			{

				robust0 = robustlines[kl];
				robust0->setIteration(iter);

				//std::cout  << " aka " << errorl.size() << " aka 0 " << wlines[kl].size() << std::endl;

				robust0->MEstimator(vpRobust::TUKEY, errorl, wlines[kl]);
				//robust.MEstimator(vpRobust::TUKEY, error_px,w);

			}


			 for (int i = 0; i < l->nbFeature; i++)
				 {
				     if (l->nbFeature > 10 || wlines[kl][i] != 0)
					 wL[nL+i] = wlines[kl][i];
				     else wL[nL+i] = 1;
				 }
			 nL += l->nbFeature;

			kl++;
			//printf(" ok2 ");

			 }
		 }




//#pragma omp parallel for
		for (int k = 0; k < points[scaleLevel].size(); k++) {
			apControlPoint *p = (points[scaleLevel])[k];
			p->computeInteractionMatrixErrorMH(cMo, _I);
			for (int j = 0; j < 6; j++) {
				L[k][j] = p->L[j];

				error[k] = p->error;
			}
		}

		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 1;

			robust.setThreshold(2 / cam.get_px()); // limite en metre
			robust.MEstimator(vpRobust::TUKEY, error, w);
			//robust.setThreshold(2); // limite en pixel
			//robust.MEstimator(vpRobust::TUKEY, error_px,w);

			//weighted_errorL.resize(nerrorL);
			//wL.resize(nerrorL);
			//wL = 1;

			//robustL.setThreshold(2 / cam.get_px()); // limite en metre
			//robustL.MEstimator(vpRobust::TUKEY, errorL, wL);

		} else {
			robust.setIteration(iter);
			robust.MEstimator(vpRobust::TUKEY, error, w);
			//robust.MEstimator(vpRobust::TUKEY, error_px,w);

			//robustL.setIteration(iter);
			//robustL.MEstimator(vpRobust::TUKEY, errorL, wL);
			//robust.MEstimator(vpRobust::TUKEY, error_px,w);
		}

		residu_1 = r;

		L_true = L;
		W_true = vpColVector(nerror);


		LL_true = LL;
		WL_true = vpColVector(nerrorL);



		//---------------------------------------
		// Compute the mean of the weight for a given line reinit if necessary
		nL = 0;
		for (int k = 0; k < lines[scaleLevel].size(); k++) {
		 l = lines[scaleLevel][k];
		if (l->nbFeature > nblm)
		 {
		 double wmean = 0;
		 for (int i = 0; i < l->nbFeature; i++) {
		 if (iter == 11)
		 {
		 wL[nL+i] = l->weight[i]*wL[nL+i];
		 //std::cout << " weight " << w[n+i] <<std::endl;
		 }
		 wmean += wL[nL + i];

		 }
		 nL += l->nbFeature;
		 wmean /= l->nbFeature;
		 if (l->nbFeature == 0)
		 wmean = 1;
		 l->wmean = wmean;

		 }
		 }

		//std::cout << " nl2 " << nL << " nerror " << nerrorL << std::endl;


		double num = 0;
		double den = 0;
		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			W_true = wi * wi;
			eri = error[i];
			num += wi * vpMath::sqr(eri);
			den += wi;

			weighted_error[i] = wi * eri;
		}

		r = sqrt(num / den); //Le critere d'arret prend en compte le poids

		double numL = 0;
		double denL = 0;
		double wiL;
		double eriL;
		for (int i = 0; i < nerrorL; i++) {
			wiL = wL[i];// * factorL[i];
			WL_true = wiL * wiL;
			eriL = errorL[i];
			numL += wiL * vpMath::sqr(eriL);
			denL += wiL;

			weighted_errorL[i] = wiL * eriL;
		}

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					L[i][j] = w[i] * factor[i] * L[i][j];
				}
			}

			for (int i = 0; i < nerrorL; i++) {
				//if(iter==0 && weighted_errorL[i] > 1)
				{
				//std::cout  << " aka " << wL[i] << " aka 0 " << weighted_errorL[i] << std::endl;
				//getchar();
				}
				for (int j = 0; j < 6; j++) {
					LL[i][j] = wL[i] * LL[i][j];
				}
			}
		}


		vpMatrix::stackMatrices(L, LL, LH);
		vpMatrix::stackMatrices(weighted_error, weighted_errorL,
		weighted_errorH);


		LH.AtA(LTL);
		//LTL = L.AtA();
		//L.AtA(LTL);
		computeJTR(LH, weighted_errorH, LTR);

		vpMatrix::stackMatrices(error, errorL, errorH);
		double stdME = sqrt((double)(weighted_errorH.t()*weighted_errorH)/(weighted_errorH.size()));
		double wghtME = ((double)1/weighted_errorH.size())*(1/stdME);

		wghtME = 1;

		//std::cout << "t3 = " << vpTime::measureTimeMs() - t0 << std::endl;

		v = -lambda * (weight_me * wghtME*LTL).pseudoInverse(LTL.getRows() * DBL_EPSILON) * (weight_me*wghtME*LTR);

		//v = -lambda * LTL.pseudoInverse(LTL.getRows() * DBL_EPSILON) * LTR;
		cMo = vpExponentialMap::direct(v).inverse() * cMo;

		iter++;
	}

	//if (cMo[2][3]>50) getchar();
	if (computeCovariance) {

		vpMatrix D;
		D.diag(W_true);
		covarianceMatrixP = computeCovarianceMatrix(L_true, -v, lambda * error,D);
		vpMatrix DL;
		DL.diag(WL_true);
		covarianceMatrixL = computeCovarianceMatrix(LL_true, -v, lambda * errorL,DL);

		vpMatrix::stackMatrices(W_true, WL_true, WH_true);
		vpMatrix::stackMatrices(error, errorL, errorH);
		vpMatrix::stackMatrices(L_true, LL_true, LH_true);
		WH_true = vpColVector(nerror+nerrorL);
		vpMatrix DH;
		DH.diag(WH_true);
		covarianceMatrixME = computeCovarianceMatrix(LH_true, -v, lambda * errorH,DH);
		covarianceMatrix = covarianceMatrixME;


	}

}



void apMbTracker::computeVVSPointsLinesCCDMHPrev(const vpImage<unsigned char>& _I, const vpImage<vpRGBa>& _IRGB) {
	double residu_1 = 1e3;
	double r = 1e3 - 1;
	double residu_1L = 1e3;
	double rL = 1e3 - 1;
	vpMatrix LTL;
	vpColVector LTR;

	CCDTracker.init(CCDParameters, cam);
	CCDTracker.setImage(_IRGB);
	CCDTracker.setPrevImage(IprecRGB);
	CCDTracker.updateCCDPoints(cMo);
	CCDTracker.computeLocalStatisticsPrev(_I);

	covarianceMatrix.resize(6,6);
	covarianceMatrixCCD.resize(6,6);
	covarianceMatrixL.resize(6,6);
	covarianceMatrixP.resize(6,6);
	covarianceMatrixME.resize(6,6);


	// compute the interaction matrix and its pseudo inverse
	apMbtDistanceLineMH *l, *l1, *l2;

	// compute the interaction matrix and its pseudo inverse
	vpColVector w;
	vpColVector weighted_error;
	vpColVector factor;

	vpColVector wL;
	vpColVector weighted_errorL;
	vpColVector factorL;

	vpColVector weighted_errorH;

	int iter = 0;

	//Nombre de moving edges
	int nbrow = 0;
	int nbrowL = 0;
	vpFeatureLine fli;

	lines[scaleLevel][0]->getCameraParameters(&cam);
	std::cout << " nb lines clusters " << lines[scaleLevel].size() << std::endl;
	for (int k = 0; k < lines[scaleLevel].size(); k++) {
		l = lines[scaleLevel][k];
		if (l->MHmeline->points_vect.size() > 0) {
			l->initInteractionMatrixErrorMH();
			nbrowL += l->nbFeature;
			//std::cout << " nb features " << l->nbFeature << std::endl;
		}
	}
	if (nbrowL == 0) {
		vpERROR_TRACE(
				"\n\t\t Error-> not enough data in the interaction matrix...");
		throw vpTrackingException(vpTrackingException::notEnoughPointError,
				"\n\t\t Error-> not enough data in the interaction matrix...");
	}
	//std::cout << " ok02 " << std::endl;
	const size_t scaleLevel_size = points[scaleLevel].size();
	for (int k = 0; k < scaleLevel_size; k++) {
		apControlPoint *p = (points[scaleLevel])[k];
		nbrow += 1;
		p->initInteractionMatrixError();

		//std::cout<<fli.getTheta()<<std::endl;
	}
	if (nbrow == 0) {
		vpERROR_TRACE(
				"\n\t\t Error-> not enough data in the interaction matrix...");
		throw vpTrackingException(vpTrackingException::notEnoughPointError,
				"\n\t\t Error-> not enough data in the interaction matrix...");
	}

	vpMatrix L(nbrow, 6), Lp;
	vpMatrix LL(nbrowL, 6), LpL;
	vpMatrix LH;
	vpMatrix LTCIL(6, 6);
	vpColVector LTCIR(6);

	// compute the error vector
	vpColVector error(nbrow);
	int nerror = error.getRows();

	vpColVector errorL(nbrowL);
	int nerrorL = errorL.getRows();

	vpColVector errorH;


	vpColVector v;
	double limite = 3; //Une limite de 3 pixels
	limite = limite / cam.get_px(); //Transformation limite pixel en limite metre.

	//Parametre pour la premiere phase d'asservissement
	double e_prev = 0, e_cur, e_next;
	bool reloop = true;
	double count = 0;

	double e_prevL = 0, e_curL, e_nextL;
	bool reloopL = true;
	double countL = 0;
	/*** First phase ***/

	while (reloop == true && iter < 1) {
		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 1;
			factor.resize(nerror);
			factor = 1;

			weighted_errorL.resize(nerrorL);
			wL.resize(nerrorL);
			wL = 1;
			factorL.resize(nerrorL);
			factorL = 1;
		}

		countL = 0;
		int nL = 0;
		//reloop = false;
		reloopL = true;


		/*for (int k = 0; k < lines[scaleLevel].size(); k++) {
		 l = lines[scaleLevel][k];
		 if(l->MHmeline->points_vect.size()>0)
		 {
		 l->computeInteractionMatrixErrorMH(_I, cMo);
		 double fac = 1;
		 //std::cout << " ok0001 " << std::endl;
		 for (int i = 0; i < l->nbFeature; i++) {
		errorL[nL + i] = l->error[i];
		 for (int j = 0; j < 6; j++) {
		 LL[nL + i][j] = l->L[i][j];
		 //std::cout << " ok0001 " << errorL[nL + i]<< " LL " << LL[nL + i][j] << std::endl;
		 }
		 if (errorL[nL + i] <= limite)
		 countL++;


		 wL[nL + i] = 1;
		 if (i == 0) {
		 e_curL = l->error[0];
		 if (l->nbFeature > 1)
		 e_nextL = l->error[1];
		 if (fabs(e_curL - e_nextL) < limite && vpMath::sign(e_curL)
		 == vpMath::sign(e_nextL)) {
		 wL[nL + i] = 1;//0.5
		 }
		 e_prevL = e_curL;
		 }

		 //else w[n+i] = 1;
		 //}
		 else if (i == l->nbFeature - 1) {
		 e_curL = l->error[i];
		 if (fabs(e_curL - e_prevL) < limite && vpMath::sign(e_curL)
		 == vpMath::sign(e_prevL)) {
		 wL[nL + i] = 1;//0.5;
		 }
		 }
		 else {
		 e_curL = l->error[i];
		 e_nextL = l->error[i + 1];
		 if (fabs(e_curL - e_prevL) < limite) {
		 wL[nL + i] += 0.5;
		 }
		 if (fabs(e_cur - e_next) < limite) {
		 wL[nL + i] += 0.5;
		 }
		 e_prevL = e_curL;
		 }

		 wL[nL + i] = 1;

		 }

		 nL += l->nbFeature;

		 }
		}

		countL = countL / (double) nbrowL;
		if (countL < 0.85) {
			reloopL = true;
		}*/

		count = 0;
		int n = 0;
		//reloop = false;
		reloop = true;

		if (iter == 0)
			(points[scaleLevel])[0]->computeInteractionMatrixErrorMH(cMo, _I);
/*		else
		{
//#pragma omp parallel for
			for (int k = 0; k < scaleLevel_size; ++k) {
				apControlPoint *p = (points[scaleLevel])[k];
				p->computeInteractionMatrixErrorMH(cMo, _I);
				//p->computeInteractionMatrixError2(cMo,_I);

				for(int j=0;j<6;j++)
					L[k][j] = p->L[j];

				error[k] = p->error; //On remplit la matrice d'erreur

				if (error[k] <= limite)
					count = count + 1.0; //Si erreur proche de 0 on incremente cur

				w[k] = 0;

				e_next = e_cur = p->error;
				if (fabs(e_cur - e_prev) < limite)
					w[k] += 1.0;
				else
					w[k] += 0.5;
				e_prev = e_cur;
			}
		}

		count = count / (double) nbrow;
		if (count < 0.85) {
			reloop = true;
		}

		double num = 0;
		double den = 0;

		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			eri = error[i];
			num += wi * vpMath::sqr(eri);
			den += wi;
			weighted_error[i] = wi * eri;
		}

		double numL = 0;
		double denL = 0;

		double wiL;
		double eriL;
		for (int i = 0; i < nerrorL; i++) {
			wiL = wL[i] * factorL[i];
			eriL = errorL[i];
			numL += wiL * vpMath::sqr(eriL);
			denL += wiL;
			weighted_errorL[i] = wiL * eriL;
		}

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					//L[i][j] = w[i]*factor[i]*L[i][j] ;
					L[i][j] = factor[i] * L[i][j];
				}
			}

			for (int i = 0; i < nerrorL; i++) {
				for (int j = 0; j < 6; j++) {
					//L[i][j] = w[i]*factor[i]*L[i][j] ;
					LL[i][j] = factorL[i] * LL[i][j];
				}
			}
		}

		//        std::cout << "t-2 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		CCDTracker.updateCCDPoints(cMo);
		CCDTracker.computeLocalStatistics();
		//        std::cout << "t-1 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		CCDTracker.updateParametersPrev(LTCIL, LTCIR);
		//        double t1 = vpTime::measureTimeMs();
		//        std::cout << "t0 = " << vpTime::measureTimeMs() - t0 << std::endl;

		if (iter > 0)
			CCDTracker.checkCCDConvergence();

		//vpMatrix::stackMatrices(L, LL, LH);
		// vpMatrix::stackMatrices(weighted_error, weighted_errorL,
		// weighted_errorH);

				LTL = L.AtA();
		//LH.AtA(LTL);
		L.AtA(LTL);
		computeJTR(L, weighted_error, LTR);
		//std::cout << " LTL0 " << LTL << std::endl;
	     //std::cout << " LTL1 " << LTL << std::endl;
		//v = -0.7 * (LTL).pseudoInverse(LTL.getRows()* DBL_EPSILON) * (LTR);
		v = -0.7 * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows()* DBL_EPSILON) * (LTR - weight_ccd * LTCIR);
		//cMo = vpExponentialMap::direct(v).inverse() * cMo;

		//std::cout << " cMo " << cMo << std::endl;

		*/

		iter++;
	}
	std::cout << "\t First minimization in " << iter << " iteration00 "
			<< std::endl;

	/*** Second phase ***/

	vpColVector W_true;
	vpMatrix L_true;
	vpRobust robust(nerror);
	robust.setIteration(0);

	vpColVector WL_true;
	vpMatrix LL_true;
	vpRobust robustL(nerrorL);
	robustL.setIteration(0);

	vpColVector WH_true;
	vpMatrix LH_true;

	vpRobust robustCCD(CCDTracker.nerror_ccd);
	robustCCD.setIteration(0);
	robustCCD.setThreshold(2 / cam.get_px());

	iter = 0;
	//vpColVector error_px(nerror);

	while (((int) ((residu_1 - r) * 1e8) != 0) && (iter < 12)) {

		int nL = 0;
		for (int k = 0; k < lines[scaleLevel].size(); k++) {
		 l = lines[scaleLevel][k];
		 if(l->MHmeline->points_vect.size()>0)
		 {
		 l->computeInteractionMatrixErrorMH(_I, cMo);
		 for (int i = 0; i < l->nbFeature; i++) {
		 for (int j = 0; j < 6; j++){
		 LL[nL + i][j] = l->L[i][j];
		 }
		 errorL[nL + i] = l->error[i];
		 }
		 nL += l->nbFeature;
		 }
		 }


//#pragma omp parallel for
		for (int k = 0; k < points[scaleLevel].size(); k++) {
			apControlPoint *p = (points[scaleLevel])[k];
			p->computeInteractionMatrixErrorMH(cMo, _I);
			for (int j = 0; j < 6; j++) {
				L[k][j] = p->L[j];

				error[k] = p->error;
			}
		}

		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 1;

			robust.setThreshold(2 / cam.get_px()); // limite en metre
			robust.MEstimator(vpRobust::TUKEY, error, w);
			//robust.setThreshold(2); // limite en pixel
			//robust.MEstimator(vpRobust::TUKEY, error_px,w);

			weighted_errorL.resize(nerrorL);
			wL.resize(nerrorL);
			wL = 1;

			robustL.setThreshold(2 / cam.get_px()); // limite en metre
			robustL.MEstimator(vpRobust::TUKEY, errorL, wL);

		} else {
			robust.setIteration(iter);
			robust.MEstimator(vpRobust::TUKEY, error, w);
			//robust.MEstimator(vpRobust::TUKEY, error_px,w);

			robustL.setIteration(iter);
			robustL.MEstimator(vpRobust::TUKEY, errorL, wL);
			//robust.MEstimator(vpRobust::TUKEY, error_px,w);
		}

		residu_1 = r;

		L_true = L;
		W_true = vpColVector(nerror);

		LL_true = LL;
		WL_true = vpColVector(nerrorL);



		//---------------------------------------
		// Compute the mean of the weight for a given line reinit if necessary
		nL = 0;
		for (int k = 0; k < lines[scaleLevel].size(); k++) {
		 l = lines[scaleLevel][k];
		 if(l->MHmeline->points_vect.size()>0)
		 {
		 double wmean = 0;
		 for (int i = 0; i < l->nbFeature; i++) {
		 if (iter == 11)
		 {
		 wL[nL+i] = l->weight[i]*l->weight[i]*wL[nL+i];
		 //std::cout << " weight " << w[n+i] <<std::endl;
		 }
		 wmean += wL[nL + i];

		 }
		 nL += l->nbFeature;
		 wmean /= l->nbFeature;
		 if (l->nbFeature == 0)
		 wmean = 1;
		 l->wmean = wmean;

		 }
		 }

		double num = 0;
		double den = 0;
		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			W_true = wi * wi;
			eri = error[i];
			num += wi * vpMath::sqr(eri);
			den += wi;

			weighted_error[i] = wi * eri;
		}

		r = sqrt(num / den); //Le critere d'arret prend en compte le poids

		double numL = 0;
		double denL = 0;
		double wiL;
		double eriL;
		for (int i = 0; i < nerrorL; i++) {
			wiL = wL[i] * factorL[i];
			WL_true = wiL * wiL;
			eriL = errorL[i];
			numL += wiL * vpMath::sqr(eriL);
			denL += wiL;

			weighted_errorL[i] = wiL * eriL;
		}

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					L[i][j] = w[i] * factor[i] * L[i][j];
				}
			}

			for (int i = 0; i < nerrorL; i++) {
				for (int j = 0; j < 6; j++) {
					LL[i][j] = wL[i] * factorL[i] * LL[i][j];
				}
			}
		}

		//std::cout << " LL" << LL << std::endl;

		//        std::cout << "t1 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		robustCCD.setIteration(iter);
		CCDTracker.updateCCDPoints(cMo);
		//        std::cout << "t2a = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		CCDTracker.computeLocalStatistics();
		//        std::cout << "t2b = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		//		double t0 = vpTime::measureTimeMs();
		//CCDTracker.updateParametersRobustPrev(LTCIL, LTCIR, robustCCD);
		CCDTracker.updateParametersPrev(LTCIL, LTCIR);
		//CCDTracker.updateParameters(LTCIL,LTCIR);
		//		double t1 = vpTime::measureTimeMs();
		//std::cout << " timeupdate " << t1 -t0 << std::endl;
		if (iter > 0)
			CCDTracker.checkCCDConvergence();
		//        std::cout << "t2c = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();

		vpMatrix::stackMatrices(L, LL, LH);
		 vpMatrix::stackMatrices(weighted_error, weighted_errorL,
		 weighted_errorH);

		LH.AtA(LTL);
		//LTL = L.AtA();
		//L.AtA(LTL);
		computeJTR(LH, weighted_errorH, LTR);

		double stdME = sqrt((double)(weighted_errorH.t()*weighted_errorH)/(weighted_errorH.size()));

		double wghtME = ((double)1/weighted_errorH.size())*(1/stdME);
		double wghtCCD = ((double)1/CCDTracker.error_ccd.size());

		wghtME = 1;

		//std::cout << "t3 = " << vpTime::measureTimeMs() - t0 << std::endl;

		v = -lambda * (weight_me * wghtME*LTL + weight_ccd * wghtCCD * LTCIL).pseudoInverse(LTL.getRows() * DBL_EPSILON) * (weight_me*wghtME*LTR - weight_ccd * wghtCCD *  LTCIR);



		//v = -lambda * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows()* DBL_EPSILON) * (LTR - weight_ccd * LTCIR);
		cMo = vpExponentialMap::direct(v).inverse() * cMo;

		iter++;
	}
	if (computeCovariance) {
		vpMatrix D;
		D.diag(W_true);
		covarianceMatrixP = computeCovarianceMatrix(L_true, -v, lambda * error,D);
		vpMatrix DL;
		DL.diag(WL_true);
		covarianceMatrixL = computeCovarianceMatrix(LL_true, -v, lambda * errorL,DL);

		vpMatrix::stackMatrices(W_true, WL_true, WH_true);
		vpMatrix::stackMatrices(error, errorL, errorH);
		vpMatrix::stackMatrices(L_true, LL_true, LH_true);
		WH_true = vpColVector(nerror+nerrorL);
		vpMatrix DH;
		DH.diag(WH_true);
		covarianceMatrixME = computeCovarianceMatrix(LH_true, -v, lambda * errorH,DH);

		covarianceMatrixCCD = CCDTracker.sigmaP;

		vpMatrix inv = (weight_me*(covarianceMatrixME.pseudoInverse()) + weight_ccd*(covarianceMatrixCCD.pseudoInverse()));
		covarianceMatrix = inv.pseudoInverse();

	}

	//   cout << "\t Robust minimization in " << iter << " iteration " << endl ;
	//    std::cout << "error: " << (residu_1 - r) << std::endl;
}

void apMbTracker::computeVVSPointsLinesCCDMHPrevSpace(const vpImage<unsigned char>& _I, const vpImage<vpRGBa>& _IRGB) {
	double residu_1 = 1e3;
	double r = 1e3 - 1;
	double residu_1L = 1e3;
	double rL = 1e3 - 1;
	vpMatrix LTL;
	vpColVector LTR;

	CCDTracker.init(CCDParameters, cam);
	CCDTracker.setImage(_IRGB);
	CCDTracker.setPrevImage(IprecRGB);
	CCDTracker.updateCCDPoints(cMo);
	CCDTracker.computeLocalStatisticsPrevSpace();

	// compute the interaction matrix and its pseudo inverse
	apMbtDistanceLineMH *l, *l1, *l2;

	// compute the interaction matrix and its pseudo inverse
	vpColVector w;
	vpColVector weighted_error;
	vpColVector factor;

	vpColVector wL;
	vpColVector weighted_errorL;
	vpColVector factorL;

	vpColVector weighted_errorH;

	int iter = 0;

	//Nombre de moving edges
	int nbrow = 0;
	int nbrowL = 0;
	vpFeatureLine fli;

	lines[scaleLevel][0]->getCameraParameters(&cam);
	std::cout << " nb lines clusters " << lines[scaleLevel].size() << std::endl;
	for (int k = 0; k < lines[scaleLevel].size(); k++) {
		l = lines[scaleLevel][k];
		if (l->MHmeline->points_vect.size() > 0) {
			l->initInteractionMatrixErrorMH();
			nbrowL += l->nbFeature;
			//std::cout << " nb features " << l->nbFeature << std::endl;
		}
	}
	if (nbrowL == 0) {
		vpERROR_TRACE(
				"\n\t\t Error-> not enough data in the interaction matrix...");
		throw vpTrackingException(vpTrackingException::notEnoughPointError,
				"\n\t\t Error-> not enough data in the interaction matrix...");
	}
	//std::cout << " ok02 " << std::endl;
	const size_t scaleLevel_size = points[scaleLevel].size();
	for (int k = 0; k < scaleLevel_size; k++) {
		apControlPoint *p = (points[scaleLevel])[k];
		nbrow += 1;
		p->initInteractionMatrixError();

		//std::cout<<fli.getTheta()<<std::endl;
	}
	if (nbrow == 0) {
		vpERROR_TRACE(
				"\n\t\t Error-> not enough data in the interaction matrix...");
		throw vpTrackingException(vpTrackingException::notEnoughPointError,
				"\n\t\t Error-> not enough data in the interaction matrix...");
	}

	vpMatrix L(nbrow, 6), Lp;
	vpMatrix LL(nbrowL, 6), LpL;
	vpMatrix LH;
	vpMatrix LTCIL(6, 6);
	vpColVector LTCIR(6);

	// compute the error vector
	vpColVector error(nbrow);
	int nerror = error.getRows();

	vpColVector errorL(nbrowL);
	int nerrorL = errorL.getRows();


	vpColVector v;
	double limite = 3; //Une limite de 3 pixels
	limite = limite / cam.get_px(); //Transformation limite pixel en limite metre.

	//Parametre pour la premiere phase d'asservissement
	double e_prev = 0, e_cur, e_next;
	bool reloop = true;
	double count = 0;

	double e_prevL = 0, e_curL, e_nextL;
	bool reloopL = true;
	double countL = 0;
	/*** First phase ***/

	while (reloop == true && iter < 3) {
		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 0;
			factor.resize(nerror);
			factor = 1;

			weighted_errorL.resize(nerrorL);
			wL.resize(nerrorL);
			wL = 0;
			factorL.resize(nerrorL);
			factorL = 1;
		}

		countL = 0;
		int nL = 0;
		//reloop = false;
		reloopL = true;


		/*for (int k = 0; k < lines[scaleLevel].size(); k++) {
		 l = lines[scaleLevel][k];
		 if(l->MHmeline->points_vect.size()>0)
		 {
		 l->computeInteractionMatrixErrorMH(_I, cMo);
		 double fac = 1;
		 //std::cout << " ok0001 " << std::endl;
		 for (int i = 0; i < l->nbFeature; i++) {
		errorL[nL + i] = l->error[i];
		 for (int j = 0; j < 6; j++) {
		 LL[nL + i][j] = l->L[i][j];
		 //std::cout << " ok0001 " << errorL[nL + i]<< " LL " << LL[nL + i][j] << std::endl;
		 }
		 if (errorL[nL + i] <= limite)
		 countL++;


		 wL[nL + i] = 1;
		 if (i == 0) {
		 e_curL = l->error[0];
		 if (l->nbFeature > 1)
		 e_nextL = l->error[1];
		 if (fabs(e_curL - e_nextL) < limite && vpMath::sign(e_curL)
		 == vpMath::sign(e_nextL)) {
		 wL[nL + i] = 1;//0.5
		 }
		 e_prevL = e_curL;
		 }

		 //else w[n+i] = 1;
		 //}
		 else if (i == l->nbFeature - 1) {
		 e_curL = l->error[i];
		 if (fabs(e_curL - e_prevL) < limite && vpMath::sign(e_curL)
		 == vpMath::sign(e_prevL)) {
		 wL[nL + i] = 1;//0.5;
		 }
		 }
		 else {
		 e_curL = l->error[i];
		 e_nextL = l->error[i + 1];
		 if (fabs(e_curL - e_prevL) < limite) {
		 wL[nL + i] += 0.5;
		 }
		 if (fabs(e_cur - e_next) < limite) {
		 wL[nL + i] += 0.5;
		 }
		 e_prevL = e_curL;
		 }

		 wL[nL + i] = 1;

		 }

		 nL += l->nbFeature;

		 }
		}*/

		countL = countL / (double) nbrowL;
		if (countL < 0.85) {
			reloopL = true;
		}

		count = 0;
		int n = 0;
		//reloop = false;
		reloop = true;

		if (iter == 0)
			(points[scaleLevel])[0]->computeInteractionMatrixErrorMH(cMo, _I);
		else
		{
//#pragma omp parallel for
			for (int k = 0; k < scaleLevel_size; ++k) {
				apControlPoint *p = (points[scaleLevel])[k];
				p->computeInteractionMatrixErrorMH(cMo, _I);
				//p->computeInteractionMatrixError2(cMo,_I);

				for(int j=0;j<6;j++)
					L[k][j] = p->L[j];

				error[k] = p->error; //On remplit la matrice d'erreur

				if (error[k] <= limite)
					count = count + 1.0; //Si erreur proche de 0 on incremente cur

				w[k] = 0;

				e_next = e_cur = p->error;
				if (fabs(e_cur - e_prev) < limite)
					w[k] += 1.0;
				else
					w[k] += 0.5;
				e_prev = e_cur;
			}
		}

		count = count / (double) nbrow;
		if (count < 0.85) {
			reloop = true;
		}

		double num = 0;
		double den = 0;

		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			eri = error[i];
			num += wi * vpMath::sqr(eri);
			den += wi;
			weighted_error[i] = wi * eri;
		}

		double numL = 0;
		double denL = 0;

		double wiL;
		double eriL;
		for (int i = 0; i < nerrorL; i++) {
			wiL = wL[i] * factorL[i];
			eriL = errorL[i];
			numL += wiL * vpMath::sqr(eriL);
			denL += wiL;
			weighted_errorL[i] = wiL * eriL;
		}

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					//L[i][j] = w[i]*factor[i]*L[i][j] ;
					L[i][j] = factor[i] * L[i][j];
				}
			}

			for (int i = 0; i < nerrorL; i++) {
				for (int j = 0; j < 6; j++) {
					//L[i][j] = w[i]*factor[i]*L[i][j] ;
					LL[i][j] = factorL[i] * LL[i][j];
				}
			}
		}

		//        std::cout << "t-2 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		CCDTracker.updateCCDPoints(cMo);
		CCDTracker.computeLocalStatisticsSpace();
		//        std::cout << "t-1 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		CCDTracker.updateParametersPrev(LTCIL, LTCIR);
		//        double t1 = vpTime::measureTimeMs();
		//        std::cout << "t0 = " << vpTime::measureTimeMs() - t0 << std::endl;

		if (iter > 0)
			CCDTracker.checkCCDConvergence();

		/*vpMatrix::stackMatrices(L, LL, LH);
		 vpMatrix::stackMatrices(weighted_error, weighted_errorL,
		 weighted_errorH);*/

				LTL = L.AtA();
		//LH.AtA(LTL);
		L.AtA(LTL);
		computeJTR(L, weighted_error, LTR);
		//std::cout << " LTL0 " << LTL << std::endl;
	     //std::cout << " LTL1 " << LTL << std::endl;
		//v = -0.7 * (LTL).pseudoInverse(LTL.getRows()* DBL_EPSILON) * (LTR);
		v = -0.7 * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows()* DBL_EPSILON) * (LTR - weight_ccd * LTCIR);
		cMo = vpExponentialMap::direct(v).inverse() * cMo;

		//std::cout << " cMo " << cMo << std::endl;

		iter++;
	}
	std::cout << "\t First minimization in " << iter << " iteration00 "
			<< std::endl;

	/*** Second phase ***/

	vpColVector W_true;
	vpMatrix L_true;
	vpRobust robust(nerror);
	robust.setIteration(0);

	vpRobust robustL(nerrorL);
	robustL.setIteration(0);

	vpRobust robustCCD(CCDTracker.nerror_ccd);
	robustCCD.setIteration(0);
	robustCCD.setThreshold(2 / cam.get_px());

	iter = 0;
	//vpColVector error_px(nerror);

	while (((int) ((residu_1 - r) * 1e8) != 0) && (iter < 10)) {

		int nL = 0;
		for (int k = 0; k < lines[scaleLevel].size(); k++) {
		 l = lines[scaleLevel][k];
		 if(l->MHmeline->points_vect.size()>0)
		 {
		 l->computeInteractionMatrixErrorMH(_I, cMo);
		 for (int i = 0; i < l->nbFeature; i++) {
		 for (int j = 0; j < 6; j++){
		 LL[nL + i][j] = l->L[i][j];
		 }
		 errorL[nL + i] = l->error[i];
		 }
		 nL += l->nbFeature;
		 }
		 }


//#pragma omp parallel for
		for (int k = 0; k < points[scaleLevel].size(); k++) {
			apControlPoint *p = (points[scaleLevel])[k];
			p->computeInteractionMatrixErrorMH(cMo, _I);
			for (int j = 0; j < 6; j++) {
				L[k][j] = p->L[j];

				error[k] = p->error;
			}
		}

		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 1;

			robust.setThreshold(2 / cam.get_px()); // limite en metre
			robust.MEstimator(vpRobust::TUKEY, error, w);
			//robust.setThreshold(2); // limite en pixel
			//robust.MEstimator(vpRobust::TUKEY, error_px,w);

			weighted_errorL.resize(nerrorL);
			wL.resize(nerrorL);
			wL = 1;

			robustL.setThreshold(2 / cam.get_px()); // limite en metre
			robustL.MEstimator(vpRobust::TUKEY, errorL, wL);

		} else {
			robust.setIteration(iter);
			robust.MEstimator(vpRobust::TUKEY, error, w);
			//robust.MEstimator(vpRobust::TUKEY, error_px,w);

			robustL.setIteration(iter);
			robustL.MEstimator(vpRobust::TUKEY, errorL, wL);
			//robust.MEstimator(vpRobust::TUKEY, error_px,w);
		}

		residu_1 = r;

		L_true = L;
		W_true = vpColVector(nerror);

		//---------------------------------------
		// Compute the mean of the weight for a given line reinit if necessary
		nL = 0;
		for (int k = 0; k < lines[scaleLevel].size(); k++) {
		 l = lines[scaleLevel][k];
		 if(l->MHmeline->points_vect.size()>0)
		 {
		 double wmean = 0;
		 for (int i = 0; i < l->nbFeature; i++) {
		 if (iter < 1)
		 {
		 wL[nL+i] = l->weight[i]*wL[nL+i];
		 //std::cout << " weight " << w[n+i] <<std::endl;
		 }
		 wmean += wL[nL + i];

		 }
		 nL += l->nbFeature;
		 wmean /= l->nbFeature;
		 if (l->nbFeature == 0)
		 wmean = 1;
		 l->wmean = wmean;

		 }
		 }

		double num = 0;
		double den = 0;
		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			W_true = wi * wi;
			eri = error[i];
			num += wi * vpMath::sqr(eri);
			den += wi;

			weighted_error[i] = wi * eri;
		}

		r = sqrt(num / den); //Le critere d'arret prend en compte le poids

		double numL = 0;
		double denL = 0;
		double wiL;
		double eriL;
		for (int i = 0; i < nerrorL; i++) {
			wiL = wL[i] * factorL[i];
			//W_trueL = wiL * wiL;
			eriL = errorL[i];
			numL += wiL * vpMath::sqr(eriL);
			denL += wiL;

			weighted_errorL[i] = wiL * eriL;
		}

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					L[i][j] = w[i] * factor[i] * L[i][j];
				}
			}

			for (int i = 0; i < nerrorL; i++) {
				for (int j = 0; j < 6; j++) {
					LL[i][j] = wL[i] * factorL[i] * LL[i][j];
				}
			}
		}

		//std::cout << " LL" << LL << std::endl;

		//        std::cout << "t1 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		robustCCD.setIteration(iter);
		CCDTracker.updateCCDPoints(cMo);
		//        std::cout << "t2a = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		CCDTracker.computeLocalStatisticsSpace();
		//        std::cout << "t2b = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		//		double t0 = vpTime::measureTimeMs();
		CCDTracker.updateParametersRobustPrev(LTCIL, LTCIR, robustCCD);
		//CCDTracker.updateParameters(LTCIL,LTCIR);
		//		double t1 = vpTime::measureTimeMs();
		//std::cout << " timeupdate " << t1 -t0 << std::endl;
		if (iter > 0)
			CCDTracker.checkCCDConvergence();
		//        std::cout << "t2c = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();

		vpMatrix::stackMatrices(L, LL, LH);
		 vpMatrix::stackMatrices(weighted_error, weighted_errorL,
		 weighted_errorH);

		LH.AtA(LTL);
		//LTL = L.AtA();
		//L.AtA(LTL);
		computeJTR(LH, weighted_errorH, LTR);
		v = -lambda * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows()* DBL_EPSILON) * (LTR - weight_ccd * LTCIR);
		cMo = vpExponentialMap::direct(v).inverse() * cMo;

		iter++;
	}
	if (computeCovariance) {
		vpMatrix D;
		D.diag(W_true);
		covarianceMatrix = computeCovarianceMatrix(L_true, -v, lambda * error,
				D);
	}

	//  std::cout << "iter = " << iter <<std::endl;
	int n = 0;
	for (int k = 0; k < points[scaleLevel].size(); k++) {
		apControlPoint *p = (points[scaleLevel])[k];
		{
			double wmean = 0;
			wmean += w[n];
			vpPointSite &site = p->s;
			if (w[n] < 0.5)
				site.suppress = 4;

			n += 1;

			wmean = 1;

			//p->setMeanWeight(wmean);

		}

	}
	//   cout << "\t Robust minimization in " << iter << " iteration " << endl ;
	//    std::cout << "error: " << (residu_1 - r) << std::endl;
}

/*!
 Compute the visual servoing loop to get the pose of the feature set.

 \exception vpTrackingException::notEnoughPointError if the number of detected
 feature is equal to zero.

 \param _I : The current greyscale image.
 \param _IRGB : The current RGB image.
 */
void apMbTracker::computeVVSCCD(const vpImage<unsigned char>& _I,
		const vpImage<vpRGBa>& _IRGB) {
	double residu_1 = 1e3;
	double r = 1e3 - 1;
	vpMatrix LTL;
	vpColVector LTR;

	// compute the interaction matrix and its pseudo inverse
	apControlPoint *p;

	vpColVector w;
	vpColVector weighted_error;
	vpColVector factor;
	CCDTracker.init(CCDParameters, cam);
	CCDTracker.setImage(_IRGB);

	covarianceMatrix.resize(6,6);
	covarianceMatrixME.resize(6,6);
	covarianceMatrixCCD.resize(6,6);

	unsigned int iter = 0;

	//Nombre de moving edges
	int nbrow = 0;
	vpFeatureLine fli;

	for (int k = 0; k < points[scaleLevel].size(); k++) {
		p = (points[scaleLevel])[k];
		nbrow += 1;
		p->initInteractionMatrixError();
	}

	if (nbrow == 0) {
		vpERROR_TRACE(
				"\n\t\t Error-> not enough data in the interaction matrix...");
		throw vpTrackingException(vpTrackingException::notEnoughPointError,
				"\n\t\t Error-> not enough data in the interaction matrix...");
	}

	vpMatrix L(nbrow, 6), Lp;

	vpMatrix LTCIL(6, 6);
	vpColVector LTCIR(6);

	// compute the error vector
	vpColVector error(nbrow);
	int nerror = error.getRows();
	vpColVector v;

	double limite = 3; //Une limite de 3 pixels
	limite = limite / cam.get_px(); //Transformation limite pixel en limite metre.

	//Parametre pour la premiere phase d'asservissement
	double e_prev = 0, e_cur, e_next;
	bool reloop = true;
	double count = 0;

	vpColVector W_true;
	vpMatrix L_true;

	/*** First phase ***/

	while (iter < 12) {
		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 1;
			factor.resize(nerror);
			factor = 1;
		}

		count = 0;
		int n = 0;
		//reloop = false;
		reloop = true;
		for (int k = 0; k < points[scaleLevel].size(); k++) {
			p = (points[scaleLevel])[k];
			p->computeInteractionMatrixError(cMo, _I);

			double fac = 1;
			/*if (iter == 0)
			 {
			 fac = 0.2;
			 break;
			 }*/

			if (iter == 0 && p != NULL)

				for (int j = 0; j < 6; j++) {
					L[n][j] = p->L[j]; //On remplit la matrice d'interaction globale
				}
			error[n] = p->error; //On remplit la matrice d'erreur

			if (error[n] <= limite)
				count = count + 1.0; //Si erreur proche de 0 on incremente cur

			w[n] = 1;

			if (iter == 0) {
				factor[n] = fac;
				vpPointSite site = p->s;
				//if (site.suppress != 0) factor[n] = 0;
				if (site.suppress != 0)
					factor[n] = 0.2;
			}
			n += 1;
		}

		count = count / (double) nbrow;
		if (count < 0.85) {
			reloop = true;
		}

		double num = 0;
		double den = 0;

		L_true = L;
		W_true = vpColVector(nerror);

		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			eri = error[i];
			W_true =
			num += wi * vpMath::sqr(eri);
			den += wi;
			W_true[i] = wi * wi;
			weighted_error[i] = wi * eri;
		}

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
                                for (int j = 0; j < 3; j++) {
					L[i][j] = w[i] * factor[i] * L[i][j];
				}
                                if (fixedrotationx==1) L[i][3] = 0;
                                if (fixedrotationy==2) L[i][4] = 0;
                                if (fixedrotationz==3) L[i][5] = 0;
			}
		}


		CCDTracker.updateCCDPoints(cMo);
		CCDTracker.computeLocalStatistics();
		double t0 = vpTime::measureTimeMs();
		CCDTracker.updateParameters(LTCIL, LTCIR);
		double t1 = vpTime::measureTimeMs();
		//std::cout << " timeupdate " << t1 -t0 << std::endl;

		if (iter > 0)
			CCDTracker.checkCCDConvergence();

		LTL = L.AtA();
		computeJTR(L, weighted_error, LTR);

		double stdME = sqrt((double)(weighted_error.t()*weighted_error)/(weighted_error.size()));

		double wghtME = ((double)1/weighted_error.size())*(1/stdME);
		double wghtCCD = ((double)1/(10*CCDTracker.error_ccd.size()));

		wghtME = 0.7;
		//std::cout << "t3 = " <<  weight_me*wghtME*LTR << std::endl;

                //std::cout << "t4 = " << -weight_ccd * wghtCCD *  LTCIR << std::endl;
                std::cout << "v " << v << std::endl;

		//wghtME = 1;
		v = - lambda * (weight_me * wghtME*LTL + weight_ccd * wghtCCD * LTCIL).pseudoInverse(LTL.getRows() * DBL_EPSILON) * (weight_me*wghtME*LTR - weight_ccd * wghtCCD *  LTCIR);
                /*vpRxyzVector veuler;
                vpThetaUVector tu;
                tu[0] = v[0];
                tu[1] = v[1];
                tu[2] = v[2];
                veuler.buildFrom(tu);
                veuler[1] = 0;
                tu.buildFrom(veuler);*/
                //v[3] = tu[0];
                //v[4] = 0;
                //v[5] = tu[2];
		//v = -0.7 * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows()* DBL_EPSILON) * (LTR - weight_ccd * LTCIR);
		cMo = vpExponentialMap::direct(v).inverse() * cMo;

		//std::cout << "t5 = " << weight_ccd/wghtCCD << std::endl;


		iter++;
	}
	/*std::cout << "\t First minimization in " << iter << " iteration "
	 << std::endl;*/

	/*** Second phase ***/

	/*vpRobust robust(nerror);
	robust.setIteration(0);

	vpRobust robustCCD(CCDTracker.nerror_ccd);
	robustCCD.setIteration(0);
	robustCCD.setThreshold(2 / cam.get_px());

	//CCDTracker.initRobust();


	iter = 0;
	//vpColVector error_px(nerror);

	while (((int) ((residu_1 - r) * 1e8) != 0) && (iter < 1)) {
		int n = 0;
		for (int k = 0; k < points[scaleLevel].size(); k++) {
			p = (points[scaleLevel])[k];
			p->computeInteractionMatrixError(cMo, _I);
			for (int j = 0; j < 6; j++) {
				L[n][j] = p->L[j];
				error[n] = p->error;
			}
			n += 1;
		}

		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 1;

			robust.setThreshold(2 / cam.get_px()); // limite en metre
			robust.MEstimator(vpRobust::TUKEY, error, w);
		} else {
			robust.setIteration(iter);
			robust.MEstimator(vpRobust::TUKEY, error, w);
		}

		residu_1 = r;

		L_true = L;
		W_true = vpColVector(nerror);

		double num = 0;
		double den = 0;
		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			eri = error[i];
			W_true[i] = wi * wi;
			num += wi * vpMath::sqr(eri);
			den += wi;

			weighted_error[i] = wi * eri;
		}

		r = sqrt(num / den); //Le critere d'arret prend en compte le poids

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					L[i][j] = w[i] * factor[i] * L[i][j];
				}
			}
		}

		robustCCD.setIteration(iter);
		CCDTracker.updateCCDPoints(cMo);
		CCDTracker.computeLocalStatistics();
		double t0 = vpTime::measureTimeMs();
		//CCDTracker.updateParametersRobust(LTCIL, LTCIR, robustCCD);
		CCDTracker.updateParameters(LTCIL,LTCIR);
		double t1 = vpTime::measureTimeMs();
		//std::cout << " timeupdate " << t1 -t0 << std::endl;
		if (iter > 0)
			CCDTracker.checkCCDConvergence();

		LTL = L.AtA();
		computeJTR(L, weighted_error, LTR);

		double stdME = sqrt((double)(weighted_error.t()*weighted_error)/(weighted_error.size()));

		double wghtME = ((double)1/weighted_error.size())*(1/stdME);
		double wghtCCD = ((double)1/CCDTracker.error_ccd.size());

		wghtME = 1;

		//std::cout << "t3 = " << vpTime::measureTimeMs() - t0 << std::endl;

		v = -lambda * (weight_me * wghtME*LTL + weight_ccd * wghtCCD * LTCIL).pseudoInverse(LTL.getRows() * DBL_EPSILON) * (weight_me*wghtME*LTR - weight_ccd * wghtCCD *  LTCIR);

		//v = -lambda * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows()* DBL_EPSILON) * (LTR - weight_ccd * LTCIR);
		//cMo = vpExponentialMap::direct(v).inverse() * cMo;

		iter++;
	}*/
	if (computeCovariance) {
		vpMatrix D;
		D.diag(W_true);
		covarianceMatrixME = computeCovarianceMatrix(L_true, -v, lambda * error,
				D);

		covarianceMatrixCCD = CCDTracker.sigmaP;

		vpMatrix inv = (weight_me*(covarianceMatrixME.pseudoInverse()) + weight_ccd*(covarianceMatrixCCD.pseudoInverse()) );
		covarianceMatrix = inv.pseudoInverse();

	}
	//   cout << "\t Robust minimization in " << iter << " iteration " << endl ;
	//    std::cout << "error: " << (residu_1 - r) << std::endl;
}

void apMbTracker::computeVVSCCDPrev(const vpImage<unsigned char>& _I,
		const vpImage<vpRGBa>& _IRGB) {
	double residu_1 = 1e3;
	double r = 1e3 - 1;
	vpMatrix LTL;
	vpColVector LTR;

	// compute the interaction matrix and its pseudo inverse
	apControlPoint *p;

	vpColVector w;
	vpColVector weighted_error;
	vpColVector factor;
	CCDTracker.init(CCDParameters, cam);
	CCDTracker.setImage(_IRGB);
	CCDTracker.setPrevImage(IprecRGB);

	CCDTracker.updateCCDPoints(cMo);
	CCDTracker.computeLocalStatisticsPrev(_I);

	covarianceMatrix.resize(6,6);
	covarianceMatrixME.resize(6,6);
	covarianceMatrixCCD.resize(6,6);

	unsigned int iter = 0;

	//Nombre de moving edges
	int nbrow = 0;
	vpFeatureLine fli;

	for (int k = 0; k < points[scaleLevel].size(); k++) {
		p = (points[scaleLevel])[k];
		nbrow += 1;
		p->initInteractionMatrixError();
	}

	if (nbrow == 0) {
		vpERROR_TRACE(
				"\n\t\t Error-> not enough data in the interaction matrix...");
		throw vpTrackingException(vpTrackingException::notEnoughPointError,
				"\n\t\t Error-> not enough data in the interaction matrix...");
	}

	vpMatrix L(nbrow, 6), Lp;

	vpMatrix LTCIL(6, 6);
	vpColVector LTCIR(6);

	// compute the error vector
	vpColVector error(nbrow);
	int nerror = error.getRows();
	vpColVector v;

	double limite = 3; //Une limite de 3 pixels
	limite = limite / cam.get_px(); //Transformation limite pixel en limite metre.

	//Parametre pour la premiere phase d'asservissement
	double e_prev = 0, e_cur, e_next;
	bool reloop = true;
	double count = 0;

	/*** First phase ***/

	while (reloop == true && iter < 1) {
		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 1;
			factor.resize(nerror);
			factor = 1;
		}

		count = 0;
		int n = 0;
		//reloop = false;
		reloop = true;
		for (int k = 0; k < points[scaleLevel].size(); k++) {
			p = (points[scaleLevel])[k];
			p->computeInteractionMatrixError(cMo, _I);

			double fac = 1;
			/*if (iter == 0)
			 {
			 fac = 0.2;
			 break;
			 }*/

			if (iter == 0 && p != NULL)

				for (int j = 0; j < 6; j++) {
					L[n][j] = p->L[j]; //On remplit la matrice d'interaction globale
				}
			error[n] = p->error; //On remplit la matrice d'erreur

			if (error[n] <= limite)
				count = count + 1.0; //Si erreur proche de 0 on incremente cur

			w[n] = 1;

			if (iter == 0) {
				factor[n] = fac;
				vpPointSite site = p->s;
				//if (site.suppress != 0) factor[n] = 0;
				if (site.suppress != 0)
					factor[n] = 0.2;
			}
			n += 1;
		}

		count = count / (double) nbrow;
		if (count < 0.85) {
			reloop = true;
		}

		/*
		double num = 0;
		double den = 0;

		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			eri = error[i];
			num += wi * vpMath::sqr(eri);
			den += wi;

			weighted_error[i] = wi * eri;
		}

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					L[i][j] = w[i] * factor[i] * L[i][j];
				}
			}
		}

		CCDTracker.updateCCDPoints(cMo);
		CCDTracker.computeLocalStatistics();
		double t0 = vpTime::measureTimeMs();
		CCDTracker.updateParametersPrev(LTCIL, LTCIR);
		double t1 = vpTime::measureTimeMs();
		//std::cout << " timeupdate " << t1 -t0 << std::endl;

		if (iter > 0)
			CCDTracker.checkCCDConvergence();

		LTL = L.AtA();
		computeJTR(L, weighted_error, LTR);
		v = -1 * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows()
				* DBL_EPSILON) * (LTR - weight_ccd * LTCIR);
		//cMo = vpExponentialMap::direct(v).inverse() * cMo;

		*/

		iter++;
	}
	/*std::cout << "\t First minimization in " << iter << " iteration "
	 << std::endl;*/

	/*** Second phase ***/
	vpColVector W_true;
	vpMatrix L_true;
	vpRobust robust(nerror);
	robust.setIteration(0);

	vpRobust robustCCD(CCDTracker.nerror_ccd);
	robustCCD.setIteration(0);
	robustCCD.setThreshold(2 / cam.get_px());

	//CCDTracker.initRobust();

	iter = 0;
	//vpColVector error_px(nerror);

	while (((int) ((residu_1 - r) * 1e8) != 0) && (iter < 12)) {
		int n = 0;
		for (int k = 0; k < points[scaleLevel].size(); k++) {
			p = (points[scaleLevel])[k];
			p->computeInteractionMatrixError(cMo, _I);
			for (int j = 0; j < 6; j++) {
				L[n][j] = p->L[j];
				error[n] = p->error;
			}
			n += 1;
		}

		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 1;

			robust.setThreshold(2 / cam.get_px()); // limite en metre
			robust.MEstimator(vpRobust::TUKEY, error, w);
		} else {
			robust.setIteration(iter);
			robust.MEstimator(vpRobust::TUKEY, error, w);
		}

		residu_1 = r;

		L_true = L;
		W_true = vpColVector(nerror);

		double num = 0;
		double den = 0;
		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			eri = error[i];
			W_true[i] = wi * wi;
			num += wi * vpMath::sqr(eri);
			den += wi;

			weighted_error[i] = wi * eri;
		}

		r = sqrt(num / den); //Le critere d'arret prend en compte le poids

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					L[i][j] = w[i] * factor[i] * L[i][j];
				}
			}
		}

		robustCCD.setIteration(iter);
		CCDTracker.updateCCDPoints(cMo);
		CCDTracker.computeLocalStatistics();
		double t0 = vpTime::measureTimeMs();
		//CCDTracker.updateParametersRobustPrev(LTCIL, LTCIR, robustCCD);
		CCDTracker.updateParametersPrev(LTCIL,LTCIR);
		double t1 = vpTime::measureTimeMs();
		//std::cout << " timeupdate " << t1 -t0 << std::endl;
		if (iter > 0)
			CCDTracker.checkCCDConvergence();

		LTL = L.AtA();
		computeJTR(L, weighted_error, LTR);

		double stdME = sqrt((double)(weighted_error.t()*weighted_error)/(weighted_error.size()));

		double wghtME = ((double)1/weighted_error.size())*(1/stdME);
		double wghtCCD = ((double)1/CCDTracker.error_ccd.size());

		wghtME = 1;

		v = -lambda * (weight_me* wghtME*LTL + weight_ccd* wghtCCD * LTCIL).pseudoInverse(LTL.getRows()* DBL_EPSILON) * (weight_me* wghtME*LTR - weight_ccd * wghtCCD * LTCIR);


		//v = -lambda * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows() * DBL_EPSILON) * (LTR - weight_ccd * LTCIR);
		cMo = vpExponentialMap::direct(v).inverse() * cMo;

		iter++;
	}
	if (computeCovariance) {

		vpMatrix D;
		D.diag(W_true);
		covarianceMatrixME = computeCovarianceMatrix(L_true, -v, lambda * error,
				D);

		covarianceMatrixCCD = CCDTracker.sigmaP;

		vpMatrix inv = (weight_me*(covarianceMatrixME.pseudoInverse()) + weight_ccd*(covarianceMatrixCCD.pseudoInverse()) );
				covarianceMatrix = inv.pseudoInverse();
	}
	//   cout << "\t Robust minimization in " << iter << " iteration " << endl ;
	//    std::cout << "error: " << (residu_1 - r) << std::endl;
}


void apMbTracker::computeVVSCCDMH(const vpImage<unsigned char>& _I,
		const vpImage<vpRGBa>& _IRGB) {
	double residu_1 = 1e3;
	double r = 1e3 - 1;
	vpMatrix LTL;
	vpColVector LTR;

	// compute the interaction matrix and its pseudo inverse

	vpColVector w;
	vpColVector weighted_error;
	vpColVector factor;

	CCDTracker.init(CCDParameters, cam);
	CCDTracker.setImage(_IRGB);

	unsigned int iter = 0;

	//Nombre de moving edges
	int nbrow = 0;

#pragma omp parallel for
	for (int k = 0; k < points[scaleLevel].size(); k++) {
		apControlPoint *p = (points[scaleLevel])[k];
		p->initInteractionMatrixError();
	}
	nbrow = points[scaleLevel].size();

	if (nbrow == 0) {
		vpERROR_TRACE(
				"\n\t\t Error-> not enough data in the interaction matrix...");
		throw vpTrackingException(vpTrackingException::notEnoughPointError,
				"\n\t\t Error-> not enough data in the interaction matrix...");
	}

	vpMatrix L(nbrow, 6);

	vpMatrix LTCIL(6, 6);
	vpColVector LTCIR(6);

	// compute the error vector
	vpColVector error(nbrow);
	int nerror = error.getRows();
	vpColVector v;

	double limite = 3; //Une limite de 3 pixels
	limite = limite / cam.get_px(); //Transformation limite pixel en limite metre.

	//Parametre pour la premiere phase d'asservissement
	bool reloop = true;
	double count = 0;

	/*** First phase ***/

	while (reloop == true && iter < 1) {
		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 1;
			factor.resize(nerror);
			factor = 1;
		}
		//        double t0 = vpTime::measureTimeMs();

		count = 0;
		//reloop = false;
		reloop = true;
		/*
 #pragma omp parallel
		{
			int local_count = 0;
   #pragma omp for nowait
			for (int k = 0; k < points[scaleLevel].size(); ++k) {
				apControlPoint *p = (points[scaleLevel])[k];
				p->computeInteractionMatrixErrorMH(cMo, _I);

				const double fac = 1;

				if (iter == 0 && p != NULL)
					for (int j = 0; j < 6; ++j)
						L[k][j] = p->L[j];  //On remplit la matrice d'interaction globale
				error[k] = p->error; //On remplit la matrice d'erreur

				if (error[k] <= limite)
					local_count = local_count + 1; //Si erreur proche de 0 on incremente cur

				w[k] = 1;

				if (iter == 0) {
					factor[k] = fac;
					const vpPointSite &site = p->s;
					//if (site.suppress != 0) factor[n] = 0;
					if (site.suppress != 0)
						factor[k] = 0.2;
				}
			}
			if (local_count != 0.0)
#pragma omp critical
			{
				count += local_count;
			}
		}

		count = count / (double) nbrow;
		if (count < 0.85) {
			reloop = true;
		}


		double num = 0;
		double den = 0;

		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			eri = error[i];
			num += wi * vpMath::sqr(eri);
			den += wi;

			weighted_error[i] = wi * eri;
		}

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					L[i][j] = w[i] * factor[i] * L[i][j];
				}
			}
		}

		//        std::cout << "t-2 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		CCDTracker.updateCCDPoints(cMo);
		CCDTracker.computeLocalStatistics();
		//        std::cout << "t-1 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		CCDTracker.updateParameters(LTCIL, LTCIR);
		//        double t1 = vpTime::measureTimeMs();
		//        std::cout << "t0 = " << vpTime::measureTimeMs() - t0 << std::endl;

		if (iter > 0)
			CCDTracker.checkCCDConvergence();

		//		LTL = L.AtA();
		//        t0 = vpTime::measureTimeMs();
		L.AtA(LTL);
		computeJTR(L, weighted_error, LTR);
		//        std::cout << "t1 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		v = -0.7 * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows()
				* DBL_EPSILON) * (LTR - weight_ccd * LTCIR);
		//cMo = vpExponentialMap::direct(v).inverse() * cMo;
		//        std::cout << "t2 = " << vpTime::measureTimeMs() - t0 << std::endl;

*/

		iter++;
	}
	/*std::cout << "\t First minimization in " << iter << " iteration "
	 << std::endl;*/

	/*** Second phase ***/
	vpColVector W_true;
	vpMatrix L_true;
	vpRobust robust(nerror);
	robust.setIteration(0);

	vpRobust robustCCD(CCDTracker.nerror_ccd);
	robustCCD.setIteration(0);
	robustCCD.setThreshold(2 / cam.get_px());

	//CCDTracker.initRobust();

	iter = 0;
	//vpColVector error_px(nerror);

        while (((int) ((residu_1 - r) * 1e8) != 0) && (iter < 12)) {
		//        double t0 = vpTime::measureTimeMs();
#pragma omp parallel for
		for (int k = 0; k < points[scaleLevel].size(); k++) {
			const int n = k;
			apControlPoint *p = (points[scaleLevel])[k];
			p->computeInteractionMatrixErrorMH(cMo, _I);
			for (int j = 0; j < 6; j++) {
				L[n][j] = p->L[j];
				error[n] = p->error;
			}
		}
		//        std::cout << "t0 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();

		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 1;

			robust.setThreshold(2 / cam.get_px()); // limite en metre
			robust.MEstimator(vpRobust::TUKEY, error, w);
		} else {
			robust.setIteration(iter);
			robust.MEstimator(vpRobust::TUKEY, error, w);
		}

		residu_1 = r;

		L_true = L;
		W_true = vpColVector(nerror);

		double num = 0;
		double den = 0;
		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			eri = error[i];
			W_true[i] = wi * wi;
			num += wi * vpMath::sqr(eri);
			den += wi;

			weighted_error[i] = wi * eri;
		}

		r = sqrt(num / den); //Le critere d'arret prend en compte le poids

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					L[i][j] = w[i] * factor[i] * L[i][j];
				}
			}
		}

		//        std::cout << "t1 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		robustCCD.setIteration(iter);
		CCDTracker.updateCCDPoints(cMo);
		//        std::cout << "t2a = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		CCDTracker.computeLocalStatistics();
		//        std::cout << "t2b = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		//		double t0 = vpTime::measureTimeMs();
		//CCDTracker.updateParametersRobust(LTCIL, LTCIR, robustCCD);
		CCDTracker.updateParameters(LTCIL,LTCIR);
		//		double t1 = vpTime::measureTimeMs();
		//std::cout << " timeupdate " << t1 -t0 << std::endl;
		if (iter > 0)
			CCDTracker.checkCCDConvergence();
		//        std::cout << "t2c = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();

		//		LTL = L.AtA();
		L.AtA(LTL);
		computeJTR(L, weighted_error, LTR);

		double stdME = sqrt((double)(weighted_error.t()*weighted_error)/(weighted_error.size()));

		double wghtME = ((double)1/weighted_error.size())*(1/stdME);
		double wghtCCD = ((double)1/CCDTracker.error_ccd.size());

		wghtME = 0.7;

		v = -lambda * (weight_me* wghtME*LTL + weight_ccd* wghtCCD * LTCIL).pseudoInverse(LTL.getRows()* DBL_EPSILON) * (weight_me* wghtME*LTR - weight_ccd * wghtCCD * LTCIR);

		//v = -lambda * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows() * DBL_EPSILON) * (LTR - weight_ccd * LTCIR);
		cMo = vpExponentialMap::direct(v).inverse() * cMo;
                        //std::cout << "t3 = " << v << std::endl;

		iter++;
	}

	if (computeCovariance) {
		vpMatrix D;
		D.diag(W_true);
		covarianceMatrixME = computeCovarianceMatrix(L_true, -v, lambda * error,
				D);


		covarianceMatrixCCD = CCDTracker.sigmaP;

		vpMatrix inv = (weight_me*(covarianceMatrixME.pseudoInverse()) + weight_ccd*(covarianceMatrixCCD.pseudoInverse()) );
                covarianceMatrix = inv.pseudoInverse();
	}

	//   cout << "\t Robust minimization in " << iter << " iteration " << endl ;
	//    std::cout << "error: " << (residu_1 - r) << std::endl;
}

void apMbTracker::computeVVSCCDKltMHPrev(const vpImage<unsigned char>& _I,
		const vpImage<vpRGBa>& _IRGB) {
	double residu_1 = 1e3;
	double r = 1e3 - 1;
	vpMatrix LTL;
	vpColVector LTR;

	vpMatrix LTL_klt;
	vpColVector LTR_klt;

	// compute the interaction matrix and its pseudo inverse

	vpColVector w;
	vpColVector weighted_error;
	vpColVector factor;

	vpColVector w_klt;
	vpColVector weighted_error_klt;
	vpColVector factor_klt;

	CCDTracker.init(CCDParameters, cam);
	CCDTracker.setImage(_IRGB);

	CCDTracker.setPrevImage(IprecRGB);

	CCDTracker.updateCCDPoints(cMo);
	CCDTracker.computeLocalStatisticsPrev(_I);

	covarianceMatrix.resize(6,6);
	covarianceMatrixME.resize(6,6);
	covarianceMatrixCCD.resize(6,6);
	covarianceMatrixP.resize(6,6);
	covarianceMatrixL.resize(6,6);
	covarianceMatrixKLT.resize(6,6);

	vpHomography H;
	unsigned int iter = 0;
	//Nombre de moving edges
	int nbrow = 0;

	int kk=0;

	std::map<int, apKltControlPoint>::const_iterator iklt = kltPoints[scaleLevel].begin();
	for( ; iklt != kltPoints[scaleLevel].end(); iklt++){
	kk++;
	}

	int nbrow_klt =  kk;


//#pragma omp parallel for
	for (int k = 0; k < points[scaleLevel].size(); k++) {
		apControlPoint *p = (points[scaleLevel])[k];
		p->initInteractionMatrixError();
	}
	nbrow = points[scaleLevel].size();

	if (nbrow == 0) {
		vpERROR_TRACE(
				"\n\t\t Error-> not enough data in the interaction matrix...");
		throw vpTrackingException(vpTrackingException::notEnoughPointError,
				"\n\t\t Error-> not enough data in the interaction matrix...");
	}

	vpMatrix L(nbrow, 6);
	vpMatrix L_klt(nbrow_klt*2, 6);

	vpMatrix LTCIL(6, 6);
	vpColVector LTCIR(6);

	// compute the error vector
	vpColVector error(nbrow);
	int nerror = error.getRows();

	vpColVector error_klt(2*nbrow_klt);
	int nerror_klt = error_klt.getRows();

	vpColVector v;

	double limite = 3; //Une limite de 3 pixels
	limite = limite / cam.get_px(); //Transformation limite pixel en limite metre.

	apKltControlPoint pklt;

	//Parametre pour la premiere phase d'asservissement
	bool reloop = true;
	double count = 0;

	/*** First phase ***/

	while (reloop == true && iter < 1) {
		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 1;
			factor.resize(nerror);
			factor = 1;

			weighted_error_klt.resize(nerror_klt);
			w_klt.resize(nerror_klt);
			w_klt = 1;
			factor_klt.resize(nerror_klt);
			factor_klt = 1;
		}
		//        double t0 = vpTime::measureTimeMs();

		count = 0;
		//reloop = false;
		reloop = true;
/*
#pragma omp parallel
		{
			int local_count = 0;
#pragma omp for nowait
			for (int k = 0; k < points[scaleLevel].size(); ++k) {
				apControlPoint *p = (points[scaleLevel])[k];
				p->computeInteractionMatrixErrorMH(cMo, _I);
				//p->computeInteractionMatrixError(cMo, _I);

				const double fac = 1;

				if (iter == 0 && p != NULL)
					for (int j = 0; j < 6; ++j)
						L[k][j] = p->L[j]; //On remplit la matrice d'interaction globale
				error[k] = p->error; //On remplit la matrice d'erreur

				if (error[k] <= limite)
					local_count = local_count + 1; //Si erreur proche de 0 on incremente cur

				w[k] = 1;

				if (iter == 0) {
					factor[k] = fac;
					const vpPointSite &site = p->s;
					//if (site.suppress != 0) factor[n] = 0;
					if (site.suppress != 0)
						factor[k] = 0.2;
				}
			}
			if (local_count != 0.0)
#pragma omp critical
			{
				count += local_count;
			}
		}

		count = count / (double) nbrow;
		if (count < 0.85) {
			reloop = true;
		}



	int kk=0;

		std::map<int, apKltControlPoint>::const_iterator iklt = kltPoints[scaleLevel].begin();
		for( ; iklt != kltPoints[scaleLevel].end(); iklt++){
			//int id(iklt->first);
			pklt = iklt->second;
			     //pklt.computeHomography(ctTc0,H);
			     //pklt.computeInteractionMatrixErrorH(_I);
			     pklt.computeInteractionMatrixError(cMo,_I);
				 const double fac_klt = 1;


				 //if (iter == 0 )
						for (int j = 0; j < 6; ++j)
						{
							L_klt[2*kk][j] = pklt.L[0][j]; //On remplit la matrice d'interaction globale
							L_klt[2*kk+1][j] = pklt.L[1][j]; //On remplit la matrice d'interaction globale
						}
					error_klt[2*kk] = pklt.error[0]; //On remplit la matrice d'erreur
					error_klt[2*kk+1] = pklt.error[1]; //On remplit la matrice d'erreur
					//std::cout << " error " << error_klt[2*kk] << " error 1 " << pklt.L[0][0] << std::endl;

					//std::cout << " ok " << error_klt[2*kk] << std::endl;

				//if (error_klt[kk] <= limite)
						//local_count = local_count + 1; //Si erreur proche de 0 on incremente cur

					w_klt[kk] = 1;
					kk++;


		}

		double num = 0;
		double den = 0;

		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			eri = error[i];
			num += wi * vpMath::sqr(eri);
			den += wi;

			weighted_error[i] = wi * eri;
		}

		double num_klt = 0;
		double den_klt = 0;

		double wi_klt;
		double eri_klt;
		for (int i = 0; i < nerror_klt; i++) {
			wi_klt = w_klt[i] * factor_klt[i];
			eri_klt = error_klt[i];
			num_klt += wi_klt * vpMath::sqr(eri_klt);
			den_klt += wi_klt;

			//weighted_error_klt[i] = wi_klt * eri_klt;
			weighted_error_klt[i] = eri_klt;
		}

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					L[i][j] = w[i] * factor[i] * L[i][j];
				}
			}
		}

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror_klt; i++) {
				for (int j = 0; j < 6; j++) {
					//L_klt[i][j] = w[i] * factor[i] * L_klt[i][j];
					L_klt[i][j] = L_klt[i][j];
				}
			}
		}

		//        std::cout << "t-2 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		CCDTracker.updateCCDPoints(cMo);
		CCDTracker.computeLocalStatistics();
		//        std::cout << "t-1 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		CCDTracker.updateParametersPrev(LTCIL, LTCIR);
		//        double t1 = vpTime::measureTimeMs();
		//        std::cout << "t0 = " << vpTime::measureTimeMs() - t0 << std::endl;

		if (iter > 0)
			CCDTracker.checkCCDConvergence();

		//		LTL = L.AtA();
		//        t0 = vpTime::measureTimeMs();
		L.AtA(LTL);
		computeJTR(L, weighted_error, LTR);
		//std::cout << " Lklt " << L_klt << std::endl;

		//std::cout << " error_klt " << error_klt << std::endl;

		L_klt.AtA(LTL_klt);
		computeJTR(L_klt, weighted_error_klt, LTR_klt);
		//        std::cout << "t1 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();


		std::cout << " frame " << frame << std::endl;
		if(frame0 <= 1)
		{
			v = -0.7 * (LTL).pseudoInverse(LTL.getRows()
							* DBL_EPSILON) * (LTR);
		}
		else {
			v = -0.7 * (LTL + weight_klt * LTL_klt).pseudoInverse(LTL.getRows()
							* DBL_EPSILON) * (LTR + weight_klt * LTR_klt);
		//v = -0.7 * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows()
			//	* DBL_EPSILON) * (LTR - weight_ccd * LTCIR);
		}
		//cMo = vpExponentialMap::direct(v).inverse() * cMo;
		ctTc0 = vpExponentialMap::direct(v).inverse()*ctTc0;
		//        std::cout << "t2 = " << vpTime::measureTimeMs() - t0 << std::endl;
		std::cout << "t1 = " << v << std::endl;

		std::cout << "ctTc0 " << ctTc0 << std::endl;

*/

		iter++;
	}
	/*std::cout << "\t First minimization in " << iter << " iteration "
	 << std::endl;*/

	/*** Second phase ***/
	vpColVector W_true;
	vpMatrix L_true;
	vpRobust robust(nerror);
	robust.setIteration(0);

	vpColVector WKLT_true;
	vpMatrix LKLT_true;
	vpRobust robust_klt(nerror_klt);
	robust_klt.setIteration(0);

	vpRobust robustCCD(CCDTracker.resolution);
	robustCCD.setIteration(0);
	robustCCD.setThreshold(2 / cam.get_px());

	//CCDTracker.initRobust();

	iter = 0;
	//vpColVector error_px(nerror);

	std::cout << "size klt " <<kltPoints[scaleLevel].size() << std::endl;

	while (/*((int) ((residu_1 - r) * 1e8) != 0) &&*/ (iter < 12)) {
		//        double t0 = vpTime::measureTimeMs();
//#pragma omp parallel for
		for (int k = 0; k < points[scaleLevel].size(); k++) {
			const int n = k;
			apControlPoint *p = (points[scaleLevel])[k];
			p->computeInteractionMatrixErrorMH(cMo, _I);
			//p->computeInteractionMatrixError(cMo, _I);
			for (int j = 0; j < 6; j++) {
				L[n][j] = p->L[j];
			}
				error[n] = p->error;
		}
		//        std::cout << "t0 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();

		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 1;

			robust.setThreshold(2 / cam.get_px()); // limite en metre
			robust.MEstimator(vpRobust::TUKEY, error, w);
		} else {
			robust.setIteration(iter);
			robust.MEstimator(vpRobust::TUKEY, error, w);
		}

		residu_1 = r;
		L_true = L;
		W_true = vpColVector(nerror);


		int kk=0;
		std::map<int, apKltControlPoint>::const_iterator iklt = kltPoints[scaleLevel].begin();
		//int nn = 0;
		for( ; iklt != kltPoints[scaleLevel].end(); iklt++){
			//int id(iklt->first);
			     pklt = iklt->second;
			     //pklt.computeHomography(ctTc0,H);
			     //pklt.computeInteractionMatrixErrorH(_I);
			     pklt.computeInteractionMatrixError(cMo,_I);
				 const double fac_klt = 1;

				 //if (iter == 0 )
						for (int j = 0; j < 6; ++j)
						{
							L_klt[2*kk][j] = pklt.L[0][j]; //On remplit la matrice d'interaction globale
							L_klt[2*kk+1][j] = pklt.L[1][j]; //On remplit la matrice d'interaction globale
						}
					error_klt[2*kk] = pklt.error[0]; //On remplit la matrice d'erreur
					error_klt[2*kk+1] = pklt.error[1]; //On remplit la matrice d'erreur

					//if (error_klt[2*kk] == 0)
						//nn++;
					kk++;
		}

		//std::cout << " nb klt points valid " << nn << std::endl;

		if (iter == 0) {
			weighted_error_klt.resize(nerror_klt);
			w_klt.resize(nerror_klt);
			w_klt = 1;

			robust_klt.setThreshold(2 / cam.get_px()); // limite en metre
			robust_klt.MEstimator(vpRobust::TUKEY, error_klt, w_klt);
		} else {
			robust_klt.setIteration(iter);
			robust_klt.MEstimator(vpRobust::TUKEY, error_klt, w_klt);
		}

		LKLT_true = L_klt;
		WKLT_true = vpColVector(nerror_klt);


		double num = 0;
		double den = 0;
		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			eri = error[i];
			W_true[i] = wi * wi;
			//num += wi * vpMath::sqr(eri);
			//den += wi;
			weighted_error[i] = wi * eri;
		}

		double num_klt = 0;
		double den_klt = 0;
		double wi_klt;
		double eri_klt;
		for (int ii = 0; ii < nerror_klt; ii++) {
			wi_klt = w_klt[ii] * factor_klt[ii];
			eri_klt = error_klt[ii];
			WKLT_true[ii] = wi_klt * wi_klt;
			//num_klt += wi_klt * vpMath::sqr(eri_klt);
			//den_klt += wi_klt;

			weighted_error_klt[ii] = wi_klt * eri_klt;
			//weighted_error_klt[ii] = eri_klt;
		}

		//r = sqrt(num / den); //Le critere d'arret prend en compte le poids

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					L[i][j] = w[i] * factor[i] * L[i][j];
				}
			}
		}

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror_klt; i++) {
				for (int j = 0; j < 6; j++) {
					L_klt[i][j] = w_klt[i] * factor_klt[i] * L_klt[i][j];

					//L_klt[i][j] =  L_klt[i][j];

				}
			}
		}

		//        std::cout << "t1 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		robustCCD.setIteration(iter);
		CCDTracker.updateCCDPoints(cMo);
		//        std::cout << "t2a = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		CCDTracker.computeLocalStatistics();

		//        std::cout << "t2b = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
				//double t0 = vpTime::measureTimeMs();
		//if(iter>5)
		//CCDTracker.updateParametersRobustPrev(LTCIL, LTCIR, robustCCD);
		//else CCDTracker.updateParametersPrev(LTCIL,LTCIR);

	    //CCDTracker.updateParametersRobustPrev(LTCIL, LTCIR, robustCCD);

	    /*int up,vp;
	    for (int ii = 0; ii < CCDTracker.w_ccd.size(); ii ++)
	    {
	    	if(CCDTracker.w_ccd[ii] < 0.7)
	    	{
	    	    apControlPoint *p = CCDTracker.pointsCCD[scaleLevel][ii];
	    	    up = (int)p->icpoint->get_u();
	    	    vp = (int)p->icpoint->get_v();
	    	    vpDisplay::displayCross(_I,vp,up,5,vpColor::red,5);
	    	}
	    }*/

            //CCDTracker.updateParametersPrev(LTCIL,LTCIR);
                CCDTracker.updateParameters(LTCIL,LTCIR);
				//double t1 = vpTime::measureTimeMs();

		if (iter > 0)
			CCDTracker.checkCCDConvergence();
		//        std::cout << "t2c = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();

		//		LTL = L.AtA();
		L.AtA(LTL);
		computeJTR(L, weighted_error, LTR);
		L_klt.AtA(LTL_klt);
		computeJTR(L_klt, weighted_error_klt, LTR_klt);

		double stdME = sqrt((double)(weighted_error.t()*weighted_error)/(weighted_error.size()));
		double stdKLT = sqrt((double)(weighted_error_klt.t()*weighted_error_klt)/(weighted_error_klt.size()));
		double stdCCD = sqrt((double)(CCDTracker.error_ccd.t()*CCDTracker.error_ccd)/(CCDTracker.error_ccd.size()));
		double wghtME = ((double)1/weighted_error.size())*(1/stdME);
	    double wghtKLT = ((double)1/weighted_error_klt.size())*(1/stdKLT);
		double wghtCCD = ((double)1/(10*CCDTracker.error_ccd.size()));
		//double wghtCCD = ((double)1/(CCDTracker.error_ccd.size()))*(1/stdCCD);


		wghtME = 0.7;
		//wghtKLT = 30000;
		wghtKLT = 10;

		//wghtME = ((double)1/weighted_error.size())/(2/cam.get_px());
		//wghtKLT = ((double)1/weighted_error_klt.size())/(3/cam.get_px());

		std::cout << " wghtme " << wghtME << " wghtklt " << wghtKLT << " stdCCD " << stdCCD << std::endl;

		//std::cout << "t3 = " << vpTime::measureTimeMs() - t0 << std::endl;



		/*v = -lambda * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows()
				* DBL_EPSILON) * (LTR - weight_ccd * LTCIR );*/
		/*if(frame0 <= 1)
		{
		v = -lambda * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows()
					* DBL_EPSILON) * (LTR - weight_ccd * LTCIR);
		}
		else
		{
		v = -lambda * (LTL + weight_ccd * LTCIL + weight_klt * LTL_klt).pseudoInverse(LTL.getRows()
				* DBL_EPSILON) * (LTR - weight_ccd * LTCIR + weight_klt * LTR_klt);
		}*/

		//std::cout << "weight0 " << weight_me * wghtME * sqrt((LTR).t()*(LTR)) << " wght1 " << weight_ccd*wghtCCD*sqrt((LTCIR).t()*(LTCIR)) << " wghtklt " << weight_klt*wghtKLT*sqrt((LTR_klt).t()*(LTR_klt)) << std::endl;
		//std::cout << "weight0 " << weight_me * wghtME << " wght1 " << weight_ccd*wghtCCD << " wghtklt " << weight_klt * wghtKLT << std::endl;

		//std::cout << "weight0 " << wghtME << " wght1 " << weight_ccd*wghtCCD << " wghtklt " << wghtKLT << std::endl;

		if(frame0 <= 1)
				{
				v = -lambda * (weight_me * wghtME*LTL + weight_ccd * wghtCCD * LTCIL).pseudoInverse(LTL.getRows()
							* DBL_EPSILON) * (weight_me*wghtME*LTR - weight_ccd * wghtCCD *  LTCIR);
				}
				else
				{
				v = -lambda * (weight_me*wghtME*LTL + weight_ccd * wghtCCD * LTCIL + weight_klt * wghtKLT * LTL_klt).pseudoInverse(LTL.getRows()
						* DBL_EPSILON) * (weight_me * wghtME*LTR - weight_ccd * wghtCCD * LTCIR + weight_klt * wghtKLT * LTR_klt);
				}



		/*v = -lambda * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows()
				* DBL_EPSILON) * (LTR - weight_ccd * LTCIR );*/
		/*if(frame0 <= 1)
		{
		v = -lambda * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows()
					* DBL_EPSILON) * (LTR - weight_ccd * LTCIR);
		}
		else
		{
		v = -lambda * (LTL + weight_ccd * LTCIL + weight_klt * LTL_klt).pseudoInverse(LTL.getRows()
				* DBL_EPSILON) * (LTR - weight_ccd * LTCIR + weight_klt * LTR_klt);
		}*/

		cMo = vpExponentialMap::direct(v).inverse() * cMo;
		ctTc0 = vpExponentialMap::direct(v).inverse()*ctTc0;
		//        std::cout << "t3 = " << vpTime::measureTimeMs() - t0 << std::endl;

		if (computeCovariance) {
			vpMatrix D;
			D.diag(W_true);
			//covarianceMatrixME =  0.25*covarianceMatrixME + 0.75*2*computeCovarianceMatrix(L_true, -v, lambda*error,D);

			vpMatrix DKLT;
			DKLT.diag(WKLT_true);
			//covarianceMatrixKLT = 0.25*covarianceMatrixME + 0.75*2*computeCovarianceMatrix(LKLT_true, -v, lambda*weighted_error_klt,DKLT);
			//covarianceMatrix = covarianceMatrixME;
		}

		iter++;
	}

	if (computeCovariance) {
		/*vpMatrix D;
		D.diag(W_true);
		covarianceMatrix = computeCovarianceMatrix(L_true, -v, lambda * error,
				D);*/

		vpMatrix D;
		D.diag(W_true);
		covarianceMatrixME = computeCovarianceMatrix(L_true, -v, lambda * error,D);


		vpMatrix DKLT;
		DKLT.diag(WKLT_true);
		covarianceMatrixKLT = computeCovarianceMatrix(LKLT_true, -v, lambda * weighted_error_klt,DKLT);



		/*std::cout << " lktl " <<covarianceMatrixKLT[0][0]+ covarianceMatrixKLT[1][1]+ covarianceMatrixKLT[2][2]<< std::endl;

		if (covarianceMatrixKLT[0][0]+ covarianceMatrixKLT[1][1]+ covarianceMatrixKLT[2][2]> 0.001)
		{
			std::cout << " lktl " << LKLT_true << " sum " << weighted_error_klt.sumSquare()<< std::endl;
			vpDisplay::getClick(_I);
		}*/

		//covarianceMatrixME =  2*computeCovarianceMatrix(L_true, -v, points[scaleLevel].size()*lambda * weighted_error,D);


		//std::cout << "covMatEdge " << covarianceMatrix[0][0] << std::endl;
		covarianceMatrixCCD = lambda*lambda*CCDTracker.sigmaP;
		//covarianceMatrix = (0*1+weight_ccd*1)*(weight_ccd**CCDTracker.sigmaP+L_true.t()*L_true).pseudoInverse();
		//std::cout << "covMatCCD0 " << CCDTracker.sigmaF[0][0] << std::endl;
		//std::cout << "covMatCCD1 " << CCDTracker.sigmaP[0][0] + CCDTracker.sigmaP[1][1] + CCDTracker.sigmaP[2][2] << " covMatCCD2 " << CCDTracker.sigmaP[3][3] + CCDTracker.sigmaP[4][4] + CCDTracker.sigmaP[5][5] << std::endl;

		vpMatrix inv = ((double)weight_me)*(covarianceMatrixME.pseudoInverse(DBL_EPSILON)) + ((double)weight_ccd)*(covarianceMatrixCCD.pseudoInverse(DBL_EPSILON)) + ((double)weight_klt)*covarianceMatrixKLT.pseudoInverse( DBL_EPSILON);
		covarianceMatrix = (weight_me+weight_ccd+weight_klt)*inv.pseudoInverse(DBL_EPSILON);

		//vpMatrix inv = ((double)weight_me/(error.size())*(covarianceMatrixME.pseudoInverse(DBL_EPSILON)) + ((double)weight_ccd/(CCDTracker.error_ccd.size()))*(covarianceMatrixCCD.pseudoInverse(DBL_EPSILON)) + ((double)weight_klt/weighted_error_klt.size())*covarianceMatrixKLT.pseudoInverse(DBL_EPSILON));
		//covarianceMatrix = ((double)weight_me/error.size() + (double)weight_ccd/(CCDTracker.error_ccd.size()) + (double)weight_klt/weighted_error_klt.size())*inv.pseudoInverse(DBL_EPSILON);
		//covarianceMatrix = (weight_me*(covarianceMatrixME) + weight_ccd*(covarianceMatrixCCD) + weight_klt*covarianceMatrixKLT);


	}
	//   cout << "\t Robust minimization in " << iter << " iteration " << endl ;
	//    std::cout << "error: " << (residu_1 - r) << std::endl;
}


void apMbTracker::computeVVSCCDKltMHPrevFAST(const vpImage<unsigned char>& _I,
		const vpImage<vpRGBa>& _IRGB) {
	double residu_1 = 1e3;
	double r = 1e3 - 1;
	vpMatrix LTL;
	vpColVector LTR;

	vpMatrix LTL_klt;
	vpColVector LTR_klt;

	// compute the interaction matrix and its pseudo inverse

	vpColVector w;
	vpColVector weighted_error;
	vpColVector factor;

	vpColVector w_klt;
	vpColVector weighted_error_klt;
	vpColVector factor_klt;

	CCDTracker.init(CCDParameters, cam);
	CCDTracker.setImage(_IRGB);

	CCDTracker.setPrevImage(IprecRGB);

	CCDTracker.updateCCDPoints(cMo);
	CCDTracker.computeLocalStatisticsPrev(_I);


	covarianceMatrix.resize(6,6);
	covarianceMatrixME.resize(6,6);
	covarianceMatrixCCD.resize(6,6);
	covarianceMatrixP.resize(6,6);
	covarianceMatrixL.resize(6,6);
	covarianceMatrixKLT.resize(6,6);

	vpHomography H;
	unsigned int iter = 0;
	//Nombre de moving edges
	int nbrow = 0;

	int kk=0;

	std::map<int, apKltControlPoint>::const_iterator iklt = kltPoints[scaleLevel].begin();
	for( ; iklt != kltPoints[scaleLevel].end(); iklt++){
	kk++;
	}

	int nbrow_klt =  kk;


#pragma omp parallel for
	for (int k = 0; k < points[scaleLevel].size(); k++) {
		apControlPoint *p = (points[scaleLevel])[k];
		p->initInteractionMatrixError();
	}
	nbrow = points[scaleLevel].size();

	if (nbrow == 0) {
		vpERROR_TRACE(
				"\n\t\t Error-> not enough data in the interaction matrix...");
		throw vpTrackingException(vpTrackingException::notEnoughPointError,
				"\n\t\t Error-> not enough data in the interaction matrix...");
	}

	vpMatrix L(nbrow, 6);
	vpMatrix L_klt(nbrow_klt*2, 6);

	vpMatrix LTCIL(6, 6);
	vpColVector LTCIR(6);

	// compute the error vector
	vpColVector error(nbrow);
	int nerror = error.getRows();

	vpColVector error_klt(2*nbrow_klt);
	int nerror_klt = error_klt.getRows();

	vpColVector v;

	double limite = 3; //Une limite de 3 pixels
	limite = limite / cam.get_px(); //Transformation limite pixel en limite metre.

	apKltControlPoint pklt;

	//Parametre pour la premiere phase d'asservissement
	bool reloop = true;
	double count = 0;

	/*** First phase ***/

	while (reloop == true && iter < 1) {
		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 1;
			factor.resize(nerror);
			factor = 1;

			weighted_error_klt.resize(nerror_klt);
			w_klt.resize(nerror_klt);
			w_klt = 1;
			factor_klt.resize(nerror_klt);
			factor_klt = 1;
		}
		//        double t0 = vpTime::measureTimeMs();

		count = 0;
		//reloop = false;
		reloop = true;
/*
#pragma omp parallel
		{
			int local_count = 0;
#pragma omp for nowait
			for (int k = 0; k < points[scaleLevel].size(); ++k) {
				apControlPoint *p = (points[scaleLevel])[k];
				p->computeInteractionMatrixErrorMH(cMo, _I);
				//p->computeInteractionMatrixError(cMo, _I);

				const double fac = 1;

				if (iter == 0 && p != NULL)
					for (int j = 0; j < 6; ++j)
						L[k][j] = p->L[j]; //On remplit la matrice d'interaction globale
				error[k] = p->error; //On remplit la matrice d'erreur

				if (error[k] <= limite)
					local_count = local_count + 1; //Si erreur proche de 0 on incremente cur

				w[k] = 1;

				if (iter == 0) {
					factor[k] = fac;
					const vpPointSite &site = p->s;
					//if (site.suppress != 0) factor[n] = 0;
					if (site.suppress != 0)
						factor[k] = 0.2;
				}
			}
			if (local_count != 0.0)
#pragma omp critical
			{
				count += local_count;
			}
		}

		count = count / (double) nbrow;
		if (count < 0.85) {
			reloop = true;
		}



	int kk=0;

		std::map<int, apKltControlPoint>::const_iterator iklt = kltPoints[scaleLevel].begin();
		for( ; iklt != kltPoints[scaleLevel].end(); iklt++){
			//int id(iklt->first);
			pklt = iklt->second;
			     //pklt.computeHomography(ctTc0,H);
			     //pklt.computeInteractionMatrixErrorH(_I);
			     pklt.computeInteractionMatrixError(cMo,_I);
				 const double fac_klt = 1;


				 //if (iter == 0 )
						for (int j = 0; j < 6; ++j)
						{
							L_klt[2*kk][j] = pklt.L[0][j]; //On remplit la matrice d'interaction globale
							L_klt[2*kk+1][j] = pklt.L[1][j]; //On remplit la matrice d'interaction globale
						}
					error_klt[2*kk] = pklt.error[0]; //On remplit la matrice d'erreur
					error_klt[2*kk+1] = pklt.error[1]; //On remplit la matrice d'erreur
					//std::cout << " error " << error_klt[2*kk] << " error 1 " << pklt.L[0][0] << std::endl;

					//std::cout << " ok " << error_klt[2*kk] << std::endl;

				//if (error_klt[kk] <= limite)
						//local_count = local_count + 1; //Si erreur proche de 0 on incremente cur

					w_klt[kk] = 1;
					kk++;


		}

		double num = 0;
		double den = 0;

		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			eri = error[i];
			num += wi * vpMath::sqr(eri);
			den += wi;

			weighted_error[i] = wi * eri;
		}

		double num_klt = 0;
		double den_klt = 0;

		double wi_klt;
		double eri_klt;
		for (int i = 0; i < nerror_klt; i++) {
			wi_klt = w_klt[i] * factor_klt[i];
			eri_klt = error_klt[i];
			num_klt += wi_klt * vpMath::sqr(eri_klt);
			den_klt += wi_klt;

			//weighted_error_klt[i] = wi_klt * eri_klt;
			weighted_error_klt[i] = eri_klt;
		}

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					L[i][j] = w[i] * factor[i] * L[i][j];
				}
			}
		}

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror_klt; i++) {
				for (int j = 0; j < 6; j++) {
					//L_klt[i][j] = w[i] * factor[i] * L_klt[i][j];
					L_klt[i][j] = L_klt[i][j];
				}
			}
		}

		//        std::cout << "t-2 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		CCDTracker.updateCCDPoints(cMo);
		CCDTracker.computeLocalStatistics();
		//        std::cout << "t-1 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		CCDTracker.updateParametersPrev(LTCIL, LTCIR);
		//        double t1 = vpTime::measureTimeMs();
		//        std::cout << "t0 = " << vpTime::measureTimeMs() - t0 << std::endl;

		if (iter > 0)
			CCDTracker.checkCCDConvergence();

		//		LTL = L.AtA();
		//        t0 = vpTime::measureTimeMs();
		L.AtA(LTL);
		computeJTR(L, weighted_error, LTR);
		//std::cout << " Lklt " << L_klt << std::endl;

		//std::cout << " error_klt " << error_klt << std::endl;

		L_klt.AtA(LTL_klt);
		computeJTR(L_klt, weighted_error_klt, LTR_klt);
		//        std::cout << "t1 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();


		std::cout << " frame " << frame << std::endl;
		if(frame0 <= 1)
		{
			v = -0.7 * (LTL).pseudoInverse(LTL.getRows()
							* DBL_EPSILON) * (LTR);
		}
		else {
			v = -0.7 * (LTL + weight_klt * LTL_klt).pseudoInverse(LTL.getRows()
							* DBL_EPSILON) * (LTR + weight_klt * LTR_klt);
		//v = -0.7 * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows()
			//	* DBL_EPSILON) * (LTR - weight_ccd * LTCIR);
		}
		//cMo = vpExponentialMap::direct(v).inverse() * cMo;
		ctTc0 = vpExponentialMap::direct(v).inverse()*ctTc0;
		//        std::cout << "t2 = " << vpTime::measureTimeMs() - t0 << std::endl;
		std::cout << "t1 = " << v << std::endl;

		std::cout << "ctTc0 " << ctTc0 << std::endl;

*/

		iter++;
	}
	/*std::cout << "\t First minimization in " << iter << " iteration "
	 << std::endl;*/

	/*** Second phase ***/
	vpColVector W_true;
	vpMatrix L_true;
	vpRobust robust(nerror);
	robust.setIteration(0);

	vpColVector WKLT_true;
	vpMatrix LKLT_true;
	vpRobust robust_klt(nerror_klt);
	robust_klt.setIteration(0);

	vpRobust robustCCD(CCDTracker.resolution);
	robustCCD.setIteration(0);
	robustCCD.setThreshold(2 / cam.get_px());

	//CCDTracker.initRobust();

	iter = 0;
	//vpColVector error_px(nerror);

	std::cout << "size klt " <<kltPoints[scaleLevel].size() << std::endl;

	while ((iter < 12)) {
		//        double t0 = vpTime::measureTimeMs();
#pragma omp parallel for
		for (int k = 0; k < points[scaleLevel].size(); k++) {
			const int n = k;
			apControlPoint *p = (points[scaleLevel])[k];
			p->computeInteractionMatrixErrorMH(cMo, _I);
			//p->computeInteractionMatrixError(cMo, _I);
			for (int j = 0; j < 6; j++) {
				L[n][j] = p->L[j];
			}
				error[n] = p->error;
		}
		//        std::cout << "t0 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();

		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 1;

			robust.setThreshold(2 / cam.get_px()); // limite en metre
			robust.MEstimator(vpRobust::TUKEY, error, w);
		} else {
			robust.setIteration(iter);
			robust.MEstimator(vpRobust::TUKEY, error, w);
		}

		residu_1 = r;
		L_true = L;
		W_true = vpColVector(nerror);


		int kk=0;
		std::map<int, apKltControlPoint>::const_iterator iklt = kltPoints[scaleLevel].begin();
		//int nn = 0;
		for( ; iklt != kltPoints[scaleLevel].end(); iklt++){
			//int id(iklt->first);
			     pklt = iklt->second;
			     //pklt.computeHomography(ctTc0,H);
			     //pklt.computeInteractionMatrixErrorH(_I);
			     pklt.computeInteractionMatrixError(cMo,_I);
				 const double fac_klt = 1;

				 //if (iter == 0 )
						for (int j = 0; j < 6; ++j)
						{
							L_klt[2*kk][j] = pklt.L[0][j]; //On remplit la matrice d'interaction globale
							L_klt[2*kk+1][j] = pklt.L[1][j]; //On remplit la matrice d'interaction globale
						}
					error_klt[2*kk] = pklt.error[0]; //On remplit la matrice d'erreur
					error_klt[2*kk+1] = pklt.error[1]; //On remplit la matrice d'erreur

					//if (error_klt[2*kk] == 0)
						//nn++;
					kk++;
		}

		//std::cout << " nb klt points valid " << nn << std::endl;

		if (iter == 0) {
			weighted_error_klt.resize(nerror_klt);
			w_klt.resize(nerror_klt);
			w_klt = 1;

			robust_klt.setThreshold(2 / cam.get_px()); // limite en metre
			robust_klt.MEstimator(vpRobust::TUKEY, error_klt, w_klt);
		} else {
			robust_klt.setIteration(iter);
			robust_klt.MEstimator(vpRobust::TUKEY, error_klt, w_klt);
		}

		LKLT_true = L_klt;
		WKLT_true = vpColVector(nerror_klt);


		//double num = 0;
		//double den = 0;
		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			eri = error[i];
			W_true[i] = wi * wi;
			//num += wi * vpMath::sqr(eri);
			//den += wi;
			weighted_error[i] = wi * eri;
		}

		//double num_klt = 0;
		//double den_klt = 0;
		double wi_klt;
		double eri_klt;

#pragma omp parallel for
		for (int ii = 0; ii < nerror_klt; ii++) {
			wi_klt = w_klt[ii] * factor_klt[ii];
			eri_klt = error_klt[ii];
			WKLT_true[ii] = wi_klt * wi_klt;
			//num_klt += wi_klt * vpMath::sqr(eri_klt);
			//den_klt += wi_klt;

			weighted_error_klt[ii] = wi_klt * eri_klt;
			//weighted_error_klt[ii] = eri_klt;
		}

		//r = sqrt(num / den); //Le critere d'arret prend en compte le poids

		if ((iter == 0) || compute_interaction) {
#pragma omp parallel for
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					L[i][j] = w[i] * factor[i] * L[i][j];
				}
			}
		}

		if ((iter == 0) || compute_interaction) {
#pragma omp parallel for
			for (int i = 0; i < nerror_klt; i++) {
				for (int j = 0; j < 6; j++) {
					L_klt[i][j] = w_klt[i] * factor_klt[i] * L_klt[i][j];

					//L_klt[i][j] =  L_klt[i][j];

				}
			}
		}

		//        std::cout << "t1 = " << vpTime::measureTimeMs() - t0 << std::endl;
		       double t0 = vpTime::measureTimeMs();
		robustCCD.setIteration(iter);
		CCDTracker.updateCCDPoints(cMo);
		        //std::cout << "t2a = " << vpTime::measureTimeMs() - t0 << std::endl;
		        //t0 = vpTime::measureTimeMs();
		CCDTracker.computeLocalStatistics();

		        //std::cout << "t2b = " << vpTime::measureTimeMs() - t0 << std::endl;
		        //t0 = vpTime::measureTimeMs();
				//double t0 = vpTime::measureTimeMs();
		//if(iter>5)
		//CCDTracker.updateParametersRobustPrev(LTCIL, LTCIR, robustCCD);
		//else CCDTracker.updateParametersPrev(LTCIL,LTCIR);

	    //CCDTracker.updateParametersRobustPrev(LTCIL, LTCIR, robustCCD);

                //CCDTracker.updateParametersPrev(LTCIL,LTCIR);
                CCDTracker.updateParameters(LTCIL,LTCIR);

				//double t1 = vpTime::measureTimeMs();

		/*if (iter > 0)
			CCDTracker.checkCCDConvergence();*/
		        //std::cout << "t2c = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();

		//		LTL = L.AtA();
		L.AtA(LTL);
		computeJTR(L, weighted_error, LTR);
		L_klt.AtA(LTL_klt);
		computeJTR(L_klt, weighted_error_klt, LTR_klt);

		/*double stdME = sqrt((double)(weighted_error.t()*weighted_error)/(weighted_error.size()));
		double stdKLT = sqrt((double)(weighted_error_klt.t()*weighted_error_klt)/(weighted_error_klt.size()));
		double wghtME = ((double)1/weighted_error.size())*(1/stdME);
		double wghtKLT = ((double)1/weighted_error_klt.size())*(1/stdKLT);*/
		double wghtCCD = ((double)1/(10*CCDTracker.error_ccd.size()));


		double wghtME = 0.7;
		//wghtKLT = 30000;
		double wghtKLT = 10;

		//wghtME = ((double)1/weighted_error.size())/(2/cam.get_px());
		//wghtKLT = ((double)1/weighted_error_klt.size())/(3/cam.get_px());

		//std::cout << " wlktl " << wghtME << " wghtme " << wghtKLT << std::endl;

		//std::cout << "t3 = " << vpTime::measureTimeMs() - t0 << std::endl;



		/*v = -lambda * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows()
				* DBL_EPSILON) * (LTR - weight_ccd * LTCIR );*/
		/*if(frame0 <= 1)
		{
		v = -lambda * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows()
					* DBL_EPSILON) * (LTR - weight_ccd * LTCIR);
		}
		else
		{
		v = -lambda * (LTL + weight_ccd * LTCIL + weight_klt * LTL_klt).pseudoInverse(LTL.getRows()
				* DBL_EPSILON) * (LTR - weight_ccd * LTCIR + weight_klt * LTR_klt);
		}*/

		//std::cout << "weight0 " << weight_me * wghtME * sqrt((LTR).t()*(LTR)) << " wght1 " << weight_ccd*wghtCCD*sqrt((LTCIR).t()*(LTCIR)) << " wghtklt " << weight_klt*wghtKLT*sqrt((LTR_klt).t()*(LTR_klt)) << std::endl;
		//std::cout << "weight0 " << weight_me * wghtME << " wght1 " << weight_ccd*wghtCCD << " wghtklt " << weight_klt * wghtKLT << std::endl;

		//std::cout << "weight0 " << wghtME << " wght1 " << weight_ccd*wghtCCD << " wghtklt " << wghtKLT << std::endl;

		if(frame0 <= 1)
				{
				v = -lambda * (weight_me * wghtME*LTL + weight_ccd * wghtCCD * LTCIL).pseudoInverse(LTL.getRows()
							* DBL_EPSILON) * (weight_me*wghtME*LTR - weight_ccd * wghtCCD *  LTCIR);
				}
				else
				{
				v = -lambda * (weight_me*wghtME*LTL + weight_ccd * wghtCCD * LTCIL + weight_klt * wghtKLT * LTL_klt).pseudoInverse(LTL.getRows()
						* DBL_EPSILON) * (weight_me * wghtME*LTR - weight_ccd * wghtCCD * LTCIR + weight_klt * wghtKLT * LTR_klt);
				}



		/*v = -lambda * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows()
				* DBL_EPSILON) * (LTR - weight_ccd * LTCIR );*/
		/*if(frame0 <= 1)
		{
		v = -lambda * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows()
					* DBL_EPSILON) * (LTR - weight_ccd * LTCIR);
		}
		else
		{
		v = -lambda * (LTL + weight_ccd * LTCIL + weight_klt * LTL_klt).pseudoInverse(LTL.getRows()
				* DBL_EPSILON) * (LTR - weight_ccd * LTCIR + weight_klt * LTR_klt);
		}*/

		cMo = vpExponentialMap::direct(v).inverse() * cMo;
		ctTc0 = vpExponentialMap::direct(v).inverse() *ctTc0;
		//        std::cout << "t3 = " << vpTime::measureTimeMs() - t0 << std::endl;

		if (computeCovariance) {
			vpMatrix D;
			D.diag(W_true);
			//covarianceMatrixME =  0.25*covarianceMatrixME + 0.75*2*computeCovarianceMatrix(L_true, -v, lambda*error,D);

			vpMatrix DKLT;
			DKLT.diag(WKLT_true);
			//covarianceMatrixKLT = 0.25*covarianceMatrixME + 0.75*2*computeCovarianceMatrix(LKLT_true, -v, lambda*weighted_error_klt,DKLT);
			//covarianceMatrix = covarianceMatrixME;
		}

		iter++;
	}


	if (computeCovariance) {
		/*vpMatrix D;
		D.diag(W_true);
		covarianceMatrix = computeCovarianceMatrix(L_true, -v, lambda * error,
				D);*/

		vpMatrix D;
		D.diag(W_true);
		covarianceMatrixME = computeCovarianceMatrix(L_true, -v, lambda * error,D);


		vpMatrix DKLT;
		DKLT.diag(WKLT_true);
		covarianceMatrixKLT = computeCovarianceMatrix(LKLT_true, -v, lambda*weighted_error_klt,DKLT);

		/*std::cout << " lktl " <<covarianceMatrixKLT[0][0]+ covarianceMatrixKLT[1][1]+ covarianceMatrixKLT[2][2]<< std::endl;

		if (covarianceMatrixKLT[0][0]+ covarianceMatrixKLT[1][1]+ covarianceMatrixKLT[2][2]> 0.001)
		{
			std::cout << " lktl " << LKLT_true << " sum " << weighted_error_klt.sumSquare()<< std::endl;
			vpDisplay::getClick(_I);
		}*/

		//covarianceMatrixME =  2*computeCovarianceMatrix(L_true, -v, points[scaleLevel].size()*lambda * weighted_error,D);


		//std::cout << "covMatEdge " << covarianceMatrix[0][0] << std::endl;
		covarianceMatrixCCD = lambda*lambda*CCDTracker.sigmaP;
		//covarianceMatrix = (0*1+weight_ccd*1)*(weight_ccd**CCDTracker.sigmaP+L_true.t()*L_true).pseudoInverse();
		//std::cout << "covMatCCD0 " << CCDTracker.sigmaF[0][0] << std::endl;
		//std::cout << "covMatCCD1 " << CCDTracker.sigmaP[0][0] + CCDTracker.sigmaP[1][1] + CCDTracker.sigmaP[2][2] << " covMatCCD2 " << CCDTracker.sigmaP[3][3] + CCDTracker.sigmaP[4][4] + CCDTracker.sigmaP[5][5] << std::endl;


		vpMatrix inv = (weight_me*(covarianceMatrixME.pseudoInverse(DBL_EPSILON)) + weight_ccd*(covarianceMatrixCCD.pseudoInverse(DBL_EPSILON)) + weight_klt*covarianceMatrixKLT.pseudoInverse( DBL_EPSILON));
                //covarianceMatrix = inv.pseudoInverse(DBL_EPSILON);
		//covarianceMatrix = (weight_me*(covarianceMatrixME) + weight_ccd*(covarianceMatrixCCD) + weight_klt*covarianceMatrixKLT);
                covarianceMatrix = (weight_me+weight_ccd+weight_klt)*inv.pseudoInverse(DBL_EPSILON);

	}
	//   cout << "\t Robust minimization in " << iter << " iteration " << endl ;
	//    std::cout << "error: " << (residu_1 - r) << std::endl;
}


void apMbTracker::computeVVSLinesCCDKltMHPrev(const vpImage<unsigned char>& _I,
		const vpImage<vpRGBa>& _IRGB) {
	double residu_1 = 1e3;
	double r = 1e3 - 1;
	vpMatrix LTL;
	vpColVector LTR;

	vpMatrix LTL_klt;
	vpColVector LTR_klt;


	double t0 = vpTime::measureTimeMs();

	// compute the interaction matrix and its pseudo inverse
	apMbtDistanceLineMH *l, *l1, *l2;

	// compute the interaction matrix and its pseudo inverse
	vpColVector w;
	vpColVector weighted_error;
	vpColVector factor;

	vpColVector wL;
	vpColVector weighted_errorL;
	vpColVector factorL;

	vpColVector weighted_errorH;
	vpColVector weighted_errorH1;


	vpColVector w_klt;
	vpColVector weighted_error_klt;
	vpColVector factor_klt;

	CCDTracker.init(CCDParameters, cam);
	CCDTracker.setImage(_IRGB);
	CCDTracker.setPrevImage(IprecRGB);
	CCDTracker.updateCCDPoints(cMo);
	CCDTracker.computeLocalStatisticsPrev(_I);


	covarianceMatrix.resize(6,6);
	covarianceMatrixME.resize(6,6);
	covarianceMatrixCCD.resize(6,6);
	covarianceMatrixP.resize(6,6);
	covarianceMatrixL.resize(6,6);
	covarianceMatrixKLT.resize(6,6);

	vpHomography H;
	unsigned int iter = 0;
	//Nombre de moving edges
	int nbrow = 0;
	int nbrowL = 0;

	vpFeatureLine fli;

	lines[scaleLevel][0]->getCameraParameters(&cam);
	std::cout << " nb lines clusters " << lines[scaleLevel].size() << std::endl;
	for (int k = 0; k < lines[scaleLevel].size(); k++) {
		l = lines[scaleLevel][k];
		if (l->MHmeline->points_vect.size() > 0) {
			l->initInteractionMatrixErrorMH();
			nbrowL += l->nbFeature;
			//std::cout << " nb features " << l->nbFeature << std::endl;
		}
	}
	if (nbrowL == 0) {
		vpERROR_TRACE(
				"\n\t\t Error-> not enough data in the interaction matrix...");
		throw vpTrackingException(vpTrackingException::notEnoughPointError,
				"\n\t\t Error-> not enough data in the interaction matrix...");
	}

	int kk=0;

	std::map<int, apKltControlPoint>::const_iterator iklt = kltPoints[scaleLevel].begin();
	for( ; iklt != kltPoints[scaleLevel].end(); iklt++){
	kk++;
	}

	int nbrow_klt =  kk;


#pragma omp parallel for
	for (int k = 0; k < points[scaleLevel].size(); k++) {
		apControlPoint *p = (points[scaleLevel])[k];
		p->initInteractionMatrixError();
	}
	nbrow = points[scaleLevel].size();

	if (nbrow == 0) {
		vpERROR_TRACE(
				"\n\t\t Error-> not enough data in the interaction matrix...");
		throw vpTrackingException(vpTrackingException::notEnoughPointError,
				"\n\t\t Error-> not enough data in the interaction matrix...");
	}



	vpMatrix L(nbrow, 6), Lp;
	vpMatrix LL(nbrowL, 6), LpL;
	vpMatrix L_klt(nbrow_klt*2, 6);

	vpMatrix LH;
	vpMatrix LTCIL(6, 6);
	vpColVector LTCIR(6);

	// compute the error vector
	vpColVector error(nbrow);
	int nerror = error.getRows();

	vpColVector errorL(nbrowL);
	int nerrorL = errorL.getRows();

	vpColVector errorH;

	vpMatrix LH1;
	vpColVector errorH1;

	vpColVector error_klt(2*nbrow_klt);
	int nerror_klt = error_klt.getRows();

	vpColVector v;

	double limite = 3; //Une limite de 3 pixels
	limite = limite / cam.get_px(); //Transformation limite pixel en limite metre.

	apKltControlPoint pklt;

	//Parametre pour la premiere phase d'asservissement
	bool reloop = true;
	double count = 0;

	bool reloopL = true;
	double countL = 0;


	/*** First phase ***/

	while (reloop == true && iter < 1) {
		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 0;
			factor.resize(nerror);
			factor = 1;

			weighted_errorL.resize(nerrorL);
			wL.resize(nerrorL);
			wL = 0;
			factorL.resize(nerrorL);
			factorL = 1;


			weighted_error_klt.resize(nerror_klt);
			w_klt.resize(nerror_klt);
			w_klt = 0;
			factor_klt.resize(nerror_klt);
			factor_klt = 1;
		}
		//        double t0 = vpTime::measureTimeMs();

		count = 0;
		//reloop = false;
		reloop = true;

		countL = 0;
		int nL = 0;
		//reloop = false;
		reloopL = true;


//#pragma omp parallel
		{
			int local_count = 0;
			if (iter == 0)
				(points[scaleLevel])[0]->computeInteractionMatrixErrorMH(cMo, _I);
			else
			{
#pragma omp for nowait
			for (int k = 0; k < points[scaleLevel].size(); ++k) {
				apControlPoint *p = (points[scaleLevel])[k];
				p->computeInteractionMatrixErrorMH(cMo, _I);
				//p->computeInteractionMatrixError(cMo, _I);

				const double fac = 1;

				if (iter == 0 && p != NULL)
					for (int j = 0; j < 6; ++j)
						L[k][j] = p->L[j]; //On remplit la matrice d'interaction globale
				error[k] = p->error; //On remplit la matrice d'erreur

				if (error[k] <= limite)
					local_count = local_count + 1; //Si erreur proche de 0 on incremente cur

				w[k] = 1;

				if (iter == 0) {
					factor[k] = fac;
					const vpPointSite &site = p->s;
					//if (site.suppress != 0) factor[n] = 0;
					if (site.suppress != 0)
						factor[k] = 0.2;
				}
			}
		}
			//#pragma omp critical
			if (local_count != 0.0)
			{
				count += local_count;
			}
		}

		count = count / (double) nbrow;
		if (count < 0.85) {
			reloop = true;
		}

		int kk=0;

		/*

		std::map<int, apKltControlPoint>::const_iterator iklt = kltPoints[scaleLevel].begin();
		for( ; iklt != kltPoints[scaleLevel].end(); iklt++){
			//int id(iklt->first);
			pklt = iklt->second;
			     //pklt.computeHomography(ctTc0,H);
			     //pklt.computeInteractionMatrixErrorH(_I);
			     pklt.computeInteractionMatrixError(cMo,_I);
				 const double fac_klt = 1;


				 //if (iter == 0 )
						for (int j = 0; j < 6; ++j)
						{
							L_klt[2*kk][j] = pklt.L[0][j]; //On remplit la matrice d'interaction globale
							L_klt[2*kk+1][j] = pklt.L[1][j]; //On remplit la matrice d'interaction globale
						}
					error_klt[2*kk] = pklt.error[0]; //On remplit la matrice d'erreur
					error_klt[2*kk+1] = pklt.error[1]; //On remplit la matrice d'erreur
					//std::cout << " error " << error_klt[2*kk] << " error 1 " << pklt.L[0][0] << std::endl;

					//std::cout << " ok " << error_klt[2*kk] << std::endl;

				//if (error_klt[kk] <= limite)
						//local_count = local_count + 1; //Si erreur proche de 0 on incremente cur

					w_klt[kk] = 1;
					kk++;


		}

		double num = 0;
		double den = 0;

		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			eri = error[i];
			num += wi * vpMath::sqr(eri);
			den += wi;

			weighted_error[i] = wi * eri;
		}

		double numL = 0;
		double denL = 0;

		double wiL;
		double eriL;
		for (int i = 0; i < nerrorL; i++) {
			wiL = wL[i] * factorL[i];
			eriL = errorL[i];
			numL += wiL * vpMath::sqr(eriL);
			denL += wiL;
			weighted_errorL[i] = wiL * eriL;
		}


		double num_klt = 0;
		double den_klt = 0;

		double wi_klt;
		double eri_klt;
		for (int i = 0; i < nerror_klt; i++) {
			wi_klt = w_klt[i] * factor_klt[i];
			eri_klt = error_klt[i];
			num_klt += wi_klt * vpMath::sqr(eri_klt);
			den_klt += wi_klt;

			//weighted_error_klt[i] = wi_klt * eri_klt;
			weighted_error_klt[i] = eri_klt;
		}

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					L[i][j] = w[i] * factor[i] * L[i][j];
				}
			}
		}

		if ((iter == 0) || compute_interaction) {

			for (int i = 0; i < nerrorL; i++) {
				for (int j = 0; j < 6; j++) {
					//L[i][j] = w[i]*factor[i]*L[i][j] ;
					LL[i][j] = factorL[i] * LL[i][j];
				}
			}
		}

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror_klt; i++) {
				for (int j = 0; j < 6; j++) {
					L_klt[i][j] = w_klt[i] * factor_klt[i] * L_klt[i][j];
					//L_klt[i][j] = L_klt[i][j];
				}
			}
		}

		//        std::cout << "t-2 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		CCDTracker.updateCCDPoints(cMo);
		CCDTracker.computeLocalStatistics();
		//        std::cout << "t-1 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		CCDTracker.updateParametersPrev(LTCIL, LTCIR);
		//        double t1 = vpTime::measureTimeMs();
		//        std::cout << "t0 = " << vpTime::measureTimeMs() - t0 << std::endl;

		if (iter > 0)
			CCDTracker.checkCCDConvergence();

		//		LTL = L.AtA();
		//        t0 = vpTime::measureTimeMs();

		//vpMatrix::stackMatrices(L, LL, LH);
				//vpMatrix::stackMatrices(weighted_error, weighted_errorL, weighted_errorH);

		L.AtA(LTL);
		//LH.AtA(LTL);

		computeJTR(L, weighted_error, LTR);
		//std::cout << " Lklt " << L_klt << std::endl;

		//std::cout << " error_klt " << error_klt << std::endl;

		L_klt.AtA(LTL_klt);
		computeJTR(L_klt, weighted_error_klt, LTR_klt);
		//        std::cout << "t1 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();


		std::cout << " frame " << frame << std::endl;
		if(frame0 <= 1)
		{
			v = -0.7 * (LTL).pseudoInverse(LTL.getRows()
							* DBL_EPSILON) * (LTR);
		}
		else {
			v = -0.7 * (LTL + weight_klt * LTL_klt).pseudoInverse(LTL.getRows()
							* DBL_EPSILON) * (LTR + weight_klt * LTR_klt);
		//v = -0.7 * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows()* DBL_EPSILON) * (LTR - weight_ccd * LTCIR);
		}

		*/


		//cMo = vpExponentialMap::direct(v).inverse() * cMo;
		//ctTc0 = vpExponentialMap::direct(v).inverse()*ctTc0;
		//        std::cout << "t2 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//std::cout << "t1 = " << v << std::endl;

		//std::cout << "ctTc0 " << ctTc0 << std::endl;


		iter++;
	}

        //std::cout << "t-2 = " << vpTime::measureTimeMs() - t0 << std::endl;

	/*std::cout << "\t First minimization in " << iter << " iteration "
	 << std::endl;*/

	/*** Second phase ***/
	vpColVector W_true;
	vpMatrix L_true;
	vpRobust robust(nerror);
	robust.setIteration(0);

	vpColVector WL_true;
	vpMatrix LL_true;
	vpRobust robustL(nerrorL);
	robustL.setIteration(0);

	vpColVector WH_true;
	vpMatrix LH_true;


	vpColVector WKLT_true;
	vpMatrix LKLT_true;
	vpRobust robust_klt(nerror_klt);
	robust_klt.setIteration(0);

	vpRobust robustCCD(CCDTracker.nerror_ccd);
	robustCCD.setIteration(0);
	robustCCD.setThreshold(2 / cam.get_px());

	//CCDTracker.initRobust();

	iter = 0;
	int nL = 0;
	//vpColVector error_px(nerror);

	while (((int) ((residu_1 - r) * 1e8) != 0) && (iter < 12)) {

        double t1 = vpTime::measureTimeMs();

                nL = 0;
//#pragma omp parallel for
				for (int k = 0; k < lines[scaleLevel].size(); k++) {
				 l = lines[scaleLevel][k];
				 if(l->MHmeline->points_vect.size()>0)
				 {
				 l->computeInteractionMatrixErrorMH(_I, cMo);
				 for (int i = 0; i < l->nbFeature; i++) {
				 for (int j = 0; j < 6; j++){
				 LL[nL + i][j] = l->L[i][j];
				 }
				 errorL[nL + i] = l->error[i];
				 }
				 nL += l->nbFeature;
				 }
				 }
#pragma omp parallel for
		for (int k = 0; k < points[scaleLevel].size(); k++) {
			const int n = k;
			apControlPoint *p = (points[scaleLevel])[k];
			p->computeInteractionMatrixErrorMH(cMo, _I);
			//p->computeInteractionMatrixError(cMo, _I);
			for (int j = 0; j < 6; j++) {
				L[n][j] = p->L[j];
			}
				error[n] = p->error;
		}
		        //std::cout << "t3 = " << vpTime::measureTimeMs() - t1 << std::endl;
		        //t0 = vpTime::measureTimeMs();

		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 1;

			robust.setThreshold(2 / cam.get_px()); // limite en metre
			robust.MEstimator(vpRobust::TUKEY, error, w);

			weighted_errorL.resize(nerrorL);
			wL.resize(nerrorL);
			wL = 1;

			robustL.setThreshold(2 / cam.get_px()); // limite en metre
			robustL.MEstimator(vpRobust::TUKEY, errorL, wL);

		} else {
			robust.setIteration(iter);
			robust.MEstimator(vpRobust::TUKEY, error, w);

			robustL.setIteration(iter);
			robustL.MEstimator(vpRobust::TUKEY,errorL,wL);
		}

		residu_1 = r;
		L_true = L;
		W_true = vpColVector(nerror);

		LL_true = LL;
		WL_true = vpColVector(nerrorL);



		//---------------------------------------
		// Compute the mean of the weight for a given line reinit if necessary
		nL = 0;
		for (int k = 0; k < lines[scaleLevel].size(); k++) {
		 l = lines[scaleLevel][k];
		 if(l->MHmeline->points_vect.size()>0)
		 {
		 double wmean = 0;
		 for (int i = 0; i < l->nbFeature; i++) {
		 if (iter < 5)
		 {
		 wL[nL+i] = l->weight[i]*wL[nL+i];
		 //std::cout << " weight " << w[n+i] <<std::endl;
		 }
		 wmean += wL[nL + i];

		 }
		 nL += l->nbFeature;
		 wmean /= l->nbFeature;
		 if (l->nbFeature == 0)
		 wmean = 1;
		 l->wmean = wmean;

		 }
		 }
        //std::cout << "t4 = " << vpTime::measureTimeMs() - t0 << std::endl;

        //t0 = vpTime::measureTimeMs();


		int kk=0;
		std::map<int, apKltControlPoint>::const_iterator iklt = kltPoints[scaleLevel].begin();
		for( ; iklt != kltPoints[scaleLevel].end(); iklt++){
			//int id(iklt->first);
			     pklt = iklt->second;
			     //pklt.computeHomography(ctTc0,H);
			     //pklt.computeInteractionMatrixErrorH(_I);
			     pklt.computeInteractionMatrixError(cMo,_I);
				 const double fac_klt = 1;

				 //if (iter == 0 )
						for (int j = 0; j < 6; ++j)
						{
							L_klt[2*kk][j] = pklt.L[0][j]; //On remplit la matrice d'interaction globale
							L_klt[2*kk+1][j] = pklt.L[1][j]; //On remplit la matrice d'interaction globale
						}
					error_klt[2*kk] = pklt.error[0]; //On remplit la matrice d'erreur
					error_klt[2*kk+1] = pklt.error[1]; //On remplit la matrice d'erreur
					kk++;
		}

        //std::cout << "tklt = " << vpTime::measureTimeMs() - t0 << std::endl;

        //t0 = vpTime::measureTimeMs();


		if (iter == 0) {
			weighted_error_klt.resize(nerror_klt);
			w_klt.resize(nerror_klt);
			w_klt = 1;

			robust_klt.setThreshold(2 / cam.get_px()); // limite en metre
			robust_klt.MEstimator(vpRobust::TUKEY, error_klt, w_klt);
		} else {
			robust_klt.setIteration(iter);
			robust_klt.MEstimator(vpRobust::TUKEY, error_klt, w_klt);
		}

		LKLT_true = L_klt;
		WKLT_true = vpColVector(nerror_klt);


		double num = 0;
		double den = 0;
		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			eri = error[i];
			W_true[i] = wi * wi;
			num += wi * vpMath::sqr(eri);
			den += wi;
			weighted_error[i] = wi * eri;
		}

		double numL = 0;
		double denL = 0;
		double wiL;
		double eriL;
		for (int i = 0; i < nerrorL; i++) {
			wiL = wL[i] * factorL[i];
			WL_true = wiL * wiL;
			eriL = errorL[i];
			numL += wiL * vpMath::sqr(eriL);
			denL += wiL;

			weighted_errorL[i] = wiL * eriL;
		}

		double num_klt = 0;
		double den_klt = 0;
		double wi_klt;
		double eri_klt;
		for (int ii = 0; ii < nerror_klt; ii++) {
			wi_klt = w_klt[ii] * factor_klt[ii];
			eri_klt = error_klt[ii];
			WKLT_true[ii] = wi_klt * wi_klt;
			num_klt += wi_klt * vpMath::sqr(eri_klt);
			den_klt += wi_klt;

			weighted_error_klt[ii] = wi_klt * eri_klt;
			//weighted_error_klt[ii] = eri_klt;
		}

        //std::cout << "tklt2 = " << vpTime::measureTimeMs() - t0 << std::endl;

        //t0 = vpTime::measureTimeMs();

		r = sqrt(num / den); //Le critere d'arret prend en compte le poids

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					L[i][j] = w[i] * factor[i] * L[i][j];
				}
			}
		}

		if ((iter == 0) || compute_interaction) {

			for (int i = 0; i < nerrorL; i++) {
				for (int j = 0; j < 6; j++) {
					LL[i][j] = wL[i] * factorL[i] * LL[i][j];
				}
			}
		}

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror_klt; i++) {
				for (int j = 0; j < 6; j++) {
					L_klt[i][j] = w_klt[i] * factor_klt[i] * L_klt[i][j];

					//L_klt[i][j] =  L_klt[i][j];

				}
			}
		}

		//std::cout << "tIntM = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        std::cout << "t1 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		robustCCD.setIteration(iter);
		CCDTracker.updateCCDPoints(cMo);
		//        std::cout << "t2a = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		CCDTracker.computeLocalStatistics();

		//        std::cout << "t2b = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		//		double t0 = vpTime::measureTimeMs();
		//if(iter>5)
		//CCDTracker.updateParametersRobustPrev(LTCIL, LTCIR, robustCCD);
		//else CCDTracker.updateParametersPrev(LTCIL,LTCIR);

	    CCDTracker.updateParametersRobustPrev(LTCIL, LTCIR, robustCCD);
		//CCDTracker.updateParametersPrev(LTCIL,LTCIR);
		//		double t1 = vpTime::measureTimeMs();
		//std::cout << " timeupdate " << t1 -t0 << std::endl;
		if (iter > 0)
			CCDTracker.checkCCDConvergence();
		//        std::cout << "t2c = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();


		//t0 = vpTime::measureTimeMs();
		vpMatrix::stackMatrices(L, LL, LH);
		 vpMatrix::stackMatrices(weighted_error, weighted_errorL,
		 weighted_errorH);

		 //vpMatrix::stackMatrices(LH,L_klt,LH1);
		 //vpMatrix::stackMatrices(weighted_errorH, weighted_error_klt,
		 //weighted_errorH1);


		LH.AtA(LTL);
		//LH1.AtA(LTL);

		//		LTL = L.AtA();
		//L.AtA(LTL);
		//computeJTR(L, weighted_error, LTR);
		computeJTR(LH, weighted_errorH, LTR);
		//computeJTR(LH1, weighted_errorH1, LTR);
		L_klt.AtA(LTL_klt);
		computeJTR(L_klt, weighted_error_klt, LTR_klt);

		//t0 = vpTime::measureTimeMs();
		vpMatrix::stackMatrices(error, errorL, errorH);

		double stdME = sqrt((double)(weighted_errorH.t()*weighted_errorH)/(weighted_errorH.size()));
		double stdKLT = sqrt((double)(weighted_error_klt.t()*weighted_error_klt)/(weighted_error_klt.size()));

		double wghtME = ((double)1/weighted_errorH.size())*(1/stdME);
		double wghtKLT = ((double)1/weighted_error_klt.size())*(1/stdKLT);
		double wghtCCD = ((double)1/CCDTracker.error_ccd.size());

		wghtME = 1;
		wghtKLT = 100;

		//std::cout << "t3 = " << vpTime::measureTimeMs() - t0 << std::endl;


		if(frame0 <= 1)
		{
		v = -lambda * (weight_me * wghtME*LTL + weight_ccd * wghtCCD * LTCIL).pseudoInverse(LTL.getRows() * DBL_EPSILON) * (weight_me*wghtME*LTR - weight_ccd * wghtCCD *  LTCIR);
		}
		else
		{
		v = -lambda * (weight_me*wghtME*LTL + weight_ccd * wghtCCD * LTCIL + weight_klt * wghtKLT * LTL_klt).pseudoInverse(LTL.getRows() * DBL_EPSILON) * (weight_me * wghtME*LTR - weight_ccd * wghtCCD * LTCIR + weight_klt * wghtKLT * LTR_klt);
		}

		/*v = -lambda * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows()
				* DBL_EPSILON) * (LTR - weight_ccd * LTCIR );*/
		/*if(frame0 <= 1)
		{
		v = -lambda * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows()
					* DBL_EPSILON) * (LTR - weight_ccd * LTCIR);
		}
		else
		{
		v = -lambda * (LTL + weight_ccd * LTCIL + weight_klt * LTL_klt).pseudoInverse(LTL.getRows()
				* DBL_EPSILON) * (LTR - weight_ccd * LTCIR + weight_klt * LTR_klt);
		}*/

		//std::cout << "weight0 " << weight_me * wghtME * sqrt((LTR).t()*(LTR)) << " wght1 " << weight_ccd*wghtCCD*sqrt((LTCIR).t()*(LTCIR)) << " wghtklt " << weight_klt*wghtKLT*sqrt((LTR_klt).t()*(LTR_klt)) << std::endl;
		//std::cout << "weight0 " << weight_me * wghtME << " wght1 " << weight_ccd*wghtCCD << " wghtklt " << weight_klt * wghtKLT << std::endl;

		//std::cout << "weight0 " << wghtME << " wght1 " << weight_ccd*wghtCCD << " wghtklt " << wghtKLT << std::endl;


		cMo = vpExponentialMap::direct(v).inverse() * cMo;
		ctTc0 = vpExponentialMap::direct(v).inverse()*ctTc0;

		std::cout << "tstack = " << vpTime::measureTimeMs() - t1 << std::endl;

		//        std::cout << "t3 = " << vpTime::measureTimeMs() - t0 << std::endl;

/*		if (computeCovariance) {

			t1  = vpTime::measureTimeMs();
			vpMatrix D;
			D.diag(W_true);

			vpMatrix::stackMatrices(W_true, WL_true, WH_true);
			vpMatrix::stackMatrices(error, errorL, errorH);
			vpMatrix::stackMatrices(L_true, LL_true, LH_true);
			vpMatrix DH;
			DH.diag(WH_true);

			//covarianceMatrixME =  0.25*covarianceMatrixME + 1.5*computeCovarianceMatrix(L_true, -v, points[scaleLevel].size()*lambda * weighted_error,D);
			covarianceMatrixME =  0.25*covarianceMatrixME + 1.5*computeCovarianceMatrix(LH_true, -v, errorH,DH);
			//covarianceMatrix = covarianceMatrixME;

			vpMatrix DKLT;
			D.diag(WKLT_true);
			covarianceMatrixKLT = 0.25*covarianceMatrixKLT + 1.5*computeCovarianceMatrix(LKLT_true, -v, error_klt,DKLT);

			std::cout << "tcov = " << vpTime::measureTimeMs() - t1 << std::endl;


		}*/

		iter++;
	}
	if (computeCovariance) {
		/*vpMatrix D;
		D.diag(W_true);
		covarianceMatrix = computeCovarianceMatrix(L_true, -v, lambda * error,
				D);*/

		vpMatrix D;
		D.diag(W_true);
		covarianceMatrixP = computeCovarianceMatrix(L_true, -v,  error,D);
		vpMatrix DL;
		DL.diag(WL_true);
		covarianceMatrixL = computeCovarianceMatrix(LL_true, -v,  errorL,DL);

		vpMatrix::stackMatrices(W_true, WL_true, WH_true);
		vpMatrix::stackMatrices(error, errorL, errorH);
		vpMatrix::stackMatrices(L_true, LL_true, LH_true);
		vpMatrix DH;
		DH.diag(WH_true);
		vpColVector sig(1);


       double stdME = sqrt((double)(errorH.t()*errorH)/(errorH.size()));
       double stdKLT = sqrt((double)(error_klt.t()*error_klt)/(error_klt.size()));
       double wghtME = ((double)1/errorH.size())*(1/stdME);
       double wghtKLT = ((double)1/error_klt.size())*(1/stdKLT);
	   double wghtCCD = ((double)1/CCDTracker.error_ccd.size());

	   sig = stdME;
	   //covarianceMatrixME = computeCovarianceMatrix(LH_true, -v, sig,DH);

	   covarianceMatrixME = computeCovarianceMatrix(LH_true, -v , errorH ,DH);


       //std::cout << " weights " <<  << " weightklt " << (1/(double)error_klt.size())*error_klt[0]/sqrt(wghtKLT) << std::endl;
       std::cout << " weights " << (wghtME)*sqrt((LTR).t()*(LTR)) << " weightklt " << (wghtKLT)*sqrt((LTR_klt).t()*(LTR_klt)) << std::endl;//<< sqrt((weight_ccd * LTCIR).t()*(weight_ccd * LTCIR)) << std::endl;
       //std::cout << " weights " << (double)sqrt((error).t()*(error))/(error.size()) << " weightklt " << (double)sqrt((error_klt).t()*(error_klt))/(error_klt.size()) << " "<< sqrt((weight_ccd * LTCIR).t()*(weight_ccd * LTCIR)) << std::endl;
       std::cout << " weights " <<  stdME << " weightklt " << CCDTracker.error_ccd.size() << std::endl;

		vpMatrix DKLT;
		DKLT.diag(WKLT_true);
		sig= stdKLT;
		//covarianceMatrixKLT = computeCovarianceMatrix(LKLT_true, -v, sig,DKLT);

		covarianceMatrixKLT = computeCovarianceMatrix(LKLT_true, -v, error_klt,DKLT);


//		covarianceMatrixME =  computeCovarianceMatrix(L_true, -v, points[scaleLevel].size()*lambda * weighted_error,D);


		std::cout << "covMatME0 " << covarianceMatrixME[0][0] << " covMatKLT " << covarianceMatrixKLT[0][0] << std::endl;

		//vpMatrix inv0 = inv.pseudoInverse();
		std::cout << "covMatP " << errorH.size() << "covMatL " << error_klt.size() << std::endl;
		covarianceMatrixCCD = CCDTracker.sigmaP;

		vpMatrix inv = (weight_me*(covarianceMatrixME.pseudoInverse()) + weight_ccd*(covarianceMatrixCCD.pseudoInverse()) + weight_klt*covarianceMatrixKLT.pseudoInverse());
		covarianceMatrix = inv.pseudoInverse();
		//covarianceMatrix = (0*1+weight_ccd*1)*(weight_ccd**CCDTracker.sigmaP+L_true.t()*L_true).pseudoInverse();
		//std::cout << "covMatCCD0 " << CCDTracker.sigmaF[0][0] << std::endl;
		std::cout << "covMatCCD1 " << CCDTracker.sigmaP[0][0] << " covMatCCD2 " << covarianceMatrix[0][0] << std::endl;

	}
	//   cout << "\t Robust minimization in " << iter << " iteration " << endl ;
	//    std::cout << "error: " << (residu_1 - r) << std::endl;
}


void apMbTracker::computeVVSCCDMHPrev(const vpImage<unsigned char>& _I,
		const vpImage<vpRGBa>& _IRGB) {
	double residu_1 = 1e3;
	double r = 1e3 - 1;
	vpMatrix LTL;
	vpColVector LTR;

	// compute the interaction matrix and its pseudo inverse

	vpColVector w;
	vpColVector weighted_error;
	vpColVector factor;

	CCDTracker.init(CCDParameters, cam);
	CCDTracker.setImage(_IRGB);

	CCDTracker.setPrevImage(IprecRGB);
	CCDTracker.updateCCDPoints(cMo);
	CCDTracker.computeLocalStatisticsPrev(_I);

	covarianceMatrix.resize(6,6);
	covarianceMatrixME.resize(6,6);
	covarianceMatrixP.resize(6,6);
	covarianceMatrixCCD.resize(6,6);

	unsigned int iter = 0;
	//Nombre de moving edges
	int nbrow = 0;

#pragma omp parallel for
	for (int k = 0; k < points[scaleLevel].size(); k++) {
		apControlPoint *p = (points[scaleLevel])[k];
		p->initInteractionMatrixError();
	}
	nbrow = points[scaleLevel].size();

	if (nbrow == 0) {
		vpERROR_TRACE(
				"\n\t\t Error-> not enough data in the interaction matrix...");
		throw vpTrackingException(vpTrackingException::notEnoughPointError,
				"\n\t\t Error-> not enough data in the interaction matrix...");
	}

	vpMatrix L(nbrow, 6);

	vpMatrix LTCIL(6, 6);
	vpColVector LTCIR(6);

	// compute the error vector
	vpColVector error(nbrow);
	int nerror = error.getRows();
	vpColVector v;

	double limite = 3; //Une limite de 3 pixels
	limite = limite / cam.get_px(); //Transformation limite pixel en limite metre.

	//Parametre pour la premiere phase d'asservissement
	bool reloop = true;
	double count = 0;

	/*** First phase ***/

	while (reloop == true && iter < 1) {
		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 1;
			factor.resize(nerror);
			factor = 1;
		}
		//        double t0 = vpTime::measureTimeMs();

		count = 0;
		//reloop = false;
		reloop = true;
/*
#pragma omp parallel shared(L,error)
		{
			int local_count = 0;
#pragma omp for nowait
			for (int k = 0; k < points[scaleLevel].size(); ++k) {
				apControlPoint *p = (points[scaleLevel])[k];
				p->computeInteractionMatrixErrorMH(cMo, _I);
				//p->computeInteractionMatrixError(cMo, _I);

				const double fac = 1;

				if (iter == 0 && p != NULL)
					for (int j = 0; j < 6; ++j)
						L[k][j] = p->L[j]; //On remplit la matrice d'interaction globale
				error[k] = p->error; //On remplit la matrice d'erreur

				if (error[k] <= limite)
					local_count = local_count + 1; //Si erreur proche de 0 on incremente cur

				w[k] = 1;

				if (iter == 0) {
					factor[k] = fac;
					const vpPointSite &site = p->s;
					//if (site.suppress != 0) factor[n] = 0;
					if (site.suppress != 0)
						factor[k] = 0.2;
				}
			}
			if (local_count != 0.0)
#pragma omp critical
			{
				count += local_count;
			}
		}

		count = count / (double) nbrow;
		if (count < 0.85) {
			reloop = true;
		}


		double num = 0;
		double den = 0;

		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			eri = error[i];
			num += wi * vpMath::sqr(eri);
			den += wi;

			weighted_error[i] = wi * eri;
		}

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					L[i][j] = w[i] * factor[i] * L[i][j];
				}
			}
		}

		//        std::cout << "t-2 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		CCDTracker.updateCCDPoints(cMo);
		CCDTracker.computeLocalStatistics();
		//        std::cout << "t-1 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		CCDTracker.updateParametersPrev(LTCIL, LTCIR);
		//        double t1 = vpTime::measureTimeMs();
		//        std::cout << "t0 = " << vpTime::measureTimeMs() - t0 << std::endl;

		if (iter > 0)
			CCDTracker.checkCCDConvergence();

		//		LTL = L.AtA();
		//        t0 = vpTime::measureTimeMs();
		L.AtA(LTL);
		computeJTR(L, weighted_error, LTR);
		//        std::cout << "t1 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		//v = -0.7 * (LTL).pseudoInverse(LTL.getRows()
			//	* DBL_EPSILON) * (LTR);


		double stdME = sqrt((double)(weighted_error.t()*weighted_error)/(weighted_error.size()));
		double wghtME = ((double)1/weighted_error.size())*(1/stdME);
		double wghtCCD = ((double)1/CCDTracker.error_ccd.size());

		wghtME = 1;


		v = -0.7 * (weight_me * wghtME*LTL + weight_ccd * wghtCCD * LTCIL).pseudoInverse(LTL.getRows() * DBL_EPSILON) * (weight_me*wghtME*LTR - weight_ccd * wghtCCD *  LTCIR);

//		v = -0.7 * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows() * DBL_EPSILON) * (LTR - weight_ccd * LTCIR);
		//cMo = vpExponentialMap::direct(v).inverse() * cMo;
		//        std::cout << "t2 = " << vpTime::measureTimeMs() - t0 << std::endl;

*/
		iter++;
	}
	/*std::cout << "\t First minimization in " << iter << " iteration "
	 << std::endl;*/

	/*** Second phase ***/
	vpColVector W_true;
	vpMatrix L_true;
	vpRobust robust(nerror);
	robust.setIteration(0);

	//vpRobust robustCCD(CCDTracker.nerror_ccd);
	vpRobust robustCCD(CCDTracker.resolution);
	robustCCD.setIteration(0);
	robustCCD.setThreshold(2 / cam.get_px());

	//CCDTracker.initRobust();

	iter = 0;
	//vpColVector error_px(nerror);

        while (/*((int) ((residu_1 - r) * 1e8) != 0) &&*/ (iter < 8)) {
		//        double t0 = vpTime::measureTimeMs();
#pragma omp parallel for //shared(L,error)
		for (int k = 0; k < points[scaleLevel].size(); k++) {
			const int n = k;
			apControlPoint *p = (points[scaleLevel])[k];
			p->computeInteractionMatrixErrorMH(cMo, _I);
			//p->computeInteractionMatrixError(cMo, _I);
                        for (int j = 0; j < 6; j++) {
			        L[n][j] = p->L[j];
/*				L[n][3] = 0;
				L[n][4] = 0;
				L[n][5] = 0;*/
			}
			error[n] = p->error;
		}
		//        std::cout << "t0 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();

		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 1;

			robust.setThreshold(2 / cam.get_px()); // limite en metre
			robust.MEstimator(vpRobust::TUKEY, error, w);
		} else {
			robust.setIteration(iter);
			robust.MEstimator(vpRobust::TUKEY, error, w);
		}

		residu_1 = r;
		L_true = L;
		W_true = vpColVector(nerror);

		double num = 0;
		double den = 0;
		double wi;
		double eri;
#pragma omp parallel for
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			eri = error[i];
			W_true[i] = wi * wi;
			//num += wi * vpMath::sqr(eri);
			//den += wi;

			weighted_error[i] = wi * eri;
		}

		//r = sqrt(num / den); //Le critere d'arret prend en compte le poids

		if ((iter == 0) || compute_interaction) {
#pragma omp parallel for
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					L[i][j] = w[i] * factor[i] * L[i][j];
				}
			}
		}

		//        std::cout << "t1 = " << vpTime::measureTimeMs() - t0 << std::endl;
		double t0 = vpTime::measureTimeMs();
		robustCCD.setIteration(iter);
		CCDTracker.updateCCDPoints(cMo);
		//        std::cout << "t2a = " << vpTime::measureTimeMs() - t0 << std::endl;
		         //t0 = vpTime::measureTimeMs();
		CCDTracker.computeLocalStatistics();
		        //std::cout << "t2b = " << vpTime::measureTimeMs() - t0 << std::endl;
		        //double t1 = vpTime::measureTimeMs();
		//if(iter>5)
		//CCDTracker.updateParametersRobustPrev(LTCIL, LTCIR, robustCCD);
		//else CCDTracker.updateParametersPrev(LTCIL,LTCIR);

	    /*CCDTracker.updateParametersRobustPrev(LTCIL, LTCIR, robustCCD);

	    int up,vp;
	    for (int ii = 0; ii < CCDTracker.w_ccd.size(); ii ++)
	    {
	    	if(CCDTracker.w_ccd[ii] < 0.7)
	    	{
	    	    apControlPoint *p = CCDTracker.pointsCCD[scaleLevel][ii];
	    	    up = (int)p->icpoint->get_u();
	    	    vp = (int)p->icpoint->get_v();
	    	    vpDisplay::displayCross(_I,vp,up,5,vpColor::red,5);
	    	}
            }*/

		CCDTracker.updateParametersPrev(LTCIL,LTCIR);
				//double t2 = vpTime::measureTimeMs();
		//std::cout << " timestats " << t1 -t0 << std::endl;
		//std::cout << " timeupdate " << t2 -t1 << std::endl;
		if (iter > 0)
			CCDTracker.checkCCDConvergence();
		//        std::cout << "t2c = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();

		//		LTL = L.AtA();
		L.AtA(LTL);
		computeJTR(L, weighted_error, LTR);


		double stdME= sqrt((double)(weighted_error.t()*weighted_error)/(weighted_error.size()));
		double wghtME = ((double)1/weighted_error.size())*(1/stdME);
		double stdCCD = sqrt((double)(CCDTracker.error_ccd.t()*CCDTracker.error_ccd)/(CCDTracker.error_ccd.size()));
		double wghtCCD = ((double)1/(10*CCDTracker.error_ccd.size()));
		//double wghtCCD = ((double)1/(CCDTracker.error_ccd.size()))*(1/stdCCD);

                wghtME = 0.7;
		//wghtME = sigmag;

		//std::cout << " wghtME " << weight_me*wghtME*LTR << " wght CCD " << weight_ccd*wghtCCD*LTCIR << std::endl;

		v = -lambda * (weight_me * wghtME*LTL + weight_ccd * wghtCCD * LTCIL).pseudoInverse(LTL.getRows() * DBL_EPSILON) * (weight_me*wghtME*LTR - weight_ccd * wghtCCD *  LTCIR);

		/*v[3] = 0;
		v[4] = 0;
		v[5] = 0;*/

		//v = -lambda * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows() * DBL_EPSILON) * (LTR - weight_ccd * LTCIR);
		cMo = vpExponentialMap::direct(v).inverse() * cMo;
		//        std::cout << "t3 = " << vpTime::measureTimeMs() - t0 << std::endl;
		if (computeCovariance) {
			vpMatrix D;
			D.diag(W_true);
			//covarianceMatrixME =  computeCovarianceMatrix(L_true, -v, points[scaleLevel].size()*lambda * weighted_error,D);
			//covarianceMatrix = covarianceMatrixME;
			//covarianceMatrixME =  0.25*covarianceMatrixME + 0.75*2*computeCovarianceMatrix(L_true, -v, lambda*error,D);

		}

		iter++;
	}
	if (computeCovariance) {

		sigmag = ((double)1/weighted_error.size())*(1/(sqrt((double)(weighted_error.t()*weighted_error)/(weighted_error.size()))));
		std::cout << " sigmag " << 1/(weighted_error.size()*sigmag) << std::endl;

		/*vpMatrix D;
		D.diag(W_true);
		covarianceMatrix = computeCovarianceMatrix(L_true, -v, lambda * error,
				D);*/
		vpMatrix D;
		D.diag(W_true);
		covarianceMatrixME = computeCovarianceMatrix(L_true, -v, lambda*error,D);


//		covarianceMatrixME =  computeCovarianceMatrix(L_true, -v, points[scaleLevel].size()*lambda * weighted_error,D);


		//std::cout << "covMatEdge " << covarianceMatrix[0][0] << std::endl;
		covarianceMatrixCCD = CCDTracker.sigmaP;
		//covarianceMatrix = (0*1+weight_ccd*1)*(weight_ccd**CCDTracker.sigmaP+L_true.t()*L_true).pseudoInverse();
		//std::cout << "covMatCCD0 " << CCDTracker.sigmaF[0][0] << std::endl;
		//std::cout << "covMatCCD1 " << CCDTracker.sigmaP[0][0] + CCDTracker.sigmaP[1][1] + CCDTracker.sigmaP[2][2] << " covMatCCD2 " << CCDTracker.sigmaP[3][3] + CCDTracker.sigmaP[4][4] + CCDTracker.sigmaP[5][5] << std::endl;

		vpMatrix inv = (weight_me*(covarianceMatrixME.pseudoInverse(DBL_EPSILON)) + weight_ccd*(covarianceMatrixCCD.pseudoInverse(DBL_EPSILON)) );
						covarianceMatrix = (weight_me + weight_ccd)*(inv.pseudoInverse(DBL_EPSILON));

	}
        //   cout << "\t Robust minimization in " << iter << " iteration " << endl ;
	//    std::cout << "error: " << (residu_1 - r) << std::endl;
}


void apMbTracker::computeVVSCCDMHPrevSpace(const vpImage<unsigned char>& _I,
		const vpImage<vpRGBa>& _IRGB) {
	double residu_1 = 1e3;
	double r = 1e3 - 1;
	vpMatrix LTL;
	vpColVector LTR;

	// compute the interaction matrix and its pseudo inverse

	vpColVector w;
	vpColVector weighted_error;
	vpColVector factor;

	CCDTracker.init(CCDParameters, cam);
	CCDTracker.setImage(_IRGB);

	CCDTracker.setPrevImage(IprecRGB);

	CCDTracker.updateCCDPoints(cMo);
	CCDTracker.computeLocalStatisticsPrev(_I);

	unsigned int iter = 0;
	//Nombre de moving edges
	int nbrow = 0;

//#pragma omp parallel for
	for (int k = 0; k < points[scaleLevel].size(); k++) {
		apControlPoint *p = (points[scaleLevel])[k];
		p->initInteractionMatrixError();
	}
	nbrow = points[scaleLevel].size();

	if (nbrow == 0) {
		vpERROR_TRACE(
				"\n\t\t Error-> not enough data in the interaction matrix...");
		throw vpTrackingException(vpTrackingException::notEnoughPointError,
				"\n\t\t Error-> not enough data in the interaction matrix...");
	}

	vpMatrix L(nbrow, 6);

	vpMatrix LTCIL(6, 6);
	vpColVector LTCIR(6);

	// compute the error vector
	vpColVector error(nbrow);
	int nerror = error.getRows();
	vpColVector v;

	double limite = 3; //Une limite de 3 pixels
	limite = limite / cam.get_px(); //Transformation limite pixel en limite metre.

	//Parametre pour la premiere phase d'asservissement
	bool reloop = true;
	double count = 0;

	/*** First phase ***/

	while (reloop == true && iter < 3) {
		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 0;
			factor.resize(nerror);
			factor = 1;
		}
		//        double t0 = vpTime::measureTimeMs();

		count = 0;
		//reloop = false;
		reloop = true;
//#pragma omp parallel
		{
			int local_count = 0;
//#pragma omp for nowait
			for (int k = 0; k < points[scaleLevel].size(); ++k) {
				apControlPoint *p = (points[scaleLevel])[k];
				p->computeInteractionMatrixErrorMH(cMo, _I);

				const double fac = 1;

				if (iter == 0 && p != NULL)
					for (int j = 0; j < 6; ++j)
						L[k][j] = p->L[j]; //On remplit la matrice d'interaction globale
				error[k] = p->error; //On remplit la matrice d'erreur

				if (error[k] <= limite)
					local_count = local_count + 1; //Si erreur proche de 0 on incremente cur

				w[k] = 1;

				if (iter == 0) {
					factor[k] = fac;
					const vpPointSite &site = p->s;
					//if (site.suppress != 0) factor[n] = 0;
					if (site.suppress != 0)
						factor[k] = 0.2;
				}
			}
			if (local_count != 0.0)
//#pragma omp critical
			{
				count += local_count;
			}
		}

		count = count / (double) nbrow;
		if (count < 0.85) {
			reloop = true;
		}

		double num = 0;
		double den = 0;

		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			eri = error[i];
			num += wi * vpMath::sqr(eri);
			den += wi;

			weighted_error[i] = wi * eri;
		}

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					L[i][j] = w[i] * factor[i] * L[i][j];
				}
			}
		}

		//        std::cout << "t-2 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		CCDTracker.updateCCDPoints(cMo);
		CCDTracker.computeLocalStatisticsSpace();
		//        std::cout << "t-1 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		CCDTracker.updateParametersPrev(LTCIL, LTCIR);
		//        double t1 = vpTime::measureTimeMs();
		//        std::cout << "t0 = " << vpTime::measureTimeMs() - t0 << std::endl;

		if (iter > 0)
			CCDTracker.checkCCDConvergence();

		//		LTL = L.AtA();
		//        t0 = vpTime::measureTimeMs();
		L.AtA(LTL);
		computeJTR(L, weighted_error, LTR);
		//        std::cout << "t1 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		v = -0.7 * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows()
				* DBL_EPSILON) * (LTR - weight_ccd * LTCIR);
		cMo = vpExponentialMap::direct(v).inverse() * cMo;
		//        std::cout << "t2 = " << vpTime::measureTimeMs() - t0 << std::endl;

		iter++;
	}
	/*std::cout << "\t First minimization in " << iter << " iteration "
	 << std::endl;*/

	/*** Second phase ***/
	vpColVector W_true;
	vpMatrix L_true;
	vpRobust robust(nerror);
	robust.setIteration(0);

	vpRobust robustCCD(CCDTracker.nerror_ccd);
	robustCCD.setIteration(0);
	robustCCD.setThreshold(2 / cam.get_px());

	//CCDTracker.initRobust();

	iter = 0;
	//vpColVector error_px(nerror);

	while (((int) ((residu_1 - r) * 1e8) != 0) && (iter < 10)) {
		//        double t0 = vpTime::measureTimeMs();
//#pragma omp parallel for
		for (int k = 0; k < points[scaleLevel].size(); k++) {
			const int n = k;
			apControlPoint *p = (points[scaleLevel])[k];
			p->computeInteractionMatrixErrorMH(cMo, _I);
			for (int j = 0; j < 6; j++) {
				L[n][j] = p->L[j];
				error[n] = p->error;
			}
		}
		//        std::cout << "t0 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();

		if (iter == 0) {
			weighted_error.resize(nerror);
			w.resize(nerror);
			w = 1;

			robust.setThreshold(2 / cam.get_px()); // limite en metre
			robust.MEstimator(vpRobust::TUKEY, error, w);
		} else {
			robust.setIteration(iter);
			robust.MEstimator(vpRobust::TUKEY, error, w);
		}

		residu_1 = r;

		L_true = L;
		W_true = vpColVector(nerror);

		double num = 0;
		double den = 0;
		double wi;
		double eri;
		for (int i = 0; i < nerror; i++) {
			wi = w[i] * factor[i];
			eri = error[i];
			W_true[i] = wi * wi;
			num += wi * vpMath::sqr(eri);
			den += wi;

			weighted_error[i] = wi * eri;
		}

		r = sqrt(num / den); //Le critere d'arret prend en compte le poids

		if ((iter == 0) || compute_interaction) {
			for (int i = 0; i < nerror; i++) {
				for (int j = 0; j < 6; j++) {
					L[i][j] = w[i] * factor[i] * L[i][j];
				}
			}
		}

		//        std::cout << "t1 = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		robustCCD.setIteration(iter);
		CCDTracker.updateCCDPoints(cMo);
		//        std::cout << "t2a = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		CCDTracker.computeLocalStatisticsSpace();
		//        std::cout << "t2b = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();
		//		double t0 = vpTime::measureTimeMs();
		//if(iter>5)
		CCDTracker.updateParametersRobustPrev(LTCIL, LTCIR, robustCCD);
		//else CCDTracker.updateParametersPrev(LTCIL,LTCIR);

	    //CCDTracker.updateParametersRobust(LTCIL, LTCIR, robustCCD);
		//CCDTracker.updateParameters(LTCIL,LTCIR);
		//		double t1 = vpTime::measureTimeMs();
		//std::cout << " timeupdate " << t1 -t0 << std::endl;
		if (iter > 0)
			CCDTracker.checkCCDConvergence();
		//        std::cout << "t2c = " << vpTime::measureTimeMs() - t0 << std::endl;
		//        t0 = vpTime::measureTimeMs();

		//		LTL = L.AtA();
		L.AtA(LTL);
		computeJTR(L, weighted_error, LTR);
		v = -lambda * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows()
				* DBL_EPSILON) * (LTR - weight_ccd * LTCIR);
		cMo = vpExponentialMap::direct(v).inverse() * cMo;
		//        std::cout << "t3 = " << vpTime::measureTimeMs() - t0 << std::endl;

		iter++;
	}
	if (computeCovariance) {
		vpMatrix D;
		D.diag(W_true);
		covarianceMatrix = computeCovarianceMatrix(L_true, -v, lambda * error,
				D);
	}
	//   cout << "\t Robust minimization in " << iter << " iteration " << endl ;
	//    std::cout << "error: " << (residu_1 - r) << std::endl;
}

/*!
 Check if the tracking failed.

 \throw vpTrackingException::fatalError if the test fails.
 */
/*void
 apMbTracker::testTracking()
 {
 int nbExpectedPoint = 0;
 int nbGoodPoint = 0;
 int nbBadPoint = 0;

 apControlPoint *p ;

 ;
 for (int k = 0; k<points[scaleLevel].size(); k++)
 {
 nbExpectedPoint += expecteddensity;
 p = (points[scaleLevel])[k];
 if (p->suppress == 0) nbGoodPoint++;
 else nbBadPoint++;
 points[scaleLevel].next();
 }

 if (nbGoodPoint < percentageGdPt *(nbGoodPoint+nbBadPoint) || nbExpectedPoint < 2)
 {
 std::cout << "nbGoodPoint :" << nbGoodPoint << std::endl;
 std::cout << "nbBadPoint :" << nbBadPoint << std::endl;
 std::cout << "nbExpectedPoint :" << nbExpectedPoint << std::endl;
 throw vpTrackingException(vpTrackingException::fatalError, "test Tracking fail");
 }

 TO DO
 }*/

/*!
 Compute each state of the tracking procedure for all the feature sets.

 If the tracking is considered as failed an exception is thrown.

 \param I : The image.
 */

void apMbTracker::track(const vpImage<unsigned char> &I, const vpImage<
		vpRGBa> &IRGB, const vpImage<vpRGBa> &Inormd, const vpImage<
		unsigned char>& Ior, const vpImage<unsigned char>& Itex,
        const double dist, bool printLog) {
	initPyramid(I, Ipyramid);
	initPyramid(Iprec, Ipyramidprec);
	switch (trackingType) {
	case CCD_SH:
	case CCD_MH:
	case CCD_LINES_MH:
	case CCD_MH_KLT:
	case CCD_LINES_MH_KLT:
		initPyramid(IRGB, IRGBpyramid);
	}

	for (int lvl = (scales.size() - 1); lvl >= 0; lvl -= 1) {
		if (scales[lvl]) {
			vpHomogeneousMatrix cMo_1 = cMo;
			try {
				downScale(lvl);
				double t0 = vpTime::measureTimeMs();
				try {
					switch (trackingType) {
					case POINTS_SH:
					case POINTS_MH:
						extractControlPoints(*Ipyramid[lvl], Inormd, Ior, Itex,
								dist);
						break;
					case LINES_MH:
						extractControlPointsLines(*Ipyramid[lvl], Inormd, Ior,
								Itex, dist);
						break;
					case CCD_SH:
					case CCD_MH:
						extractControlPointsCCD(*Ipyramid[lvl], Inormd, Ior,
								Itex, dist);
						break;
					case CCD_MH_KLT:
						extractControlPointsCCD(*Ipyramid[lvl], Inormd, Ior,
								Itex, dist);
						extractKltControlPointsFAST(*Ipyramid[lvl],Inormd, Ior, Itex, dist);
						break;
					case CCD_LINES_MH:
						extractControlPointsLinesCCD(*Ipyramid[lvl], Inormd,
								Ior, Itex, dist);
						break;
					case CCD_LINES_MH_KLT:
						extractControlPointsLinesCCD(*Ipyramid[lvl], Inormd, Ior,
								Itex, dist);
						extractKltControlPoints(*Ipyramid[lvl],Inormd, Ior, Itex, dist);
						break;
					}
				} catch (...) {
					vpTRACE("Error in points extraction");
				}

				double t1 = vpTime::measureTimeMs();
                if(printLog)
                    std::cout << "timeextract " << t1 - t0 << std::endl;
				timeextract = t1 - t0;
				double t2 = vpTime::measureTimeMs();
				try {
					switch (trackingType) {
					case POINTS_SH:
					case CCD_SH:
						trackControlPoints(*Ipyramid[lvl]);
						break;
					case POINTS_MH:
					case CCD_MH:
						trackControlPointsMH(*Ipyramid[lvl]);
						break;
					case CCD_MH_KLT:
						trackControlPointsMH(*Ipyramid[lvl]);
						trackKltControlPoints(*Ipyramid[lvl]);
						break;
					case LINES_MH:
					case CCD_LINES_MH:
						trackControlPointsMH(*Ipyramid[lvl]);
						trackControlPointsLinesMH(*Ipyramid[lvl]);
						break;
					case CCD_LINES_MH_KLT:
						trackControlPointsMH(*Ipyramid[lvl]);
						trackControlPointsLinesMH(*Ipyramid[lvl]);
						trackKltControlPoints(*Ipyramid[lvl]);
						break;
					}
				} catch (...) {
					vpTRACE("Error in tracking");
					throw;
				}
				double t3 = vpTime::measureTimeMs();
                if(printLog)
                    std::cout << "timetrack " << t3 - t2 << std::endl;
				timetrack = t3-t2;

				try {
					double t4 = vpTime::measureTimeMs();
					switch (trackingType) {
					case POINTS_SH:
						computeVVS(*Ipyramid[lvl]);
						break;
					case POINTS_MH:
						computeVVSMH(*Ipyramid[lvl]);
						break;
					case LINES_MH:
						nbdraw = 1;

						if (lines[lvl].size() != 0)
						computeVVSPointsLinesMH(*Ipyramid[lvl]);
						else computeVVSMH(*Ipyramid[lvl]);

						break;
					case CCD_SH:
						computeVVSCCD(*Ipyramid[lvl], *IRGBpyramid[lvl]);
						break;
					case CCD_MH:
						//if (frame > 1300 && frame < 3100)
						//if (frame > 300 && frame < 1460)
						//computeVVSCCDMHPrevSpace(*Ipyramid[lvl], *IRGBpyramid[lvl]);
						//else
						{
                                                computeVVSCCDMH(*Ipyramid[lvl], *IRGBpyramid[lvl]);
						}
						break;
					case CCD_MH_KLT:
						if (kltPoints[lvl].size() > 2)
						//computeVVSCCDKltMHPrev(*Ipyramid[lvl], *IRGBpyramid[lvl]);
						computeVVSCCDKltMHPrevFAST(*Ipyramid[lvl], *IRGBpyramid[lvl]);
						else computeVVSCCDMHPrev(*Ipyramid[lvl], *IRGBpyramid[lvl]);
						break;
					case CCD_LINES_MH:
						nbdraw = 1;
						//if (frame > 300 && frame < 1460)
						//computeVVSPointsLinesCCDMHPrevSpace(*Ipyramid[lvl], *IRGBpyramid[lvl]);
						//else
						if (lines[lvl].size() != 0)
						{
						computeVVSPointsLinesCCDMHPrev(*Ipyramid[lvl], *IRGBpyramid[lvl]);
						}
						else computeVVSCCDMHPrev(*Ipyramid[lvl], *IRGBpyramid[lvl]);
						break;
					case CCD_LINES_MH_KLT:
						nbdraw = 1;
						//if (frame > 300 && frame < 1460)
						//computeVVSPointsLinesCCDMHPrevSpace(*Ipyramid[lvl], *IRGBpyramid[lvl]);
						//else
						if (lines[lvl].size() != 0 && kltPoints[lvl].size() != 0 )
						{
						computeVVSLinesCCDKltMHPrev(*Ipyramid[lvl],
													*IRGBpyramid[lvl]);
						}
						else if (kltPoints[lvl].size() != 0) computeVVSCCDKltMHPrev(*Ipyramid[lvl], *IRGBpyramid[lvl]);
						else if (lines[lvl].size() != 0) computeVVSPointsLinesCCDMHPrev(*Ipyramid[lvl], *IRGBpyramid[lvl]);
						break;
					}
					double t5 = vpTime::measureTimeMs();
                    if(printLog)
                        std::cout << "timeVVS " << t5 - t4 << std::endl;
					timevvs = t5-t4;
				} catch (...) {
					vpTRACE("Error in computeVVS");
					throw vpException(vpException::fatalError,
							"Error in computeVVS");
				}
				// try
				// {
				// testTracking();
				// }
				// catch(...)
				// {
				// throw vpTrackingException(vpTrackingException::fatalError, "test Tracking fail");
				// }

				try {
					//displayControlPoints();
				} catch (...) {
					vpTRACE("Error in moving edge updating");
					throw;
				}

			} catch (...) {
				if (lvl != 0) {
					cMo = cMo_1;
					reInitLevel(lvl);
					upScale(lvl);
				} else {
					upScale(lvl);
					throw;
				}
			}
		}
	}

	cleanPyramid(Ipyramid);
	switch (trackingType) {
	case CCD_SH:
	case CCD_MH:
	case CCD_LINES_MH:
	case CCD_MH_KLT:
	case CCD_LINES_MH_KLT:
		cleanPyramid(IRGBpyramid);
		CCDTracker.clearCCDTracker();
	}
	cleanPyramid(Ipyramidprec);
	Iprec = I;
	IprecRGB = IRGB;

	frame++;
	frame0++;
}


/*!
 Compute each state of the tracking procedure for all the feature sets.

 If the tracking is considered as failed an exception is thrown.

 \param I : The image.
 */

void apMbTracker::trackDef(const vpImage<unsigned char> &I, const vpImage<
                vpRGBa> &IRGB, const vpImage<vpRGBa> &Inormd, const vpImage<
                unsigned char>& Ior, const vpImage<unsigned char>& Itex,
                const double distmin, const double distmax) {
        initPyramid(I, Ipyramid);
        initPyramid(Iprec, Ipyramidprec);
        switch (trackingType) {
        case CCD_SH:
        case CCD_MH:
        case CCD_LINES_MH:
        case CCD_MH_KLT:
        case CCD_LINES_MH_KLT:
                initPyramid(IRGB, IRGBpyramid);
        }

        double dist = (distmin + distmax)/2;

        for (int lvl = (scales.size() - 1); lvl >= 0; lvl -= 1) {
                if (scales[lvl]) {
                        vpHomogeneousMatrix cMo_1 = cMo;
                        try {
                                downScale(lvl);
                                double t0 = vpTime::measureTimeMs();
                                try {
                                        switch (trackingType) {
                                        case POINTS_SH:
                                        case POINTS_MH:
                                                extractControlPoints(*Ipyramid[lvl], Inormd, Ior, Itex,
                                                                distmin, distmax);
                                                extractKltControlPointsFAST(*Ipyramid[lvl],Inormd, Ior, Itex, dist);
                                                break;
                                        }
                                } catch (...) {
                                        vpTRACE("Error in points extraction");
                                }

                                double t1 = vpTime::measureTimeMs();
                                std::cout << "timeextract " << t1 - t0 << std::endl;
                                timeextract = t1 - t0;
                                double t2 = vpTime::measureTimeMs();
                                try {
                                        switch (trackingType) {
                                        case POINTS_SH:
                                        case CCD_SH:
                                                trackControlPoints(*Ipyramid[lvl]);
                                                break;
                                        case POINTS_MH:
                                        case CCD_MH:
                                                trackControlPointsMH(*Ipyramid[lvl]);
                                                trackKltControlPointsFromSynth(*Ipyramid[lvl],Itex);
                                                break;
                                        case CCD_MH_KLT:
                                                trackControlPointsMH(*Ipyramid[lvl]);
                                                trackKltControlPoints(*Ipyramid[lvl]);
                                                break;
                                        }
                                } catch (...) {
                                        vpTRACE("Error in tracking");
                                        throw;
                                }
                                double t3 = vpTime::measureTimeMs();
                                std::cout << "timetrack " << t3 - t2 << std::endl;
                                timetrack = t3-t2;

                                try {
                                        double t4 = vpTime::measureTimeMs();
                                        switch (trackingType) {
                                        case POINTS_SH:
                                                //exportCorrespondencesEdgesMean(*Ipyramid[lvl]);
#ifdef ENABLE_ZMQ
                                            exportCorrespondencesKLT(*Ipyramid[lvl]);
#endif // ENABLE_ZMQ
                                                break;
                                        case POINTS_MH:
                                                //exportCorrespondencesEdgesMean(*Ipyramid[lvl]);
#ifdef ENABLE_ZMQ
                                                exportCorrespondencesKLT(*Ipyramid[lvl]);
#endif // ENABLE_ZMQ
                                                break;
                                        /*case CCD_SH:
                                                exportCorrespondencesEdgesCCD(*Ipyramid[lvl], *IRGBpyramid[lvl]);
                                                break;
                                        case CCD_MH:
                                                exportCorrespondencesEdgesCCDMH(*Ipyramid[lvl], *IRGBpyramid[lvl]);
                                                break;
                                        case CCD_MH_KLT:
                                                if (kltPoints[lvl].size() > 2)
                                                exportCorrespondencesEdgesCCDMHKlt(*Ipyramid[lvl], *IRGBpyramid[lvl]);
                                                else exportCorrespondencesEdgesCCDMH(*Ipyramid[lvl], *IRGBpyramid[lvl]);
                                            break;*/
                                        }
                                        double t5 = vpTime::measureTimeMs();
                                        std::cout << "timeExportCorrespondences " << t5 - t4 << std::endl;
                                        timevvs = t5-t4;
                                } catch (...) {
                                        vpTRACE("Error in computeVVS");
                                        throw vpException(vpException::fatalError,
                                                        "Error in computeVVS");
                                }
                                // try
                                // {
                                // testTracking();
                                // }
                                // catch(...)
                                // {
                                // throw vpTrackingException(vpTrackingException::fatalError, "test Tracking fail");
                                // }

                                try {
                                        //displayControlPoints();
                                } catch (...) {
                                        vpTRACE("Error in moving edge updating");
                                        throw;
                                }

                        } catch (...) {
                                if (lvl != 0) {
                                        cMo = cMo_1;
                                        reInitLevel(lvl);
                                        upScale(lvl);
                                } else {
                                        upScale(lvl);
                                        throw;
                                }
                        }
                }
        }

        cleanPyramid(Ipyramid);
        switch (trackingType) {
        case CCD_SH:
        case CCD_MH:
        case CCD_LINES_MH:
        case CCD_MH_KLT:
        case CCD_LINES_MH_KLT:
                cleanPyramid(IRGBpyramid);
                CCDTracker.clearCCDTracker();
        }
        cleanPyramid(Ipyramidprec);
        Iprec = I;
        IprecRGB = IRGB;

        frame++;
        frame0++;
}


/*!
 Compute each state of the tracking procedure for all the feature sets.

 If the tracking is considered as failed an exception is thrown.

 \param I : The image.
 */

void apMbTracker::trackDef2D(const vpImage<unsigned char> &I, const vpImage<
                vpRGBa> &IRGB, std::vector<point2d> &positions, std::vector<point2dd> &normals, std::vector<point2d> &trackededges, std::vector<int> &suppress) {
        initPyramid(I, Ipyramid);
        initPyramid(Iprec, Ipyramidprec);
        switch (trackingType) {
        case CCD_SH:
        case CCD_MH:
        case CCD_LINES_MH:
        case CCD_MH_KLT:
        case CCD_LINES_MH_KLT:
                initPyramid(IRGB, IRGBpyramid);
        }

        for (int lvl = (scales.size() - 1); lvl >= 0; lvl -= 1) {
                if (scales[lvl]) {
                        vpHomogeneousMatrix cMo_1 = cMo;
                        try {
                                downScale(lvl);
                                double t0 = vpTime::measureTimeMs();
                                try {
                                        switch (trackingType) {
                                        case POINTS_SH:
                                        case POINTS_MH:
                                                buildControlPoints2D(*Ipyramid[lvl],positions, normals);
                                                break;
                                        }
                                } catch (...) {
                                        vpTRACE("Error in points extraction");
                                }

                                double t1 = vpTime::measureTimeMs();
                                std::cout << "timeextract " << t1 - t0 << std::endl;
                                timeextract = t1 - t0;
                                double t2 = vpTime::measureTimeMs();
                                try {
                                        switch (trackingType) {
                                        case POINTS_SH:
                                        case CCD_SH:
                                                trackControlPoints(*Ipyramid[lvl]);
                                                break;
                                        case POINTS_MH:
                                        case CCD_MH:
                                                trackControlPointsMH(*Ipyramid[lvl]);
                                                break;
                                        case CCD_MH_KLT:
                                                trackControlPointsMH(*Ipyramid[lvl]);
                                                trackKltControlPoints(*Ipyramid[lvl]);
                                                break;
                                        }
                                } catch (...) {
                                        vpTRACE("Error in tracking");
                                        throw;
                                }
                                double t3 = vpTime::measureTimeMs();
                                std::cout << "timetrack " << t3 - t2 << std::endl;
                                timetrack = t3-t2;

                                try {
                                        double t4 = vpTime::measureTimeMs();
                                        switch (trackingType) {
                                        case POINTS_SH:
                                                buildCorrespondencesEdges2D(trackededges, suppress);
                                                break;
                                        case POINTS_MH:
                                                buildCorrespondencesEdges2D(trackededges, suppress);
                                                break;
                                        /*case CCD_SH:
                                                exportCorrespondencesEdgesCCD(*Ipyramid[lvl], *IRGBpyramid[lvl]);
                                                break;
                                        case CCD_MH:
                                                exportCorrespondencesEdgesCCDMH(*Ipyramid[lvl], *IRGBpyramid[lvl]);
                                                break;
                                        case CCD_MH_KLT:
                                                if (kltPoints[lvl].size() > 2)
                                                exportCorrespondencesEdgesCCDMHKlt(*Ipyramid[lvl], *IRGBpyramid[lvl]);
                                                else exportCorrespondencesEdgesCCDMH(*Ipyramid[lvl], *IRGBpyramid[lvl]);
                                            break;*/
                                        }
                                        double t5 = vpTime::measureTimeMs();
                                        std::cout << "timeExportCorrespondences " << t5 - t4 << std::endl;
                                        timevvs = t5-t4;
                                } catch (...) {
                                        vpTRACE("Error in computeVVS");
                                        throw vpException(vpException::fatalError,
                                                        "Error in computeVVS");
                                }
                                // try
                                // {
                                // testTracking();
                                // }
                                // catch(...)
                                // {
                                // throw vpTrackingException(vpTrackingException::fatalError, "test Tracking fail");
                                // }

                                try {
                                        //displayControlPoints();
                                } catch (...) {
                                        vpTRACE("Error in moving edge updating");
                                        throw;
                                }

                        } catch (...) {
                                if (lvl != 0) {
                                        cMo = cMo_1;
                                        reInitLevel(lvl);
                                        upScale(lvl);
                                } else {
                                        upScale(lvl);
                                        throw;
                                }
                        }
                }
        }

        cleanPyramid(Ipyramid);
        switch (trackingType) {
        case CCD_SH:
        case CCD_MH:
        case CCD_LINES_MH:
        case CCD_MH_KLT:
        case CCD_LINES_MH_KLT:
                cleanPyramid(IRGBpyramid);
                CCDTracker.clearCCDTracker();
        }
        cleanPyramid(Ipyramidprec);
        Iprec = I;
        IprecRGB = IRGB;

        frame++;
        frame0++;
}



/*void apMbTracker::track(const vpImage<unsigned char> &I, const vpImage<
		vpRGBa> &IRGB, const vpImage<vpRGBa> &Inormd, const vpImage<
		unsigned char>& Ior, const vpImage<unsigned char>& Itex,
		const double dist) {
	initPyramid(I, Ipyramid);
	initPyramid(Iprec, Ipyramidprec);
	switch (trackingType) {
	case CCD_SH:
	case CCD_MH:
	case CCD_LINES_MH:
	case CCD_MH_KLT:
		initPyramid(IRGB, IRGBpyramid);
	}

	for (int lvl = (scales.size() - 1); lvl >= 0; lvl -= 1) {
		if (scales[lvl]) {
			vpHomogeneousMatrix cMo_1 = cMo;
			try {
				downScale(lvl);
				try {
					double t0 = vpTime::measureTimeMs();
					switch (trackingType) {
					case POINTS_SH:
					case POINTS_MH:
						extractControlPoints(*Ipyramid[lvl], Inormd, Ior, Itex,
								dist);
						break;
					case LINES_MH:
						extractControlPointsLines(*Ipyramid[lvl], Inormd, Ior,
								Itex, dist);
						break;
					case CCD_SH:
					case CCD_MH:
						extractControlPointsCCD(*Ipyramid[lvl], Inormd, Ior,
								Itex, dist);
						break;
					case CCD_MH_KLT:
						extractControlPointsCCD(*Ipyramid[lvl], Inormd, Ior,
								Itex, dist);
						extractKltControlPoints(*Ipyramid[lvl],Inormd, Ior, Itex, dist);
						break;
					case CCD_LINES_MH:
						extractControlPointsLinesCCD(*Ipyramid[lvl], Inormd,
								Ior, Itex, dist);
						break;
					}
					double t1 = vpTime::measureTimeMs();
					std::cout << "timeextract " << t1 - t0 << std::endl;
				} catch (...) {
					vpTRACE("Error in points extraction");
				}
				double t2 = vpTime::measureTimeMs();
				try {
					switch (trackingType) {
					case POINTS_SH:
					case CCD_SH:
						trackControlPoints(*Ipyramid[lvl]);
						break;
					case POINTS_MH:
					case CCD_MH:
						trackControlPointsMH(*Ipyramid[lvl]);
						break;
					case CCD_MH_KLT:
						trackControlPointsMH(*Ipyramid[lvl]);
						trackKltControlPoints(*Ipyramid[lvl]);
					case LINES_MH:
					case CCD_LINES_MH:
						trackControlPointsMH(*Ipyramid[lvl]);
						trackControlPointsLinesMH(*Ipyramid[lvl]);
						break;
					}
				} catch (...) {
					vpTRACE("Error in tracking");
					throw;
				}
				double t3 = vpTime::measureTimeMs();
				std::cout << "timetrack " << t3 - t2 << std::endl;

				try {
					double t4 = vpTime::measureTimeMs();

					computeVVSAll(*Ipyramid[lvl], *IRGBpyramid[lvl]);
					switch (trackingType) {
					case POINTS_SH:
						computeVVS(*Ipyramid[lvl]);
						break;
					case POINTS_MH:
						computeVVSMH(*Ipyramid[lvl]);
						break;
					case LINES_MH:
						nbdraw = 1;

						if (lines[lvl].size() != 0)
						computeVVSPointsLinesMH(*Ipyramid[lvl]);
						else computeVVSMH(*Ipyramid[lvl]);

						break;
					case CCD_SH:
						computeVVSCCD(*Ipyramid[lvl], *IRGBpyramid[lvl]);
						break;
					case CCD_MH:
						//if (frame > 1300 && frame < 3100)
						//if (frame > 300 && frame < 1460)
						//computeVVSCCDMHPrevSpace(*Ipyramid[lvl], *IRGBpyramid[lvl]);
						//else
						{
						computeVVSCCDMHPrev();
						}
						break;
					case CCD_MH_KLT:
						computeVVSCCDKltMHPrev(*Ipyramid[lvl], *IRGBpyramid[lvl]);
						break;
					case CCD_LINES_MH:
						nbdraw = 1;
						//if (frame > 300 && frame < 1460)
						//computeVVSPointsLinesCCDMHPrevSpace(*Ipyramid[lvl], *IRGBpyramid[lvl]);
						//else
						{
						computeVVSPointsLinesCCDMHPrev(*Ipyramid[lvl],
								*IRGBpyramid[lvl]);
						}
						break;
					}
					double t5 = vpTime::measureTimeMs();
					std::cout << "timeVVS " << t5 - t4 << std::endl;
				} catch (...) {
					vpTRACE("Error in computeVVS");
					throw vpException(vpException::fatalError,
							"Error in computeVVS");
				}


				try {
					//displayControlPoints();
				} catch (...) {
					vpTRACE("Error in moving edge updating");
					throw;
				}

			} catch (...) {
				if (lvl != 0) {
					cMo = cMo_1;
					reInitLevel(lvl);
					upScale(lvl);
				} else {
					upScale(lvl);
					throw;
				}
			}
		}
	}

	cleanPyramid(Ipyramid);
	switch (trackingType) {
	case CCD_SH:
	case CCD_MH:
	case CCD_LINES_MH:
	case CCD_MH_KLT:
		cleanPyramid(IRGBpyramid);
		CCDTracker.clearCCDTracker();
	}
	cleanPyramid(Ipyramidprec);
	Iprec = I;
	IprecRGB = IRGB;

	frame++;
	frame0++;
}*/


/*!
 Compute each state of the tracking procedure for all the feature sets, in case of the multiple hypothesis solution.

 If the tracking is considered as failed an exception is thrown.

 \param I : The image.
 */

/*!
 Compute each state of the tracking procedure for all the feature sets, in case of dense tracking alon the edges normals.

 If the tracking is considered as failed an exception is thrown.

 \param I : The image.
 */
void apMbTracker::track(const vpImage<unsigned char> &I, const vpImage<
		double> &Igrad, const vpImage<double> &Igradx,
		const vpImage<double> & Igrady, const vpImage<vpRGBa> &Inormd,
		const vpImage<unsigned char>& Ior, const vpImage<unsigned char>& Itex,
		const double dist) {
	initPyramid(I, Ipyramid);
	initPyramid(Iprec, Ipyramidprec);

	for (int lvl = (scales.size() - 1); lvl >= 0; lvl -= 1) {
		if (scales[lvl]) {
			vpHomogeneousMatrix cMo_1 = cMo;
			try {
				downScale(lvl);
				try {
					double t0 = vpTime::measureTimeMs();
					extractControlPointsP(*Ipyramid[lvl], Inormd, Ior, Itex,
							dist);
					double t1 = vpTime::measureTimeMs();
					std::cout << "timeextract " << t1 - t0 << std::endl;
				} catch (...) {
					vpTRACE("Error in points extraction");
				}
				double t2 = vpTime::measureTimeMs();
				trackControlPoints(*Ipyramid[lvl]);
				double t3 = vpTime::measureTimeMs();
				std::cout << "timetrack " << t3 - t2 << std::endl;

				try {
					double t4 = vpTime::measureTimeMs();
					//computeVVSCorr(I, Igrad, Igradx, Igrady);
					computeVVSHybrid(*Ipyramid[lvl], Igrad, Igradx, Igrady);
					//computeVVS(*Ipyramid[lvl]);
					double t5 = vpTime::measureTimeMs();
					std::cout << "timeVVS " << t5 - t4 << std::endl;
				} catch (...) {
					vpTRACE("Error in computeVVS");
					throw vpException(vpException::fatalError,
							"Error in computeVVS");
				}

				try {
					//displayControlPoints();
				} catch (...) {
					vpTRACE("Error in moving edge updating");
					throw;
				}

			} catch (...) {
				if (lvl != 0) {
					cMo = cMo_1;
					reInitLevel(lvl);
					upScale(lvl);
				} else {
					upScale(lvl);
					throw;
				}
			}
		}
	}

	cleanPyramid(Ipyramid);
	cleanPyramid(Ipyramidprec);
	Iprec = I;
}

void apMbTracker::trackXrayIntensityContour(const vpImage<unsigned char> &I, const vpImage<
                vpRGBa> &IRGB, const vpImage<vpRGBa> &Inormd, const vpImage<
                unsigned char>& Ior, const vpImage<unsigned char>& Itex,
                const double dist) {
        initPyramid(I, Ipyramid);
        initPyramid(Iprec, Ipyramidprec);
        switch (trackingType) {
        case CCD_SH:
        case CCD_MH:
        case CCD_LINES_MH:
        case CCD_MH_KLT:
        case CCD_LINES_MH_KLT:
                initPyramid(IRGB, IRGBpyramid);
        }

        for (int lvl = (scales.size() - 1); lvl >= 0; lvl -= 1) {
                if (scales[lvl]) {
                        vpHomogeneousMatrix cMo_1 = cMo;
                        try {
                                downScale(lvl);
                                double t0 = vpTime::measureTimeMs();
                                try {
                                        switch (trackingType) {
                                        case POINTS_SH:
                                        case POINTS_MH:
                                                extractControlPoints(*Ipyramid[lvl], Inormd, Ior, Itex,
                                                                dist);
                                                break;
                                        case LINES_MH:
                                                extractControlPointsLines(*Ipyramid[lvl], Inormd, Ior,
                                                                Itex, dist);
                                                break;
                                        case CCD_SH:
                                        case CCD_MH:
                                                extractControlPointsCCD(*Ipyramid[lvl], Inormd, Ior,
                                                                Itex, dist);
                                                break;
                                        case CCD_MH_KLT:
                                                extractControlPointsCCD(*Ipyramid[lvl], Inormd, Ior,
                                                                Itex, dist);
                                                extractKltControlPointsFAST(*Ipyramid[lvl],Inormd, Ior, Itex, dist);
                                                break;
                                        case CCD_LINES_MH:
                                                extractControlPointsLinesCCD(*Ipyramid[lvl], Inormd,
                                                                Ior, Itex, dist);
                                                break;
                                        case CCD_LINES_MH_KLT:
                                                extractControlPointsLinesCCD(*Ipyramid[lvl], Inormd, Ior,
                                                                Itex, dist);
                                                extractKltControlPoints(*Ipyramid[lvl],Inormd, Ior, Itex, dist);
                                                break;
                                        }
                                } catch (...) {
                                        vpTRACE("Error in points extraction");
                                }

                                double t1 = vpTime::measureTimeMs();
                                std::cout << "timeextract " << t1 - t0 << std::endl;
                                timeextract = t1 - t0;
                                double t2 = vpTime::measureTimeMs();
                                try {
                                        switch (trackingType) {
                                        case POINTS_SH:
                                        case CCD_SH:
                                                trackControlPoints(*Ipyramid[lvl]);
                                                break;
                                        case POINTS_MH:
                                        case CCD_MH:
                                                trackControlPointsMH(*Ipyramid[lvl]);
                                                break;
                                        case CCD_MH_KLT:
                                                trackControlPointsMH(*Ipyramid[lvl]);
                                                trackKltControlPoints(*Ipyramid[lvl]);
                                                break;
                                        case LINES_MH:
                                        case CCD_LINES_MH:
                                                trackControlPointsMH(*Ipyramid[lvl]);
                                                trackControlPointsLinesMH(*Ipyramid[lvl]);
                                                break;
                                        case CCD_LINES_MH_KLT:
                                                trackControlPointsMH(*Ipyramid[lvl]);
                                                trackControlPointsLinesMH(*Ipyramid[lvl]);
                                                trackKltControlPoints(*Ipyramid[lvl]);
                                                break;
                                        }
                                } catch (...) {
                                        vpTRACE("Error in tracking");
                                        throw;
                                }
                                double t3 = vpTime::measureTimeMs();
                                std::cout << "timetrack " << t3 - t2 << std::endl;
                                timetrack = t3-t2;

                                try {
                                        double t4 = vpTime::measureTimeMs();
                                        switch (trackingType) {
                                        case POINTS_SH:
                                                computeVVS(*Ipyramid[lvl]);
                                                break;
                                        case POINTS_MH:
                                                computeVVSMH(*Ipyramid[lvl]);
                                                break;
                                        case LINES_MH:
                                                nbdraw = 1;

                                                if (lines[lvl].size() != 0)
                                                computeVVSPointsLinesMH(*Ipyramid[lvl]);
                                                else computeVVSMH(*Ipyramid[lvl]);

                                                break;
                                        case CCD_SH:
                                                computeVVSCCD(*Ipyramid[lvl], *IRGBpyramid[lvl]);
                                                break;
                                        case CCD_MH:
                                                //if (frame > 1300 && frame < 3100)
                                                //if (frame > 300 && frame < 1460)
                                               // computeVVSCCDMHPrevSpace(*Ipyramid[lvl], *IRGBpyramid[lvl]);
                                                //computeVVSCCDMH(*Ipyramid[lvl], *IRGBpyramid[lvl]);
                                                //else
#ifdef ENABLE_ZMQ
                                                {
                                                computeVVSCCDMHPhotometric(*Ipyramid[lvl], *IRGBpyramid[lvl]);
                                                }
#endif // ENABLE_ZMQ
                                                break;
                                        case CCD_MH_KLT:
                                                if (kltPoints[lvl].size() > 2)
                                                //computeVVSCCDKltMHPrev(*Ipyramid[lvl], *IRGBpyramid[lvl]);
                                                computeVVSCCDKltMHPrevFAST(*Ipyramid[lvl], *IRGBpyramid[lvl]);
                                                else computeVVSCCDMHPrev(*Ipyramid[lvl], *IRGBpyramid[lvl]);
                                                break;
                                        case CCD_LINES_MH:
                                                nbdraw = 1;
                                                //if (frame > 300 && frame < 1460)
                                                //computeVVSPointsLinesCCDMHPrevSpace(*Ipyramid[lvl], *IRGBpyramid[lvl]);
                                                //else
                                                if (lines[lvl].size() != 0)
                                                {
                                                computeVVSPointsLinesCCDMHPrev(*Ipyramid[lvl], *IRGBpyramid[lvl]);
                                                }
                                                else computeVVSCCDMHPrev(*Ipyramid[lvl], *IRGBpyramid[lvl]);
                                                break;
                                        case CCD_LINES_MH_KLT:
                                                nbdraw = 1;
                                                //if (frame > 300 && frame < 1460)
                                                //computeVVSPointsLinesCCDMHPrevSpace(*Ipyramid[lvl], *IRGBpyramid[lvl]);
                                                //else
                                                if (lines[lvl].size() != 0 && kltPoints[lvl].size() != 0 )
                                                {
                                                computeVVSLinesCCDKltMHPrev(*Ipyramid[lvl],
                                                                                                        *IRGBpyramid[lvl]);
                                                }
                                                else if (kltPoints[lvl].size() != 0) computeVVSCCDKltMHPrev(*Ipyramid[lvl], *IRGBpyramid[lvl]);
                                                else if (lines[lvl].size() != 0) computeVVSPointsLinesCCDMHPrev(*Ipyramid[lvl], *IRGBpyramid[lvl]);
                                                break;
                                        }
                                        double t5 = vpTime::measureTimeMs();
                                        std::cout << "timeVVS " << t5 - t4 << std::endl;
                                        timevvs = t5-t4;
                                } catch (...) {
                                        vpTRACE("Error in computeVVS");
                                        throw vpException(vpException::fatalError,
                                                        "Error in computeVVS");
                                }
                                // try
                                // {
                                // testTracking();
                                // }
                                // catch(...)
                                // {
                                // throw vpTrackingException(vpTrackingException::fatalError, "test Tracking fail");
                                // }

                                try {
                                        //displayControlPoints();
                                } catch (...) {
                                        vpTRACE("Error in moving edge updating");
                                        throw;
                                }

                        } catch (...) {
                                if (lvl != 0) {
                                        cMo = cMo_1;
                                        reInitLevel(lvl);
                                        upScale(lvl);
                                } else {
                                        upScale(lvl);
                                        throw;
                                }
                        }
                }
        }

        cleanPyramid(Ipyramid);
        switch (trackingType) {
        case CCD_SH:
        case CCD_MH:
        case CCD_LINES_MH:
        case CCD_MH_KLT:
        case CCD_LINES_MH_KLT:
                cleanPyramid(IRGBpyramid);
                CCDTracker.clearCCDTracker();
        }
        cleanPyramid(Ipyramidprec);
        Iprec = I;
        IprecRGB = IRGB;

        frame++;
        frame0++;
}


/*!
 Compute each state of the tracking procedure for all the feature sets for the prediction phase.

 If the tracking is considered as failed an exception is thrown.

 \param I : The image.
 */
void apMbTracker::trackPred(const vpImage<unsigned char> &I) {
	initPyramid(I, Ipyramid);
	//initPyramid(Iprec, Ipyramidprec);

	for (int lvl = (scales.size() - 1); lvl >= 0; lvl -= 1) {
		if (scales[lvl]) {
			vpHomogeneousMatrix cMo_1 = cMo;
			try {
				downScale(lvl);
				double t2 = vpTime::measureTimeMs();
				try {
					trackControlPointsPred(*Ipyramid[lvl]);
				} catch (...) {
					vpTRACE("Error in moving edge tracking");
					throw;
				}
				double t3 = vpTime::measureTimeMs();
				std::cout << "timetrack " << t3 - t2 << std::endl;

				/*apControlPoint *p ;
				 points[lvl].front() ;
				 vpFeatureLine fli;
				 while (!points[lvl].outside()){
				 p = points[lvl].value() ;
				 //p->initInteractionMatrixError();
				 //fli=p->getFeatureLine();
				 //fli.display(cam,I,vpColor::yellow,1);
				 //fli=p->getFeatureLine();
				 //fli.print();
				 //std::cout<<fli.getTheta()<<std::endl;

				 points[lvl].next() ;
				 }*/
				try {
					computeVVS(*Ipyramid[lvl]);
				} catch (...) {
					vpTRACE("Error in computeVVS");
					throw vpException(vpException::fatalError,
							"Error in computeVVS");
				}
				/* try
				 {
				 testTracking();
				 }
				 catch(...)
				 {
				 throw vpTrackingException(vpTrackingException::fatalError, "test Tracking fail");
				 }*/

				try {
					//displayControlPoints();
				} catch (...) {
					vpTRACE("Error in moving edge updating");
					throw;
				}
			} catch (...) {
				if (lvl != 0) {
					cMo = cMo_1;
					reInitLevel(lvl);
					upScale(lvl);
				} else {
					upScale(lvl);
					throw;
				}
			}
		}
	}

	cleanPyramid(Ipyramid);
	//cleanPyramid(Ipyramidprec);
	//Iprec=I;
}

/*!
 Initialize the tracking thanks to the initial pose of the camera.

 \param I : The image.
 \param _cMo : The initial pose used to initialize the tracking.
 */
void apMbTracker::init(const vpImage<unsigned char>& I,
		const vpHomogeneousMatrix &_cMo) {
	this->cMo = _cMo;

	sId.init(I.getHeight(), I.getWidth(), 1);
	sI.init(I.getHeight(), I.getWidth(), 1);

        vpCameraParameters camCt(cam.get_px(), cam.get_py(), cam.get_u0(), cam.get_v0() );


        sId.setCameraParameters(cam);
        sI.setCameraParameters(cam);

	/*apControlPoint *p;
	 vpColVector norm(3);
	 norm[0]=1;
	 norm[1]=1;
	 norm[2]=1;
	 double a=1;

	 for (unsigned int i = 0; i < points_1.size(); i += 1){
	 if(scales[i]){
	 apControlPoint *p ;
	 p=new apControlPoint;
	 p->setCameraParameters(&cam);
	 p->setMovingEdge(&me);
	 p->buildPoint(0,0,a,0.0,norm,cMo);
	 points_1[i]+=p;
	 }
     }*/
	//bool a = false;
	//initPyramid(I, Ipyramid);
	Iprec = I;

}


void apMbTracker::initKltTracker(const vpImage<unsigned char>& I)
{
	std::cout << " init " << std::endl;
	if(trackingType == apTrackingType::CCD_MH_KLT || trackingType == apTrackingType::CCD_LINES_MH_KLT)
	{

cv::Mat frame_;
vpImageConvert::convert(I,frame_);
nkltPointsO = 0;
kltTracker.setTrackerId(1);//1
kltTracker.setMaxFeatures(KLTTrackerParameters.npoints);		//nbPoint
kltTracker.setWindowSize(KLTTrackerParameters.windowsize);				//6//10
kltTracker.setQuality(KLTTrackerParameters.quality);				//0.00000001//0.01
kltTracker.setMinDistance(KLTTrackerParameters.mindist);				//10//15
kltTracker.setHarrisFreeParameter(KLTTrackerParameters.harrisfree);	//0.1//0.04
kltTracker.setBlockSize(KLTTrackerParameters.blocksize);				//9
kltTracker.setUseHarris(KLTTrackerParameters.useharris);				//1
kltTracker.setPyramidLevels(KLTTrackerParameters.pyramidlevels);			//3
kltTracker.initTracking(frame_);
}
}

void apMbTracker::displayKltPoints(const vpImage<unsigned char>& I)
{
std::map<int, apKltControlPoint>::const_iterator iter = kltPoints[scaleLevel].begin();
for( ; iter != kltPoints[scaleLevel].end(); iter++){
     int id(iter->first);
     vpImagePoint iP;
     iP.set_i(static_cast<double>(iter->second.icpoint.get_i()));
     iP.set_j(static_cast<double>(iter->second.icpoint.get_j()));
     //vpDisplay::displayCross(I, iP, 10, vpColor::red);
     iP.set_i( vpMath::round( iP.get_i() + 7 ) );
     iP.set_j( vpMath::round( iP.get_j() + 7 ) );
     char ide[10];
     sprintf(ide, "%ld", static_cast<long int>(id));
     //vpDisplay::displayCharString(I, iP, ide, vpColor::red);
   }
}

/*cleanPyramid(Ipyramid);
 }*/

/*!
 Load the xml configuration file.
 Write the parameters in the corresponding objects (Ecm, camera, 3D rendering).

 \param _filename : full name of the xml file.
 */
void apMbTracker::loadConfigFile(const std::string& _filename) {
	loadConfigFile(_filename.c_str());
}

/*!
 Load the xml configuration file.
 Write the parameters in the corresponding objects (Ecm, camera, 3D rendering).
 
 \throw vpException::ioError if the file has not been properly parsed (file not
 found or wrong format for the data).

 \param filename : full name of the xml file.
 */
void apMbTracker::loadConfigFile(const char* filename) {
	LuaConfig cfg(filename);
	vpCameraParameters camera(cfg.getAsNumber("conf.camera.px"),
			cfg.getAsNumber("conf.camera.py"),
			cfg.getAsNumber("conf.camera.u0"),
			cfg.getAsNumber("conf.camera.v0"));
	vpMe meParser;
	meParser.sample_step = cfg.getAsNumber("conf.sample.step");
	meParser.ntotal_sample = cfg.getAsNumber("conf.sample.nb_sample");
	meParser.setMaskSize(cfg.getAsNumber("conf.ecm.mask.size"));
	meParser.setMaskNumber(cfg.getAsNumber("conf.ecm.mask.nb_mask"));
	meParser.range = cfg.getAsNumber("conf.ecm.range.tracking");
	meParser.mu1 = cfg.getAsNumber("conf.ecm.contrast.mu1");
	meParser.mu2 = cfg.getAsNumber("conf.ecm.contrast.mu2");
	meParser.threshold = cfg.getAsNumber("conf.ecm.contrast.edge_threshold");
	apRend rend;
	rend.edgeR_th = cfg.getAsNumber("conf.rendering.edgeRend_threshold");
	rend.clipDist = cfg.getAsNumber("conf.rendering.clipDist");
	rend.sampleR = cfg.getAsNumber("conf.rendering.sampleRend");
	rend.scaleModel = cfg.getAsNumber("conf.rendering.scaleModel");
	if(cfg.getAsNumber("conf.rendering.useNPoints")==1)
	rend.useNPoints = true;
	else rend.useNPoints = false;
	rend.nPoints = cfg.getAsNumber("conf.rendering.nPoints");
	rend.Normx = cfg.getAsNumber("conf.rendering.xDir");
	rend.Normy = cfg.getAsNumber("conf.rendering.yDir");
	rend.Normz = cfg.getAsNumber("conf.rendering.zDir");

	int tracktype = cfg.getAsNumber("conf.trackingtype");
	switch (tracktype) {
	case 0:
		trackingType = POINTS_SH;
		break;
	case 1:
		trackingType = POINTS_MH;
		break;
	case 2:
		trackingType = LINES_MH;
		break;
	case 3:
		trackingType = CCD_SH;
		useRGB = true;
		break;
	case 4:
		trackingType = CCD_MH;
		useRGB = true;
		break;
	case 5:
		trackingType = CCD_LINES_MH;
		useRGB = true;
		break;
	case 6:
		trackingType = CCD_MH_KLT;
		useRGB = true;
		break;
	case 7:
		trackingType = CCD_LINES_MH_KLT;
		useRGB = true;
	}

	if (cfg.getAsNumber("conf.computecovariance") == 1)
		computeCovariance = true;
	else
		computeCovariance = false;

	if (cfg.getAsNumber("conf.usekalman") == 1) {
		useKalman = true;
		computeCovariance = true;
	} else {
		useKalman = false;
	}

	apKalmanParameters kalmanParam;
	kalmanParam.sigmaPT = cfg.getAsNumber("conf.kalman.sigmapt");
	kalmanParam.sigmaPR = cfg.getAsNumber("conf.kalman.sigmapr");
	kalmanParam.sigmaQT = cfg.getAsNumber("conf.kalman.sigmaqt");
	kalmanParam.sigmaQR = cfg.getAsNumber("conf.kalman.sigmaqr");

	apCCDParameters ccdParams;
	ccdParams.gamma_1 = cfg.getAsNumber("conf.ccd.gamma_1");
	ccdParams.gamma_2 = cfg.getAsNumber("conf.ccd.gamma_2");
	ccdParams.gamma_3 = cfg.getAsNumber("conf.ccd.gamma_3");
	ccdParams.gamma_4 = cfg.getAsNumber("conf.ccd.gamma_4");
	ccdParams.alpha = cfg.getAsNumber("conf.ccd.alpha");
	ccdParams.beta = cfg.getAsNumber("conf.ccd.beta");
	ccdParams.kappa = cfg.getAsNumber("conf.ccd.kappa");
	ccdParams.c = cfg.getAsNumber("conf.ccd.c");
	ccdParams.h = cfg.getAsNumber("conf.ccd.h");
	ccdParams.delta_h = cfg.getAsNumber("conf.ccd.delta_h");
	ccdParams.resolution = cfg.getAsNumber("conf.ccd.resolution");
	ccdParams.degree = cfg.getAsNumber("conf.ccd.degree");
	ccdParams.phi_dim = cfg.getAsNumber("conf.ccd.phi_dim");
        ccdParams.fixedrotationx = cfg.getAsNumber("conf.fixedrotationx");
        ccdParams.fixedrotationy = cfg.getAsNumber("conf.fixedrotationy");
        ccdParams.fixedrotationz = cfg.getAsNumber("conf.fixedrotationz");

	apKLTTrackerParameters kltParams;
	kltParams.blocksize = cfg.getAsNumber("conf.klt.blocksize");
	kltParams.npoints = cfg.getAsNumber("conf.klt.npoints");
	kltParams.harrisfree = cfg.getAsNumber("conf.klt.harrisfree");
	kltParams.quality = cfg.getAsNumber("conf.klt.quality");
	kltParams.mindist = cfg.getAsNumber("conf.klt.mindist");
	kltParams.pyramidlevels = cfg.getAsNumber("conf.klt.pyramidlevels");
	kltParams.windowsize = cfg.getAsNumber("conf.klt.windowsize");
	kltParams.useharris = cfg.getAsNumber("conf.klt.useharris");
	kltParams.history = cfg.getAsNumber("conf.klt.history");

	apDetection detection;
	if (cfg.getAsNumber("conf.detection.similarity") == 0)
		detection.similarityMs = apDetection::ORIENTED_CHAMFER;
	else if (cfg.getAsNumber("conf.detection.similarity") == 1)
		detection.similarityMs = apDetection::SHAPE_CONTEXT;
	else if (cfg.getAsNumber("conf.detection.similarity") == 2)
			detection.similarityMs = apDetection::STEGER;
	detection.nbimax = cfg.getAsNumber("conf.detection.nbimax");
	detection.startingLevel = cfg.getAsNumber("conf.detection.startinglevel");
	detection.nscales = cfg.getAsNumber("conf.detection.nscales");
	detection.sample = cfg.getAsNumber("conf.detection.sample");
	detection.nbParticles = cfg.getAsNumber(
			"conf.detection.pfilter.nbparticles");
	detection.lambda = cfg.getAsNumber("conf.detection.pfilter.lambda");
	detection.nr = cfg.getAsNumber("conf.detection.shapecontext.nradius");
	detection.nw = cfg.getAsNumber("conf.detection.shapecontext.ntheta");
	detection.sx = cfg.getAsNumber("conf.detection.shapecontext.samplex");
	detection.sy = cfg.getAsNumber("conf.detection.shapecontext.sampley");
	detection.stheta = cfg.getAsNumber(
			"conf.detection.shapecontext.sampletheta");
	detection.cannyTh1 = cfg.getAsNumber("conf.detection.canny.cannyTh1");
	detection.cannyTh2 = cfg.getAsNumber("conf.detection.canny.cannyTh2");
	detection.mud = cfg.getAsNumber("conf.detection.orientedchamfer.mud");
	detection.lambdao = cfg.getAsNumber(
			"conf.detection.orientedchamfer.lambdao");
	detection.sigmaf = cfg.getAsNumber("conf.detection.bayesian.sigmaf");

	apLearn learning;
	if (cfg.getAsNumber("conf.learn.similarityL") == 0)
		learning.similarityMS = apDetection::ORIENTED_CHAMFER;
	else if (cfg.getAsNumber("conf.learn.similarityL") == 1)
		learning.similarityMS = apDetection::SHAPE_CONTEXT;
	learning.sampleViewsRho = cfg.getAsNumber("conf.learn.srho");
	learning.sampleViewsTheta = cfg.getAsNumber("conf.learn.stheta");
	learning.sampleViewsPhi = cfg.getAsNumber("conf.learn.sphi");
	learning.sampleRViewsTheta = cfg.getAsNumber("conf.learn.srtheta");
	learning.sampleRViewsPhi = cfg.getAsNumber("conf.learn.srphi");
	learning.nOverlap = cfg.getAsNumber("conf.learn.overlap");
	learning.dist = cfg.getAsNumber("conf.learn.dist");

	apSegmentationParameters segmentation;
	segmentation.KLTParams.block_size = cfg.getAsNumber(
			"conf.segmentation.klt.blocksize");
	segmentation.KLTParams.grid_bg = cfg.getAsNumber(
			"conf.segmentation.klt.gridbg");
	segmentation.KLTParams.grid_fg = cfg.getAsNumber(
			"conf.segmentation.klt.gridfg");
	segmentation.KLTParams.harris_free = cfg.getAsNumber(
			"conf.segmentation.klt.harrisfree");
	segmentation.KLTParams.history = cfg.getAsNumber(
			"conf.segmentation.klt.history");
	segmentation.KLTParams.level_pyr = cfg.getAsNumber(
			"conf.segmentation.klt.pyramidlevels");
	segmentation.KLTParams.min_dist = cfg.getAsNumber(
			"conf.segmentation.klt.mindist");
	segmentation.KLTParams.nbPoints = cfg.getAsNumber(
			"conf.segmentation.klt.npoints");
	segmentation.KLTParams.quality = cfg.getAsNumber(
			"conf.segmentation.klt.quality");
	segmentation.KLTParams.use_Harris = cfg.getAsNumber(
			"conf.segmentation.klt.useharris");
	segmentation.KLTParams.window_size = cfg.getAsNumber(
			"conf.segmentation.klt.windowsize");
	segmentation.energyParams.alpha_0 = cfg.getAsNumber(
			"conf.segmentation.energy.alpha0");
	segmentation.energyParams.beta_0 = cfg.getAsNumber(
			"conf.segmentation.energy.beta0");
	segmentation.energyParams.gamma_0 = cfg.getAsNumber(
			"conf.segmentation.energy.gamma0");
	segmentation.energyParams.alpha = cfg.getAsNumber(
			"conf.segmentation.energy.alpha");
	segmentation.energyParams.beta = cfg.getAsNumber(
			"conf.segmentation.energy.beta");
	segmentation.energyParams.gamma = cfg.getAsNumber(
			"conf.segmentation.energy.gamma");
	segmentation.kernelParams.bwidth_col_0 = cfg.getAsNumber(
			"conf.segmentation.kernel.bwidthcol0");
	segmentation.kernelParams.bwidth_col_1 = cfg.getAsNumber(
			"conf.segmentation.kernel.bwidthcol1");
	segmentation.kernelParams.bwidth_spat = cfg.getAsNumber(
			"conf.segmentation.kernel.bwidthspat");
	segmentation.RANSACParams.backgroundthresh = cfg.getAsNumber(
			"conf.segmentation.ransac.backgroundthresh");
	segmentation.RANSACParams.consensus = cfg.getAsNumber(
			"conf.segmentation.ransac.consensus");
	segmentation.RANSACParams.minimalsizemodel = cfg.getAsNumber(
			"conf.segmentation.ransac.minimalsizemodel");
	segmentation.RANSACParams.nmaxiterations = cfg.getAsNumber(
			"conf.segmentation.ransac.nmaxiterations");
	segmentation.RANSACParams.pointstobuildP = cfg.getAsNumber(
			"conf.segmentation.ransac.pointstobuildp");

	int bgdtype = cfg.getAsNumber("conf.segmentation.backgroundtype");
		switch (bgdtype) {
		case 0:
			segmentation.bType = apSegmentationParameters::EARTH;
			break;
		case 1:
			segmentation.bType = apSegmentationParameters::DEEPSPACE;
			break;
		case 2:
			segmentation.bType = apSegmentationParameters::LIMB;
		}

	segmentation.startFrame = cfg.getAsNumber("conf.segmentation.startframe");
	segmentation.nGaussians = cfg.getAsNumber("conf.segmentation.ngaussians");
	segmentation.deltaHomography = cfg.getAsNumber(
			"conf.segmentation.deltahomography");
	segmentation.nbins = cfg.getAsNumber("conf.segmentation.nbins");
	weight_me =  cfg.getAsNumber("conf.weightme");
	weight_ccd = cfg.getAsNumber("conf.weightccd");
	weight_klt = cfg.getAsNumber("conf.weightklt");

        fixedrotationx = cfg.getAsNumber("conf.fixedrotationx");
        fixedrotationy = cfg.getAsNumber("conf.fixedrotationy");
        fixedrotationz = cfg.getAsNumber("conf.fixedrotationz");

	setCameraParameters(camera);
	setMovingEdge(meParser);
	setRendParameters(rend);
	setDetectionParameters(detection);
	setLearningParameters(learning);
	setKalmanParameters(kalmanParam);
	setCCDParameters(ccdParams);
	setKLTTrackerParameters(kltParams);
	setSegmentationParameters(segmentation);
}

/*!
 Display the 3D model from a given position of the camera.

 \param I : The image.
 \param _cMo : Pose used to project the 3D model into the image.
 \param cam : The camera parameters.
 \param col : The desired color.
 \param thickness : The thickness of the points.
 */
void apMbTracker::display(const vpImage<unsigned char>& I,
		const vpHomogeneousMatrix &_cMo, const vpCameraParameters &cam,
		const vpColor& col, const unsigned int thickness) {
	apControlPoint *p;
	for (unsigned int i = 0; i < scales.size(); i += 1) {
		if (scales[i]) {
			std::cout << " size p " << points[i].size() << std::endl;
			for (int k = 0; k < points[i].size(); k++) {
				p = (points[i])[k];
				//p->display(I, vpColor::blue, thickness) ;
				p->update(_cMo);
				p->display(I, col, thickness);
                                vpImagePoint ip;
                                vpPointSite site = p->s;
                                ip.set_i(site.i);
                                ip.set_j(site.j);
                            vpDisplay::displayCross(I, ip, 3, vpColor::blue);
			}
			break; //displaying model on one clase only
		}
	}
}

/*!
 Display the 3D model from a given position of the camera.

 \param I : The image.
 \param _cMo : Pose used to project the 3D model into the image.
 \param cam : The camera parameters.
 \param col : The desired color.
 \param thickness : The thickness of the points.
 */
void apMbTracker::display(const vpImage<vpRGBa>& I,
		const vpHomogeneousMatrix &_cMo, const vpCameraParameters &cam,
		const vpColor& col, const unsigned int thickness) {
	apControlPoint *p;

	for (unsigned int i = 0; i < scales.size(); i += 1) {
		if (scales[i]) {
			for (int k = 0; k < points[i].size(); k++) {
				p = (points[i])[k];
				p->display(I, col, thickness);
			}
			break; //displaying model on one clase only
		}
	}
}

void apMbTracker::displayRend(const vpImage<vpRGBa>& I, const vpImage<
		vpRGBa>& Inormd, const vpImage<unsigned char> &Ior, const vpColor& col,
		const unsigned int thickness) {
        vpImagePoint p0;

	/*for (unsigned int i = 0; i < scales.size(); i += 1) {
	 if (scales[i]) {*/
	for (int k = 0; k < Ior.getHeight(); k++)
		for (int l = 0; l < Ior.getWidth(); l++) {
			if (Ior[k][l] != 100 && Inormd[k][l].A > 0) {
				p0.set_i(k);
				p0.set_j(l);
				vpDisplay::displayCross(I, p0, 2, col, thickness);
			}
                }
	/*break; //displaying model on one clase only
	 }
	 }*/

	float x, y;
	int id=0;
	/*for( ; iter != kltPoints[scaleLevel].end(); iter++){
	     int id(iter->first);
	     kltTracker.getFeature((int)i, id, x, y);
	           kltPoints[scaleLevel][id].icpoint_curr = vpImagePoint(static_cast<double>(y),static_cast<double>(x));
	}*/
         /*if (kltPoints[scaleLevel].size()>2)
	   for (unsigned int i = 0; i < kltPoints[scaleLevel].size(); i++){
             {
	    	 vpFeaturePoint featurepoint;

	         if(kltPoints[scaleLevel][i].cpoint.oP[0] !=0)
	         {
	    	 kltPoints[scaleLevel][i].cpointo.changeFrame(cMo);
	    	 kltPoints[scaleLevel][i].cpointo.projection();

	    	 vpFeatureBuilder::create(featurepoint,kltPoints[scaleLevel][i].cpointo);

	    	 double x0 = featurepoint.get_x();
	    	 double y0 = featurepoint.get_y();

	    	 double mx = 1.0/cam.get_px();
	    	 double my = 1.0/cam.get_py();
	    	 double xc = cam.get_u0();
	    	 double yc = cam.get_v0();

	    	 vpImagePoint iP0T;
	    	 vpMeterPixelConversion::convertPoint(cam, x0, y0, iP0T);
	    	       //icpointi = iP0T;
	    	 vpDisplay::displayCross(I,iP0T,4,vpColor::red,4);


	      //kltPoints[scaleLevel][i].icpoint_curr = vpImagePoint(static_cast<double>(y),static_cast<double>(x));
	      //vpDisplay::displayCross(I,kltPoints[scaleLevel][i].icpoint0,4,vpColor::red,4);
	      vpDisplay::displayCross(I,kltPoints[scaleLevel][i].icpoint_curr,4,vpColor::blue,4);
	      //if ((kltPoints[scaleLevel][id0].icpoint0.get_u()-kltPoints[scaleLevel][id0].icpoint_curr.get_u())*(kltPoints[scaleLevel][id0].icpoint0.get_u()-kltPoints[scaleLevel][id0].icpoint_curr.get_u()) + (kltPoints[scaleLevel][id0].icpoint0.get_v()-kltPoints[scaleLevel][id0].icpoint_curr.get_v())*(kltPoints[scaleLevel][id0].icpoint0.get_v()-kltPoints[scaleLevel][id0].icpoint_curr.get_v())>400)
	    	//  kltPoints[scaleLevel][id0].valid = false;
	      //std::cout << " error " << (kltPoints[scaleLevel][id0].icpoint0.get_u()-kltPoints[scaleLevel][id0].icpoint_curr.get_u())*(kltPoints[scaleLevel][id0].icpoint0.get_u()-kltPoints[scaleLevel][id0].icpoint_curr.get_u()) + (kltPoints[scaleLevel][id0].icpoint0.get_v()-kltPoints[scaleLevel][id0].icpoint_curr.get_v())*(kltPoints[scaleLevel][id0].icpoint0.get_v()-kltPoints[scaleLevel][id0].icpoint_curr.get_v()) << std::endl;
	         }
	     }
              }*/
        //   std::cout << "ok 1 " << std::endl;
	//kltTracker.initTracking(frame_);
}

/*void savepair(char *buffer, const std::pair<point3d, point2d> &pair) {
    memcpy(buffer, &pair.first.x, sizeof(double));
    memcpy(buffer + sizeof(double), &pair.first.y, sizeof(double));
    memcpy(buffer + sizeof(double), &pair.first.z, sizeof(double));
    memcpy(buffer + sizeof(double), &pair.second.i, sizeof(int));
    memcpy(buffer + sizeof(int), &pair.second.j, sizeof(int));

}*/

void apMbTracker::savepair(std::string &message, const std::pair<point3d, point2d> &pair) {
    message = std::to_string(pair.first.x) + " " + std::to_string(pair.first.y) + " " + std::to_string(pair.first.z) + " "
            + std::to_string(pair.second.i) + " " + std::to_string(pair.second.j) + " ";
}

void apMbTracker::save3dpoint(std::string &message, const point3d &point3d) {
    message = std::to_string(point3d.x) + " " + std::to_string(point3d.y) + " " + std::to_string(point3d.z) + " ";
}

void apMbTracker::save2dpoint(std::string &message, const point2d &point2d) {
    message = std::to_string(point2d.i) + " " + std::to_string(point2d.j) + " ";
}

void apMbTracker::buildCorrespondencesEdges2D(std::vector<point2d> &trackededges, std::vector<int> &suppress) {

trackededges.resize(0);
suppress.resize(0);

//#pragma omp parallel for
        for (int k = 0; k < points[scaleLevel].size(); k++)
        {
            vpPointSite site = points[scaleLevel][k]->s;

            point2d p2d;
            p2d.i = site.i;
            p2d.j = site.j;
            trackededges.push_back(p2d);
            suppress.push_back(site.suppress);

        }

}

/*!
 Initialize the control points thanks to a given pose of the camera.

 \param I : The image.
 \param _cMo : The pose of the camera used to initialize the control points.
 */
/*void
 apMbTracker::initControlPoints(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &_cMo)
 {
 apControlPoint *p ;

 ;
 for (int k = 0; k<points[scaleLevel].size(); k++)
 {
 p = (points[scaleLevel])[k] ;
 p->initControlPoint(I,_cMo) ;
 }

 }
 }*/


/*!
 Track the control points in the image.

 \param I : the image.
 */
void apMbTracker::trackControlPoints(const vpImage<unsigned char> &I) {
//#pragma omp parallel for
	for (int k = 0; k < points[scaleLevel].size(); k++)
		(points[scaleLevel])[k]->track(I, Iprec);
}

void apMbTracker::trackKltControlPoints(const vpImage<unsigned char> &I) {


float x, y;
long id=0;
vpImagePoint iP0T;
/*if (predictKLT)
	for (int j = 0; j < kltPoints[scaleLevel].size();j++)
	{
	    vpFeaturePoint featurepoint;
	    if (kltPoints[scaleLevel][j].cpoint.oP[0] !=0)
	    {
		//vpDisplay::displayCross(I,kltPoints[scaleLevel][i].icpoint_curr,4,vpColor::blue,4);
		kltPoints[scaleLevel][j].cpointo.changeFrame(cMo);
		kltPoints[scaleLevel][j].cpointo.projection();
		//std::cout << " cpointo " << kltPoints[scaleLevel][j].cpointo.oP << std::endl;
	    vpFeatureBuilder::create(featurepoint,kltPoints[scaleLevel][j].cpointo);
	    double x0 = featurepoint.get_x();
	    double y0 = featurepoint.get_y();
	    vpMeterPixelConversion::convertPoint(cam, x0, y0, iP0T);

	    for (unsigned int i = 0; i < static_cast<unsigned int>(kltTracker.getNbFeatures()); i++){
	    kltTracker.getFeature((int)i, id, x, y);
	    if (j == id)
	    {
	    	//kltTracker.getFeatures()[j].x = iP0T.get_u();
	        //kltTracker.getFeatures()[j].y = iP0T.get_v();
	    }
	    }
	}
	}*/
//getchar();

cv::Mat frame_;
vpImageConvert::convert(I, frame_);
kltTracker.track(frame_);

id = 0;
std::map<int, apKltControlPoint>::const_iterator iter = kltPoints[scaleLevel].begin();
/*for( ; iter != kltPoints[scaleLevel].end(); iter++){
     int id(iter->first);
     kltTracker.getFeature((int)i, id, x, y);
           kltPoints[scaleLevel][id].icpoint_curr = vpImagePoint(static_cast<double>(y),static_cast<double>(x));
}*/
for( ; iter != kltPoints[scaleLevel].end(); iter++){
//	std::cout << "ok 0 " << std::endl;
 int id0 = iter->first;
   for (unsigned int i = 0; i < static_cast<unsigned int>(kltTracker.getNbFeatures()); i++){
     kltTracker.getFeature((int)i, id, x, y);
     if(id0 == id)
     {
      kltPoints[scaleLevel][id0].icpoint_curr = vpImagePoint(static_cast<double>(y),static_cast<double>(x));
      //if ((kltPoints[scaleLevel][id0].icpoint0.get_u()-kltPoints[scaleLevel][id0].icpoint_curr.get_u())*(kltPoints[scaleLevel][id0].icpoint0.get_u()-kltPoints[scaleLevel][id0].icpoint_curr.get_u()) + (kltPoints[scaleLevel][id0].icpoint0.get_v()-kltPoints[scaleLevel][id0].icpoint_curr.get_v())*(kltPoints[scaleLevel][id0].icpoint0.get_v()-kltPoints[scaleLevel][id0].icpoint_curr.get_v())>400)
    	//  kltPoints[scaleLevel][id0].valid = false;
      //std::cout << " error " << (kltPoints[scaleLevel][id0].icpoint0.get_u()-kltPoints[scaleLevel][id0].icpoint_curr.get_u())*(kltPoints[scaleLevel][id0].icpoint0.get_u()-kltPoints[scaleLevel][id0].icpoint_curr.get_u()) + (kltPoints[scaleLevel][id0].icpoint0.get_v()-kltPoints[scaleLevel][id0].icpoint_curr.get_v())*(kltPoints[scaleLevel][id0].icpoint0.get_v()-kltPoints[scaleLevel][id0].icpoint_curr.get_v()) << std::endl;
     }
      }
//   std::cout << "ok 1 " << std::endl;
   }
//kltTracker.initTracking(frame_);
}

void apMbTracker::trackKltControlPointsFromSynth(const vpImage<unsigned char> &I, const vpImage<unsigned char> &Itex) {


float x, y;
long id=0;
vpImagePoint iP0T;
/*if (predictKLT)
        for (int j = 0; j < kltPoints[scaleLevel].size();j++)
        {
            vpFeaturePoint featurepoint;
            if (kltPoints[scaleLevel][j].cpoint.oP[0] !=0)
            {
                //vpDisplay::displayCross(I,kltPoints[scaleLevel][i].icpoint_curr,4,vpColor::blue,4);
                kltPoints[scaleLevel][j].cpointo.changeFrame(cMo);
                kltPoints[scaleLevel][j].cpointo.projection();
                //std::cout << " cpointo " << kltPoints[scaleLevel][j].cpointo.oP << std::endl;
            vpFeatureBuilder::create(featurepoint,kltPoints[scaleLevel][j].cpointo);
            double x0 = featurepoint.get_x();
            double y0 = featurepoint.get_y();
            vpMeterPixelConversion::convertPoint(cam, x0, y0, iP0T);

            for (unsigned int i = 0; i < static_cast<unsigned int>(kltTracker.getNbFeatures()); i++){
            kltTracker.getFeature((int)i, id, x, y);
            if (j == id)
            {
                //kltTracker.getFeatures()[j].x = iP0T.get_u();
                //kltTracker.getFeatures()[j].y = iP0T.get_v();
            }
            }
        }
        }*/
//getchar();

for (int j = 0; j < kltPoints[scaleLevel].size();j++)
{
        kltTracker.suppressFeature(j);

}


//#pragma omp parallel for
        for (int k = 0; k < controlpoints.size(); k++)
        {

            double i0, j0;

            vpMeterPixelConversion::convertPoint(cam, controlpoints[k].x/controlpoints[k].z, controlpoints[k].y/controlpoints[k].z, j0,i0);

            kltTracker.addFeature(k, j0, i0);
        }


cv::Mat frame_,frame_synth;
vpImageConvert::convert(I, frame_);
vpImageConvert::convert(Itex, frame_synth);
kltTracker.track(frame_synth);
kltTracker.track(frame_);

id = 0;
std::map<int, apKltControlPoint>::const_iterator iter = kltPoints[scaleLevel].begin();
/*for( ; iter != kltPoints[scaleLevel].end(); iter++){
     int id(iter->first);
     kltTracker.getFeature((int)i, id, x, y);
           kltPoints[scaleLevel][id].icpoint_curr = vpImagePoint(static_cast<double>(y),static_cast<double>(x));
}*/

//kltTracker.initTracking(frame_);
}

/*!
 Track the control points in the image, in case of a multiple hypothesis solution.

 \param I : the image.
 */
void apMbTracker::trackControlPointsMH(const vpImage<unsigned char> &I) {

//#pragma omp parallel for
	for (int k = 0; k < points[scaleLevel].size(); ++k)
		(points[scaleLevel])[k]->trackMH(I, Iprec);
}

/*!
 Track the control points in the image, in case of a multiple hypothesis solution, for a prediction phase.

 \param I : the image.
 */
void apMbTracker::trackControlPointsPred(const vpImage<unsigned char> &I) {
//#pragma omp parallel for
	for (int k = 0; k < points[scaleLevel].size(); k++)
		(points[scaleLevel])[k]->trackPred(I, Iprec);
}

void apMbTracker::trackControlPointsLinesMH(
		const vpImage<unsigned char> &I) {
//#pragma omp parallel for
	for (int k = 0; k < lines[scaleLevel].size(); k++)
		lines[scaleLevel][k]->trackMovingEdgeMHP(I, gradMap, cMo);
}

/*!
 Update the control points at the end of the virtual visual servoing.

 \param I : the image.
 */
void apMbTracker::updateControlPoints() {

	//points_1=points;
	/*apControlPoint *p;
	 apControlPoint *p1;
	 vpPoint P;
	 vpPoint P1;
	 //vpColVector profile(3);
	 ;
	 for (int k = 0; k<points[scaleLevel].size(); k++){
	 p = (points[scaleLevel])[k];
	 points1[scaleLevel].front();
	 P=p->cpoint;
	 while (!points1[scaleLevel].outside()){
	 p1 = points1[scaleLevel].value();
	 P1=p->cpoint;
	 if (!samePoint(P1,P2))
	 {
	 profile=p1->getProfile();
	 }
	 p->setProfile(p);

	 }
	 //if (l->nbFeature == 0 && l->isVisible()) l->Reinit = true;
	 points[scaleLevel].next();
	 }  */
	//delete *points;
	apControlPoint *p;
	for (unsigned int i = 0; i < scales.size(); i++) {
		if (scales[i]) {
			for (int k = 0; k < points[i].size(); k++) {
				p = (points[i])[k];
				if (p != NULL)
					delete p;
				p = NULL;
			}
		}
	}

}

/*!
 Check if two vpPoints are similar.

 To be similar : \f$ (X_1 - X_2)^2 + (Y_1 - Y_2)^2 + (Z_1 - Z_2)^2 < threshold \f$.

 \param P1 : The first point to compare
 \param P2 : The second point to compare
 \param threshold : The threshold  used to decide if the points are similar or not.
 */
bool samePoint(const vpPoint &P1, const vpPoint &P2, double threshold = 1e-4) {
	double d = vpMath::sqr(P1.get_oX() - P2.get_oX()) + vpMath::sqr(P1.get_oY()
			- P2.get_oY()) + vpMath::sqr(P1.get_oZ() - P2.get_oZ());
	if (d < threshold)
		return true;
	else
		return false;
}

void process(const vpImage<vpRGBa>& Inormd, vpImage<unsigned char>& Iout) {
	int rows = Inormd.getHeight();
	int cols = Inormd.getWidth();
	vpImage<unsigned char> Iin(rows, cols);
	vpImage<unsigned char> Ig(rows, cols);
#pragma omp parallel
	{
#pragma omp for
		for (int nn = 3; nn < rows - 3; nn++)
			for (int mm = 3; mm < cols - 3; mm++)
				Iin[nn][mm] = Inormd[nn][mm].A;
/*#pragma omp for
		for (int nn = 3; nn < rows - 3; nn++)
			for (int mm = 3; mm < cols - 3; mm++)
				Ig[nn][mm] = vpImageFilter::gaussianFilter(Iin, nn, mm);*/
#pragma omp for
		for (int ii = 3; ii < rows - 3; ii++) {
			for (int jj = 3; jj < cols - 3; jj++) {
				double a = (apImageFilter::sobelFilterX(Iin, ii, jj));
				double b = (apImageFilter::sobelFilterY(Iin, ii, jj));
				if ((a != 0 || b != 0) && sqrt(
						(vpMath::sqr(a) + vpMath::sqr(b))) > 0)
					Iout[ii][jj] = 255 * (-(atan(a / b)) / M_PI + 1 / 2);
				else
					Iout[ii][jj] = 0;
			}
		}
	}
	//vpImageIo::writePNG(Iout, "ihough1.png");
}

void process1(const vpImage<vpRGBa>& Inormd, vpImage<unsigned char>& Ilap) {
	int rows = Inormd.getHeight();
	int cols = Inormd.getWidth();
	vpImage<unsigned char> Iin(rows, cols);
	vpImage<unsigned char> Ig(rows, cols);
	vpImage<unsigned char> Iout(rows, cols);
	Ig = Iin;
#pragma omp parallel
	{
#pragma omp for
		for (int nn = 3; nn < rows - 3; nn++)
			for (int mm = 3; mm < cols - 3; mm++)
				Iin[nn][mm] = Inormd[nn][mm].A;
#pragma omp for
		for (int nn = 3; nn < rows - 3; nn++)
			for (int mm = 3; mm < cols - 3; mm++)
				Ig[nn][mm] = vpImageFilter::gaussianFilter(Iin, nn, mm);

#pragma omp for
		for (int nn = 3; nn < rows - 3; nn++)
			for (int mm = 3; mm < cols - 3; mm++) {
				Ilap[nn][mm] = (int) ((double) apImageFilter::lapFilter(Iin,
						nn, mm) / 255.0);
				//std::cout << " ilap " << (double)Ilap[nn][mm] << std::endl;
			}

#pragma omp for
		for (int ii = 3; ii < rows - 3; ii++) {
			for (int jj = 3; jj < cols - 3; jj++) {
				double a = (apImageFilter::sobelFilterX(Ig, ii, jj));
				double b = (apImageFilter::sobelFilterY(Ig, ii, jj));
				if ((a != 0 || b != 0) && sqrt(
						(vpMath::sqr(a) + vpMath::sqr(b))) > 0)
					Iout[ii][jj] = 255 * (-(atan(a / b)) / M_PI + 1 / 2);
				else
					Iout[ii][jj] = 0;
			}
		}
#pragma omp for
		for (int nn = 3; nn < rows - 3; nn++)
			for (int mm = 3; mm < cols - 3; mm++) {
				if (Ilap[nn][mm] * 255 > 0.00000000000001)
					Ilap[nn][mm] = Iout[nn][mm];
				else
					Ilap[nn][mm] = 100;
			}
	}
}


/*!
 Extract 3D control points from the depth edges, the texture or color edges, given the depth buffer and the normal map
 \param I : input image.
 \param Inormd : Normal map (RGB channels) and depth buffer (A channel)
 \param Ior : oriented edge map from depth edge.
 \param Itex : oriented edge map from texture or color edges.
 \param dist : Z coordinate of the center of the object, in order to define and update the near and far clip distances
 */

void apMbTracker::buildControlPoints2D(const vpImage<unsigned char> &I, std::vector<point2d> &points2d, std::vector<point2dd> &normals) {

	npoints = 0;
//#pragma omp parallel
	{
		std::vector<apControlPoint*> local_insert_table;
		vpColVector norm(3);
		for (unsigned int i = 0; i < scales.size(); ++i) {
			if (scales[i]) {
//#pragma omp master
				downScale(i);
//#pragma omp barrier
//#pragma omp for
				for (int k = 0; k < points[i].size(); ++k) {
					apControlPoint *p = (points[i])[k];
					if (p != NULL)
						delete p;
				}
//#pragma omp master
				points[i].clear();
                                for (int k = 0; k < points2d.size(); k++)
                                {
                                                double theta = acos(-normals[k].x/normals[k].y);
                                                norm[0] = 1;
                                                norm[1] = 0;
                                                norm[2] = 0;
						const double l = std::sqrt(norm[0] * norm[0] + norm[1]
								* norm[1] + norm[2] * norm[2]);
                                                if (l > 1e-1)
                                                {
                                                        double Z = 1;
							apControlPoint *p = new apControlPoint;
							p->setCameraParameters(&cam);
							p->setMovingEdge(&me);
                                                        p->buildPoint(points2d[k].i, points2d[k].j, Z, theta, norm, cMo);
							p->initControlPoint(I, 0);
							npoints += 1;
							local_insert_table.push_back(p);
						}
				}
				if (!local_insert_table.empty())
//#pragma omp critical
				{
					points[i].insert(points[i].end(),
							local_insert_table.begin(),
							local_insert_table.end());
					//npoints += local_insert_table.size();
				}
				local_insert_table.clear();

				//std::cout << " size " << points[i].size() << std::endl;
			}
//#pragma omp barrier
//#pragma omp master
			upScale(i);
		}
	}
	std::cout << " size points " << points[scaleLevel].size() << std::endl;

}

void apMbTracker::extractControlPoints(const vpImage<unsigned char> &I,
                const vpImage<vpRGBa>& Inormd, const vpImage<unsigned char>& Ior,
                const vpImage<unsigned char>& Itex, const double distmin, const double distmax) {
        int sample = rendparam.sampleR;
        const double znear = distmin;
        const double zfar = distmax;

        const int rows = Ior.getHeight();
        const int cols = Ior.getWidth();

double t0 = vpTime::measureTimeMs();
        vpImage<unsigned char> I2(rows, cols);
        process(Inormd, I2);

        npoints = 0;

        if(rendparam.useNPoints)
        {
        int npointsparam = rendparam.nPoints;
        int ntotalpoints = 0;
        bool flag = false;
        for (int n = 0; n < rows; n++)
                for (int m = 0; m < cols; m++)
                {
                        flag = false;
        if (Itex[n][m] != 100) {
                ntotalpoints++;
                flag = true;
        } else if (Ior[n][m] != 100) {
                if (!flag)
                ntotalpoints++;
        } else
                continue;
                }
        if(npointsparam < ntotalpoints)
        {
     sample = 2*floor((double)ntotalpoints/npointsparam);
        }
        else{
         sample = 1;
        }
        std::cout << " sample " << sample << " ntotalpoints " << ntotalpoints << std::endl;
        }

double t1 = vpTime::measureTimeMs();
                                std::cout << "timeprocess " << t1 - t0 << std::endl;

//#pragma omp parallel
        {
                std::vector<apControlPoint*> local_insert_table;
                vpColVector norm(3);
                for (unsigned int i = 0; i < scales.size(); ++i) {
                        if (scales[i]) {
//#pragma omp master
                                downScale(i);
//#pragma omp barrier

//#pragma omp for
                                for (int k = 0; k < points[i].size(); ++k) {
                                        apControlPoint *p = (points[i])[k];
                                        if (p != NULL)
                                                delete p;
                                }
//#pragma omp master
                                points[i].clear();

                                const int m0 = (20 + sample - 1) / sample * sample;
                                const int n1 = rows - 20;
                                const int m1 = cols - 20;
//#pragma omp for nowait
                                for (int n = m0; n < n1; ++n) {
                                        const int sample0 = ((n - m0) % sample) == 0 ? 1 : sample;
                                        for (int m = m0; m < m1; m += sample0) {
                                                double theta;
                                                if (Itex[n][m] != 100)
                                                        theta = 3.1416 * (double) ((double) I2[n][m] / 255
                                                                        - 0.5);
                                                else if (Ior[n][m] != 100)
                                                        theta = 3.1416 * (double) ((double) I2[n][m] / 255
                                                                        - 0.5);
                                                else
                                                        continue;

                                                norm[0] = (rendparam.Normx)
                                                                * ((double) ((double) Inormd[n][m].R) / 255
                                                                                - 0.5);
                                                norm[1] = (rendparam.Normy)
                                                                * ((double) ((double) Inormd[n][m].G) / 255
                                                                                - 0.5);
                                                norm[2] = (rendparam.Normz)
                                                                * ((double) ((double) Inormd[n][m].B) / 255
                                                                                - 0.5);
                                                const double l = std::sqrt(norm[0] * norm[0] + norm[1]
                                                                * norm[1] + norm[2] * norm[2]);
                                                if (l > 1e-1) {

                                                        double z = 2.0 * (double) Inormd[n][m].A/ 255 - 1.0;
                                                        /*double Z = -(znear * zfar)
                                                                        / (((double) ((double) Inormd[n][m].A)
                                                                                        / 255) * (zfar - znear) - zfar);*/
                                                        double Z = -2*(znear * zfar) / (z * (zfar - znear) - (zfar + znear));
                                                        apControlPoint *p = new apControlPoint;
                                                        p->setCameraParameters(&cam);
                                                        p->setMovingEdge(&me);
                                                        p->buildPoint(n, m, Z, theta, norm, cMo);
                                                        p->initControlPoint(I, 0);
                                                        npoints += 1;
                                                        local_insert_table.push_back(p);
                                                }
                                        }
                                }
                                if (!local_insert_table.empty())
//#pragma omp critical
                                {
                                        points[i].insert(points[i].end(),
                                                        local_insert_table.begin(),
                                                        local_insert_table.end());
                                        //npoints += local_insert_table.size();
                                }
                                local_insert_table.clear();

                                //std::cout << " size " << points[i].size() << std::endl;
                        }
//#pragma omp barrier
//#pragma omp master
                        upScale(i);
                }
        }

        std::cout << " size points " << points[scaleLevel].size() << std::endl;

}

/*!
 Extract 3D control points from the depth edges, the texture or color edges, given the depth buffer and the normal map
 \param I : input image.
 \param Inormd : Normal map (RGB channels) and depth buffer (A channel)
 \param Ior : oriented edge map from depth edge.
 \param Itex : oriented edge map from texture or color edges.
 \param dist : Z coordinate of the center of the object, in order to define and update the near and far clip distances
 */

void apMbTracker::extractControlPoints(const vpImage<unsigned char> &I,
                const vpImage<vpRGBa>& Inormd, const vpImage<unsigned char>& Ior,
                const vpImage<unsigned char>& Itex, const double dist) {
        int sample = rendparam.sampleR;
        const double znear = dist - rendparam.clipDist;
        const double zfar = dist + rendparam.clipDist;

        const int rows = Ior.getHeight();
        const int cols = Ior.getWidth();

double t0 = vpTime::measureTimeMs();
        vpImage<unsigned char> I2(rows, cols);
        process(Inormd, I2);

        npoints = 0;

        if(rendparam.useNPoints)
        {
        int npointsparam = rendparam.nPoints;
        int ntotalpoints = 0;
        bool flag = false;
        for (int n = 0; n < rows; n++)
                for (int m = 0; m < cols; m++)
                {
                        flag = false;
        if (Itex[n][m] != 100) {
                ntotalpoints++;
                flag = true;
        } else if (Ior[n][m] != 100) {
                if (!flag)
                ntotalpoints++;
        } else
                continue;
                }
        if(npointsparam < ntotalpoints)
        {
     sample = 2*floor((double)ntotalpoints/npointsparam);
        }
        else{
         sample = 1;
        }
        std::cout << " sample " << sample << " ntotalpoints " << ntotalpoints << std::endl;
        }

double t1 = vpTime::measureTimeMs();
                                std::cout << "timeprocess " << t1 - t0 << std::endl;

//#pragma omp parallel
        {
                std::vector<apControlPoint*> local_insert_table;
                vpColVector norm(3);
                for (unsigned int i = 0; i < scales.size(); ++i) {
                        if (scales[i]) {
//#pragma omp master
                                downScale(i);
//#pragma omp barrier

//#pragma omp for
                                for (int k = 0; k < points[i].size(); ++k) {
                                        apControlPoint *p = (points[i])[k];
                                        if (p != NULL)
                                                delete p;
                                }
//#pragma omp master
                                points[i].clear();

                                const int m0 = (20 + sample - 1) / sample * sample;
                                const int n1 = rows - 20;
                                const int m1 = cols - 20;
//#pragma omp for nowait
                                for (int n = m0; n < n1; ++n) {
                                        const int sample0 = ((n - m0) % sample) == 0 ? 1 : sample;
                                        for (int m = m0; m < m1; m += sample0) {
                                                double theta;
                                                if (Itex[n][m] != 100)
                                                        theta = 3.1416 * (double) ((double) I2[n][m] / 255
                                                                        - 0.5);
                                                else if (Ior[n][m] != 100)
                                                        theta = 3.1416 * (double) ((double) I2[n][m] / 255
                                                                        - 0.5);
                                                else
                                                        continue;

                                                norm[0] = (rendparam.Normx)
                                                                * ((double) ((double) Inormd[n][m].R) / 255
                                                                                - 0.5);
                                                norm[1] = (rendparam.Normy)
                                                                * ((double) ((double) Inormd[n][m].G) / 255
                                                                                - 0.5);
                                                norm[2] = (rendparam.Normz)
                                                                * ((double) ((double) Inormd[n][m].B) / 255
                                                                                - 0.5);
                                                const double l = std::sqrt(norm[0] * norm[0] + norm[1]
                                                                * norm[1] + norm[2] * norm[2]);
                                                if (l > 1e-1) {
                                                        double Z = -(znear * zfar)
                                                                        / (((double) ((double) Inormd[n][m].A)
                                                                                        / 255) * (zfar - znear) - zfar);
                                                        apControlPoint *p = new apControlPoint;
                                                        p->setCameraParameters(&cam);
                                                        p->setMovingEdge(&me);
                                                        p->buildPoint(n, m, Z, theta, norm, cMo);
                                                        p->initControlPoint(I, 0);
                                                        npoints += 1;
                                                        local_insert_table.push_back(p);
                                                }
                                        }
                                }
                                if (!local_insert_table.empty())
//#pragma omp critical
                                {
                                        points[i].insert(points[i].end(),
                                                        local_insert_table.begin(),
                                                        local_insert_table.end());
                                        //npoints += local_insert_table.size();
                                }
                                local_insert_table.clear();

                                //std::cout << " size " << points[i].size() << std::endl;
                        }
//#pragma omp barrier
//#pragma omp master
                        upScale(i);
                }
        }

        std::cout << " size points " << points[scaleLevel].size() << std::endl;

}

void apMbTracker::extractControlPointsP(const vpImage<unsigned char> &I,
		const vpImage<vpRGBa>& Inormd, const vpImage<unsigned char>& Ior,
                const vpImage<unsigned char>& Itex, const double dist) {
    int sample = rendparam.sampleR;
        const double znear = dist - rendparam.clipDist;
        const double zfar = dist + rendparam.clipDist;

	const int rows = Ior.getHeight();
	const int cols = Ior.getWidth();

	vpImage<unsigned char> I2(rows, cols);
	process(Inormd, I2);

	if(rendparam.useNPoints)
	{
	int npointsparam = rendparam.nPoints;
	int ntotalpoints = 0;
	bool flag = false;
	for (int n = 0; n < rows; n++)
		for (int m = 0; m < cols; m++)
		{
			flag = false;
	if (Itex[n][m] != 100) {
		ntotalpoints++;
		flag = true;
	} else if (Ior[n][m] != 100) {
		if (!flag)
		ntotalpoints++;
	} else
		continue;
		}
	if(npointsparam < ntotalpoints)
	{
     sample = 2*floor((double)ntotalpoints/npointsparam);
	}
	else{
	 sample = 1;
	}
	}

	#pragma omp parallel
	{
		std::vector<apControlPoint*> local_insert_table;
		vpColVector norm(3);
		for (unsigned int i = 0; i < scales.size(); ++i) {
			if (scales[i]) {
#pragma omp master
				downScale(i);
#pragma omp barrier

				//#pragma omp for
				for (int k = 0; k < points[i].size(); ++k) {
					apControlPoint *p = (points[i])[k];
					if (p != NULL)
						delete p;
				}
#pragma omp master
				points[i].clear();

				const int m0 = (20 + sample - 1) / sample * sample;
				const int n1 = rows - 20;
				const int m1 = cols - 20;
#pragma omp for nowait
				for (int n = m0; n < n1; ++n) {
					const int sample0 = ((n - m0) % sample) == 0 ? 1 : sample;
					for (int m = m0; m < m1; m += sample0) {
						double theta;
						if (Itex[n][m] != 100)
							theta = 3.1416 * (double) ((double) I2[n][m] / 255
									- 0.5);
						else if (Ior[n][m] != 100)
							theta = 3.1416 * (double) ((double) I2[n][m] / 255
									- 0.5);
						else
							continue;

						norm[0] = (rendparam.Normx)
								* ((double) ((double) Inormd[n][m].R) / 255
										- 0.5);
						norm[1] = (rendparam.Normy)
								* ((double) ((double) Inormd[n][m].G) / 255
										- 0.5);
						norm[2] = (rendparam.Normz)
								* ((double) ((double) Inormd[n][m].B) / 255
										- 0.5);
						const double l = std::sqrt(norm[0] * norm[0] + norm[1]
								* norm[1] + norm[2] * norm[2]);
						if (l > 1e-1) {
							double Z = -(znear * zfar)
									/ (((double) ((double) Inormd[n][m].A)
											/ 255) * (zfar - znear) - zfar);
							apControlPoint *p = new apControlPoint;
							p->setCameraParameters(&cam);
							p->setMovingEdge(&me);
							p->buildPoint(n, m, Z, theta, norm, cMo);
							p->initControlPoint(I, 0);
							p->setProfile(Iprec);
							npoints += 1;
							local_insert_table.push_back(p);
						}
					}
				}

				if (!local_insert_table.empty())
#pragma omp critical
				{
					points[i].insert(points[i].end(),
							local_insert_table.begin(),
							local_insert_table.end());
					npoints += local_insert_table.size();
				}
				local_insert_table.clear();

				//std::cout << " size " << points[i].size() << std::endl;
			}
#pragma omp barrier
#pragma omp master
			upScale(i);
		}
	}

}

/*!
 Extract 3D control points from the depth edges, the texture or color edges, given the depth buffer and the normal map
 \param I : input image.
 \param Inormd : Normal map (RGB channels) and depth buffer (A channel)
 \param Ior : oriented edge map from depth edge.
 \param Itex : oriented edge map from texture or color edges.
 \param dist : Z coordinate of the center of the object, in order to define and update the near and far clip distances
 */

void apMbTracker::extractControlPointsCCD(
		const vpImage<unsigned char> &I, const vpImage<vpRGBa>& Inormd,
		const vpImage<unsigned char>& Ior, const vpImage<unsigned char>& Itex,
		const double dist) {

	int sample = rendparam.sampleR;
	double znear = dist - rendparam.clipDist;
	double zfar = dist + rendparam.clipDist;

	int rows = Ior.getHeight();
	int cols = Ior.getWidth();
	npointssil = 0;
	npoints = 0;

	vpImage<unsigned char> I2(rows, cols);
	process(Inormd, I2);
	vpImage<unsigned char> Isil(rows, cols);

	if(rendparam.useNPoints)
	{
	int npointsparam = rendparam.nPoints;
	int ntotalpoints = 0;
	bool flag = false;
	for (int n = 0; n < rows; n++)
		for (int m = 0; m < cols; m++)
		{
			flag = false;
	if (Itex[n][m] != 100) {
		ntotalpoints++;
		flag = true;
	} /*else if (Ior[n][m] != 100) {
		if (!flag)
		ntotalpoints++;
	}*/ else
		continue;
		}
	if(npointsparam < ntotalpoints)
	{
     sample = 2*floor((double)ntotalpoints/npointsparam);
	}
	else{
	 sample = 1;
	}
	//std::cout << " sample " << sample << " ntotalpoints " << ntotalpoints << std::endl;
	}


	#pragma omp parallel
	{
		std::vector<apControlPoint*> local_points;
		std::vector<apControlPoint*> local_pointsCCD;
		vpColVector norm(3);
		for (unsigned int i = 0; i < scales.size(); ++i) {
#pragma omp barrier
			if (scales[i]) {
#pragma omp master
				downScale(i);
#pragma omp barrier

#pragma omp for nowait
				for (int k = 0; k < points[i].size(); k++) {
					apControlPoint *p = (points[i])[k];
					if (p != NULL)
						delete p;
				}
#pragma omp master
				points[i].clear();

#pragma omp for nowait
				for (int k = 0; k < CCDTracker.pointsCCD[i].size(); k++) {
					apControlPoint *pccd = CCDTracker.pointsCCD[i][k];
					if (pccd != NULL)
						delete pccd;
				}

#pragma omp master
				CCDTracker.pointsCCD[i].clear();
#pragma omp for
				for (int k = 0; k < rows; k++)
					for (int l = 0; l < cols; l++) {
						if (Inormd[k][l].A != 0)
							Isil[k][l] = Inormd[k][l].A;
						else
							Isil[k][l] = 255;
					}

				local_points.clear();
				local_pointsCCD.clear();

				const int m0 = (20 + sample - 1) / sample * sample;
				const int n1 = rows - 20;
				const int m1 = cols - 20;
#pragma omp for nowait
				for (int n = m0; n < n1; ++n) {
					const int sample0 = ((n - m0) % sample) == 0 ? 1 : sample;
					for (int m = m0; m < m1; m += sample0) {
						double theta;
						if (Itex[n][m] != 100) {
							theta = 3.1416 * (double) ((double) I2[n][m] / 255
									- 0.5);
						} else if (Ior[n][m] != 100) {
							theta = 3.1416 * (double) ((double) I2[n][m] / 255
									- 0.5);
						} else
							continue;

						norm[0] = (rendparam.Normx)
								* ((double) ((double) Inormd[n][m].R) / 255
										- 0.5);
						norm[1] = (rendparam.Normy)
								* ((double) ((double) Inormd[n][m].G) / 255
										- 0.5);
						norm[2] = (rendparam.Normz)
								* ((double) ((double) Inormd[n][m].B) / 255
										- 0.5);
						//std::cout<<norm[0]<<" "<<norm[1]<<" "<<norm[2]<<std::endl;
						const double l = std::sqrt(norm[0] * norm[0] + norm[1]
								* norm[1] + norm[2] * norm[2]);
						if (l > 1e-1) {
							double Z = -(znear * zfar)
									/ (((double) ((double) Inormd[n][m].A)
											/ 255) * (zfar - znear) - zfar);

							apControlPoint *p = new apControlPoint;
							p->setCameraParameters(&cam);
							p->setMovingEdge(&me);
							p->buildPoint(n, m, Z, theta, norm, cMo);
							p->initControlPoint(I, 0);
							npoints++;
							local_points.push_back(p);
							apControlPoint *pccd = new apControlPoint;
							pccd->setCameraParameters(&cam);
							pccd->setMovingEdge(&me);
							pccd->buildSilhouettePoint(n, m, Z, theta, norm,
									cMo);
							pccd->initControlPoint(Isil, 0);
							pccd->detectSilhouette(Isil,I, 0);
							if (pccd->isSilhouette)
							{
								local_pointsCCD.push_back(pccd);
								npointssil ++;
							}
							else
								delete pccd;
						}
					}
				}
				if (!local_points.empty())
#pragma omp critical
				{
					npoints += local_points.size();
					points[i].insert(points[i].end(), local_points.begin(),
							local_points.end());
					CCDTracker.pointsCCD[i].insert(
							CCDTracker.pointsCCD[i].end(),
							local_pointsCCD.begin(), local_pointsCCD.end());
					CCDTracker.npointsCCD += local_pointsCCD.size();
				}

			}
#pragma omp barrier
#pragma omp master
			upScale(i);
		}
	}
}

/*!
 Extract 3D control points from the depth edges, the texture or color edges, given the depth buffer and the normal map
 Cluster points into lines through Hough transform
 \param I : input image.
 \param Inormd : Normal map (RGB channels) and depth buffer (A channel)
 \param Ior : oriented edge map from depth edge.
 \param Itex : oriented edge map from texture or color edges.
 \param dist : Z coordinate of the center of the object, in order to define and update the near and far clip distances
 */

void apMbTracker::extractControlPointsLines(
		const vpImage<unsigned char> &I, const vpImage<vpRGBa>& Inormd,
		const vpImage<unsigned char>& Ior, const vpImage<unsigned char>& Itex,
		const double dist) {
	apControlPoint *p;
	apControlPoint *p0;
	vpImagePoint ip, ip1, ip2;
	apMbtDistanceLineMH *l;
	vpLine *lineL;
	double t0, t1;
	int bi1, bi2, bj1, bj2;
	double px = cam.get_px();
	double py = cam.get_py();
	int jc = cam.get_u0();
	int ic = cam.get_v0();
	int i1, j1;
	vpPoint Pc;
	double zb1, zb2;
	vpColVector Pc0(3);
	vpColVector ip0(2);
	double x, y, Z;
	vpColVector Normc(3);
	double rho, theta, rho_l, theta_l;
	int sample = rendparam.sampleR;
	double znear = dist - rendparam.clipDist;
	double zfar = dist + rendparam.clipDist;
	zn = znear;
	zf = zfar;
	vpImage<unsigned char> Ig;
	Ig = I;

	int rows = Ior.getHeight();
	int cols = Ior.getWidth();

	vpImage<unsigned char> I2(rows, cols);
	vpImage<unsigned char> I2_(rows, cols);
    I2_ = Ior;
	process(Inormd, I2);

	if(rendparam.useNPoints)
		{
		int npointsparam = rendparam.nPoints;
		int ntotalpoints = 0;
		bool flag = false;
		for (int n = 0; n < rows; n++)
			for (int m = 0; m < cols; m++)
			{
				flag = false;
		if (Itex[n][m] != 100) {
			ntotalpoints++;
			flag = true;
		} else if (Ior[n][m] != 100) {
			if (!flag)
			ntotalpoints++;
		} else
			continue;
			}
		if(npointsparam < ntotalpoints)
		{
	     sample = 2*floor((double)ntotalpoints/npointsparam);
		}
		else{
		 sample = 1;
		}
		std::cout << " sample " << sample << " ntotalpoints " << ntotalpoints << std::endl;
		}



	std::vector<apControlPoint*> local_points;
	vpColVector norm(3);

	for (unsigned int i = 0; i < scales.size(); i += 1) {
		if (scales[i]) {
			downScale(i);

			for (int k = 0; k < points[i].size(); k++) {
				p = (points[i])[k];
				if (p != NULL)
					delete p;
				p = NULL;
			}
			points[i].clear();

			for (int k = 0; k < lines[i].size(); k++) {
				l = lines[i][k];
				if (l != NULL) {
					for (int ii; ii < l->pointsvect.size(); ii++) {
						p0 = l->pointsvect[ii];
						delete p0;
					}
					l->pointsvect.resize(0);
					delete l;
				}
				l = NULL;
			}
			lines[i].clear();

			double n_sample;
			vpRotationMatrix R;
			cMo.extract(R);
			vpColVector norm(3);
			int step = 1;
			int range;
			int il = 0;

			gradMap.resize(rows, cols);
			vpImage<unsigned char> imG(rows, cols);
			vpImage<unsigned char> imGrad(rows, cols);
			vpImage<unsigned char> IorG(rows, cols);
			IorG = Ior;
			double a, b;
			for (int nn = 3; nn < rows - 3; nn++) {
				for (int mm = 3; mm < cols - 3; mm++) {
					imG[nn][mm] = vpImageFilter::gaussianFilter(Ig, nn, mm);
					//IorG[nn][mm] = vpImageFilter::gaussianFilter(Ior, nn, mm);
				}
			}
			for (int ii = 3; ii < rows - 3; ii++) {
				for (int jj = 3; jj < cols - 3; jj++) {
					a = (apImageFilter::sobelFilterX(imG, ii, jj));
					b = (apImageFilter::sobelFilterY(imG, ii, jj));
					if ((a != 0 || b != 0) && sqrt((vpMath::sqr(a)
							+ vpMath::sqr(b))) > 0)
						imGrad[ii][jj] = 255 * (-(atan(a / b)) / M_PI + 1 / 2);
					else
						imGrad[ii][jj] = 0;
				}
			}
			gradMap = imGrad;
			int w = 0;

			vpImage<unsigned char> Ip0(rows, cols);
			vpImage<unsigned char> I3(rows, cols);

			for (int n = 0; n < rows; n++) {
				for (int m = 0; m < cols; m++) {
					if (Ior[n][m] != 100)
						Ip0[n][m] = 255;
					else
						Ip0[n][m] = 0;
				}
			}

                        cv::Mat Ip,Ip1;
			vpImageConvert::convert(Ip0, Ip);
			Mat color_dst;
			vector<Vec4i> linesV;
			Mat dst(Ip);
			cvtColor(dst, color_dst, CV_GRAY2BGR);
			HoughLinesP(dst, linesV, 1, CV_PI / 180, 50, 50, 5);
			for (size_t i = 0; i < linesV.size(); i++) {
				line(color_dst, Point(linesV[i][0], linesV[i][1]), Point(
						linesV[i][2], linesV[i][3]), Scalar(255, 0, 255), 2, 8);
			}
			loadLines(I, linesV, Inormd, cMo);

			std::cout << " nblines " << linesV.size() << std::endl;
//
//			vpImage<vpRGBa> Ioverlay;
//						 Ip1 = new IplImage(color_dst);
//						 vpImageConvert::convert(Ip1, I3);
//						 delete Ip1;
//						 vpDisplayX display;
//						 display.init(I3, 1500, 10, "Hough");
//						 vpDisplay::display(I3);
//						 vpDisplay::flush(I3);
//						 vpDisplay::getImage(I3,Ioverlay);
//						 std::cout << " ok " << std::endl;
//						 vpImageIo::writePNG(I2_, "ihough0.png");
//						 vpDisplay::getClick(I3);

			//getchar();
			int kdisp = 0;

			const int m0 = (20 + sample - 1) / sample * sample;
			const int n1 = rows - 20;
			const int m1 = cols - 20;
			//#pragma omp for nowait
			for (int n = m0; n < n1; ++n) {
				const int sample0 = ((n - m0) % sample) == 0 ? 1 : sample;
				for (int m = m0; m < m1; m += sample0) {
					double theta;
					if (Itex[n][m] != 100)
						theta = 3.1416 * (double) ((double) I2[n][m] / 255
								- 0.5);
					else if (Ior[n][m] != 100)
						theta = 3.1416 * (double) ((double) I2[n][m] / 255
								- 0.5);
					else
						continue;

					norm[0] = (rendparam.Normx)
							* ((double) ((double) Inormd[n][m].R) / 255 - 0.5);
					norm[1] = (rendparam.Normy)
							* ((double) ((double) Inormd[n][m].G) / 255 - 0.5);
					norm[2] = (rendparam.Normz)
							* ((double) ((double) Inormd[n][m].B) / 255 - 0.5);
					const double nrm = std::sqrt(norm[0] * norm[0] + norm[1]
							* norm[1] + norm[2] * norm[2]);
					if (nrm > 1e-1) {
						double Z = -(znear * zfar)
								/ (((double) ((double) Inormd[n][m].A) / 255)
										* (zfar - znear) - zfar);
						bool flag = false;
						il = 0;
						double distmin = 1000;
						double dist = 0, dist0 = 0, dist1 = 0;
						int kmin;
						double dtheta0, dtheta1, dtheta2;
						x = (m - jc) / px;
						y = (n - ic) / py;
						for (int k = 0; k < lines[scaleLevel].size(); k++) {
							l = lines[scaleLevel][k];
							lineL = l->line;
							lineL->changeFrame(cMo);
							lineL->projection();
							rho_l = lineL->getRho();
							theta_l = lineL->getTheta();
							rho = x * cos(theta) + y * sin(theta);
							zb1
									= ((double) ((double) Inormd[linesV[il][1]][linesV[il][0]].A)
											/ 255);
							zb2
									= ((double) ((double) Inormd[linesV[il][3]][linesV[il][2]].A)
											/ 255);
							if (linesV[il][1] > linesV[il][3]) {
								bi1 = linesV[il][3];
								bi2 = linesV[il][1];
							} else {
								bi2 = linesV[il][3];
								bi1 = linesV[il][1];
							}

							if (linesV[il][0] > linesV[il][2]) {
								bj1 = linesV[il][2];
								bj2 = linesV[il][0];
							} else {
								bj2 = linesV[il][2];
								bj1 = linesV[il][0];
							}

							//std::cout << " ok " << rho << " rho l " << rho_l << " tetha " << theta_l <<std::endl;
							dist0 = abs(rho - rho_l);
							dist1 = abs(rho + rho_l);
							if (dist0 < dist1)
								dist = dist0;
							else
								dist = dist1;

							if ((dist < distmin) && (n <= bi2 && n >= bi1 && m
									<= bj2 && m >= bj1)) //  && abs(theta - theta_l) < 0.1)
							{
								distmin = dist;
								kmin = k;
								//std::cout << " distmin " << distmin << std::endl;
								dtheta0 = abs(theta - theta_l);
								dtheta1 = abs(theta - theta_l - M_PI);
								dtheta2 = abs(theta - theta_l + M_PI);
							}
							il++;
						}
						//if ((distmin < 0.0015) && (dtheta0 < 0.2 || dtheta1 < 0.2 || dtheta2 < 0.2 ))
						if ((distmin < 0.002))// && (dtheta0 < 0.3 || dtheta1 < 0.3 || dtheta2 < 0.3))
							{
							//if ((distmin < 0.0025) && (dtheta0 < 0.2 || dtheta1< 0.2 || dtheta2 < 0.25)) {
							flag = true;
							p0 = new apControlPoint;
							p0->setCameraParameters(&cam);
							p0->setMovingEdge(&me);
							p0->buildPoint(n, m, Z, theta, norm, cMo);
							l = lines[scaleLevel][kmin];
							l->pointsvect.push_back(p0);
							ip.set_i(n);
							ip.set_j(m);
							//if (sqrt((linesV[kmin][1] -linesV[kmin][3])*(linesV[kmin][1] -linesV[kmin][3]) + (linesV[kmin][0] -linesV[kmin][2])*(linesV[kmin][0] -linesV[kmin][2]))>34 && sqrt((linesV[kmin][1] -linesV[kmin][3])*(linesV[kmin][1] -linesV[kmin][3]) + (linesV[kmin][0] -linesV[kmin][2])*(linesV[kmin][0] -linesV[kmin][2]))<35)
							{
								//vpDisplay::displayCross(I, n, m, 2, vpColor::blue);
								kdisp++;
							}

							//vpDisplay::displayCross(I, ip, 2, vpColor::red, 2);
						}

						if (flag == false) {
							p = new apControlPoint;
							p->setCameraParameters(&cam);
							p->setMovingEdge(&me);
							p->buildPoint(n, m, Z, theta, norm, cMo);
							p->initControlPoint(I, 0);
							//npoints += 1;
							//points[i].push_back(p);
							local_points.push_back(p);
						}

					}
					w++;
				}
			}
			//#pragma omp critical
			if (!local_points.empty()) {
				npoints += local_points.size();
				points[i].insert(points[i].end(), local_points.begin(),
						local_points.end());
			}

			std::cout << " kdisp " << points[i].size() << std::endl;
			for (int k = 0; k < lines[scaleLevel].size(); k++) {
				l = lines[scaleLevel][k];
				//std::cout << " ok " << l->pointsvect.size() << " length " << sqrt((linesV[k][1] -linesV[k][3])*(linesV[k][1] -linesV[k][3]) + (linesV[k][0] -linesV[k][2])*(linesV[k][0] -linesV[k][2])) << std::endl;
				l->initMovingEdgeMHP(I, gradMap, cMo);
			}

			std::cout << " lines size " <<  lines[scaleLevel].size() << std::endl;

			//getchar();

			std::cout << " time " << t1 - t0 << std::endl;

		}
		upScale(i);
	}

}



//void apMbTracker::extractControlPointsLines(
//		const vpImage<unsigned char> &I, const vpImage<vpRGBa>& Inormd,
//		const vpImage<unsigned char>& Ior, const vpImage<unsigned char>& Itex,
//		const double dist) {
//
//	vpImagePoint ip, ip1, ip2;
//	vpLine *lineL;
//	double t0, t1;
//	int bi1, bi2, bj1, bj2;
//	double px = cam.get_px();
//	double py = cam.get_py();
//	int jc = cam.get_u0();
//	int ic = cam.get_v0();
//	int i1, j1;
//	vpPoint Pc;
//	double zb1, zb2;
//	vpColVector Pc0(3);
//	vpColVector ip0(2);
//	double x, y, Z;
//	vpColVector Normc(3);
//	double rho, theta, rho_l, theta_l;
//
//	vpImage<unsigned char> Ig;
//	Ig = I;
//
//
//
//	int sample = rendparam.sampleR;
//	double znear = dist - rendparam.clipDist;
//	double zfar = dist + rendparam.clipDist;
//
//	int rows = Ior.getHeight();
//	int cols = Ior.getWidth();
//
//	vpImage<unsigned char> I2(rows, cols);
//	vpImage<unsigned char> I2_(rows, cols);
//	I2_ = Ior;
//	process(Inormd, I2);
//	//vpImage<unsigned char> Isil(rows, cols);
//
//	if(rendparam.useNPoints)
//	{
//	int npointsparam = rendparam.nPoints;
//	int ntotalpoints = 0;
//	bool flag = false;
//	for (int n = 0; n < rows; n++)
//		for (int m = 0; m < cols; m++)
//		{
//			flag = false;
//	if (Itex[n][m] != 100) {
//		ntotalpoints++;
//		flag = true;
//	} else if (Ior[n][m] != 100) {
//		if (!flag)
//		ntotalpoints++;
//	} else
//		continue;
//		}
//	if(npointsparam < ntotalpoints)
//	{
//     sample = 2*floor((double)ntotalpoints/npointsparam);
//	}
//	else{
//	 sample = 1;
//	}
//	std::cout << " sample " << sample << " ntotalpoints " << ntotalpoints << std::endl;
//	}
//
//#pragma omp parallel
//{
//	std::vector<apControlPoint*> local_points;
//	//std::vector<apControlPoint*> local_pointsCCD;
//	vpColVector norm(3);
//	for (unsigned int i = 0; i < scales.size(); ++i) {
//#pragma omp barrier
//		if (scales[i]) {
//#pragma omp master
//			downScale(i);
//#pragma omp barrier
//
//#pragma omp for nowait
//			for (int k = 0; k < points[i].size(); k++) {
//				apControlPoint *p = (points[i])[k];
//				if (p != NULL)
//					delete p;
//			}
//#pragma omp master
//			points[i].clear();
//
//	        for (int k = 0; k < lines[i].size(); k++) {
//		      apMbtDistanceLineMH *l = lines[i][k];
//				if (l != NULL) {
//					for (int ii; ii < l->pointsvect.size(); ii++) {
//						apControlPoint *p0 = l->pointsvect[ii];
//						delete p0;
//					}
//					l->pointsvect.resize(0);
//					delete l;
//				}
//				l = NULL;
//			}
//			lines[i].clear();
//
//
////#pragma omp for nowait
////			for (int k = 0; k < CCDTracker.pointsCCD[i].size(); k++) {
////				apControlPoint *pccd = CCDTracker.pointsCCD[i][k];
////				if (pccd != NULL)
////					delete pccd;
////			}
////
////#pragma omp master
////			CCDTracker.pointsCCD[i].clear();
////#pragma omp for
////			for (int k = 0; k < rows; k++)
////				for (int l = 0; l < cols; l++) {
////					if (Inormd[k][l].A != 0)
////						Isil[k][l] = Inormd[k][l].A;
////					else
////						Isil[k][l] = 255;
////				}
//
//			local_points.clear();
////			local_pointsCCD.clear();
//
//			int il = 0;
//
//			gradMap.resize(rows, cols);
//			vpImage<unsigned char> imG(rows, cols);
//			vpImage<unsigned char> imGrad(rows, cols);
//			vpImage<unsigned char> IorG(rows, cols);
//			IorG = Ior;
//			double a, b;
//			for (int nn = 3; nn < rows - 3; nn++) {
//				for (int mm = 3; mm < cols - 3; mm++) {
//					imG[nn][mm] = vpImageFilter::gaussianFilter(Ig, nn, mm);
//					//IorG[nn][mm] = vpImageFilter::gaussianFilter(Ior, nn, mm);
//				}
//			}
//			for (int ii = 3; ii < rows - 3; ii++) {
//				for (int jj = 3; jj < cols - 3; jj++) {
//					a = (apImageFilter::sobelFilterX(imG, ii, jj));
//					b = (apImageFilter::sobelFilterY(imG, ii, jj));
//					if ((a != 0 || b != 0) && sqrt((vpMath::sqr(a)
//							+ vpMath::sqr(b))) > 0)
//						imGrad[ii][jj] = 255 * (-(atan(a / b)) / M_PI + 1 / 2);
//					else
//						imGrad[ii][jj] = 0;
//				}
//			}
//			gradMap = imGrad;
//			int w = 0;
//
//			vpImage<unsigned char> Ip0(rows, cols);
//			vpImage<unsigned char> I3(rows, cols);
//
//			for (int n = 0; n < rows; n++) {
//				for (int m = 0; m < cols; m++) {
//					if (Ior[n][m] != 100)
//						Ip0[n][m] = 255;
//					else
//						Ip0[n][m] = 0;
//				}
//			}
//
//			IplImage* Ip = NULL;
//			IplImage* Ip1;
//			vpImageConvert::convert(Ip0, Ip);
//			Mat color_dst;
//			vector<Vec4i> linesV;
//			Mat dst(Ip);
//			cvtColor(dst, color_dst, CV_GRAY2BGR);
//			HoughLinesP(dst, linesV, 1, CV_PI / 180, 20, 30, 2);
//			for (size_t i = 0; i < linesV.size(); i++) {
//				line(color_dst, Point(linesV[i][0], linesV[i][1]), Point(
//						linesV[i][2], linesV[i][3]), Scalar(255, 0, 255), 2, 8);
//			}
//			loadLines(I, linesV, Inormd, cMo);
//			/*vpImage<vpRGBa> Ioverlay;
//			 Ip1 = new IplImage(color_dst);
//			 vpImageConvert::convert(Ip1, I3);
//			 delete Ip1;
//			 vpDisplayX display;
//			 display.init(I3, 1500, 10, "Hough");
//			 vpDisplay::display(I3);
//			 vpDisplay::flush(I3);
//			 vpDisplay::getImage(I3,Ioverlay);
//			 std::cout << " ok " << std::endl;
//			 vpImageIo::writePNG(I2_, "ihough0.png");
//			 vpDisplay::getClick(I3);*/
//			//getchar();
//			int kdisp = 0;
//
//
//			const int m0 = (30 + sample - 1) / sample * sample;
//			const int n1 = rows - 30;
//			const int m1 = cols - 30;
//#pragma omp for nowait
//			for (int n = m0; n < n1; ++n) {
//				const int sample0 = ((n - m0) % sample) == 0 ? 1 : sample;
//				for (int m = m0; m < m1; m += sample0) {
//					double theta;
//					if (Itex[n][m] != 100) {
//						theta = 3.1416 * (double) ((double) I2[n][m] / 255
//								- 0.5);
//					} else if (Ior[n][m] != 100) {
//						theta = 3.1416 * (double) ((double) I2[n][m] / 255
//								- 0.5);
//					} else
//						continue;
//
//					norm[0] = (rendparam.Normx)
//							* ((double) ((double) Inormd[n][m].R) / 255
//									- 0.5);
//					norm[1] = (rendparam.Normy)
//							* ((double) ((double) Inormd[n][m].G) / 255
//									- 0.5);
//					norm[2] = (rendparam.Normz)
//							* ((double) ((double) Inormd[n][m].B) / 255
//									- 0.5);
//					//std::cout<<norm[0]<<" "<<norm[1]<<" "<<norm[2]<<std::endl;
//					const double nrm = std::sqrt(norm[0] * norm[0] + norm[1]
//							* norm[1] + norm[2] * norm[2]);
//					if (nrm > 1e-1) {
//						double Z = -(znear * zfar)
//								/ (((double) ((double) Inormd[n][m].A)
//										/ 255) * (zfar - znear) - zfar);
//
//						bool flag = false;
//						il = 0;
//						double distmin = 1000;
//						double dist = 0, dist0 = 0, dist1 = 0;
//						int kmin;
//						double dtheta0, dtheta1, dtheta2;
//						x = (m - jc) / px;
//						y = (n - ic) / py;
//						for (int k = 0; k < lines[scaleLevel].size(); k++) {
//							apMbtDistanceLineMH *l = lines[scaleLevel][k];
//							lineL = l->line;
//							lineL->changeFrame(cMo);
//							lineL->projection();
//							rho_l = lineL->getRho();
//							theta_l = lineL->getTheta();
//							rho = x * cos(theta) + y * sin(theta);
//							zb1
//									= ((double) ((double) Inormd[linesV[il][1]][linesV[il][0]].A)
//											/ 255);
//							zb2
//									= ((double) ((double) Inormd[linesV[il][3]][linesV[il][2]].A)
//											/ 255);
//							if (linesV[il][1] > linesV[il][3]) {
//								bi1 = linesV[il][3];
//								bi2 = linesV[il][1];
//							} else {
//								bi2 = linesV[il][3];
//								bi1 = linesV[il][1];
//							}
//
//							if (linesV[il][0] > linesV[il][2]) {
//								bj1 = linesV[il][2];
//								bj2 = linesV[il][0];
//							} else {
//								bj2 = linesV[il][2];
//								bj1 = linesV[il][0];
//							}
//
//							//std::cout << " ok " << rho << " rho l " << rho_l << " tetha " << theta_l <<std::endl;
//							dist0 = abs(rho - rho_l);
//							dist1 = abs(rho + rho_l);
//							if (dist0 < dist1)
//								dist = dist0;
//							else
//								dist = dist1;
//
//							if ((dist < distmin) && (n <= bi2 && n >= bi1 && m
//									<= bj2 && m >= bj1)) //  && abs(theta - theta_l) < 0.1)
//							{
//								distmin = dist;
//								kmin = k;
//								//std::cout << " distmin " << distmin << std::endl;
//								dtheta0 = abs(theta - theta_l);
//								dtheta1 = abs(theta - theta_l - M_PI);
//								dtheta2 = abs(theta - theta_l + M_PI);
//							}
//							il++;
//						}
//
//						if ((distmin < 0.0025) && (dtheta0 < 0.3 || dtheta1
//								< 0.3 || dtheta2 < 0.3)) {
//							flag = true;
//							apControlPoint *p0 = new apControlPoint;
//							p0->setCameraParameters(&cam);
//							p0->setMovingEdge(&me);
//							p0->buildPoint(n, m, Z, theta, norm, cMo);
//							apMbtDistanceLineMH *l = lines[scaleLevel][kmin];
//							l->pointsvect.push_back(p0);
//							ip.set_i(n);
//							ip.set_j(m);
//							//if (sqrt((linesV[kmin][1] -linesV[kmin][3])*(linesV[kmin][1] -linesV[kmin][3]) + (linesV[kmin][0] -linesV[kmin][2])*(linesV[kmin][0] -linesV[kmin][2]))>34 && sqrt((linesV[kmin][1] -linesV[kmin][3])*(linesV[kmin][1] -linesV[kmin][3]) + (linesV[kmin][0] -linesV[kmin][2])*(linesV[kmin][0] -linesV[kmin][2]))<35)
//							{
//								vpDisplay::displayCross(I, n, m, 2,
//										vpColor::blue);
//								kdisp++;
//							}
//
//							//vpDisplay::displayCross(I, ip, 2, vpColor::red, 2);
//						}
//
//						if (flag == false) {
//
//						apControlPoint *p = new apControlPoint;
//						p->setCameraParameters(&cam);
//						p->setMovingEdge(&me);
//						p->buildPoint(n, m, Z, theta, norm, cMo);
//						p->initControlPoint(I, 0);
//						local_points.push_back(p);
////						apControlPoint *pccd = new apControlPoint;
////						pccd->setCameraParameters(&cam);
////						pccd->setMovingEdge(&me);
////						pccd->buildSilhouettePoint(n, m, Z, theta, norm,
////								cMo);
////						pccd->initControlPoint(Isil, 0);
////						pccd->detectSilhouette(Isil,I, 0);
////						if (pccd->isSilhouette)
////						{
////							local_pointsCCD.push_back(pccd);
////						}
////						else
////							delete pccd;
//						}
//					}
//				}
//			}
//
//			if (!local_points.empty())
//#pragma omp critical
//			{
//				npoints += local_points.size();
//				points[i].insert(points[i].end(), local_points.begin(),
//						local_points.end());
////				CCDTracker.pointsCCD[i].insert(
////						CCDTracker.pointsCCD[i].end(),
////						local_pointsCCD.begin(), local_pointsCCD.end());
////				CCDTracker.npointsCCD += local_pointsCCD.size();
//			}
//
//			std::cout << " kdisp " << points[i].size() << std::endl;
//			for (int k = 0; k < lines[scaleLevel].size(); k++) {
//				apMbtDistanceLineMH *l = lines[scaleLevel][k];
//				//std::cout << " ok " << l->pointsvect.size() << " length " << sqrt((linesV[k][1] -linesV[k][3])*(linesV[k][1] -linesV[k][3]) + (linesV[k][0] -linesV[k][2])*(linesV[k][0] -linesV[k][2])) << std::endl;
//				l->initMovingEdgeMHP(I, gradMap, cMo);
//			}
//
//			std::cout << " lines size " <<  lines[scaleLevel].size() << std::endl;
//
//			//getchar();
//
//			std::cout << " time " << t1 - t0 << std::endl;
//
//
//		}
//#pragma omp barrier
//#pragma omp master
//		upScale(i);
//	}
//}
//
//}


void apMbTracker::extractControlPointsLinesCCD(
		const vpImage<unsigned char> &I, const vpImage<vpRGBa>& Inormd,
		const vpImage<unsigned char>& Ior, const vpImage<unsigned char>& Itex,
		const double dist) {
	apControlPoint *p;
	apControlPoint *p0;
	vpImagePoint ip, ip1, ip2;
	apMbtDistanceLineMH *l;
	vpLine *lineL;
	double t0, t1;
	int bi1, bi2, bj1, bj2;
	double px = cam.get_px();
	double py = cam.get_py();
	int jc = cam.get_u0();
	int ic = cam.get_v0();
	int i1, j1;
	vpPoint Pc;
	double zb1, zb2;
	vpColVector Pc0(3);
	vpColVector ip0(2);
	double x, y, Z;
	vpColVector Normc(3);
	double rho, theta, rho_l, theta_l;
	int sample = rendparam.sampleR;
	double znear = dist - rendparam.clipDist;
	double zfar = dist + rendparam.clipDist;
	zn = znear;
	zf = zfar;
	vpImage<unsigned char> Ig;
	Ig = I;

	int rows = Ior.getHeight();
	int cols = Ior.getWidth();

	vpImage<unsigned char> I2(rows, cols);
	vpImage<unsigned char> I2_(rows, cols);
	vpImage<unsigned char> Isil(rows, cols);
	I2_ = Ior;
	process(Inormd, I2);
	//process1(Inormd,I2_);

	if(rendparam.useNPoints)
	{
	int npointsparam = rendparam.nPoints;
	int ntotalpoints = 0;
	bool flag = false;
	for (int n = 0; n < rows; n++)
		for (int m = 0; m < cols; m++)
		{
			flag = false;
	if (Itex[n][m] != 100) {
		ntotalpoints++;
		flag = true;
	} else if (Ior[n][m] != 100) {
		if (!flag)
		ntotalpoints++;
	} else
		continue;
		}
	if(npointsparam < ntotalpoints)
	{
     sample = 2*floor((double)ntotalpoints/npointsparam);
	}
	else{
	 sample = 1;
	}
	std::cout << " sample " << sample << " ntotalpoints " << ntotalpoints << std::endl;
	}

//#pragma omp parallel
	{
	std::vector<apControlPoint*> local_points;
	std::vector<apControlPoint*> local_pointsCCD;
	vpColVector norm(3);

	for (unsigned int i = 0; i < scales.size(); i += 1) {
#pragma omp barrier
		if (scales[i]) {
#pragma omp master
			downScale(i);

#pragma omp for nowait
			for (int k = 0; k < points[i].size(); k++) {
				p = (points[i])[k];
				if (p != NULL)
					delete p;
				p = NULL;
			}
#pragma omp master
			points[i].clear();

			for (int k = 0; k < lines[i].size(); k++) {
				l = lines[i][k];
				if (l != NULL) {
					for (int ii; ii < l->pointsvect.size(); ii++) {
						p0 = l->pointsvect[ii];
						delete p0;
					}
					l->pointsvect.resize(0);
					delete l;
				}
				l = NULL;
			}
			lines[i].clear();

#pragma omp for nowait
				for (int k = 0; k < CCDTracker.pointsCCD[i].size(); k++) {
					apControlPoint *pccd = CCDTracker.pointsCCD[i][k];
					if (pccd != NULL)
						delete pccd;
				}

#pragma omp master
				CCDTracker.pointsCCD[i].clear();
#pragma omp for
				for (int k = 0; k < rows; k++)
					for (int l = 0; l < cols; l++) {
						if (Inormd[k][l].A != 0)
							Isil[k][l] = Inormd[k][l].A;
						else
							Isil[k][l] = 255;
					}

				local_points.clear();
				local_pointsCCD.clear();

			double n_sample;
			vpRotationMatrix R;
			cMo.extract(R);
			//vpColVector norm(3);
			int step = 1;
			//int range;
			int il = 0;

			gradMap.resize(rows, cols);
			vpImage<unsigned char> imG(rows, cols);
			//vpImage<unsigned char> imGrad(rows, cols);
			//vpImage<unsigned char> IorG(rows, cols);
			//IorG = Ior;
			double a, b;
			for (int nn = 3; nn < rows - 3; nn++) {
				for (int mm = 3; mm < cols - 3; mm++) {
					imG[nn][mm] = vpImageFilter::gaussianFilter(Ig, nn, mm);
					//IorG[nn][mm] = vpImageFilter::gaussianFilter(Ior, nn, mm);
				}
			}
			for (int ii = 3; ii < rows - 3; ii++) {
				for (int jj = 3; jj < cols - 3; jj++) {
					a = (apImageFilter::sobelFilterX(imG, ii, jj));
					b = (apImageFilter::sobelFilterY(imG, ii, jj));
					if ((a != 0 || b != 0) && sqrt((vpMath::sqr(a)
							+ vpMath::sqr(b))) > 0)
						gradMap[ii][jj] = 255 * (-(atan(a / b)) / M_PI + 1 / 2);
					else
						gradMap[ii][jj] = 0;
				}
			}
			//gradMap = imGrad;
			int w = 0;

			vpImage<unsigned char> Ip0(rows, cols);
			for (int n = 0; n < rows; n++) {
				for (int m = 0; m < cols; m++) {
					if (Ior[n][m] != 100)
						Ip0[n][m] = 255;
					else
						Ip0[n][m] = 0;
				}
			}

                        cv::Mat Ip, Ip1;
			vpImageConvert::convert(Ip0, Ip);
			Mat color_dst;
			vector<Vec4i> linesV;
			Mat dst(Ip);
			cvtColor(dst, color_dst, CV_GRAY2BGR);
			HoughLinesP(dst, linesV, 1, CV_PI / 180, 40, 40, 5);
			for (size_t i = 0; i < linesV.size(); i++) {
				line(color_dst, Point(linesV[i][0], linesV[i][1]), Point(
						linesV[i][2], linesV[i][3]), Scalar(255, 0, 255), 2, 8);
			}
			loadLines(I, linesV, Inormd, cMo);
			/*vpImage<vpRGBa> Ioverlay;
			 Ip1 = new IplImage(color_dst);
			 vpImage<unsigned char> I3(rows, cols);
			 vpImageConvert::convert(Ip1, I3);
			 delete Ip1;
			 vpDisplayX display;
			 display.init(I3, 1500, 10, "Hough");
			 vpDisplay::display(I3);
			 vpDisplay::flush(I3);
			 vpDisplay::getImage(I3,Ioverlay);
			 std::cout << " ok " << std::endl;
			 vpImageIo::writePNG(I2_, "ihough0.png");
			 vpDisplay::getClick(I3);*/
			//getchar();
			int kdisp = 0;

			/* for (unsigned int i = 0; i < scales.size(); ++i)
			 {
			 if (scales[i])
			 {
			 #pragma omp master
			 downScale(i);
			 #pragma omp barrier

			 #pragma omp for
			 for (int k = 0; k < points[i].size(); ++k)
			 {
			 apControlPoint *p = (points[i])[k];
			 if (p != NULL)
			 delete p;
			 }
			 #pragma omp master
			 points[i].clear();*/

			const int m0 = (20 + sample - 1) / sample * sample;
			const int n1 = rows - 20;
			const int m1 = cols - 20;

			//t1 = vpTime::measureTimeMs();

			//#pragma omp for nowait
			for (int n = m0; n < n1; ++n) {
				const int sample0 = ((n - m0) % sample) == 0 ? 1 : sample;
				for (int m = m0; m < m1; m += sample0) {
					double theta;
					if (Itex[n][m] != 100)
						theta = 3.1416 * (double) ((double) I2[n][m] / 255
								- 0.5);
					else if (Ior[n][m] != 100)
						theta = 3.1416 * (double) ((double) I2[n][m] / 255
								- 0.5);
					else
						continue;

					norm[0] = (rendparam.Normx)
							* ((double) ((double) Inormd[n][m].R) / 255 - 0.5);
					norm[1] = (rendparam.Normy)
							* ((double) ((double) Inormd[n][m].G) / 255 - 0.5);
					norm[2] = (rendparam.Normz)
							* ((double) ((double) Inormd[n][m].B) / 255 - 0.5);
					const double nrm = std::sqrt(norm[0] * norm[0] + norm[1]
							* norm[1] + norm[2] * norm[2]);
					if (nrm > 1e-1) {
						double Z = -(znear * zfar)
								/ (((double) ((double) Inormd[n][m].A) / 255)
										* (zfar - znear) - zfar);
						bool flag = false;
						il = 0;
						double distmin = 1000;
						double dist = 0, dist0 = 0, dist1 = 0;
						int kmin;
						double dtheta0, dtheta1, dtheta2;
						x = (m - jc) / px;
						y = (n - ic) / py;
						for (int k = 0; k < lines[scaleLevel].size(); k++) {
							l = lines[scaleLevel][k];
							lineL = l->line;
							lineL->changeFrame(cMo);
							lineL->projection();
							rho_l = lineL->getRho();
							theta_l = lineL->getTheta();
							rho = x * cos(theta) + y * sin(theta);
							//zb1= ((double) ((double) Inormd[linesV[il][1]][linesV[il][0]].A)/ 255);
							//zb2= ((double) ((double) Inormd[linesV[il][3]][linesV[il][2]].A)/ 255);
							if (linesV[il][1] > linesV[il][3]) {
								bi1 = linesV[il][3];
								bi2 = linesV[il][1];
							} else {
								bi2 = linesV[il][3];
								bi1 = linesV[il][1];
							}

							if (linesV[il][0] > linesV[il][2]) {
								bj1 = linesV[il][2];
								bj2 = linesV[il][0];
							} else {
								bj2 = linesV[il][2];
								bj1 = linesV[il][0];
							}

							//std::cout << " ok " << rho << " rho l " << rho_l << " tetha " << theta_l <<std::endl;
							dist0 = abs(rho - rho_l);
							dist1 = abs(rho + rho_l);
							if (dist0 < dist1)
								dist = dist0;
							else
								dist = dist1;

							if ((dist < distmin) && (n <= bi2 && n >= bi1 && m
									<= bj2 && m >= bj1)) //  && abs(theta - theta_l) < 0.1)
							{
								distmin = dist;
								kmin = k;
								//std::cout << " distmin " << distmin << std::endl;
								dtheta0 = abs(theta - theta_l);
								dtheta1 = abs(theta - theta_l - M_PI);
								dtheta2 = abs(theta - theta_l + M_PI);
							}
							il++;
						}
						//if ((distmin < 0.0015) && (dtheta0 < 0.2 || dtheta1 < 0.2 || dtheta2 < 0.2 ))
						if ((distmin < 0.0025) && (dtheta0 < 0.3 || dtheta1< 0.3 || dtheta2 < 0.3)) {
							//if ((distmin < 0.0055) && (dtheta0 < 0.5 || dtheta1< 0.5 || dtheta2 < 0.5)) {
							flag = true;
							p0 = new apControlPoint;
							p0->setCameraParameters(&cam);
							p0->setMovingEdge(&me);
							p0->buildPoint(n, m, Z, theta, norm, cMo);
							l = lines[scaleLevel][kmin];
							l->pointsvect.push_back(p0);
							ip.set_i(n);
							ip.set_j(m);
							//if (sqrt((linesV[kmin][1] -linesV[kmin][3])*(linesV[kmin][1] -linesV[kmin][3]) + (linesV[kmin][0] -linesV[kmin][2])*(linesV[kmin][0] -linesV[kmin][2]))>34 && sqrt((linesV[kmin][1] -linesV[kmin][3])*(linesV[kmin][1] -linesV[kmin][3]) + (linesV[kmin][0] -linesV[kmin][2])*(linesV[kmin][0] -linesV[kmin][2]))<35)
							{
								//vpDisplay::displayCross(I, n, m, 2, vpColor::blue);
								kdisp++;
							}

							//vpDisplay::displayCross(I, ip, 2, vpColor::red, 2);
						}

						if (flag == false) {
							p = new apControlPoint;
							p->setCameraParameters(&cam);
							p->setMovingEdge(&me);
							p->buildPoint(n, m, Z, theta, norm, cMo);
							p->initControlPoint(I, 0);
							//npoints += 1;
							//points[i].push_back(p);
							local_points.push_back(p);
						}

						apControlPoint *pccd = new apControlPoint;
						pccd->setCameraParameters(&cam);
						pccd->setMovingEdge(&me);
						pccd->buildSilhouettePoint(n, m, Z, theta, norm,
															cMo);
						pccd->initControlPoint(Isil, 0);
						pccd->detectSilhouette(Isil,I, 0);
						if (pccd->isSilhouette)
							local_pointsCCD.push_back(pccd);
						else
						delete pccd;

					}
					w++;
				}
			}
			//#pragma omp critical
			if (!local_points.empty()) {
				npoints += local_points.size();
				points[i].insert(points[i].end(), local_points.begin(),
						local_points.end());
				CCDTracker.pointsCCD[i].insert(
						CCDTracker.pointsCCD[i].end(),
						local_pointsCCD.begin(), local_pointsCCD.end());
				CCDTracker.npointsCCD += local_pointsCCD.size();
			}

			std::cout << " kdisp " << points[i].size() << std::endl;
			for (int k = 0; k < lines[scaleLevel].size(); k++) {
				l = lines[scaleLevel][k];
				//std::cout << " ok " << l->pointsvect.size() << " length " << sqrt((linesV[k][1] -linesV[k][3])*(linesV[k][1] -linesV[k][3]) + (linesV[k][0] -linesV[k][2])*(linesV[k][0] -linesV[k][2])) << std::endl;
				l->initMovingEdgeMHP(I, gradMap, cMo);
			}
			//getchar();

			std::cout << " time " << t1 - t0 << std::endl;

		}
		upScale(i);
	}
}

}


void apMbTracker::extractKltControlPointsFAST(
		const vpImage<unsigned char> &I, const vpImage<vpRGBa>& Inormd, const vpImage<unsigned char> &Ior,const vpImage<unsigned char> &Itex,
		const double dist) {

	int sample = rendparam.sampleR;
	double znear = dist - rendparam.clipDist;
	double zfar = dist + rendparam.clipDist;

	int rows = I.getHeight();
	int cols = I.getWidth();
	vpImage<unsigned char> Isil(rows,cols);
	vpImage<unsigned char> Ibound(rows,cols);

	nkltPoints = 0;

	/*int borni_inf = 0;
	int borni_sup = rows;
	int bornj_inf = 0;
	int bornj_sup = cols;*/

	double t0 = vpTime::measureTimeMs();


	for (int k = 0; k < rows; k++)
		for (int l = 0; l < cols; l++) {
			if(Inormdprec[k][l].A != 0)
			{
				Isil[k][l] = 255;
			}
			else Isil[k][l] = 0;
		}

        cv::Rect rect;
        std::vector<std::vector<cv::Point> > contours;
        vector< Vec4i> hierarchy;
        cv::Mat frame_1;
       vpImageConvert::convert(Isil,frame_1);
/*    std::vector<vpImagePoint> pointssil;
    pointssil.resize(0);
vpImagePoint pt;
	bool flag = false;
	for (int k = 0; k < rows; k++)
		for (int l = 0; l < cols; l++) {
			if(Iorprec[k][l] != 100)
			{
				pt.set_i(k);
				pt.set_j(l);
				pointssil.push_back(pt);
			}
		}*/
        cv::findContours(frame_1, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE );

	int area = 0;
	int indx;
	int kkk =0;
	int wd,hg,xx,yy;
        for(int ic; ic < contours.size(); ic++)
	{
                rect = cv::boundingRect(contours[ic]);
	    if(rect.height*rect.width>area && rect.height < 450)
	    {
	        indx = kkk;
	        area = rect.height*rect.width;
	        wd = rect.width;
	        hg = rect.height;
	        xx = rect.x;
	        yy = rect.y;
	    }
	    kkk++;
	}

	Ibound.resize(hg,wd);
	for (int k = 0; k < hg; k++)
		for (int l = 0; l < wd; l++) {
			{
				Ibound[k][l] = Iprec[k+yy][l+xx];
			}
		}

	vpKltOpencv kltTracker1;
	kltTracker1.setTrackerId(1);//1
	kltTracker1.setMaxFeatures(KLTTrackerParameters.npoints);		//nbPoint
	kltTracker1.setWindowSize(KLTTrackerParameters.windowsize);				//6//10
	kltTracker1.setQuality(KLTTrackerParameters.quality);				//0.00000001//0.01
	kltTracker1.setMinDistance(KLTTrackerParameters.mindist);				//10//15
	kltTracker1.setHarrisFreeParameter(KLTTrackerParameters.harrisfree);	//0.1//0.04
	kltTracker1.setBlockSize(KLTTrackerParameters.blocksize);				//9
	kltTracker1.setUseHarris(KLTTrackerParameters.useharris);				//1
	kltTracker1.setPyramidLevels(KLTTrackerParameters.pyramidlevels);

	// Calculates the bounding rect of the largest area contour
	std::cout << " time extract harris corners " << xx << " " << yy << std::endl;

	double t1 = vpTime::measureTimeMs();
	std::cout << " time extract harris corners " << t1 - t0 << std::endl;
	 //vpImageIo::writePNG(Ibound, "isil00.png");

	if( frame0 == 0 || frame0%20 == 0)
	{
	c0Mo = cMo;
	ctTc0.setIdentity();
	}

//	#pragma omp parallel
	{
		std::map<int, apKltControlPoint> curPoints = std::map<int, apKltControlPoint>();
		std::map<int, int> curPointsInd = std::map<int, int>();

		vpColVector norm(3);
		for (unsigned int i = 0; i < scales.size(); ++i) {
//#pragma omp barrier
			if (scales[i]) {
//#pragma omp master
				downScale(i);
//#pragma omp barrier

				std::map<int, vpImagePoint> curpoints0;
				vpImagePoint ip(0,0);
				curpoints0[0] = ip;

				const int m0 = (30 + sample - 1) / sample * sample;
				const int n1 = rows - 30;
				const int m1 = cols - 30;

				float x,y;
                                long id;
                                int n,m;
				id =0;
				double t0 = vpTime::measureTimeMs();


				if (frame0 <= 1 || frame0%1 == 0)
				{
					kltPoints[i].clear();
                                        kltPoints[i] = std::map<int, apKltControlPoint>();
                                        cv::Mat frame_;
					vpImageConvert::convert(Ibound,frame_);
					kltTracker1.initTracking(frame_);
				}

				double t1 = vpTime::measureTimeMs();


				std::cout << " time extract harris corners " << t1 - t0 << std::endl;


//#pragma omp for
				for (unsigned int k = 0; k < static_cast<unsigned int>(kltTracker.getNbFeatures()); k++){

				    kltTracker1.getFeature((int)k, id, x, y);
				    //kltTracker.getFeature()[k] = kltTracker1.getFeature()[k];
				    	kltTracker.getFeatures()[k].x = x + xx;
				        kltTracker.getFeatures()[k].y = y + yy;

				 kltTracker.getFeature((int)k, id, x, y);
				 n = (int)y;
				 m = (int)x;

				 if(n > 0 && n < Inormd.getHeight() && m > 0 && m < Inormd.getWidth())
				 {

						norm[0] = (rendparam.Normx)
								* ((double) ((double) Inormdprec[n][m].R) / 255
										- 0.5);
						norm[1] = (rendparam.Normy)
								* ((double) ((double) Inormdprec[n][m].G) / 255
										- 0.5);
						norm[2] = (rendparam.Normz)
								* ((double) ((double) Inormdprec[n][m].B) / 255
										- 0.5);
						const double l = std::sqrt(norm[0] * norm[0] + norm[1]
								* norm[1] + norm[2] * norm[2]);
						double Z = -(znear * zfar)
								/ (((double) ((double) Inormdprec[n][m].A)
										/ 255) * (zfar - znear) - zfar);
						if (Inormdprec[n][m].A!= 255 && l > 1e-1 && Iorprec[n][m] == 100 && Itexprec[n][m] == 100) {
							if( frame0 <= 1 || frame0%1 == 0)
							{
							apKltControlPoint p;
							p.setCameraParameters(cam);
							p.setMovingEdge(me);
							//p.initPoint(n, m, Z, norm, cMo);
							//p.buildPoint(n, m, Z, norm, cMo);
							p.buildPoint(n, m, Z, norm, cMoprec);
							kltPoints[i][id] = p;
							nkltPoints++;
							}
							/*else
							{
							apKltControlPoint p;
							p = kltPoints[i][id];
							p.buildPoint(n, m, Z, norm, cMoprec);
							kltPoints[i][id] = p;}*/
						}
				}
			}

/*
				if (!curPoints.empty())
#pragma omp critical
				{
					nkltPoints += curPoints.size();
					kltPoints[i].insert(kltPoints[i].end(), curPoints.begin(),
							curPoints.end());
				}
*/
			}
//#pragma omp barrier
//#pragma omp master
			upScale(i);
		}
	}
}

void apMbTracker::extractKltControlPoints(
		const vpImage<unsigned char> &I, const vpImage<vpRGBa>& Inormd, const vpImage<unsigned char> &Ior,const vpImage<unsigned char> &Itex,
		const double dist) {

	int sample = rendparam.sampleR;
	double znear = dist - rendparam.clipDist;
	double zfar = dist + rendparam.clipDist;

	int rows = I.getHeight();
	int cols = I.getWidth();

	if( frame0 == 0 || frame0%20 == 0)
	{
	c0Mo = cMo;
	ctTc0.setIdentity();
	}

	nkltPoints =0;

	//#pragma omp parallel
	{
		std::map<int, apKltControlPoint> curPoints = std::map<int, apKltControlPoint>();
		std::map<int, int> curPointsInd = std::map<int, int>();

		vpColVector norm(3);
		for (unsigned int i = 0; i < scales.size(); ++i) {
//#pragma omp barrier
			if (scales[i]) {
//#pragma omp master
				downScale(i);
//#pragma omp barrier

				std::map<int, vpImagePoint> curpoints0;
				vpImagePoint ip(0,0);
				curpoints0[0] = ip;

				const int m0 = (30 + sample - 1) / sample * sample;
				const int n1 = rows - 30;
				const int m1 = cols - 30;

				float x,y;
                                long id;
                                int n,m;
				id =0;


				//if (frame0 <= 1 || frame0%20 == 0)
				{
					kltPoints[i].clear();
					kltPoints[i] = std::map<int, apKltControlPoint>();				
                                        cv::Mat frame_;
					vpImageConvert::convert(Iprec,frame_);
					kltTracker.initTracking(frame_);
				}

				for (unsigned int k = 0; k < static_cast<unsigned int>(kltTracker.getNbFeatures()); k++){

				 kltTracker.getFeature((int)k, id, x, y);
				 n = (int)y;
				 m = (int)x;

				 if(n > 0 && n < Inormd.getHeight() && m > 0 && m < Inormd.getWidth())
				 {

						norm[0] = (rendparam.Normx)
								* ((double) ((double) Inormdprec[n][m].R) / 255
										- 0.5);
						norm[1] = (rendparam.Normy)
								* ((double) ((double) Inormdprec[n][m].G) / 255
										- 0.5);
						norm[2] = (rendparam.Normz)
								* ((double) ((double) Inormdprec[n][m].B) / 255
										- 0.5);
						const double l = std::sqrt(norm[0] * norm[0] + norm[1]
								* norm[1] + norm[2] * norm[2]);
						double Z = -(znear * zfar)
								/ (((double) ((double) Inormdprec[n][m].A)
										/ 255) * (zfar - znear) - zfar);
						if (Inormdprec[n][m].A!= 255 && l > 1e-1 && Iorprec[n][m] == 100 && Itexprec[n][m] == 100) {
							//if( frame0 <= 1 || frame0%20 == 0)
							{
							apKltControlPoint p;
							p.setCameraParameters(cam);
							p.setMovingEdge(me);
							//p.initPoint(n, m, Z, norm, cMo);
							//p.buildPoint(n, m, Z, norm, cMo);
							p.buildPoint(n, m, Z, norm, cMoprec);
							kltPoints[i][id] = p;
							nkltPoints++;
							}
							/*else
							{
							apKltControlPoint p;
							p = kltPoints[i][id];
							p.buildPoint(n, m, Z, norm, cMo);
							kltPoints[i][id] = p;}*/
						}
				}
			}


/*
				if (!curPoints.empty())
#pragma omp critical
				{
					nkltPoints += curPoints.size();
					kltPoints[i].insert(kltPoints[i].end(), curPoints.begin(),
							curPoints.end());
				}
*/
			}
//#pragma omp barrier
//#pragma omp master
			upScale(i);
		}
	}
}

void apMbTracker::loadLines(const vpImage<unsigned char>& I,
		vector<Vec4i>& Lines, const vpImage<vpRGBa>& Inormd,
		const vpHomogeneousMatrix &cMo) {
	vpPoint P1, P2, p1;
	vpImagePoint ip1, ip2;
	vpColVector P10, P20;
	double x1, y1, x2, y2, zb1, zb2, z1, z2;
	apMbtDistanceLineMH *l;
	for (size_t i = 0; i < Lines.size(); i++) {
	//for (size_t i = 0; i < 1; i++) {
		ip1.set_i(Lines[i][1]);
		ip1.set_j(Lines[i][0]);
		ip2.set_i(Lines[i][3]);
		ip2.set_j(Lines[i][2]);
		zb1 = ((double) ((double) Inormd[Lines[i][1]][Lines[i][0]].A) / 255);
		zb2 = ((double) ((double) Inormd[Lines[i][3]][Lines[i][2]].A) / 255);
		z1 = -(zn * zf) / (zb1 * (zf - zn) - zf);
		z2 = -(zn * zf) / (zb2 * (zf - zn) - zf);
		vpPixelMeterConversion::convertPoint(cam, ip1, x1, y1);
		vpPixelMeterConversion::convertPoint(cam, ip2, x2, y2);
		P1.setWorldCoordinates(x1 * z1, y1 * z1, z1);
		P2.setWorldCoordinates(x2 * z2, y2 * z2, z2);
		P1.changeFrame(cMo.inverse(), P10);
		P2.changeFrame(cMo.inverse(), P20);
		P1.setWorldCoordinates(P10[0], P10[1], P10[2]);
		P2.setWorldCoordinates(P20[0], P20[1], P20[2]);
		vpDisplay::displayLine(I, ip1, ip2, vpColor::blue, 1);
		//vpDisplay::displayCross(I,ip2,2,vpColor::red,1);
		std::string nm = "jj";
		addLine(I, P1, P2, -1, nm);
	}
}

void apMbTracker::loadLinesLm(const vpImage<unsigned char>& I,
		vector<Vec4i>& Lines, const vpImage<vpRGBa>& Inormd,
		const vpHomogeneousMatrix &cMo) {
	/*vpPoint P1, P2,P1c,P2c,p1;
	vpImagePoint ip1, ip2;
	vpColVector P10, P20;
	double x1, y1, x2, y2, zb1, zb2, z1, z2;
	apMbtDistanceLineMH *l;
	for (size_t i = 0; i < Lines.size(); i++) {
		if (Inormd[Lines[i][1]][Lines[i][0]].A==255 && Inormd[Lines[i][3]][Lines[i][2]].A==255)
		{
	//for (size_t i = 0; i < 1; i++) {
		ip1.set_i(Lines[i][1]);
		ip1.set_j(Lines[i][0]);
		ip2.set_i(Lines[i][3]);
		ip2.set_j(Lines[i][2]);
		zb1 = ((double) ((double) Inormd[Lines[i][1]][Lines[i][0]].A) / 255);
		zb2 = ((double) ((double) Inormd[Lines[i][3]][Lines[i][2]].A) / 255);
		z1 = cMo[2][3];
		z2 = cMo[2][3];
		vpPixelMeterConversion::convertPoint(cam, ip1, x1, y1);
		vpPixelMeterConversion::convertPoint(cam, ip2, x2, y2);
		P1c.setWorldCoordinates(x1 * z1, y1 * z1, z1);
		P2c.setWorldCoordinates(x2 * z2, y2 * z2, z2);
		P1c.changeFrame(cMo.inverse(), P10);
		P2c.changeFrame(cMo.inverse(), P20);
		P1.setWorldCoordinates(P10[0], P10[1], P10[2]);
		P2.setWorldCoordinates(P20[0], P20[1], P20[2]);
		vpDisplay::displayLine(I, ip1, ip2, vpColor::red, 1);
		//vpDisplay::displayCross(I,ip2,2,vpColor::red,1);
		std::string nm = "jj";
		bool already_here = false;
		apMbtDistanceLineMH *l;
		for (unsigned int i = 0; i < scales.size(); i += 1) {
			if (scales[i]) {
				downScale(i);
				l = new apMbtDistanceLineMH;
				l->setCameraParameters(&cam);
				l->buildFrom(P1, P2);
				l->initEKF(P1c,P2c,cMo);
				l->Lindex_polygon.push_back(-1);
				l->setMovingEdgeMH(&me);
				l->setName(nm);
				nline += 1;
				//newLinesLm[i].push_back(l);
				upScale(i);
			}
		}		}
	}*/
}


void apMbTracker::addLine(const vpImage<unsigned char> &I, vpPoint &P1,
		vpPoint &P2, int polygone, std::string &name) {
	bool already_here = false;
	apMbtDistanceLineMH *l;
	for (unsigned int i = 0; i < scales.size(); i += 1) {
		if (scales[i]) {
			downScale(i);
			l = new apMbtDistanceLineMH;
			l->setCameraParameters(&cam);
			l->buildFrom(P1, P2);
			l->Lindex_polygon.push_back(polygone);
			l->setMovingEdgeMH(&me);
			l->setName(name);
			//l->initMovingEdge(I,cMo);
			//l->initMovingEdgeMHG(I,gradMap,cMo);
			nline += 1;
			lines[i].push_back(l);
			upScale(i);
		}
	}
}

/*!
 Load a 3D model contained in a file.

 \param file : Path to the file containing the 3D model description.
 */
void apMbTracker::loadModel(const char* file) {
	std::string model(file);
	vpMbTracker::loadModel(model);
}

/*void
 apMbTracker::loadLines(const vpImage<unsigned char>& I, vector<Vec4i>& Lines, const vpMatrix &Zc, const vpHomogeneousMatrix &cMo)
 {
 vpPoint P1,P2,p1;
 vpImagePoint ip1,ip2;
 vpColVector P10,P20;
 double x1,y1,x2,y2,z1,z2;
 vpMbtDistanceLine *l;
 for( size_t i = 0; i < Lines.size(); i++)
 {
 ip1.set_i(Lines[i][1]-8);
 ip1.set_j(Lines[i][0]+18);
 ip2.set_i(Lines[i][3]-8);
 ip2.set_j(Lines[i][2]+18);
 z1=Zc[Lines[i][1]][Lines[i][0]]/2;
 z2=Zc[Lines[i][3]][Lines[i][2]]/2;
 vpPixelMeterConversion::convertPoint(cam,ip1,x1,y1);
 vpPixelMeterConversion::convertPoint(cam,ip2,x2,y2);
 P1.setWorldCoordinates(x1*z1,y1*z1,z1);
 P2.setWorldCoordinates(x2*z2,y2*z2,z2);
 P1.changeFrame(cMo.inverse(),P10);
 P2.changeFrame(cMo.inverse(),P20);
 P1.setWorldCoordinates(P10[0],P10[1],P10[2]);
 P2.setWorldCoordinates(P20[0],P20[1],P20[2]);
 std::cout<<" ok "<<z1<<std::endl;
 vpDisplay::displayPoint(I,ip1,vpColor::red);
 addLine(P1,P2,-1);
 }
 lines[0].front();
 l=lines[0].value();
 l->p1;
 //std::cout<<" ok "<<z1<<std::endl;
 //std::cout<< "ok " << lines[0].size() << std::endl;
 //getchar();
 }

 /*!
 Reset the tracker. The model is removed and the pose is set to identity.
 The tracker needs to be initialized with a new model and a new pose.

 */
void apMbTracker::resetTracker() {
	this->cMo.setIdentity();
	apControlPoint *p;

	for (unsigned int i = 0; i < scales.size(); i += 1) {
		if (scales[i]) {
			for (int k = 0; k < points[i].size(); k++) {
				p = (points[i])[k];
				if (p != NULL)
					delete p;
				p = NULL;
			}
		}
	}

	//faces.reset();

	//index_polygon =0;
	compute_interaction = 1;
	npoints = 0;
	lambda = 1;
	//nbvisiblepolygone = 0;
	percentageGdPt = 0.4;

	// reinitialisation of the scales.
	this->setScales(scales);
}

/*!
 Re-initialise the model used by the tracker.

 \param _I : The image containing the object to initialize.
 \param _cad_name : Path to the file containing the 3D model description.
 \param _cMo : The new vpHomogeneousMatrix between the camera and the new model
 */
/*void
 apMbTracker::reInitModel(const vpImage<unsigned char>& _I, const char* _cad_name, const vpHomogeneousMatrix& _cMo)
 {
 resetTracker();
 loadModel(_cad_name);
 init(_I, _cMo);
 }

 void
 apMbTracker::reInitConfigModel(const vpImage<unsigned char>& I, const char* cad_name,const char* config_name, const vpHomogeneousMatrix& _cMo)
 {
 resetTracker();
 loadConfigFile(cad_name);
 loadModel(config_name);
 init(I, _cMo);*/
//}

/*!
 Return the number of good points (vpPointSite) tracked. A good point is a
 vpPointSite with its flag "suppress" equal to 0. Only these points are used
 during the virtual visual servoing stage.

 \exception vpException::dimensionError if _level does not represent a used
 level.

 \return the number of good points.
 */
/*unsigned int apMbTracker::getNbPoints(const unsigned int _level) {
	if ((_level > scales.size()) || !scales[_level]) {
		throw vpException(vpException::dimensionError, "Level is not used");
	}

	unsigned int nbGoodPoints = 0;
	apControlPoint *p;
	for (int k; k < points[_level].size(); k++) {
		p = (points[_level])[k];
		if (p != NULL) {
				if ((p->s).suppress == 0)
					nbGoodPoints++;
		}
	}
	return nbGoodPoints;
}*/

/*!
 Set the scales to use to realise the tracking. The vector of boolean activates
 or not the scales to se for the object tracking. The first element of the list
 correspond to the tracking on the full image, the second element corresponds
 to the tracking on an image sbsampled by two.

 Using multi scale tracking allows to track the object with greater moves. It
 requires the computation of a pyramid of images, but the total tracking can be
 faster than a tracking based only on the full scale. The pose is computed from
 the smallest image to the biggest. This may be dangerous if the object to
 track is small in the image, because the subsampled scale(s) will have only
 few points to compute the pose (it could result in a loss of precision).

 \warning This method must be used before the tracker has been initialised (
 before the call of the init() or the initClick() method).

 \warning At least one level must be activated.

 \param _scales : The vector describing the levels to use.
 */
void apMbTracker::setScales(const std::vector<bool>& _scales) {
	unsigned int nbActivatedLevels = 0;
	for (unsigned int i = 0; i < _scales.size(); i += 1) {
		if (_scales[i]) {
			nbActivatedLevels++;
		}
	}
	if ((_scales.size() < 1) || (nbActivatedLevels == 0)) {
		vpERROR_TRACE(
				" !! WARNING : must use at least one level for the tracking. Use the global one");
		scales.resize(0);
		scales.push_back(true);
		points.resize(1);
		points[0].resize(0);
	} else {
		scales = _scales;
		points.resize(_scales.size());
		for (unsigned int i = 0; i < points.size(); i += 1) {
			points[i].resize(0);
		}
	}
}

/*!
 Compute the pyramid of image associated to the image in parameter. The scales
 computed are the ones corresponding to the scales  attribte of the class. If
 OpenCV is detected, the functions used to computed a smoothed pyramid come
 from OpenCV, otherwise a simple subsampling (no smoothing, no interpolation)
 is realised.

 \warning The pyramid contains pointers to vpImage. To properly deallocate the
 pyramid. All the element but the first (which is a pointer to the input image)
 must be freed. A proper cleaning is implemented in the cleanPyramid() method.

 \param _I : The input image.
 \param _pyramid : The pyramid of image to build from the input image.
 */
void apMbTracker::initPyramid(const vpImage<unsigned char>& _I,
		std::vector<const vpImage<unsigned char>*>& _pyramid) {
	_pyramid.resize(scales.size());

	if (scales[0]) {
		_pyramid[0] = &_I;
	} else {
		_pyramid[0] = NULL;
	}

	for (unsigned int i = 1; i < _pyramid.size(); i += 1) {
		if (scales[i]) {
			unsigned int cScale = static_cast<unsigned int> (pow(2., (int) i));
			vpImage<unsigned char>* I = new vpImage<unsigned char> (
					_I.getHeight() / cScale, _I.getWidth() / cScale);
#ifdef VISP_HAVE_OPENCV
                        cv::Mat vpI0,vpI;
                        vpI0.create(_I.getWidth(),_I.getHeight(),CV_8U);
                        vpI0.data = (uchar*) (_I.bitmap);
                        vpI.create(_I.getWidth()/cScale,_I.getHeight()/cScale,CV_8U);
                        cv::resize(vpI0, vpI, vpI.size(), 0, 0, INTER_NEAREST);
			vpImageConvert::convert(vpI, *I);
                        vpI.release();
                        vpI0.release();
#else
			for (unsigned int k = 0, ii = 0; k < I->getHeight(); k += 1, ii += cScale) {
				for (unsigned int l = 0, jj = 0; l < I->getWidth(); l += 1, jj += cScale) {
					(*I)[k][l] = _I[ii][jj];
				}
			}
#endif
			_pyramid[i] = I;
		} else {
			_pyramid[i] = NULL;
		}
	}
}

void apMbTracker::initPyramid(const vpImage<vpRGBa>& _I, std::vector<
		const vpImage<vpRGBa>*>& _pyramid) {
	_pyramid.resize(scales.size());

	if (scales[0]) {
		_pyramid[0] = &_I;
	} else {
		_pyramid[0] = NULL;
	}

	for (unsigned int i = 1; i < _pyramid.size(); i += 1) {
		if (scales[i]) {
			unsigned int cScale = static_cast<unsigned int> (pow(2., (int) i));
			vpImage<vpRGBa>* I = new vpImage<vpRGBa> (_I.getHeight() / cScale,
					_I.getWidth() / cScale);
#ifdef VISP_HAVE_OPENCV

                        cv::Mat vpI0,vpI;
                        vpI0.create(_I.getWidth(),_I.getHeight(),CV_8U);
                        vpI0.data = (uchar*) (_I.bitmap);
                        vpI.create(_I.getWidth()/cScale,_I.getHeight()/cScale,CV_8U);
                        cv::resize(vpI0, vpI, vpI.size(), 0, 0, INTER_NEAREST);
                        vpImageConvert::convert(vpI, *I);
                        vpI.release();
                        vpI0.release();
#else
			for (unsigned int k = 0, ii = 0; k < I->getHeight(); k += 1, ii += cScale) {
				for (unsigned int l = 0, jj = 0; l < I->getWidth(); l += 1, jj += cScale) {
					(*I)[k][l].R = _I[ii][jj].R;
					(*I)[k][l].G = _I[ii][jj].G;
					(*I)[k][l].B = _I[ii][jj].B;
				}
			}
#endif
			_pyramid[i] = I;
		} else {
			_pyramid[i] = NULL;
		}
	}
}

/*!
 Clean the pyramid of image allocated with the initPyramid() method. The vector
 has a size equal to zero at the end of the method.

 \param _pyramid : The pyramid of image to clean.
 */
void apMbTracker::cleanPyramid(
		std::vector<const vpImage<unsigned char>*>& _pyramid) {
	if (_pyramid.size() > 0) {
		_pyramid[0] = NULL;
		for (unsigned int i = 1; i < _pyramid.size(); i += 1) {
			if (_pyramid[i] != NULL) {
				delete _pyramid[i];
				_pyramid[i] = NULL;
			}
		}
		_pyramid.resize(0);
	}
}

void apMbTracker::cleanPyramid(
		std::vector<const vpImage<vpRGBa>*>& _pyramid) {
	if (_pyramid.size() > 0) {
		_pyramid[0] = NULL;
		for (unsigned int i = 1; i < _pyramid.size(); i += 1) {
			if (_pyramid[i] != NULL) {
				delete _pyramid[i];
				_pyramid[i] = NULL;
			}
		}
		_pyramid.resize(0);
	}
}

/*!
 Get the list of the control points tracked for the specified level.

 \throw vpException::dimensionError if the parameter does not correspond to an
 used level.

 \param _level : Level corresponding to the list to return.
 \return Pointer to the list of the lines tracked.
 */
std::vector<apControlPoint *>*
apMbTracker::getPpoint(const unsigned int _level) {
	if (_level > scales.size() || !scales[_level]) {
		std::ostringstream oss;
		oss << _level;
		std::string errorMsg = "level " + oss.str()
				+ " is not used, cannot get its distance lines.";
		throw vpException(vpException::dimensionError, errorMsg);
	}

	return &points[_level];
}

/*!
 Modify the camera parameters to have them corresponding to the current scale.
 The new parameters are divided by \f$ 2^{\_scale} \f$.

 \param _scale : Scale to use.
 */
void apMbTracker::downScale(const unsigned int _scale) {
	const double ratio = pow(2., (int) _scale);
	scaleLevel = _scale;

	vpMatrix K = cam.get_K();

	K[0][0] /= ratio;
	K[1][1] /= ratio;
	K[0][2] /= ratio;
	K[1][2] /= ratio;

	cam.initFromCalibrationMatrix(K);
}

/*!
 Modify the camera parameters to have them corresponding to the current scale.
 The new parameters are multiplied by \f$ 2^{\_scale} \f$.

 \param _scale : Scale to use.
 */
void apMbTracker::upScale(const unsigned int _scale) {
	const double ratio = pow(2., (int) _scale);
	scaleLevel = 0;

	vpMatrix K = cam.get_K();

	K[0][0] *= ratio;
	K[1][1] *= ratio;
	K[0][2] *= ratio;
	K[1][2] *= ratio;

	cam.initFromCalibrationMatrix(K);
}

/*!
 Re initialise the moving edges associated to a given level. This method is
 used to re-initialise the level if the tracking failed on this level but
 succedded on the other one.

 \param _lvl : The level to re-initialise.
 */
void apMbTracker::reInitLevel(const unsigned int _lvl) {
	unsigned int scaleLevel_1 = scaleLevel;
	scaleLevel = _lvl;

	apControlPoint *p;
	for (int k = 0; k < points[scaleLevel].size(); k++) {
		p = (points[scaleLevel])[k];
		p->initControlPoint(*Ipyramid[_lvl], 0);

	}

	trackControlPoints(*Ipyramid[_lvl]);
	//updateMovingEdge(*Ipyramid[_lvl]);
	scaleLevel = scaleLevel_1;
}

void apMbTracker::detectLandmarks(vpImage<unsigned char> &I, vpImage<vpRGBa> &Inormd)
{

	/*vpImage<unsigned char> Iedge = I;
	apViews view;
	view.edgeOrientMap(Iedge);
	IplImage* Ip = NULL;
	IplImage* Ip1;
	vpImageConvert::convert(Iedge, Ip);
	Mat color_dst;
	vector<Vec4i> linesV;
	Mat dst(Ip);
	cvtColor(dst, color_dst, CV_GRAY2BGR);
	HoughLinesP(dst, linesV, 1, CV_PI / 180, 40, 40, 2);
	for (size_t i = 0; i < linesV.size(); i++) {
		line(color_dst, Point(linesV[i][0], linesV[i][1]), Point(
				linesV[i][2], linesV[i][3]), Scalar(255, 0, 255), 2, 8);
	}
	loadLinesLm(I, linesV, Inormd, cMo);*/
}

void apMbTracker::initLandmarks(vpImage<unsigned char> &I, vpImage<vpRGBa> &Inormd)
{

	/*vpImage<unsigned char> Iedge = I;
	apViews view;
	view.edgeOrientMap(Iedge);
	IplImage* Ip = NULL;
	IplImage* Ip1;
	vpImageConvert::convert(Iedge, Ip);
	Mat color_dst;
	vector<Vec4i> linesV;
	Mat dst(Ip);
	cvtColor(dst, color_dst, CV_GRAY2BGR);
	HoughLinesP(dst, linesV, 1, CV_PI / 180, 40, 40, 2);
	for (size_t i = 0; i < linesV.size(); i++) {
		line(color_dst, Point(linesV[i][0], linesV[i][1]), Point(
				linesV[i][2], linesV[i][3]), Scalar(255, 0, 255), 2, 8);
	}

	std::map<int, apKltControlPoint>::const_iterator iter = newLinesLm[scaleLevel].begin();
	//for( ; iter != kltPoints[scaleLevel].end(); iter++){
	   //  int id(iter->first);
	   //  kltTracker.getFeature((int)i, id, x, y);
	  //         kltPoints[scaleLevel][id].icpoint_curr = vpImagePoint(static_cast<double>(y),static_cast<double>(x));
	//}
	int id = 0;
	for( ; iter != newLinesLm[scaleLevel].end(); iter++)
	{
	 int id0 = iter->first;
	 apMbtDistanceLineMH l = iter->second;
     l.filter(linesV);
     if (l.confidence<th)
     {

    	 linesLm[scalelevel][id] = l;
    	 id++;
     }
	}*/

}

vpMatrix apMbTracker::computeCovarianceMatrix(const vpMatrix &A,
		const vpColVector &x, const vpColVector &b) {
	double sigma2 = (((b.t()) * b) - ((b.t()) * A * x));
	return (A.t() * A).pseudoInverse(b.getRows() * DBL_EPSILON) * sigma2;
}

vpMatrix apMbTracker::computeCovarianceMatrix(const vpMatrix &A,
		const vpColVector &x, const vpColVector &b, const vpMatrix &W) {
	//double sigma2 = (((W * b).t()) * W * b - (((W * b).t()) * W * A * x));
	//double sigma2 = (((b).t()) * b)/((double)b.getRows());
	double sigma2 = (((W * b).t()) * W * b)/((double)b.getRows());
	//double sigma2 = (0.01*0.01);
	//std::cout << " error mean Edge" << sqrt((((b).t()) * b)/((double)b.getRows())) << std::endl;
	return (A.t() * W * A).pseudoInverse(b.getRows() * DBL_EPSILON) * sigma2;
	//return (A.t() * A).pseudoInverse() * sigma2;

}

vpMatrix apMbTracker::computeCovarianceMatrix2(const vpMatrix &A,
		const vpColVector &x, const vpColVector &b, const vpMatrix &W) {
	//double sigma2 = (((W * b).t()) * W * b - (((W * b).t()) * W * A * x));
	//double sigma2 = (((b).t()) * b)/((double)b.getRows());
	double sigma2 = (((W * b).t()) * W * b)/((double)b.getRows());
	vpMatrix A2 = (A.t() * (W.t() * W) * A).pseudoInverse(b.getRows() * DBL_EPSILON);
	//double sigma2 = (0.01*0.01);
	//std::cout << " error mean Edge" << sqrt((((b).t()) * b)/((double)b.getRows())) << std::endl;
	return A2 * sigma2 * A.t()* W * W * W * W * A * ((A2).t());
	//return (A.t() * A).pseudoInverse() * sigma2;

}


void apMbTracker::setGroundTruth(std::string input, std::string output,
		const int firstframe) {
	fstream file;
	file.open(input, std::fstream::in);
	firstFrame = firstframe;
	int time = 0;

	vpColVector quatVraie(4);
	vpTranslationVector trVraie, tT0, t0morb;
	vpRotationMatrix RVraie, R0, Rq, RqT;
	vpHomogeneousMatrix orbMcV, cVMorb, TMT0, T0Morb, TMorb, TMo, cMoTrue,
			orbMo;
	double value;
	vpPoseVector truepose;
	double qa, qb, qc, qd;
	double stheta;

	vpTranslationVector tr, trV;
	vpRotationMatrix R, RV;
	vpRxyzVector Rxyz, RxyzV;

	tT0[0] = 0;
	tT0[1] = 0;
	tT0[2] = 0;
	int timek = 0;

	while (file.good()) {
		time++;
		truePose.resize(timek+1, 6, false);
		for (int samp = 0; samp < 4; samp++) {
			file >> value;
			for (int i = 0; i < 3; i++) {
				file >> value;
				trVraie[i] = value;
				//trVraie[i]=0;
			}
			file >> value;
			quatVraie[0] = value;
			//quatVraie[0] = 1;
			for (int i = 1; i < 4; i++) {
				file >> value;
				quatVraie[i] = -value;
				//quatVraie[i]=0;
			}
		}

		RVraie[0][0] = vpMath::sqr(quatVraie[0]) + vpMath::sqr(quatVraie[1])
				- vpMath::sqr(quatVraie[2]) - vpMath::sqr(quatVraie[3]);
		RVraie[0][1] = 2 * quatVraie[1] * quatVraie[2] - 2 * quatVraie[0]
				* quatVraie[3];
		RVraie[0][2] = 2 * quatVraie[0] * quatVraie[2] + 2 * quatVraie[1]
				* quatVraie[3];
		RVraie[1][0] = 2 * quatVraie[0] * quatVraie[3] + 2 * quatVraie[1]
				* quatVraie[2];
		RVraie[1][1] = vpMath::sqr(quatVraie[0]) - vpMath::sqr(quatVraie[1])
				+ vpMath::sqr(quatVraie[2]) - vpMath::sqr(quatVraie[3]);
		RVraie[1][2] = 2 * quatVraie[2] * quatVraie[3] - 2 * quatVraie[0]
				* quatVraie[1];
		RVraie[2][0] = 2 * quatVraie[1] * quatVraie[3] - 2 * quatVraie[0]
				* quatVraie[2];
		RVraie[2][1] = 2 * quatVraie[0] * quatVraie[1] + 2 * quatVraie[2]
				* quatVraie[3];
		RVraie[2][2] = vpMath::sqr(quatVraie[0]) - vpMath::sqr(quatVraie[1])
				- vpMath::sqr(quatVraie[2]) + vpMath::sqr(quatVraie[3]);
		orbMcV.buildFrom(trVraie, RVraie);
		//orbMcV.buildFrom(trVraie,R0);
		cVMorb = orbMcV.inverse();
		//cMo0=cMo;
		//stheta = 0.10759*(4*time/54000.0) * 500.0 + 2.0  * M_PI / 4.0;
		stheta = 8 * time * 0.10759 * 500 / 54000 + 2.0 * M_PI / 4 - 2 * M_PI
				/ 3;
		//stheta = M_PI/3;
		qa = cos(stheta / 2);
		qb = sin(stheta / 2) / sqrt(3);
		qc = -sin(stheta / 2) / sqrt(3);
		qd = sin(stheta / 2) / sqrt(3);
		/*cout << " quat " << qa << " " << qb << " " << qc << " " << qd << endl;
		 cout << " stheta " << stheta<< endl;*/
		Rq[0][0] = vpMath::sqr(qa) + vpMath::sqr(qb) - vpMath::sqr(qc)
				- vpMath::sqr(qd);
		Rq[0][1] = 2 * qb * qc - 2 * qa * qd;
		Rq[0][2] = 2 * qa * qc + 2 * qb * qd;
		Rq[1][0] = 2 * qa * qd + 2 * qb * qc;
		Rq[1][1] = vpMath::sqr(qa) - vpMath::sqr(qb) + vpMath::sqr(qc)
				- vpMath::sqr(qd);
		Rq[1][2] = 2 * qc * qd - 2 * qa * qb;
		Rq[2][0] = 2 * qb * qd - 2 * qa * qc;
		Rq[2][1] = 2 * qa * qb + 2 * qc * qd;
		Rq[2][2] = vpMath::sqr(qa) - vpMath::sqr(qb) - vpMath::sqr(qc)
				+ vpMath::sqr(qd);
		double phi = 0.10759 * (4 * time / 54000.0) * 500.0 + (3.0 / 8.0)
				* M_PI;
		//phi =0;
		//cout << " phi " << Rq << endl;
		RqT = vpRotationMatrix(vpRxyzVector(0, -phi - M_PI / 2, 0));
		//RqT = vpRotationMatrix(vpRxyzVector(0,-phi,0));
		//RqT = vpRotationMatrix(vpRxyzVector(0,0,0));

		double theta0 = -8 * M_PI / 180;
		/*trorb[0]=(6478137+1000000)*cos(phi);
		 trorb[1]=-(6478137+1000000)*sin(phi)*sin(theta0);
		 trorb[2]=(6478137+1000000)*sin(phi)*cos(theta0);

		 vpTranslationVector trT;
		 trT[0]=trorb[0]-(-trVraie[0]*sin(phi)-trVraie[2]*cos(phi));
		 trT[1]=trorb[1]-(trVraie[1]*cos(theta0) - sin(theta0)*(cos(phi)*trVraie[0]-sin(phi)*trVraie[2]));
		 trT[2]=trorb[2]-(trVraie[1]*sin(theta0) + cos(theta0)*(cos(phi)*trVraie[0]-sin(phi)*trVraie[2]));
		 cout << "trT " << trorb << endl;*/

		vpRotationMatrix RqT0 = vpRotationMatrix(vpRxyzVector(theta0, 0, 0));
		TMT0.buildFrom(tT0, RqT0);
		t0morb[0] = (6478137 + 1000000) * cos(phi);
		t0morb[1] = 0;
		t0morb[2] = (6478137 + 1000000) * sin(phi);
		T0Morb.buildFrom(tT0, RqT);
		TMorb = TMT0 * T0Morb;
		TMo.buildFrom(tT0, Rq * vpRotationMatrix(vpRxyzVector(-M_PI / 2, 0, 0)));
		//TMo.buildFrom(tT0,Rq*vpRotationMatrix(vpRxyzVector(0,0,0)));
		vpHomogeneousMatrix oMob;
		vpHomogeneousMatrix oMob0;
		oMob.buildFrom(tT0, vpRotationMatrix(vpRxyzVector(M_PI / 2, 0, 0)));
		oMob0.buildFrom(tT0, vpRotationMatrix(vpRxyzVector(M_PI, 0, 0)));
		//oMob0.buildFrom(tT0,vpRotationMatrix(vpRxyzVector(0,0,0)));
		orbMo = (TMorb.inverse()) * (TMo);
		vpThetaUVector tut;
		cMoTrue = oMob0 * cVMorb * orbMo;
		//cMoTrue.extract(tut);
		truepose.buildFrom(cMoTrue);
		cMoTrue.extract(RV);
		RxyzV.buildFrom(RV);

		if (time==1 || (time+6)%7 == 0)
		{
		truePose[timek][0] = truepose[0];
		truePose[timek][1] = -truepose[1];
		truePose[timek][2] = -truepose[2];
		/*truePose[time - 1][3] = truepose[3];
		truePose[time - 1][4] = truepose[4];
		truePose[time - 1][5] = truepose[5];*/
		truePose[timek][3] = RxyzV[0];
                truePose[timek][4] = RxyzV[1];
		truePose[timek][5] = RxyzV[2];
		timek++;
		}
	}
	truePose.saveMatrix(output, truePose, false, "");
}

void apMbTracker::computeError(vpColVector &error) {
	vpTranslationVector tr, trV;
	vpRotationMatrix R, RV;
	vpRxyzVector Rxyz, RxyzV;
	vpPoseVector truepose;
        truepose[0] = truePose[/*frame +*/ firstFrame - 1][0];
        truepose[1] = truePose[/*frame +*/ firstFrame - 1][1];
        truepose[2] = truePose[/*frame +*/ firstFrame - 1][2];
        truepose[3] = truePose[/*frame +*/ firstFrame - 1][3];
        truepose[4] = truePose[/*frame +*/ firstFrame - 1][4];
        truepose[5] = truePose[/*frame +*/ firstFrame - 1][5];
	cMoV.buildFrom(truepose);
	cMo.extract(tr);
	cMoV.extract(trV);
	cMo.extract(R);
	cMoV.extract(RV);
	Rxyz.buildFrom(R);
        RxyzV.buildFrom(RV);

        std::cout << " cMoVraie " << cMoV << std::endl;

	/*error[0] = tr[0] - trV[0];
	error[1] = tr[1] - trV[1];
	error[2] = tr[2] - trV[2];
	error[3] = Rxyz[0] - RxyzV[0];
	error[4] = Rxyz[1] - RxyzV[1];
	error[5] = Rxyz[2] - RxyzV[2];*/
	error[0] = tr[0] - truepose[0];
	error[1] = tr[1] - truepose[1];
	error[2] = tr[2] - truepose[2];
	error[3] = Rxyz[0] - truepose[3];
	error[4] = Rxyz[1] - truepose[4];
	error[5] = Rxyz[2] - truepose[5];
}

void apMbTracker::computeError(vpColVector &error, vpHomogeneousMatrix &_cMo) {
        vpTranslationVector tr, trV;
        vpRotationMatrix R, RV;
        vpRxyzVector Rxyz, RxyzV;
        vpPoseVector truepose;
        truepose[0] = truePose[/*frame +*/ firstFrame - 1][0];
        truepose[1] = truePose[/*frame +*/ firstFrame - 1][1];
        truepose[2] = truePose[/*frame +*/ firstFrame - 1][2];
        truepose[3] = truePose[/*frame +*/ firstFrame - 1][3];
        truepose[4] = truePose[/*frame +*/ firstFrame - 1][4];
        truepose[5] = truePose[/*frame +*/ firstFrame - 1][5];
        cMoV.buildFrom(truepose);
        _cMo.extract(tr);
        cMoV.extract(trV);
        _cMo.extract(R);
        cMoV.extract(RV);
        Rxyz.buildFrom(R);
        RxyzV.buildFrom(RV);

        std::cout << " cMoVraie " << cMoV << " cMo " << _cMo << std::endl;

        error[0] = tr[0] - trV[0];
        error[1] = tr[1] - trV[1];
        error[2] = tr[2] - trV[2];
        error[3] = Rxyz[0] - RxyzV[0];
        error[4] = Rxyz[1] - RxyzV[1];
        error[5] = Rxyz[2] - RxyzV[2];

}


#ifdef ENABLE_ZMQ

void apMbTracker::initComm()
{
    m_socketPub =new zmq::socket_t(m_context, ZMQ_PUB);
    m_socketPub->bind("tcp://127.0.0.1:6666");

    m_socketSub =new zmq::socket_t(m_context, ZMQ_SUB);
    m_socketSub->setsockopt(ZMQ_SUBSCRIBE, "", 0);
    m_socketSub->connect("tcp://127.0.0.1:6667");
    if(m_socketSub->connected())
    {
        std::cout << "Connected to tcp://127.0.0.1:6667" << std::endl;
    }
    else
    {
        std::cout << "Could not connect to tcp://127.0.0.1:6667" << std::endl;
    }
}

void apMbTracker::sendPose()
{
    string messageStr;
    messageStr = std::to_string(cMo[0][3]) + " " + std::to_string(cMo[1][3]) + " " + std::to_string(cMo[2][3]) + " "
            + std::to_string(cMo[0][0]) + " " + std::to_string(cMo[0][1]) + " " + std::to_string(cMo[0][2]) + " "
            + std::to_string(cMo[1][0]) + " " + std::to_string(cMo[1][1]) + " " + std::to_string(cMo[1][2]) + " "
            + std::to_string(cMo[2][0]) + " " + std::to_string(cMo[2][1]) + " " + std::to_string(cMo[2][2]) + " ";

    zmq::message_t message(messageStr.length());
    memcpy(message.data(), messageStr.c_str(), messageStr.length());

    bool status = m_socketPub->send(message);
}

void apMbTracker::receiveImage(vpImage<vpRGBa> &Icol)
{
    cv::Mat img;
    cv::Mat img1 = Mat::zeros( Icol.getHeight(),Icol.getWidth(), CV_8UC3);

    zmq::message_t message1;

    bool status1 = m_socketSub->recv(&message1);
    if(status1){
    std::string rpl = std::string(static_cast<char*>(message1.data()), message1.size());

    const char *cstr = rpl.c_str();
    loadImage(img,cstr);

   // memcpy(img.data, message1.data(), imgSize);
    std::cout << " ok receive " << std::endl;
//    cv::imwrite("/Users/froy/test_apbm.png", img);

//  // Assign pixel value to img
//  for (int i = 0;  i < img1.rows; i++) {
//   for (int j = 0; j < img1.cols; j++) {
//    img1.at<Vec4b>(i,j)[0] = img.at<uchar>(0,i*img1.cols+j);
//    img1.at<Vec4b>(i,j)[1] = img.at<uchar>(0,i*img1.cols+j + 1);
//    img1.at<Vec4b>(i,j)[2] = img.at<uchar>(0,i*img1.cols+j + 2);
//    img1.at<Vec4b>(i,j)[3] = img.at<uchar>(0,i*img1.cols+j + 3);
//    }
//   }
//  cv::imwrite("socketimage100.png", img1);
    }
vpImageConvert::convert(img,Icol);
}


void apMbTracker::loadImagePoseMesh( cv::Mat &mat, vpHomogeneousMatrix &cMo, std::vector<point3d> &vertices, std::vector<point3d> &normals, std::vector<triangle> &triangles)
{

    zmq::message_t message1;

    std::cout << " status 0 " << std::endl;

    bool status1 = m_socketSub->recv(&message1);

    std::cout << " status 1 " << status1 << std::endl;
    if(status1){
    std::string rpl = std::string(static_cast<char*>(message1.data()), message1.size());
    const char *cstr = rpl.c_str();

    ifstream file;//(filename.c_str(), ios_base::in);
    float x, y, z, w;
    string line;
    int n;

    //if ((n = parseFormat(line.c_str(), "v %f %f %f %f", x, y, z, w)) > 0)
    std::stringstream stream;
    stream << cstr;

    vpCameraParameters camparam;
    vpRotationMatrix R0;
    vpTranslationVector t0,t;
    vpMatrix intrinsic;
    intrinsic.resize(3,3);

    vertices.resize(0);
    normals.resize(0);
    triangles.resize(0);

   // for( size_t i=0; i<stream.length(); i++)
        //char c = stream[i];
        //if( c == '[' ) i++;

    //std::cout << " str " << stream.str() << std::endl;

    //getchar();

        if (stream.peek() == '[')
            stream.ignore();

            //while (stream[i] != ']')
             for (int j = 0; j < 3; j++)
                 for (int k = 0; k < 3; k++){
             stream >> intrinsic[j][k];
             if (stream.peek() == ',')
                 stream.ignore();
             std::cout << " camparam " << intrinsic[j][k] << std::endl;
                 }
                 stream.ignore();
                 stream.ignore();

        if (stream.peek() == '[')
            stream.ignore();

            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++){
                 stream >> R0[j][k];
                 if (stream.peek() == ',')
                     stream.ignore();
                 std::cout << " rotation " << R0[j][k] << std::endl;
                }
                stream.ignore();
                stream.ignore();

        if (stream.peek() == '[')
            stream.ignore();

            for (int k = 0; k < 3; k++){
                stream >> t0[k];
                if (stream.peek() == ',')
                    stream.ignore();
                std::cout << " translation " << t0[k] << std::endl;
                }
                stream.ignore();
                stream.ignore();

                t[0]=-t0[0];
                t[1]=-t0[1];
                t[2]=-t0[2];

                t0 = R0*t;

                cMo.buildFrom(t0,R0);

             int npoints = 0;

             desserialize(stream,mat);

             stream.ignore();
             stream.ignore();

             while (stream.peek()!=';')
             {
                 point3d vertex;

                 stream >> vertex.x;
                 stream >> vertex.y;
                 stream >> vertex.z;


                 //std::cout << " vertex " <<  vertex.x << " " <<  vertex.y << " " <<  vertex.z << std::endl;
                 npoints ++;


                 vertices.push_back(vertex);

             }
             stream.ignore();

             while (stream.peek()!=';')
             {
                 point3d normal;

                 stream >> normal.x;
                 stream >> normal.y;
                 stream >> normal.z;

                 //std::cout << " normals " <<  normal.x << " " <<  normal.y << " " <<  normal.z << std::endl;

                 normals.push_back(normal);

             }
             stream.ignore();

             while (stream.peek()!=';')
             {
                 triangle tri;

                 stream >> tri.v1;
                 tri.n1 = tri.v1;
                 stream >> tri.v2;
                 tri.n2 = tri.v2;
                 stream >> tri.v3;
                 tri.n3 = tri.v3;

                 //std::cout << " triangle " <<  tri.v1 << " " <<  tri.v2  << " " <<  tri.v3 << std::endl;
                 triangles.push_back(tri);

             }
         }


}

void apMbTracker::loadImagePoseMeshControlPoints( cv::Mat &mat, vpHomogeneousMatrix &cMo, std::vector<point3d> &vertices, std::vector<point3d> &normals, std::vector<triangle> &triangles)
{

    zmq::message_t message1;

    std::cout << " status 0 " << std::endl;

    bool status1 = m_socketSub->recv(&message1);

    std::cout << " status 1 " << status1 << std::endl;
    if(status1){
    std::string rpl = std::string(static_cast<char*>(message1.data()), message1.size());
    const char *cstr = rpl.c_str();

    ifstream file;//(filename.c_str(), ios_base::in);
    float x, y, z, w;
    string line;
    int n;

    //if ((n = parseFormat(line.c_str(), "v %f %f %f %f", x, y, z, w)) > 0)
    std::stringstream stream;
    stream << cstr;

    vpCameraParameters camparam;
    vpRotationMatrix R0;
    vpTranslationVector t0,t;
    vpMatrix intrinsic;
    intrinsic.resize(3,3);

    vertices.resize(0);
    normals.resize(0);
    triangles.resize(0);
    controlpoints.resize(0);

   // for( size_t i=0; i<stream.length(); i++)
        //char c = stream[i];
        //if( c == '[' ) i++;

    //std::cout << " str " << stream.str() << std::endl;

    //getchar();

        if (stream.peek() == '[')
            stream.ignore();

            //while (stream[i] != ']')
             for (int j = 0; j < 3; j++)
                 for (int k = 0; k < 3; k++){
             stream >> intrinsic[j][k];
             if (stream.peek() == ',')
                 stream.ignore();
             std::cout << " camparam " << intrinsic[j][k] << std::endl;
                 }
                 stream.ignore();
                 stream.ignore();

        if (stream.peek() == '[')
            stream.ignore();

            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++){
                 stream >> R0[j][k];
                 if (stream.peek() == ',')
                     stream.ignore();
                 std::cout << " rotation " << R0[j][k] << std::endl;
                }
                stream.ignore();
                stream.ignore();

        if (stream.peek() == '[')
            stream.ignore();

            for (int k = 0; k < 3; k++){
                stream >> t0[k];
                if (stream.peek() == ',')
                    stream.ignore();
                std::cout << " translation " << t0[k] << std::endl;
                }
                stream.ignore();
                stream.ignore();

                t[0]=-t0[0];
                t[1]=-t0[1];
                t[2]=-t0[2];

                t0 = R0*t;

                cMo.buildFrom(t0,R0);

             int npoints = 0;

             desserialize(stream,mat);

             stream.ignore();
             stream.ignore();

             while (stream.peek()!=';')
             {
                 point3d vertex;

                 stream >> vertex.x;
                 stream >> vertex.y;
                 stream >> vertex.z;


                 //std::cout << " vertex " <<  vertex.x << " " <<  vertex.y << " " <<  vertex.z << std::endl;
                 npoints ++;


                 vertices.push_back(vertex);

             }
             stream.ignore();

             while (stream.peek()!=';')
             {
                 point3d normal;

                 stream >> normal.x;
                 stream >> normal.y;
                 stream >> normal.z;

                 //std::cout << " normals " <<  normal.x << " " <<  normal.y << " " <<  normal.z << std::endl;

                 normals.push_back(normal);

             }
             stream.ignore();

             while (stream.peek()!=';')
             {
                 triangle tri;

                 stream >> tri.v1;
                 tri.n1 = tri.v1;
                 stream >> tri.v2;
                 tri.n2 = tri.v2;
                 stream >> tri.v3;
                 tri.n3 = tri.v3;

                 //std::cout << " triangle " <<  tri.v1 << " " <<  tri.v2  << " " <<  tri.v3 << std::endl;
                 triangles.push_back(tri);

             }

             stream.ignore();

             while (stream.peek()!=';')
             {

                 vpColVector vertex(4);
                 vpColVector vertexcam(4);
                 vertex[3] = 1;

                 stream >> vertex[0];
                 stream >> vertex[1];
                 stream >> vertex[2];

                 vertexcam = cMo * vertex;


                 point3d controlpoint;

                 controlpoint.x = vertexcam[0];
                 controlpoint.y = vertexcam[1];
                 controlpoint.z = vertexcam[2];

                 std::cout << " CP " <<  controlpoint.x << " " <<  controlpoint.y << " " <<  controlpoint.z << std::endl;

                 controlpoints.push_back(controlpoint);

             }
         std::cout << "NUMBER OF CONTROLPOINTS: " << controlpoints.size() << std::endl;
         }


}


void apMbTracker::loadPointsNormals2d(std::vector<std::vector<point2d>> points2d, std::vector<std::vector<point2dd>> normals2d)
{

    zmq::message_t message1;
    bool status1 = m_socketSub->recv(&message1);

    std::cout << " status 1 " << status1 << std::endl;
    if(status1){
    std::string rpl = std::string(static_cast<char*>(message1.data()), message1.size());
    const char *cstr = rpl.c_str();

    std::stringstream stream;
    stream << cstr;

    points2d.resize(0);
    normals2d.resize(0);

    int nimages = 0;

        while (!stream.eof())
        {
            std::vector<point2d> points2d0;
            std::vector<point2dd> normals2d0;

           points2d0.resize(0);
           normals2d0.resize(0);

           int npoints = 0;

             while (stream.peek()!=';')
             {
                 point2d point;
                 double x,y;
                 stream >> x;
                 stream >> y;
                 npoints ++;
                 point.i = (int) x;
                 point.j = (int) y;
                 std::cout << " pos " << x << " " << y << std::endl;

                 points2d0.push_back(point);
             }
             stream.ignore();
             points2d.push_back(points2d0);


             while (stream.peek()!=';')
             {
                 point2dd normal;
                 stream >> normal.x;
                 stream >> normal.y;
                 std::cout << " norm " << normal.x << " " << normal.y << std::endl;
                 normals2d0.push_back(normal);
             }
             stream.ignore();
             stream.ignore();
             normals2d.push_back(normals2d0);
             nimages++;
         }
        }

    std::cout << " size normals " << normals2d.size() << " " << points2d.size() << std::endl;


}

void
 apMbTracker::computeVVSPhotometric(const vpImage<unsigned char>& _I)
 {
 double residu_1 =1e3;
 double r =1e3-1;
 vpMatrix LTL;
 vpColVector LTR;

 // compute the interaction matrix and its pseudo inverse
 apControlPoint *p ;

 vpColVector w;
 vpColVector weighted_error;
 vpColVector factor;

 vpTranslationVector tr;
 cMo.extract(tr);

 unsigned int iter = 0;

 //Nombre de moving edges
 int nbrow  = 0;
 vpFeatureLine fli;

 vpMatrix L(nbrow,6), Lsd, LT;
 // matrice d'interaction a la position desiree
 vpMatrix Hsd;  // hessien a la position desiree
 vpMatrix H ; // Hessien utilise pour le levenberg-Marquartd

 vpColVector errorG ;

 // compute the error vector
 vpColVector error(nbrow);
 int nerror = error.getRows();
 int nerrorG;
 vpColVector v ;

 double limite = 3; //Une limite de 3 pixels
 limite = limite / cam.get_px(); //Transformation limite pixel en limite metre.

 //Parametre pour la premiere phase d'asservissement
 double e_prev = 0, e_cur, e_next;
 bool reloop = true;
 double count = 0;

 int hght = _I.getHeight();
 int wdth = _I.getWidth();

 int nbr =hght;
 int nbc = wdth;
 vpImage<unsigned char> imG(hght,wdth);
 vpImage<unsigned char> Igd(hght,wdth);
  vpImage<unsigned char> Igdgroundtruth(hght,wdth);
 vpImage<unsigned char> Ig(hght,wdth);
 vpImage<unsigned char> Idiff(hght,wdth);
 vpColVector e;

 //vpImageIo::read(Igdgroundtruth, "imagePig3.png");
 Igdgroundtruth = _I;


 /*for (int i=3; i < nbr-3 ; i++)
 {
 //   cout << i << endl ;
 for (int j = 3 ; j < nbc-3; j++)
 {
 // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
 double Igf =   vpImageFilter::gaussianFilter(_I,i,j) ;
 imG[i][j] = Igf ;

 }
 }*/

 /* for (int i=150; i < 350 ; i++)
 {
 for (int j = 150 ; j < 550; j++)
 {
 // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
 double Ix =   1 * vpImageFilter::sobelFilterX(imG,i,j);
 double Iy =   1 * vpImageFilter::sobelFilterY(imG,i,j);
 Igd[i][j]= (unsigned char)sqrt(vpMath::sqr(Ix)+vpMath::sqr(Iy));
 //I2[i][j] = (unsigned char)Igrad[i][j];
 }
 }*/

 Igd = _I;

 sId.init(imG.getHeight(), imG.getWidth(), tr[2]);
 sI.init(imG.getHeight(), imG.getWidth(), tr[2]);
 sId.buildFrom(Igd);
 sId.interaction(Lsd);
 //Lsd=2*Lsd;
 nerrorG = Lsd.getRows();
 vpColVector errorT(nerror+nerrorG);
 vpDisplayX displayo;
 //displayo.init(Idiff, 10, 10, "display");
 double mu = 0.000;
 vpMatrix diagHsd(6,6);
 vpMatrix diagLTL(6,6);
 std::cout << " cmo " << cMo << std::endl;


 Hsd = Lsd.AtA();
 diagHsd.eye(6);
 for(int i = 0 ; i < 6 ; i++) diagHsd[i][i] = Hsd[i][i];

 H = ((mu * diagHsd) + Hsd).pseudoInverse();

 /*** First phase ***/
  vpImageTools::imageDifference(Ig,Igdgroundtruth,Idiff);

  //vpDisplay::display(Idiff);
  //vpDisplay::flush(Idiff);

  vpVideoWriter writer1;
  writer1.setCodec( CV_FOURCC('P','I','M','1') );
  writer1.setFileName("I_d.mpg");

  vpVideoWriter writer2;
  writer2.setCodec( CV_FOURCC('P','I','M','1') );
  writer2.setFileName("I.mpg");

  vpVideoWriter writer3;
  writer3.setCodec( CV_FOURCC('P','I','M','1') );
  writer3.setFileName("I_diff.mpg");

  writer1.open(Igd);
  writer2.open(Ig);
  writer3.open(Idiff);

  vpHomogeneousMatrix cMo1 = cMo;

  vpHomogeneousMatrix cMo0;

  vpRotationMatrix R0;
  vpTranslationVector t0;

  while ( reloop == true && iter<3)
 {
      double t00= vpTime::measureTimeMs();
      R0[0][0] = cMo[0][0];
      R0[0][1] = cMo[0][1];
      R0[0][2] = cMo[0][2];
      R0[1][0] = cMo[1][0];
      R0[1][1] = cMo[1][1];
      R0[1][2] = cMo[1][2];
      R0[2][0] = cMo[2][0];
      R0[2][1] = cMo[2][1];
      R0[2][2] = cMo[2][2];

      t0[0]=-cMo[0][3];
      t0[1]=-cMo[1][3];
      t0[2]=-cMo[2][3];

      t0 = R0.inverse()*t0;

      cMo0.buildFrom(t0,R0);
      string messageStr;
      messageStr = std::to_string(cMo0[0][3]) + " " + std::to_string(cMo0[1][3]) + " " + std::to_string(cMo0[2][3]) + " "
              + std::to_string(cMo0[0][0]) + " " + std::to_string(cMo0[0][1]) + " " + std::to_string(cMo0[0][2]) + " "
              + std::to_string(cMo0[1][0]) + " " + std::to_string(cMo0[1][1]) + " " + std::to_string(cMo0[1][2]) + " "
              + std::to_string(cMo0[2][0]) + " " + std::to_string(cMo0[2][1]) + " " + std::to_string(cMo0[2][2]) + " ";

      zmq::message_t message(messageStr.length());
      std::cout << "cmo" <<  cMo0 << std::endl;
      memcpy(message.data(), messageStr.c_str(), messageStr.length());

      bool status = m_socketPub->send(message);

      if(!status)
         std::cout << "Problem with communication" << std::endl;

               std::cout << " ok send " << std::endl;

         cv::Mat  img;
         cv::Mat img1 = Mat::zeros( hght,wdth, CV_8UC1);
         /*int  imgSize = img.total()*img.elemSize();
         uchar sockData[imgSize];
         int bytes;*/

         zmq::message_t message1;

         bool status1 = m_socketSub->recv(&message1);
         if(status1){
         std::string rpl = std::string(static_cast<char*>(message1.data()), message1.size());
         const char *cstr = rpl.c_str();
        loadImage(img,cstr);

        // memcpy(img.data, message1.data(), imgSize);

         std::cout << " ok receive " << std::endl;

         /*for (int i = 0; i < imgSize; i += bytes) {
         bytes=m_socket1.recv(sockData +i) == 0;
         }*/

       // Assign pixel value to img

       int ptr=0;
       for (int i = 0;  i < img1.rows; i++) {
        for (int j = 0; j < img1.cols; j++) {
         img1.at<uchar>(i,j) = img.at<uchar>(0,i*img1.cols+j);
         ptr=ptr+3;
         }
        }

       cv::imwrite("socketimage100.png", img1);
         }

 cMo.extract(tr);

 sI.init(Ig.getHeight(), Ig.getWidth(),tr[2]);
 vpImageConvert::convert(img1,Ig);
 sI.buildFrom(Ig);
       sI.interaction(Lsd);
 sI.error(sId, errorG);

 vpImageTools::imageDifference(Ig,Igdgroundtruth,Idiff);
 //vpDisplay::display(Idiff);
 //vpDisplay::flush(Idiff);

 //vpImageIo::write(Idiff, "Idiff.png");


 if (iter >1){

     writer1.saveFrame(Igd);
     writer2.saveFrame(Ig);
     writer3.saveFrame(Idiff);
     Hsd = Lsd.AtA();
     diagHsd.eye(6);
     for(int i = 0 ; i < 6 ; i++) diagHsd[i][i] = Hsd[i][i];

     H = ((mu * diagHsd) + Hsd).pseudoInverse();
 //	compute the control law
 e = H * Lsd.t() *errorG;
v =  -1*e;
 cMo =  vpExponentialMap::direct(v).inverse() * cMo;

 std::cout << " v " << v << std::endl;
}

 iter++;
 }
  std::cout << " cmo diff0 " << cMo1.inverse()*cMo << std::endl;
 std::cout << "\t First minimization in " << iter << " iteration00 " << std::endl ;

 //   cout << "\t Robust minimization in " << iter << " iteration " << endl ;
 //    std::cout << "error: " << (residu_1 - r) << std::endl;
 }

void apMbTracker::computeVVSCCDMHPhotometric(const vpImage<unsigned char>& _I,
                const vpImage<vpRGBa>& _IRGB) {
        double residu_1 = 1e3;
        double r = 1e3 - 1;
        vpMatrix LTL;
        vpColVector LTR;

        // compute the interaction matrix and its pseudo inverse

        vpColVector w;
        vpColVector weighted_error;
        vpColVector factor;

        CCDTracker.init(CCDParameters, cam);
        CCDTracker.setImage(_IRGB);

        vpTranslationVector tr;
        cMo.extract(tr);

        unsigned int iter = 0;

        //Nombre de moving edges
        int nbrow  = 0;


        #pragma omp parallel for
        for (int k = 0; k < points[scaleLevel].size(); k++) {
                apControlPoint *p = (points[scaleLevel])[k];
                p->initInteractionMatrixError();
        }
        nbrow = points[scaleLevel].size();

        if (nbrow == 0) {
                vpERROR_TRACE(
                                "\n\t\t Error-> not enough data in the interaction matrix...");
                throw vpTrackingException(vpTrackingException::notEnoughPointError,
                                "\n\t\t Error-> not enough data in the interaction matrix...");
        }


        vpMatrix L(nbrow,6), Lsd;
        vpColVector LTG;
        // matrice d'interaction a la position desiree
        vpMatrix Hsd;  // hessien a la position desiree
        vpMatrix H ; // Hessien utilise pour le levenberg-Marquartd

        vpColVector errorG;

        vpMatrix LTCIL(6, 6);
        vpColVector LTCIR(6);

        // compute the error vector
        vpColVector error(nbrow);
        int nerror = error.getRows();
        vpColVector v;

        double limite = 3; //Une limite de 3 pixels
        limite = limite / cam.get_px(); //Transformation limite pixel en limite metre.

        //Parametre pour la premiere phase d'asservissement
        bool reloop = true;
        double count = 0;

        int nerrorG;

        int hght = _I.getHeight();
        int wdth = _I.getWidth();

        vpImage<unsigned char> Igd(hght,wdth);
        vpImage<unsigned char> Igdgroundtruth(hght,wdth);
        vpImage<unsigned char> Ig(hght,wdth);
        vpImage<unsigned char> Idiff(hght,wdth);
        vpColVector e;

        //vpImageIo::read(Igdgroundtruth, "imagePig3.png");
        //Igdgroundtruth = _I;
        Igdgroundtruth = IdN;

        //if (frame0<10)
        Igd = IdN;
        //else Igd = _I;

        vpHomogeneousMatrix cMo0, cMct;
        vpRotationMatrix R0;
        vpTranslationVector t0;

        cMct = cMo*oMct;

        /*for (int k = 0; k < Igd.getHeight(); k++)
                for (int l = 0; l < Igd.getWidth(); l++) {
                        if (Inormdprec[k][l].A != 0)
                        Igd[k][l] = 0;
        }*/

        sId.init(Igd.getHeight(), Igd.getWidth(), tr[2]);
        sI.init(Igd.getHeight(), Igd.getWidth(), tr[2]);

        sId.buildFrom(Igd);
        sId.interaction(Lsd);
        nerrorG = Lsd.getRows();
        //vpDisplayX displayo;
        //displayo.init(Ig, 10, 10, "display");
        double mu = 0.000;
        vpMatrix diagHsd(6,6);

        /*** First phase ***/
         vpImageTools::imageDifference(Ig,Igdgroundtruth,Idiff);

         //vpDisplay::display(Idiff);
         //vpDisplay::flush(Idiff);

        /*** First phase ***/

        while (reloop == true && iter < 3)
        {
                if (iter == 0)
                {
                        weighted_error.resize(nerror);
                        w.resize(nerror);
                        w = 1;
                        factor.resize(nerror);
                        factor = 1;
                }

                count = 0;
                reloop = true;
                /*
                #pragma omp parallel
                {
                 int local_count = 0;
                #pragma omp for nowait
                  for (int k = 0; k < points[scaleLevel].size(); ++k) {
                                apControlPoint *p = (points[scaleLevel])[k];
                                p->computeInteractionMatrixErrorMH(cMo, _I);

                                const double fac = 1;

                                if (iter == 0 && p != NULL)
                                        for (int j = 0; j < 6; ++j)
                                                L[k][j] = p->L[j];  //On remplit la matrice d'interaction globale
                                error[k] = p->error; //On remplit la matrice d'erreur

                                if (error[k] <= limite)
                                        local_count = local_count + 1; //Si erreur proche de 0 on incremente cur

                                w[k] = 1;

                                if (iter == 0) {
                                        factor[k] = fac;
                                        const vpPointSite &site = p->s;
                                        //if (site.suppress != 0) factor[n] = 0;
                                        if (site.suppress != 0)
                                                factor[k] = 0.2;
                                }
                        }
                        if (local_count != 0.0)
                #pragma omp critical
                        {
                                count += local_count;
                        }
                }
                count = count / (double) nbrow;
                if (count < 0.85) {
                        reloop = true;
                }


                double num = 0;
                double den = 0;

                double wi;
                double eri;
                for (int i = 0; i < nerror; i++) {
                        wi = w[i] * factor[i];
                        eri = error[i];
                        num += wi * vpMath::sqr(eri);
                        den += wi;

                        weighted_error[i] = wi * eri;
                }

                if ((iter == 0) || compute_interaction) {
                        for (int i = 0; i < nerror; i++) {
                                for (int j = 0; j < 6; j++) {
                                        L[i][j] = w[i] * factor[i] * L[i][j];
                                }
                        }
                }

                //        std::cout << "t-2 = " << vpTime::measureTimeMs() - t0 << std::endl;
                //        t0 = vpTime::measureTimeMs();
                CCDTracker.updateCCDPoints(cMo);
                CCDTracker.computeLocalStatistics();
                //        std::cout << "t-1 = " << vpTime::measureTimeMs() - t0 << std::endl;
                //        t0 = vpTime::measureTimeMs();
                CCDTracker.updateParameters(LTCIL, LTCIR);
                //        double t1 = vpTime::measureTimeMs();
                //        std::cout << "t0 = " << vpTime::measureTimeMs() - t0 << std::endl;

                if (iter > 0)
                CCDTracker.checkCCDConvergence();

                //		LTL = L.AtA();
                //        t0 = vpTime::measureTimeMs();
                L.AtA(LTL);
                computeJTR(L, weighted_error, LTR);
                //        std::cout << "t1 = " << vpTime::measureTimeMs() - t0 << std::endl;
                //        t0 = vpTime::measureTimeMs();
                v = -0.7 * (LTL + weight_ccd * LTCIL).pseudoInverse(LTL.getRows()
                                * DBL_EPSILON) * (LTR - weight_ccd * LTCIR);
                //cMo = vpExponentialMap::direct(v).inverse() * cMo;
                //        std::cout << "t2 = " << vpTime::measureTimeMs() - t0 << std::endl;

*/

                iter++;
        }
        /*std::cout << "\t First minimization in " << iter << " iteration "
         << std::endl;*/

        /*** Second phase ***/

        vpColVector W_true;
        vpMatrix L_true;
        vpRobust robust(nerror);
        robust.setIteration(0);

        vpRobust robustCCD(CCDTracker.nerror_ccd);
        robustCCD.setIteration(0);
        robustCCD.setThreshold(2 / cam.get_px());

        //CCDTracker.initRobust();

        iter = 0;

        while (((int) ((residu_1 - r) * 1e8) != 0) && (iter < 30))
        {
            //double t0 = vpTime::measureTimeMs();
            cMct = cMo*oMct;

            R0[0][0] = cMct[0][0];
            R0[0][1] = cMct[0][1];
            R0[0][2] = cMct[0][2];
            R0[1][0] = cMct[1][0];
            R0[1][1] = cMct[1][1];
            R0[1][2] = cMct[1][2];
            R0[2][0] = cMct[2][0];
            R0[2][1] = cMct[2][1];
            R0[2][2] = cMct[2][2];

            t0[0]=-cMct[0][3];
            t0[1]=-cMct[1][3];
            t0[2]=-cMct[2][3];

            t0 = R0.inverse()*t0;

            cMo0.buildFrom(t0,R0);

            string messageStr;
            messageStr = std::to_string(cMo0[0][3]) + " " + std::to_string(cMo0[1][3]) + " " + std::to_string(cMo0[2][3]) + " "
                    + std::to_string(cMo0[0][0]) + " " + std::to_string(cMo0[0][1]) + " " + std::to_string(cMo0[0][2]) + " "
                    + std::to_string(cMo0[1][0]) + " " + std::to_string(cMo0[1][1]) + " " + std::to_string(cMo0[1][2]) + " "
                    + std::to_string(cMo0[2][0]) + " " + std::to_string(cMo0[2][1]) + " " + std::to_string(cMo0[2][2]) + " ";

            zmq::message_t message(messageStr.length());
            memcpy(message.data(), messageStr.c_str(), messageStr.length());

            bool status = m_socketPub->send(message);

            if(!status)
            std::cout << "Problem with communication" << std::endl;

            cv::Mat  img;
            cv::Mat img1 = Mat::zeros( hght,wdth, CV_8UC1);

            zmq::message_t message1;
            bool status1 = m_socketSub->recv(&message1);
            if(status1)
            {
             std::string rpl = std::string(static_cast<char*>(message1.data()), message1.size());
             const char *cstr = rpl.c_str();
             loadImage(img,cstr);
             //Assign pixel value to img

             int ptr=0;
             for (int i = 0;  i < img1.rows; i++) {
              for (int j = 0; j < img1.cols; j++) {
               img1.at<uchar>(i,j) = img.at<uchar>(0,i*img1.cols+j);
               ptr=ptr+3;
               }
              }

            //cv::imwrite("socketimage100.png", img1);
            }

       cMo.extract(tr);

       sI.init(Ig.getHeight(), Ig.getWidth(),tr[2]);
       vpImageConvert::convert(img1,Ig);

       /*if (frame0 >= 10){
       for (int k = 0; k < imG.getHeight(); k++)
               for (int l = 0; l < imG.getWidth(); l++) {
                       if (Inormdprec[k][l].A == 0)
                       Ig[k][l] = 127;
               }
       }*/

       sI.buildFrom(Ig);
       sI.interaction(Lsd);
       sI.error(sId, errorG);

       vpImageTools::imageDifference(Ig,Igdgroundtruth,IdiffI);
       //vpDisplay::display(Ig);
       //vpDisplay::flush(Ig);
       //vpImageIo::write(Idiff, "Idiff.png");


       if (iter >1){

           Hsd = Lsd.AtA();
           diagHsd.eye(6);
           for(int i = 0 ; i < 6 ; i++) diagHsd[i][i] = Hsd[i][i];

           H = ((mu * diagHsd) + Hsd).pseudoInverse();

           computeJTR(Lsd, errorG, LTG);

       //std::cout << " v " << v << " cmo " << cMo << std::endl;

                #pragma omp parallel for
                for (int k = 0; k < points[scaleLevel].size(); k++) {
                        const int n = k;
                        apControlPoint *p = (points[scaleLevel])[k];
                        p->computeInteractionMatrixErrorMH(cMo, _I);
                        for (int j = 0; j < 6; j++) {
                                L[n][j] = p->L[j];
                                error[n] = p->error;
                        }
                }

                //std::cout << " v00 " << v << " cmo " << cMo << std::endl;

                if (iter == 0) {
                        weighted_error.resize(nerror);
                        w.resize(nerror);
                        w = 1;

                        robust.setThreshold(2 / cam.get_px()); // limite en metre
                        robust.MEstimator(vpRobust::TUKEY, error, w);
                } else {
                        robust.setIteration(iter);
                        robust.MEstimator(vpRobust::TUKEY, error, w);
                }

                residu_1 = r;

                L_true = L;
                W_true = vpColVector(nerror);

                double num = 0;
                double den = 0;
                double wi;
                double eri;
                for (int i = 0; i < nerror; i++) {
                        wi = w[i] * factor[i];
                        eri = error[i];
                        W_true[i] = wi * wi;
                        num += wi * vpMath::sqr(eri);
                        den += wi;

                        weighted_error[i] = wi * eri;
                }
                r = sqrt(num / den);

                if ((iter == 0) || compute_interaction) {
                        for (int i = 0; i < nerror; i++) {
                                for (int j = 0; j < 6; j++) {
                                        L[i][j] = w[i] * factor[i] * L[i][j];
                                }
                        }
                }
                L.AtA(LTL);
                computeJTR(L, weighted_error, LTR);


                // Error/Jacobian for intensity features

                //        t0 = vpTime::measureTimeMs();
                robustCCD.setIteration(iter);
                CCDTracker.updateCCDPoints(cMo);
                //        std::cout << "t2a = " << vpTime::measureTimeMs() - t0 << std::endl;
                //        t0 = vpTime::measureTimeMs();
                CCDTracker.computeLocalStatistics();
                //        std::cout << "t2b = " << vpTime::measureTimeMs() - t0 << std::endl;
                //        t0 = vpTime::measureTimeMs();
                //		double t0 = vpTime::measureTimeMs();
                //CCDTracker.updateParametersRobust(LTCIL, LTCIR, robustCCD);
                CCDTracker.updateParameters(LTCIL,LTCIR);

                //		double t1 = vpTime::measureTimeMs();
                //std::cout << " timeupdate " << t1 -t0 << std::endl;
                if (iter > 0)
                        CCDTracker.checkCCDConvergence();
                //        std::cout << "t2c = " << vpTime::measureTimeMs() - t0 << std::endl;
                //        t0 = vpTime::measureTimeMs();

                double stdME = sqrt((double)(weighted_error.t()*weighted_error)/(weighted_error.size()));

                double wghtME = ((double)1/weighted_error.size())*(1/stdME);
                double wghtCCD = ((double)1/CCDTracker.error_ccd.size());

                wghtME = 0.7;

                //double weight_P = 0.0000000004;

                double weight_P = 0.0000000004;

                /*std::cout << " hessian " <<  weight_P*Hsd << std::endl;
                std::cout << " ltcil " << weight_ccd* wghtCCD*LTCIL << std::endl;

                std::cout << " ltl " << weight_me* wghtME*LTL << std::endl;
                std::cout << " sum hessian " << weight_me* wghtME*LTL + weight_ccd* wghtCCD * LTCIL + weight_P*Hsd << std::endl;
                std::cout << " pseudo inverse 1 " << (weight_me* wghtME*LTL + weight_ccd* wghtCCD * LTCIL + weight_P*Hsd).pseudoInverse()  << std::endl;
                std::cout << " pseudo inverse 2 " << (weight_me* wghtME*LTL + weight_ccd* wghtCCD * LTCIL + weight_P*Hsd).pseudoInverse((LTL.getRows()) * DBL_EPSILON)  << std::endl;
                std::cout << " pseudo inverse 3 " << (weight_me* wghtME*LTL + weight_ccd* wghtCCD * LTCIL).pseudoInverse((LTL.getRows()) * DBL_EPSILON)  << std::endl;
                std::cout << " pseudo inverse 4 " << (weight_me* wghtME*LTL + weight_ccd* wghtCCD * LTCIL).inverseByLU()  << std::endl;*/

                //Synthetic
                //if (frame0 < 10)

                //Real arterial
                 if (frame0 < 20)
                {v = -3*lambda * ((1 * diagHsd) + Hsd).inverseByLU() * (LTG);
                 v = -4*lambda * (weight_me* wghtME*LTL + 0.25*weight_ccd* wghtCCD * LTCIL + weight_P*Hsd).pseudoInverse() * (weight_me* wghtME*LTR - 0.25*weight_ccd * wghtCCD * LTCIR + weight_P*LTG);

                }
               else
                {

                    //for(int i = 0 ; i < 6 ; i++) diagHsd[i][i] = (weight_me* wghtME*LTL + weight_ccd* wghtCCD * LTCIL + 0.5*weight_P*Hsd)[i][i];
                    this->me.range = 60;
                    v = -1*lambda * (weight_me* wghtME*LTL + weight_ccd* wghtCCD * LTCIL).pseudoInverse((LTL.getRows()) * DBL_EPSILON) * (weight_me* wghtME*LTR - weight_ccd * wghtCCD * LTCIR);

                }

                cMo = vpExponentialMap::direct(v).inverse() * cMo;
        }
                iter++;
        }

        if (computeCovariance) {
                vpMatrix D;
                D.diag(W_true);
                covarianceMatrixME = computeCovarianceMatrix(L_true, -v, lambda * error,
                                D);


                covarianceMatrixCCD = CCDTracker.sigmaP;

                vpMatrix inv = (weight_me*(covarianceMatrixME.pseudoInverse()) + weight_ccd*(covarianceMatrixCCD.pseudoInverse()) );
                covarianceMatrix = inv.pseudoInverse();
        }

        //   cout << "\t Robust minimization in " << iter << " iteration " << endl ;
        //    std::cout << "error: " << (residu_1 - r) << std::endl;
}


/*!
 Export 3D-2d correspondences in the image.

 \param I : the image.
 */
/*void apMbTracker::exportCorrespondencesEdges(const vpImage<unsigned char> &I) {

std::vector <std::pair <point3d,point2d>> correspondences;

string messageStr;
string messagePair;

int length = 0;

//#pragma omp parallel for
        for (int k = 0; k < points[scaleLevel].size(); k++)
        {
            vpPointSite site = points[scaleLevel][k]->s;
            std::pair <point3d,point2d> correspondence;
            point3d p3d;
            p3d.x = points[scaleLevel][k]->cpointo.get_oX();
            p3d.y = points[scaleLevel][k]->cpointo.get_oY();
            p3d.z = points[scaleLevel][k]->cpointo.get_oZ();

            point2d p2d;
            p2d.i = site.i;
            p2d.j = site.j;

            correspondence.first = p3d;
            correspondence.second = p2d;

            correspondences.push_back(correspondence);


            //savepair(buffer,correspondence);
            savepair(messagePair,correspondence);
            messageStr += messagePair;
            length += messagePair.length();
        }

        zmq::message_t message(length);

        memcpy(message.data(), messageStr.c_str(), length);

        bool status = m_socketPub->send(message);
        std::cout << "Problem with communication" <<  messageStr.length() << std::endl;

}*/

/*!
 Export 3D-2d correspondences in the image.

 \param I : the image.
 */
void apMbTracker::exportCorrespondencesEdges(const vpImage<unsigned char> &I) {

std::vector <std::pair <point3d,point2d>> correspondences;

string messageStr;
string messagePoint3d;
string messagePoint2d;

int length = 0;

//#pragma omp parallel for
        for (int k = 0; k < points[scaleLevel].size(); k++)
        {
            vpPointSite site = points[scaleLevel][k]->s;
            std::pair <point3d,point2d> correspondence;
            point3d p3d;

            vpColVector vertexop(4);
            vpColVector vertex(4);

            vertexop[0] = points[scaleLevel][k]->cpointo.get_oX();
            vertexop[1] = points[scaleLevel][k]->cpointo.get_oY();
            vertexop[2] = points[scaleLevel][k]->cpointo.get_oZ();
            vertexop[3] = 1;

            //vertex = opMo*vertexop;
            vertex = vertexop;

            p3d.x = vertex[0];
            p3d.y = vertex[1];
            p3d.z = vertex[2];

            point2d p2d;
            p2d.i = site.i;
            p2d.j = site.j;

            correspondence.first = p3d;
            correspondence.second = p2d;


            if (site.suppress==0){
            correspondences.push_back(correspondence);

            save3dpoint(messagePoint3d,p3d);
            messageStr += messagePoint3d;
            length += messagePoint3d.length();
            }

        }

        messageStr += ";";

        for (int k = 0; k < points[scaleLevel].size(); k++)
        {
            vpPointSite site = points[scaleLevel][k]->s;

            point2d p2d;
            p2d.i = site.i;
            p2d.j = site.j;

            if (site.suppress==0){
            save2dpoint(messagePoint2d,p2d);
            messageStr += messagePoint2d;
            length += messagePoint2d.length();
            }

        }
        zmq::message_t message(length);

        memcpy(message.data(), messageStr.c_str(), length);
        //std::cout << " message correspondences " << messageStr << std::endl;

        bool status = m_socketPub->send(message);
        std::cout << "Problem with communication" <<  messageStr.length() << std::endl;

}

/*!
 Export 3D-2d correspondences in the image.

 \param I : the image.
 */
void apMbTracker::exportCorrespondencesEdgesMean(const vpImage<unsigned char> &I) {

std::vector <std::pair <point3d,point2d>> correspondences;

string messageStr;
string messagePoint3d;
string messagePoint2d;

int length = 0;

vpImagePoint ip;


//#pragma omp parallel for
        for (int k = 0; k < controlpoints.size(); k++)
        {
            std::pair <point3d,point2d> correspondence;

            int kk = 0;
            double meancorri = 0;
            double meancorrj = 0;

            double i0, j0;

            vpMeterPixelConversion::convertPoint(cam, controlpoints[k].x/controlpoints[k].z, controlpoints[k].y/controlpoints[k].z, j0,i0);

            for (int j = 0; j< points[scaleLevel].size(); j++)
            {
            point3d p3d;
            vpPointSite site = points[scaleLevel][j]->s;


            vpColVector vertexop(4);

            vertexop[0] = points[scaleLevel][j]->cpointo.get_oX();
            vertexop[1] = points[scaleLevel][j]->cpointo.get_oY();
            vertexop[2] = points[scaleLevel][j]->cpointo.get_oZ();
            vertexop[3] = 1;

        std::cout << site.i << " " << site.j << " i0 " << i0 << " j0 " << j0 << " " << controlpoints[k].x << " " << cam.get_u0() << std::endl;

            //if (sqrt((i0 - site.i_1)*(i0 - site.i_1) + (j0 - site.j_1)*(j0 - site.j_1)) < 20 && site.suppress==0)
            if (sqrt((controlpoints[k].x - vertexop[0])*(controlpoints[k].x - vertexop[0]) + (controlpoints[k].y - vertexop[1])*(controlpoints[k].y - vertexop[1]) + (controlpoints[k].z - vertexop[2])*(controlpoints[k].z - vertexop[2])) < 0.05 && site.suppress==0)
            {

            meancorri += site.i;
            meancorrj += site.j;

            kk++;

            }

            //vertex = opMo*vertexop;
            }
            point2d p2d;


        std::cout << " mean corr " << meancorri << std::endl;
            if (kk>0){
            meancorri /= (double)kk;
            meancorrj /= (double)kk;

            p2d.i = (int)meancorri;
            p2d.j = (int)meancorrj;
            }
            else
            {
                p2d.i = (int)i0;
                p2d.j = (int)j0;
            }

            ip.set_i(p2d.i);
            ip.set_j(p2d.j);

            vpDisplay::displayCross(I,ip,4,vpColor::blue,4);


            correspondence.first = controlpoints[k];
            correspondence.second = p2d;


            {
            correspondences.push_back(correspondence);

            save3dpoint(messagePoint3d,controlpoints[k]);
            messageStr += messagePoint3d;
            length += messagePoint3d.length();
            }

        }

        messageStr += ";";

        for (int k = 0; k < correspondences.size(); k++)
        {
            point2d p2d = correspondences[k].second;

            save2dpoint(messagePoint2d,p2d);
            messageStr += messagePoint2d;
            length += messagePoint2d.length();

        }
        zmq::message_t message(length);

        memcpy(message.data(), messageStr.c_str(), length);
        //std::cout << " message correspondences " << messageStr << std::endl;

        bool status = m_socketPub->send(message);
        std::cout << "Problem with communication" <<  messageStr.length() << std::endl;

}

void apMbTracker::exportCorrespondencesKLT(const vpImage<unsigned char> &I) {

std::vector <std::pair <point3d,point2d>> correspondences;

string messageStr;
string messagePoint3d;
string messagePoint2d;

int length = 0;

vpImagePoint ip;


//#pragma omp parallel for
        for (int k = 0; k < controlpoints.size(); k++)
        {
            std::pair <point3d,point2d> correspondence;

            int kk = 0;
            double meancorri = 0;
            double meancorrj = 0;

            double i0, j0;

            vpMeterPixelConversion::convertPoint(cam, controlpoints[k].x/controlpoints[k].z, controlpoints[k].y/controlpoints[k].z, j0,i0);

            float x,y;
            long id;
            kltTracker.getFeature((int)k, id, x, y);
            point2d p2d;

            p2d.i = (int)y;
            p2d.j = (int)x;

            ip.set_i(p2d.i);
            ip.set_j(p2d.j);

            vpDisplay::displayCross(I,ip,4,vpColor::blue,4);


            correspondence.first = controlpoints[k];
            correspondence.second = p2d;


            {
            correspondences.push_back(correspondence);

            save3dpoint(messagePoint3d,controlpoints[k]);
            messageStr += messagePoint3d;
            length += messagePoint3d.length();
            }

        }

        messageStr += ";";

        for (int k = 0; k < correspondences.size(); k++)
        {
            point2d p2d = correspondences[k].second;

            save2dpoint(messagePoint2d,p2d);
            messageStr += messagePoint2d;
            length += messagePoint2d.length();

        }
        zmq::message_t message(length);

        memcpy(message.data(), messageStr.c_str(), length);
        //std::cout << " message correspondences " << messageStr << std::endl;

        bool status = m_socketPub->send(message);
        std::cout << "Problem with communication" <<  messageStr.length() << std::endl;

}

void apMbTracker::exportCorrespondencesEdges2D(std::vector<std::vector<point2d>> &trackededgesIm, std::vector<std::vector<int>> &suppressIm) {


    string messagePoint2d;
    string messageSuppress;
    string messageStr;

    int length = 0;

    for (int im = 0; im < trackededgesIm.size(); im++)
    {
    for (int k = 0; k < trackededgesIm[im].size(); k++)
    {
        point2d p2d;
        p2d.i =  trackededgesIm[im][k].i;
        p2d.j =  trackededgesIm[im][k].j;

        std::cout << " export p2d " <<  p2d.i << " " << p2d.j << std::endl;
        {
        save2dpoint(messagePoint2d,p2d);
        messageStr += messagePoint2d;
        length += messagePoint2d.length();
        }
    }
    messageStr += ";";

    for (int k = 0; k < suppressIm[im].size(); k++)
    {
        messageSuppress = std::to_string(suppressIm[im][k]) + " ";
        messageStr += messageSuppress;
        length += messageSuppress.length();

        std::cout << " export supp " << suppressIm[im][k] << std::endl;


    }

    messageStr += ";";

    }


    /*for (int im = 0; im < suppressIm.size(); im++)
    {
    messageStr += ";";
    }*/

    zmq::message_t message(messageStr.length());
    memcpy(message.data(), messageStr.c_str(), messageStr.length());
    std::cout << " message correspondences " << messageStr << std::endl;

    bool status = m_socketPub->send(message);
    std::cout << "Problem with communication send " <<  messageStr.length() << std::endl;

}


void apMbTracker::trackXray(const vpImage<unsigned char> &I, double dist) {
    initPyramid(I, Ipyramid);
    initPyramid(Iprec, Ipyramidprec);

    for (int lvl = (scales.size() - 1); lvl >= 0; lvl -= 1) {
        if (scales[lvl]) {
            vpHomogeneousMatrix cMo_1 = cMo;
            try {
                downScale(lvl);

                try {
                    double t4 = vpTime::measureTimeMs();
                    computeVVSPhotometric(*Ipyramid[lvl]);
                    double t5 = vpTime::measureTimeMs();
                    std::cout << "timeVVS " << t5 - t4 << std::endl;
                } catch (...) {
                    vpTRACE("Error in computeVVS");
                    throw vpException(vpException::fatalError,
                            "Error in computeVVS");
                }

            } catch (...) {
                if (lvl != 0) {
                    cMo = cMo_1;
                    reInitLevel(lvl);
                    upScale(lvl);
                } else {
                    upScale(lvl);
                    throw;
                }
            }
        }
    }

    cleanPyramid(Ipyramid);
    cleanPyramid(Ipyramidprec);
    Iprec = I;
}


void loadImage(cv::Mat & mat, const char * data_str)
{
    std::stringstream ss;
    ss << data_str;

    boost::archive::text_iarchive tia(ss);
    tia >> mat;
}

void desserializeuc(std::istream &in, cv::Mat &s)
{
    std::string tmpDelim;
    uchar *buff = (uchar *)(s.data);
    bool read = true;
    while (read)
    {
        in >> tmpDelim;
        if (tmpDelim == "[") continue;
        if (tmpDelim.find(']') != std::string::npos)
            break;
        else
        {
            *buff = atoi(tmpDelim.c_str());
            buff++;
        }

    }
    //std::cout << " img " << s << std::endl;
}

void desserializec(std::istream &in, cv::Mat &s)
{
    std::string tmpDelim;
    char *buff = (char *)(s.data);
    bool read = true;
    while (read)
    {
        in >> tmpDelim;
        if (tmpDelim == "[") continue;
        if (tmpDelim.find(']') != std::string::npos)
            break;
        else
        {
            *buff = atoi(tmpDelim.c_str());
            buff++;
        }
    }
}

void desserializeus(std::istream &in, cv::Mat &s)
{
    std::string tmpDelim;
    ushort *buff = (ushort *)(s.data);
    bool read = true;
    while (read)
    {
        in >> tmpDelim;
        if (tmpDelim == "[") continue;
        if (tmpDelim.find(']') != std::string::npos)
            break;
        else
        {
            *buff = atoi(tmpDelim.c_str());
            buff++;
        }
    }
}

void desserializes(std::istream &in, cv::Mat &s)
{
    std::string tmpDelim;
    ushort *buff = (ushort *)(s.data);
    bool read = true;
    while (read)
    {
        in >> tmpDelim;
        if (tmpDelim == "[") continue;
        if (tmpDelim.find(']') != std::string::npos)
            break;
        else
        {
            *buff = atoi(tmpDelim.c_str());
            buff++;
        }
    }
}

void desserializei(std::istream &in, cv::Mat &s)
{
    std::string tmpDelim;
    short *buff = (short *)(s.data);
    bool read = true;
    while (read)
    {
        in >> tmpDelim;
        if (tmpDelim == "[") continue;
        if (tmpDelim.find(']') != std::string::npos)
            break;
        else
        {
            *buff = atoi(tmpDelim.c_str());
            buff++;
        }
    }
}

void desserializef(std::istream &in, cv::Mat &s)
{
    std::string tmpDelim;
    float *buff = (float *)(s.data);
    bool read = true;
    while (read)
    {
        in >> tmpDelim;
        if (tmpDelim == "[") continue;
        if (tmpDelim.find(']') != std::string::npos)
            break;
        else
        {
            *buff = atof(tmpDelim.c_str());
            buff++;
        }
    }
}

void desserialized(std::istream &in, cv::Mat &s)
{
    std::string tmpDelim;
    double *buff = (double *)(s.data);
    bool read = true;
    while (read)
    {
        in >> tmpDelim;
        if (tmpDelim == "[") continue;
        if (tmpDelim.find(']') != std::string::npos)
            break;
        else
        {
            *buff = atof(tmpDelim.c_str());
            buff++;
        }
    }
}


void desserialize(std::istream &in, cv::Mat &s)
{
    int rows, cols, type;

    in >> rows >> cols >> type;

    //std::cout << " rows " << rows << " cols " << cols << " type " << type << std::endl;
    s.create(rows, cols, type);
    //std::cout << " rows " << rows << " cols " << cols << " type " << type << std::endl;

    switch (s.depth())
    {
    case CV_8U:
        desserializeuc(in, s);
        break;
    case CV_8S:
        desserializec(in, s);
        break;
    case CV_16U:
        desserializeus(in, s);
        break;
    case CV_16S:
        desserializes(in, s);
        break;
    case CV_32S:
        desserializei(in, s);
        break;
    case CV_32F:
        desserializef(in, s);
        break;
    case CV_64F:
        desserialized(in, s);
        break;
    }
}


#endif // ENABLE_ZMQ




