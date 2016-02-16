/*
 * apPFFilter.cpp
 *
 *  Created on: Mar 1, 2012
 *      Author: agpetit
 */
#include <omp.h>
#include "apPFilter.h"


apPFilter::apPFilter() {
	// TODO Auto-generated constructor stub

}

apPFilter::~apPFilter() {
	// TODO Auto-generated destructor stub
	//delete ModelView;
	apContourPoint *cpoints;
	vpImage<unsigned char> *MView;
	std::vector<apContourPoint*> cpointv;
	for (int i = 1; i < ContourPoints.size() ; i++)
	{
		cpoints = ContourPoints[i];
		delete cpoints;
	}
	for (int i = 1; i<ContourPointsSc.size();i++)
	{
		cpointv = ContourPointsSc[i];
		for (int j = 1; j < cpointv.size() ; j++)
		{
			cpoints = cpointv[j];
			delete cpoints;
		}
	}
	for (int i = 1; i < ModelViewSc.size() ; i++)
	{
		MView = ModelViewSc[i];
		delete MView;
	}
}

/*!
  Set the associated prototype view and its scale pyramid,
  in terms of edge maps or vectors of contour points.


  \param MView : the original prototype model view (edge oriented map).
  \param cg : center of gravity of the silhouette of the model view.
  \param surf : area of the silhouette of the model view.
  \param ori : orientation of the silhouette of the model view.
*/

void apPFilter::setModel(vpImage<unsigned char> &MView, vpImagePoint &cg, int surf, double ori, vpPoseVector &_pose)
{
ModelView = MView;
cog=cg;
surface=surf;
orientation = ori;
pose = _pose;
ContourPoints.resize(1);
ContourPointsSc.resize(1);
ModelViewSc.resize(1);
vpImage<unsigned char>* mviewsc;
double scale;
std::vector<apContourPoint*> contourpoints;
double resx,resy;
for (int n = 0; n < detect.nscales; n++)
{
	if (n<floor(detect.nscales/2))
	{
		scale = 0.3+n*(1-0.3)/floor(detect.nscales/2);
	}
	else
	{
		scale = 1+4*(n-floor(detect.nscales/2))/floor(detect.nscales/2);
	}
	contourpoints.resize(1);
	mviewsc = new vpImage<unsigned char>;
	mviewsc->resize(ModelView.getHeight(), ModelView.getWidth());
	resize(ModelView,*mviewsc,scale);
	ModelViewSc.push_back(mviewsc);
	apContourPoint *cp;
	for (int y = 0; y < ModelView.getHeight(); y++)
	  for (int x = 0; x < ModelView.getWidth(); x++) {
		resx = (double)x/detect.sample-(double)floor(x/detect.sample);
		resy = (double)y/detect.sample-(double)floor(y/detect.sample);
		  if((resy<0.0005 || resx <0.0005) && ((*mviewsc)[y][x]!=100) && x!=(int)ModelView.getWidth()/2)
		  //if(((*mviewsc)[y][x]!=100) && x!=(int)ModelView.getWidth()/2)
		  {
		  cp = new apContourPoint;
		  cp->set_u(x);
		  cp->set_v(y);
		  cp->set_ori((*mviewsc)[y][x]);
		  contourpoints.push_back(cp);
		  }
	  }
	ContourPointsSc.push_back(contourpoints);


}
apContourPoint *cp;
std::vector<double> xx;
std::vector<double> yy;
for (int y = 0; y < ModelView.getHeight(); y++)
  for (int x = 0; x < ModelView.getWidth(); x++) {
		resx = (double)x/detect.sample-(double)floor(x/detect.sample);
		resy = (double)y/detect.sample-(double)floor(y/detect.sample);
		if((resy<0.0005 || resx <0.0005) && ((*mviewsc)[y][x]!=100) && x!=(int)ModelView.getWidth()/2)
	  //if(((ModelView)[y][x]!=100) && x!=(int)ModelView.getWidth()/2)
	  {
	  cp = new apContourPoint;
	  cp->set_u(x);
	  cp->set_v(y);
	  cp->set_ori(ModelView[y][x]);
	  ContourPoints.push_back(cp);
	  xx.push_back(x);
	  yy.push_back(y);
	  }
  }
apLogPolarHist lgPHist;
logPolarHist = lgPHist.RunCenter(xx,yy,cg,detect.nr,detect.nw);

}

/*!
  Init the particles in the case of a shape context-based similarity measure.

*/

void apPFilter::initSC()
{
	ctParticle *prcl;
	//Initialisation of the logpolar histograms
	for (int i = 0 ; i < particleVect.size(); i++)
	{
		prcl = particleVect[i];
		prcl->initLogPolarHist(detect.nr,detect.nw, ModelView.getHeight(), ModelView.getWidth());
		prcl->set_sx(detect.sx);
		prcl->set_sy(detect.sy);
		prcl->set_stheta(detect.stheta);
		prcl->set_sigmauv(sigmauv);
		prcl->set_sigmar(sigmar);
		prcl->set_nr(detect.nr);
		prcl->set_nw(detect.nw);
		//particleVect[i] = prcl;
	}
}

/*!
  Compute or load the shape contexts for the scale pyramid, in terms of their log polar histograms.

  SHOULD BE ADDED TO THE LEARNING PART
\param filename : path to the files to load or save the histograms
\param nfilt : index of the filter

*/

void apPFilter::computeSC(std::string filename, int nfilt)
{
	logPolarHistV.resize(1);
	apLogPolarHist lgPHist;
	lgPHist.init(detect.nr,detect.nw,ModelView.getHeight(), ModelView.getWidth());
	std::vector<double> logPolarHist_;
	std::vector<std::vector<double>*> logPolarHistV_;
	//logPolarHistV_.resize(1);
	std::vector<apContourPoint*> _contourpoints;
	std::vector<double> *logPolarHist0_;
	double theta;
	vpImagePoint pI;
	int ncs=0;
	vpImagePoint center;
	center.set_u(cam.get_u0());
	center.set_v(cam.get_v0());
	//center = cog;

    char buf[FILENAME_MAX];

	for (int kk = 0; kk < ContourPointsSc.size()-1; kk++)
	{
		double t0= vpTime::measureTimeMs();
       _contourpoints = ContourPointsSc[kk+1];
		//std::cout << " sizecp " << _contourpoints.size()-1 << std::endl;
       for (int l = 0; l < sample_x;l++)
    	   for (int m = 0; m < sample_y; m++ )
    		   for(int n = 0; n<sample_theta; n++)
    		   {
                   pI.set_u(center.get_u()-2*sigmauv + l*(4*sigmauv)/((double)sample_x-1));
                   pI.set_v(center.get_v()-2*sigmauv +  m*(4*sigmauv)/((double)sample_y-1));
                   //theta =-M_PI/2 + n*(M_PI/sample_theta);
                   //std::cout << " pi u " << pI.get_u() << " pi v " << pI.get_v() << std::endl;
                   theta =0;
    			   sprintf(buf, filename.c_str(), nfilt*1000 + ncs);
    			   std::string filename_(buf);

    			   /*logPolarHist_ = lgPHist.RunCenterRotOpt(_contourpoints,center,pI,theta);
    			   lgPHist.SaveToFile(filename_,logPolarHist_);*/

    			   logPolarHist_ = lgPHist.ReadFromFile(filename_);
    			   logPolarHistV.push_back(logPolarHist_);
    			   ncs++;
    			   /*logPolarHist0_ = new std::vector<double>;
    			   *logPolarHist0_ = (lgPHist.RunCenterRotOpt(_contourpoints,cog,pI,theta));
                   logPolarHistV_.push_back(logPolarHist0_);*/
    		   }


		/*apContourPoint *cp;
		for (int y = 0; y < ModelView.getHeight(); y++)
		  for (int x = 0; x < ModelView.getWidth(); x++) {
			  if(((*mviewsc)[y][x]!=100) && x!=(int)ModelView.getWidth()/2)
			  {
			  cp = new apContourPoint;
			  cp->set_u(x);
			  cp->set_v(y);
			  cp->set_ori((*mviewsc)[y][x]);
			  contourpoints.push_back(cp);
			  }
		  }
		ContourPointsSc.push_back(contourpoints);*/
   	double t1= vpTime::measureTimeMs();
   	//std::cout << " sizelogVect " << logPolarHistV.size()  << std::endl;
	}

}


/*!
  Compute the likelihoods of the particles of the filter for an Oriented Chamfer based similarity measure.

\param dTOI : oriented edge map (distance transform) of the input image.
\param dTOIIseg : oriented edge map (distance transform) of the segmented input image.

*/
void apPFilter::likelihoodViewOC(vpImage<vpRGBa> *dTOI, vpImage<vpRGBa> *dTOIseg, bool weights)
{
	//  	vpTRACE("Start colLikelihood");
		err_sum = 0;
		err_min = 0;
		err_max = 0;
		float oriefpart=0;

		float err_curr = 0;
		float err_curr1 = 0;
		float err_curr2 = 0;
		float err_curr3 = 0;
		float err_curr4 = 0;
		ctParticle *prcl;
		int nPI = 0;
		int n0 = 0;
		//int nPI2 =0;
		//int nPI2i = 0;
		//std::cout << "orientation " << orientation << " surface " << surface << std::endl;

		nb_prcl_out = 0; //nb de particules pas enti�rement dans l'image

	    std::string simpath = "/local/agpetit/soft/apTracking/mbt-co/build/simOC.txt";
	    //fstream fout("/local/agpetit/soft/apTracking/mbt-co/build/simOCRotZ2.txt", std::ios_base::out);


		int ng = 100;
	    //vpMatrix sim(ng*ng,3);

		//!-------------------------Parcours de la liste de particules-----------------------//

        //#pragma omp parallel for
		//for (int k = 0; k < ng ; k++)
		//{
			//for (int l = 0; l < ng; l++)
			//{
		for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
		{
			//int i = k*ng + l;
			prcl = particleVect[i];

			 err_curr1 = prcl->computeSimErrorCPV(dTOI,ContourPointsSc,surface,orientation, weights);
			 err_curr2 = prcl->computeSimErrorCPV(dTOI,ContourPointsSc,surface,orientation-M_PI, weights);

			//err_curr3 = prcl->computeSimErrorCPV(dTOIseg,ContourPointsSc,surface,orientation-M_PI/2, weights);
			//err_curr4 = prcl->computeSimErrorCPV(dTOIseg,ContourPointsSc,surface,orientation+M_PI/2, weights);



			if (err_curr1 > err_curr2)// && err_curr2 < err_curr3 && err_curr2 < err_curr4)
			{
				err_curr = err_curr2;
				oriefpart = M_PI;
				nPI++;
			}
			else if (err_curr2 > err_curr1)// && err_curr1 < err_curr3 && err_curr1 < err_curr4)
			{
				err_curr = err_curr1;
				oriefpart = 0;
				n0++;
			}
			err_curr3 = prcl->computeSimErrorCPV(dTOIseg,ContourPointsSc,surface,orientation-oriefpart, weights);


			/*else if (err_curr2 > err_curr3 && err_curr3 < err_curr1 && err_curr3 < err_curr4)
			{
				err_curr = err_curr3;
				oriefpart = M_PI/2;
				nPI2++;
			}
			else if (err_curr2 > err_curr4 && err_curr1 > err_curr4 && err_curr3 > err_curr4)
			{
				err_curr = err_curr4;
				oriefpart = -M_PI/2;
				nPI2i++;
			}*/
			//err_curr3 = prcl.computeSimErrorCPV(dTOI,ContourPointsSc,surface,orientation-oriefpart, weights);

			err_curr = 0.5*(err_curr+err_curr3);
			//err_curr = err_curr1;

			//! On compte les particules qui ne sont pas enti�rement dans l'image (pour ensuite d�tecter si l'objet est perdu)

			if (prcl->get_nbPartOut() > 0) {//4) {// object_ref.get_nbPart()/2) {
				nb_prcl_out ++;
			}
			//std::cout << " err_curr0 " << particleVect[i].getDist() << std::endl;

			//!Associate error with particle (could be done in ctParticle.cpp ?)
			//!Note : Tester : set distance as a mean of err_curr and err_curr_global ?
			prcl->setDist(err_curr);
			prcl->setOri(oriefpart);
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

			//std::cout << " err_curr " << err_curr << std::endl;

			err_sum += err_curr;

			//! Update prcl
			//particleVect[i] = prcl;
			//std::cout << " err_curr1 " << particleVect[i].getDist() << std::endl;

	  //}
		//fout  << endl;
}

		//sim.saveMatrix(simpath,sim,false,"");

		if(nPI > n0 )//&& nPI > nPI2 && nPI > nPI2i)
			orieff = M_PI;
		else if(n0 > nPI)// && n0 > nPI2 && n0 > nPI2i)
			orieff = 0;
		/*else if(nPI2 > n0 && nPI2 > nPI && nPI2 > nPI2i)
			orieff = M_PI/2;
		else if(nPI2i > n0 && nPI2i > nPI && nPI2i > nPI2)
			orieff = -M_PI/2;*/
}

void apPFilter::likelihoodViewSteger(vpImage<unsigned char> &Igrad, vpImage<vpRGBa> *dTOIseg, bool weights)
{
	//  	vpTRACE("Start colLikelihood");
		err_sum = 0;
		err_min = 0;
		err_max = 0;
		float oriefpart=0;

		float err_curr = 0;
		float err_curr1 = 0;
		float err_curr2 = 0;
		float err_curr3 = 0;
		float err_curr4 = 0;
		ctParticle *prcl;
		int nPI = 0;
		int n0 = 0;
		//int nPI2 =0;
		//int nPI2i = 0;
		//std::cout << "orientation " << orientation << " surface " << surface << std::endl;

	    std::string simpath = "/local/agpetit/soft/apTracking/mbt-co/build/simSteger.txt";
	    //fstream fout("/local/agpetit/soft/apTracking/mbt-co/build/simStegerRotZ.txt", std::ios_base::out);


		int ng = 100;
	    vpMatrix sim(ng*ng,3);

		nb_prcl_out = 0; //nb de particules pas enti�rement dans l'image


		//!-------------------------Parcours de la liste de particules-----------------------//

        //#pragma omp parallel for
		//for (int k = 0; k < ng ; k++)
		//{
			//for (int l = 0; l < ng; l++)
			//{
		for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
		{
			//int i = k*ng + l;
			prcl = particleVect[i];

			err_curr1 = prcl->computeSimErrorSteger(Igrad,ContourPointsSc,surface,orientation, weights);
			err_curr2 = prcl->computeSimErrorSteger(Igrad,ContourPointsSc,surface,orientation-M_PI, weights);
			//err_curr3 = prcl->computeSimErrorCPV(dTOIseg,ContourPointsSc,surface,orientation-M_PI/2, weights);
			//err_curr4 = prcl->computeSimErrorCPV(dTOIseg,ContourPointsSc,surface,orientation+M_PI/2, weights);

			/*sim[i][0] = (double)k*360/ng;
			sim[i][1] = (double)l*360/ng;
			sim[i][2] = err_curr;*/




			if (err_curr1 > err_curr2)// && err_curr2 < err_curr3 && err_curr2 < err_curr4)
			{
				err_curr = err_curr2;
				oriefpart = M_PI;
				nPI++;
			}
			else if (err_curr2 > err_curr1)// && err_curr1 < err_curr3 && err_curr1 < err_curr4)
			{
				err_curr = err_curr1;
				oriefpart = 0;
				n0++;
			}
			//err_curr3 = prcl->computeSimErrorCPV(dTOIseg,ContourPointsSc,surface,orientation-oriefpart, weights);



			/*else if (err_curr2 > err_curr3 && err_curr3 < err_curr1 && err_curr3 < err_curr4)
			{
				err_curr = err_curr3;
				oriefpart = M_PI/2;
				nPI2++;
			}
			else if (err_curr2 > err_curr4 && err_curr1 > err_curr4 && err_curr3 > err_curr4)
			{
				err_curr = err_curr4;
				oriefpart = -M_PI/2;
				nPI2i++;
			}*/
			//err_curr3 = prcl.computeSimErrorCPV(dTOI,ContourPointsSc,surface,orientation-oriefpart, weights);


			//err_curr = 0.5*(err_curr+err_curr3);


			//err_curr = err_curr1;

			//! On compte les particules qui ne sont pas enti�rement dans l'image (pour ensuite d�tecter si l'objet est perdu)

			if (prcl->get_nbPartOut() > 0) {//4) {// object_ref.get_nbPart()/2) {
				nb_prcl_out ++;
			}


			//std::cout << " err_curr0 " << particleVect[i].getDist() << std::endl;

			//!Associate error with particle (could be done in ctParticle.cpp ?)
			//!Note : Tester : set distance as a mean of err_curr and err_curr_global ?
			prcl->setDist(err_curr);
			prcl->setOri(oriefpart);
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

			//std::cout << " err_curr " << err_curr << std::endl;

			err_sum += err_curr;

			//! Update prcl
			//particleVect[i] = prcl;
			//std::cout << " err_curr1 " << particleVect[i].getDist() << std::endl;

	  //}
		//fout << endl;
}

		//sim.saveMatrix(simpath,sim,false,"");
		if(nPI > n0 )//&& nPI > nPI2 && nPI > nPI2i)
			orieff = M_PI;
		else if(n0 > nPI)// && n0 > nPI2 && n0 > nPI2i)
			orieff = 0;
		/*else if(nPI2 > n0 && nPI2 > nPI && nPI2 > nPI2i)
			orieff = M_PI/2;
		else if(nPI2i > n0 && nPI2i > nPI && nPI2i > nPI2)
			orieff = -M_PI/2;*/
}


void apPFilter::likelihoodViewOCFast0(vpImage<vpRGBa> *dTOI, vpImage<vpRGBa> *dTOIseg, bool weights, int niseg)
{
	//  	vpTRACE("Start colLikelihood");
		err_sum = 0;
		err_min = 0;
		err_max = 0;
		float oriefpart=0;

		float err_curr = 0;
		float err_curr1 = 0;
		float err_curr2 = 0;
		float err_curr3 = 0;
		float err_curr4 = 0;
		ctParticle *prcl;
		int nPI = 0;
		int n0 = 0;
		int nPI2 =0;
		int nPI2i = 0;
		int nTest = 20;
		//std::cout << "orientation " << orientation << " surface " << surface << std::endl;

		nb_prcl_out = 0; //nb de particules pas enti�rement dans l'image


		//!-------------------------Parcours de la liste de particules-----------------------//

        //#pragma omp parallel for
		for (int i = 0 ; i < particleVect.size() ; i++)// Parcours de la liste de particules
		{
			prcl = particleVect[i];
			/*err_curr1 = prcl->computeSimErrorCPV(dTOIseg,ContourPointsSc,surface,orientation, weights);
			err_curr2 = prcl->computeSimErrorCPV(dTOIseg,ContourPointsSc,surface,orientation-M_PI, weights);
			err_curr3 = prcl->computeSimErrorCPV(dTOIseg,ContourPointsSc,surface,orientation-M_PI/2, weights);
			err_curr4 = prcl->computeSimErrorCPV(dTOIseg,ContourPointsSc,surface,orientation+M_PI/2, weights);*/
			//std::cout << " 0 " <<  std::endl;
			err_curr1 = prcl->computeSimErrorCPVN(dTOIseg,ContourPointsSc,surface,orientation, weights, niseg, detect.sample);
			//std::cout << " M_PI " <<  std::endl;
			err_curr2 = prcl->computeSimErrorCPVN(dTOIseg,ContourPointsSc,surface,orientation-M_PI, weights, niseg, detect.sample);

			//err_curr3 = prcl->computeSimErrorCPVN(dTOIseg,ContourPointsSc,surface,orientation-M_PI/2, weights, niseg, detect.sample);
			//err_curr4 = prcl->computeSimErrorCPVN(dTOIseg,ContourPointsSc,surface,orientation+M_PI/2, weights, niseg, detect.sample);


			if (err_curr1 > err_curr2 )//&& err_curr2 < err_curr3 && err_curr2 < err_curr4)
			{
				err_curr = err_curr2;
				oriefpart = M_PI;
				nPI++;
			}
			else if (err_curr2 > err_curr1 )//&& err_curr1 < err_curr3 && err_curr1 < err_curr4)
			{
				err_curr = err_curr1;
				oriefpart = 0;
				n0++;
			}
			/*else if (err_curr2 > err_curr3 && err_curr3 < err_curr1 && err_curr3 < err_curr4)
			{
				err_curr = err_curr3;
				oriefpart = M_PI/2;
				nPI2++;
			}
			else if (err_curr2 > err_curr4 && err_curr1 > err_curr4 && err_curr3 > err_curr4)
			{
				err_curr = err_curr4;
				oriefpart = -M_PI/2;
				nPI2i++;
			}*/
			//err_curr3 = prcl.computeSimErrorCPV(dTOI,ContourPointsSc,surface,orientation-oriefpart, weights);
			//err_curr = 0.5*(err_curr+err_curr3);
			//err_curr = err_curr1;

			//! On compte les particules qui ne sont pas enti�rement dans l'image (pour ensuite d�tecter si l'objet est perdu)

			if (prcl->get_nbPartOut() > 0) {//4) {// object_ref.get_nbPart()/2) {
				nb_prcl_out ++;
			}
			//std::cout << " err_curr0 " << particleVect[i].getDist() << std::endl;

			//!Associate error with particle (could be done in ctParticle.cpp ?)
			//!Note : Tester : set distance as a mean of err_curr and err_curr_global ?
			prcl->setDist(err_curr);
			prcl->setOri(oriefpart);
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

			//std::cout << " err_curr " << err_curr << std::endl;

			err_sum += err_curr;

			//! Update prcl
			//particleVect[i] = prcl;
			//std::cout << " err_curr1 " << particleVect[i].getDist() << std::endl;

	  }
		if(nPI > n0 )//&& nPI > nPI2 && nPI > nPI2i)
			orieff = M_PI;
		else if(n0 > nPI)// && n0 > nPI2 && n0 > nPI2i)
			orieff = 0;
		/*else if(nPI2 > n0 && nPI2 > nPI && nPI2 > nPI2i)
			orieff = M_PI/2;
		else if(nPI2i > n0 && nPI2i > nPI && nPI2i > nPI2)
			orieff = -M_PI/2;*/

		/*for (int i = nTest ; i < particleVect.size(); i++)// Parcours de la liste de particules
		{
			prcl = particleVect[i];
			err_curr = prcl->computeSimErrorCPV(dTOIseg,ContourPointsSc,surface,orientation-orieff, weights);

			//err_curr3 = prcl.computeSimErrorCPV(dTOI,ContourPointsSc,surface,orientation-oriefpart, weights);
			//err_curr = 0.5*(err_curr+err_curr3);
			//err_curr = err_curr1;

			//! On compte les particules qui ne sont pas enti�rement dans l'image (pour ensuite d�tecter si l'objet est perdu)

			if (prcl->get_nbPartOut() > 0) {//4) {// object_ref.get_nbPart()/2) {
				nb_prcl_out ++;
			}
			//std::cout << " err_curr0 " << particleVect[i].getDist() << std::endl;

			//!Associate error with particle (could be done in ctParticle.cpp ?)
			//!Note : Tester : set distance as a mean of err_curr and err_curr_global ?
			prcl->setDist(err_curr);
			prcl->setOri(orieff);
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

			//std::cout << " err_curr " << err_curr << std::endl;

			err_sum += err_curr;

			//! Update prcl
			//particleVect[i] = prcl;
			//std::cout << " err_curr1 " << particleVect[i].getDist() << std::endl;



	  }*/
}



void apPFilter::likelihoodViewOCFast(vpImage<vpRGBa> *dTOI, vpImage<vpRGBa> *dTOIseg, bool weights)
{
	//  	vpTRACE("Start colLikelihood");
		err_sum = 0;
		err_min = 0;
		err_max = 0;
		float oriefpart=0;

		float err_curr = 0;
		float err_curr1 = 0;
		float err_curr2 = 0;
		float err_curr3 = 0;
		float err_curr4 = 0;
		ctParticle *prcl;
		//std::cout << "orientation " << orientation << " surface " << surface << std::endl;

		nb_prcl_out = 0; //nb de particules pas enti�rement dans l'image


		//!-------------------------Parcours de la liste de particules-----------------------//

        //#pragma omp parallel for
		for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
		{
			prcl = particleVect[i];
			err_curr = prcl->computeSimErrorCPV(dTOIseg,ContourPointsSc,surface,orientation-orieff, weights);

			//err_curr3 = prcl.computeSimErrorCPV(dTOI,ContourPointsSc,surface,orientation-oriefpart, weights);
			//err_curr = 0.5*(err_curr+err_curr3);
			//err_curr = err_curr1;

			//! On compte les particules qui ne sont pas enti�rement dans l'image (pour ensuite d�tecter si l'objet est perdu)

			if (prcl->get_nbPartOut() > 0) {//4) {// object_ref.get_nbPart()/2) {
				nb_prcl_out ++;
			}
			//std::cout << " err_curr0 " << particleVect[i].getDist() << std::endl;

			//!Associate error with particle (could be done in ctParticle.cpp ?)
			//!Note : Tester : set distance as a mean of err_curr and err_curr_global ?
			prcl->setDist(err_curr);
			prcl->setOri(orieff);
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

			//std::cout << " err_curr " << err_curr << std::endl;

			err_sum += err_curr;

			//! Update prcl
			//particleVect[i] = prcl;
			//std::cout << " err_curr1 " << particleVect[i].getDist() << std::endl;



	  }
}


double apPFilter::computeSimErrorCPV(ctParticle *prcl, vpImage<vpRGBa> *I,bool weights)
{
	double err_curr = 0;
	float err_curr1 = 0;
	float err_curr2 = 0;
	float oriefpart=0;

	err_curr = prcl->computeSimErrorCPV(I,ContourPointsSc,surface,orientation, weights);
	//! On compte les particules qui ne sont pas enti�rement dans l'image (pour ensuite d�tecter si l'objet est perdu)

	if (prcl->get_nbPartOut() > 0) {//4) {// object_ref.get_nbPart()/2) {
		nb_prcl_out ++;
	}

	prcl->setDist(err_curr);
	prcl->setOri(oriefpart);

	return err_curr;

}

void apPFilter::likelihoodViewOCOpt(vpImage<vpRGBa> *dTOI, vpImage<vpRGBa> *dTOIseg, bool weights)
{
	//  	vpTRACE("Start colLikelihood");
		err_sum = 0;
		err_min = 0;
		err_max = 0;

		float err_curr = 0;
		ctParticle *prcl;

		double err[particleVect.size()];
		//std::cout << "orientation " << orientation << " surface " << surface << std::endl;
		nb_prcl_out = 0; //nb de particules pas enti�rement dans l'image

		//!-------------------------Parcours de la liste de particules-----------------------//

        #pragma omp parallel for shared(err)
		for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
		{
			ctParticle *prcl = particleVect[i];
			err[i] = computeSimErrorCPV(prcl,dTOIseg, weights);
		}

		for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
		{
			err_curr = err[i];

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

			//std::cout << " err_curr " << err_curr << std::endl;

			err_sum += err_curr;

			//! Update prcl
			//particleVect[i] = prcl;
			//std::cout << " err_curr1 " << particleVect[i].getDist() << std::endl;



	  }
}

/*!
  Compute the likelihoods of the particles of the filter for an Oriented Chamfer based similarity measure.

\param logPolarHistIseg : shape context (log polar histogram) of the segmented input image.
\param cogIseg : center of gravity of the segmented silhouette.
\param angleIseg : orientation of the segmented silhouette.

*/
void apPFilter::likelihoodViewSC(std::vector<double> &logPolarHistIseg, vpImagePoint &cogIseg, double angleIseg, bool weights)
{
	//  	vpTRACE("Start colLikelihood");
		err_sum = 0;
		err_min = 0;
		err_max = 0;
		float oriefpart=0;

		float err_curr = 0;
		float err_curr1 = 0;
		float err_curr2 = 0;
		float err_curr3 = 0;
		float err_curr4 = 0;
		ctParticle *prcl;
		apViews gen;

		nb_prcl_out = 0; //nb de particules pas enti�rement dans l'image

		//!-------------------------Parcours de la liste de particules-----------------------//


		for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
		{
			//!Distance of the current particle
			prcl = particleVect[i];

			err_curr1 = prcl->computeSimErrorSCOpt(logPolarHistIseg, cogIseg, angleIseg, logPolarHistV,surface,orientation, weights);
			err_curr2 = prcl->computeSimErrorSCOpt(logPolarHistIseg, cogIseg, angleIseg, logPolarHistV,surface,orientation-M_PI, weights);

			if (err_curr1 > err_curr2)
			{
				err_curr = err_curr2;
				oriefpart = M_PI;
			}
			else
			{
				err_curr = err_curr1;
				oriefpart = 0;
			}

			//! On compte les particules qui ne sont pas enti�rement dans l'image (pour ensuite d�tecter si l'objet est perdu)

			if (prcl->get_nbPartOut() > 0) {//4) {// object_ref.get_nbPart()/2) {
				nb_prcl_out ++;
			}

			//!Associate error with particle (could be done in ctParticle.cpp ?)
			//!Note : Tester : set distance as a mean of err_curr and err_curr_global ?
			prcl->setDist(err_curr);
			prcl->setOri(oriefpart);
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
			//particleVect[i] = prcl;


	  }
}

void apPFilter::reWeightP(ctParticle &prclEst)
{
	ctParticle *prcl;
	float wght0, wght1, weight_cumul;
	for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
	{
		//!Distance of the current particle
		prcl = particleVect[i];
		wght0 = prcl->getWeight();
		wght1 = wght0/exp(((prclEst.get_u()-prcl->get_u())*(prclEst.get_u()-prcl->get_u())/(sigmauv*sigmauv)+ (prclEst.get_v()-prcl->get_v())*(prclEst.get_v()-prcl->get_v())/(sigmauv*sigmauv) + (prclEst.get_angle()-prcl->get_angle())*(prclEst.get_angle()-prcl->get_angle())/(sigmar*sigmar) + sqrt((prclEst.get_size()-prcl->get_size())*(prclEst.get_size()-prcl->get_size()))/(sigmaz*sigmaz))/detect.lambda);

		//std::cout << " wght 1 "  << wght1 << std::endl;
		/*if (wght1 > best_weight)
			{best_weight = wght1;// Memorise best weight
			}*/
		weight_cumul += wght1;
		prcl->setWeight(wght1);
		//std::cout << " weights " << wght << std::endl;
		prcl->setWghtCumul(weight_cumul);

	}
}

/*void apPFilter::init(const unsigned int pNumber, const ctRect & location, const MotionModel &Model, std::vector< std::vector<vpImage<unsigned char>*>*> Views, vpMatrix &hierarchy, vpMatrix &dataTemp0, vpMatrix &dataTemp1)
{
	init(pNumber, location, Model);
	apParticle prcl;
	for (unsigned int i = 0; i < pNumber ; i++)
	{
		prcl.initViews(Views, hierarchy,dataTemp0,dataTemp1);
	}
}

void apPFilter::likelihoodViewInit(vpImage<vpRGBa> *dTOI, bool weights)
{
	//  	vpTRACE("Start colLikelihood");
		err_sum = 0;
		err_min = 0;
		err_max = 0;
		float oriefpart=0;

		float err_curr = 0;
		float err_curr1 = 0;
		float err_curr2 = 0;
		float err_curr3 = 0;
		float err_curr4 = 0;
		ctParticle prcl;
		std::cout << "orientation " << orientation << " surface " << surface << std::endl;

		nb_prcl_out = 0; //nb de particules pas enti�rement dans l'image

		//!-------------------------Parcours de la liste de particules-----------------------//


		for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
		{
			//!Distance of the current particle
			prcl = particleVect[i];
			//err_curr1 = prcl.computeSimErrorC(dTOI,ModelView,cog,surface,orientation, weights);//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ! -> distance 1 lorsque la particule sort)
			//err_curr2 = prcl.computeSimErrorC(dTOI,ModelView,cog,surface,orientation-M_PI, weights);
			//err_curr = prcl.computeSimError(dTOI,ModelView,surface,orientation, weights);//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ! -> distance 1 lorsque la particule sort)
			//err_curr1 = prcl.computeSimError(dTOI,ModelView,cog,surface,orientation, weights);
			err_curr1 = prcl.computeSimError(dTOI,ModelViewSc,cog,surface,orientation, weights);
			//err_curr = err_curr1;
			//oriefpart = 0;
			//err_curr2 = prcl.computeSimError(dTOI,ModelView,cog,surface,orientation-M_PI, weights);
			err_curr2 = prcl.computeSimError(dTOI,ModelViewSc,cog,surface,orientation-M_PI, weights);
			//if (err_curr > err_curr2)
			//			{
			//				err_curr = err_curr2;
			//				oriefpart = M_PI;
			//			}
			//err_curr3 = prcl.computeSimError(dTOI,ModelView,surface,orientation-M_PI/2, weights);
			//if (err_curr > err_curr3)
			//			{
			//				err_curr = err_curr3;
			//				oriefpart = M_PI/2;
			//			}
			//err_curr4 = prcl.computeSimError(dTOI,ModelView,surface,orientation+M_PI/2, weights);
			//if (err_curr > err_curr4)
			//			{
			//				err_curr = err_curr4;
			//				oriefpart = -M_PI/2;
			//			}
			if (err_curr1 > err_curr2)
			{
				err_curr = err_curr2;
				oriefpart = M_PI;
			}
			else
			{
				err_curr = err_curr1;
				oriefpart = 0;
			}

			//! On compte les particules qui ne sont pas enti�rement dans l'image (pour ensuite d�tecter si l'objet est perdu)

			if (prcl.get_nbPartOut() > 0) {//4) {// object_ref.get_nbPart()/2) {
				nb_prcl_out ++;
			}

			//!Associate error with particle (could be done in ctParticle.cpp ?)
			//!Note : Tester : set distance as a mean of err_curr and err_curr_global ?
			prcl.setDist(err_curr);
			prcl.setOri(oriefpart);
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

			//std::cout << " err_curr " << err_curr << std::endl;

			err_sum += err_curr;

			//! Update prcl
			particleVect[i] = prcl;


	  }
}

void apPFilter::likelihoodViewInitCP(vpImage<vpRGBa> *dTOI, bool weights)
{
	//  	vpTRACE("Start colLikelihood");
		err_sum = 0;
		err_min = 0;
		err_max = 0;
		float oriefpart=0;

		float err_curr = 0;
		float err_curr1 = 0;
		float err_curr2 = 0;
		float err_curr3 = 0;
		float err_curr4 = 0;
		ctParticle prcl;
		//std::cout << "orientation " << orientation << " surface " << surface << std::endl;

		nb_prcl_out = 0; //nb de particules pas enti�rement dans l'image

		//!-------------------------Parcours de la liste de particules-----------------------//


		for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
		{
			//!Distance of the current particle
			prcl = particleVect[i];
			//err_curr1 = prcl.computeSimErrorC(dTOI,ModelView,cog,surface,orientation, weights);//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ! -> distance 1 lorsque la particule sort)
			//err_curr2 = prcl.computeSimErrorC(dTOI,ModelView,cog,surface,orientation-M_PI, weights);
			//err_curr = prcl.computeSimError(dTOI,ModelView,surface,orientation, weights);//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ! -> distance 1 lorsque la particule sort)
			//err_curr1 = prcl.computeSimErrorCP(dTOI,ContourPoints,surface,orientation, weights);
			err_curr1 = prcl.computeSimErrorCPV(dTOI,ContourPointsSc,surface,orientation, weights);
			//err_curr = err_curr1;
			//oriefpart = 0;
			//err_curr2 = prcl.computeSimErrorCP(dTOI,ContourPoints,surface,orientation-M_PI, weights);
			err_curr2 = prcl.computeSimErrorCPV(dTOI,ContourPointsSc,surface,orientation-M_PI, weights);


			if (err_curr1 > err_curr2)
			{
				err_curr = err_curr2;
				oriefpart = M_PI;
			}
			else
			{
				err_curr = err_curr1;
				oriefpart = 0;
			}

			//! On compte les particules qui ne sont pas enti�rement dans l'image (pour ensuite d�tecter si l'objet est perdu)

			if (prcl.get_nbPartOut() > 0) {//4) {// object_ref.get_nbPart()/2) {
				nb_prcl_out ++;
			}

			//!Associate error with particle (could be done in ctParticle.cpp ?)
			//!Note : Tester : set distance as a mean of err_curr and err_curr_global ?
			prcl.setDist(err_curr);
			prcl.setOri(oriefpart);
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

			//std::cout << " err_curr " << err_curr << std::endl;

			err_sum += err_curr;

			//! Update prcl
			particleVect[i] = prcl;


	  }
}

void apPFilter::likelihoodViewInitSC(vpImage<unsigned char> &Iseg, bool weights)
{
	//  	vpTRACE("Start colLikelihood");
		err_sum = 0;
		err_min = 0;
		err_max = 0;
		float oriefpart=0;

		float err_curr = 0;
		float err_curr1 = 0;
		float err_curr2 = 0;
		float err_curr3 = 0;
		float err_curr4 = 0;
		ctParticle prcl;
		apViews gen;
		int nr=30;
		int nw=50;
		std::vector<double> logPolarHistIseg;
		vpImagePoint cogIseg;
		double angleIseg;
		int surfaceIseg;
		vpImage<unsigned char> IsegEdge;
		logPolarHistIseg = gen.computeSC(Iseg, IsegEdge, cogIseg,angleIseg,surfaceIseg,nr,nw);

		//std::cout << "orientation " << orientation << " surface " << surface << std::endl;

		nb_prcl_out = 0; //nb de particules pas enti�rement dans l'image

		//!-------------------------Parcours de la liste de particules-----------------------//


		for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
		{
			//!Distance of the current particle
			prcl = particleVect[i];


			err_curr1 = prcl.computeSimErrorSC(logPolarHistIseg, cogIseg,ContourPointsSc,surface,orientation, weights);
			//err_curr1 = prcl.computeSimErrorSC(logPolarHistIseg, cog,ModelViewSc,surface,orientation, weights);

			//std::cout<< "errcurr " << err_curr1 << std::endl;

			err_curr2 = prcl.computeSimErrorSC(logPolarHistIseg, cogIseg,ContourPointsSc,surface,orientation-M_PI, weights);
			//err_curr2 = prcl.computeSimErrorSC(logPolarHistIseg, cog,ModelViewSc,surface,orientation-M_PI, weights);


			if (err_curr1 > err_curr2)
			{
				err_curr = err_curr2;
				oriefpart = M_PI;
			}
			else
			{
				err_curr = err_curr1;
				oriefpart = 0;
			}

			//! On compte les particules qui ne sont pas enti�rement dans l'image (pour ensuite d�tecter si l'objet est perdu)

			if (prcl.get_nbPartOut() > 0) {//4) {// object_ref.get_nbPart()/2) {
				nb_prcl_out ++;
			}

			//!Associate error with particle (could be done in ctParticle.cpp ?)
			//!Note : Tester : set distance as a mean of err_curr and err_curr_global ?
			prcl.setDist(err_curr);
			prcl.setOri(oriefpart);
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

			//std::cout << " err_curr " << err_curr << std::endl;

			err_sum += err_curr;

			//! Update prcl
			particleVect[i] = prcl;


	  }
}

void apPFilter::likelihoodViewInitSC(std::vector<double> &logPolarHistIseg, vpImagePoint &cogIseg, bool weights)
{
	//  	vpTRACE("Start colLikelihood");
		err_sum = 0;
		err_min = 0;
		err_max = 0;
		float oriefpart=0;

		float err_curr = 0;
		float err_curr1 = 0;
		float err_curr2 = 0;
		float err_curr3 = 0;
		float err_curr4 = 0;
		ctParticle prcl;
		apViews gen;
		//int nr=30;
		//int nw=50;
		//std::vector<double> logPolarHistIseg;
		//vpImagePoint cogIseg;
		//double angleIseg;
		//int surfaceIseg;
		//logPolarHistIseg = gen.computeSC(Iseg,cogIseg,angleIseg,surfaceIseg,nr,nw);

		//std::cout << "orientation " << orientation << " surface " << surface << std::endl;

		nb_prcl_out = 0; //nb de particules pas enti�rement dans l'image

		//!-------------------------Parcours de la liste de particules-----------------------//


		for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
		{
			//!Distance of the current particle
			prcl = particleVect[i];


			err_curr1 = prcl.computeSimErrorSC(logPolarHistIseg, cogIseg,ContourPointsSc,surface,orientation, weights);
			//oriefpart = 0;
			//err_curr1 = prcl.computeSimErrorSC(logPolarHistIseg, cog,ModelViewSc,surface,orientation, weights);

			//std::cout<< "errcurr " << err_curr1 << std::endl;

			err_curr2 = prcl.computeSimErrorSC(logPolarHistIseg, cogIseg,ContourPointsSc,surface,orientation-M_PI, weights);


			if (err_curr1 > err_curr2)
			{
				err_curr = err_curr2;
				oriefpart = M_PI;
			}
			else
			{
				err_curr = err_curr1;
				oriefpart = 0;
			}

			//! On compte les particules qui ne sont pas enti�rement dans l'image (pour ensuite d�tecter si l'objet est perdu)

			if (prcl.get_nbPartOut() > 0) {//4) {// object_ref.get_nbPart()/2) {
				nb_prcl_out ++;
			}

			//!Associate error with particle (could be done in ctParticle.cpp ?)
			//!Note : Tester : set distance as a mean of err_curr and err_curr_global ?
			prcl.setDist(err_curr);
			prcl.setOri(oriefpart);
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

			//std::cout << " err_curr " << err_curr << std::endl;

			err_sum += err_curr;

			//! Update prcl
			particleVect[i] = prcl;


	  }
}


void apPFilter::likelihoodViewSegInit(vpImage<vpRGBa> *dTOI, vpImage<vpRGBa> *dTOIseg, bool weights)
{
	//  	vpTRACE("Start colLikelihood");
		err_sum = 0;
		err_min = 0;
		err_max = 0;
		float oriefpart=0;

		float err_curr = 0;
		float err_curr1 = 0;
		float err_curr2 = 0;
		float err_curr3 = 0;
		float err_curr4 = 0;
		ctParticle prcl;
		std::cout << "orientation " << orientation << " surface " << surface << std::endl;

		nb_prcl_out = 0; //nb de particules pas enti�rement dans l'image

		//!-------------------------Parcours de la liste de particules-----------------------//


		for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
		{
			//!Distance of the current particle
			prcl = particleVect[i];
			//err_curr1 = prcl.computeSimErrorC(dTOI,ModelView,cog,surface,orientation, weights);//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ! -> distance 1 lorsque la particule sort)
			//err_curr2 = prcl.computeSimErrorC(dTOI,ModelView,cog,surface,orientation-M_PI, weights);
			//err_curr = prcl.computeSimError(dTOI,ModelView,surface,orientation, weights);//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ! -> distance 1 lorsque la particule sort)
			err_curr1 = prcl.computeSimErrorSeg(dTOI,dTOIseg,ModelView,surface,orientation, weights);
			err_curr = err_curr1;
			err_curr2 = prcl.computeSimErrorSeg(dTOI,dTOIseg,ModelView,surface,orientation-M_PI, weights);
			if (err_curr > err_curr2)
						{
							err_curr = err_curr2;
							oriefpart = M_PI;
						}
			err_curr3 = prcl.computeSimErrorSeg(dTOI,dTOIseg,ModelView,surface,orientation-M_PI/2, weights);
			if (err_curr > err_curr3)
						{
							err_curr = err_curr3;
							oriefpart = M_PI/2;
						}
			err_curr4 = prcl.computeSimErrorSeg(dTOI,dTOIseg,ModelView,surface,orientation+M_PI/2, weights);
			if (err_curr > err_curr4)
						{
							err_curr = err_curr4;
							oriefpart = -M_PI/2;
						}

			//if (err_curr1 > err_curr2)
			//{
			//	err_curr = err_curr2;
			//	oriefpart = M_PI;
			//}
			//else
			//{
			//	err_curr = err_curr1;
			//	oriefpart = 0;
			//}

			//! On compte les particules qui ne sont pas enti�rement dans l'image (pour ensuite d�tecter si l'objet est perdu)

			if (prcl.get_nbPartOut() > 0) {//4) {// object_ref.get_nbPart()/2) {
				nb_prcl_out ++;
			}

			//!Associate error with particle (could be done in ctParticle.cpp ?)
			//!Note : Tester : set distance as a mean of err_curr and err_curr_global ?
			prcl.setDist(err_curr);
			prcl.setOri(oriefpart);
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

			//std::cout << " err_curr " << err_curr << std::endl;

			err_sum += err_curr;

			//! Update prcl
			particleVect[i] = prcl;


	  }
}

void apPFilter::likelihoodView(vpImage<vpRGBa> *dTOI, bool weights)
{
	//  	vpTRACE("Start colLikelihood");
		err_sum = 0;
		err_min = 0;
		err_max = 0;
		float oriefpart=0;

		float err_curr = 0;
		float err_curr1 = 0;
		float err_curr2 = 0;
		ctParticle prcl;
		std::cout << "orientation " << orientation << std::endl;

		nb_prcl_out = 0; //nb de particules pas enti�rement dans l'image

		//!-------------------------Parcours de la liste de particules-----------------------//


		for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
		{
			//!Distance of the current particle
			prcl = particleVect[i];
			//err_curr1 = prcl.computeSimErrorC(dTOI,ModelView,cog,surface,orientation, weights);//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ! -> distance 1 lorsque la particule sort)
			//err_curr2 = prcl.computeSimErrorC(dTOI,ModelView,cog,surface,orientation-M_PI, weights);
			//err_curr = prcl.computeSimError(dTOI,ModelView,surface,orientation, weights);//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ! -> distance 1 lorsque la particule sort)
			err_curr = prcl.computeSimError(dTOI,ModelView,cog,surface,orientation-orieff, weights);

			//! On compte les particules qui ne sont pas enti�rement dans l'image (pour ensuite d�tecter si l'objet est perdu)

			if (prcl.get_nbPartOut() > 0) {//4) {// object_ref.get_nbPart()/2) {
				nb_prcl_out ++;
			}

			//!Associate error with particle (could be done in ctParticle.cpp ?)
			//!Note : Tester : set distance as a mean of err_curr and err_curr_global ?
			prcl.setDist(err_curr);
			prcl.setOri(oriefpart);


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
}*/

void apPFilter::likelihoodStruct(vpImage<vpRGBa> *dTOI, bool weights, int indFilt, std::string hpath)
{
	//  	vpTRACE("Start colLikelihood");
		/*err_sum = 0;
		err_min = 0;
		err_max = 0 ;*/

		float sim1 = 0;
		float sim2 = 0;
		float sim = 0;

		float simmax = 1000;
		ctParticle *prcl;
		ctParticle prclOpt;

		vpImage<unsigned char> ViewS;
		int surfaceS;
		double oriS;
		std::cout << "orientation " << orientation << std::endl;
        vpImagePoint cogS;
		int i=hrchy.getRows()-1;
		int level;
			level = hrchy[i][0];
			//level--;
			//i=i-hrchy[i][1];
			int j=1;
			vpImagePoint cogT;
			int nbt = hrchy[i][1];
			int ind0=indFilt;
			int ind=0;

		while(level>0){
        simmax = 1000;
		int arb =0;
		level--;
		j=hrchy[i][1];
		std::cout <<"level "<<level<< " okd "<< ind0 << " " << i << " " << j << std::endl;
		if(hrchy[i-j+ind0][3]>0)
		{
		while (hrchy[i-j+ind0][3+arb]>0)
	 {

			if(level>0){
			oriS = data1[i-j-hrchy[i-j][1]+(int)hrchy[i-j+ind0][3+arb]][2];
			surfaceS = data1[i-j-hrchy[i-j][1]+(int)hrchy[i-j+ind0][3+arb]][3];
			cogT.set_i(data1[i-j-hrchy[i-j][1]+(int)hrchy[i-j+ind0][3+arb]][0]);
			cogT.set_j(data1[i-j-hrchy[i-j][1]+(int)hrchy[i-j+ind0][3+arb]][1]);
			}
			else
			{
				std::cout <<"data "<< (int)hrchy[i-j+ind0][3+arb]-1 << std::endl;
			oriS = data0[(int)hrchy[i-j+ind0][3+arb]-1][2];
			surfaceS = data0[(int)hrchy[i-j+ind0][3+arb]-1][3];
			cogT.set_i(data0[(int)hrchy[i-j+ind0][3+arb]-1][0]);
			cogT.set_j(data0[(int)hrchy[i-j+ind0][3+arb]-1][1]);
			}

			char buf[FILENAME_MAX];
			sprintf(buf, hpath.c_str(),level*1000+(int)hrchy[i-j+ind0][3+arb]-1);
			std::string filename(buf);
			vpImageIo::readPNG(ViewS, filename);

			double t0= vpTime::measureTimeMs();
			nb_prcl_out = 0; //nb de particules pas enti�rement dans l'image

			//!-------------------------Parcours de la liste de particules-----------------------//

			for (int ii = 0 ; ii < particleVect.size(); ii++)// Parcours de la liste de particules
			{
				//!Distance of the current particle
				prcl = particleVect[ii];
			sim = prcl->computeSimError(dTOI,ViewS,cogT,surfaceS,oriS, weights);//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ! -> distance 1 lorsque la particule sort)
			//sim2 = prcl.computeSimError(dTOI,ViewS,surfaceS,oriS+M_PI, weights);
			/*if(sim1>sim2)
			{
				sim=sim2;
			}
			else
			{
				sim=sim1;
			}*/
			if(sim<simmax)
			{
				simmax=sim;
				ind=(int)hrchy[i-j+ind0][3+arb];
				prclOpt = *prcl;
			}
			}
			std::cout <<"data1 "<< oriS << " surfaceS " << surfaceS << " simmaxl " << simmax << std::endl;

	arb++;
	 }
		ind0=ind;

		}
	i-=j;
	std::cout << " okd0 " << level << " okd1 " << ind << " simmax "<< simmax <<" prclOpt "<< prclOpt.get_u() << " "<< prclOpt.get_v() << " "<< prclOpt.get_angle() << std::endl;
	}

}

void apPFilter::likelihoodStruct2(vpImage<vpRGBa> *dTOI, bool weights, int indFilt, std::string hpath)
{
	//  	vpTRACE("Start colLikelihood");
		err_sum = 0;
		err_min = 0;
		err_max = 0;

		float err_curr = 0;

		ctParticle *prcl;
		ctParticle prclOpt;
		//!-------------------------Parcours de la liste de particules-----------------------//

		for (int ii = 0 ; ii < particleVect.size(); ii++)// Parcours de la liste de particules
		{
			prcl = particleVect[ii];

			float sim1 = 0;
			float sim2 = 0;
			float sim = 0;

			float simmax = 1000;

		vpImage<unsigned char> ViewS;
		int surfaceS;
		double oriS;
		//std::cout << "orientation " << orientation << std::endl;
        vpImagePoint cogS;
		int i=hrchy.getRows()-1;
		int level;
			level = hrchy[i][0];
			//level--;
			//i=i-hrchy[i][1];
			int j=1;
			vpImagePoint cogT;
			int nbt = hrchy[i][1];
			int ind0=indFilt;
			int ind=0;

		while(level>0){
        simmax = 1000;
		int arb =0;
		level--;
		j=hrchy[i][1];
		//std::cout <<"level "<<level<< " okd "<< ind0 << " " << i << " " << j << std::endl;
		if(hrchy[i-j+ind0][3]>0)
		{
		while (hrchy[i-j+ind0][3+arb]>0)
	 {
			double t0= vpTime::measureTimeMs();

			if(level>0){
			oriS = data1[i-j-hrchy[i-j][1]+(int)hrchy[i-j+ind0][3+arb]][2];
			surfaceS = data1[i-j-hrchy[i-j][1]+(int)hrchy[i-j+ind0][3+arb]][3];
			cogT.set_i(data1[i-j-hrchy[i-j][1]+(int)hrchy[i-j+ind0][3+arb]][0]);
			cogT.set_j(data1[i-j-hrchy[i-j][1]+(int)hrchy[i-j+ind0][3+arb]][1]);
			}
			else
			{
				//std::cout <<"data "<< (int)hrchy[i-j+ind0][3+arb]-1 << std::endl;
			oriS = data0[(int)hrchy[i-j+ind0][3+arb]-1][2];
			surfaceS = data0[(int)hrchy[i-j+ind0][3+arb]-1][3];
			cogT.set_i(data0[(int)hrchy[i-j+ind0][3+arb]-1][0]);
			cogT.set_j(data0[(int)hrchy[i-j+ind0][3+arb]-1][1]);
			}

			char buf[FILENAME_MAX];
			sprintf(buf, hpath.c_str(),level*1000+(int)hrchy[i-j+ind0][3+arb]-1);
			std::string filename(buf);
			vpImageIo::readPNG(ViewS, filename);
			double t1= vpTime::measureTimeMs();
			std::cout<< " t0 " << t1-t0 << std::endl;

			nb_prcl_out = 0; //nb de particules pas enti�rement dans l'image

				//!Distance of the current particle
			sim = prcl->computeSimError(dTOI,ViewS,cogT,surfaceS,oriS, weights);//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ! -> distance 1 lorsque la particule sort)
			//sim2 = prcl.computeSimError(dTOI,ViewS,surfaceS,oriS+M_PI, weights);
			/*if(sim1>sim2)
			{
				sim=sim2;
			}
			else
			{
				sim=sim1;
			}*/
			if(sim<simmax)
			{
				simmax=sim;
				ind=(int)hrchy[i-j+ind0][3+arb];
			}
			//std::cout <<"data1 "<< oriS << " surfaceS " << surfaceS << " simmaxl " << simmax << std::endl;

	arb++;
	 }
		ind0=ind;

		}
	i-=j;
	//std::cout << " okd0 " << level << " okd1 " << ind << " simmax "<< simmax <<" prclOpt "<< prclOpt.get_u() << " "<< prclOpt.get_v() << " "<< prclOpt.get_angle() << std::endl;
	}

		std::cout << " okd0 " << level << " okd1 " << ind << " simmax "<< simmax  << std::endl;
		err_curr = simmax;//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ! -> distance 1 lorsque la particule sort)

		//! On compte les particules qui ne sont pas enti�rement dans l'image (pour ensuite d�tecter si l'objet est perdu)

		if (prcl->get_nbPartOut() > 0) {//4) {// object_ref.get_nbPart()/2) {
			nb_prcl_out ++;
		}

		//!Associate error with particle (could be done in ctParticle.cpp ?)
		//!Note : Tester : set distance as a mean of err_curr and err_curr_global ?
		prcl->setDist(err_curr);


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
		//particleVect[i] = prcl;

			}

}

void apPFilter::likelihoodStruct3(vpImage<vpRGBa> *dTOI, vpImage<vpRGBa> *dTOIseg, bool weights, int indFilt, int startingLevel)
{
		err_sum = 0;
		err_min = 0;
		err_max = 0;

		float err_curr = 0;
		ctParticle *prcl;
		ctParticle *prclOpt;
		vpImage<unsigned char> *viewS;
		int surfaceS;
		double oriS;
		int level,i,j,ind0,ind,arb;
		float sim,simmax,sim2;
		//std::cout << "orientation " << orientation << std::endl;
        vpImagePoint cogS;


		nb_prcl_out = 0; //nb de particules pas enti�rement dans l'image
		//!-------------------------Parcours de la liste de particules-----------------------//

		for (int ii = 0 ; ii < particleVect.size(); ii++)// Parcours de la liste de particules
		//for (int ii = 0 ; ii < 1; ii++)// Parcours de la liste de particules
		{
			prcl = particleVect[ii];

			//*prcl = pEstimate;

			/*float sim1 = 0;
			float sim2 = 0;*/
			sim = 0;
			sim2 = 0;

			simmax = 10000;

			i=hrchy.getRows()-1;
			int numberofLevels = hrchy[i][0];
			level = numberofLevels;
			int diff = numberofLevels-startingLevel;
			//std::cout << " diff " << diff << " level " << level << std::endl;
			while(diff>1){i=i-hrchy[i][1];
			level--;
			diff--;}

		//vpImage<unsigned char> ViewS;
		   /* i=hrchy.getRows()-1;
			level = hrchy[i][0];
			level--;
			i=i-hrchy[i][1];*/
			j=1;
			vpImagePoint cogT;
			ind0=indFilt;
			ind=0;

			//std::cout << " IndFilt " << indFilt << " i " << i << " diff " << diff <<  std::endl;
		while(level>0){
        simmax = 10000;
		arb =0;
		level--;
		j=hrchy[i][1];
		//std::cout <<"level "<<level<< " okd "<< ind0 << " " << i << " " << j << " oripart " << prcl.getOri() << std::endl;
		//std::cout <<"level "<<level<< std::endl;
		if(hrchy[i-j+ind0][3]>0)
		{
		while (hrchy[i-j+ind0][3+arb]>0)
	 {
			double t0= vpTime::measureTimeMs();
            //std::cout << " Level " << level << std::endl;
			if(level>0){
			//std::cout << " data " << i-j-hrchy[i-j][1]+(int)hrchy[i-j+ind0][3+arb] << std::endl;
			oriS = data1[i-j-hrchy[i-j][1]+(int)hrchy[i-j+ind0][3+arb]][2];
			surfaceS = data1[i-j-hrchy[i-j][1]+(int)hrchy[i-j+ind0][3+arb]][3];
			cogT.set_i(data1[i-j-hrchy[i-j][1]+(int)hrchy[i-j+ind0][3+arb]][0]);
			cogT.set_j(data1[i-j-hrchy[i-j][1]+(int)hrchy[i-j+ind0][3+arb]][1]);
			}
			else
			{
			//std::cout << "hrchy " << hrchy[i-j+ind0][3+arb] << std::endl;
				//std::cout <<"data "<< (int)hrchy[i-j+ind0][3+arb]-1 << std::endl;
			oriS = data0[(int)hrchy[i-j+ind0][3+arb]-1][2];
			surfaceS = data0[(int)hrchy[i-j+ind0][3+arb]-1][3];
			cogT.set_i(data0[(int)hrchy[i-j+ind0][3+arb]-1][0]);
			cogT.set_j(data0[(int)hrchy[i-j+ind0][3+arb]-1][1]);
			}
			//std::cout << "hrchy " << hrchy[i-j+ind0][3+arb] << std::endl;
			viewS = (Hviews[Hviews.size()-level-1]->at((int)hrchy[i-j+ind0][3+arb]));
			//vpImageIo::writePPM(*viewS,"view.png");

			double t1= vpTime::measureTimeMs();
			//std::cout<< " t0 " << t1-t0 << std::endl;
				//!Distance of the current particle
			//sim = prcl->computeSimErrorCPV(dTOI,ContourPointsSc,surface,orientation-orieff, weights);

			sim = prcl->computeSimError(dTOI,*viewS,cogT,surfaceS,oriS-orieff, weights);//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ! -> distance 1 lorsque la particule sort)
			sim2 = prcl->computeSimError(dTOIseg,*viewS,cogT,surfaceS,oriS-orieff, weights);
			sim = 0.5*(sim+sim2);
			/*sim2 = prcl.computeSimError(dTOI,ViewS,surfaceS,oriS-M_PI, weights);
			if(sim1>sim2)
			{
				sim=sim2;
				oriefpart = M_PI;
			}
			else
			{
				sim=sim1;
				oriefpart = 0;
			}*/
			if(sim<simmax)
			{
				simmax=sim;
				ind=(int)hrchy[i-j+ind0][3+arb];
				prclOpt = prcl;
			}
			//std::cout <<"data1 "<< oriS << " surfaceS " << surfaceS << " simmaxl " << simmax << std::endl;
			//std::cout <<"ind "<< ind << " simmax " << simmax << std::endl;

	arb++;
	 }
		ind0=ind;

		}
	i-=j;
	//std::cout << " okd0 " << level << " okd1 " << ind << " simmax "<< simmax <<" prclOpt "<< prclOpt.get_u() << " "<< prclOpt.get_v() << " "<< prclOpt.get_angle() << std::endl;
	}

		err_curr = simmax;//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ! -> distance 1 lorsque la particule sort)
		//std::cout << " okd0 " << level << " okd1 " << ind << " simmax "<< err_curr  << std::endl;

		//! On compte les particules qui ne sont pas enti�rement dans l'image (pour ensuite d�tecter si l'objet est perdu)

		if (prcl->get_nbPartOut() > 0) {//4) {// object_ref.get_nbPart()/2) {
			nb_prcl_out ++;
		}

		//!Associate error with particle (could be done in ctParticle.cpp ?)
		//!Note : Tester : set distance as a mean of err_curr and err_curr_global ?
		prcl->setDist(err_curr);
		prcl->set_ind(ind);
		//prcl.setOri(oriefpart);


		//! err_min, err_max
		if (ii == 0) {
			err_min = err_curr;
			err_max = err_curr;
			ind_min = ind;
			prclOpt_ = *prclOpt;
		}
		else {
			if (err_curr < err_min) {
				err_min = err_curr;
				ind_min = ind;
				prclOpt_ = *prclOpt;

			}
			if (err_curr > err_max) {
				err_max = err_curr;
			}
		}

		err_sum += err_curr;

		//! Update prcl
		//particleVect[ii] = prcl;
			}
		std::cout << "ind min " << ind_min << " err_min " << err_min << std::endl;

}

void apPFilter::likelihoodStruct4(vpImage<vpRGBa> *dTOI, bool weights, int indFilt, int startingLevel)
{
	//  	vpTRACE("Start colLikelihood");
		err_sum = 0;
		err_min = 0;
		err_max = 0;

		float err_curr = 0;

		ctParticle *prcl;
		ctParticle prclOpt;
		vpImage<unsigned char> *viewS;
		int surfaceS;
		double oriS;
		int level,i,j,ind0,ind,arb;
		float sim,simmax;
		//std::cout << "orientation " << orientation << std::endl;
        vpImagePoint cogS;

		nb_prcl_out = 0; //nb de particules pas enti�rement dans l'image

			*prcl = pEstimate;

			std::cout << " angpEst " <<prcl->get_angle() << " upEst " << prcl->get_u()<< " vpEst " << prcl->get_v() << std::endl;

			float sim1 = 0;
			float sim2 = 0;
			sim = 0;

			simmax = 1000;

			i=hrchy.getRows()-1;
			int numberofLevels = hrchy[i][0]+1;
			level = numberofLevels - 1;
			int diff = numberofLevels-startingLevel;

			while(diff>1){i=i-hrchy[i][1];
			level--;
			diff--;}

		/*i=hrchy.getRows()-1;
			level = hrchy[i][0];
			level--;
			i=i-hrchy[i][1];*/
			j=1;
			vpImagePoint cogT;
			ind0=indFilt;
			ind=0;


			//----------------------------------------------------------

			/*apViews gen;
            vpImage<unsigned char> IT;
            double Ang,ang,angle,AngOpt;
        	int nc = (int)60;
        	int nr = (int)60;
        	int intc = (int)nc/5;
        	int intr = (int)nr/5;
        	angle = prcl.get_angle();
        	vpImagePoint pI,pI0,Popt;
        	pI.set_i((int)prcl.get_v()+50);
        	pI.set_j((int)prcl.get_u()+100);
            std::string opath = "/local/agpetit/images/imagesSpot/hierarchy/I%04d.pgm";
            for (int nim=0 ; nim < 14 ; nim++)
            {

			char buf[FILENAME_MAX];
			sprintf(buf, opath.c_str(),2000+nim);
			std::string filename(buf);
			vpImageIo::readPPM(IT, filename);
			oriS = data0[nim][2];
			surfaceS = data0[nim][3];
			cogT.set_i(data0[nim][0]);
			cogT.set_j(data0[nim][1]);
			Ang = -angle+oriS;
			Ang = 0;
			//vpImageIo::readPPM(*argImDT, filename);
			double t0= vpTime::measureTimeMs();

			for (int cc=0; cc<1;cc++)
			{
				for (int rr=0; rr<1;rr++)
				{
				pI0.set_i(pI.get_i()-nr/2 + rr*intr);
				pI0.set_j(pI.get_j()-nc/2 + cc*intc);
				pI0.set_i(256);
				pI0.set_j(256);
				for(int rot=0;rot<100;rot++)
				{
					//ang=Ang-(rotm-rot*(rotm/25));
					//ang=rot*(rotm/10)-rotm;
					double t0= vpTime::measureTimeMs();
					ang = Ang - (M_PI/2-rot*((M_PI/2)/49));
					//cout << " rot " << ang << endl;
					//rotateTemp(IT,Irot,ang);
				    //pointsPos.push_back(pI);
					//sim = computeSimilarityPos(argImDT,&Irot,pI0);
				sim = gen.computeSimilarityPosRot(dTOI,&IT,pI0,ang);
				std::cout << "sim " << sim << " Ang " << ang << " simmax "<< simmax << " nim " << nim << "ind " << ind << " Popt " << Popt << " AngOpt "<< AngOpt << " oriT " << oriS<< std::endl;
		if(sim<simmax)
		{
			simmax=sim;
			ind=nim;
			Popt = pI0;
			AngOpt = ang;
		}

				}
				}
			}
            }
			std::cout << " simmax "<< simmax << "ind " << ind << " Popt " << Popt << " AngOpt "<< AngOpt << " oriT " << oriS<< std::endl;

getchar();*/
				//----------------------------------------------------------
			int level0 = level-1;

		while(level>0){
        simmax = 1000;
		arb =0;
		level--;
		j=hrchy[i][1];
		//std::cout <<"level "<<level<< " okd "<< ind0 << " " << i << " " << j << std::endl;
		if(hrchy[i-j+ind0][3]>0)
		{
		while (hrchy[i-j+ind0][3+arb]>0)
	 {
			double t0= vpTime::measureTimeMs();

			if(level>0){
			oriS = data1[i-j-hrchy[i-j][1]+(int)hrchy[i-j+ind0][3+arb]][2];
			surfaceS = data1[i-j-hrchy[i-j][1]+(int)hrchy[i-j+ind0][3+arb]][3];
			cogT.set_i(data1[i-j-hrchy[i-j][1]+(int)hrchy[i-j+ind0][3+arb]][0]);
			cogT.set_j(data1[i-j-hrchy[i-j][1]+(int)hrchy[i-j+ind0][3+arb]][1]);
			}
			else
			{
				std::cout <<"data "<< (int)hrchy[i-j+ind0][3+arb]-1 << std::endl;
			oriS = data0[(int)hrchy[i-j+ind0][3+arb]-1][2];
			surfaceS = data0[(int)hrchy[i-j+ind0][3+arb]-1][3];
			cogT.set_i(data0[(int)hrchy[i-j+ind0][3+arb]-1][0]);
			cogT.set_j(data0[(int)hrchy[i-j+ind0][3+arb]-1][1]);
			}
			viewS = (Hviews[Hviews.size()-level-1]->at((int)hrchy[i-j+ind0][3+arb]));
			double t1= vpTime::measureTimeMs();
			//std::cout<< " t0 " << t1-t0 << std::endl;
			/*if(level == level0){
				//!Distance of the current particle
			sim1 = prcl.computeSimError(dTOI,*viewS,cogT,surfaceS,oriS, weights);
			//sim = prcl.computeSimErrorC(dTOI,*viewS,cogT,surfaceS,oriS, weights);//calcul de l'erreur de cette particule (tient compte de parties hors de l'image ! -> distance 1 lorsque la particule sort)
			sim2 = prcl.computeSimError(dTOI,*viewS,cogT,surfaceS,oriS-M_PI, weights);
			if(sim1>sim2)
			{
				sim=sim2;
				oriefpart = M_PI;
			}
			else
			{
				sim=sim1;
				oriefpart = 0;
			}
			}
			else*/
			{
				sim = prcl->computeSimError(dTOI,*viewS,cogT,surfaceS,oriS, weights);
			}
			if(sim<simmax)
			{
				simmax=sim;
				ind=(int)hrchy[i-j+ind0][3+arb];
			}
			//std::cout <<"data1 "<< oriS << " surfaceS " << surfaceS << " simmaxl " << simmax << std::endl;

	arb++;
	 }
		ind0=ind;

		}
	i-=j;
	//std::cout << " okd0 " << level << " okd1 " << ind << " simmax "<< simmax <<" prclOpt "<< prclOpt.get_u() << " "<< prclOpt.get_v() << " "<< prclOpt.get_angle() << std::endl;
	}

		std::cout << " okd0 " << level << " okd1 " << ind << " simmax "<< simmax  << std::endl;
		ind_min = ind;


}

void apPFilter::reWeight(vpImage<unsigned char> &Iseg)
{
	float err_curr = 0;
	ctParticle *prcl;

	//nb_prcl_out = 0; //nb de particules pas enti�rement dans l'image

	//!-------------------------Parcours de la liste de particules-----------------------//


	for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
	{
		//!Distance of the current particle
		prcl = particleVect[i];
        prcl->reWeight(Iseg);

	}
}
void apPFilter::weightsUpdate2(vpImage<unsigned char> &Iseg)//, const float & min_threshold, const float & lost_prcl_ratio)
{
	ctParticle *prcl;
	float wght0 = 0; float dist_pI = 0;
	float dist_angle;
	best_weight = 0;
	float wght1=0;
    apViews gen;
    vpImagePoint pIseg;
    double angle;
    int surface;
    gen.computePosOri(Iseg,pIseg,angle,surface);

	float weight_cumul = 0;

	//! test if object is lost
	/*if ((err_min > min_threshold)||((float)nb_prcl_out/(float)this->get_pNum() > lost_prcl_ratio)) {
		std::cout << "erreur min : " << err_min << std::endl;
		std::cout << "min threshold was : " << min_threshold  << std::endl;
		std::cout << "ratio lost particles : " << (float)nb_prcl_out/(float)this->get_pNum() << std::endl;
		std::cout << "min threshold ratio was : " << lost_prcl_ratio << std::endl;
		object_lost = true;*/

		//#pragma omp parallel for

		for (int i = 0 ; i < particleVect.size(); i++)// Parcours de la liste de particules
		{
			prcl = particleVect[i];

			dist_pI = (pIseg.get_u()-prcl->u)*(pIseg.get_u()-prcl->u)+(pIseg.get_v()-prcl->v)*(pIseg.get_v()-prcl->v);
			if (angle -prcl->angle + M_PI < angle -prcl->angle)
			dist_angle = (angle - prcl->angle + M_PI)*(angle - prcl->angle + M_PI);
			else
			dist_angle = (angle - prcl->angle)*(angle - prcl->angle);



		//Calcul nouveau poids
			{
// 				wght= exp(-3*(dist_curr-err_min)/(err_max-err_min)*(dist_curr-err_min)/(err_max-err_min)); // Somme ne vaut pas 1 !!
// 				wght = exp(-0.5*dist_curr*dist_curr);// Somme ne vaut pas 1 !!
				wght1 = exp(-0.1*dist_pI)*exp(-dist_angle);// Somme ne vaut pas 1 !!
				//wght = 1 - ((dist_curr-err_min)/(err_max-err_min)*(dist_curr-err_min)/(err_max-err_min));// Somme ne vaut pas 1 !!
				//wght =  (err_sum-dist_curr)/(err_sum*(particleVect.size()-1));// mesure de vraisemblance lin�aire - Somme = 1
			}
			if (wght1*wght0 > best_weight) best_weight = wght1*wght0;// Memorise best weight
			weight_cumul += wght1*wght0;
            wght0 = prcl->getWeight();
			prcl->setWeight(wght1*wght0);
			prcl->setWghtCumul(weight_cumul);
			//particleVect[i] = prcl;
		}
		std::cout << "bestweigth2 : " << best_weight << std::endl;
		this->setWght_sum(weight_cumul);
}


void
apPFilter::resize(const vpImage<unsigned char>& _I, vpImage<unsigned char>&  Io, double scale)
{

      //unsigned int cScale = 2;
      vpImage<unsigned char>* I = new vpImage<unsigned char>((int)_I.getHeight() * scale, (int)_I.getWidth() * scale);
      IplImage* vpI0 = cvCreateImageHeader(cvSize(_I.getWidth(), _I.getHeight()), IPL_DEPTH_8U, 1);
      vpI0->imageData = (char*)(_I.bitmap);
      IplImage* vpI = cvCreateImage(cvSize((int)_I.getWidth() * scale, (int)_I.getHeight() * scale), IPL_DEPTH_8U, 1);
      cvResize(vpI0, vpI, CV_INTER_NN);
      vpImageConvert::convert(vpI, *I);
      cvReleaseImage(&vpI);
      vpI0->imageData = NULL;
      cvReleaseImageHeader(&vpI0);
      if(scale>1){
      for (int i = 0;i<Io.getHeight();i++)
      {
    	  for(int j = 0;j<Io.getWidth();j++)
    	  {
      Io[i][j] = (*I)[i+(int)(0.5*(I->getHeight()-Io.getHeight()))][j+(int)(0.5*(I->getWidth()-Io.getWidth()))];
    	  }
      }
      }
      else
      {
          for (int i = 0;i<Io.getHeight();i++)
          {
        	  for(int j = 0;j<Io.getWidth();j++)
        	  {
        		  if(i+(int)(0.5*((int)I->getHeight()-(int)Io.getHeight()))>0 && j+(int)(0.5*((int)I->getWidth()-(int)Io.getWidth()))>0 && i+(int)(0.5*((int)I->getHeight()-(int)Io.getHeight()))<(int)I->getHeight() && j+(int)(0.5*((int)I->getWidth()-(int)Io.getWidth()))<(int)I->getWidth())
          {Io[i][j] = (*I)[i+(int)(0.5*((int)I->getHeight()-(int)Io.getHeight()))][j+(int)(0.5*((int)I->getWidth()-(int)Io.getWidth()))];}
        		  else
        			  {Io[i][j] = 100;}
        	  }
          }
      }
      delete I;


}


void apPFilter::displayTemp(vpImage<unsigned char> &I)
{
	apViews gen;
	vpImage<unsigned char> MViewRot;
	vpImage<unsigned char> MViewRotSc(I.getHeight(), I.getWidth());
	ctParticle prcl = pEstimate;
	double scale;
	scale = sqrt(prcl.get_size()/surface);
	/*if (scale<0.3)
	{
	scale = 0.3;
	}
	if (scale>3)
	{
	scale = 3;
	}*/
	//scale = 1.3;
	std::cout << " ang " <<prcl.get_angle() << " u " << prcl.get_u()<< " v " << prcl.get_v() << "orientation " << orientation << " orieff " << orieff << " orieff2 " << scale << std::endl;
	gen.rotateView(ModelView,MViewRot,orientation-orieff-prcl.get_angle());
	resize(MViewRot,MViewRotSc,scale);
	//vpImageIo::writePPM(MViewRot,"ModelView.pgm");
	//vpImageIo::writePNG(ModelView,"ModelViewS.png");
	/*vpImageIo::writePPM(MViewRot, "modelV.pgm");
	getchar();*/
	vpImagePoint ip1;
	vpImagePoint ip2;
	for(int i = 0; i < I.getHeight(); i++)
	{
	for(int j =0; j < I.getWidth(); j++)
	{
		if(MViewRotSc[i][j]!=100)
		{
			/*ip1.set_u(j+prcl.get_u()-(cog.get_u()-(int)I.getWidth()/2)-(int)I.getWidth()/2);
			ip1.set_v(i+prcl.get_v()-(cog.get_v()-(int)I.getHeight()/2)-(int)I.getHeight()/2);*/
			ip1.set_u(j+prcl.get_u()-(int)I.getWidth()/2);
			ip1.set_v(i+prcl.get_v()-(int)I.getHeight()/2);
			ip2.set_u(prcl.get_u());
			ip2.set_v(prcl.get_v());
	vpDisplay::displayCross(I,ip1,2,vpColor::green,2);
	vpDisplay::displayCross(I,ip2,2,vpColor::blue,2);
		}
	}
	}
	for (int ii = 0 ; ii < particleVect.size(); ii++)// Parcours de la liste de particules
	{
		//!Distance of the current particle
		prcl = *particleVect[ii];
		ip1.set_u(prcl.get_u());
		ip1.set_v(prcl.get_v());
		//vpDisplay::displayCross(I,ip1,2,vpColor::red,2);

	}
}

void apPFilter::displayBestTemp(vpImage<unsigned char> &I)
{
	apViews gen;
	vpImage<unsigned char> BestView;
	vpImage<unsigned char> MViewRot;
	vpImage<unsigned char> MViewRotSc(I.getHeight(), I.getWidth());
	//ctParticle prcl = pEstimate;
	ctParticle prcl = prclOpt_;
	double scale;
	BestView=*(Hviews[Hviews.size()-1]->at(ind_min));
	double oriBest = data0[ind_min-1][2];
	double surfaceBest = data0[ind_min-1][3];
	vpImagePoint cogBest;
	cogBest.set_i(data0[ind_min-1][0]);
	cogBest.set_j(data0[ind_min-1][1]);
	scale = sqrt(prcl.get_size()/surfaceBest);
	std::cout << " ang " <<prcl.get_angle() << " u " << prcl.get_u()<< " v " << prcl.get_v() << " ind " << ind_min << " oriBest " << oriBest << std::endl;
	gen.rotateView(BestView,MViewRot,oriBest-prcl.get_angle()-orieff);
	resize(MViewRot,MViewRotSc,scale);
	//vpImageIo::writePNG(MViewRotSc,"ModelViewB.png");
	//vpImageIo::writePNG(MViewRot,"ModelView.png");
	/*vpImageIo::writePPM(MViewRot, "modelV.pgm");
	getchar();*/
	vpImagePoint ip1;
	vpImagePoint ip2;
	for(int i = 0; i < I.getHeight(); i++)
	{
	for(int j =0; j < I.getWidth(); j++)
	{
		if(MViewRotSc[i][j]!=100)
		{
			ip1.set_u(j+prcl.get_u()-(int)I.getWidth()/2);
			ip1.set_v(i+prcl.get_v()-(int)I.getHeight()/2);
			ip2.set_u(prcl.get_u());
			ip2.set_v(prcl.get_v());
	vpDisplay::displayCross(I,ip1,2,vpColor::green,2);
	vpDisplay::displayCross(I,ip2,2,vpColor::blue,2);
		}
	}
	}
	/*for (int ii = 0 ; ii < particleVect.size(); ii++)// Parcours de la liste de particules
	{
		//!Distance of the current particle
		prcl = *particleVect[ii];
		ip1.set_u(prcl.get_u());
		ip1.set_v(prcl.get_v());
		vpDisplay::displayCross(I,ip1,2,vpColor::red,2);
	}*/
}

vpHomogeneousMatrix apPFilter::getTemplatePose(vpImage<unsigned char> &I)
{
vpHomogeneousMatrix cMo;
ctParticle prcl = pEstimate;
//double angle = orientation-orieff-prcl.get_angle();
double angle = orientation-orieff-prcl.get_angle();
vpRotationMatrix R;
vpTranslationVector tr;
vpRotationMatrix Rz = vpRotationMatrix(vpRxyzVector(0,0,0));
cMo.buildFrom(pose);
cMo.extract(R);
cMo.extract(tr);
double xx0,yy0,xx,yy,xx1,yy1,x0,y0,x1,y1,rho,theta;
vpPixelMeterConversion::convertPoint(cam,cog,xx0,yy0);
tr[2] = tr[2]*sqrt(surface/prcl.get_size());
xx = xx0*tr[2];
yy = yy0*tr[2];
xx = pose[0];
yy = pose[1];
tr[0] = -xx+tr[2]*(prcl.get_u()-cam.get_u0())/cam.get_px();
tr[1] = -yy + tr[2]*(prcl.get_v()-cam.get_v0())/cam.get_py();
//tr[0] = tr[2]*(-(cog.get_u()-cam.get_u0())+prcl.get_u() -cam.get_u0())/cam.get_px();
//tr[1] = tr[2]*(-(cog.get_v()-cam.get_v0())+prcl.get_v() -cam.get_v0())/cam.get_py();
x0 = tr[0]/tr[2];
y0 = tr[1]/tr[2];
xx1 = (prcl.get_u()-cam.get_u0())/cam.get_px();
yy1 = (prcl.get_v()-cam.get_v0())/cam.get_py();
vpImagePoint ip1,ip2,ip3;
ip1.set_u(-(cog.get_u()-cam.get_u0())+prcl.get_u());
ip1.set_v(-(cog.get_v()-cam.get_v0())+prcl.get_v());
ip2.set_u(prcl.get_u());
ip2.set_v(prcl.get_v());
rho = sqrt((yy1-y0)*(yy1-y0)+(xx1-x0)*(xx1-x0));
theta = atan2(-(yy1-y0),(xx1-x0));
theta = theta+angle;
x1 = rho*cos(theta)+x0;
y1 = -rho*sin(theta)+y0;
ip3.set_u(x1*cam.get_px()+cam.get_u0());
ip3.set_v(y1*cam.get_py()+cam.get_v0());
tr[0] = tr[0]+(xx1-x1)*tr[2];
tr[1] = tr[1]+(yy1-y1)*tr[2];

std::cout << " xx " << tr[0] << " yy " << tr[1] << std::endl;
vpRotationMatrix Rz0 = vpRotationMatrix(vpRxyzVector(0,0,-angle));
R=Rz0*R;
cMo.buildFrom(tr,R);
return cMo;
}

vpHomogeneousMatrix apPFilter::getBestPose()
{
	vpHomogeneousMatrix cMo;
	ctParticle prcl = prclOpt_;

	poseOpt_[0]=data0[ind_min-1][4];
	poseOpt_[1]=data0[ind_min-1][5];
	poseOpt_[2]=data0[ind_min-1][6];
	poseOpt_[3]=data0[ind_min-1][7];
	poseOpt_[4]=data0[ind_min-1][8];
	poseOpt_[5]=data0[ind_min-1][9];

	double oriBest = data0[ind_min-1][2];
	double surfaceBest = data0[ind_min-1][3];
	vpImagePoint cogBest;
	cogBest.set_i(data0[ind_min-1][0]);
	cogBest.set_j(data0[ind_min-1][1]);

	//double angle = orientation-orieff-prcl.get_angle();
	double angle = oriBest-prcl.get_angle()-orieff;
	vpRotationMatrix R;
	vpTranslationVector tr;
	vpRotationMatrix Rz = vpRotationMatrix(vpRxyzVector(0,0,0));
	cMo.buildFrom(poseOpt_);
	cMo.extract(R);
	cMo.extract(tr);
	double xx0,yy0,xx,yy,xx1,yy1,x0,y0,x1,y1,rho,theta;
	vpPixelMeterConversion::convertPoint(cam,cogBest,xx0,yy0);
	tr[2] = tr[2]*sqrt(surfaceBest/prcl.get_size());
	xx = xx0*tr[2];
	yy = yy0*tr[2];
	xx = poseOpt_[0];
	yy = poseOpt_[1];
	tr[0] = -xx+tr[2]*(prcl.get_u()-cam.get_u0())/cam.get_px();
	tr[1] = -yy + tr[2]*(prcl.get_v()-cam.get_v0())/cam.get_py();
	//tr[0] = tr[2]*(-(cog.get_u()-cam.get_u0())+prcl.get_u() -cam.get_u0())/cam.get_px();
	//tr[1] = tr[2]*(-(cog.get_v()-cam.get_v0())+prcl.get_v() -cam.get_v0())/cam.get_py();
	x0 = tr[0]/tr[2];
	y0 = tr[1]/tr[2];
	xx1 = (prcl.get_u()-cam.get_u0())/cam.get_px();
	yy1 = (prcl.get_v()-cam.get_v0())/cam.get_py();
	vpImagePoint ip1,ip2,ip3;
	ip1.set_u(-(cogBest.get_u()-cam.get_u0())+prcl.get_u());
	ip1.set_v(-(cogBest.get_v()-cam.get_v0())+prcl.get_v());
	ip2.set_u(prcl.get_u());
	ip2.set_v(prcl.get_v());
	rho = sqrt((yy1-y0)*(yy1-y0)+(xx1-x0)*(xx1-x0));
	theta = atan2(-(yy1-y0),(xx1-x0));
	theta = theta+angle;
	x1 = rho*cos(theta)+x0;
	y1 = -rho*sin(theta)+y0;
	ip3.set_u(x1*cam.get_px()+cam.get_u0());
	ip3.set_v(y1*cam.get_py()+cam.get_v0());
	tr[0] = tr[0]+(xx1-x1)*tr[2];
	tr[1] = tr[1]+(yy1-y1)*tr[2];

	std::cout << " xx " << tr[0] << " yy " << tr[1] << std::endl;
	vpRotationMatrix Rz0 = vpRotationMatrix(vpRxyzVector(0,0,-angle));
	R=Rz0*R;
	cMo.buildFrom(tr,R);
	return cMo;
}

void apPFilter::setStructure(vpMatrix &hier, vpMatrix &dat0, vpMatrix &dat1, std::vector< std::vector<vpImage<unsigned char>*>*> &HV)
{
	hrchy = hier;
	data0 = dat0;
	data1 = dat1;
	Hviews = HV;
}



