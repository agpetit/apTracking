/*
 * apMHMeSite.h
 *
 *  Created on: Jul 9, 2012
 *      Author: agpetit
 */
#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>
#include <vector>
#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <visp/vpMe.h>
//#include <visp/vpMeSite.h>


#ifndef APMHMESITE_H_
#define APMHMESITE_H_

class apMHMeSite //: public vpMeSite

{
public:
	  typedef enum
	    {
	      NONE,
	      RANGE,
	      RESULT,
	      RANGE_RESULT
	    } vpMeSiteDisplayType;
	  int i,j ;
	  int i_1, j_1 ;
	  double ifloat, jfloat ;
	  unsigned char v ;
	  int mask_sign ;

	  // Angle of tangent at site
	  double alpha;

	  // Convolution of Site in previous image
	  double convlt ;
	 // Convolution of Site in previous image
	  double normGradient ;

	  //! Flag to indicate whether point is rejected or not
	  //! 1 = contrast, 2 = threshold, 3 = M-estimator, 0 = nosupp
	  int suppress;

	  // Uncertainty of point given as a probability between 0 and 1
	  double weight;

  apMHMeSite * candidateList;
  double thetaGrad;
  //std::vector<apMHMeSite>* candidateVect;

private:
  double weight_s;
  int numCandidates; //Number of candidates considered in tracking

public:
  void init() ;
  void init(double ip, double jp, double alphap) ;
  void init(double ip, double jp, double alphap, double convltp) ;
  void init(double ip, double jp, double alphap, double convltp, int sign) ;

	apMHMeSite();
	apMHMeSite(double ip, double jp) ;
	apMHMeSite(const apMHMeSite &m) ;
	virtual ~apMHMeSite();
	apMHMeSite &operator=(const apMHMeSite &m);

	  int operator!=(const apMHMeSite  &m) ;

	  // Needed in order to use it in vpList
	  friend std::ostream& operator<<(std::ostream& os, apMHMeSite& vpMeS);

	  void getSign(const vpImage<unsigned char> &I, const int range) ;
	  double convolution(const vpImage<unsigned char>& ima, const vpMe *me) ;

	  apMHMeSite *getQueryList(const vpImage<unsigned char> &I, const int range) ;

	apMHMeSite *getQueryListMH(const vpImage<unsigned char> &I, const int range) ;

	void track(const vpImage<unsigned char>& I,
			     const vpMe *me,
			     const  bool test_contraste=true);

	void track0(const vpImage<unsigned char>& I,
		     const vpMe *me,
		     const  bool test_contraste=true);
	void trackMH(const vpImage<unsigned char>& I,
		     const vpMe *me,
		     const  bool test_contraste=true);
	void trackMHGrad(const vpImage<unsigned char>& I,
			     const vpImage<unsigned char>& edgeMap,
			     const vpMe *me,
			     const  bool test_contraste=true);
	void setNumCandidates(int Num) {numCandidates = Num;}
	int getNumCandidates() const {return numCandidates;}
	void setWeight(double wght){weight_s = wght;}
	double getWeight(){return weight_s;}

	  void setDisplay(vpMeSiteDisplayType select) { selectDisplay = select ; }

	  /*!

	    Compute the distance \f$ |S1 - S2| = \sqrt{(i_1-i_2)^2+(j_1-j_2)^2} \f$

	    \param S1 : First site
	    \param S2 : Second site

	    \return the distance between the two sites.
	  */
	  static double distance (const apMHMeSite S1, const apMHMeSite S2) {
	    return(sqrt(vpMath::sqr(S1.ifloat-S2.ifloat)+vpMath::sqr(S1.jfloat-S2.jfloat)));}

	  /*!

	    Compute the distance \f$ |S1 - S2| = (i_1-i_2)^2+(j_1-j_2)^2 \f$

	    \param S1 : First site
	    \param S2 : Second site

	    \return the distance between the two sites.
	  */
	  static double sqrDistance (const apMHMeSite S1, const apMHMeSite S2) {
	    return(vpMath::sqr(S1.ifloat-S2.ifloat)+vpMath::sqr(S1.jfloat-S2.jfloat));}

	private:
	  vpMeSiteDisplayType selectDisplay ;
};

#endif /* APMHMESITE_H_ */
