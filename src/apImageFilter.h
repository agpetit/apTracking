/*
 * apImageFilter.h
 *
 *  Created on: Apr 5, 2012
 *      Author: agpetit
 */

#ifndef APIMAGEFILTER_H_
#define APIMAGEFILTER_H_
#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>

#include <visp/vpConfig.h>
#include <visp/vpImageException.h>
#include <visp/vpImage.h>
#include <visp/vpMatrix.h>
#include <visp/vpMath.h>
#include <visp/vpImageFilter.h>

class apImageFilter: public vpImageFilter
{
public:

	  static void filterTh(const vpImage<unsigned char> &I,
			     vpImage<unsigned char> &If,
	  		     const vpMatrix& M, const double th) ;

	  static double  sobelFilterX(const vpImage<double> &I,
					   int &r, int &c) ;

	  static double  sobelFilterY(const vpImage<double> &I,
					   int &r, int &c) ;

	  static double  sobelFilterX(const vpImage<unsigned char> &I,
					   int &r, int &c) ;

	  static double  sobelFilterY(const vpImage<unsigned char> &I,
					   int &r, int &c) ;
	  static double lapFilter(const vpImage<unsigned char> & fr, int &r, int &c);


};

#endif /* APIMAGEFILTER_H_ */
