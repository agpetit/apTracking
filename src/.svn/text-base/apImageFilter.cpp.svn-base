/*
 * apImageFilter.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: agpetit
 */

#include "apImageFilter.h"


void
apImageFilter::filterTh(const vpImage<unsigned char> &I, vpImage<unsigned char> &Ifc,
		      const vpMatrix& M, const double th)
{

  unsigned int size = M.getRows() ;
  unsigned int half_size = floor(size/2);

  Ifc.resize(I.getHeight(),I.getWidth()) ;
  Ifc = 0;

  for (unsigned int i=half_size ; i < I.getHeight()-half_size ; i++)
  {
    for (unsigned int j=half_size ; j < I.getWidth()-half_size ; j++)
    {
      double   conv_x = 0 ;

      for(unsigned int a = 0 ; a < size ; a++ )
        for(unsigned int b = 0 ; b < size ; b++ )
    {
	  double val =  I[i-half_size+a][j-half_size+b] ;
	  conv_x += M[a][b] * val ;
	}
      if (conv_x>th) {Ifc[i][j]=255;}
      else {Ifc[i][j]=0;}
    }
  }

}


double
apImageFilter::sobelFilterX(const vpImage<double> & fr, int &r, int &c)
{
	return 255*(2*(fr[r][c+1] - fr[r][c-1])
					+(fr[r+1][c+1] - fr[r+1][c-1])
					+(fr[r-1][c+1] - fr[r-1][c-1]));
}

/*!
 Apply a 3x1 Derivative Filter to an image pixel.

 \param fr : Image to filter
 \param r : coordinates (row) of the pixel
 \param c : coordinates (column) of the pixel
 */

double
apImageFilter::sobelFilterY(const vpImage<double> & fr, int &r, int &c)
{
	return 255*(2*(fr[r+1][c] - fr[r-1][c])
					+(fr[r+1][c+1] - fr[r-1][c+1])
					+(fr[r+1][c-1] - fr[r-1][c-1]));
}

double
apImageFilter::sobelFilterX(const vpImage<unsigned char> & fr, int &r, int &c)
{
	return (2*(fr[r][c+1] - fr[r][c-1])
					+(fr[r+1][c+1] - fr[r+1][c-1])
					+(fr[r-1][c+1] - fr[r-1][c-1]));
}

/*!
 Apply a 3x1 Derivative Filter to an image pixel.

 \param fr : Image to filter
 \param r : coordinates (row) of the pixel
 \param c : coordinates (column) of the pixel
 */

double
apImageFilter::sobelFilterY(const vpImage<unsigned char> & fr, int &r, int &c)
{
	return (2*(fr[r+1][c] - fr[r-1][c])
					+(fr[r+1][c+1] - fr[r-1][c+1])
					+(fr[r+1][c-1] - fr[r-1][c-1]));
}

double
apImageFilter::lapFilter(const vpImage<unsigned char> & fr, int &r, int &c)
{
	return (-4*fr[r][c] + fr[r-1][c]
					+fr[r][c+1] + fr[r+1][c]
					+fr[r][c-1]);
}

