/*
 * minE.cpp
 *
 *  Created on: Nov 29, 2012
 *      Author: agpetit
 */

#include "minE.h"
#include <visp/vpDisplayX.h>
#include <omp.h>

minE::minE() {
	// TODO Auto-generated constructor stub

}

minE::~minE() {
	// TODO Auto-generated destructor stub
}

void minE::init(int res) {
	// TODO Auto-generated constructor stub
	resolution = res;
	//mask = new int[resolution];

}

void minE::minEnergy(double *Edatab, double *Edataf, double *EsmoothH, double *EsmoothV, unsigned char* mask_, vpImage<unsigned char> &I)
{

    int mrows, ncols, count, i, j, temp;
    
	std::vector<Energy::Var> alpha(resolution);     //

    Energy *e = new Energy();
    
    mrows = I.getHeight();
    ncols = I.getWidth();

    std::vector<int> mask(resolution);
    //add  variables


    /////////////////////////// Edata ////////////////////////////////////////////

    for(count = 0; count < mrows*ncols; count++)
    {
        alpha[count] = e -> add_variable();
    }
    
    for(count = 0; count < resolution; count++)
    {
        e -> add_term1(alpha[count], Edatab[count], Edataf[count]); /* add Term Edata */

    }

    /////////////////////////// Esmooth ////////////////////////////////////////////

    //add horizontal terms
    for(i = 0; i < mrows; i++)
        for(j = 0; j < ncols - 1; j++)
            e -> add_term2(alpha[j*mrows + i], alpha[(j+1)*mrows + i], 0, EsmoothH[j*mrows + i], EsmoothH[j*mrows + i], 0);


    //add vertical terms
    for(j = 0; j < ncols; j++)
        for(i=0; i < mrows - 1; i++)
            e -> add_term2(alpha[j*mrows + i], alpha[j*mrows + i + 1], 0, EsmoothV[j*(mrows-1) + i], EsmoothV[j*(mrows-1) + i], 0);


    ///////////////////////////////////////////////////////////////////////////
	Energy::TotalValue Emin = e -> minimize();
    //plhs[0] = mxCreateDoubleMatrix(mrows, ncols, mxREAL);
    
    //mask = mxGetPr(plhs[0]);

    for(count = 0; count < resolution; count++)
        {
    	mask[count] = (int)(e -> get_var(alpha[count]));
    	mask_[count] = (e -> get_var(alpha[count]));
    	int ii = floor(count/I.getHeight());
    	int jj = count - I.getHeight()*floor(count/I.getHeight());
    	vpImagePoint p0(ii,jj);
    	    			if(mask[count] == 0)
    	    			{
    	    				vpDisplay::displayCross(I, p0, 5, vpColor::black);
    	    			}
        }

	delete e;
}
