/*
 * minE.cpp
 *
 *  Created on: Nov 29, 2012
 *      Author: agpetit
 */

#include "minE.h"
#include <visp/vpDisplayX.h>

minE::minE() {
	// TODO Auto-generated constructor stub

}

minE::~minE() {
	// TODO Auto-generated destructor stub
}

minE::minE(int res) {
	// TODO Auto-generated constructor stub
	resolution = res;
	//mask = new int[resolution];

}

void minE::minEnergy(double *Edatab, double *Edataf, double *EsmoothH, double *EsmoothV, vpImage<unsigned char> &I)
{
	/* Minimize the following function of 3 binary variables:
	   E(x, y, z) = x - 2*y + 3*(1-z) - 4*x*y + 5*|y-z| */

    //double *UCb, *UCf, *VSh, *VSv, *UMb, *UMb_not, *UMf, *UMf_not, *Vaddh, *Vaddv, *VTb, *VTb_not, *VTf, *VTf_not, *mask;
    int mrows, ncols, count, i, j, temp;
    
	Energy::Var alpha[resolution];     // OK in g++ compiler, not Microsoft Visual C/C++ version 7.1

    Energy *e = new Energy();
    
    mrows = I.getHeight();
    ncols = I.getWidth();

    int mask[resolution];
    //add  variables

    /////////////////////////// Edata ////////////////////////////////////////////

    for(count = 0; count < resolution; count++)
        alpha[count] = e -> add_variable();
    
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
    	int ii = floor(count/I.getHeight());
    	int jj = count - I.getHeight()*floor(count/I.getHeight());
    	vpImagePoint p0(ii,jj);
    	    			if(mask[count] == 0)
    	    			{
    	    				vpDisplay::displayCross(I, p0, 5, vpColor::black);
    	    			}
        }
    	/*for (int ii = 0; ii < I.getHeight(); ii++)
    		for (int jj =0 ; jj < I.getWidth() ; jj++)
    		{
    		    vpImagePoint p0(ii,jj);
    		    std::cout << (int)mask[count] << std::endl;
    			if((int)mask[count] == 1)
    			{
    				vpDisplay::displayCross(I, p0, 5, vpColor::black);
    			}
    		}*/

	delete e;
}
