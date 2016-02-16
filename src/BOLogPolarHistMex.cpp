/***************************************************************************
 *   Copyright (C) 2008 by Boguslaw Obara                                  *
 *   http://boguslawobara.net                                              *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include <vector>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "BOLogPolarHist.h"
#include "BOMatrix.h"

using namespace std;

extern "C" {
	#include "mex.h"
}
//------------------------------------------------------------------------------
// function: mex_bimread - entry point from Matlab via mexFucntion()
// INPUTS:
//   nlhs 		- number of left hand side arguments (outputs)
//   plhs[] 	- pointer to table where created matrix pointers are
//            		to be placed
//   nrhs 		- number of right hand side arguments (inputs)
//   prhs[] 	- pointer to table of input matrices
//------------------------------------------------------------------------------

//mex BOLogPolarHistMex.cpp BOLogPolarHist.cpp BOMatrix.cpp
//H = BOLogPolarHistMex( p_x, p_y, nr, nw);

void BOLogPolarHistMex( int nlhs, mxArray *plhs[], int nrhs, const mxArray  *prhs[] )
{
	// Check for proper number of input and output arguments
	if (nrhs < 1) mexErrMsgTxt("Input arguments required. Usage: = BOLogPolarHistMex(x[], y[], nr, nw).");
	if (nlhs < 1) mexErrMsgTxt("One output argument required. Usage: H = BOLogPolarHistMex.");
	if (nrhs > 4) mexErrMsgTxt("Too many input arguments.");
	if (nlhs > 1) mexErrMsgTxt("Too many output arguments.");
	//-----------------------------------------------------------
	// get input
	//-----------------------------------------------------------
	int nr, nw;
	vector<double> x;
	vector<double> y;

	if (nrhs > 0 && nlhs > 0) {
		printf("BOLogPolarHistMex -> Start  :)\n");	

		// get inputs
		//-----------------------------------------------------------
		//Get vector x
		double* xValues = mxGetPr(prhs[0]);
		int xrowLen = mxGetN(prhs[0]);
		int xcolLen = mxGetM(prhs[0]);
		for(int i=0;i<xrowLen;i++)
		for(int j=0;j<xcolLen;j++){
			x.push_back( (double)xValues[(i*xcolLen)+j] );
			//printf("rowsol: %d \n",rowsol[i]);
		}
		
		//Get vector y
		double* yValues = mxGetPr(prhs[1]);
		int yrowLen = mxGetN(prhs[1]);
		int ycolLen = mxGetM(prhs[1]);
		//printf("Array: %d , %d ",assigncostrowLen, assigncostcolLen);
		for(int i=0;i<yrowLen;i++)
		for(int j=0;j<ycolLen;j++){
			y.push_back( (double)yValues[(i*ycolLen)+j] );
			//printf("rowsol: %d \n",rowsol[i]);
		}


		//Get the Integer nr and nw
		nr = (int)mxGetScalar(prhs[2]);
		nw = (int)mxGetScalar(prhs[3]);
		//printf("nr: %d \n", nr);

		// create output
		//-----------------------------------------------------------
		//double *HistD = new double [xcolLen*nr*nw];
		//plhs[0]    = mxCreateDoubleMatrix(xcolLen, nr*nw, mxREAL);
		//HistD = mxGetPr(plhs[0]);
		const int dims[] = { xcolLen, nr*nw};
		mxClassID t = mxDOUBLE_CLASS;
  		plhs[0] = mxCreateNumericArray( 2, dims, t, mxREAL);
    	double *HistD = (double *) mxGetData(plhs[0]);

		// run
		//-----------------------------------------------------------
		//BOMatrix<double> Hist(xcolLen,nr*nw);
		BOLogPolarHist* lph = new BOLogPolarHist();
		BOMatrix<double> Hist = lph->Run(x, y, nr, nw);

		
		for(int j=0;j<nr*nw;j++)
			for(int i=0;i<xcolLen;i++){
	      		*HistD = Hist[i][j];
				++HistD;
			}
		//vector<double> xp = lph->ReadFromFile("files/x.txt");
		//vector<double> yp = lph->ReadFromFile("files/y.txt");
		//int nr =5, nw = 12;
		printf("BOLogPolarHistMex -> Finished :)\n");
	}
	//-----------------------------------------------------------
}


extern "C" {
  //--------------------------------------------------------------
  // mexFunction - Entry point from Matlab. From this C function,
  //   simply call the C++ application function, above.
  //--------------------------------------------------------------
  void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray  *prhs[] )
  {
    BOLogPolarHistMex(nlhs, plhs, nrhs, prhs);
  }
}
