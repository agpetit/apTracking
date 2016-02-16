#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include <time.h>

#include "minE.h"
//MRF library
#include "mrf.h"
#include "GCoptimization.h"


const int numLabels = 2;
int numOfRows, numOfCols;
//int numOfElements;
unsigned char *mask;      //the mask contains fg, bg and unknown
unsigned char* Y;         //the lumi of the image
unsigned char* U;         //chrominance
unsigned char* V;         //chrominance
MRF::CostVal* DCf;      //data cost as fg
MRF::CostVal* DCb;      //data cost as bg
//int* posR;
//int* posC;
//mxLogical* res;
FILE* costFile;
//mxArray* res;
unsigned char *mask_;


minE::minE(){

}

minE::~minE() {
	// TODO Auto-generated destructor stub
}

void minE::init(int res) {
	// TODO Auto-generated constructor stub
	resolution = res;
	mask_ = new unsigned char[resolution];

}

//void computeMRF();


#define SCALE_FACTOR 1
#define INF 10000000000
//set up data terms using a function
MRF::CostVal dCost(int pix, int i)
{
    MRF::CostVal answer;
	if (i == 1) {
        //energy required to label it as foreground
        if (mask[pix] == 1) {
            answer = 0;
        } else if (mask[pix] == 0) {
            answer = INF;
            //fprintf(costFile, "fg: %f; pix: %d\n", answer, pix);
        } else {
            answer = SCALE_FACTOR*DCf[pix];
            //fprintf(costFile, "fg: %f; pix: %d\n", answer, pix);
        }
	} else if (i == 0) {
        if (mask[pix] == 1) {
            answer = INF;
            //mexPrintf("fg: %f\n", answer);
            //fprintf(costFile, "bg: %f; pix: %d\n", answer, pix);
        } else if (mask[pix] == 0) {
            answer = 0;
        } else {
            answer = SCALE_FACTOR*DCb[pix];
            //fprintf(costFile, "bg: %f; pix: %d\n", answer, pix);
        }
	}
    return answer;
}

//#define SIG 2.5 //for test1.avi
#define SIG 2.5
//set up smoothness terms using a function
MRF::CostVal fnCost(int pix1, int pix2, int i, int j)
{
    if (pix2 < pix1) { // ensure that fnCost(pix1, pix2, i, j) == fnCost(pix2, pix1, j, i)
		int tmp;
		tmp = pix1; pix1 = pix2; pix2 = tmp;
		tmp = i; i = j; j = tmp;
    }
	MRF::CostVal answer;
	//
	if (i == j) {
		//if both pix1 and pix2 belong to same category, answer is 0 as |Xi-Xj| = 0
		answer = 0.0;
	} else if ((mask[pix1] == 0 || mask[pix1] == 1) && (mask[pix2] == 0 || mask[pix2] == 1)) {
        //if the pixels are not in the unknown region, we set the smoonth cost as 0, the labelling will depend on data cost
        //this is to reduce computation time
        answer = 0.0;
    } else {
		//if pix1 and pix2 belong to different categories
		MRF::CostVal colorDiff = (U[pix1]-U[pix2])*(U[pix1]-U[pix2]) + (V[pix1]-V[pix2])*(V[pix1]-V[pix2]) + (Y[pix1]-Y[pix2])*(Y[pix1]-Y[pix2]); //color difference^2
		//spatial distance
        int row1 = pix1/numOfCols; int col1 = pix1%numOfCols;
		int row2 = pix2/numOfCols; int col2 = pix2%numOfCols;
		MRF::CostVal distDiff = sqrt((double)(row1-row2)*(row1-row2)+(col1-col2)*(col1-col2));
        //it's not necessary, as it's always have a distance of 1
        /*if (distDiff <= 0.000000000001) {
            //to avoid dividing by 0
            distDiff = 0.000000000001;
        } */
        //equation 4 of the paper
		answer=2*SCALE_FACTOR*6000000*exp(-0.5*colorDiff/(SIG*SIG))/distDiff;     //for test1.avi
        //answer=SCALE_FACTOR*60000000*exp(-0.5*colorDiff/(SIG*SIG))/distDiff;
        //fprintf(costFile, "sm cost: %f; pix1: %d; pix2: %d\n", answer, pix1, pix2);
	}
    //mexPrintf("%f\n", answer);
    return answer;
}

EnergyFunction* generate_DataFUNCTION_SmoothGENERAL_FUNCTION()
{
    DataCost *data         = new DataCost(dCost);
    SmoothnessCost *smooth = new SmoothnessCost(fnCost);
    EnergyFunction *energy = new EnergyFunction(data,smooth);

    return energy;
}

void computeMRF(vpImage<unsigned char> &I) {
	MRF* mrf;
    EnergyFunction *energy;
    MRF::EnergyVal E, ES, ED;
    double lowerBound;
    float t,tot_t;
    int iter;
	int label, ind;
    //FILE* logfile;

    /*logfile = fopen("log.txt", "w");
    if (logfile == NULL) {
        mexPrintf("error creating log file\n");
        return;
    } else {
        //mexPrintf("log.txt created\n");
    }*/

	/*for (int i = 0; i < numOfRows; ++i) {
		for (int j = 0; j < numOfCols; ++j) {
			//mask_[j*numOfRows + i] = (mrf->getLabel(j*numOfRows + i)==1?true:false);
			std::cout << (int)mask[j*numOfRows + i] << std::endl;
			//fprintf(logfile, "%d ", mrf->getLabel(j*numOfRows + i));
		}
		//fprintf(logfile, "\n");
	}*/

	energy = generate_DataFUNCTION_SmoothGENERAL_FUNCTION();
	//mexPrintf("using general smoothness functions\n");

	////////////////////////////////////////////////
	//          Graph-cuts expansion              //
	////////////////////////////////////////////////
	//use expansion instead of swap
	//see if we can reduce the iterations and also the optimze looping time
	try {
		//print out the mask before mrf
		/*for (int i = 0; i < numOfRows; ++i) {
			for (int j = 0; j < numOfCols; ++j) {
				fprintf(logfile, "%d ", mask[j*numOfRows + i]);
			}
			fprintf(logfile, "\n");
		}
		fprintf(logfile, "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");*/
		mrf = new Expansion(numOfCols, numOfRows, numLabels, energy);
		//mrf = new Swap(numOfCols, numOfRows, numLabels, energy);
		//mrf = new Expansion(numOfElements, numLabels, energy);
		//todo: set neighbors

		mrf->initialize();
		mrf->clearAnswer();

		E = mrf->totalEnergy();
		//fprintf(logfile, "Energy at the Start= %g (%g,%g)\n", (float)E,
		//   (float)mrf->smoothnessEnergy(), (float)mrf->dataEnergy());
		//tot_t = 0;
		//for (iter=0; iter<6; iter++) {
		mrf->optimize(1, t);
		//E = mrf->totalEnergy();
		//ES = mrf->smoothnessEnergy();
		//ED = mrf->dataEnergy();
		//tot_t = tot_t + t ;
		//fprintf(logfile, "energy = %g; s: %g; d: %g(%f secs)\n", (float)E, (float)ES, (float)ED, tot_t);
		//fprintf("Data Energy: \n");
		//}

		vpImagePoint p1;
		/*int *mask_;
		mask_ = minEnergy.mask;*/
		for (int i = 0; i<I.getHeight(); i++)
			for (int j =0 ; j < I.getWidth() ; j++)
			{
				p1.set_i(i);
				p1.set_j(j);
				//std::cout << minEnergy.mask[i*I.getWidth() + j] << std::endl;
				if( mrf->getLabel(i*numOfRows + j) == 0)
					vpDisplay::displayCross(I, p1, 5, vpColor::black);
			}

		for (int i = 0; i < numOfRows; ++i) {
			for (int j = 0; j < numOfCols; ++j) {
				//mask_[j*numOfRows + i] = (mrf->getLabel(j*numOfRows + i)==1?true:false);
				mask_[j*numOfRows + i] = mrf->getLabel(j*numOfRows + i);
				//std::cout << (int)mask_[j*numOfRows + i] << std::endl;
				//fprintf(logfile, "%d ", mrf->getLabel(j*numOfRows + i));
			}
			//fprintf(logfile, "\n");
		}
		delete mrf;

    }
    catch (std::bad_alloc) {
		exit(1);
    }
    //fclose(logfile);
    return;
}

unsigned char* minEn(MRF::CostVal *DeF, MRF::CostVal *DeB, unsigned char *R, unsigned char *G, unsigned char *B, unsigned char *_mask, int n, int m, vpImage<unsigned char> &I)
{
	/*FILE* logfile;
    costFile = fopen("cost.txt", "w");
    logfile = fopen("input.txt", "w");*/
	DCf = DeF;
	DCb = DeB;

	Y = R;
	U = G;
	V = B;
	mask = _mask;
	numOfRows = m;
	numOfCols = n;
    
    //printf("DCF\n");
    /*for (int i = 0; i < numOfRows*numOfCols; ++i) {
        fprintf(logfile, "%f ", DCf[i]);
        //mexPrintf("% f", DCf[i]);
    }
    fprintf(logfile, "\n");
    //printf("DCB\n");
    for (int i = 0; i < numOfRows*numOfCols; ++i) {
        fprintf(logfile, "%f\n", DCb[i]);
    }
    fprintf(logfile, "\n");
    fclose(logfile);   */
	//mexPrintf("%d\n", numOfRows);
	//mexPrintf("%d\n", numOfCols);
	//plhs[0] = mxCreateLogicalMatrix(1, numOfRows*numOfCols);
	//res = (mxLogical*)mxGetData(plhs[0]);
    //nlhs = 1;
    computeMRF(I);

    return mask_;
    //mexPrintf("end of c++ processing\n");
}

void minE::minEnergy(MRF::CostVal *DeF, MRF::CostVal *DeB, unsigned char *R, unsigned char *G, unsigned char *B, unsigned char *_mask, int n, int m, vpImage<unsigned char> &I)
{
	mask = minEn(DeF,DeB,R,G,B,_mask,n,m, I);

	_mask = mask;
}
