#pragma once
#include <string>
//#include "BOMatrix.h"
#include <visp/vpMatrix.h>
#include <visp/vpImagePoint.h>
#include "apContourPoint.h"

using namespace std;

class apLogPolarHist
{
public:
	apLogPolarHist(void);
	void init(int nr, int nw, int height, int width);
	vpMatrix Run(vector<double> x, vector<double> y, int nr, int nw);
	vector<double> RunCenter(vector<double> x, vector<double> y, vpImagePoint cog, int nr, int nw);
	vector<double> RunCenterRot(std::vector<apContourPoint*> &CP, const vpImagePoint &cog, vpImagePoint &pI, double Rot, int nr, int nw, int height, int width);
	vector<double> RunCenterRot0(std::vector<apContourPoint*> &CP, const vpImagePoint &cog, vpImagePoint &pI, double Rot);
	vector<double> RunCenterRotOpt(std::vector<apContourPoint*> &CP, const vpImagePoint &cog, vpImagePoint &pI, double Rot);
	//vector<double> RunV(vector<double> x, vector<double> y, int nr, int nw);
	vector<double> ReadFromFile(string filename);
	vpMatrix Rem(vpMatrix &m, double value);
	vector<double> Rem(vector<double> &m, double value);
	vector<double> LogSpace(double a, double b, int n);
	vector<double> ThetaSpace(int nw);
	void SaveToFile(string filename, vpMatrix &CostF);
	void SaveToFile(string filename, std::vector<double> &CostF);
	void setR(int nr){ nR = nr;}
	void setW(int nw){ nW = nw;}
	void setHeight(int height){ imH = height;}
	void setWidth(int width){ imW = width;}

public:
	~apLogPolarHist(void);

private:
	int nR;
	int nW;
	int imH;
	int imW;

	vector<double> logRBins;
	vector<double> logWBins;
};
