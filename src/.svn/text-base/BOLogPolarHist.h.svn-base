#pragma once
#include <string>
#include "BOMatrix.h"
#include <visp/vpMatrix.h>

using namespace std;

class BOLogPolarHist
{
public:
	BOLogPolarHist(void);
	vpMatrix<double> Run(vector<double> x, vector<double> y, int nr, int nw);
	vector<double> ReadFromFile(string filename);
	vpMatrix<double> Rem(vpMatrix<double> m, double value);
	vector<double> LogSpace(double a, double b, int n);
	vector<double> ThetaSpace(int nw);
	void SaveToFile(string filename, vpMatrix<double> CostF);
public:
	~BOLogPolarHist(void);
};
