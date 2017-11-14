#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <algorithm>
#include <cstdlib>
#include <string>
#include <cmath>
#include "BOLogPolarHist.h"

#define PI 3.141592653589793238462643

using namespace std;

BOLogPolarHist::BOLogPolarHist(void){}

BOLogPolarHist::~BOLogPolarHist(void){}

vpMatrix<double> BOLogPolarHist::Run(vector<double> x, vector<double> y, int nr, int nw){
//g++ -c bolap.cpp - COMPILE ONLY
	try{
		int nsize = (int)x.size();
		double rmean = 0; 
		vpMatrix<double> r(nsize, nsize);
		vpMatrix<double> theta(nsize, nsize);
		for(int i = 0; i < nsize; i++)
			for(int j = 0; j < nsize; j++)
				if( j != i ){
					r[i][j] = sqrt(pow(x.at(i)-x.at(j),2) + pow(y.at(i)-y.at(j),2) );
					//cout << r[i][j] << endl;
					theta[i][j] = atan2( y.at(i)-y.at(j), x.at(i)-x.at(j) );
					rmean += r[i][j];
				}

		rmean = rmean / pow((double)nsize,2);
		r /= rmean;
		cout <<"rmean: "<< rmean << endl;
		theta = Rem(theta, 2*PI);

		double rinner = 0.125;
		double router = 2.0;
		vector<double> logRbins = LogSpace(rinner, router, nr);
		vector<double> logWbins = ThetaSpace(nw);
		for(int i = 0; i < logRbins.size(); i++) cout<<logRbins[i]<<" | "; 
		cout<<endl;
		for(int i = 0; i < logWbins.size(); i++) cout<<logWbins[i]<<" | ";
		cout<<endl;

		vpMatrix<double> LogPolarHist( nsize, nr*nw );
		double RR, WW;
		for(int i = 0; i < nsize; i++)
			for(int rr = 0; rr < nr; rr++)
				for(int ww = 0; ww < nw; ww++)
					for(int j = 0; j < nsize; j++){
						RR = r[i][j]; WW = theta[i][j];
						if( (RR >= logRbins[rr]) && (RR <= logRbins[rr+1]) ) 
							if( (WW >= logWbins[ww]) && (WW <= logWbins[ww+1]) ){
								LogPolarHist[i][ww+(rr*nw)]++;
								//cout<<LogPolarHist[i][ww+(rr*nw)]<<"|";
							}
					}

		return LogPolarHist;
	}
	catch ( const exception & r_e ){
		cout << "Error: " << r_e.what();
		cout << endl;
		vpMatrix<double> LogPolarHist(0,0);
		return LogPolarHist;
	}
}

vector<double> BOLogPolarHist::Run(vector<double> x, vector<double> y, int nr, int nw){
//g++ -c bolap.cpp - COMPILE ONLY
	try{
		int nsize = (int)x.size();
		double rmean = 0;
		vector<double> r(nsize);
		vector<double> theta(nsize);
		for(int i = 0; i < nsize; i++)
			for(int j = 0; j < nsize; j++)
				if( j != i ){
					r[i][j] = sqrt(pow(x.at(i),2) + pow(y.at(i),2) );
					//cout << r[i][j] << endl;
					theta[i][j] = atan2( y.at(i), x.at(i));
					rmean += r[i][j];
				}

		rmean = rmean / pow((double)nsize,2);
		r /= rmean;
		cout <<"rmean: "<< rmean << endl;
		theta = Rem(theta, 2*PI);

		double rinner = 0.125;
		double router = 2.0;
		vector<double> logRbins = LogSpace(rinner, router, nr);
		vector<double> logWbins = ThetaSpace(nw);
		for(int i = 0; i < logRbins.size(); i++) cout<<logRbins[i]<<" | ";
		cout<<endl;
		for(int i = 0; i < logWbins.size(); i++) cout<<logWbins[i]<<" | ";
		cout<<endl;

		vpMatrix<double> LogPolarHist( nsize, nr*nw );
		double RR, WW;
		for(int i = 0; i < nsize; i++)
			for(int rr = 0; rr < nr; rr++)
				for(int ww = 0; ww < nw; ww++)
					for(int j = 0; j < nsize; j++){
						RR = r[i][j]; WW = theta[i][j];
						if( (RR >= logRbins[rr]) && (RR <= logRbins[rr+1]) )
							if( (WW >= logWbins[ww]) && (WW <= logWbins[ww+1]) ){
								LogPolarHist[i][ww+(rr*nw)]++;
								//cout<<LogPolarHist[i][ww+(rr*nw)]<<"|";
							}
					}

		return LogPolarHist;
	}
	catch ( const exception & r_e ){
		cout << "Error: " << r_e.what();
		cout << endl;
		vpMatrix<double> LogPolarHist(0,0);
		return LogPolarHist;
	}
}

vector<double> BOLogPolarHist::ReadFromFile(string filename){
	vector<double> v;
	double value;

	ifstream vfile (filename.c_str());
	if (vfile.is_open())
	{
		while (! vfile.eof())
		{
			vfile >> value;
			v.push_back( value );
		}
		vfile.close();
	}
	else 
		cout << "Unable to open file"; 

	return v;
}

vpMatrix<double> BOLogPolarHist::Rem(vpMatrix<double> m, double value){
	vpMatrix<double> rem(m.r,m.c);
	for(int i = 0; i < rem.r; i++){
		for(int j = 0; j < rem.c; j++){
			double mod_val1 = fmod (m[i][j], value);
			double mod_val2 = fmod (mod_val1 + 2*PI, value);
			rem[i][j] = mod_val2;
		}
	}

	return rem;
}

vector<double> BOLogPolarHist::LogSpace(double a, double b, int n){
	vector<double> lspace;
	double ls = 0;
	double la = log10(a); 
	double lb = log10(b);

	for(int i = 0; i < n; i++){
		ls = la + i*(lb-la)/(floor(n));
		ls = pow(10,ls); 
		lspace.push_back(ls);
	}
	lspace.push_back(b);

	return lspace;
}

vector<double> BOLogPolarHist::ThetaSpace(int nw){
	vector<double> tspace;
	double ts = 0;

	for(int i = 0; i < nw; i++){
		ts = i*2*PI/nw;
		tspace.push_back(ts);
	}
	tspace.push_back(2*PI);

	return tspace;
}

void BOLogPolarHist::SaveToFile(string filename, vpMatrix<double> CostF){

	ofstream cfile (filename.c_str(),ios::out);
	if (cfile.is_open())
	{
		int sizeR = CostF.r;
		int sizeC = CostF.c;
		for(int i = 0; i < sizeR; i++){
			for(int j = 0; j < sizeC; j++){
				cfile << CostF[i][j] <<"\t";
			}
			cfile <<"\n";
		}
		cfile.close();
	}
	else 
		cout << "Unable to open file"; 
}
