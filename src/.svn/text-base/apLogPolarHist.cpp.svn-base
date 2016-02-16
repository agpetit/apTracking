#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <algorithm>
#include <cstdlib>
#include <string>
#include <cmath>
#include "apLogPolarHist.h"
#include "apContourPoint.h"
#include <visp/vpDisplayX.h>
#include <visp/vpDisplay.h>
#define PI 3.141592653589793238462643

using namespace std;

apLogPolarHist::apLogPolarHist(void){}

apLogPolarHist::~apLogPolarHist(void){}

void apLogPolarHist::init(int nr, int nw, int height, int width)
{
	nR = nr;
	nW = nw;
	imH = height;
	imW = width;
	double rinner = 10;
	double router = 300;
	logRBins = LogSpace(rinner, router, nR);
	logWBins = ThetaSpace(nW);
}

vpMatrix apLogPolarHist::Run(vector<double> x, vector<double> y, int nr, int nw){
//g++ -c bolap.cpp - COMPILE ONLY
	try{
		int nsize = (int)x.size();
		double rmean = 0; 
		vpMatrix r(nsize, nsize);
		vpMatrix theta(nsize, nsize);
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
		//cout <<"rmean: "<< rmean << endl;
		theta = Rem(theta, 2*PI);

		double rinner = 0.125;
		double router = 2.0;
		vector<double> logRbins = LogSpace(rinner, router, nr);
		vector<double> logWbins = ThetaSpace(nw);
		for(int i = 0; i < logRbins.size(); i++) cout<<logRbins[i]<<" | "; 
		cout<<endl;
		for(int i = 0; i < logWbins.size(); i++) cout<<logWbins[i]<<" | ";
		cout<<endl;

		vpMatrix LogPolarHist( nsize, nr*nw );
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
		vpMatrix LogPolarHist(0,0);
		return LogPolarHist;
	}
}

vector<double> apLogPolarHist::RunCenter(vector<double> x, vector<double> y, vpImagePoint cog, int nr, int nw){
//g++ -c bolap.cpp - COMPILE ONLY
	try{
		int nsize = (int)x.size();
		double rmean = 0;
		vector<double> r(nsize);
		vector<double> theta(nsize);
		for(int i = 0; i < nsize; i++)
			   {
					r[i] = sqrt(pow(x.at(i)-cog.get_u(),2) + pow(y.at(i)-cog.get_v(),2) );
					//cout << r[i] << endl;
					theta[i] = atan2( y.at(i)-cog.get_v(), x.at(i)-cog.get_u() );
					rmean += r[i];
				}

		rmean = rmean / pow((double)nsize,1);
		/*for(int i = 0; i < nsize; i++)
					   {
			r[i]/=rmean;
					   }*/
		//r /= rmean;
		//cout <<"rmean: "<< rmean << endl;
		theta = Rem(theta, 2*PI);

		/*double rinner = 0.125;
		double router = 2.0;*/
		double rinner = 10;
		double router = 300;
		vector<double> logRbins = LogSpace(rinner, router, nr);
		vector<double> logWbins = ThetaSpace(nw);
		/*for(int i = 0; i < logRbins.size(); i++) cout<<logRbins[i]<<" | ";
		cout<<endl;
		for(int i = 0; i < logWbins.size(); i++) cout<<logWbins[i]<<" | ";
		cout<<endl;*/

		vector<double> LogPolarHist;
		LogPolarHist.resize(nr*nw);
		double RR, WW;
			for(int rr = 0; rr < nr; rr++)
				for(int ww = 0; ww < nw; ww++)
					for(int j = 0; j < nsize; j++){
						RR = r[j]; WW = theta[j];
						if( (RR >= logRbins[rr]) && (RR <= logRbins[rr+1]) )
							if( (WW >= logWbins[ww]) && (WW <= logWbins[ww+1]) ){
								LogPolarHist[ww+(rr*nw)]++;
								//cout<<LogPolarHist[ww+(rr*nw)]<<"|";
							}
					}
			/*for(int si=0;si<LogPolarHist.size();si++)
			{
				cout<< " Log Pol " << LogPolarHist[si]<<endl;
			}*/

		return LogPolarHist;
	}
	catch ( const exception & r_e ){
		cout << "Error: " << r_e.what();
		cout << endl;
		vector<double> LogPolarHist;
		LogPolarHist.resize(0);
		return LogPolarHist;
	}
}

vector<double> apLogPolarHist::RunCenterRot(std::vector<apContourPoint*> &CP, const vpImagePoint &cog, vpImagePoint &pI, double Rot, int nr, int nw, int height, int width){
//g++ -c bolap.cpp - COMPILE ONLY
	try{
		vpImage<unsigned char> I1(512,512);
		//vpDisplayX display1;
		//display1.init(I1, 800, 10, "Dots");
		//vpDisplay::display(I1);
		int nsize = (int)CP.size()-1;
		double rmean = 0;
		vector<double> r(nsize);
		vector<double> theta(nsize);
		apContourPoint *cpoint;
		double x,y,rhoP,thetaP,xrot,yrot,yp,xp;
		vpImagePoint ip;
		for(int i = 0; i < nsize; i++)
			   {
			cpoint = CP[i+1];
			x = (int)cpoint->get_u();
			y = (int)cpoint->get_v();
			//xp =x;
			//yp = y;
				  rhoP = sqrt((y-(int)height/2)*(y-(int)height/2)+(x-(int)width/2)*(x-(int)width/2));
				  thetaP = atan2(-(y-(int)height/2),(x-(int)width/2));
			      thetaP = thetaP + Rot;
				  xrot = (int)(rhoP*cos(thetaP)+(int)width/2);
				  yrot = (int)(-rhoP*sin(thetaP)+(int)height/2);
				  yp = yrot+(int)(pI.get_i()-(int)height/2);
				  xp = xrot+(int)(pI.get_j()-(int)width/2);
					//ip.set_u(xp);
					//ip.set_v(yp);
			//vpDisplay::displayCross(I1,ip,2,vpColor::green,2);
					r[i] = sqrt(pow(xp-cog.get_u(),2) + pow(yp-cog.get_v(),2) );
					//cout << r[i] << endl;
					theta[i] = atan2( yp-cog.get_v(), xp-cog.get_u() );
					rmean += r[i];
				}
		//vpDisplay::flush(I1);
		//getchar();

		rmean = rmean / pow((double)nsize,1);
		/*for(int i = 0; i < nsize; i++)
					   {
			r[i]/=rmean;
					   }*/
		theta = Rem(theta, 2*PI);

		/*double rinner = 0.125;
		double router = 2.0;*/
		double rinner = 10;
		double router = 300;
		vector<double> logRbins = LogSpace(rinner, router, nr);
		vector<double> logWbins = ThetaSpace(nw);
		/*for(int i = 0; i < logRbins.size(); i++) cout<<logRbins[i]<<" | ";
		cout<<endl;
		for(int i = 0; i < logWbins.size(); i++) cout<<logWbins[i]<<" | ";
		cout<<endl;*/

		vector<double> LogPolarHist;
		LogPolarHist.resize(nr*nw);
		double RR, WW;
			for(int rr = 0; rr < nr; rr++)
				for(int ww = 0; ww < nw; ww++)
					for(int j = 0; j < nsize; j++){
						RR = r[j]; WW = theta[j];
						if( (RR >= logRbins[rr]) && (RR <= logRbins[rr+1]) )
							if( (WW >= logWbins[ww]) && (WW <= logWbins[ww+1]) ){
								LogPolarHist[ww+(rr*nw)]++;
								//cout<<LogPolarHist[ww+(rr*nw)]<<"|";
							}
					}

		return LogPolarHist;
	}
	catch ( const exception & r_e ){
		cout << "Error: " << r_e.what();
		cout << endl;
		vector<double> LogPolarHist;
		LogPolarHist.resize(0);
		return LogPolarHist;
	}
}

vector<double> apLogPolarHist::RunCenterRot0(std::vector<apContourPoint*> &CP, const vpImagePoint &cog, vpImagePoint &pI, double Rot){
//g++ -c bolap.cpp - COMPILE ONLY
	try{
		//vpImage<unsigned char> I1(imH,imW);
		//vpDisplayX display1;
		//display1.init(I1, 800, 10, "Dots");
		//vpDisplay::display(I1);

		int nsize = (int)CP.size()-1;
		double rmean = 0;
		vector<double> r(nsize);
		vector<double> theta(nsize);
		apContourPoint *cpoint;
		double x,y,rhoP,thetaP,xrot,yrot,yp,xp;
		vpImagePoint ip;
		for(int i = 0; i < nsize; i++)
			   {
			cpoint = CP[i+1];
			x = (int)cpoint->get_u();
			y = (int)cpoint->get_v();
			//xp =x;
			//yp = y;
				  rhoP = sqrt((y-(int)imH/2)*(y-(int)imH/2)+(x-(int)imW/2)*(x-(int)imW/2));
				  thetaP = atan2(-(y-(int)imH/2),(x-(int)imW/2));
			      thetaP = thetaP + Rot;
				  xrot = (int)(rhoP*cos(thetaP)+(int)imW/2);
				  yrot = (int)(-rhoP*sin(thetaP)+(int)imH/2);
				  yp = yrot+(int)(pI.get_i()-(int)imH/2);
				  xp = xrot+(int)(pI.get_j()-(int)imW/2);
					//ip.set_u(xp);
					//ip.set_v(yp);
			//vpDisplay::displayCross(I1,ip,2,vpColor::green,2);
					r[i] = sqrt(pow(xp-cog.get_u(),2) + pow(yp-cog.get_v(),2) );
					//cout << r[i] << endl;
					theta[i] = atan2( yp-cog.get_v(), xp-cog.get_u() );
					rmean += r[i];
				}
		rmean = rmean / pow((double)nsize,1);
		/*for(int i = 0; i < nsize; i++)
					   {
			r[i]/=rmean;
					   }*/
		double t0 = vpTime::measureTimeMs();
		theta = Rem(theta, 2*PI);
		double t1 = vpTime::measureTimeMs();
		//std::cout << " time " << t1 - t0 << std::endl;
		vector<double> LogPolarHist;
		LogPolarHist.resize(nR*nW);
		double RR, WW;
			for(int rr = 0; rr < nR; rr++)
				for(int ww = 0; ww < nW; ww++)
					for(int j = 0; j < nsize; j++){
						RR = r[j]; WW = theta[j];
						if( (RR >= logRBins[rr]) && (RR <= logRBins[rr+1]) )
							if( (WW >= logWBins[ww]) && (WW <= logWBins[ww+1]) ){
								LogPolarHist[ww+(rr*nW)]++;
								//cout<<LogPolarHist[ww+(rr*nw)]<<"|";
							}
					}


		return LogPolarHist;
	}
	catch ( const exception & r_e ){
		cout << "Error: " << r_e.what();
		cout << endl;
		vector<double> LogPolarHist;
		LogPolarHist.resize(0);
		return LogPolarHist;
	}
}

vector<double> apLogPolarHist::RunCenterRotOpt(std::vector<apContourPoint*> &CP, const vpImagePoint &cog, vpImagePoint &pI, double Rot){
//g++ -c bolap.cpp - COMPILE ONLY
	try{
		double t0 = vpTime::measureTimeMs();
		int nsize = (int)CP.size()-1;
		double rmean = 0;
		vector<double> r(nsize);
		vector<double> theta(nsize);
		apContourPoint *cpoint;
		double x,y,rhoP,thetaP,xrot,yrot,yp,xp;
		vpImagePoint ip;
		vector<double> LogPolarHist;
		LogPolarHist.resize(nR*nW);
		double RR, WW;
		double t1 = vpTime::measureTimeMs();
		//std::cout << "RunHist "<< t1-t0 << std::endl;
		for(int i = 0; i < nsize; i++)
			   {
			cpoint = CP[i+1];
			x = (int)cpoint->get_u();
			y = (int)cpoint->get_v();
			//xp =x;
			//yp = y;
				  rhoP = sqrt((y-(int)imH/2)*(y-(int)imH/2)+(x-(int)imW/2)*(x-(int)imW/2));
				  thetaP = atan2(-(y-(int)imH/2),(x-(int)imW/2));
			      thetaP = thetaP + Rot;
				  xrot = (int)(rhoP*cos(thetaP)+(int)imW/2);
				  yrot = (int)(-rhoP*sin(thetaP)+(int)imH/2);
				  yp = yrot+(int)(pI.get_i()-(int)imH/2);
				  xp = xrot+(int)(pI.get_j()-(int)imW/2);
				  //yp = (int)(-sqrt(((int)cpoint->get_v()-(int)imH/2)*((int)cpoint->get_v()-(int)imH/2)+((int)cpoint->get_u()-(int)imW/2)*((int)cpoint->get_u()-(int)imW/2))*sin(atan2(-((int)cpoint->get_v()-(int)imH/2),((int)cpoint->get_u()-(int)imW/2)) + Rot)+(int)imH/2) +(int)(pI.get_i()-(int)imH/2);
				  //xp = (int)(sqrt(((int)cpoint->get_v()-(int)imH/2)*((int)cpoint->get_v()-(int)imH/2)+((int)cpoint->get_u()-(int)imW/2)*((int)cpoint->get_u()-(int)imW/2))*cos(atan2(-((int)cpoint->get_v()-(int)imH/2),((int)cpoint->get_u()-(int)imW/2)) + Rot)+(int)imW/2) + (int)(pI.get_j()-(int)imW/2);
					//ip.set_u(xp);
					//ip.set_v(yp);
			//vpDisplay::displayCross(I1,ip,2,vpColor::green,2);
					r[i] = sqrt(pow(xp-cog.get_u(),2) + pow(yp-cog.get_v(),2) );
					//cout << r[i] << endl;
					theta[i] = atan2( yp-cog.get_v(), xp-cog.get_u() );
					double mod_val2 = fmod (fmod (theta[i], 2*PI) + 2*PI, 2*PI);
					theta[i] = mod_val2;

					for(int rr = 0; rr < nR; rr++)
						for(int ww = 0; ww < nW; ww++)
								if( (r[i] >= logRBins[rr]) && (r[i] <= logRBins[rr+1]) )
									if( (theta[i] >= logWBins[ww]) && (theta[i] <= logWBins[ww+1]) ){
										LogPolarHist[ww+(rr*nW)]++;
										//cout<<LogPolarHist[ww+(rr*nw)]<<"|";
									}


					//rmean += r[i];
				}

		//std::cout << " time " << t1 - t0 << std::endl;


		return LogPolarHist;
	}
	catch ( const exception & r_e ){
		cout << "Error: " << r_e.what();
		cout << endl;
		vector<double> LogPolarHist;
		LogPolarHist.resize(0);
		return LogPolarHist;
	}
}


/*vector<double> apLogPolarHist::RunV(vector<double> x, vector<double> y, int nr, int nw){
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

		vpMatrix LogPolarHist( nsize, nr*nw );
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
		vector<double> LogPolarHist;
		LogPolarHist.resize(0);
		return LogPolarHist;
	}
}*/

vector<double> apLogPolarHist::ReadFromFile(string filename){
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

vpMatrix apLogPolarHist::Rem(vpMatrix &m, double value){
	vpMatrix rem(m.getRows(),m.getCols());
	for(int i = 0; i < rem.getRows(); i++){
		for(int j = 0; j < rem.getCols(); j++){
			double mod_val1 = fmod (m[i][j], value);
			double mod_val2 = fmod (mod_val1 + 2*PI, value);
			rem[i][j] = mod_val2;
		}
	}

	return rem;
}

vector<double> apLogPolarHist::Rem(vector<double> &m, double value){
	vector<double> rem(m.size());
	for(int i = 0; i < rem.size(); i++){
			double mod_val1 = fmod (m[i], value);
			double mod_val2 = fmod (mod_val1 + 2*PI, value);
			rem[i] = mod_val2;
		}

	return rem;
}

vector<double> apLogPolarHist::LogSpace(double a, double b, int n){
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

vector<double> apLogPolarHist::ThetaSpace(int nw){
	vector<double> tspace;
	double ts = 0;

	for(int i = 0; i < nw; i++){
		ts = i*2*PI/nw;
		tspace.push_back(ts);
	}
	tspace.push_back(2*PI);

	return tspace;
}

void apLogPolarHist::SaveToFile(string filename, vpMatrix &CostF){

	ofstream cfile (filename.c_str(),ios::out);
	if (cfile.is_open())
	{
		int sizeR = CostF.getRows();
		int sizeC = CostF.getCols();
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

void apLogPolarHist::SaveToFile(string filename, std::vector<double> &CostF){

	ofstream cfile (filename.c_str(),ios::out);
	if (cfile.is_open())
	{
		int size = CostF.size();
		for(int i = 0; i < size; i++){
				cfile << CostF[i] <<"\t";
			}
		cfile.close();
	}
	else
		cout << "Unable to open file";
}
