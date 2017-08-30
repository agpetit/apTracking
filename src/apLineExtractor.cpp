#include "apLineExtractor.h"


void
apLineExtractor::setParameterSpace(const int n, const int m, vpImage<unsigned char> &I)
{
//V.init(n,m);
Theta.resize(n,true);
Rho.resize(m,true);
Theta[0]=0;
int h,w;
double r;
h=I.getHeight();
w=I.getWidth();
r=floor(sqrt(h*h+w*w));
Rho.resize(r,true);
V.init(n,r);
Rho[0]=-r/2;
//Rho[0]=0;
int i,j;
for (i=0;i<n-1;i++)
{
Theta[i+1]=180/n+Theta[i];
}
for (j=0;j<m-1;j++)
//for (j=0;j<r-1;j++)
{
Rho[j+1]=(r)/m+Rho[j];
//Rho[j+1]=Rho[j]+1;
}
}


void
apLineExtractor::setLinePoints(const vpImage<unsigned char> &I,const int th, const apZBuffer zbuf, vpCameraParameters &cam)
{
//Zc=zbuf.getZ();
//cout<<Zc<<endl;
vpPoint P1,P2;
vpImagePoint ip1;
vpImagePoint ip2;//(100,100);
vpMbtDistanceLine *l;
int k=0;
//vpDisplay::displayLine(I,ip1,ip2,vpColor::red,2);
double x1,y1,x2,y2,z1,z2;
int i,j;
for (i=0;i<V.getVRows();i++)
	for (j=0;j<V.getVCols();j++)
	{
{
	if(V.getVote(i,j)>th)
{ip1=V.getExt1(i*j);
ip2=V.getExt2(i*j);
vpPixelMeterConversion::convertPoint(cam,ip1,x1,y1);
vpPixelMeterConversion::convertPoint(cam,ip2,x2,y2);
if (vpImagePoint::distance(ip1,ip2)>40){
vpDisplay::displayLine(I,ip1,ip2,vpColor::red,2);
k++;
cout<<k<<endl;
}
//getchar();
/*z1=Zc[(int)ip1.get_i()][(int)ip1.get_j()];
z2=Zc[(int)ip2.get_i()][(int)ip2.get_j()];
P1.setWorldCoordinates(x1,y1,z1);
P2.setWorldCoordinates(x2,y2,z2);
l= new vpMbtDistanceLine;
l->setCameraParameters(&cam) ;
//getchar();
l->buildFrom(P1,P2);
l->setMovingEdge(&me) ;
l->setIndex(nline);
nline +=1;
lines.push_back(l);*/
}
}
}
}

/*void
apLineExtractor::setLines(vector<Vec4i> &Lines,const apZBuffer &zbuf, vpCameraParameters &cam)
{
Zc=zbuf.getZ();
//cout<<Zc<<endl;
vpPoint P1,P2;
vpImagePoint ip1;
vpImagePoint ip2;//(100,100);
vpMbtDistanceLine *l;
double x1,y1,x2,y2,z1,z2;

for( size_t i = 0; i < Lines.size(); i++)
                            {
	ip1.set_i(Lines[i][0]);
	ip1.set_j(Lines[i][1]);
	ip2.set_i(Lines[i][2]);
	ip2.set_j(Lines[i][3]);
	z1=Zc[Lines[i][0]][Lines[i][1]];
	z2=Zc[Lines[i][2]][Lines[i][3]];
vpPixelMeterConversion::convertPoint(cam,ip1,x1,y1);
vpPixelMeterConversion::convertPoint(cam,ip2,x2,y2);
P1.setWorldCoordinates(x1,y1,z1);
P2.setWorldCoordinates(x2,y2,z2);
l= new vpMbtDistanceLine;
l->setCameraParameters(&cam) ;
//getchar();
l->buildFrom(P1,P2);
l->setMovingEdge(&me) ;
l->setIndex(nline);
nline +=1;
lines.push_back(l);
                            }


}*/

void
apLineExtractor::buildHoughVote(const vpImage<unsigned char> &I)
{
vpImagePoint ip,ip1;
double n,m;
double rho;
int i,j;
setFeaturePoints(I);
for (int k=0;k<points.size();k++){
        ip = points[k];
        n=ip.get_i();
        m=ip.get_j();
        //cout<<Rho<<endl;
        for (i=0;i<Theta.getRows();i++)
        {rho = (n-240)*cos(Theta[i]*(3.1416/180))+(m-320)*sin(Theta[i]*(3.1416/180));
        j=0;
        //cout<<Rho<<endl;
        //getchar();
        //cout<<rho<<endl;
        //for (j=0;j<Rho.getRows();j++){
        	while(rho>(double)Rho[j]+0.5){
        		j++;
        	}
        		//cout<<j<<endl;
        		/*cout<<rho<<endl;
        		cout<<((double)Rho[j-1])<<endl;
        		getchar();*/
        		//getchar();
        		if(V.getVote(i,j)<1)
        		{V.setExt1(ip,i,j);
        		V.setExt2(ip,i,j);
        		V.increment(i,j);
        		//cout<<V.getVote(i,j)<<endl;
        		}
        		else
        		{        	//cout<<V.getVote(i,j)<<endl;
        			ip1=V.getExt2(i*(j));
        			if(vpImagePoint::distance(ip1,ip)<5){
                        V.setExt2(ip,i,j);
                        V.increment(i,j);
        			}
        			/*else {
        				V.setExt1(ip,i,j);
        				V.setExt2(ip,i,j);
        				V.setV(i,j);
        			}*/
        		}
        		/*if (V.getVote(i,j)>100){
        			cout<< i <<" "<<j<<endl;}*/
        	}

      }
}

void
apLineExtractor::setFeaturePoints(const vpImage<unsigned char> &I)
{
int h=I.getHeight();
int w=I.getWidth();
int n,m;
vpImagePoint point;
points.resize(0);
for (n=3;n<h;n++){
					for (m=0;m<w;m++){
	if (I[n][m]==255)
		{point.set_i((double)n);
	    point.set_j((double)m);
	    //cout<<n<<m<<endl;
		points.push_back(point);}
	}
	}
//points.print();

}

vpMatrix
apLineExtractor::getVMatrix()
{
	return V.getVoteMatrix();
}



