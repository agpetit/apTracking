/*
 * apPartDescriptor.cpp
 *
 *  Created on: Dec 19, 2011
 *      Author: agpetit
 */
#include "apPartDescriptor.h"




void
apPartDescriptor::init(int w, int h, int nr, int nw)
{
width=w;
height=h;
xcoord.resize(1);
ycoord.resize(1);
nR=nr;
nW=nw;
//Histogram.resize()
}

void
apPartDescriptor::buildFrom(vpImage<unsigned char> &EOMap, vpImagePoint IO)
{
for (int k=0;k<width;k++)
{
	for(int l=0;l<height;l++)
	{
		if (EOMap[IO.get_i()-(int)height/2+l][IO.get_j()-(int)width/2+k]<100 || EOMap[IO.get_i()-(int)height/2+l][IO.get_j()-(int)width/2+k]>100)
		{
xcoord.push_back(IO.get_j()-(int)width/2+k);
ycoord.push_back(IO.get_i()-(int)height/2+l);
		}
	}
}
//Histogram = Hist.Run(xcoord,ycoord,nR,nW);


}

