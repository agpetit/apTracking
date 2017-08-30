#include "apHoughVoteCircle.h"

apHoughVoteCircle::apHoughVoteCircle()
{
}

void
apHoughVoteCircle::init(const int r, const int c, const int d)
{
VoteCenter.resize(r,c,true);
VoteRad.resize(d);
Ext1.resize(r*c);
Ext2.resize(r*c);

}


apHoughVoteCircle::apHoughVoteCircle(const int r, const int c, const int d)
{
	init(r,c,d);
}

void
apHoughVoteCircle::increment(const int i, const int j, const int k)
{

VoteCenter[i][j]++;
VoteRad[k]++;
}

void
apHoughVoteCircle::setExt1(const vpImagePoint ip, const int i, const int j)
{
Ext1[i*j]=ip;
}

void
apHoughVoteCircle::setExt2(const vpImagePoint ip, const int i, const int j)
{
Ext2[i*j]=ip;
}

void
apHoughVoteCircle::setV(const int i, const int j, const int k)
{
VoteCenter[i][j]=1;
VoteRad[k] = 1;
}

/*inline int apHoughVoteCircle::sizeExt()
{
	return Ext1.size();
}*/

int apHoughVoteCircle::getVRows()
{
	return VoteCenter.getRows();
}

int apHoughVoteCircle::getVCols()
{
	return VoteCenter.getCols();
}

vpImagePoint  apHoughVoteCircle::getExt1(const int i)
{
	return Ext1[i];
}

vpImagePoint  apHoughVoteCircle::getExt2(const int i)
{
	return Ext2[i];
}

int  apHoughVoteCircle::getVote(const int i,const int j, const int k)
{
	return VoteCenter[i][j];
}

std::vector<triple> apHoughVoteCircle::getBestVotes(const int nbmax)
{
	std::vector<triple> indices;
	triple cp;
	int imax,jmax,kmax;
for (int n =0;n<nbmax;n++)
{
		int votemax = 0;
		int votemaxrad = 0;
for (int i = 0; i< VoteCenter.getRows();i++)
{
	for (int j = 0; j<VoteCenter.getCols();j++)
	{
		if(VoteCenter[i][j]>votemax)
		{
	      votemax = VoteCenter[i][j];
	      for (int k =0; k < VoteRad.size();k++)
	      {
	    	  if(VoteRad[k] > votemaxrad)
	    	  {
	      votemaxrad = VoteRad[k];
          imax = i;
          jmax = j;
          kmax = k;
	    	  }
	      }
		}
	}
}
cp.i1 = imax;
cp.j1 = jmax;
cp.k1 = kmax;
cp.vote = VoteCenter[imax][jmax];
VoteCenter[imax][jmax] = 0;
VoteRad[kmax] = 0;
indices.push_back(cp);
}
return indices;
}





