#include "apHoughVote.h"

apHoughVote::apHoughVote()
{
}

void
apHoughVote::init(const int r, const int c)
{
VoteMatrix.resize(r,c,true);
Ext1.resize(r*c);
Ext2.resize(r*c);

}


apHoughVote::apHoughVote(const int r, const int c)
{
	init(r,c);
}

void
apHoughVote::increment(const int i, const int j)
{
int vote=(int)VoteMatrix[i][j];
VoteMatrix[i][j]=vote+1;
}

void
apHoughVote::setExt1(const vpImagePoint ip, const int i, const int j)
{
Ext1[i*j]=ip;
}

void
apHoughVote::setExt2(const vpImagePoint ip, const int i, const int j)
{
Ext2[i*j]=ip;
}

void
apHoughVote::setV(const int i, const int j)
{
VoteMatrix[i][j]=1;
}

/*inline int apHoughVote::sizeExt()
{
	return Ext1.size();
}*/

int apHoughVote::getVRows()
{
	return VoteMatrix.getRows();
}

int apHoughVote::getVCols()
{
	return VoteMatrix.getCols();
}

vpImagePoint  apHoughVote::getExt1(const int i)
{
	return Ext1[i];
}

vpImagePoint  apHoughVote::getExt2(const int i)
{
	return Ext2[i];
}

int  apHoughVote::getVote(const int i,const int j)
{
	return VoteMatrix[i][j];
}

std::vector<couple> apHoughVote::getBestVotes(const int nbmax)
{
	std::vector<couple> indices;
	couple cp;
	int imax,jmax;
for (int n =0;n<nbmax;n++)
{
		int votemax = 0;
for (int i = 0; i< VoteMatrix.getRows();i++)
{
	for (int j = 0; j<VoteMatrix.getCols();j++)
	{
		if(VoteMatrix[i][j]>votemax)
		{
          votemax = VoteMatrix[i][j];
          imax = i;
          jmax = j;
		}
	}
}
cp.i1 = imax;
cp.j1 = jmax;
cp.vote = VoteMatrix[imax][jmax];
VoteMatrix[imax][jmax] = 0;
indices.push_back(cp);
}
return indices;
}





