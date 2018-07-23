#ifndef apHoughVoteCircle_HH
#define apHoughVoteCircle_HH

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
#include <fstream>
#include <math.h>
#include <string.h>

#include <visp/vpException.h>
#include <visp/vpImageException.h>
#include <visp/vpIoTools.h>
#include <visp/vpParseArgv.h>
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpImagePoint.h>
#include <vector>

#include <iostream>

struct triple
{
 triple(): i1(0), j1(0), k1(0), vote(0)
  {
  }
  triple(int p1,
            int p2,
            int p3,
            int p4
            )
  {
    i1 = p1;
    j1 = p2;
    k1 = p3;
    vote = p4;
  }

  ~triple()
  {
  }
  int i1;
  int j1;
  int k1;
  int vote;
};

class apHoughVoteCircle
{

public :
vpMatrix VoteCenter;
std::vector<int> VoteRad;
std::vector< vpImagePoint > Ext1;
std::vector< vpImagePoint > Ext2;

public:
void init(const int r, const int c, const int d);

apHoughVoteCircle();
apHoughVoteCircle(const int r, const int c, const int d);

void increment(const int i, const int j, const int k);
void setV(const int i, const int j, const int k);
void setExt1(const vpImagePoint ip,const int i, const int j);
void setExt2(const vpImagePoint ip,const int i, const int j);
void buildLines();
void buildCircle();
vpMatrix getVoteCenter() const {return VoteCenter;}
std::vector<int> getVoteRadius() const {return VoteRad;}

//inline int sizeExt();
int getVRows();
int getVCols();
vpImagePoint  getExt1(const int i);
vpImagePoint  getExt2(const int i);
int  getVote(const int i,const int j, const int k);
std::vector<triple> getBestVotes(const int nbmax);
}
;
#endif
