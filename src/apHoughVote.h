#ifndef apHoughVote_HH
#define apHoughVote_HH

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

struct couple
{
 couple(): i1(0), j1(0), vote(0)
  {
  }
  couple(int p1,
            int p2,
            int p3
            )
  {
    i1 = p1;
    j1 = p2;
    vote = p3;
  }

  ~couple()
  {
  }
  int i1;
  int j1;
  int vote;
};

class apHoughVote
{

public :
vpMatrix VoteMatrix;
std::vector< vpImagePoint > Ext1;
std::vector< vpImagePoint > Ext2;

public:
void init(const int r, const int c);

apHoughVote();
apHoughVote(const int r, const int c);

void increment(const int i, const int j);
void setV(const int i, const int j);
void setExt1(const vpImagePoint ip,const int i, const int j);
void setExt2(const vpImagePoint ip,const int i, const int j);
void buildLines();
vpMatrix getVoteMatrix() const {return VoteMatrix;}
//inline int sizeExt();
int getVRows();
int getVCols();
vpImagePoint  getExt1(const int i);
vpImagePoint  getExt2(const int i);
int  getVote(const int i,const int j);
std::vector<couple> getBestVotes(const int nbmax);
}
;
#endif
