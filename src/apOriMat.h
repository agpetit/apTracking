#ifndef apOriMat_HH
#define apOriMat_HH

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

class VISP_EXPORT apOriMat
{

protected :
vpMatrix VoteMatrix;
std::vector< vpImagePoint > Ext1;
std::vector< vpImagePoint > Ext2;

public:
void init(const int r, const int c);

apDomOri();
apDomOri(const int r, const int c);

void resize(const int r, const int c);
void getDomOri(const int i, const int j);
void addDomOri(const int i, const int j);
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
}
;
#endif
