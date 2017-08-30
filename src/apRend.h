/*
 * apRend.h
 *
 *  Created on: May 10, 2011
 *      Author: Antoine Petit
 */

#ifndef apRend_H
#define apRend_H

#include <visp/vpConfig.h>
#include <visp/vpMath.h>
#include <visp/vpMatrix.h>
#include <visp/vpImage.h>

class VISP_EXPORT apRend
{
public:
  double edgeR_th;
  double clipDist;
  int sampleR;
  int Normx;
  int Normy;
  int Normz;
  double scaleModel;
  bool useNPoints;
  int nPoints;


  apRend();
  apRend(const apRend &rend);
  virtual ~apRend() ;
  
  const apRend& operator=(const apRend &rend);

  void print( ) ;

  void setThreshold(double lambda) { edgeR_th = lambda ; }
  void setNorm(int a, int b, int c) { Normx =a  ; Normy = b; Normz=c; }
  void setClipDistance(double dist){clipDist = dist;}
  void setScaleModel(double scale){scaleModel = scale;}
  void setSampleStep(int a) { sampleR = a ; }

};


#endif


