/****************************************************************************
*
* $Id $
*
* This software was developed at:
* INRIA Rennes Bretagne Atlantique
* Equipe Projet Lagadic
* Campus Universitaire de Beaulieu
* 35042 Rennes Cedex
* http://www.irisa.fr/lagadic
*
* This file is part of the Aronnax project.
*
* Copyright (C) 2009 Inria. All rights reserved.
* The content of this file cannot be modified, transmitted,
* licensed, transferred, sold or distributed by any means 
* without permission of INRIA.
*
* Authors:
* Laneurit Jean
*
*****************************************************************************/


#include "vpSL3SE3Kalman.h"
#include "visp/vpGEMM.h"

vpSL3SE3Kalman::vpSL3SE3Kalman(){
  
  /** Homography Observation */
  _Xh.resize(9);
  _Qh.resize(9,9);
  _IQh.resize(9,9);
  _Ih.resize(9);
  _Hh.resize(9,12);
  _Kh.resize(12,9);
  
  /** Normalized Homography Observation */
  _Xhn.resize(8);
  _Qhn.resize(8,8);
  _IQhn.resize(8,8);
  _Ihn.resize(8);
  _Hhn.resize(8,12);
  _Khn.resize(12,8);
  
  
  /** d Xv / d Ou*/
  _dH_dOu.init(_Hh,0,3,9,3);
  /** d H / d P*/
  _dH_dP.init(_Hh,0,0,9,3);
  
  /** Intrinsic parameters */
  _KKK.resize(9,9);
  _K3.resize(3);
  
  /**  Hogmoraphy*/
  _Hm.resize(3,3);
  _H.resize(9);
  _Hn.resize(8);
  
  
  /** Normal vector */
  _N.resize(3);
  _NT.resize(3);
  
  /**buffer */
  _buffer11.resize(1,1);
  _buffer19.resize(9);
  _buffer93c.resize(9,3);  
  _buffer88.resize(8,8);
  _buffer129.resize(12,9);
  _buffer1212.resize(12,12);
  _eye333.resize(3,3);
  _eye333.setIdentity(3.0); 
  _eye89.resize(8,9);
  for(int i=0;i<8;i++)
    _eye89[i][i]=1;
  
  _Q.setIdentity();
}


void vpSL3SE3Kalman::ComputeAlpha(){
   _alpha=1/(_H[8]*_H[8]);
  #if KDEBUG
  std::cout << " alpha  "<< std::endl;
  std::cout << _alpha << std::endl;
  #endif 
}

/** compute d H / d Ou*/
void vpSL3SE3Kalman::ComputeDH_DOu(const bool &normalized ){

   if(normalized){
    _buffer13[0]=0;     
    _buffer13[1]=0;     
    _buffer13[2]=1; 

    vpMatrix::mult2Matrices(_KKK,_dROu_dOu,_buffer93a);
    vpMatrix::kron(_buffer13,_K3,_buffer19);
    vpMatrix::mult2Matrices(_H,_buffer19,_buffer99a);
    vpMatrix::mult2Matrices(_buffer99a,_dROu_dOu,_buffer93b);
    vpMatrix::add2WeightedMatrices(_buffer93a,_alpha,_buffer93b,-_alpha,_dH_dOu);
   }else{
    vpGEMM(_KKK,_dROu_dOu,1.0,null,0,_dH_dOu);// kron(K^(-T),K) (I + H*P'*N')*dROu/dOu
   }
  
//   #if KDEBUG
//   std::cout << "_dH_dOu" << std::endl;
//   std::cout << _dH_dOu << std::endl;
//   #endif
  
}

/** compute d H / d P*/
void vpSL3SE3Kalman::ComputeDH_DP(const bool &normalized ){

  if(normalized){
    _buffer13[0]=_K3[0]*_N[2];
    _buffer13[1]=_K3[1]*_N[2];
    _buffer13[2]=_K3[2]*_N[2];

    vpMatrix::kron(_N,_eye33,_buffer93a);
    vpMatrix::mult2Matrices(_KKK,_buffer93a,_buffer93b);   
    vpGEMM(_H,_buffer13,-_alpha,_buffer93b,_alpha,_dH_dP);
  }else{
    vpMatrix::kron(_N,_eye33,_buffer93a);
    vpGEMM(_KKK,_buffer93a,1.0,null,0,_dH_dP);
  }
  #if KDEBUG
  std::cout << "_dH_dP" << std::endl;
  std::cout << _dH_dP << std::endl;
  #endif
  
}

void vpSL3SE3Kalman::init(const vpThetaUVector Ou, const vpTranslationVector &t){
  _Q.setIdentity();
  for(int i=0;i<3;i++){
    _POu[i]=t[i];
    _POu[i+3]=Ou[i];
  }
   
  #if KDEBUG
  std::cout << "\n\n X  Q intialisation "<< std::endl;
  std::cout<<_X<< std::endl;
  std::cout<< _Q << std::endl;
  #endif 
}

void vpSL3SE3Kalman::homographyUpdate(){
  
  // Prepare relative matrices of vector Ou

  PrepareVec(_Ou);  
  //compute ROu = exp(skew(Ou))
  Compute_RVec(_ROu);  
  //Compute  d exp(skew(Ou)) d Ou
  Compute_dRVec_dVec(_dROu_dOu);  

  //Compute homography
  vpGEMM(_P,_NT,1.0,_ROu,1.0,_Hm);  
  vpGEMM(_KKK,_Hm.stackColumns(),1.0,null,1.0,_H);
   
  //Compute dH/dOu
  ComputeDH_DOu(false);  
  //Compute dP/dOu
  ComputeDH_DP(false);  
  
  
  #if KDEBUG
  std::cout << "\n\n H  Qh correction "<< std::endl;
  std::cout <<_Xh << std::endl;
  //   std::cout << _Qv << std::endl;
  #endif
  
  //Compute Inonvation  
  vpMatrix::sub2Matrices(_Xh,_H,_Ih);// innovation vector Xv-HvX
  vpGEMM(_Q,_Hh,1.0,null,0,_buffer129,VP_GEMM_B_T);
  vpGEMM(_Hh,_buffer129,1.0,_Qh,1.0,_IQh);//HvQHv' +Qv
  
  #if KDEBUG
  std::cout << "\n\n Innov "<< std::endl;
  std::cout << _Hh << std::endl;
  std::cout << _Ih << std::endl;
  std::cout << _IQh << std::endl;
  #endif
  
  //Compute Kalman gain
  _buffer99a= _IQh.inverseByLU(); //(HvQHv' +Qv)^(-1)
  vpGEMM(_Hh,_buffer99a,1.0,null,0,_buffer129,VP_GEMM_A_T); //Hv'(HvQHv' +Qv)^(-1)
  vpMatrix::mult2Matrices(_Q,_buffer129,_Kh);//Q Hv'(HvQHv' +Qv)^(-1)
  
  //Compute new state	
  vpMatrix::mult2Matrices(_Kh,_Ih,_Xtmp);//X=X+Kv(Xv-HvX)
  _X+=_Xtmp;
  
  #if KDEBUG
  std::cout << "\n\n  K "<< std::endl;
  std::cout << _Kh << std::endl; 
  #endif
  
  //Compute new covariance
  vpGEMM(_Kh,_Hh,-1.0,_eye1212,1.0,_buffer1212);
  vpMatrix::mult2Matrices(_buffer1212,_Q,_Qtmp);
  _Q=_Qtmp;
  
  
  #if KDEBUG
  std::cout << "\n\n X  Q update "<< std::endl;
  std::cout<<_X<< std::endl;
  std::cout<< _Q << std::endl;
  #endif
  
}


void vpSL3SE3Kalman::normHomographyUpdate(){
  // Prepare relative matrices of vector Ou
  PrepareVec(_Ou);  
  //compute ROu = exp(skew(Ou))
  Compute_RVec(_ROu);  
  //Compute  d exp(skew(Ou)) d Ou
  Compute_dRVec_dVec(_dROu_dOu);  
  //Compute homography
  vpGEMM(_P,_NT,1.0,_ROu,1.0,_Hm);
  vpGEMM(_KKK,_Hm.stackColumns(),1,null,1.0,_H);

  //Compute alpha
  ComputeAlpha();  
  //Compute dH/dOu
  ComputeDH_DOu(true);  
  //Compute dP/dOu
  ComputeDH_DP(true);  
  
  #if KDEBUG
  std::cout << "\n\n H  Qh correction "<< std::endl;
  std::cout <<_Xhn << std::endl;
  std::cout << _Qhn << std::endl;
  #endif
   
  vpGEMM(_eye89,_H,1/_H[8],null,0,_Hn);
  vpMatrix::mult2Matrices(_eye89,_Hh,_Hhn);
  vpMatrix::sub2Matrices(_Xhn,_Hn,_Ihn);// innovation vector Xv-HvX
  vpGEMM(_Q,_Hhn,1.0,null,0,_buffer128,VP_GEMM_B_T);
  vpGEMM(_Hhn,_buffer128,1.0,_Qhn,1.0,_IQhn);//HvQHv' +Qv
  
  #if KDEBUG
  std::cout << "\n\n Innov "<< std::endl;
  std::cout << _Hhn << std::endl;
  std::cout << _Ihn << std::endl;
  std::cout << _IQhn<< std::endl;
  #endif
  
  //Compute Kalman gain
  _buffer88= _IQhn.inverseByLU(); //(HvQHv' +Qv)^(-1)
  vpGEMM(_Hhn,_buffer88,1.0,null,0,_buffer128,VP_GEMM_A_T); //Hv'(HvQHv' +Qv)^(-1)
  vpMatrix::mult2Matrices(_Q,_buffer128,_Khn);//Q Hv'(HvQHv' +Qv)^(-1)
  
  //Compute new state	
  vpMatrix::mult2Matrices(_Khn,_Ihn,_Xtmp);//X=X+Kv(Xv-HvX)
  _X+=_Xtmp;
  
  #if KDEBUG
  std::cout << "\n\n  K "<< std::endl;
  std::cout << _Khn << std::endl; 
  #endif
  
  //Compute new covariance
  vpGEMM(_Khn,_Hhn,-1.0,_eye1212,1.0,_buffer1212);
  vpMatrix::mult2Matrices(_buffer1212,_Q,_Qtmp);
  _Q=_Qtmp;
  
  
  #if KDEBUG
  std::cout << "\n\n X  Q update "<< std::endl;
  std::cout<<_X<< std::endl;
  //   std::cout<< _Q << std::endl;
  #endif
  
}

void   vpSL3SE3Kalman::setIntrinsics(const vpMatrix &K){
  vpMatrix Kinv=K.inverseByLU();
  vpMatrix::kron(Kinv.t(),K,_KKK); 
  for(int i=0;i<3;i++)
    _K3[i]=_KKK[8][i*3+2];
}

void   vpSL3SE3Kalman::setNormal(const vpColVector &N){
  _N=N;
  _NT=N.transpose();
}
  
  void vpSL3SE3Kalman::setXh(const vpColVector &Xh){
    _Xh=Xh;
  }
  
  void vpSL3SE3Kalman::setQh(const vpMatrix &Qh){
    _Qh=Qh;
  }
  
  
    void vpSL3SE3Kalman::setXhn(const vpColVector &Xhn){
    _Xhn=Xhn;
  }
  
  void vpSL3SE3Kalman::setQhn(const vpMatrix &Qhn){
    _Qhn=Qhn;
  }