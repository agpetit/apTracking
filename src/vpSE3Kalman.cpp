#include "vpSE3Kalman.h"
#include <visp/vpGEMM.h>



vpSE3Kalman::vpSE3Kalman():epsilon(0.00001){
  _X.resize(12);
  _Q.resize(12,12);
  _Xtmp.resize(12);
  _Qtmp.resize(12,12);
  
  _P.init(_X,0,3);
  _Ou.init(_X,3,3);
  _V.init(_X,6,3);
  _W.init(_X,9,3);
  _POu.init(_X,0,6);
  _QPOu.init(_Q,0,0,6,6);
  
  
  _Xv.resize(9);
  _Ouv.init(_Xv,3,3);
  _Qv.resize(9,9);
  _IQv.resize(9,9);
  _Iv.resize(9);
  _Hv.resize(6,12);
  _Kv.resize(12,6);
  _D.resize(1,1);
  for(int i=0;i<6;i++)
      _Hv[i][i]=1;
  
  //Jacobian
  _Jx.resize(12,12);
  _Jx.setIdentity();
  _ROu.resize(3,3);
  _ROu.setIdentity();
  _RwDt.init(_Jx,0,0,3,3);
  _dRwDt_dwDt.resize(9,3);
  _SwDt.resize(3,3);
  _dSwDt_dwDt.resize(9,3);
  _dROu_dOu.resize(9,3);
  _SwDtDt.init(_Jx,0,6,3,3);
  _dP_dw.init(_Jx,0,9,3,3);
  _dOu_dw.init(_Jx,3,9,3,3);
  _dOu_dOu.init(_Jx,3,3,3,3);
    
  //Input data
  _Qu.resize(6,6);
  _Ju.init(_Jx,0,6,12,6);
  
  //Matrices needed to compute  _dRwDt_dwDt and _dSwDt_dwDt
  _Vec.resize(3); 
  _SkVec.resize(3,3); 
  _Sk2Vec.resize(3,3); 
  _SSkVec.resize(9); 
  _SSk2Vec.resize(9); 
  _dSSkVec_dVec.resize(9,3); 
  _dSSk2Vec_dVec.resize(9,3);
  
  _dSSkVec_dVec[5][0]=1;
  _dSSkVec_dVec[7][0]=-1;
  _dSSkVec_dVec[2][1]=-1;
  _dSSkVec_dVec[6][1]=1;
  _dSSkVec_dVec[1][2]=1;
  _dSSkVec_dVec[3][2]=-1;
  
  _A.resize(3);
  _dA_dSR.resize(3,9);
  _dCs_dSR.resize(9);
  _dPsi_dSR.resize(3,9);
  //_RwDtROu.resize(3,3);
  
  
  _dA_dSR[0][5]=1;
  _dA_dSR[0][7]=-1;
  _dA_dSR[1][2]=-1;
  _dA_dSR[1][6]=1;
  _dA_dSR[2][1]=1;
  _dA_dSR[2][3]=-1;
  
  
  _buffer31.resize(3);
  _buffer13.resize(3);
  _buffer39.resize(3,9);
  _buffer93a.resize(9,3);
  _buffer93b.resize(9,3);
  _buffer99a.resize(9,9);
  _buffer99b.resize(9,9);
  _buffer99c.resize(9,9);
  _buffer612.resize(6,12);
  _buffer126.resize(12,6);
  _buffer66.resize(6,6);
  _buffer61.resize(6);
  _buffer123.resize(12,3);
  _buffer33.resize(3,3);
  _buffer1212a.resize(12,12);
  _buffer1212b.resize(12,12);
  
  _eye1212.resize(12,12);
  _eye1212.setIdentity();
  _eye99.resize(9,9);
  _eye99.setIdentity();
  _eye33.resize(3,3);
  _eye33.setIdentity();
  
  setVarV(0.1);//Default value
  setVarW(0.1);//Default value
  //setVarV(0.5);//Default value
   //setVarW(0.1);
  
}




void vpSE3Kalman::ComputeQu(){
  
  for(int r=0;r<3;r++)
    _Qu[r][r]=_varv;
  
  for(int r=3;r<6;r++)
    _Qu[r][r]=_varw;
  
  #ifdef KDEBUG
  std::cout << "_Qu"<<std::endl;
  std::cout << _Qu<<std::endl;
  #endif
  
}

void vpSE3Kalman::PrepareVec(const vpColVector & V, const double & dt){
  
  _Vec=V*dt;
  norm =_Vec.euclideanNorm ();
  
  if(norm>epsilon){
    norm2=norm*norm;
    norm3=norm*norm*norm;
    norm4=norm*norm*norm*norm;
    
    alpha1=sin(norm)/norm;
    alpha2=(1-cos(norm))/norm2;
    alpha3=(norm-sin(norm))/norm3;
    
    alpha4=(norm *cos(norm) -sin(norm))/norm2;
    alpha5=(sin(norm)*norm-2+2*cos(norm))/norm3;
    alpha6=-(2*norm+cos(norm)*norm-3*sin(norm))/norm4;
  }else{
    // 	    std::cout << "coucou" << std::endl;
    // 	    exit(0); ???????
  }
  
  //skew(V)
  _SkVec=vpColVector::skew(_Vec);
  //skew^2(V)
  vpMatrix::mult2Matrices(_SkVec,_SkVec,_Sk2Vec);	
  
  //compute d skew^2(V) /dV
  // transpose of skew(V) 
  _SkVec.transpose(_buffer33);
  //Compute _SkV^T kron I33 
  vpMatrix::kron(_buffer33,_eye33,_buffer99a);
  //Compute I33 kron _SkV 
  vpMatrix::kron(_eye33,_SkVec,_buffer99b);
  //compute _SkV^T kron I33 + I33 kron _SkV 
  vpMatrix::add2Matrices(_buffer99a,_buffer99b,_buffer99c);
  //compute (_SkV^T kron I33 + I33 kron _SkV) _dSkV_dV
  vpMatrix::mult2Matrices(_buffer99c,_dSSkVec_dVec,_dSSk2Vec_dVec);
  
  // stack(skew(V)) 	
  _SkVec.stackColumns(_SSkVec);
  //stack(skew^2(V))
  _Sk2Vec.stackColumns(_SSk2Vec);
  
}

void vpSE3Kalman::Compute_RVec(vpMatrix &out){
  
  if(norm>epsilon){	
    vpMatrix::add2WeightedMatrices(_Sk2Vec,alpha2,_SkVec,alpha1,_buffer33);
    vpMatrix::add2Matrices(_eye33,_buffer33,out);
  }else{
    out.setIdentity();;
  }
  
  
}

void vpSE3Kalman::Compute_dRVec_dVec(vpMatrix &out){
  
  if(norm>epsilon){
    vpGEMM(_SSkVec,_Vec,alpha4/norm,_dSSkVec_dVec,alpha1,_buffer93a,VP_GEMM_B_T);
    vpGEMM(_SSk2Vec,_Vec,alpha5/norm,_dSSk2Vec_dVec,alpha2,_buffer93b,VP_GEMM_B_T);
    vpMatrix::add2Matrices(_buffer93a,_buffer93b,out);
  }else{
    out=_dSSkVec_dVec;
  }
}


void vpSE3Kalman::Compute_SVec(vpMatrix &out){
  
  //Compute SwDt
  if(norm>epsilon){	
    vpMatrix::add2WeightedMatrices(_Sk2Vec,alpha3,_SkVec,alpha2,_buffer33);
    vpMatrix::add2Matrices(_eye33,_buffer33,out);
  }else{
    out.setIdentity();
  }
}


void vpSE3Kalman::Compute_dSVec_dVec(vpMatrix &out){
  
  if(norm>epsilon){
    //compute dSV /d V
    vpGEMM(_SSkVec,_Vec,alpha5/norm,_dSSkVec_dVec,alpha2,_buffer93a,VP_GEMM_B_T);
    vpGEMM(_SSk2Vec,_Vec,alpha6/norm,_dSSk2Vec_dVec,alpha3,_buffer93b,VP_GEMM_B_T);
    vpMatrix::add2Matrices(_buffer93a,_buffer93b,out);
    
  }else{
    out=_dSSkVec_dVec;
  }
}


void vpSE3Kalman::Compute_dP_dw(const double & dt){
  std::cout << _P << std::endl;
  _P.transpose(_buffer13); 
  vpMatrix::kron(_buffer13,_eye33,_buffer39);
  std::cout << _buffer39 << std::endl;
  vpGEMM(_buffer39,_dRwDt_dwDt,dt,null,0,_buffer33);
  vpGEMM(_eye33,_V,dt,null,0,_buffer31);
  _buffer31.transpose(_buffer13);
  vpMatrix::kron(_buffer13,_eye33,_buffer39);
  vpGEMM(_buffer39,_dSwDt_dwDt,dt,_buffer33,1.0,_dP_dw);
  
}

void vpSE3Kalman::Compute_dOu_dOu(){
  vpMatrix::kron(_eye33,_RwDt,_buffer99a);
  vpMatrix::mult2Matrices(_buffer99a,_dROu_dOu,_buffer93a);
  vpMatrix::mult2Matrices(_dPsi_dSR,_buffer93a,_dOu_dOu);
}

void vpSE3Kalman::Compute_dOu_dw(const double &dt){
  
  _ROu.transpose(_buffer33);
  vpMatrix::kron(_buffer33,_eye33,_buffer99a);
  vpGEMM(_buffer99a,_dRwDt_dwDt,dt,null,0,_buffer93a);
  vpMatrix::mult2Matrices(_dPsi_dSR,_buffer93a,_dOu_dw);
}

void vpSE3Kalman::Compute_dPsi_dR(const vpMatrix &R){
  
 	double trace = R[0][0] + R[1][1] + R[2][2];
	double rnorm = acos((trace -1.0)/2.0);
	double beta1, beta2,beta3;

	if(rnorm>epsilon){
		beta1=0.5*(sin(rnorm)-rnorm*cos(rnorm))/pow(sin(rnorm),2);
		beta2=-1/sqrt(3-trace*trace+2*trace);
		beta3=rnorm/(2* sin(rnorm));
		
 		if ((3-trace*trace+2*trace)<0){
		  std::cout << _RwDt << std::endl;
		  std::cout << _ROu << std::endl;
		  std::cout << R << std::endl;
		  std::cout << trace << " "<<rnorm << " "<<beta1<<" "<< beta2 << " "<< beta3<< std::endl;
		  //exit(0);
		}
// //             throw(ctException(ctException::negativeRoot,   "(3-trace*trace+2*trace)<0")) ;
		
	}else{
		beta1=0;
		beta2=-0.5/sqrt(3.0);
		beta3=1/2.0;
	}
   

	// A = [R(3,2) - R(2,3) , R(1,3)-R(3,1) , R(2,1) -R(2,1)]^T
	_A[0] = R[2][1]-R[1][2];
	_A[1] = R[0][2]-R[2][0];
	_A[2] = R[1][0]-R[0][1];

	// d (rnorm /2*sin(rnorm)) / dR
	_dCs_dSR[0]=beta1*beta2;
	_dCs_dSR[4]=beta1*beta2;
	_dCs_dSR[8]=beta1*beta2;

	// compute dPsi dR
	vpGEMM(_A,_dCs_dSR,1.0,_dA_dSR,beta3,_dPsi_dSR);
	
/*	#ifdef KDEBUG
	std::cout << "dPsi/dR" << std::endl;
	std::cout << "trace" << " rnorm" << " beta1 "<< " beta2 " << " beta3" << std::endl; 
  	std::cout << trace << " "<<rnorm << " "<<beta1<<" "<< beta2 << " "<< beta3<< std::endl; 
	std::cout << "A" <<std::endl;
	std::cout << _A <<std::endl;
	std::cout<<"_dCs_dSR"<<std::endl;
	std::cout<<_dCs_dSR<<std::endl;
	std::cout<<"_dPsi_dSR"<<std::endl;
	std::cout<<_dPsi_dSR<<std::endl;
	#endif*/
 
}


void vpSE3Kalman::predict(const double &dt){
  
  
  //Compute Qu
  //ComputeQu();
  
  // Jacobian computation
  // Prepare relative matrices of vector W*dt
  PrepareVec(_W,dt);
  //compute RwDt
  Compute_RVec(_RwDt);
  //Compute SwDt *Dt
  Compute_SVec(_SwDt);
  _SwDtDt=_SwDt*dt;
  //Compute d exp(skew(w*dt)) d wdt
  Compute_dRVec_dVec(_dRwDt_dwDt);
  //Compute d S(wdt)/ d wdt
  Compute_dSVec_dVec(_dSwDt_dwDt);
  
//     #if KDEBUG
//     std::cout << " W " <<std::endl;
//     std::cout << _W << std::endl;
//     std::cout << " V " <<std::endl;
//     std::cout << _V << std::endl;
//     std::cout << " RwDt " <<std::endl;
//     std::cout << _RwDt<<std::endl;
//     std::cout << " SwDt *Dt " <<std::endl;
//     std::cout <<_SwDt<<std::endl;
//     std::cout << " _RwDt_dwDt " <<std::endl;
//     std::cout <<_dRwDt_dwDt<<std::endl;
//     std::cout << " _dSwDt_dwDt " <<std::endl;
//     std::cout << _dSwDt_dwDt << std::endl;
//     #endif 
//   
  // Prepare relative matrices of vector Ou
  PrepareVec(_Ou);
  //compute ROu = exp(skew(Ou))
  Compute_RVec(_ROu);
  //Compute  d exp(skew(Ou)) d Ou
  Compute_dRVec_dVec(_dROu_dOu);
  
//     #if KDEBUG
//    	std::cout << " Ou " <<std::endl;
//    	std::cout << _Ou<< std::endl;
//    	std::cout << " Rou " <<std::endl;
//    	std::cout << _ROu << std::endl;
//    	std::cout << " _dROu_dOu " <<std::endl;
//    	std::cout << _dROu_dOu << std::endl;
//     #endif
  
  
  // compute dP/dt
  Compute_dP_dw(dt);
  // compute d Psi(R) / d stack(R)
  vpMatrix::mult2Matrices(_RwDt,_ROu,_RwDtROu);
  Compute_dPsi_dR(_RwDtROu);
  //compute dOu_dOu
  Compute_dOu_dOu();
  //compute dOu_dOu
  Compute_dOu_dw(dt);
  
//     #if KDEBUG
//    	std::cout << " dP/dw " <<std::endl;
//    	std::cout << _dP_dw << std::endl;
//    	std::cout << " dPsi(R)/dSR " <<std::endl;
//    	std::cout <<_dPsi_dSR<< std::endl;
//    	std::cout << " dOu/dOu " <<std::endl;
//    	std::cout <<_dOu_dOu<< std::endl;
//    	std::cout << " dOu/dw " <<std::endl;
//    	std::cout <<_dOu_dw<< std::endl;
//    	std::cout << " Jx " <<std::endl;
//    	std::cout <<_Jx<< std::endl;
//     #endif
  
  //State prediction
  //predict position
  vpMatrix::mult2Matrices(_RwDt,_P,_buffer31);
  vpGEMM(_SwDt,_V,dt,_buffer31,1.0,_P);
  //predict orientation
  _Ou= vpThetaUVector(_RwDtROu);
  
  //Compute new covariance matrix
  vpGEMM(_Qu,_Ju,1.0,null,0,_buffer126,VP_GEMM_B_T); // QuJu'
  vpMatrix::mult2Matrices(_Ju,_buffer126,_buffer1212a); // JuQuJu'
  vpGEMM(_Q,_Jx,1.0,null,0,_buffer1212b,VP_GEMM_B_T); //QJx'
  vpGEMM(_Jx,_buffer1212b,1.0,_buffer1212a,1.0,_Q); //JxQJx' + Qu
  
  
  #if KDEBUG
  std::cout << "\n\n X  Q prediction "<< std::endl;

  //std::cout <<_X << std::endl;
  //std::cout <<_Q<< std::endl;
  //std::cout <<_Qu << std::endl;
  //std::cout <<_Qv<< std::endl;

  #endif 
}


void vpSE3Kalman::init(){
  
  _Q.setIdentity();
  _POu=_Xv;
  _QPOu=_Qv;
  
  
  #if KDEBUG
  std::cout << "\n\n X  Q intialisation "<< std::endl;
  std::cout<<_X<< std::endl;
  std::cout<< _Q << std::endl;
  #endif
}

bool vpSE3Kalman::visionMahaUpdate(){
  
  // Mahalanobis distance
  _buffer66=_IQv.inverseByLU();
  vpMatrix::mult2Matrices(_buffer66,_Iv,_buffer61);
  vpGEMM(_Iv,_buffer61,1.0,null,0,_D,VP_GEMM_A_T);

//       #if KDEBUG
//     std::cout << "\n\n Innov "<< std::endl;
//     std::cout << "_Iv" << std::endl;
//     std::cout << _Iv << std::endl;
//     std::cout << "_IQv" << std::endl;
//     std::cout << _IQv << std::endl;
//     std::cout << "Maha  distance" << std::endl;
//     std::cout << sqrt(_D[0][0]) << std::endl;
//     #endif

  //Update
  if(sqrt(_D[0][0])<3){  
    
    //Compute Kalman gain
    vpGEMM(_Hv,_buffer66,1.0,null,0,_buffer126,VP_GEMM_A_T); //Hv'(HvQHv' +Qv)^(-1)
    vpMatrix::mult2Matrices(_Q,_buffer126,_Kv);//Q Hv'(HvQHv' +Qv)^(-1)

/*    #if KDEBUG
    std::cout << "\n\n X  K correction "<< std::endl;
    std::cout << _Kv << std::endl;
    #endif*/
    
    //Compute new state
    vpMatrix::mult2Matrices(_Kv,_Iv,_Xtmp);//X=X+Kv(Xv-HvX)
    _X+=_Xtmp;
    
    //Compute new covariance
    vpGEMM(_Kv,_Hv,-1.0,_eye1212,1.0,_buffer1212a);
    vpMatrix::mult2Matrices(_buffer1212a,_Q,_Qtmp);
    _Q=_Qtmp;
        
    return true;
  } 
  return false;
}

void vpSE3Kalman::visionUpdate(){
  
  #if MYDEBUG
  std::cout << "\n\n Xv  Qv correction "<< std::endl;
  std::cout <<_Xv << std::endl;
  std::cout << _Qv << std::endl;
  #endif
  
  //Innovation
  vpMatrix::sub2Matrices(_Xv,_POu,_Iv);// innovation vector Xv-HvX
  vpMatrix::add2Matrices(_QPOu,_Qv,_IQv);//HvQHv' +Qv
  
  if(visionMahaUpdate()){ 
    //update
  }else{	
    //change ThetaU representation for Theta [0, pi[
    double theta=sqrt(_Ouv.sumSquare());
   // Ou2O_U(_Ouv,theta,_buffer31);
    theta = M_PI+(M_PI-theta); // pi +(pi-theta)
    vpGEMM(_eye33,_Ouv/theta,-theta,null,0,_Ouv);  // -(pi +(pi-theta)) *U	
    vpMatrix::sub2Matrices(_Xv,_POu,_Iv);// recompute innovation vector Xv-HvX
    visionMahaUpdate();//update
  }
  
  #if KDEBUG
  std::cout << "\n\n X  Q correction "<< std::endl;
  std::cout << _X << std::endl;
  std::cout << _Q << std::endl;
  #endif
}
