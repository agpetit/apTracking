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


#include <iostream>
#include <fstream>
#include <time.h>
#include <kalman/vpSE3Kalman.h>

static std::ofstream state("state.txt");
 

int main(){

	std::ifstream f("celine2.dat");
	int iter, count=0;

	vpSE3Kalman filter;
	vpColVector Xv(6);
	vpMatrix Qv(6,6);
	vpColVector X(12);
	//f >> iter >> x >> y>> z>>beta>>gamma>>theta;
	f >> iter >> Xv[0] >> Xv[1] >> Xv[2] >> Xv[3] >> Xv[4] >> Xv[5];
	Qv.setIdentity();	
	
	std::cout << "Xv Qv" << std::endl;
	std::cout << Xv << std::endl;
	std::cout << Qv << std::endl;
	
	filter.setXv(Xv);
	filter.setQv(Qv);
	filter.init();

	Qv*=0.00001;

	srand ( time(NULL) );

	while(!f.eof()){
		//prediction
		filter.predict(1);

		//correction
	 	f >> iter >> Xv[0] >> Xv[1] >> Xv[2] >> Xv[3] >> Xv[4] >> Xv[5];
		#if KDEBUG
 		std::cout << "iter "<< iter << std::endl;
		#endif
// 		Xv[4]+=rand()/RAND_MAX*0.016-0.008);
// 		Xv[5]+=rand()/RAND_MAX*0.016-0.008);
		filter.setXv(Xv);
		filter.setQv(Qv);
		filter.visionUpdate();
		X=filter.getState();
		
		for(int i=0;i<12;i++)
		state<<X[i]<<"\t";
		state<<std::endl;
	}
}