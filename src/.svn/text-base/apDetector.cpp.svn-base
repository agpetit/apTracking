/*
 * apDetector.cpp
 *
 *  Created on: Mar 2, 2012
 *      Author: agpetit
 */

#include "apDetector.h"
#include <visp/vpDisplayX.h>
#include <omp.h>

apDetector::apDetector() {
	// TODO Auto-generated constructor stub

}

apDetector::~apDetector() {
	// TODO Auto-generated destructor stub
	vpImage<unsigned char>* Iview;
	std::vector<vpImage<unsigned char>*>* Hview;
	std::vector<vpImage<unsigned char>*> Hview_;
	for(int k = 1; k<Hviews.size();k++)
	{
		Hview = Hviews[k];
		Hview_ = *Hviews[k];
		for(int l = 1; l < Hview_.size(); l++)
		{
			Iview = Hview_[l];
			delete Iview;
		}
		delete Hview;
	}
	Hviews.clear();
	apPFilter* filter;
	for (int k = 0; k<filters.size()-1;k++)
	{
		filter = filters[k+1];
		delete filter;
	}
}

void apDetector::init(apDetection &_detect, std::string _object){
	Hviews.resize(1);
	filters.resize(1);
	detection = _detect;
	object = _object;
}

void apDetector::setStartingLevel()
{
vpImage<unsigned char> I;
int level;
int numberofLevels;
int i=hierarchy.getRows()-1;
int j;
level = hierarchy[i][0];
startingLevel = level;
numberofLevels = level+1;
std::cout << "Hierarchical view graph loaded - Number of levels " << numberofLevels << std::endl;
while(level>0)
{
j=1;
while(hierarchy[i-j+1][0]==level){
j++;
if(i-j+1==-1)
{
	break;
}
}
std::cout << "Level " << level << " - Number of model views " << j-1 << std::endl;
level--;
i-=hierarchy[i][1];
}
std::cout << "Enter starting level of the hierarchical view graph" << std::endl;
std::cin >> startingLevel;
if(startingLevel>numberofLevels)
	std::cout << "!> bad input" << std::endl;
}

void apDetector::loadViews(std::string viewspath)
{

	// File where the graph is stored
std::string hf = "h" + object + ".txt";
char *filenameh = (char *)hf.c_str();
// File to store the data (pose...) of each view at the first level
std::string data0f = "data" + object + "0.txt";
char *filenamed0 = (char *)data0f.c_str();
//File to store the data (pose...) of the views at each level of the hierarchical view graph
std::string data1f = "data" + object + "1.txt";
char *filenamed1 = (char *)data1f.c_str();


	hierarchy.loadMatrix(filenameh,hierarchy,false);
	dataViews0.loadMatrix(filenamed0,dataViews0,false);
	dataViews1.loadMatrix(filenamed1,dataViews1,false);
	int i=hierarchy.getRows()-1;
	int numberofLevels = hierarchy[i][0]+1;
	setStartingLevel();
	int level = numberofLevels-1;
	int diff = numberofLevels-startingLevel;
	/*int level = hierarchy[i][0];
	std::cout << " Level " << level << std::endl;
	level--;
	i=i-hierarchy[i][1];*/
	while(diff>1){i=i-hierarchy[i][1];
	level--;
	diff--;}
	vpImage<unsigned char> IT;
	int ind;
	char buf[FILENAME_MAX];
	std::vector<vpImage<unsigned char>*>* views;
	vpImage<unsigned char> *I00;
    while(level>0)
	{
    int j=1;
	views = new std::vector<vpImage<unsigned char>*>;
	views->resize(1);
	while(hierarchy[i-j+1][0]==level){
	sprintf(buf, viewspath.c_str(),level*10000+j-1);
	std::string filename(buf);
	vpImageIo::readPNG(IT, filename);
	I00 = new vpImage<unsigned char>;
	I00->resize(IT.getHeight(),IT.getWidth());
	*I00=IT;
	views->push_back(I00);
	j++;
	//delete I00;
	if(i-j+1==-1)
	{
		break;
	}
	}
	std::cout << " Level " << level << " : " << j-1 << " views loaded " << std::endl;
	Hviews.push_back(views);
	level--;
	i=i-hierarchy[i][1];
	}
    views = new std::vector<vpImage<unsigned char>*>;
    views->resize(1);
    for (int ii=0;ii<dataViews0.getRows();ii++)
    {
    	I00 = new vpImage<unsigned char>;
    	sprintf(buf, viewspath.c_str(),ii);
    	std::string filename1(buf);
    	vpImageIo::readPNG(IT, filename1);
    	*I00=IT;
    	views->push_back(I00);
    }
	std::cout << " Level " << level << " : " << dataViews0.getRows() << " views loaded " << std::endl;
	std::cout << " Ok views loaded " << std::endl;
    Hviews.push_back(views);
}

void apDetector::setFilters(std::string hpath, vpCameraParameters &cam)
{
	vpImage<unsigned char> ModelView;
	int i=hierarchy.getRows()-1;
	int numberofLevels = hierarchy[i][0]+1;
	int level = numberofLevels-1;
	int diff = numberofLevels-startingLevel;
	while(diff>1){i=i-hierarchy[i][1];
	level--;
	diff--;}
	/*int i=hierarchy.getRows()-1;
	int level;
		level = hierarchy[i][0];
		level--;
		level--;
		i=i-hierarchy[i][1];
		i=i-hierarchy[i][1];*/
		int j=1;
		int ind=0;
		int m;
		std::vector<int> underT;
		double oriT;
		int surfaceT;
		double Ang;
		vpImagePoint cogT;
		int nbt = hierarchy[i][1];
		apPFilter *filter;
		vpPoseVector pose;
	while(hierarchy[i-j+1][0]==level){
		char buf[FILENAME_MAX];
		sprintf(buf, hpath.c_str(),level*10000+j-1);
		std::string filename(buf);
		vpImageIo::readPNG(ModelView, filename);
		oriT = dataViews1[i+j-nbt][2];
		surfaceT = dataViews1[i+j-nbt][3];
		cogT.set_i(dataViews1[i+j-nbt][0]);
		cogT.set_j(dataViews1[i+j-nbt][1]);
		pose[0]=dataViews1[i+j-nbt][4];
		pose[1]=dataViews1[i+j-nbt][5];
		pose[2]=dataViews1[i+j-nbt][6];
		pose[3]=dataViews1[i+j-nbt][7];
		pose[4]=dataViews1[i+j-nbt][8];
		pose[5]=dataViews1[i+j-nbt][9];

		filter = new apPFilter;
		filter->setCameraParameters(cam);
		filter->setDetectionParameters(detection);
		filter->setModel(ModelView,cogT,surfaceT,oriT,pose);
		filter->setStructure(hierarchy,dataViews0,dataViews1,Hviews);
		std::cout <<"Filter " << j-1 << " set "<< "- COG :" << cogT << " - Orientation : " << oriT << " - Area : " << surfaceT << std::endl;
		filters.push_back(filter);
		j++;
		if (i-j+1 ==0)
		{
			break;
		}
		//delete filter;
	}
}

void apDetector::detect(std::string ipath, std::string isegpath, std::string hpath, std::string scpath, vpCameraParameters &cam, int fframe, double thld, const SearchStrategy &searchStrategy, const PFEstimateType &PFEType)
{
	vpImage<unsigned char> Iseg;
	vpImage<vpRGBa> IsegCol;
	vpImage<unsigned char> IsegEdge;
	vpImage<unsigned char> Iseg0;
	vpImage<unsigned char> I;
	vpImage<unsigned char> Iin;
	vpImage<unsigned char> Igrad;
	vpImage<vpRGBa> Igrey;
	vpImage<vpRGBa> Icol;
	vpDisplayX display;
	vpDisplayX displaySeg;
	vpVideoReader reader;
	vpVideoReader readerSeg;
	vpVideoReader readerCol;
	reader.setFileName(ipath.c_str());
	readerSeg.setFirstFrameIndex(fframe);
	readerSeg.setFileName(isegpath.c_str());
	readerCol.setFirstFrameIndex(fframe);
	readerCol.setFileName(ipath.c_str());
	reader.setFirstFrameIndex(fframe);
	reader.open(I);
	reader.acquire(I);
	readerCol.open(Icol);
	readerCol.acquire(Icol);
	I.resize(I.getHeight(),I.getWidth());
	Igrey.resize(I.getHeight(),I.getWidth());
	Iseg.resize(I.getHeight(),I.getWidth());
	IsegCol.resize(I.getHeight(),I.getWidth());
	Iseg0.resize(I.getHeight(),I.getWidth());
	Igrad.resize(I.getHeight(),I.getWidth());
	display.init(I,100,100,"Detection");
	displaySeg.init(Iseg,100,800,"Segmentation");

	std::string opathSeg = "outSeg"+object+"/Iseg%06d.png";
	std::string opathI = "outSeg"+object+"/I%06d.png";
	std::string opathIclust = "outSeg"+object+"/Iclust%06d.png";



	segment.init(Icol);
	int im = segment.segParam.startFrame+2000;
	int ll = -1;
	int kseg = 0;

    char bufSeg[FILENAME_MAX];
    char bufI[FILENAME_MAX];
    char bufIclust[FILENAME_MAX];
	segment.init(Icol);

	for (int k = 0; k < im; k++)
	{
		Iin.resize(I.getHeight(),I.getWidth());
		I.resize(I.getHeight(),I.getWidth());
		reader.acquire(I);
		Iin = I;
		readerCol.acquire(Icol);
		/*for (int i = 0; i<I.getHeight(); i++)
			for (int j = 0; j<I.getWidth() ; j++)
		{
			Igrey[i][j].R  = 0.2126*Icol[i][j].R  + 0.7152 * Icol[i][j].G  + 0.0722 * Icol[i][j].B;
			Igrey[i][j].G  = 0.2126*Icol[i][j].R + 0.7152 * Icol[i][j].G  + 0.0722 * Icol[i][j].B;
			Igrey[i][j].B  = 0.2126*Icol[i][j].R  + 0.7152 * Icol[i][j].G  + 0.0722 * Icol[i][j].B;
			Igrey[i][j].A  = 0.2126*Icol[i][j].R  + 0.7152 * Icol[i][j].G  + 0.0722 * Icol[i][j].B;
		}*/
		//vpDisplay::display(Iseg);
		//readerSeg.open(Iseg);
		if(k%40==0)
		{
			segment.display.close(segment.I_uchar);
			segment.clear();
			segment.init(Icol);
			ll++;
		}

		segment.segmentFgdBgd(Icol,IsegCol,k-ll*40);
		vpImageConvert::convert(IsegCol,Iseg);
		vpDisplay::flush(Iseg);
		vpDisplay::display(Iseg);

		if (k-ll*40>=segment.segParam.startFrame)
		{
	    sprintf(bufSeg, opathSeg.c_str(), kseg);
	    std::string filenameSeg(bufSeg);
	    //std::cout << "Write: " << filename4 << std::endl;
	    //vpImageIo::write(IsegCol, filenameSeg);
	    sprintf(bufI, opathI.c_str(), kseg);
	    std::string filenameI(bufI);
	    //std::cout << "Write: " << filename4 << std::endl;
	    //vpImageIo::write(Icol, filenameI);
	    sprintf(bufIclust, opathIclust.c_str(), kseg);
	    std::string filenameIclust(bufIclust);
	    //std::cout << "Write: " << filename4 << std::endl;
	    //vpImageIo::write(segment.Iclust, filenameIclust);
	    kseg++;
		}
	}

	//std::vector<apPFilter*> fltrs;
	apPFilter *filter;
	ctParticle pInit;
    ctParticle prclB;
	int filt = 0;
	int filt_max = 0;
	int filt_Max;
	double Proba_Max=0;
	double wght_f;
	double wght_f_max = 0;
    apViews gen;
    gen.initDetection(detection);
    vpImage<vpRGBa> *dTOI;
    vpImage<vpRGBa> *dTOIseg;
	pInit.buildFrom(Iseg);
	std::vector<double> logPolarHistIseg;
	vpImagePoint cogIseg;
	double angleIseg;
	int surfaceIseg;
	/*int nr = 40;
	int nw = 80;*/
	/*int sample_x = 20;
	int sample_y = 20;
	int sample_theta = 1;*/
	while(filt<filters.size()-1){
	filter=filters[filt+1];
	filter->init(detection.nbParticles,pInit,ctPFilter::GSTEP);
	switch(detection.similarityMs){
		case apDetection::SHAPE_CONTEXT:
	filter->initSC();
    filter->computeSC(scpath, filt+1);
	std::cout << " ComputeSC filt " << filt  << std::endl;
	break;
	}
	filter->setProba((double)1/(filters.size()-1));
	filt++;
	}
	vpColVector measures;

	fr = 8;
	double t1,t0;
	int kk = 0;
	int niseg = 0;
	int nis;
	//Iseg0 = Iseg;

	while (kk<detection.nbimax)
		{

		//readerSeg.acquire(Iseg);
		im++;
		kk++;
		vpDisplay::display(I);
	    double t5= vpTime::measureTimeMs();
		switch(detection.similarityMs){
		case apDetection::SHAPE_CONTEXT:
		logPolarHistIseg = gen.computeSC(Iseg,IsegEdge,cogIseg,angleIseg,surfaceIseg,detection.nr,detection.nw);
		break;
		case apDetection::ORIENTED_CHAMFER:
		case apDetection::STEGER:
		 {

				for (int i=0;i<I.getHeight()-0;i++)
								{
									for (int j=0;j<I.getWidth()-0;j++)
									{
										if(i>3 && i < I.getHeight()-3 && j>3 && j < I.getWidth()-3){
										double a=(apImageFilter::sobelFilterX(Iin,i,j));
										double b=(apImageFilter::sobelFilterY(Iin,i,j));
										if(a*a + b*b > 300)
										{
										 Igrad[i][j]=255*(-(atan(a/b))/M_PI+1/2);
										}
										else {Igrad[i][j] = 100;}
									}
									}
							}

				//vpImageIo::writePNG(Igrad, "Igrad1.png");
			 /*if(kk == 1)
	    dTOIseg = gen.dTO(I);
			 else*/
		//dTOIseg = gen.dtOSeg2(Iseg);
		dTOIseg = gen.dtOSeg3(Iseg,niseg);
		nis = niseg;
		dTOI = gen.dTO(I);

		//vpImageIo::writePNG(Iin, "Iin.png");

		std::cout << " nis " << nis << std::endl;
		 }
		}
	    double t6= vpTime::measureTimeMs();
	    std::cout << " time dto " << t6 - t5 << " niseg " << nis << std::endl;

		//gen.bin(Iseg);
		//dTOIseg = gen.dTO(Iseg);
		wght_f_max =0;
		filt = 0;

		//computeInteractionMixingBest(detection.nbParticles);


		t0= vpTime::measureTimeMs();

		//while(filt<filters.size()-1){
		//#pragma omp parallel for private(filt,filter,wght_f) shared(wght_f_max,filt_max,kk)
		for (filt = 0; filt < filters.size()-1 ; filt++)
		{
		//std::cout << " filt " << filt << " " << cogIseg << std::endl;
			//filt = 30;
		 filter=filters[filt+1];
		 /*if(kk>1)// && filt!=filt_Max)
			 {
		 filter->init(detection.nbParticles,prclB,ctPFilter::GSTEP);
		 }*/
		//computePosOri(Iseg, pI, angle, surface);


	     filter->evolution(ctPFilter::GSTEP,measures,1);


	     //if(im == 1)

	     //vpImageIo::writePNG(Iseg0, "Iseg0.png");
	     //filter.likelihoodViewInit(dTOI,true);
			switch(searchStrategy){
			case TOP:
			{
				switch(detection.similarityMs){
				case apDetection::ORIENTED_CHAMFER:

				//if( kk == 1 )
			    //filter->likelihoodViewOCFast0(dTOI, dTOI, true, nis);
					if( kk == 1 )
					filter->likelihoodViewOC(dTOI, dTOIseg, true);
					else filter->likelihoodViewOC(dTOI, dTOIseg, true);

				//else
				//filter->likelihoodViewOCFast(dTOI, dTOI, true);

			    break;
				case apDetection::SHAPE_CONTEXT:
				filter->likelihoodViewSC(logPolarHistIseg, cogIseg, angleIseg, true);
				break;
				case apDetection::STEGER:
				filter->likelihoodViewSteger(Igrad, dTOIseg, true);
			}
			}
			break;
			case TOPDOWN:
				filter->likelihoodStruct3(dTOIseg,dTOIseg,true,filt+1,startingLevel);
			}
	     //filter.likelihoodViewSegInit(dTOI,dTOIseg,true);
	     /*else
	     filter.likelihoodView(dTOI,true);*/
	     //filter.likelihoodStruct2(dTOI,true,filt+1,hpath);
	     //filter.likelihoodStruct3(dTOI,true,filt+1);
		     //double t7= vpTime::measureTimeMs();
	     filter->weightsUpdate(500,0.1);

			switch(PFEType){
			case BEST :
				filter->estimateBest();
			break;
			case MEAN :
				filter->estimateMean();
			}


			//filter.estimateMean();
	     //wght_f = filter->best_weight;
	     wght_f = filter->getWght_sum()/detection.nbParticles;
			std::cout << " filt " << filt << " " << wght_f << std::endl;
	     if(kk>1 && filt!=filt_Max)
	     {
	    	 filter->reWeightP(prclB);
	     }


	     filter->weightedDraw(detection.nbParticles);


	     //filter->simpleDraw();

	     //std::cout << " ok01 " << filter->orieff << std::endl;
		 //double t8= vpTime::measureTimeMs();
					    //std::cout << " time dto " << t8 - t7 << std::endl;
	     //filter.estimateMeanSimple();
	     //filter.weightsUpdate2(Iseg);
	     //filter.estimateBest();
	     //wght_f = filter.getWght_sum()/nbParticles;
	     //std::cout << "bestwght "<< wght_f << std::endl;
	     if (wght_f>wght_f_max)
	     {
	    	wght_f_max = wght_f;
	    	filt_max = filt;
	     }
	     //computePosOri(Iseg, pI, angle, surface);
	     //filters[filt+1]=filter;
	     //filt++;
	}

	     //Iseg = Iseg0;
	std::cout << " filter max " << filt_max << " wght " << wght_f_max << std::endl;

	//computeModeProbBest(detection.nbParticles, Proba_Max, filt_Max, prclB);

	computeCRF(detection.nbParticles, Proba_Max, filt_Max, prclB, kk-1);

	std::cout << " Filter Max " << filt_Max << " pmax " << Proba_Max << std::endl;



	//std::cout << " filterMax " << filt_Max << " proba " <<  Proba_Max << " prclb " << prclB.get_u()  << " " << prclB.get_v() << " " << prclB.get_size() << " " << prclB.get_angle() << std::endl;
	//filter = filters[filt_max+1];
	//filt_Max = 30;
	filter = filters[filt_Max+1];
	switch(searchStrategy){
	case TOP :
	    //filter->likelihoodStruct3(dTOI,true,filt_Max+1,startingLevel-1);
	    //filter->weightsUpdate(500,0.1);
	    //filter->estimateBest();
		prclB = filter->getEstim();
		//filter = filters[46];

	    t1= vpTime::measureTimeMs();
	    std::cout << "Time detect "<< t1-t0 << std::endl;
		cMo = filter->getTemplatePose(I);
		//tracker.setPose(cMo);
		//tracker.track(Id,Inormd,Ior,Ior,tr[2],1);
		//std::cout << " cMo " << cMo << std::endl;
		//filter->displayBestTemp(I);
		filter->displayTemp(I);
	    t0= vpTime::measureTimeMs();
	    std::cout << " time display " << t0 - t1 << std::endl;

	break;
	case TOPDOWN :
		prclB = filter->getEstim();
		cMo = filter->getTemplatePose(I);
	    t1= vpTime::measureTimeMs();
	    std::cout << "Time detect "<< t1-t0 << std::endl;
		//tracker.setPose(cMo);
		//tracker.track(Id,Inormd,Ior,Ior,tr[2],1);
		std::cout << "cMo " << cMo << std::endl;
	    filter->displayBestTemp(I);
	break;
	}

    //filters[filt_max] = filter;

    vpDisplay::flush(I);
	//vpDisplay::getClick(I,true);
	vpImage<vpRGBa> Ioverlay(I.getHeight(),I.getWidth());
	vpDisplay::getImage(I,Ioverlay);
	//std::string opath_ = "IdetectSimStegerRotZ.png";
	std::string opath_ = "Idetectsoyuz0%02d.png";
    char buf4[FILENAME_MAX];
    sprintf(buf4, opath_.c_str(), kk);
    std::string filename4(buf4);
    //std::cout << "Write: " << filename4 << std::endl;
    //vpImageIo::write(Ioverlay, filename4);

	//readerSeg.acquire(Iseg);
	for (int k = 0; k<1;k++)
	{//readerSeg.acquire(Iseg);
	    readerCol.acquire(Icol);
	    double t3= vpTime::measureTimeMs();
		segment.segmentFgdBgd(Icol,IsegCol,im);
		vpImageConvert::convert(IsegCol,Iseg);
	    double t4= vpTime::measureTimeMs();
	    std::cout << " time seg " << t4 - t3 << std::endl;
		Iseg0 = Iseg;
		vpDisplay::flush(Iseg);
		vpDisplay::display(Iseg);
		//vpImageIo::writePNG(Iseg, "Iseg1.png");
		Iin.resize(I.getHeight(),I.getWidth());
		I.resize(I.getHeight(),I.getWidth());
    	reader.acquire(I);
    	Iin = I;

			}
		}

    filter->likelihoodStruct3(dTOI,dTOI,true,filt_Max+1,startingLevel-1);
	prclB = filter->getEstim();
    t1= vpTime::measureTimeMs();
	cMo = filter->getBestPose();
	vpDisplay::display(I);
	std::cout << " cMoBest " << cMo << std::endl;
	filter->displayBestTemp(I);
	vpDisplay::flush(I);
	vpDisplay::getClick(I);
	vpImage<vpRGBa> Ioverlay(I.getHeight(),I.getWidth());
	vpDisplay::getImage(I,Ioverlay);
	std::string opath_ = "Idetectsoyuz0f%02d.png";
    char buf5[FILENAME_MAX];
    sprintf(buf5, opath_.c_str(), kk+1);
    std::string filename5(buf5);
    //std::cout << "Write: " << filename4 << std::endl;
    //vpImageIo::write(Ioverlay, filename5);

    sprintf(bufSeg, opathSeg.c_str(), kk+1);
    std::string filenameSeg1(bufSeg);
    //std::cout << "Write: " << filename4 << std::endl;
    //vpImageIo::write(IsegCol, filenameSeg1);

	display.close(I);
	displaySeg.close(Iseg);

}

void apDetector::computeInteractionMixing(int nbParticles)
{
	apPFilter *filter0;
	apPFilter *filter1;
	double filtProba;
	ctParticle *pMix;
	ctParticle *pFilt;
	ctParticle *prcl;
	float u_cum , v_cum;
	float size_cum;
	float angle_cum;
	u_cum = 0;
	v_cum = 0;
	size_cum = 0;
	angle_cum = 0;
	double nF;
	double transProba,mixProba;
	normFact.resize(filters.size()-1, nbParticles,false);
	for(int f = 0 ; f<filters.size()-1; f++)
	{
		filter0 = filters[f+1];
	for (int i=0; i < nbParticles ; i++)
	{
	    nF = 0;
	for (int g = 0; g < filters.size()-1; g++)
	{
	filter1 = filters[g+1];
	prcl = filter1->particleVect[i];
	filtProba = prcl->getProba();
	transProba = filtTransProb[g][f];
	nF += transProba*filtProba;
	}
	normFact[f][i] = nF;
	for (int g = 0; g < filters.size()-1; g++)
	{
	filter1 = filters[g+1];
	pFilt = filter1->particleVect[i];
	filtProba = pFilt->getProba();
	transProba = filtTransProb[g][f];
	mixProba = transProba*filtProba/normFact[f][i];

	u_cum += pFilt->get_u() * mixProba;
	v_cum += pFilt->get_v() * mixProba;
	angle_cum += pFilt->get_angle() * mixProba;
	size_cum += pFilt->get_size() * mixProba;
	}
	pMix = filter0->particleVect[i];
	pMix->set_u((int)u_cum);
	pMix->set_v((int)v_cum);
	pMix->set_angle(angle_cum);
	pMix->set_size(size_cum);
	//filter.setMix(pMix);
	//filter.reWeightMix();
	//filter0->particleVect[i] = pMix;
	}
	//filters[f+1] = filter0;
	}
}


void apDetector::computeInteractionMixingBest(int nbParticles)
{
	apPFilter *filter0;
	apPFilter *filter1;
	double filtProba;
	ctParticle *pMix;
	ctParticle *pFilt;
	ctParticle *prcl;
	float u_cum , v_cum;
	float size_cum;
	float angle_cum;
	u_cum = 0;
	v_cum = 0;
	size_cum = 0;
	angle_cum = 0;
	double nF;
	double transProba,mixProba;
	std::vector <ctParticle> prclVect;
	normFact.resize(filters.size()-1, 1,false);

	for(int f = 0 ; f<filters.size()-1; f++)
	{
		filter0 = filters[f+1];
		//filtProba = filter0->getProba();
	//for (int i=0; i < nbParticles ; i++)
	//{
	    nF = 0;
	for (int g = 0; g < filters.size()-1; g++)
	{
		double t0= vpTime::measureTimeMs();
	filter1 = filters[g+1];
	//prcl = filter1.particleVect[i];
	//filtProba = prcl.getProba();
	filtProba = filter1->getProba();
	filtProba = (double)1/(filters.size()-1);
	transProba = filtTransProb[g][f];
	nF += transProba*filtProba;


	double t1= vpTime::measureTimeMs();
	///std::cout << " time mixing " << filtProba << std::endl;
	}
	normFact[f][0] = nF;
	/*for (int g = 0; g < filters.size()-1; g++)
	{
	filter1 = filters[g+1];
	//pFilt = filter1.particleVect[i];
	//filtProba = pFilt.getProba();
	filtProba = filter1.getProba();
	transProba = filtTransProb[g][f];
	mixProba = transProba*filtProba/normFact[f][i];

	u_cum += pFilt.get_u() * mixProba;
	v_cum += pFilt.get_v() * mixProba;
	angle_cum += pFilt.get_angle() * mixProba;
	size_cum += pFilt.get_size() * mixProba;
	}
	prclVect = filter0.particleVect;
	for (int i=0; i < nbParticles ; i++)
	{
	pMix = prclVect
	pMix.set_u((int)u_cum);
	pMix.set_v((int)v_cum);
	pMix.set_angle(angle_cum);
	pMix.set_size(size_cum);
	}*/
	//filter.setMix(pMix);
	//filter.reWeightMix();
	//filter0.particleVect[i] = pMix;
	//}
	//filters[f+1] = filter0;
	}

}

void apDetector::computeCRF(int nbParticles, double &ProbaMax, int &filtMax, ctParticle &pComb,int k)
{
	likelihoods.resize(filters.size()-1,k+1,false);
	probasfilt.resize(filters.size()-1,k+1,false);
	filtProb.resize(k+1,filters.size()-1,false);
	apPFilter *filter0;
		double filtProba, partProba,partProba2, transProba;
		ctParticle pFilt;
		ctParticle prcl;
		ctParticle pEstim;
		double wght;
		double delta;
		//double normFactPart[nbParticles];
		double normFactPart;
		//std::vector<ctParticle>  particleVect;
		double nFP;
		float u_cum, u_filt , v_cum, v_filt;
		float size_cum, size_filt;
		float angle_cum, angle_filt;
		u_cum = 0;
		v_cum = 0;
		size_cum = 0;
		angle_cum = 0;
		u_filt = 0;
		v_filt = 0;
		size_filt = 0;
		angle_filt = 0;
		ProbaMax = 0;
		double t0 = vpTime::measureTimeMs();
		double deltaMax = 0;
		nFP = 0;

		if(k == 0)
		{
			computeInteractionMixingBest(nbParticles);
			for(int f = 0 ; f<filters.size()-1; f++)
			{
			filter0 = filters[f+1];
		    //wght = filter0->best_weight;
		    wght = filter0->getWght_sum()/detection.nbParticles;
		    nFP += wght;//*normFact[f][0];
			}
			normFactPart = nFP;
			//}

			for(int f = 0 ; f<filters.size()-1; f++)
			{
				filter0 = filters[f+1];
				//for (int i=0; i < nbParticles ; i++)
				//{
		    //wght = filter0->best_weight;
		    wght = filter0->getWght_sum()/detection.nbParticles;
		    partProba = wght/normFactPart;//*normFact[f][0];
		    //std::cout << " part proba " << f << " " << partProba << " weight " << wght << " trans " << filtTransProb[86][9] << " " << filtTransProb[86][85] <<  std::endl;
		    //prcl.setProba(partProba);
		    probasfilt[f][k] = partProba;
		    filtProb[k][f] = partProba;

		    if (partProba>ProbaMax)
		    {
		      ProbaMax = partProba;
		      filtMax = f;
		    }
		    //filter0.particleVect[i] = prcl;
		    prcl = filter0->getEstim();
			u_cum += prcl.get_u() * partProba;
			v_cum += prcl.get_v() * partProba;
			angle_cum += prcl.get_angle() * partProba;
			size_cum += prcl.get_size() * partProba;

			}

			pComb.set_u(u_cum);
			pComb.set_v(v_cum);
			pComb.set_angle(angle_cum);
			pComb.set_size(size_cum);


		}
		else
		{
			for (int l = 0; l <= k; l++)
			{
				if(l == 0)
				{
					computeInteractionMixingBest(nbParticles);
					for(int f = 0 ; f<filters.size()-1; f++)
					{
					filter0 = filters[f+1];
				    //wght = filter0->best_weight;
				    wght = filter0->getWght_sum()/detection.nbParticles;
				    nFP += wght;//*normFact[f][0];
					}
					normFactPart = nFP;
					//}

					for(int f = 0 ; f<filters.size()-1; f++)
					{
						filter0 = filters[f+1];
						//for (int i=0; i < nbParticles ; i++)
						//{
				    //wght = filter0->best_weight;

				    wght = filter0->getWght_sum()/detection.nbParticles;

				    partProba2 = wght/normFactPart;//*normFact[f][0];
				    probasfilt[f][k] = partProba2;
				    partProba = probasfilt[f][l];
				    //std::cout << " part proba " << f << " " << partProba << " weight " << wght << " trans " << filtTransProb[86][9] << " " << filtTransProb[86][85] <<  std::endl;
				    //prcl.setProba(partProba);
				    filter0->setProba(partProba2);
				    likelihoods[f][l] = partProba;
				    //likelihoods[f][k] = probasfilt[f][k];
				    //filtProb[k][f] = 1/((double)filters.size()-1);
				    //filtProb[k][f] = partProba;
				    //filtProb[f][k] = probasfilt[f][k];


					//}
					/*u_filt += u_cum/nbParticles;
					v_filt += v_cum/nbParticles;
					angle_filt += angle_cum/nbParticles;
					size_filt += size_cum/nbParticles;*/

					//filters[f+1] = filter0;
					}
				}
				else{

				/*for(int f = 0 ; f<filters.size()-1; f++)
				{
				filter0 = filters[f+1];
			    wght = filter0->best_weight;
			    nFP += wght;
				}
				normFactPart = nFP;*/

				for(int f = 0 ; f<filters.size()-1; f++)
				{
					filter0 = filters[f+1];
					//wght = filter0->best_weight;
					wght = filter0->getWght_sum()/detection.nbParticles;
					deltaMax = 0;
					for(int g = 0 ; g < filters.size()-1; g++)
					{
					//transProba = filtTransProb[g][f];
					transProba = filtTransProb[g][f];
					//std::cout << transProba << std::endl;

					//partProba = wght*transProba;
					partProba = probasfilt[g][l]*transProba;
					delta = likelihoods[g][l-1]*partProba;
					if(delta > deltaMax)
					{
						deltaMax = delta;
						filtMax = g;
					}
					}
					likelihoods[f][l] = deltaMax;
				}

				if (l == k)
				{
					normFactPart = 0;
					ProbaMax = 0;
					for(int f = 0 ; f<filters.size()-1; f++)
					{
						normFactPart += likelihoods[f][l];
					}
					for(int f = 0 ; f<filters.size()-1; f++)
					{
						filter0 = filters[f+1];
						partProba = likelihoods[f][l]/normFactPart;
						if(partProba>ProbaMax)
						{
						ProbaMax = partProba;
						filtMax = f;
						}
						filter0->setProba(partProba);
					    prcl = filter0->getEstim();
						u_cum += prcl.get_u() * partProba;
						v_cum += prcl.get_v() * partProba;
						angle_cum += prcl.get_angle() * partProba;
						size_cum += prcl.get_size() * partProba;
						filtProb[l][f] = partProba;
						//if(f==0 || f == 19 || f==45 || f==46 || f == 50)
						//{
					    //std::cout << " part proba " << f << " " << partProba << " weight " << filter0->getWght_sum()/detection.nbParticles <<  std::endl;
					    if(f==0)
					    std::cout << probasfilt << std::endl;
						//}
					}

					pComb.set_u(u_cum);
					pComb.set_v(v_cum);
					pComb.set_angle(angle_cum);
					pComb.set_size(size_cum);

				}
			}

			}
		}
filtProb.saveMatrix("filtprobasoyuz0.txt", filtProb, false);
probasfilt.saveMatrix("filtlikelihoodsoyuz0.txt", probasfilt.transpose(), false);

}


ctParticle apDetector::computeModeProb(int nbParticles)
{
	apPFilter *filter0;
	double filtProba, partProba;
	ctParticle pComb;
	ctParticle pFilt;
	ctParticle prcl;
	double wght;
	double normFactPart[nbParticles];
	//std::vector<ctParticle>  particleVect;
	double nFP;
	float u_cum, u_filt , v_cum, v_filt;
	float size_cum, size_filt;
	float angle_cum, angle_filt;
	u_cum = 0;
	v_cum = 0;
	size_cum = 0;
	angle_cum = 0;
	u_filt = 0;
	v_filt = 0;
	size_filt = 0;
	angle_filt = 0;
	for (int i=0; i < nbParticles ; i++)
	{
    nFP = 0;
	for(int f = 0 ; f<filters.size()-1; f++)
	{
	filter0 = filters[f+1];
    prcl = *filter0->particleVect[i];
    wght = prcl.getWeight();
    nFP += wght*normFact[f][i];
	}
	normFactPart[i] = nFP;
	}

	for(int f = 0 ; f<filters.size()-1; f++)
	{
		filter0 = filters[f+1];
		for (int i=0; i < nbParticles ; i++)
		{
    prcl = *filter0->particleVect[i];
    wght = prcl.getWeight();
    partProba = wght*normFact[f][i]/normFactPart[i];
    prcl.setProba(partProba);
    *filter0->particleVect[i] = prcl;
	u_cum += prcl.get_u() * partProba;
	v_cum += prcl.get_v() * partProba;
	angle_cum += prcl.get_angle() * partProba;
	size_cum += prcl.get_size() * partProba;
	}
	u_filt += u_cum/nbParticles;
	v_filt += v_cum/nbParticles;
	angle_filt += angle_cum/nbParticles;
	size_filt += size_cum/nbParticles;
	//filters[f+1] = filter0;
	}
	pComb.set_u(u_filt);
	pComb.set_v(v_filt);
	pComb.set_angle(angle_filt);
	pComb.set_size(size_filt);
	return pComb;
}

void apDetector::computeModeProbBest(int nbParticles, double &ProbaMax, int &filtMax, ctParticle &pComb)
{
	apPFilter *filter0;
	double filtProba, partProba;
	ctParticle pFilt;
	ctParticle prcl;
	double wght;
	//double normFactPart[nbParticles];
	double normFactPart;
	//std::vector<ctParticle>  particleVect;
	double nFP;
	float u_cum, u_filt , v_cum, v_filt;
	float size_cum, size_filt;
	float angle_cum, angle_filt;
	u_cum = 0;
	v_cum = 0;
	size_cum = 0;
	angle_cum = 0;
	u_filt = 0;
	v_filt = 0;
	size_filt = 0;
	angle_filt = 0;
	ProbaMax = 0;
	double t0 = vpTime::measureTimeMs();
	//for (int i=0; i < nbParticles ; i++)
	//{
    nFP = 0;
	for(int f = 0 ; f<filters.size()-1; f++)
	{
	filter0 = filters[f+1];
    //wght = filter0->best_weight;
    wght = filter0->getWght_sum()/detection.nbParticles;
    nFP += wght*normFact[f][0];
	}
	normFactPart = nFP;
	//}

	for(int f = 0 ; f<filters.size()-1; f++)
	{
		filter0 = filters[f+1];
		//for (int i=0; i < nbParticles ; i++)
		//{
    //wght = filter0->best_weight;
    wght = filter0->getWght_sum()/detection.nbParticles;
    partProba = wght*normFact[f][0]/normFactPart;
    //std::cout << " part proba " << f << " " << partProba << std::endl;
    //prcl.setProba(partProba);
    filter0->setProba(partProba);
    if (partProba>ProbaMax)
    {
      ProbaMax = partProba;
      filtMax = f;
    }
    //filter0.particleVect[i] = prcl;
    prcl = filter0->getEstim();
	u_cum += prcl.get_u() * partProba;
	v_cum += prcl.get_v() * partProba;
	angle_cum += prcl.get_angle() * partProba;
	size_cum += prcl.get_size() * partProba;
	//}
	/*u_filt += u_cum/nbParticles;
	v_filt += v_cum/nbParticles;
	angle_filt += angle_cum/nbParticles;
	size_filt += size_cum/nbParticles;*/

	//filters[f+1] = filter0;
	}
	pComb.set_u(u_cum);
	pComb.set_v(v_cum);
	pComb.set_angle(angle_cum);
	pComb.set_size(size_cum);

	double t1= vpTime::measureTimeMs();
	std::cout << " time mixing " << t1 -t0 << std::endl;
}


void apDetector::computeTransitionProba(const char *filenameTP)
{
vpMatrix transProb;
filtTransProb.resize(filters.size()-1,filters.size()-1, false);
transProb.loadMatrix(filenameTP,transProb,false);
double sum=0;
std::cout <<" okP " << filters.size() << std::endl;
for (int ii = 0; ii < transProb.getRows(); ii++)
{
		sum+=transProb[ii][2];
}
sum+=(filters.size()-1)*100;
int kk=0;
for (int ii = 0; ii <filtTransProb.getRows(); ii++)
	for (int jj = 0; jj <filtTransProb.getCols(); jj++)
{
		//std::cout << " row " << ii*filtTransProb.getCols() + jj << std::endl;
		if(ii!=jj)
		//filtTransProb[ii][jj] = transProb[ii*(filtTransProb.getCols()-1) + jj -1][2];//sum;
			{filtTransProb[ii][jj] = (transProb[kk][2])/sum;
			kk++;
			}
		else filtTransProb[ii][jj] = 100/sum;
}
filtTransProb.saveMatrix("transProbPA.txt", filtTransProb, false);
}

void apDetector::computeTransitionProbV(const char *filenameTP)
{
vpMatrix transProb;
filtTransProb.resize(filters.size()-1,filters.size()-1, false);

apPFilter *filter0, *filter1;
double sum=0;
vpPoseVector p0;
vpPoseVector p1;

vpHomogeneousMatrix cMo0,oMc0,cMo1,oMc1;
vpTranslationVector tr0, tr1;
sum+=(filters.size()-1)*2;
int kk=0;
double likelihood;
for (int ii = 0; ii <filtTransProb.getRows(); ii++)
{	filter0=filters[ii+1];
    p0  = filter0->getPose();
    cMo0.buildFrom(p0);
    oMc0 = cMo0.inverse();
    oMc0.extract(tr0);
    std::cout << "p0 " << tr0[1] << " " << tr0[2] << " " << tr0[3] << std::endl;
	for (int jj = 0; jj <filtTransProb.getCols(); jj++)
{
		filter1=filters[jj+1];
		p1  = filter1->getPose();
	    cMo1.buildFrom(p1);
	    oMc1 = cMo1.inverse();
	    oMc1.extract(tr1);
		//std::cout << " row " << ii*filtTransProb.getCols() + jj << std::endl;
		//if(ii!=jj)
			{
			likelihood = exp(-(0.5/(detection.sigmaf*detection.sigmaf))*acos((tr0[0]*tr1[0]+ tr0[1]*tr1[1] + tr0[2]*tr1[2])/(sqrt(tr0[0]*tr0[0]+ tr0[1]*tr0[1] + tr0[2]*tr0[2])*sqrt(tr1[0]*tr1[0]+ tr1[1]*tr1[1] + tr1[2]*tr1[2] ))));
			sum+=likelihood;
			}
}
}

for (int ii = 0; ii <filtTransProb.getRows(); ii++)
{	filter0=filters[ii+1];
    p0  = filter0->getPose();
    cMo0.buildFrom(p0);
    oMc0 = cMo0.inverse();
    oMc0.extract(tr0);
	for (int jj = 0; jj <filtTransProb.getCols(); jj++)
{
		filter1=filters[jj+1];
		p1  = filter1->getPose();
	    cMo1.buildFrom(p1);
	    oMc1 = cMo1.inverse();
	    oMc1.extract(tr1);
		//std::cout << " row " << ii*filtTransProb.getCols() + jj << std::endl;
		//if(ii!=jj)
			{
			likelihood = exp(-(0.5/(detection.sigmaf*detection.sigmaf))*acos((tr0[0]*tr1[0]+ tr0[1]*tr1[1])/(sqrt(tr0[0]*tr0[0]+ tr0[1]*tr0[1])*sqrt(tr1[0]*tr1[0]+ tr1[1]*tr1[1]))));
			filtTransProb[ii][jj]=likelihood ;
			}
		//else filtTransProb[ii][jj] = 1 /sum;
}
}
filtTransProb.saveMatrix("transProbaMatsoyuz0.txt", filtTransProb, false);
}


/*void apDetector::computeTransitionProb()
{
vpMatrix transProb;
filtTransProb.resize(filters.size()-1,filters.size()-1, false);

double sum=0;
for (int ii = 0; ii < transProb.getRows(); ii++)
{
		sum+=transProb[ii][2];
}
sum+=(filters.size()-1)*100;
int kk=0;
for (int ii = 0; ii <filtTransProb.getRows(); ii++)
	for (int jj = 0; jj <filtTransProb.getCols(); jj++)
{
		std::cout << " row " << ii*filtTransProb.getCols() + jj << std::endl;
		if(ii!=jj)
		//filtTransProb[ii][jj] = transProb[ii*(filtTransProb.getCols()-1) + jj -1][2];//sum;
			{filtTransProb[ii][jj] = (transProb[kk][2])/sum;
			kk++;
			}
		else filtTransProb[ii][jj] = 100/sum;
}
filtTransProb.saveMatrix("transProbMat.txt", filtTransProb, false);
}*/


