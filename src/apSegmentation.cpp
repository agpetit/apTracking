/*
 * apSegmentation.cpp
 *
 *  Created on: Nov 15, 2012
 *      Author: agpetit
 */

#include "apSegmentation.h"
#include "GL/glut.h"
//#include "minE.cpp"

apSegmentation::apSegmentation() {
	// TODO Auto-generated constructor stub

}

apSegmentation::~apSegmentation() {
	// TODO Auto-generated destructor stub
}


void apSegmentation::init(vpImage<vpRGBa> &_I)
{
   type = PETIT;
	switch(type){
	case OPENCV_GMM_ZIVKOVIC:
		{break;
		}
	case OPENCV_GMM_BAF:
	{

	}
	case CRIMINISI:
	{
	}
	case PETIT:
	{

		int xpic = _I.getWidth();
		int ypic =  _I.getHeight();
		vpImage<vpRGBa> I_col (ypic ,xpic);
		//vpImage<unsigned char> I_uchar (ypic ,xpic);
		vpImageConvert::convert(I_col,I_uchar);
		CvSize frame_size;
		frame_size.height = I_uchar.getHeight();
		frame_size.width =  I_uchar.getWidth();
		frame_1c_orig = NULL;
		allocateOnDemand( &frame_1c_orig, frame_size, IPL_DEPTH_8U, 1 );
		vpImageConvert::convert(_I,I_uchar);
		vpImageConvert::convert(I_uchar, frame_1c_orig) ;
		std::cout << " npoints " << segParam.KLTParams.nbPoints << std::endl;
		display.init(I_uchar, 0 ,1500 ,"__1");
		segMot.setSegmentationParameters(segParam);
		segMot.init();
		segMot.initTracker();
		segMot.initTrajectories(frame_1c_orig);
		segMot.initEnergy(I_uchar);
		segMot.histIm.push_back(_I);
	}
	}

}

void apSegmentation::clear()
{
	segMot.clear();
}

void apSegmentation::segmentFgdBgd(vpImage<vpRGBa> &_I, vpImage<vpRGBa> &I_, int frame)
{
	switch(type){
	case OPENCV_GMM_ZIVKOVIC:
	{
		break;
	}
	case OPENCV_GMM_BAF:
	{
		break;
	}
	case CRIMINISI:
	{

	break;
	}
	case PETIT:
	{

	int xpic = _I.getWidth();
	int ypic =  _I.getHeight();
	//vpImage<unsigned char> I_uchar (ypic ,xpic);
	I_uchar.resize(ypic,xpic);
	vpImage<unsigned char> I_seg (ypic ,xpic);

	vpImage<vpRGBa> I2 (ypic ,xpic);
	unsigned int current_frame = 1;
	CvSize frame_size;
	frame_size.height = I_uchar.getHeight();
	frame_size.width =  I_uchar.getWidth();

	/*IplImage *frame_1c_orig = NULL;
	allocateOnDemand( &frame_1c_orig, frame_size, IPL_DEPTH_8U, 1 );*/

	//vpDisplayX display (I_uchar, frame_size.height ,frame_size.width ,"__1");
	//vpDisplayX display (I2, frame_size.height ,frame_size.width ,"__1");

	vpImageConvert::convert(_I,I_uchar);
	//vpImageIo::writePNG(I_uchar,"Is1.png");
	vpImageConvert::convert(I_uchar, frame_1c_orig) ;

	vpDisplay::display(I_uchar);
	//add new features at each 25 frames


	if ((current_frame % 25) ==  1)
	{
		apSegMotionCol segTemp;
		segTemp.setSegmentationParameters(segParam);
		segTemp.init();
		segTemp.initTrajectories(frame_1c_orig);
		segMot.fusionTrajectories(segTemp);
		cout << segMot.tracker.getNbFeatures()<< endl;


	}
	double t0= vpTime::measureTimeMs();
	segMot.tracker.track(frame_1c_orig);
	segMot.updateTrajectories();
	double t1= vpTime::measureTimeMs();
					    std::cout << " time klt " << t1 - t0 << std::endl;
	segMot.displayTrajectories(I_uchar);
	if (frame>0)
	{
		segMot.histIm.push_back(_I);
		//segMot.labelSelectPoints(I_uchar, _I);
		//if(frame>3)
		//segMot.labelPoints(I_uchar, _I);
		//segMot.computeLikelihoodHomography(I_uchar, _I);
		//if(frame <= segmentParam.startFrame)
		//segMot.computeHistKLTPoints(_I);
	}

	 if(frame>=segParam.startFrame)
	{
		 t0= vpTime::measureTimeMs();


		 switch(segParam.bType)
		 {
		 case apSegmentationParameters::EARTH:
			    segMot.labelPoints(I_uchar, _I);
			    //segMot.computeHistKLTPoints(_I);
			    break;
		 case apSegmentationParameters::LIMB:
			    segMot.labelSelectPoints(I_uchar, _I);
			    segMot.detectLimb(I_uchar,_I);
		 }
		 display.flush(I_uchar);
		 display.getImage(I2);
		 Iclust = I2;
		 t1= vpTime::measureTimeMs();
		 std::cout << " time selectPoints " << t1 - t0 << std::endl;

		 if(frame == segParam.startFrame)
		 {
		 switch(segParam.bType)
		 {
		 case apSegmentationParameters::EARTH:
			    segMot.computeDataEnergyKernel(I_uchar, _I);
			    break;
		 case apSegmentationParameters::DEEPSPACE:
			    segMot.computeDataEnergyHistInitDeepSpace(I_uchar, _I);
			    break;
		 case apSegmentationParameters::LIMB:
			    segMot.computeDataEnergyKernelLimb(I_uchar, _I);
		 }
		 }
		 else
		 {
			 switch(segParam.bType)
			 {
			 case apSegmentationParameters::EARTH:
				    segMot.computeDataEnergyHist(I_uchar, _I);
				    break;
			 case apSegmentationParameters::DEEPSPACE:
				    segMot.computeDataEnergyHistDeepSpace(I_uchar, _I);
				    break;
			 case apSegmentationParameters::LIMB:
				    //segMot.computeDataEnergyKernelHistLimb(I_uchar, _I);
				    segMot.computeDataEnergyHist(I_uchar,_I);
			 }
		 //else segMot.computeEnergyHist(I_uchar, _I);
		 }

	double t2= vpTime::measureTimeMs();
	segMot.computeSpatialEnergy(I_uchar, _I);
	double t3= vpTime::measureTimeMs();

	std::cout << " time spatial " << t3 - t2 << std::endl;

	segMot.minimizeEnergy(I_uchar, _I);

	double t4= vpTime::measureTimeMs();

	std::cout << " time min " << t4 - t3 << std::endl;

	 switch(segParam.bType)
	 {
	 case apSegmentationParameters::EARTH:
	 case apSegmentationParameters::DEEPSPACE:
		    segMot.computeColorHist(_I);
		    break;
	 case apSegmentationParameters::LIMB:
			segMot.computeColorHistLimb(_I);
	 }

	double t5 = vpTime::measureTimeMs();

	std::cout << " time hist " << t5 - t4 << std::endl;

	std::cout << " time seg " << t4 - t0 << std::endl;

	/*
	if(frame == segParam.startFrame)
	{
	segMot.computeColorHist(_I);
	}*/
	//segMot.computeHistKLTPoints(_I);
	//}
	I_ = _I;
	segMot.getIseg(I_uchar,I_seg,I_);

	/*std::string opath_ = "Isamazonas_%01d.png";
    char buf4[FILENAME_MAX];
    sprintf(buf4, opath_.c_str(), frame-8);
    std::string filename4(buf4);
    //std::cout << "Write: " << filename4 << std::endl;
    vpImageIo::write(I_, filename4);*/
	}
	display.flush(I_uchar);
	vpDisplay::display(I_uchar);
	//vpDisplay::getClick(I_uchar);

	break;
	}
	}
}



