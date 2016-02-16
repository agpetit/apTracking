/*
 * apData.h
 *
 *  Created on: Dec 4, 2012
 *      Author: agpetit
 */

#ifndef APDATA_H_
#define APDATA_H_

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <cv.h>
#include <visp/vpConfig.h>
#include <visp/vpMath.h>
#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpImagePoint.h>

class apData {
public:

	vpImagePoint pt;
	vpRGBa col;
	apData();
	virtual ~apData();
};

#endif /* APDATA_H_ */
