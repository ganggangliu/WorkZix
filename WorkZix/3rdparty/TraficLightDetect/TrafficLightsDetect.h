#include "cv.h"
#include "cxcore.h"
#include "highgui.h"
#include <cmath>
#include <vector>
#include <iostream>
#include<windows.h>
#include<winbase.h>
#include <ml.h>

#include "opencv2/core/core.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


using namespace std;
using namespace cv;




#ifndef TrafficLightsDetectDll_H_
#define TrafficLightsDetectDll_H_
#ifdef MYLIBDLL
#define  MYLIBDLL extern "C" _declspec(dllimport)
#else
#define MYLIBDLL extern "C" _declspec(dllexport)
#endif
MYLIBDLL typedef struct _light_pos
{
	CvRect pos;
	int type;	// 0: green, 1: yellow, 2: red
	_light_pos()
	{
		type = 2;
	}
}light_pos_t;
MYLIBDLL  int TrafficLights(Mat& imgsrc, CvRect& roi,vector<light_pos_t> &plight,int size);
#endif