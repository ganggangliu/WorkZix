// FileName: OpenCVInc.h
// Author: Bo Chen
// Create Time: 2016-02-18
// 
#ifndef _OPENCV_INC_H_
#define _OPENCV_INC_H_

#include <stdio.h>
#include <iostream>
#include <fstream>

#ifdef WIN32
#pragma warning(disable:4819)
#endif // WIN32

#include "cv.h"
#include "opencv2/opencv.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/legacy/legacy.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/gpu/gpu.hpp"

#ifdef WIN32

#define CV_VERSION_ID       CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)

#ifdef _DEBUG
    #define cvLIB(name) "opencv_" name CV_VERSION_ID "d"
#else
    #define cvLIB(name) "opencv_" name CV_VERSION_ID
#endif

#pragma comment( lib, cvLIB("core") )
#pragma comment( lib, cvLIB("imgproc") )
#pragma comment( lib, cvLIB("highgui") )
#pragma comment( lib, cvLIB("flann") )
#pragma comment( lib, cvLIB("features2d") )
#pragma comment( lib, cvLIB("calib3d") )
#pragma comment( lib, cvLIB("gpu") )
#pragma comment( lib, cvLIB("legacy") )
#pragma comment( lib, cvLIB("ml") )
#pragma comment( lib, cvLIB("objdetect") )
#pragma comment( lib, cvLIB("ts") )
#pragma comment( lib, cvLIB("video") )
#pragma comment( lib, cvLIB("contrib") )
#pragma comment( lib, cvLIB("nonfree") )

#endif // WIN32

#endif // _OPENCV_INC_H_


