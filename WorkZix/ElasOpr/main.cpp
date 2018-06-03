/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libelas.
Authors: Andreas Geiger

libelas is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 3 of the License, or any later version.

libelas is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libelas; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

// Demo program showing how libelas can be used, try "./elas -h" for help

#include <iostream>
#include "elas.h"
#include "image.h"
#include <cv.h>
#include <highgui.h>
#include <contrib\contrib.hpp>

using namespace cv;
using namespace std;

Mat Grey2FateColor(Mat& mat_in);
// compute disparities of pgm image input pair file_1, file_2
void process (const char* file_1,const char* file_2) 
{

	string szRoot = "D:\\测试资料\\data_odometry_gray\\dataset\\sequences\\02_\\";

	for (int i = 700/*800*/; i < 10000; i++)
	{
		char szCont[10];
		sprintf(szCont,"%06d",i);
		string strL = szRoot+"image_0\\"+szCont+".png";
		string strR = szRoot+"image_1\\"+szCont+".png";
		Mat imageL = imread(strL,0);
		Mat imageR = imread(strR,0);
		if (imageL.empty() || imageR.empty())
			break;

		// load images
		Mat I1 = imageL/*imread(file_1,0)*/;
		Mat I2 = imageR/*imread(file_2,0)*/;

		Mat out1_(I1.rows,I1.cols,CV_32F);
		Mat out2_(I2.rows,I2.cols,CV_32F);

		// process
		const int32_t dims[3] = {I1.cols,I1.rows,I1.cols}; // bytes per line = width
		Elas::parameters param;
		param.postprocess_only_left = true;
		param.disp_max = 96;
		Elas elas(param);
		elas.process(I1.data,I2.data,(float*)out1_.data,(float*)out2_.data,dims);

		Mat disp = Grey2FateColor(out1_);
		cvNamedWindow("dist",0);
		imshow("dist",disp);
		waitKey();

	}


// 	file_1 = "D:\\WorkDirDevice\\VisO - 副本\\img\\urban1_left.pgm";
// 	file_2 = "D:\\WorkDirDevice\\VisO - 副本\\img\\urban1_right.pgm";
// 	std::cout << "Processing: " << file_1 << ", " << file_2 << endl;


}

int main (int argc, char** argv) {

  // run demo
  if (argc==2 && !strcmp(argv[1],"demo")) {
    process("img/cones_left.pgm",   "img/cones_right.pgm");
    process("img/aloe_left.pgm",    "img/aloe_right.pgm");
    process("img/raindeer_left.pgm","img/raindeer_right.pgm");
    process("img/urban1_left.pgm",  "img/urban1_right.pgm");
    process("img/urban2_left.pgm",  "img/urban2_right.pgm");
    process("img/urban3_left.pgm",  "img/urban3_right.pgm");
    process("img/urban4_left.pgm",  "img/urban4_right.pgm");
    cout << "... done!" << endl;

  // compute disparity from input pair
  } else if (/*argc==3*/1) {
    process(argv[1],argv[2]);
    cout << "... done!" << endl;

  // display help
  } else {
    cout << endl;
    cout << "ELAS demo program usage: " << endl;
    cout << "./elas demo ................ process all test images (image dir)" << endl;
    cout << "./elas left.pgm right.pgm .. process a single stereo pair" << endl;
    cout << "./elas -h .................. shows this help" << endl;
    cout << endl;
    cout << "Note: All images must be pgm greylevel images. All output" << endl;
    cout << "      disparities will be scaled such that disp_max = 255." << endl;
    cout << endl;
  }

  return 0;
}

Mat Grey2FateColor(Mat& mat_in)
{
	Mat temp(mat_in.rows,mat_in.cols,CV_8UC1),temp0;
	mat_in.convertTo(temp,CV_8U);
	temp*=5;
// 	Mat temp(mat_in.rows,mat_in.cols,CV_8UC1);
// 	cvNormalize( &CvMat(mat_in), &CvMat(temp), 0, 350, CV_MINMAX );

	CvMat* red = cvCreateMat(temp.rows, temp.cols, CV_8U);
	CvMat* green = cvCreateMat(temp.rows, temp.cols, CV_8U);
	CvMat* blue = cvCreateMat(temp.rows, temp.cols, CV_8U);
	CvMat* mask = cvCreateMat(temp.rows, temp.cols, CV_8U);

	Mat color_mat(temp.rows,temp.cols,CV_8UC3);
	// 计算各彩色通道的像素值
	cvSubRS(&CvMat(temp), cvScalar(255), blue);	// blue(I) = 255 - gray(I)
	cvCopy(&CvMat(temp), red);			// red(I) = gray(I)
	cvCopy(&CvMat(temp), green);			// green(I) = gray(I),if gray(I) < 128
	cvCmpS(green, 128, mask, CV_CMP_GE );	// green(I) = 255 - gray(I), if gray(I) >= 128
	cvSubRS(green, cvScalar(255), green, mask);
	cvConvertScale(green, green, 2.0, 0.0);

	// 合成伪彩色图
	cvMerge(blue, green, red, NULL, &CvMat(color_mat));

	cvReleaseMat( &red );
	cvReleaseMat( &green );
	cvReleaseMat( &blue );
	cvReleaseMat( &mask );

	return color_mat;
}