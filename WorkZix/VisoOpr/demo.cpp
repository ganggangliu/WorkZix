/*
Copyright 2012. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

/*
  Documented C++ sample code of stereo visual odometry (modify to your needs)
  To run this demonstration, download the Karlsruhe dataset sequence
  '2010_03_09_drive_0019' from: www.cvlibs.net!
  Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019
*/
//#include <Windows.h>
#include <iostream>
#include <string>
#include <vector>
#include <stdint.h>

#include <viso_stereo.h>
#include <viso_mono.h>
//#include <png++/png.hpp>
#include <cv.h>
#include <highgui.h>
#include "OpenGl.h"
#include "reconstruction.h"
using namespace cv;
using namespace std;

int main(int argc, char** argv) 
{
	COpenGl* pGL = COpenGl::GetInstance();
	pGL->Show(600,600);
  // we need the path name to 2010_03_09_drive_0019 as input argument
  if (argc<2) {
    cerr << "Usage: D:\测试资料\data_odometry_gray\dataset\sequences\05_" << endl;
    return 1;
  }

  // sequence directory
//  string dir = argv[1];
  string dir = "D:\\测试资料\\data_odometry_gray\\dataset\\sequences\\05_";

  // set most important visual odometry parameters
  // for a full parameter list, look at: viso_stereo.h
  VisualOdometryStereo::parameters param;
  
  // calibration parameters for sequence 2010_03_09_drive_0019 
  param.calib.f  = 7.070912000000e+02; // focal length in pixels
  param.calib.cu = 6.018873000000e+02; // principal point (u-coordinate) in pixels
  param.calib.cv = 1.831104000000e+02; // principal point (v-coordinate) in pixels
  param.base     = /*0.5707*/0.3334597000000; // baseline in meters
  param.bucket.bucket_height = 20;
  param.bucket.bucket_width = 20;
  param.ransac_iters = 100;
  param.inlier_threshold = 5.0;
//  param.match.match_radius = 400;
  param.reweighting = false;
//  param.match.nms_tau = 30;
  
  Reconstruction Rebuld;
  Rebuld.setCalibration(param.calib.f, param.calib.cu, param.calib.cv);
  // init visual odometry
  VisualOdometryStereo viso(param);
  
  // current pose (this matrix transforms a point from the current
  // frame's camera coordinates to the first frame's camera coordinates)
  Matrix pose = Matrix::eye(4);
    
  // loop through all frames i=0:372
  LARGE_INTEGER t0,t1,tc;
  QueryPerformanceFrequency(&tc);
  QueryPerformanceCounter(&t0);
  long nCloudInd = 0;
  for (int32_t i=870; i<10000; i++) //800 2000 2340 2620
  {
	QueryPerformanceCounter(&t1);
	printf("Use Time:%f\n",(t1.QuadPart - t0.QuadPart)*1.0/tc.QuadPart);
	t0 = t1;
    // input file names
    char base_name[256]; sprintf(base_name,"%06d.png",i);
    string left_img_file_name  = dir + "/image_0/" + base_name;
    string right_img_file_name = dir + "/image_1/" + base_name;
    
    // catch image read/write errors here
    try {

      // load left and right input image
		Mat left_img = imread(left_img_file_name,0);
		Mat right_img = imread(right_img_file_name,0);
		if (left_img.data == 0)
		{
			continue;
		}
		
//      png::image< png::gray_pixel > left_img(left_img_file_name);
//      png::image< png::gray_pixel > right_img(right_img_file_name);

      // image dimensions
//       int32_t width  = left_img.get_width();
//       int32_t height = left_img.get_height();
		int32_t width  = left_img.cols;
		int32_t height = left_img.rows;


      // convert input images to uint8_t buffer
      uint8_t* left_img_data  = (uint8_t*)malloc(width*height*sizeof(uint8_t));
      uint8_t* right_img_data = (uint8_t*)malloc(width*height*sizeof(uint8_t));
      int32_t k=0;
      for (int32_t v=0; v<height; v++) {
        for (int32_t u=0; u<width; u++) {
//           left_img_data[k]  = left_img.get_pixel(u,v);
//           right_img_data[k] = right_img.get_pixel(u,v);
			left_img_data[k]  = left_img.at<uchar>(v,u);
			right_img_data[k] = right_img.at<uchar>(v,u);
          k++;
        }
      }
      // status
      cout << "Processing: Frame: " << i;
      
      // compute visual odometry
      int32_t dims[] = {width,height,width};
      if (viso.process(left_img_data,right_img_data,dims)) 
	  {
		  Mat ImgDisp1(left_img.rows,left_img.cols,CV_8U,left_img.data);
		  Mat ImgDisp;
		  cvtColor(ImgDisp1,ImgDisp,CV_GRAY2BGR);
		  vector<Matcher::p_match> matches = viso.getMatches();
		  vector<int32_t> valid = viso.getInlierIndices();
		  /*for (int i = 0; i < valid.size(); i++)
		  {
			  Point2d pt0 = cvPoint(matches[valid[i]].u1p,matches[valid[i]].v1p);
			  Point2d pt1 = cvPoint(matches[valid[i]].u1c,matches[valid[i]].v1c);
			  line(ImgDisp,pt0,pt1,CV_RGB(0,255,0),2);
		  }*/
		  long nInd = 0;
		  for (int i = 0; i < matches.size(); i++)
		  {

			  Point2d pt0 = cvPoint(matches[i].u1p,matches[i].v1p);
			  Point2d pt1 = cvPoint(matches[i].u1c,matches[i].v1c);
			  if (i == valid[nInd] && nInd < valid.size()-1)
			  {
				  line(ImgDisp,pt0,pt1,CV_RGB(0,255,0),1);
				  nInd++;
			  }
			  else
			  {
				  line(ImgDisp,pt0,pt1,CV_RGB(255,0,0),1);
			  }

		  }
		  cvNamedWindow("123",0);
		  imshow("123",ImgDisp);

        // on success, update current pose
        pose = pose * Matrix::inv(viso.getMotion());
      
		std::vector<VisualOdometryStereo::MoveObject> VecMoveObj;
		viso.GetMoveObj(VecMoveObj);

		Point3f pt;
		FLOATX fT[16] = {};
		pose.getData(fT);
		pt.x = fT[3];
		pt.y = fT[7];
		pt.z = fT[11];
		Mat mm;
		cout << pt << endl;
//		pGL->AddPoints(pt,mm);

		vector<Matcher::p_match> VecMatchGood,VecMatchBad;
		long nInd1 = 0;
		for (int i = 0; i < matches.size(); i++)
		{
			if (i == valid[nInd1] && nInd1 < valid.size()-1)
			{
				VecMatchGood.push_back(matches[i]);
				nInd1++;
			}
			else
			{
				VecMatchBad.push_back(matches[i]);
			}
		}
		Rebuld.update(/*VecMatchGood*/matches,viso.getMotion(),2,3);
		vector<Reconstruction::point3d> VecPtRe = Rebuld.getPoints();
		printf("Total Cont:%d\n",VecPtRe.size());
		Mat CloudAdd = Mat::zeros(3,VecPtRe.size()-nCloudInd,CV_32F);
		for (int i = nCloudInd, j = 0; i < VecPtRe.size(); i++, j++)
		{
			CloudAdd.at<float>(0,j) = VecPtRe[i].x;
			CloudAdd.at<float>(1,j) = VecPtRe[i].y;
			CloudAdd.at<float>(2,j) = VecPtRe[i].z;
		}
		pGL->AddPoints2(pt,CloudAdd);
		nCloudInd = VecPtRe.size();
        // output some statistics
        double num_matches = viso.getNumberOfMatches();
        double num_inliers = viso.getNumberOfInliers();
        cout << ", Matches: " << num_matches;
        cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
        cout << pose << endl << endl;
		waitKey(1);
      } else {
        cout << " ... failed!" << endl;
      }

      // release uint8_t buffers
      free(left_img_data);
      free(right_img_data);

    // catch image read errors here
    } catch (...) {
      cerr << "ERROR: Couldn't read input files!" << endl;
      return 1;
    }
  }
  
  // output
  cout << "Demo complete! Exiting ..." << endl;
  waitKey();
  // exit
  return 0;
}

