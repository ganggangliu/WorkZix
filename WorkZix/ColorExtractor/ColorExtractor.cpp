#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
using namespace std;
using namespace boost::filesystem;
#include "OpenCVInc.h"

unsigned char g_H_low = 0;
unsigned char g_H_up = 255;
unsigned char g_S_low = 0;
unsigned char g_S_up = 255;
unsigned char g_V_low = 0;
unsigned char g_V_up = 255;

int main(int argc, char* argv[])
{
	if (argc > 1)
	{
		g_H_low = atoi(argv[1]);
		g_H_up = atoi(argv[2]);
		g_S_low = atoi(argv[3]);
		g_S_up = atoi(argv[4]);
		g_V_low = atoi(argv[5]);
		g_V_up = atoi(argv[6]);
	}
	string fullpath = boost::filesystem::initial_path<boost::filesystem::path>().string(); 
	path full_path(fullpath, boost::filesystem::native);

	while(1)
	{
		directory_iterator item_begin(full_path);
		directory_iterator item_end;
		for ( ; item_begin != item_end;)
		{
			if (is_directory(item_begin->path()))
			{
				cout << item_begin->path().string() << endl;
				item_begin++;
				continue;
			}
			Mat Img = imread(item_begin->path().string());
			if (Img.data == 0)
			{
				cout << item_begin->path().string() << endl;
				item_begin++;
				continue;
			}
			cout << item_begin->path().string() << endl;
			blur(Img, Img, Size(5,5), Point(2,2));
			Mat OriBlur = Img.clone();
			Mat ImgRoiHsv;
			cvtColor(Img, ImgRoiHsv, CV_RGB2HSV);
			Mat ColorArea;
			inRange(ImgRoiHsv,Scalar(g_H_low,g_S_low,g_V_low),Scalar(g_H_up,g_S_up,g_V_up),ColorArea);

			Mat ColorAreaErode;
			erode(ColorArea, ColorAreaErode, Mat::ones(3,3,CV_8U), cvPoint(1,1));
			ColorArea = ColorAreaErode;

			vector<Vec4i> hierarchy;
			vector<vector<Point>> contours;
			findContours(ColorArea.clone(), contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

			Mat mat_contours = Mat::zeros(ColorArea.rows, ColorArea.cols, CV_8UC3);
			for (unsigned int i = 0; i < contours.size(); i++)
			{
				Scalar color( rand()&255, rand()&255, rand()&255 );
				drawContours(mat_contours, contours, i, color, CV_FILLED);
			}

// 			vector<Moments> mu(contours.size() );
// 			for(unsigned int i = 0; i < contours.size(); i++ )
// 			{ 
// 				mu[i] = moments( contours[i], false ); 
// 			}

			for (unsigned int i = 0; i < contours.size(); i++)
			{
				Rect rect_ = boundingRect(contours[i]);
				double dContourArea = contourArea(contours[i]);
				Scalar color = CV_RGB(0,255,0);
				if (dContourArea/(rect_.width*rect_.height) < 0.5)
				{
					color = CV_RGB(255,255,0);
				}
				if (dContourArea <= 10)
				{
					color = CV_RGB(255,0,0);
				}
				double dPer = (double)rect_.width/(double)rect_.height;
				if (dPer <= 0.5 || dPer >= 2.0)
				{
				 	color = CV_RGB(255,0,0);
				}
				rectangle(Img, rect_, color,2);
			}

			char szDisp[256] = {0};
			sprintf(szDisp, "%d %d %d %d %d %d", g_H_low, g_H_up, g_S_low, g_S_up, g_V_low, g_V_up);
			putText(Img, szDisp, Point(100,100),0,1,CV_RGB(255,255,255),3);
			putText(Img, szDisp, Point(100,200),0,1,CV_RGB(0,0,0),3);
			cvNamedWindow("mat_contours",0);
			imshow("mat_contours",mat_contours);
			cvNamedWindow("OriImg",0);
			imshow("OriImg",Img);
			cvNamedWindow("ColorArea",0);
			imshow("ColorArea",ColorArea);
			cvNamedWindow("OriBlur",0);
			imshow("OriBlur",OriBlur);
			imwrite("log\\mat_contours.bmp", mat_contours);
			imwrite("log\\OriImg.bmp", Img);
			imwrite("log\\ColorArea.bmp", ColorArea);
			int c = waitKey();
			if (c == '1')
				g_H_up++;
			if (c == 'q')
				g_H_up--;
			if (c == 'a')
				g_H_low++;
			if (c == 'z')
				g_H_low--;

			if (c == '2')
				g_S_up++;
			if (c == 'w')
				g_S_up--;
			if (c == 's')
				g_S_low++;
			if (c == 'x')
				g_S_low--;

			if (c == '3')
				g_V_up++;
			if (c == 'e')
				g_V_up--;
			if (c == 'd')
				g_V_low++;
			if (c == 'c')
				g_V_low--;

			if (c == 32)
			{
				item_begin++;
			}
		}
	}
	



	return 0;
}

