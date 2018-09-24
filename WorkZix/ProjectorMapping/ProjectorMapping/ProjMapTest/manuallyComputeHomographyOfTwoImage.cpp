//�ֶ���������ͼ��֮��ĵ�Ӧ����
//#include <opencv2/opencv.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
//#include <iostream>
#include "OpenCVInc.h"

using namespace std;
using namespace cv;

int main()
{
	Mat cameraImage = imread("camera_.png");
	Mat projectorImage = imread("pattern_.png");
	namedWindow("camera", 0);
	imshow("camera", cameraImage);
	if(cameraImage.empty() || projectorImage.empty())
	{
		cout << "camera or projector source data error. Please check." << endl;
		cin.get();
		return -1;
	}

	vector<Point2f> cameraPoints(4);
	vector<Point2f> projectorPoints(4);
	vector<Point2f> calibProjectorPoints(4);

	//�ֶ�ȡ��
				//ԭʼͼ���Ӧ��
	cameraPoints[0] = Point(505, 140);	
	cameraPoints[1] = Point(382, 836);	
	cameraPoints[2] = Point(1446, 359);	
	cameraPoints[3] = Point(1492, 936);

	projectorPoints[0] = Point(0, 0);		//���Ͻ�
	projectorPoints[1] = Point(0, 1330);	//���½�
	projectorPoints[2] = Point(1830, 0);	//���Ͻ�
	projectorPoints[3] = Point(1830, 1330);	//���½�

	//ͶӰ��У�������Ӧ��
	calibProjectorPoints[0] = Point(487, 301);	//���Ͻ�
	calibProjectorPoints[1] = Point(487, 841);	//���½�
	calibProjectorPoints[2] = Point(1149, 301);	//���Ͻ�
	calibProjectorPoints[3] = Point(1149, 841);	//���½�


//	Mat image1= cameraImage(Rect(Point(0, 0), Point(1024, 768)));
//	Mat image2 =  projectorImage(Rect(Point(496, 155), Point(1440, 1019)));
	Mat homographyCamera2Projector = findHomography(projectorPoints, cameraPoints, CV_RANSAC );

// 	Mat point1 = (Mat_<double>(3, 1) << cameraPoints[0].x, cameraPoints[0].y, 1);
// 	Mat result = homographyCamera2Projector * point1;
// 	result /= result.at<double>(2, 0);

	Mat dstProjectorImage;
	Mat calibHomograhpyProjector2Camera = findHomography(projectorPoints, calibProjectorPoints,  CV_RANSAC);
	Mat H = homographyCamera2Projector.inv()*calibHomograhpyProjector2Camera;
	warpPerspective(projectorImage, dstProjectorImage, H, projectorImage.size(), CV_INTER_LINEAR);
	namedWindow("1", 0);
	imshow("1", dstProjectorImage);
	imwrite("dstProjectorImage.bmp", dstProjectorImage);
	waitKey(1);
	cout << calibHomograhpyProjector2Camera << endl;

	Mat frame;
	VideoCapture capture(1);
	capture.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
	if(!capture.isOpened())
		return -1;
	while(1)
	{
		capture >> frame;
		if(frame.empty())
			continue;
		namedWindow("camera", WINDOW_NORMAL);
		imshow("camera", frame);
		imwrite("cameratemp.png", frame);
		int nRt = waitKey(1);
		if (nRt == 's')
		{
			return 0;
		}
	}

	cin.get();
	return 0;
}