#include "stdafx.h"
#include "GetMaxInnerRectYaoP.h"

using namespace std;
using namespace cv;

//相机拍摄投影仪投射原始图像
Mat getProjectorMappingSrcFrame(VideoCapture capture);

//手动选取棋盘图在投射面的四个顶点
vector<Point> manuallySelectCameraCapturedChessboardVertexes(Mat frame);

//自动获取最大内接矩形
//1、获取中心点、质心
//求取多边形的质心(centroid)与中心(core)
//数据源：相机照射的投影仪区域最外围的多边形形状
//质心：多边形力矩质心
//中心：多边形最小外接矩形中心
void findContoursSpecificPoint(Mat &frame, vector<vector<Point> > &contours, Point &centroid, Point &core, double edgeThreshold, bool drawFlag = false)
{
	if(frame.empty())
	{
		cerr << "input data error. please check out."<< endl;
		exit(-1);
	}

	Mat tempFrame;
	frame.copyTo(tempFrame);

	if(tempFrame.channels() > 1)
		cvtColor(tempFrame, tempFrame, CV_BGR2GRAY);
	Canny(tempFrame, tempFrame, edgeThreshold, 3*edgeThreshold);
	findContours(tempFrame, contours, RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	imshow("src", tempFrame);
	//采用最大轮廓
	//int maxArea = 0;
	//vector<Point> maxContour(1);
	//for(int i = 0; i < contours.size(); i++)
	//{
	//	long area = contourArea(contours[i]);
	//	if(maxArea < area)
	//	{
	//		maxArea = area;
	//		maxContour = contours[i];
	//	}
	//}
	//	cout << maxContour << endl;

	//rectangle(tempFrame, boundingRect(maxContour), Scalar(255));
	//imshow("", tempFrame);
	//waitKey();

	//最小外界矩形中心
	Rect r0 = boundingRect(Mat(contours[0]));
	//	r0 = Rect(r0.x, r0.y, r0.width, displayRatio * r0.width);//构建指定宽高比
	//	rectangle(frame, r0, Scalar(0, 255, 0), 2);
	//	Point point = *((Point *)&r0);					//将rect转换为Point
	core = Point(0.5*(r0.tl().x + r0.br().x), 0.5*(r0.tl().y + r0.br().y));

	//最大外围轮廓质心
	Point tempPoint = Point(0, 0);
	vector<vector<Point> >::iterator itc = contours.begin();
	while(itc != contours.end())
	{
		Moments mon = moments(Mat(*itc++));
		tempPoint = Point(mon.m10/mon.m00, mon.m01/mon.m00);
	}
	centroid = tempPoint;

	if(drawFlag)
	{
		circle(frame, centroid, 2, Scalar(0, 255, 0), 4);
		circle(frame, core, 2, Scalar(0, 0, 255), 4);
		drawContours(frame, contours, -1, Scalar(0, 0, 255), 2);
	}

	return;
}

//2、以一点为中心、向内收缩找到一个内接矩形
//2.1 判断点是否在轮廓内
bool isPointInPolygon(vector<Point> edgeContours, Point testPoint)
{
	int polySides = edgeContours.size();
	int i, j = polySides -1;

	bool  oddNodes = false;

	for (i=0;i<polySides; i++) 
	{
		Point edgePoint_i = edgeContours[i];
		Point edgePoint_j = edgeContours[j];

		int edgeX_i = edgePoint_i.x;
		int edgeY_i = edgePoint_i.y;
		int edgeX_j = edgePoint_j.x;
		int edgeY_j = edgePoint_j.y;

		int testY = testPoint.y;
		int testX = testPoint.x;

		if(((edgeY_i < testY && edgeY_j >= testY) || (edgeY_j < testY && edgeY_i >= testY)) && (edgeX_i <= testX || edgeX_j <= testX))
		{
			oddNodes ^=(edgeX_i + (testY - edgeY_i)/(edgeY_j - edgeY_i) *(edgeX_j - edgeX_i) < testX);
		}
		j=i;
	}

	return oddNodes;
}

//2.2 判断生成矩形是否在轮廓内
bool isRectangleInPolygon(Rect rectangle, vector<Point> contours)
{
	int rows = rectangle.height;
	int cols = rectangle.width;

	//先判断四个矩形点是否在多边形内
	Point leftTopPoint = rectangle.tl();
	Point leftDownPoint = Point(rectangle.tl().x, rectangle.br().y);
	Point rightTopPoint = Point(rectangle.br().x, rectangle.tl().y);
	Point rightDownPoint = rectangle.br();

	if( !isPointInPolygon(contours, leftTopPoint) || !isPointInPolygon(contours, leftDownPoint) 
		|| !isPointInPolygon(contours, rightTopPoint) || !isPointInPolygon(contours, rightDownPoint))
		return false;

	//在判断所有点是否都在多边形中
	for(int i = 0; i < rows; i += 10)
	{
		for(int j = 0; j < cols; j += 10)
		{
			Point testPoint = Point(rectangle.x+j, rectangle.y+i);

			bool flag = isPointInPolygon(contours, testPoint);
			if(true == flag)
			{
				break;
			}
			else
				return false;
		}
	}
	return true;
}

//判断在多边形A内以某点为中心的最大内接矩形B
//1、将初步内接矩形遍历邻域内，看是否还有更大的
//1.1 四个方向Rect
vector<Rect> moveNeighbourRectangle(Rect srcRect, int strideLength = 2)
{
	int x = srcRect.x;
	int y = srcRect.y;
	int width = srcRect.width;
	int height = srcRect.height;

	Rect leftRect = Rect(x - strideLength, y, width, height);
	Rect rightRect = Rect(x + strideLength, y, width, height);
	Rect upRect = Rect(x, y - strideLength, width, height);
	Rect downRect = Rect(x, y + strideLength, width, height);

	vector<Rect> tempRect(4);
	tempRect[0] = leftRect;
	tempRect[1] = rightRect;
	tempRect[2] = upRect;
	tempRect[3] = downRect;

	return tempRect;
}

//1.2 将矩形放大后是否还在轮廓内
//TODO：梳理下1.2与1.3关系
bool zoomInscribedRectangle(vector<Rect> rect, vector<Point> contours,  Rect & dstRect,  int length = 2) 
{
	Rect tempRect;
	vector<Rect> candidateRects;
	bool expandRectangleFlag = false; // 矩形扩张标记
	int falseNums = 0;

	for(int i = 0; i < rect.size(); i++)
	{
		bool result = isRectangleInPolygon(rect[i], contours);
		if(result == false)
		{
			falseNums++;
			continue;
		}

		Point center = 0.5*(rect[i].tl() + rect[i].br());
		int width = rect[i].width + length;
		int height = rect[i].height + length;
		bool isTrue = false;
		while(!expandRectangleFlag)
		{
			int x = center.x - width*0.5;
			int y = center.y - height*0.5;
			tempRect = Rect(x, y, width, height);
			isTrue = isRectangleInPolygon(tempRect, contours);
			if(isTrue == true)
			{
				width += length;
				height += length;
			}
			else
			{
				//TODO:保存最后一个tempRect
				candidateRects.push_back(tempRect);
				expandRectangleFlag = true;
			}
		}
	}

	if(falseNums == rect.size())
	{
		return false;
	}

	//对所有Rect进行面积筛选，输出最大面积
	int maxArea = 0;
	for(int i = 0; i < candidateRects.size(); i++)
	{
		if(candidateRects[i].area() > maxArea)
		{
			maxArea = candidateRects[i].area();
			dstRect = candidateRects[i];
		}
	}

	//	dstRect = candidateRects[0];
	if(dstRect == Rect(-1, -1, -1, -1))
		return false;
	else
		return true;
}

//1.3 在得到的新轮廓继续扩，如果为false停止扩充并输入Rect
void getMaxCandinateRect(Rect rect, vector<Point> contours,  Rect & dstRect,  int length = 1 )
{
	vector<Rect> tempRects =  moveNeighbourRectangle(rect);
	bool result = zoomInscribedRectangle(tempRects, contours,  dstRect);
	if(result == false)
	{
		dstRect = rect;
		return;
	}

	Rect tempRect = dstRect;
	bool endingFlag = false;
	while(!endingFlag)
	{
		vector<Rect> tempRects1 =  moveNeighbourRectangle(tempRect);
		bool tmpResult = zoomInscribedRectangle(tempRects1, contours,  tempRect);
		if(tmpResult == false)
		{
			endingFlag = true;
			dstRect = tempRect;

		}
	}
	cout << dstRect.x << ", " << dstRect.y << ", " << dstRect.width << ", " << dstRect.height << endl;
}

//2、得到初步最大内接矩形
void findMaxInscribedRectangle(const vector<Point> contours, Point center, Rect &dstRect, int reduceSteps = 2)
{
	vector<Rect> tempCandinateRects;
	//宽、高同时放缩
	Rect tmpRectangle = boundingRect(contours);

	bool isAllRectanglePointInPolygon = isRectangleInPolygon(tmpRectangle, contours);
	Rect tempRect = tmpRectangle;
	while(!isAllRectanglePointInPolygon)
	{
		tempRect.width = tempRect.width - reduceSteps*2;
		tempRect.height = tempRect.height - reduceSteps*2;
		tempRect.x = center.x - (tempRect.width/2 - reduceSteps);
		tempRect.y = center.y - (tempRect.height/2 - reduceSteps);

		isAllRectanglePointInPolygon = isRectangleInPolygon(tempRect, contours);
		if(true == isAllRectanglePointInPolygon)
		{
			//		dstRect = tempRect;
			//将第2步、第3步插入这里再重新筛选一次
			getMaxCandinateRect(tempRect, contours,  dstRect);
			//bool result = zoomInscribedRectangle(tempCandinateRects, contours, dstRect); 
			//if(!result)
			//	dstRect = tempRect;
		}
	}
}

//对生成的结果指定固定大小尺寸
//宽高比：
Rect getSpecifiedProportionRect(Rect rect, vector<Point> contours, double widthHeightRatio = 3/4.)
{
	int width = rect.width;
	int height = rect.height;
	Point center = 0.5 * (rect.tl() + rect.br());
	//构建初始比例矩形
	int recommendRatio;
	width > height ? recommendRatio = height : recommendRatio = width;

	int x = center.x - recommendRatio*0.5;
	int y = center.y - recommendRatio*0.5;
	Rect dstRect;
	Rect initalRect;
	if(widthHeightRatio >= 1)
		initalRect = Rect(x, y, recommendRatio, recommendRatio/widthHeightRatio);
	else if(widthHeightRatio < 1)
		initalRect = Rect(x, y, widthHeightRatio*recommendRatio, recommendRatio);
	getMaxCandinateRect(initalRect, contours,  dstRect);
	return dstRect;
}

//调用函数
//输入给定点，输出Rect
Rect getMaxInscribedRectangle(vector<Point>& contours, double recommendRatio)
{
	int maxX = 0;
	int maxY = 0;
	for(int i = 0; i < contours.size(); i++)
	{
		int x = contours[i].x;
		int y = contours[i].y;
		if(maxX < x)
			maxX = x;
		if(maxY < y)
			maxY = y;
	}

	//生成轮廓图
	Mat tmpFrame = Mat::zeros(maxY + 50, maxX + 50, CV_8UC1);
	vector<vector<Point> > tmpContours;
	tmpContours.push_back(contours);
	drawContours(tmpFrame, tmpContours, -1, Scalar(255));
/*	imshow("src", tmpFrame);*/

	//提取轮廓
	vector<vector<Point> > tContours;
	findContours(tmpFrame, tContours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	Rect tmpRect = boundingRect(tContours[0]);
	Point  core = Point(0.5*(tmpRect.tl().x + tmpRect.br().x), 0.5*(tmpRect.tl().y + tmpRect.br().y));

	//最大外围轮廓质心
	Point tempPoint = Point(0, 0);
	vector<vector<Point> >::iterator itc = tContours.begin();
	while(itc != tContours.end())
	{
		Moments mon = moments(Mat(*itc++));
		tempPoint = Point(mon.m10/mon.m00, mon.m01/mon.m00);
	}
	Point centroid = tempPoint;

	Rect dstRect;
	findMaxInscribedRectangle(tContours[0], centroid, dstRect, 2);
	dstRect = getSpecifiedProportionRect(dstRect, tContours[0], recommendRatio);

	rectangle(tmpFrame, dstRect, Scalar(255));
// 	imshow("", tmpFrame);
// 	waitKey();

	return dstRect;
}