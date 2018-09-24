#ifndef GIS_TRANSFORM_H
#define GIS_TRANSFORM_H

#include "OpenCVInc.h"

//calc 
//pt0，pt1为gps点，返回平面坐标系中pt1相对于pt0的坐标
cv::Point2d GisWgsPoint2Point(cv::Point2d& pt0, cv::Point2d& pt1);
cv::Point2d Point2GisWgsPoint(cv::Point2d& pt0, cv::Point2d& pt1);
cv::Point2d Point2GisWgsPointRotate(cv::Point2d& pt0, cv::Point2d& pt1, double dHeading);
double GisWgsPoint2PointDist(cv::Point2d& pt0, cv::Point2d& pt1);
cv::Point2d GisWgsPoint2PointRotate(cv::Point2d& pt0, cv::Point2d& pt1, double dHeading);
cv::Point2d GisCartPoint2PointRotate(cv::Point2d& pt0, cv::Point2d& pt1, double dHeading);

cv::Point2d Vehicle2Cart(cv::Point2d& ptVehicle, double dHeadingVehicle, cv::Point2d Target);


class CGisTransform
{
public:
	CGisTransform();
	~CGisTransform();
	void SetBaseLocatin(double dLat, double dLon, double dHead);
	cv::Point2d Wgs2Cart(cv::Point2d pt);
	cv::Point2d Wgs2CartRotate(cv::Point2d pt);
	cv::Point2d CartRotate(cv::Point2d pt);

private:
	double m_BaseLat;
	double m_BaseLon;
	double m_BaseHead;
	bool m_bIsSetBase;
};



#endif