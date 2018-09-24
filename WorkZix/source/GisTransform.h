#ifndef GISTRANSFORM_H
#define GISTRANSFORM_H

#include "OpenCVInc.h"
#include "LCM_POINT2D_F.hpp"

//calc 
//pt0，pt1为gps点，返回平面坐标系中pt1相对于pt0的坐标
Point2d GisWgsPoint2Point(Point2d& pt0, Point2d& pt1);
Point2d Point2GisWgsPoint(Point2d& pt0, Point2d& pt1);
double GisWgsPoint2PointDist(Point2d& pt0, Point2d& pt1);
Point2d GisWgsPoint2PointRotate(Point2d& pt0, Point2d& pt1, double dHeading);
Point2d GisCartPoint2PointRotate(Point2d& pt0, Point2d& pt1, double dHeading);


class CGisTransform
{
public:
	CGisTransform();
	~CGisTransform();
	void SetBaseLocatin(double dLat, double dLon, double dHead);
	Point2d Wgs2Cart(Point2d pt);
	Point2d Wgs2CartRotate(Point2d pt);
	Point2d CartRotate(Point2d pt);

private:
	double m_BaseLat;
	double m_BaseLon;
	double m_BaseHead;
	bool m_bIsSetBase;
};



#endif