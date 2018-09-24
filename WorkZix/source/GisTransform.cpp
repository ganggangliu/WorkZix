#include "GisTransform.h"


CGisTransform::CGisTransform()
{
	m_bIsSetBase = false;
}
CGisTransform::~CGisTransform()
{

}
void CGisTransform::SetBaseLocatin(double dLat, double dLon, double dHead)
{
	if (m_bIsSetBase)
	{
		return;
	}
	m_BaseHead = dHead;
	m_BaseLon = dLon;
	m_BaseLat = dLat;
	m_bIsSetBase = true;
}
Point2d CGisTransform::Wgs2Cart(Point2d pt)
{
	return GisWgsPoint2Point(Point2d(m_BaseLat,m_BaseLon), Point2d(pt.x,pt.y));
}

Point2d CGisTransform::Wgs2CartRotate(Point2d pt)
{
	return GisWgsPoint2PointRotate(Point2d(m_BaseLat,m_BaseLon), Point2d(pt.x,pt.y), m_BaseHead);
}

Point2d CGisTransform::CartRotate(Point2d pt)
{
	return GisCartPoint2PointRotate(Point2d(m_BaseLat,m_BaseLon), Point2d(pt.x,pt.y), m_BaseHead);
}

Point2d GisWgsPoint2Point(Point2d& pt0, Point2d& pt1)
{
	double dY = 111319.55*(pt1.x - pt0.x);
	double dX = 111319.55*(pt1.y - pt0.y)*cos(pt0.x/180.0*CV_PI);
	Point2d out(dX,dY);
	return out;
}

Point2d Point2GisWgsPoint(Point2d& pt0, Point2d& pt1)
{
	Point2d PtOut;
	PtOut.x = pt1.y/111319.55 + pt0.x;
	PtOut.y = pt1.x/111319.55/cos(pt0.x/180.0*CV_PI) + pt0.y;
	return PtOut;
}

double GisWgsPoint2PointDist(Point2d& pt0, Point2d& pt1)
{
	Point2d pt = GisWgsPoint2Point(pt0, pt1);
	double dDist = sqrt(pow(pt.x, 2) + pow(pt.y, 2));
	return dDist;
}
Point2d GisCartPoint2PointRotate(Point2d& pt0, Point2d& pt1, double dHeading)
{
	Point2d pt = pt1 - pt0;
	Point2d out;
	double dAng = dHeading/180.f*CV_PI;
	out.x = pt.x*cos(dAng) - pt.y*sin(dAng);
	out.y = pt.x*sin(dAng) + pt.y*cos(dAng);
	return out;
}
Point2d GisWgsPoint2PointRotate(Point2d& pt0, Point2d& pt1, double dHeading)
{
	Point2d pt = GisWgsPoint2Point(pt0, pt1);
	Point2d out;
	double dAng = dHeading/180.f*CV_PI;
	out.x = pt.x*cos(dAng) - pt.y*sin(dAng);
	out.y = pt.x*sin(dAng) + pt.y*cos(dAng);
	return out;
}