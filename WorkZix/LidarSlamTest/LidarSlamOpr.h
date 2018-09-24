#ifndef LIDAR_SALM_OPR_H
#define LIDAR_SALM_OPR_H

#include "OpenCVInc.h"
#include <vector>
#include "LCM_SENSOR_FUSION_PACKAGE.hpp"

using namespace std;

class CLidarSlamOpr 
{
public:
	CLidarSlamOpr();
	~CLidarSlamOpr();

	void Init(char* pszTickPatten, double dDegreePerTick);
	int AlignLidarData(double* pDist, double* pAngle, int nDataLen);
	int AlignLidarDataIbeo(LCM_IBEO_CLOUD_POINTS* pIbeoPoints);
	Mat Polar2CartPoints(vector<double>& dDistAlign, vector<double>& dAngleAlign);
//	int Gridding();
	int FilterPoints(Mat& Points_in, Mat& Points_out, vector<vector<Point2d>>& Gates_o);
	Mat DrawPoints(Mat& Points);
	Mat DrawPolyArea(Mat& Points);
	Mat DrawGates(vector<vector<Point2d>>& Gates);

	Mat GetTrackImage(Mat& Area, double dWheelAngle);
	Mat GetTrackImageEx(Mat& Area, double dWheelAngle, vector<Point2d>& VecTrack);
	Mat GetBestTrackEx(Mat& Area, double& dWheelAngle);

	void TestTick(LCM_IBEO_CLOUD_POINTS* pIbeoPoints);
	void SaveTick();
	void TickRecord(int nTick);


	vector<vector<Point2d>> m_Gates;
	Mat m_imgGate;

	Mat m_Points;
	Mat m_PointsFilter;

	Mat m_PolyArea;
	Mat m_PolyAreaFilter;

	Mat m_imgPoints;
	Mat m_imgPointsErode;
	Mat m_imgPointsFilter;

	Mat m_imgPolyArea;
	Mat m_imgPolyAreaErode;
	Mat m_imgPolyAreaFilter;

	Mat m_imgTrack;

	double m_dVehicleWidthHalf;

private:
	vector<double> m_dDistAlign;
	vector<double> m_dAngleAlign;

	char m_szTickFile[256];
	vector<int> m_nTickStore;
	double m_dDegreePerTick;

	long m_nGridFront;
	long m_nGridBack;
	long m_nGridSide;
	long m_nGridSize;

	vector<double> m_dDistAlignFilter;
	vector<double> m_dAngleAlignFilter;
	double m_dGateWidth;
	int m_nFilterLen;

	int IterationTrack(Mat& Area_i, Point2i Anchor_i, double dDistPix_i, Mat& Tracking_io, Point2i& PtCur_io, int& nDirect_io);
	int IterationTrackEx(Mat& Area_i, Point2d Anchor_i, double dDistPix_i, Mat& Tracking_io, Point2d& PtCur_io, double& dDirect_io);
	Point2i Car2Image(Point2d PtCar);

	double m_dVehicleLen;
};

#endif