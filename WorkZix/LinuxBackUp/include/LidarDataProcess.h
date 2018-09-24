#ifndef LIDAR_DATA_PROCESS_H
#define LIDAR_DATA_PROCESS_H

#include "OpenCVInc.h"
#include "list"

using namespace std;

class CLidarDataProcess
{
public:
	struct CLidarDataProcessParam
	{					
		double dVehicleWidthHalf;
		double dVehicleLen;
		double dPathChangeToleratePer;
        double dPathChangeDist;
        int nStableCont;
        int nStableValid;
		CLidarDataProcessParam()
		{
            dVehicleWidthHalf = 1500.f;
			dVehicleLen = 3000;
            dPathChangeToleratePer = 1.0;
            dPathChangeDist = 10;
            nStableCont = 3;
            nStableValid = 3;
		};
	};

	CLidarDataProcess();
	~CLidarDataProcess();

	void Init(CLidarDataProcessParam& Param);
    cv::Mat Grid(cv::Mat CloudPoint);
    int GetBestTrackEx(cv::Mat& Area, double& dWheelAngle, std::vector<cv::Point2d>& Track, cv::Mat& TrackImg);
    cv::Mat GetTrackImageEx(cv::Mat& Area, double dWheelAngle, std::vector<cv::Point2d>& VecTrack);
    int IterationTrackEx(cv::Mat& Area_i, cv::Point2d Anchor_i, double dDistPix_i, cv::Mat& Tracking_io, cv::Point2d& PtCur_io, double& dDirect_io);

private:
    cv::Point2d CarXY2ImageUV(cv::Point2d CarXY);
    cv::Point2i Car2Image(cv::Point2d PtCar);
    int GetStableDirect(int nDirect);
	long m_nGridFront;
	long m_nGridBack;
	long m_nGridSide;
	long m_nGridSize;
	CLidarDataProcessParam m_Param;
    std::list<int> m_StableStatistic;
};

#endif // CIBEOLIDAROPR_H
