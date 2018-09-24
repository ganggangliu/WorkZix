#ifndef TRACK_PROCESS_H
#define TRACK_PROCESS_H

#include "OpenCVInc.h"
#include "UbloxReader.h"
#include "BaslerProcessor.h"

class CTrackProcessParam
{
public:
	cv::Point2d PlaceOffSet;
	double dAngleOffSet;
	double dFocus;
	CTrackProcessParam()
	{
		PlaceOffSet = cv::Point2d(0.0, 0.0);
		dAngleOffSet = 0.0;
		dFocus = 0.001/3;
	}
};

class CTrackData
{
public:
	uint64_t nInd;		//Message index
	uint64_t nSysTime;	//System time
	float fPixelX;		//Optical flow pixel
	float fPixelY;
	double dMoveX;		//Camera Ego move distance Unit:m
	double dMoveY;
	double dAngleDelta;	//Camera Ego angle change
	double dCamAngleCart;	//angle accumulate in cart
	double dCartX;		//Place in Cartesian
	double dCartY;
	double dLatitude;	//Global location
	double dLongitude;
	double dHeading;//Global heading
	double dStereoErr;
	CTrackData()
	{
		memset(this, 0, sizeof(CTrackData));
	}
};

class CTrackProcess
{
public:
	CTrackProcess();
	int Init(/*CTrackProcessParam Param*/);
	int Process(std::string szPath);

public:
	int AlignData(std::vector<CTrackData>& MotionL, std::vector<CTrackData>& MotionR,
		std::vector<CUbloxData>& UbloxInfo);
	int WriteResult(std::string szName, std::vector<CTrackData>& Motion, 
		cv::Scalar color);
	int ReadMotion(std::string szLog, std::vector<CTrackData>& Motion);
	int Motion2Track(std::vector<CTrackData>& Motion,
		std::vector<cv::Point3d>& Track,
		CTrackProcessParam Param,
		int64_t nStartInd);
	int CompareTrackAndUblox(std::vector<CTrackData>& Motion,
		std::vector<CUbloxData>& UbloxInfo,
		CTrackProcessParam& Param,
		int64_t nStartInd);
	//以左目为旋转中心，计算右目相对于左目的运动向量，计算左目的旋转角度
	int Mition2TrackStereo(std::vector<CTrackData>& MotionL,
		CTrackProcessParam ParamL,
		std::vector<CTrackData>& MotionR,
		CTrackProcessParam ParamR,
		std::vector<CUbloxData>& UbloxInfo,
		std::vector<CTrackData>& MotionStereo);
	//只用2个相机运动方向切线计算交点，并作为旋转中心
	int Mition2TrackStereoEx(std::vector<CTrackData>& MotionL,
		CTrackProcessParam ParamL,
		std::vector<CTrackData>& MotionR,
		CTrackProcessParam ParamR,
		std::vector<CUbloxData>& UbloxInfo,
		std::vector<CTrackData>& MotionStereo);
	//胡老师方法，只使用2个相机速度计算旋转中心
	int Mition2TrackStereoEx1(std::vector<CTrackData>& MotionL,
		CTrackProcessParam ParamL,
		std::vector<CTrackData>& MotionR,
		CTrackProcessParam ParamR,
		std::vector<CUbloxData>& UbloxInfo,
		std::vector<CTrackData>& MotionStereo);
	//使用左目速度和方向，及右目速度，计算旋转中心
	int Mition2TrackStereoEx2(std::vector<CTrackData>& MotionL,
		CTrackProcessParam ParamL,
		std::vector<CTrackData>& MotionR,
		CTrackProcessParam ParamR,
		std::vector<CUbloxData>& UbloxInfo,
		std::vector<CTrackData>& MotionStereo);
	CTrackProcessParam m_ParamL;
	CTrackProcessParam m_ParamR;
	cv::Point2d m_UbloxPlaceOffSet;
	double m_dUbloxHeadingOffSet;
	std::string m_szPath;
	CUbloxReader m_UbloxOpr;
	std::vector<CTrackData> m_MotionL;
	std::vector<CTrackData> m_MotionR;
	std::vector<CTrackData> m_MotionStereo;
	std::vector<cv::Point3d> m_TrackL;
	std::vector<cv::Point3d> m_TrackR;
	std::vector<cv::Point2d> m_UbloxInCart;
	int64_t m_nStartIndUblox;
	int64_t m_nStartIndTrackL;
	int64_t m_nStartIndTrackR;
	std::vector<double> m_DeltaHead;
};


#endif