#ifndef BASLER_PROCESSOR_H
#define BASLER_PROCESSOR_H

#include <iostream>
#include <vector>
#include "OpenCVInc.h"
#include "RawDataReader.h"
#include "UbloxReader.h"

#define BAD_MOTION_ cv::Point2f(3e8, 3e8)

enum MotionState
{
	CALC_NAOMAL = 0,

};

class CBaslerProParam
{
public:
	//Ignore the boundary area
	double dIgnorAreaPer;
	//The max count of Orb feature point detected
	int nOrbCont;
	//Orb detect threshold
	double dOrbDetThreshold;
	//Orb match Threshold
	double dMatchThreshold;
	//Run mode:
	//0(full speed mode):Preload (nPreLoadMaxCont frames of) data from disk to memory, and run in full speed
	//1(fast mode):Load data and run frame by frame, without display
	//2(slow mode):Load data and run frame by frame, with display
	int nRunMode;
	//It work when nRunMode=0;
	int nPreLoadMaxCont;
	CBaslerProParam()
	{
		dIgnorAreaPer = 1.0/3.0;
		nOrbCont = 50;
		dOrbDetThreshold = 5;
		dMatchThreshold = 80;
		nRunMode = 2;
		nPreLoadMaxCont = 5000;
	};
};

class CBaslerProcessor
{
public:
	void Init(CBaslerProParam Param);
	int Process(std::string szProPath);

private:
	cv::Point2f getMotion(std::vector<cv::KeyPoint>& pt0,
		std::vector<cv::KeyPoint>& pt1,
		std::vector<std::vector<cv::DMatch> >& matches,
		std::vector<cv::Point2f>& optFlowStart,
		std::vector<cv::Point2f>& optFlowEnd,
		std::vector<float>& optDist,
		std::vector<int>& optFLowStatus,
		cv::Point2f refMotion = BAD_MOTION_);
    cv::Point2f getMotionRansac(std::vector<cv::KeyPoint>& pt0,
        std::vector<cv::KeyPoint>& pt1,
        std::vector<std::vector<cv::DMatch> >& matches,
        std::vector<cv::Point2f>& optFlowStart,
        std::vector<cv::Point2f>& optFlowEnd,
        std::vector<float>& optDist,
        std::vector<int>& optFLowStatus,
        cv::Point2f refMotion = BAD_MOTION_);
    cv::Point2f getMotionKeep(std::vector<cv::KeyPoint>& pt0,
        std::vector<cv::KeyPoint>& pt1,
        std::vector<std::vector<cv::DMatch> >& matches,
        std::vector<cv::Point2f>& optFlowStart,
        std::vector<cv::Point2f>& optFlowEnd,
        std::vector<float>& optDist,
        std::vector<int>& optFLowStatus,
        cv::Point2f refMotion = BAD_MOTION_);
	cv::Mat drawOrbMatches(cv::Mat& img,
	 	std::vector<cv::Point2f>& optFlowStart,
	 	std::vector<cv::Point2f>& optFlowEnd,
	 	std::vector<float> optDist,
	 	std::vector<int>& optFLowStatus,
	 	cv::Point2f motion);
	cv::Mat drawOrbMatchesEx(cv::Mat& img0,
	 	cv::Mat& img1,
	 	std::vector<cv::Point2f>& optFlowStart,
	 	std::vector<cv::Point2f>& optFlowEnd,
	 	std::vector<float> optDist,
	 	std::vector<int>& optFLowStatus,
	 	cv::Point2f motion);
 	cv::Point2f GuessMotion(std::vector<cv::Point2f>& optFlowStart,
 		std::vector<cv::Point2f>& optFlowEnd,
 		std::vector<int>& optFLowStatus);
	cv::Point2f GuessMotionKmeans(std::vector<cv::Point2f>& optFlowStart,
		std::vector<cv::Point2f>& optFlowEnd,
		std::vector<int>& optFLowStatus);
	int ProcessSingle(std::string szDataPath, std::string szSavePath);
	CBaslerProParam m_Param;
	cv::Rect m_MaskRect;
	cv::Mat m_Mask;
	cv::gpu::GpuMat m_d_Mask;
	CRawDataReader m_Reader;
	std::string m_ProjPath;
	std::string m_LeftPath;
	std::string m_RightPath;
	std::string m_UbloxPath;
	std::string m_ResultSavePath;
	cv::gpu::ORB_GPU* m_pOrbDet;
	CUbloxReader m_UbloxOPr;
};

#endif
