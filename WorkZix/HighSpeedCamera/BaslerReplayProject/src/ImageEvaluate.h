#ifndef IMAGE_EVALUATE_H
#define IMAGE_EVALUATE_H

#include "OpenCVInc.h"

class CImageEvaluateParam
{
public:
	cv::Rect Area;
	float fFastThreshold;
	int nFeatureCont;
	float fScaleFactor;
	int nLevels;
	int nEdgeThreshold;
	int nFirstLevel;
	int WTA_K;
	int nScoreType;
	int nPatchSize;
	CImageEvaluateParam()
	{
		Area = cv::Rect(800*1.0/3.0,600*1.0/3.0,800*1.0/3.0,600*1.0/3.0);
		fFastThreshold = 20.f;
		nFeatureCont = 10000;
		fScaleFactor = 1.2f;
		nLevels = 8;
		nEdgeThreshold = 31;
		nFirstLevel = 0;
		WTA_K = 2;
		nScoreType = cv::ORB::FAST_SCORE;
		nPatchSize = 31;
	}
};

class CImageEvaluate
{
public:
	CImageEvaluate();
	~CImageEvaluate();
	int Init(CImageEvaluateParam Param);
	int Evaluate(const cv::Mat& image, cv::Mat& image_out, int& nFeatureCont, float& fAvgResponse);

private:
	CImageEvaluateParam m_Param;
	cv::ORB* m_pOrbDet; 
	cv::Mat m_Mask;
};

#endif