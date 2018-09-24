#include "ImageEvaluate.h"

using namespace cv;

CImageEvaluate::CImageEvaluate()
{
	m_pOrbDet = 0;
}

CImageEvaluate::~CImageEvaluate()
{
	if (m_pOrbDet)
	{
		delete m_pOrbDet;
		m_pOrbDet = 0;
	}
}

int CImageEvaluate::Init(CImageEvaluateParam Param)
{
	m_Param = Param;
	if (m_pOrbDet)
	{
		delete m_pOrbDet;
		m_pOrbDet = 0;
	}

	m_pOrbDet = new ORB(m_Param.nFeatureCont, m_Param.fScaleFactor,
		m_Param.nLevels, m_Param.nEdgeThreshold, m_Param.nFirstLevel,
		m_Param.WTA_K, m_Param.nScoreType, m_Param.nPatchSize);

	m_Mask = Mat::zeros(600, 800, CV_8U);
	m_Mask(m_Param.Area) = 255;

	return 1;
}

int CImageEvaluate::Evaluate(const cv::Mat& image, cv::Mat& image_out, int& nFeatureCont, float& fAvgResponse)
{
	vector<KeyPoint> keypoints;
	Mat descriptors;
//	(*m_pOrbDet)(image, m_Mask, keypoints);
	FASTX(image,keypoints,m_Param.fFastThreshold,true,FastFeatureDetector::TYPE_9_16);
//	FAST(image,keypoints,20,true);

	KeyPointsFilter filter;
//	filter.runByPixelsMask(keypoints, m_Mask);
	filter.removeDuplicated(keypoints);
	filter.retainBest(keypoints, m_Param.nFeatureCont);

	cvtColor(image, image_out, CV_GRAY2RGB);
//	rectangle(image_out, m_Param.Area, CV_RGB(0,0,255));
	fAvgResponse = 0.f;
	for (unsigned int i = 0; i < keypoints.size(); i++)
	{
		fAvgResponse += keypoints[i].response;
		circle(image_out, keypoints[i].pt, 1, CV_RGB(255,0,0), -1);
	}
	if (keypoints.size() > 0)
	{
		fAvgResponse = fAvgResponse/keypoints.size();
	}

	char szDisp[256] = {0};
	sprintf(szDisp, "Point count:%d  Response:%.2f",
		keypoints.size(), fAvgResponse);
	putText(image_out, szDisp, Point(0, 25), 0, 1.0, CV_RGB(255,0,0), 2);

	nFeatureCont = keypoints.size();

	return 1;
}