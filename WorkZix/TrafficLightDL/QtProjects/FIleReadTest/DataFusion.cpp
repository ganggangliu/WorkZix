#include "DataFusion.h"

using namespace cv;

CDataFusion::CDataFusion()
{
	m_nFrameDiffR2L = 0;
	m_nFrameDiffF2L = 0;
	m_nCoverStartIndL = -1;
	m_nCoverEndIndL = -1;
	m_nCoverStartIndR = -1;
	m_nCoverEndIndR = -1;
	m_nCoverStartTime = -1;
	m_nCoverEndTime = -1;
	m_nCoverStartIndF = -1;
	m_nCoverEndIndF = -1;

	m_nAlignIndL = -1;
	m_nAlignIndR = -1;
	m_nAlignIndF = -1;
}

int CDataFusion::Init(std::string szProjctPath)
{
	m_ProjPath = szProjctPath;
    if (m_ProjPath.back() != '/')
	{
        m_ProjPath += "/";
	}
	CRawDataReaderParam Param;
	m_ReaderL.Init(Param);
	int nRt = m_ReaderL.Open(m_ProjPath+"left");
	if (nRt < 0)
	{
		return -1;
	}
	m_ReaderR.Init(Param);
	nRt = m_ReaderR.Open(m_ProjPath+"right");
	if (nRt < 0)
		return -1;

	CRawDataReaderParam ParamF;
	ParamF.nImgHeight = 480;
	ParamF.nImgWidth = 640;
	ParamF.nImgPattern = CV_BayerRG2RGB;
	m_ReaderF.Init(ParamF);
	int nRtF = m_ReaderF.Open(m_ProjPath+"front");
	if (nRtF < 0)
		return -1;

	printf("**************************************************\n");
	int64_t nStartTimeL = m_ReaderL.GetStartTime();
//	int64_t nEndTimeL = m_ReaderL.GetEndTimeSafe();
	int64_t nStartTimeR = m_ReaderR.GetStartTime();
//	int64_t nEndTimeR = m_ReaderR.GetEndTimeSafe();
	int64_t nStartTimeF = m_ReaderF.GetStartTime();
//	int64_t nEndTimeF = m_ReaderF.GetEndTimeSafe();

 	m_nCoverStartTime = max(nStartTimeL, nStartTimeR);
	m_nCoverStartTime = max(m_nCoverStartTime, nStartTimeF);
// 	m_nCoverEndTime = min(nEndTimeL, nEndTimeR);

	m_nCoverStartIndL = m_ReaderL.GetIndByTime(m_nCoverStartTime);
//	m_nCoverEndIndL = m_ReaderL.GetIndByTime(nEndTimeL);
	m_nCoverStartIndR = m_ReaderR.GetIndByTime(m_nCoverStartTime);
//	m_nCoverEndIndR = m_ReaderR.GetIndByTime(nEndTimeR);
	m_nCoverStartIndF = m_ReaderF.GetIndByTime(m_nCoverStartTime);
//	m_nCoverEndIndF = m_ReaderF.GetIndByTime(nEndTimeF);

	m_nFrameDiffR2L = m_nCoverStartIndR - m_nCoverStartIndL;

	m_nFrameDiffF2L = m_nCoverStartIndF - m_nCoverStartIndL;

	m_nCurId = m_nCoverStartIndL;

	m_Buffer = Mat::zeros(m_ReaderL.m_Param.nImgHeight*2, m_ReaderL.m_Param.nImgWidth*2, CV_8UC3);
	
	Point ptText(m_ReaderL.m_Param.nImgWidth, m_ReaderL.m_Param.nImgHeight);
	putText(m_Buffer, "'A':step backward", ptText+Point(0,20), 0, 1.0, CV_RGB(255,0,0), 2);
	putText(m_Buffer, "'D':step forward", ptText+Point(0,50), 0, 1.0, CV_RGB(255,0,0), 2);
	putText(m_Buffer, "Space: Auto run/stop", ptText+Point(0,80), 0, 1.0, CV_RGB(255,0,0), 2);

	if (AlignData() < 0)
		return -1;

	return 1;
}

int64_t CDataFusion::GetFrameByInd(int64_t nInd, cv::Mat& Img)
{
	int64_t nIndL = nInd;
	if (nIndL <= 0)
		nIndL = 0;
	if (nIndL >= m_ReaderL.m_nValidFrameCont)
		nIndL = m_ReaderL.m_nValidFrameCont - 1;
	int64_t nIndR = (nIndL - m_nAlignIndL) + m_nAlignIndR;
	int64_t nIndF = floor((double)(nIndL - m_nAlignIndL)/25.0) + m_nAlignIndF;

	Mat imgL;
	int64_t nRtL = m_ReaderL.GetFrameByIndex(nIndL, imgL);
	if (nRtL < 0)
	{
		return nRtL;
	}
	Mat imgR;
	int64_t nRtR = m_ReaderR.GetFrameByIndex(nIndR, imgR);
	if (nRtR < 0)
	{
		imgR = Mat::zeros(m_ReaderR.m_Param.nImgHeight, m_ReaderR.m_Param.nImgWidth, CV_8UC3);
	}
	Mat imgF;
	int64_t nRtF = m_ReaderF.GetFrameByIndex(nIndF, imgF);
	if (nRtF < 0)
	{
		imgF = Mat::zeros(m_ReaderF.m_Param.nImgHeight, m_ReaderF.m_Param.nImgWidth, CV_8UC3);
	}
	resize(imgF, imgF, cv::Size(imgL.cols, imgL.rows));

	imgL.copyTo(m_Buffer(Range(0,imgL.rows),Range(0,imgL.cols)));
	imgR.copyTo(m_Buffer(Range(0,imgL.rows),Range(imgL.cols,2*imgL.cols)));
	imgF.copyTo(m_Buffer(Range(imgL.rows,imgL.rows*2),Range(0,imgL.cols)));

	Img = m_Buffer;

	m_nCurId = nRtL;

	return nRtL;
}

int64_t CDataFusion::GetFrameByTime(int64_t nTime, cv::Mat& Img)
{
	int64_t nIndL = m_ReaderL.GetIndByTime(nTime);

	return GetFrameByInd(nIndL, Img);
}

int64_t CDataFusion::GetTotalCount()
{
	return m_ReaderL.m_nValidFrameCont;
}

// float CDataFusion::GetProcessPercent()
// {
//	return (float)m_nCurId/(float)(m_nCoverEndIndL - m_nCoverStartIndL);
//}
//
//int64_t CDataFusion::GetIndByPercent(float fPer)
//{
//	return fPer*(m_nCoverEndIndL - m_nCoverStartIndL) + m_nCoverStartIndL;
//}

// int64_t CDataFusion::AbsInd2RelInd(int64_t nAbsInd)
// {
// 	return (nAbsInd - m_nCoverStartIndL);
// }
// 
// int64_t CDataFusion::RelInd2AbsInd(int64_t nRelInd)
// {
// 	return (m_nCoverStartIndL + nRelInd);
// }

int CDataFusion::AlignData()
{
	int64_t nEndTimeL = m_ReaderL.GetEndTimeSafe();
	int64_t nEndTimeR = m_ReaderR.GetEndTimeSafe();

	int64_t nAlignEndTimeLR = min(nEndTimeL, nEndTimeR);
	int64_t nAlignEndTimeFLR = m_ReaderF.GetAlignTime(nAlignEndTimeLR);
	if (nAlignEndTimeFLR < 0)
	{
		return -1;
	}

	m_nAlignIndL = m_ReaderL.GetIndByTime(nAlignEndTimeFLR);
	m_nAlignIndR = m_ReaderR.GetIndByTime(nAlignEndTimeFLR);
	m_nAlignIndF = m_ReaderF.GetIndByTime(nAlignEndTimeFLR);
	if (m_nAlignIndL < 0 ||m_nAlignIndR < 0 ||m_nAlignIndF < 0)
	{
		return -1;
	}

	printf("m_nAlignIndL:%lld m_nAlignIndR:%lld m_nAlignIndF:%lld\n",
		m_nAlignIndL, m_nAlignIndR, m_nAlignIndF);

	return 1;
}
