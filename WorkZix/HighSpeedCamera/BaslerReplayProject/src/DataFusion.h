#ifndef DATA_FUSION_H
#define DATA_FUSION_H

#include "RawDataReader.h"
#include "UbloxReader.h"
#include "LcmReceiver.h"
#include "GPSLCMDATA.hpp"

class CDataFusion
{
public:
	CDataFusion();
    int Init(std::string szProjctPath);
	int64_t GetFrameByInd(int64_t nInd, cv::Mat& Img);
	int64_t GetFrameByTime(int64_t nTime, cv::Mat& Img);
	int64_t GetTotalCount();
// 	float GetProcessPercent();
// 	int64_t GetIndByPercent(float fPer);
// 	int64_t AbsInd2RelInd(int64_t nAbsInd);
// 	int64_t RelInd2AbsInd(int64_t nRelInd);
	int AlignData();

public:
	int CastUbloxData2Navi(CUbloxData& UbloxData);
    std::string m_ProjPath;
	CRawDataReaderParam m_ParamL;
	CRawDataReaderParam m_ParamR;
	CRawDataReaderParam m_ParamF;
	CRawDataReader m_ReaderL;
	CRawDataReader m_ReaderR;
	CRawDataReader m_ReaderF;
	int64_t m_nFrameDiffR2L;
	int64_t m_nFrameDiffF2L;
	int64_t m_nCoverStartIndL;
	int64_t m_nCoverEndIndL;
	int64_t m_nCoverStartIndR;
	int64_t m_nCoverEndIndR;
	int64_t m_nCoverStartIndF;
	int64_t m_nCoverEndIndF;
	int64_t m_nCoverStartTime;
	int64_t m_nCoverEndTime;

	int64_t m_nAlignIndL;
	int64_t m_nAlignIndR;
	int64_t m_nAlignIndF;

	int64_t m_nCurId;
	cv::Mat m_Buffer;

	CUbloxReader m_UbloxReader;
	CLcmRevicer<gpslcmdata> m_LcmVehicle;
};


#endif