#ifndef RAW_DATA_READER_H
#define RAW_DATA_READER_H

#include <string>
#include <cstdint>
#include <stdio.h>
#include "OpenCVInc.h"
#include "FileOpr.h"

class CRawDataReaderParam
{
public:
	int nImgWidth;
	int nImgHeight;
	int nImgDepth;
	int nImgPattern;
	CRawDataReaderParam();
};

class CRawInfo
{
public:
	int64_t nInd;
	int64_t nIndFormDev;
	int64_t nSysTime;
};

class CRawDataReader
{
public:
	CRawDataReader();
	void Init(CRawDataReaderParam& Param);
	int Open(std::string szLogDir);
	int Open(std::string szImageFilePath, std::string szTimeLogPath);
	//nRot=1: rotate image clock-wise 2:counter clock-wise 0: not rotate
	//nWithStamp=0: with out stamp 1:with stamp
	int64_t GetFrameByIndex(int64_t nInd, cv::Mat& Img, int nRot = 0, int nWithStamp = 1);
//	int64_t GetFrameByTime(int64_t nTime, cv::Mat& Img);
	int64_t GetIndByTime(int64_t nTime);
	int64_t GetTimeByInd(int64_t nInd);
	int Close();
	int64_t GetStartTime();
//	int64_t GetStartTimeSafe();
	int64_t GetEndTimeSafe();
	int64_t GetAlignTime(int64_t nTime);
	void Reset();
	double GetImageEntropy(cv::Mat& image);

	int m_nFPSFromFileName;

public:
	int64_t GetRawFrameCont(std::string szRawFilePath);
	int ParseTimeLog(std::string szTimeLogPath);
	CRawDataReaderParam m_Param;
	cv::Mat m_buffer;
	int m_nFrameInd;
	std::vector<CRawInfo> m_VecTimeStamp;
	std::vector<std::string> m_VecTimeStr;
	int64_t m_nFrameSize;
	int64_t m_nFileLen;
	int64_t m_nTotalFrameCont;
	int64_t m_nValidFrameCont;
	CFileOpr m_FileOpr;
};

#endif