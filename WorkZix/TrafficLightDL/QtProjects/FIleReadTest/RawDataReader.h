#ifndef RAW_DATA_READER_H
#define RAW_DATA_READER_H

#include "OpenCVInc.h"
#include <fstream>
#include <string>
#include <cstdint>
#include <stdio.h>

class CRawDataReaderParam
{
public:
	int nImgWidth;
	int nImgHeight;
	int nImgDepth;
	int nImgPattern;
	double dScale;
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
	int64_t GetFrameByIndex(int64_t nInd, cv::Mat& Img);
//	int64_t GetFrameByTime(int64_t nTime, cv::Mat& Img);
	int64_t GetIndByTime(int64_t nTime);
	int Close();
	int64_t GetStartTime();
//	int64_t GetStartTimeSafe();
	int64_t GetEndTimeSafe();
	int64_t GetAlignTime(int64_t nTime);

public:
	int64_t GetRawFrameCont(std::string szRawFilePath);
	int ParseTimeLog(std::string szTimeLogPath);
	CRawDataReaderParam m_Param;
	std::ifstream m_fs;
	FILE* m_fp;  
	cv::Mat m_buffer;
	int m_nFrameInd;
	std::vector<CRawInfo> m_VecTimeStamp;
	std::vector<std::string> m_VecTimeStr;
	int64_t m_nFrameSize;
	int64_t m_nFileLen;
	int64_t m_nTotalFrameCont;
	int64_t m_nValidFrameCont;
};

#endif