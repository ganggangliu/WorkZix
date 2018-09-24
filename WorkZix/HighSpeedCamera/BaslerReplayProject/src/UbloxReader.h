#ifndef UBLOX_READER_H
#define UBLOX_READER_H

#include <stdint.h>
#include <string>
#include <vector>
#include <fstream>

#include "OpenCVInc.h"

class CUbloxData
{
public:
	int64_t nSystemTime;	//us
	uint16_t nDataLen;
	uint32_t nGpsTimeOfWeek;//ms
	uint16_t nYear;			//UTC year
	uint8_t nMonth;
	uint8_t nDay;
	uint8_t nHour;
	uint8_t nMinite;
	uint8_t nSecond;
    uint8_t nValid;
	int32_t nNano;			//Fraction of second
	//0:no fix 1:Dead Reckoning only 2:2D fix 3:3D fix 
	//4:gps+reckoning combined 5:Time only fix
	uint8_t nGpsFix;		
    uint8_t nFlags;
    uint8_t nReserved;
	int32_t nLongitude;		//degree,1e-7
	int32_t nLatitude;		//degree,1e-7
	int32_t nHeight;		//mm, ellipsoid height
	int32_t nHMSL;			//mm, Mean Sea Level hight
	int32_t ngSpeed;		//mm/s, Ground speed
	int32_t nSpeed;			//mm/s, 3D speed
	int32_t nHeadMot;		//degree, Heading of motion, 1e-5
	int32_t	nHeadVeh;		//degree, Heading of vehicle, 1e-5
	uint8_t nCK_A;
	uint8_t nCK_B;
	CUbloxData()
	{
		memset(this, 0, sizeof(CUbloxData));
	}
};

class CUbloxReader
{
public:
	int ReadLog(std::string szLogPath);
	cv::Mat GetImageByTime(int64_t nTime, CUbloxData* pUbloxData = 0);
	cv::Mat GetImageByInd(int64_t nInd);
    int GetDataByTime(int64_t nTime, CUbloxData& Data);
	int Ublox2KmlFile(std::string szFilePath, cv::Scalar color);
	int Ublox2KmlFileWithPop(std::string szFilePath, cv::Scalar color);
	std::vector<CUbloxData> GetUbloxInfo();

private:
	int ParseLine(std::string szLine, CUbloxData& Data);
	std::string m_szLogPath;
	std::ifstream m_fs;
	std::vector<CUbloxData> m_VecBlox;
	cv::Mat m_Map;
	int64_t m_nMinLon;
	int64_t m_nMaxLon;
	int64_t m_nMinLat;
	int64_t m_nMaxLat;
	double m_dPixelPerLon;
	double m_dPixelPerLat;
};

#endif
