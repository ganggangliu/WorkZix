#include "RawDataReader.h"
#include "Ohter.h"
#include <algorithm>

using namespace std;
using namespace cv;

CRawDataReaderParam::CRawDataReaderParam()
{
	nImgWidth = 800;
	nImgHeight = 600;
	nImgDepth = 1;
	nImgPattern = CV_BayerBG2RGB;
	dScale = 1.0;
}

CRawDataReader::CRawDataReader()
{
	m_nFrameInd = 0;
	m_VecTimeStamp.clear();
	m_fp = 0;
	m_nValidFrameCont = 0;
}

void CRawDataReader::Init(CRawDataReaderParam& Param)
{
	m_Param = Param;
	if (m_Param.nImgDepth == 1)
	{
		m_buffer = Mat::zeros(m_Param.nImgHeight, m_Param.nImgWidth, CV_8U);
	}
	else if (m_Param.nImgDepth == 3)
	{
		m_buffer = Mat::zeros(m_Param.nImgHeight, m_Param.nImgWidth, CV_8UC3);
	}
	m_nFrameSize = m_Param.nImgHeight * m_Param.nImgWidth * m_Param.nImgDepth;
}

int CRawDataReader::ParseTimeLog(std::string szTimeLogPath)
{
    ifstream fs(szTimeLogPath.c_str());
	if (!fs.is_open())
	{
        printf("%s open failed!\n", szTimeLogPath.c_str());
		return -1;
	}
	while(1)
	{
		if (fs.eof())
		{
			break;
		}
		CRawInfo RawInfo;
		string szLine;
		getline(fs, szLine);
        string szSep = "-";
        vector<string> Items = string_split(szLine, szSep);
		if (Items.size() < 3)
		{
			break;
		}
		stringstream stream;
		stream << Items[0];
		stream >> RawInfo.nInd;
		stream.clear();
		stream << Items[1];
		stream >> RawInfo.nIndFormDev;
		stream.clear();
		stream << Items[2];
		stream >> RawInfo.nSysTime;
		m_VecTimeStamp.push_back(RawInfo);
		m_VecTimeStr.push_back(szLine);
	}

	printf("Checking data...\n");
	if (m_VecTimeStamp.size() < 10)
	{
		printf("Data check failed: m_VecTimeStamp.size() < 10\n");
		return -1;
	}
	for (size_t i = 0; i+1 < m_VecTimeStamp.size(); i++)
	{
		if (m_VecTimeStamp[i+1].nInd - m_VecTimeStamp[i].nInd != 1 ||
			m_VecTimeStamp[i+1].nIndFormDev - m_VecTimeStamp[i].nIndFormDev != 1)
		{
			printf("Data check failed in line %d\n", i);
			return -1;
		}
	}

	printf("Data check finish\n");

	return 1;
}

int CRawDataReader::Open(std::string szLogDir)
{
	printf("**************************************************\n");
	printf("Open Log: %s\n", szLogDir.c_str());
    if (szLogDir.back() != '/')
	{
        szLogDir += "/";
	}
	vector<string> RawFiles;
	getFiles(szLogDir, "raw", RawFiles);
	if (RawFiles.size() <= 0)
	{
		printf("%s*.raw not found!\n", szLogDir.c_str());
		return -1;
	}
	vector<string> TxtFiles;
	getFiles(szLogDir, "txt", TxtFiles);
	if (TxtFiles.size() <= 0)
	{
		printf("%s*.txt not found!\n", szLogDir.c_str());
		return -1;
	}

	return Open(szLogDir+RawFiles[0], szLogDir+TxtFiles[0]);
}

int CRawDataReader::Open(std::string szImageFilePath, std::string szTimeLogPath)
{
	if (ParseTimeLog(szTimeLogPath) < 0)
		return -1;
    printf("%s\n", szTimeLogPath.c_str());
    printf("From %s\n", m_VecTimeStr.front().c_str());
    printf("To %s\n",  m_VecTimeStr.back().c_str());

//	m_fp = fopen(szImageFilePath.c_str(), "rb");
//	if (!m_fp)
//	{
//		return -1;
//	}
//	fseek(m_fp, 0L, SEEK_END);
//	int64_t nEnd = _ftelli64(m_fp);
//	fseek(m_fp, 0L, SEEK_SET);
//	int64_t nStart = _ftelli64(m_fp);
//	m_nFileLen = nEnd - nStart;
//	m_nTotalFrameCont = m_nFileLen/m_nFrameSize;
//	fclose(m_fp);
	m_nTotalFrameCont = GetRawFrameCont(szImageFilePath);
	m_nValidFrameCont = std::min<int64_t>(m_nTotalFrameCont, m_VecTimeStamp.size());
	m_nFrameInd = 0;
	printf("%s :\n", szImageFilePath.c_str());
	printf("Frame count: %d \n", m_nTotalFrameCont);
	printf("Valid count: %d \n", m_nValidFrameCont);

	m_fs.open(szImageFilePath, ios::in|ios::binary);
	if (!m_fs.is_open())
	{
		return -1;
	}

	return 1;
}

int64_t CRawDataReader::GetFrameByIndex(int64_t nInd, cv::Mat& Img)
{
	if (nInd < 0)
	{
		return -2;
	}
	if (nInd >= m_nValidFrameCont)
	{
		return -1;
	}
// 	if(fseek(m_fp, nInd * m_nFrameSize, SEEK_SET) < 0)
// 		return -1;
// 	fread(m_buffer.data, m_nFrameSize, 1, m_fp);
	m_fs.seekg(nInd * m_nFrameSize);
	if (m_fs.eof())
		return -1;
	m_fs.read((char*)m_buffer.data, m_nFrameSize);
    if (m_fs.eof())
    {
        m_fs.clear();
    }
	cvtColor(m_buffer, Img, m_Param.nImgPattern);
	putText(Img, m_VecTimeStr[nInd], Point(0,20), 0, 1, CV_RGB(255,0,0), 2);
	if (m_Param.dScale != 1.0)
	{
		resize(Img, Img, Size(Img.cols*m_Param.dScale, Img.rows*m_Param.dScale));
	}
	m_nFrameInd = nInd;

	return m_nFrameInd;
}

 int64_t CRawDataReader::GetIndByTime(int64_t nTime)
 {
 	int64_t nInd = -1;
 	for (size_t i = 0; i < m_VecTimeStamp.size(); i++)
 	{
 		//if (nTime > m_VecTimeStamp[i-1].nSysTime &&
 		//	nTime <= m_VecTimeStamp[i].nSysTime)
 		if (nTime == m_VecTimeStamp[i].nSysTime)
 		{
 			nInd = i;
 			break;
 		}
 	}
 	if (nInd < 0)
 	{
 		return -1;
 	}
 
 	return nInd;
 }

// int64_t CRawDataReader::GetFrameByTime(int64_t nTime, cv::Mat& Img)
// {
// 	int64_t nInd = GetIndByTime(nTime);
// 	if (nInd < 0)
// 	{
// 		return -1;
// 	}
// 	return GetFrameByIndex(m_VecTimeStamp[nInd].nInd, Img);
// }

int CRawDataReader::Close()
{
	m_fs.close();
	return 1;
}

int64_t CRawDataReader::GetStartTime()
{
	if (m_VecTimeStamp.size() <= 0)
	{
		return -1;
	}

	return m_VecTimeStamp.front().nSysTime;
}

//int64_t CRawDataReader::GetStartTimeSafe()
//{
//	for (size_t i = 0; i+1 < m_VecTimeStamp.size(); i++)
//	{
//		if (m_VecTimeStamp[i].nSysTime != m_VecTimeStamp[i+1].nSysTime)
//		{
//			return m_VecTimeStamp[i+1].nSysTime;
//		}
//	}
//
//	return -1;
//}

int64_t CRawDataReader::GetEndTimeSafe()
{
	if (m_nValidFrameCont <= 0 || m_VecTimeStamp.size() <= 0)
	{
		return -1;
	}
	
// 	if (m_nValidFrameCont > m_VecTimeStamp.size())
// 	{
// 		return m_VecTimeStamp.back().nSysTime;
// 	}

	return m_VecTimeStamp[m_nValidFrameCont-1].nSysTime;
}

int64_t CRawDataReader::GetAlignTime(int64_t nTime)
{
	if (m_nValidFrameCont <= 0)
		return -1;
	for (size_t i = m_nValidFrameCont-1; i >= 0; i--)
	{
		if (m_VecTimeStamp[i].nSysTime <= nTime)
		{
			return m_VecTimeStamp[i].nSysTime;
		}
	}

	return -1;
}

int64_t CRawDataReader::GetRawFrameCont(std::string szRawFilePath)
{
	ifstream fs(szRawFilePath);
	if (!fs.is_open())
	{
		return -1;
	}

	fs.seekg(0);
	int64_t nTotalCont = 0;
	while(!fs.eof())
	{
		fs.read((char*)m_buffer.data, m_nFrameSize);
		nTotalCont++;
	}
	fs.close();
	
	return nTotalCont;
}
