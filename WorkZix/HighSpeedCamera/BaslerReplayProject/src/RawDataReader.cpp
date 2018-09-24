#include "RawDataReader.h"
#include "Other.h"
#include <algorithm>

using namespace std;
using namespace cv;

CRawDataReaderParam::CRawDataReaderParam()
{
	nImgWidth = 800;
	nImgHeight = 600;
	nImgDepth = 1;
	nImgPattern = CV_BayerBG2RGB;
}

CRawDataReader::CRawDataReader()
{
	m_nFrameInd = 0;
	m_VecTimeStamp.clear();
	m_nValidFrameCont = 0;
	m_nFPSFromFileName = 20;
}

void CRawDataReader::Init(CRawDataReaderParam& Param)
{
	Reset();
	m_Param = Param;
	m_buffer = Mat::zeros(m_Param.nImgHeight, 
		m_Param.nImgWidth, CV_8UC(m_Param.nImgDepth));
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

// 		char sz0[128] = {0};
// 		char sz1[128] = {0};
// 		char sz2[128] = {0};
// 		sscanf(szLine.c_str(), "%[^-]-%[^-]-%[^-]", sz0, sz1, sz2);

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
    if (szLogDir.back() != '/' && szLogDir.back() != '\\')
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
	//Get frame rate from file name
	int nPos = TxtFiles[0].find("_30.txt", 0);
	if (nPos >= 0)
	{
		m_nFPSFromFileName = 30;
	}

	return Open(szLogDir+RawFiles[0], szLogDir+TxtFiles[0]);
}

int CRawDataReader::Open(std::string szImageFilePath, std::string szTimeLogPath)
{
	if (ParseTimeLog(szTimeLogPath) < 0)
		return -1;
    printf("%s\n", szTimeLogPath.c_str());
    printf("From %s\n", m_VecTimeStr.front().c_str());
    printf("To   %s\n",  m_VecTimeStr.back().c_str());

	if (!m_FileOpr.Open(szImageFilePath))
	{
		return -1;
	}

	m_nTotalFrameCont = m_FileOpr.GetFileSize()/m_nFrameSize;
	m_nValidFrameCont = std::min<int64_t>(m_nTotalFrameCont, m_VecTimeStamp.size());
	m_nFrameInd = 0;
	printf("%s :\n", szImageFilePath.c_str());
	printf("Frame count: %d \n", m_nTotalFrameCont);
	printf("Valid count: %d \n", m_nValidFrameCont);

	if (m_nValidFrameCont <= 10)
	{
		return -1;
	}

	return 1;
}

int64_t CRawDataReader::GetFrameByIndex(int64_t nInd, cv::Mat& Img, int nRot, int nWithStamp)
{
	int64_t nRt = nInd;
	if (nInd < 0)
	{
		nRt = -1;
	}
	if (nInd >= m_nValidFrameCont)
	{
		nRt = -2;
	}

	if (nRt < 0)
	{
        m_buffer = cvScalar(0);
	}
	else
	{
		int64_t nState = m_FileOpr.ReadData(m_buffer.data, m_nFrameSize, nInd * m_nFrameSize);
	}
	cvtColor(m_buffer, Img, m_Param.nImgPattern);

	if (nRot > 0)
	{
		Img = Img.t();
		if (nRot == 1)
		{
			flip(Img, Img, 0);
		}
		else
		{
			flip(Img, Img, 1);
		}
	}
	if (nWithStamp == 1)
	{
		if (nRt >= 0)
		{
			putText(Img, m_VecTimeStr[nRt], Point(0,25), 0, 0.8, CV_RGB(255,0,0), 2);
		}
		double dEntropy = GetImageEntropy(Img);
		char szFrame[256] = {0};
		sprintf(szFrame, "SMD:%.2f Frame:%lld/%lld", dEntropy, nInd, m_nValidFrameCont);
		putText(Img, szFrame, Point(0,Img.rows-5), 0, 0.8, CV_RGB(255,0,0), 2);
	}
	
//	m_nFrameInd = nInd;

	return nRt;
}

 int64_t CRawDataReader::GetIndByTime(int64_t nTime)
 {
 	int64_t nInd = -1;
 	for (size_t i = 1/*0*/; i < m_VecTimeStamp.size(); i++)
 	{
    	if (nTime > m_VecTimeStamp[i-1].nSysTime &&
    		nTime <= m_VecTimeStamp[i].nSysTime)
// 		if (nTime == m_VecTimeStamp[i].nSysTime)
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

 int64_t CRawDataReader::GetTimeByInd(int64_t nInd)
 {
	 if (m_VecTimeStamp.size() <= 0)
	 {
		 return -1;
	 }
	 else if (nInd+1 >= m_VecTimeStamp.size())
	 {
		 return m_VecTimeStamp.back().nSysTime;
	 }
	 else if (nInd <= 0)
	 {
		 return m_VecTimeStamp.front().nSysTime;
	 }
	 
	 return m_VecTimeStamp[nInd].nSysTime;
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
//	m_fs.close();
	m_FileOpr.Close();
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
	ifstream fs(szRawFilePath.c_str(), ios::in|ios::binary);
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

void CRawDataReader::Reset()
{
	m_nFrameInd = 0;
	m_VecTimeStamp.clear();
	m_VecTimeStr.clear();
	m_nFrameSize = 0;
	m_nFileLen = 0;
	m_nTotalFrameCont = 0;
	m_nValidFrameCont = 0;
	m_FileOpr.Close();
}

double CRawDataReader::GetImageEntropy(cv::Mat& image)
{
	Mat ImageGrey;
	if (image.channels() == 3)
	{
		cvtColor(image, ImageGrey, CV_RGB2GRAY);
	}
	else
	{
		image.copyTo(ImageGrey);
	}
//	ImageGrey = (Mat_<uchar>(2,2) << 1, 2, 3, 4);
	Mat Kernel0 = (Mat_<float>(1,2) << 1.f, -1.f);
	Mat Kernel1 = (Mat_<float>(2,1) << 1.f, -1.f);
	Mat Filtered0, Filtered1;
//	cout << ImageGrey << endl;
	filter2D(ImageGrey, Filtered0, Kernel0.depth(), Kernel0, Point(0,0));
	filter2D(ImageGrey, Filtered1, Kernel1.depth(), Kernel1, Point(0,0));
// 	cout << Filtered0 << endl;
// 	cout << Filtered1 << endl;

	return sum(abs(Filtered0).mul(abs(Filtered1))).val[0]/(ImageGrey.cols*ImageGrey.rows);

// 	const int histSize = 256;  
// 	float range[] = {0, 255};  
// 	const float *ranges[] = {range};  
// 	const int channels = 0;
// 	Mat histogram;
// 	calcHist(&image, 1, &channels, cv::Mat(), histogram, 1, &histSize, &ranges[0], true, false);
// 
// 	histogram /= image.rows * image.cols;
// 
// 	Mat log_histogram;
// 	log(histogram, log_histogram);
// 
// 	Scalar out = sum(histogram.mul(log_histogram));


//	return 1;
}
