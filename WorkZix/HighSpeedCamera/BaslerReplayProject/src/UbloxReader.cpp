#include <stdio.h>
#include "UbloxReader.h"
#include "Other.h"

using namespace std;
using namespace cv;

int CUbloxReader::ReadLog(std::string szLogPath)
{
	m_szLogPath = szLogPath;
	if (m_szLogPath.back() != '/' && m_szLogPath.back() != '\\')
	{
		m_szLogPath += "/";
	}
	vector<string> RawFiles;
	getFiles(m_szLogPath, "txt", RawFiles);
	if (RawFiles.size() <= 0)
	{
		printf("%s*.txt not found!\n", m_szLogPath.c_str());
		return -1;
	}
	m_fs.open(m_szLogPath+RawFiles[0]);
	if (!m_fs.is_open())
	{
		return -1;
	}

	while(!m_fs.eof())
	{
		string szLine;
		std::getline(m_fs, szLine);
		CUbloxData Data;
		if (ParseLine(szLine, Data) > 0)
		{
			m_VecBlox.push_back(Data);
		}
		else
		{
			//breack;
			continue;
		}
	}

	if (m_VecBlox.size() < 1)
	{
		return -2;
	}

	m_nMinLon = INT64_MAX;
	m_nMaxLon = INT64_MIN;
	m_nMinLat = INT64_MAX;
	m_nMaxLat = INT64_MIN;
	for (unsigned int i = 0; i < m_VecBlox.size(); i++)
	{
		if (m_nMinLon >= m_VecBlox[i].nLongitude)
			m_nMinLon = m_VecBlox[i].nLongitude;
		if (m_nMaxLon <= m_VecBlox[i].nLongitude)
			m_nMaxLon = m_VecBlox[i].nLongitude;
		if (m_nMinLat >= m_VecBlox[i].nLatitude)
			m_nMinLat = m_VecBlox[i].nLatitude;
		if (m_nMaxLat <= m_VecBlox[i].nLatitude)
			m_nMaxLat = m_VecBlox[i].nLatitude;
	}

	int64_t nCenterLon = (m_nMaxLon + m_nMinLon)/2;
	int64_t nCenterLat = (m_nMaxLat + m_nMinLat)/2;

	int64_t nDeltaLon = m_nMaxLon - m_nMinLon;
	int64_t nDeltaLat = m_nMaxLat - m_nMinLat;
	int64_t nDeltaMax = max(nDeltaLon, nDeltaLat);

	m_nMinLon -= (nDeltaMax/10+1);
	m_nMaxLon += (nDeltaMax/10+1);
	m_nMinLat -= (nDeltaMax/10+1);
	m_nMaxLat += (nDeltaMax/10+1);

	m_dPixelPerLon = 560.0/(double)(m_nMaxLon - m_nMinLon);
	m_dPixelPerLat = 480.0/(double)(m_nMaxLat - m_nMinLat);

	m_Map = Mat::zeros(480, 560, CV_8UC3);
	for (unsigned int i = 1; i < m_VecBlox.size(); i++)
	{
		Point pt0((m_VecBlox[i-1].nLongitude - m_nMinLon)*m_dPixelPerLon,
			(m_VecBlox[i-1].nLatitude - m_nMinLat)*m_dPixelPerLat);
		Point pt1((m_VecBlox[i].nLongitude - m_nMinLon)*m_dPixelPerLon,
			(m_VecBlox[i].nLatitude - m_nMinLat)*m_dPixelPerLat);
		line(m_Map, pt0, pt1, CV_RGB(255,255,255), 2);
	}

	return m_VecBlox.size();
}

int CUbloxReader::ParseLine(string szLine, CUbloxData& Data)
{
	char szHeader[1024] = {0};
	int64_t nSystemTime = 0;
	int nBinLen = 0;
	char szBinText[1024] = {0};

	int nRt = sscanf(szLine.c_str(), "%[^,],%lld,%d,%s", 
		&szHeader, &nSystemTime, &nBinLen, &szBinText);

 	if (nRt < 4)
 	{
 		return -1;
 	}

	string szBinTextT(szBinText);
	char szBin[60] = {0};
 	for (unsigned int i = 0; i < 60; i++)
 	{
		string szSub = szBinTextT.substr(2*i, 2);
		char szT[10];
 		sscanf(szSub.c_str(), "%X", &szT);
		memcpy(&(szBin[i]), szT, 1);
 	}

	uint8_t nCK_A = 0, nCK_B = 0;
	for (unsigned int i = 2; i < 58; i++)
	{
		nCK_A = nCK_A + uint8_t(szBin[i]);
		nCK_B = nCK_B + nCK_A;
	}


	Data.nSystemTime = nSystemTime;
	memcpy(&Data.nDataLen, &(szBin[4]), 2);
	memcpy(&Data.nGpsTimeOfWeek, &(szBin[6+0]), 4);
	memcpy(&Data.nYear, &(szBin[6+4]), 2);
	memcpy(&Data.nMonth, &(szBin[6+6]), 1);
	memcpy(&Data.nDay, &(szBin[6+7]), 1);
	memcpy(&Data.nHour, &(szBin[6+8]), 1);
	memcpy(&Data.nMinite, &(szBin[6+9]), 1);
	memcpy(&Data.nSecond, &(szBin[6+10]), 1);
	memcpy(&Data.nValid, &(szBin[6+11]), 1);
	memcpy(&Data.nNano, &(szBin[6+12]), 4);
	memcpy(&Data.nGpsFix, &(szBin[6+16]), 1);
	memcpy(&Data.nFlags, &(szBin[6+17]), 1);
	memcpy(&Data.nReserved, &(szBin[6+18]), 2);
	memcpy(&Data.nLongitude, &(szBin[6+20]), 4);
	memcpy(&Data.nLatitude, &(szBin[6+24]), 4);
	memcpy(&Data.nHeight, &(szBin[6+28]), 4);
	memcpy(&Data.nHMSL, &(szBin[6+32]), 4);
	memcpy(&Data.ngSpeed, &(szBin[6+36]), 4);
	memcpy(&Data.nSpeed, &(szBin[6+40]), 4);
	memcpy(&Data.nHeadMot, &(szBin[6+44]), 4);
	memcpy(&Data.nHeadVeh, &(szBin[6+48]), 4);
	memcpy(&Data.nCK_A, &(szBin[6+52]), 1);
	memcpy(&Data.nCK_B, &(szBin[6+53]), 1);

	if (nCK_A != Data.nCK_A || nCK_B != Data.nCK_B)
	{
		printf("Check sum faild! Blox system time:%lld\n", nSystemTime);
		return -1;
	}

	return 1;
}

Mat CUbloxReader::GetImageByTime(int64_t nTime, CUbloxData* pUbloxData)
{
	Mat out;
	m_Map.copyTo(out);
	if (m_VecBlox.size() < 1)
	{
        flip(out, out, 0);
		return out;
	}
	int64_t nInd = 0;
	if (nTime < m_VecBlox.front().nSystemTime)
	{
        flip(out, out, 0);
		return out;
	}
	else if (nTime > m_VecBlox.back().nSystemTime)
	{
        flip(out, out, 0);
		return out;
	}
	else
	{
		int64_t nMinInd = -1;
		int64_t nMinDist = INT64_MAX;
		for (unsigned int i = 0; i < m_VecBlox.size(); i++)
		{
			int64_t nDist = abs(m_VecBlox[i].nSystemTime-nTime);
			if (nDist <= nMinDist)
			{
				nMinDist = nDist;
				nMinInd = i;
			}
		}
		if (nMinInd == -1)
		{
			flip(out, out, 0);
			return out;
		}
		if (nMinDist >= 1000)
		{
			flip(out, out, 0);
			return out;
		}
		nInd = nMinInd;
	}
	
    Point pt((m_VecBlox[nInd].nLongitude - m_nMinLon)*m_dPixelPerLon,
        (m_VecBlox[nInd].nLatitude - m_nMinLat)*m_dPixelPerLat);
    circle(out, pt, 4, CV_RGB(255,0,0),-1);
    flip(out, out, 0);

	char szText[1024] = {0};
	sprintf(szText, "%06lld-%lld", nInd, m_VecBlox[nInd].nSystemTime);
	putText(out, szText, Point(0,25), 0, 1.0, CV_RGB(255,0,0), 2);
	sprintf(szText, "%02dkm/h", m_VecBlox[nInd].ngSpeed*3600/1000000);
	putText(out, szText, Point(400,470), 0, 1.0, CV_RGB(255,0,0), 2);
	sprintf(szText, "valid:%02X fix:%02X flags:%02X", m_VecBlox[nInd].nValid,
		m_VecBlox[nInd].nGpsFix, m_VecBlox[nInd].nFlags);
	putText(out, szText, Point(0,470), 0, 1.0, CV_RGB(255,0,0), 2);

	if (pUbloxData)
	{
		*pUbloxData = m_VecBlox[nInd];
	}

	return out;
}

cv::Mat CUbloxReader::GetImageByInd(int64_t nInd)
{
	Mat out;
	m_Map.copyTo(out);
	if (m_VecBlox.size() < 1 || nInd < 0 ||
		nInd >= m_VecBlox.size())
	{
		flip(out, out, 0);
		return out;
	}

	Point pt((m_VecBlox[nInd].nLongitude - m_nMinLon)*m_dPixelPerLon,
		(m_VecBlox[nInd].nLatitude - m_nMinLat)*m_dPixelPerLat);
	circle(out, pt, 4, CV_RGB(255,0,0),-1);
	flip(out, out, 0);

	char szText[1024] = {0};
	sprintf(szText, "%06lld-%lld", nInd, m_VecBlox[nInd].nSystemTime);
	putText(out, szText, Point(0,25), 0, 1.0, CV_RGB(255,0,0), 2);
	sprintf(szText, "%02dkm/h", m_VecBlox[nInd].ngSpeed*3600/1000000);
	putText(out, szText, Point(400,470), 0, 1.0, CV_RGB(255,0,0), 2);
	sprintf(szText, "valid:%02X fix:%02X flags:%02X", m_VecBlox[nInd].nValid,
		m_VecBlox[nInd].nGpsFix, m_VecBlox[nInd].nFlags);
	putText(out, szText, Point(0,470), 0, 1.0, CV_RGB(255,0,0), 2);

	return out;
}

int CUbloxReader::GetDataByTime(int64_t nTime, CUbloxData& Data)
{
    if (m_VecBlox.size() <= 0)
    {
        return -1;
    }
    if (nTime < m_VecBlox.front().nSystemTime)
    {
        return -1;
    }
    if (nTime > m_VecBlox.back().nSystemTime)
    {
        return -1;
    }
    int64_t nMinDist = INT64_MAX;
    int nMinInd = -1;
    for (unsigned int i = 0; i < m_VecBlox.size(); i++)
    {
        int64_t nDist = abs(m_VecBlox[i].nSystemTime - nTime);
        if (nDist <= nMinDist)
        {
            nMinDist = nDist;
            nMinInd = i;
        }
    }
    if (nMinInd == -1 || nMinDist >= 1000)//如果时间间隔小于1000ms，则认为找不到对应的UBLOX数据
    {
        return -1;
    }

    Data = m_VecBlox[nMinInd];

    return 1;
}

int CUbloxReader::Ublox2KmlFile(std::string szFilePath, cv::Scalar color)
{
	if (m_VecBlox.size() <= 0)
	{
		return -1;
	}

	vector<Point3d> Line;
	Line.reserve(m_VecBlox.size());
	for (unsigned int i = 0; i < m_VecBlox.size(); i++)
	{
		Line.push_back(Point3d(m_VecBlox[i].nLongitude*0.0000001,
			m_VecBlox[i].nLatitude*0.0000001, 0.0));
	}

	WriteKmlFile(szFilePath, "ublox", Line);

	return m_VecBlox.size();
}

int CUbloxReader::Ublox2KmlFileWithPop(std::string szFilePath, cv::Scalar color)
{
	if (m_VecBlox.size() <= 0)
	{
		return -1;
	}

	char szColor[64] = {0};
	sprintf(szColor, "ff%02x%02x%02x", uchar(color.val[0]), uchar(color.val[1]), uchar(color.val[2]));

	ofstream fsKlm(szFilePath);
	fsKlm << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << endl;
	fsKlm << "<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\" xmlns:kml=\"http://www.opengis.net/kml/2.2\" xmlns:atom=\"http://www.w3.org/2005/Atom\">" << endl;
	fsKlm << "<Document>" << endl;
	fsKlm << "<name>ublox.kml</name>" << endl;

	fsKlm << "<StyleMap id=\"msn_ylw-pushpin\">" << endl;
	fsKlm << "<Pair>" << endl;
	fsKlm << "<key>normal</key>" << endl;
	fsKlm << "<styleUrl>#sn_ylw-pushpin</styleUrl>" << endl;
	fsKlm << "</Pair>" << endl;
	fsKlm << "</StyleMap>" << endl;
	fsKlm << "<Style id=\"sn_ylw-pushpin\">" << endl;
	fsKlm << "<LineStyle>" << endl;
	//	fsKlm << "<color>ff0000ff</color>" << endl;
	fsKlm << "<color>";
	fsKlm << szColor;
	fsKlm << "</color>" << endl;
	fsKlm << "<width>2</width>" << endl;
	fsKlm << "</LineStyle>" << endl;
	fsKlm << "</Style>" << endl;

	fsKlm << "<Placemark>" << endl;
	fsKlm << "<name>ublox</name>" << endl;
	fsKlm << "<styleUrl>#msn_ylw-pushpin</styleUrl>" << endl;
	fsKlm << "<LineString>" << endl;
	fsKlm << "<extrude>1</extrude>" << endl;					//显示对地投影线
	fsKlm << "<tessellate>1</tessellate>" << endl;
	fsKlm << "<altitudeMode>absolute</altitudeMode>" << endl;	//以绝对高度显示
	fsKlm << "<coordinates>" << endl;

	for (unsigned int i = 0; i < m_VecBlox.size(); i++)
	{
		char szLine[1024] = {0};
		sprintf(szLine, "%.7f,%.7f,%.3f", m_VecBlox[i].nLongitude*1e-7,
			m_VecBlox[i].nLatitude*1e-7, m_VecBlox[i].nHMSL*1e-3);
		if (i != 0)
		{
			fsKlm << ",";
		}
		fsKlm << szLine;
	}
	fsKlm << endl;

	fsKlm << "</coordinates>" << endl;
	fsKlm << "</LineString>" << endl;
	fsKlm << "</Placemark>" << endl;

	//////////////////////////////////////////////////////////////////////////
	for (unsigned int i = 0; i < m_VecBlox.size(); i++)
	{
		fsKlm << "<Placemark>" << endl;
		fsKlm << "<name>id:" << i << "</name>" << endl;
		char szDescript[1024] = {0};
		sprintf(szDescript, "System time:%lldms\nGps day:%d-%d-%d\nGps seconds:%.3fs\nGps Time:%d:%d:%d.%03d\nLongitude:%.7fdegree\nLatitude:%.7fdegree\nHeight:%.3fm HMSL:%.3fm\nSpeed:%.2fm/s Speed3D:%.2fm/s\nHeadMot:%.2fdegree HeadVeh:%.2fdegree\nValid:%02X Fix:%02X Flags:%02X", 
			m_VecBlox[i].nSystemTime,
			m_VecBlox[i].nYear,
			m_VecBlox[i].nMonth,
			m_VecBlox[i].nDay,
			m_VecBlox[i].nGpsTimeOfWeek*0.001,
			m_VecBlox[i].nHour,
			m_VecBlox[i].nMinite,
			m_VecBlox[i].nSecond,
			(int)(m_VecBlox[i].nNano*1e-6),
			m_VecBlox[i].nLongitude*1e-7,
			m_VecBlox[i].nLatitude*1e-7,
			m_VecBlox[i].nHeight/1000.0,
			m_VecBlox[i].nHMSL/1000.0,
			m_VecBlox[i].ngSpeed/1000.0,
			m_VecBlox[i].nSpeed/1000.0,
			m_VecBlox[i].nHeadMot*1e-5,
			m_VecBlox[i].nHeadVeh*1e-5,
			m_VecBlox[i].nValid,
			m_VecBlox[i].nGpsFix,
			m_VecBlox[i].nFlags);
		fsKlm << "<description>" << szDescript << "</description>" << endl;
		fsKlm << "<Point>" << endl;
		fsKlm << "<altitudeMode>absolute</altitudeMode>" << endl;
		fsKlm << "<gx:drawOrder>1</gx:drawOrder>" << endl;
		sprintf(szDescript, "%7f,%7f,%.3f", m_VecBlox[i].nLongitude*1e-7,
			m_VecBlox[i].nLatitude*1e-7, m_VecBlox[i].nHMSL*1e-3);
		fsKlm << "<coordinates>" << szDescript << "</coordinates>" << endl;
		fsKlm << "</Point>" << endl;
		fsKlm << "</Placemark>" << endl;
	}
	//////////////////////////////////////////////////////////////////////////

	fsKlm << "</Document>" << endl;
	fsKlm << "</kml>" << endl;

	fsKlm.close();

	return m_VecBlox.size();
}

std::vector<CUbloxData> CUbloxReader::GetUbloxInfo()
{
	return m_VecBlox;
}