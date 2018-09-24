#include "TraficLightRec.h"


CTraficLightRec::CTraficLightRec()
{
	m_CamImuCalib.CalibByFiles();
	return;
}

long CTraficLightRec::LoadTraficLightInfo()
{
	FileStorage fs("TraficLightInfo.yml", CV_STORAGE_READ);
	if(!fs.isOpened())
	{
		printf("Failed to open file %s\n","TraficLightInfo.yml");
		return 0;
	}

	m_TraficLightInfo.clear();
	int i = 0;
	while(1)
	{
		Mat TL;
		char szHead[MAX_PATH] = {0};
		sprintf(szHead,"TraficLight%d",i);
		fs[szHead] >> TL;
		if (TL.rows == 0 || TL.cols == 0)
		{
			break;
		}
		cout << TL << endl;
		m_TraficLightInfo.push_back(TL);
		i++;
	}
	printf("%d Trafic Lights Detected\n",m_TraficLightInfo.size());

	return 1;
}

//Mat CTraficLightRec::GpsData2Mat(CGpsManager::GPS_IMU_INFO ImuData)
//{
//	Mat ImuDataMat = (Mat_<double>(1,14) << 
//		ImuData.GPSTime,
//		ImuData.Heading,
//		ImuData.Pitch,
//		ImuData.Roll,
//		ImuData.Lattitude,
//		ImuData.Longitude,
//		ImuData.Altitude,
//		ImuData.Ve,
//		ImuData.Vn,
//		ImuData.Vu,
//		ImuData.NSV1,
//		ImuData.NSV2,
//		ImuData.Status,
//		ImuData.Type
//		);
//
//	return ImuDataMat;
//}

// vector<TraficLightResult> CTraficLightRec::TraficLightLocate(CGpsManager::GPS_IMU_INFO ImuData)
// {
// 	return TraficLightLocate(GpsData2Mat(ImuData));
// }

vector<TraficLightResult> CTraficLightRec::TraficLightLocate(Mat& ImuData)
{
	vector<TraficLightResult> ResultOut;
	Mat ImuDataMat = ImuData;
	for (int i = 0; i < m_TraficLightInfo.size(); i++)
	{
		TraficLightResult ResultTemp;
		Mat ptTarget = Mat::zeros(1,3,CV_64F);
		ptTarget.at<double>(0,0) = m_TraficLightInfo[i].at<double>(0,TARGET_LATTITUDE);
		ptTarget.at<double>(0,1) = m_TraficLightInfo[i].at<double>(0,TARGET_LONGITUDE);
		ptTarget.at<double>(0,2) = m_TraficLightInfo[i].at<double>(0,TARGET_ALTITUDE);
// 		cout << ImuDataMat << endl;
// 		cout << ptTarget << endl;
		Mat ptImg = m_CamImuCalib.LocaleAbsoluteEx(ImuDataMat,ptTarget);
//		cout << ptImg << endl;
// 		if (ptImg.at<double>(0,2) <= 0)
// 			continue;
		ptImg = ptImg/ptImg.at<double>(0,2);
		ResultTemp.nIndex = m_TraficLightInfo[i].at<double>(0,TARGET_INDEX);
		ResultTemp.dX = m_CamImuCalib.m_ptCamera.at<double>(0,0);
		ResultTemp.dY = m_CamImuCalib.m_ptCamera.at<double>(1,0);
		ResultTemp.dZ = m_CamImuCalib.m_ptCamera.at<double>(2,0);
		double f = m_CamImuCalib.GetCameraMatrix().at<double>(0,0);
		double dW = f*m_TraficLightInfo[i].at<double>(0,TARGET_RECT_WIDTH)/ResultTemp.dZ;
		double dH = f*m_TraficLightInfo[i].at<double>(0,TARGET_RECT_HIGHT)/ResultTemp.dZ;
		ResultTemp.RecBox.x = ptImg.at<double>(0,0) - dW/2;
		ResultTemp.RecBox.y = ptImg.at<double>(0,1) - dH/2;
		ResultTemp.RecBox.width = dW;
		ResultTemp.RecBox.height = dH;
		ResultOut.push_back(ResultTemp);
	}

	return ResultOut;
}

//double CTraficLightRec::GetDist(Mat& GpsInfo0, Mat& GpsInfo1)
//{
//	double dX = 111319.55*(GpsInfo0.at<double>(0,LATTITUDE) - GpsInfo1.at<double>(0,LATTITUDE));
//	double dY = 111319.55*(GpsInfo0.at<double>(0,LONGITUDE) - GpsInfo1.at<double>(0,LONGITUDE))*cos(rad(GpsInfo1.at<double>(0,LATTITUDE)));
//	double dDist = sqrt(pow(dX,2) + pow(dY,2));
//
//	return dDist;
//}
//
//double CTraficLightRec::GetDist(double lat0, double long0, double lat1, double long1)
//{
//	double dX = 111319.55*(lat0 - lat1);
//	double dY = 111319.55*(long0 - long1)*cos(rad(lat1));
//	double dDist = sqrt(pow(dX,2) + pow(dY,2));
//
//	return dDist;
//}

double CTraficLightRec::rad(double d)
{
	return d * 3.141592654 / 180.0;
} 

long CTraficLightRec::DrawRect(Mat& img, vector<TraficLightResult>& Result)
{
	for (int i = 0; i < Result.size(); i++)
	{
		rectangle(img,Result[i].RecBox,CV_RGB(0,255,0),2);
		char szText[MAX_PATH] = {0};
		sprintf(szText,"%.2f",Result[i].dZ);
		putText(img,szText,cvPoint(Result[i].RecBox.x+Result[i].RecBox.width,Result[i].RecBox.y+Result[i].RecBox.height),0,1,CV_RGB(255,0,0),2);
	}
	return 1;
}

void CTraficLightRec::DrawRefAxis(Mat& image, Mat& ImuData)
{
	m_CamImuCalib.DrawRefAxis(image,ImuData);
}

long CTraficLightRec::DrawGrid(Mat& image, Mat& ptLocal)
{
	return m_CamImuCalib.DrawGrid(image,ptLocal);
}