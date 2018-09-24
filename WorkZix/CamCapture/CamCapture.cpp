#pragma once
#include "DataCapture.h"
#include "StereoOpr.h"
#include "GpsManager.h"
#include "ProjectOpr.h"
#include "TraficLightRec.h"

using namespace cv;

long LoadParam(CCameraParam& CamParam);

int main(int argc, char** argv)
{
// 	CTraficLightRec TraficLightRec;
// 	TraficLightRec.LoadTraficLightInfo();
	/////////////////////////////////////////////////////////////////
	//Load params;
	CCameraParam CamParam;
	LoadParam(CamParam);
	//////////////////////////////////////////////////////////////////////////
	//GPS Triger
	CGpsManager GpsMgr;//((CGpsManager::GPS_MANAGER_TYPE)CamParam.nGpsDeviceType,(double)CamParam.nCaptureDist);
	GpsMgr.Init((CGpsManager::GPS_MANAGER_TYPE)CamParam.nGpsDeviceType,(double)CamParam.nCaptureDist);
	GpsMgr.Start();
	//////////////////////////////////////////////////////////////////////////
	//Prepare Path for save
	CProjectWriter ProjWriter;
	CProjectParam ProParam;
	ProjWriter.Init(ProParam);
	//////////////////////////////////////////////////////////////////////////
	//Start capture
	CDataCapture DataCapt(CamParam.nDataType);
	DataCapt.InitParam(&CamParam);
	DataCapt.Start();
	Mat OriMat[2];
	Mat DispMat[2];
	unsigned long nInd = 0;
	cvNamedWindow("img",0);
	bool bIsAutoSave = false;
	bool bIsAutoSaveVideo = false;
	long nIsCheckRight = 0;
	//////////////////////////////////////////////////////////////////////////
	//Recieve data
	while(1)
	{
		LCM_GPS_DATA ImuData;
		double dDistT;
		nIsCheckRight = GpsMgr.CheckMile(ImuData,dDistT);

		long nRt = DataCapt.GetLatestImagePair(&nInd,OriMat,50);
		if (nRt == 0)
		{
			Sleep(1);
			continue;
		}
		printf("%d\n",nInd);

		OriMat[LEFT].copyTo(DispMat[LEFT]);
		OriMat[RIGHT].copyTo(DispMat[RIGHT]);

		Mat GpsDataMat = (Mat_<double>(1,6) << ImuData.GPS_HEADING,
			ImuData.GPS_PITCH,
			ImuData.GPS_ROLL,
			ImuData.GPS_LATITUDE,
			ImuData.GPS_LONGITUDE,
			ImuData.GPS_ALTITUDE);
		//vector<TraficLightResult> Result = TraficLightRec.TraficLightLocate(GpsDataMat);
		//TraficLightRec.DrawRect(DispMat[LEFT],Result);
		//TraficLightRec.DrawRefAxis(DispMat[LEFT],GpsDataMat);
		//TraficLightRec.DrawGrid(DispMat[LEFT],GpsDataMat);

		Mat matC;
		matC.create(DispMat[LEFT].rows,DispMat[LEFT].cols*2,CV_8UC3);
		DispMat[LEFT].copyTo(matC(Range(0,1200),Range(0,1600)));
		if (CamParam.nCamCont == 2)
		{
			DispMat[RIGHT].copyTo(matC(Range(0,1200),Range(1600,3200)));
		}
		imshow("img",matC);
		char szkey = waitKey(1);
		//////////////////////////////////////////////////////////////////////////
		//Button Active
		if (szkey == 'j' || szkey == 'J')//加曝光
		{
			DataCapt.GetParam().nInteTimeL*=1.1;
			if (DataCapt.GetParam().nInteTimeL >= 1500)
				DataCapt.GetParam().nInteTimeL = 1500;
			DataCapt.SetInteTime(DataCapt.GetParam().nInteTimeL);
			printf("ExposureTime:%d\n",DataCapt.GetParam().nInteTimeL);
		}
		if (szkey == 'k' || szkey == 'K')//减曝光
		{
			DataCapt.GetParam().nInteTimeL/=1.1;
			if (DataCapt.GetParam().nInteTimeL <= 10)
				DataCapt.GetParam().nInteTimeL = 10;
			DataCapt.SetInteTime(DataCapt.GetParam().nInteTimeL);
			printf("ExposureTime:%d\n",DataCapt.GetParam().nInteTimeL);
		}
		if (szkey == 'S' ||szkey == 's')//手动单帧保存
		{
			ProjWriter.SaveImageManual(nInd,OriMat,ImuData);
		}
		if (szkey == 'a' || szkey == 'A')//自动保存开关
		{
			bIsAutoSave = !bIsAutoSave;
		}
		if (bIsAutoSave&&nIsCheckRight)
		{
			nIsCheckRight = 0;
			ProjWriter.SaveImageAuto(nInd,OriMat,ImuData);
		}
		if (szkey == 'v' || szkey == 'V')//自动保存开关
		{
			bIsAutoSaveVideo = !bIsAutoSaveVideo;
		}
		if (bIsAutoSaveVideo)
		{
			ProjWriter.SaveImageAutoVideo(nInd,OriMat,ImuData);
		}
		else
		{
			ProjWriter.ReleaseVideo();
		}
	}

	return 0;
}

long LoadParam(CCameraParam& CamParam)
{
	char szFilePath[MAX_PATH] = {0};
	GetModuleFileNameA(NULL,szFilePath,MAX_PATH);
	char* p = strrchr(szFilePath,'\\');
	*p = 0x00;
	strcat(szFilePath,"\\");
	strcat(szFilePath,"CamCapture.ini");
	CamParam.LoadParams(szFilePath);

	return 1;
}