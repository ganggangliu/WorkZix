#pragma once
#include <Windows.h>

typedef struct tag_CameraParam
{
	int	nCamCont;					//相机个数 1：单目 2：双目
	char szMacLeft[MAX_PATH];		//视频源地址 左目
	char szMacRight[MAX_PATH];		//视频源地址 右目
	long nDataType;					//数据来源 0：camera  1：avi 2：pic doc
	long nImageRows;				//图像行数
	long nImageCols;				//图像列数
	long nInteTimeL;				//初始曝光L
	long nInteTimeR;				//初始曝光R
	long nCaptureDist;				//按照GPS位置按距离采集（毫米）0：时刻采集
	long nGpsDeviceType;			//图片定位信息来源 0：gps&Dgps  1：XW-9000（IMU）
	long nTrigerMode;				//触发类型，0：外触发， 1：内触发
	long nFrameRate;
	long LoadParams(char* psz)
	{
		nCamCont = GetPrivateProfileIntA("StereoParam","nCamCont",2,psz);
		char szTemp[MAX_PATH] = {0};
		GetPrivateProfileStringA("StereoParam","LeftCamMac","",szTemp,MAX_PATH,psz);
		strcpy(szMacLeft,szTemp);
		GetPrivateProfileStringA("StereoParam","RightCamMac","",szTemp,MAX_PATH,psz);
		strcpy(szMacRight,szTemp);
		nInteTimeL = GetPrivateProfileIntA("StereoParam","ExposTimeLeft",1000,psz);
		nInteTimeR = GetPrivateProfileIntA("StereoParam","ExposTimeRight",1000,psz);
		nDataType = GetPrivateProfileIntA("StereoParam","nDataType",0,psz);
		nCaptureDist = GetPrivateProfileIntA("StereoParam","nCaptureDist",0,psz);
		nGpsDeviceType = GetPrivateProfileIntA("StereoParam","nGpsDeviceType",0,psz);
		nTrigerMode = GetPrivateProfileIntA("StereoParam","nTrigerMode",0,psz);
		nFrameRate = GetPrivateProfileIntA("StereoParam","nFrameRate",20,psz);
		return 1;
	}
	tag_CameraParam::tag_CameraParam()
	{
		nCamCont = 2;
		strcpy(szMacLeft,"00-11-1C-F1-37-17");
		strcpy(szMacRight,"00-11-1C-F1-32-45");
	// 	strcpy(szMacLeft,"D:\\测试资料\\日本路测视频\\camera20140908165300L.avi");//5952 2605 3746 5093 6785
	// 	strcpy(szMacRight,"D:\\测试资料\\日本路测视频\\camera20140908165300R.avi");//2777 1737 12687
		nImageCols = 1600;
		nImageRows = 1200;
		nDataType = 0;
		nInteTimeL = 1000;
		nInteTimeR = 1000;
		nCaptureDist = 0;
		nGpsDeviceType = 1;
		nFrameRate = 20;
	}
}CCameraParam;