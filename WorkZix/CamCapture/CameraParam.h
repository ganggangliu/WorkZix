#pragma once
#include <Windows.h>

typedef struct tag_CameraParam
{
	int	nCamCont;					//������� 1����Ŀ 2��˫Ŀ
	char szMacLeft[MAX_PATH];		//��ƵԴ��ַ ��Ŀ
	char szMacRight[MAX_PATH];		//��ƵԴ��ַ ��Ŀ
	long nDataType;					//������Դ 0��camera  1��avi 2��pic doc
	long nImageRows;				//ͼ������
	long nImageCols;				//ͼ������
	long nInteTimeL;				//��ʼ�ع�L
	long nInteTimeR;				//��ʼ�ع�R
	long nCaptureDist;				//����GPSλ�ð�����ɼ������ף�0��ʱ�̲ɼ�
	long nGpsDeviceType;			//ͼƬ��λ��Ϣ��Դ 0��gps&Dgps  1��XW-9000��IMU��
	long nTrigerMode;				//�������ͣ�0���ⴥ���� 1���ڴ���
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
	// 	strcpy(szMacLeft,"D:\\��������\\�ձ�·����Ƶ\\camera20140908165300L.avi");//5952 2605 3746 5093 6785
	// 	strcpy(szMacRight,"D:\\��������\\�ձ�·����Ƶ\\camera20140908165300R.avi");//2777 1737 12687
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