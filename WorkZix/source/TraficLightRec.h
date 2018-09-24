#ifndef TRAFICLIGHTREC_H
#define TRAFICLIGHTREC_H

#include <Windows.h>
#include "OpenCVInc.h"
#include "CalibSensor.h"

//using namespace cv;
using namespace std;

typedef struct tagTraficLightResult
{
	long nIndex;
	double dX;
	double dY;
	double dZ;
	Rect RecBox;
}TraficLightResult;

class CTraficLightRec:public CCamImuCalib
{
public:
	enum TargetDataSegment
	{
		TARGET_INDEX,
		TARGET_LATTITUDE,
		TARGET_LONGITUDE,
		TARGET_ALTITUDE,
		TARGET_RECT_WIDTH,
		TARGET_RECT_HIGHT
	};
	CTraficLightRec(void);
	
	//��ȡ��ͨ����Ϣ
	long LoadTraficLightInfo();

	//��ȡ��ͨ������
//	vector<TraficLightResult> TraficLightLocate(CGpsManager::GPS_IMU_INFO ImuData); 
	vector<TraficLightResult> TraficLightLocate(Mat& ImuData); 

	//Gps��Ϣ��ʽת��
//	Mat GpsData2Mat(CGpsManager::GPS_IMU_INFO ImuData);

	//������ͨ������
	long DrawRect(Mat& img, vector<TraficLightResult>& Result);

	//���ο������ᣬ���IMU�ռ�λ�ò���
	void DrawRefAxis(Mat& image, Mat& ImuData);

	//�Ե�ǰIMU��Ϊ�������ɾ�γ����������
	long DrawGrid(Mat& image, Mat& ptLocal);
	
	//���ο���
private:
// 	double GetDist(Mat& GpsInfo0, Mat& GpsInfo1);
// 	double GetDist(double lat0, double long0, double lat1, double long1);
	double rad(double d);
	CCamImuCalib m_CamImuCalib;
	vector<Mat> m_TraficLightInfo;
};

#endif