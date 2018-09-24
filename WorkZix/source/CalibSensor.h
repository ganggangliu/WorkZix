#ifndef CALIBSENSOR_H
#define CALIBSENSOR_H

#include <Windows.h>
#include "OpenCVInc.h"

using namespace std;

class CCalibSensor 
{
public:
	CCalibSensor(void);
	
};

//CCamImuCalib
//�ο�������˵����
//�����������X����ǰZ������Y
//�ߵ�����ϵ����X����ǰY������Z
class CCamImuCalib
{
public:
	CCamImuCalib();
	~CCamImuCalib();

	enum ImuDataSegment
	{
		HEADING,
		PITCH,
		ROLL,
		LATITUDE,
		LONGTITUDE,
		ALTITUDE
	};

public:
	//ʹ��CameraImuCalib.yml�ڵ����ݽ��б궨�����ɱ궨����ļ�extrinsicsCamAndImu.yml
	long CalibByFiles();

	////װ������ڲ��Լ���ߵ������extrinsicsCamAndImu.yml
	//long LoadParams();

	//ImuData����ǰIMU���ݣ�����һ�и߾��ȶ�λ����,���ݸ�ʽ����ImuDataSegment �磺128.99,2.0,3.0,30.4093913,114.4425757,19.8800000
	//Target��Ŀ����ľ������꣬ ÿһ����һ��Ŀ��������꣬�磺30.4093509,114.4426281,19.88
	//����ͼ���е�����
	Mat LocaleAbsolute(Mat& ImuData, Mat& ptTarget);
	//LocaleAbsolute�ֽ��
	Mat LocaleAbsoluteEx(Mat& ImuData, Mat& ptTarget);

	//Target��Ŀ���������IMU�����꣬ ����ͼ���е�����
	Mat LocaleRelative(Mat& ptTarget);

	//���ο������ᣬ���IMU�ռ�λ�ò���
	void DrawRefAxis(Mat& image, Mat& ImuData);

	//�Ե�ǰIMU��Ϊ�������ɾ�γ����������
	long DrawGrid(Mat& image, Mat& ImuData);

	//��Ŀ���
	long DrawAxis(Mat& image, Mat& ptTarget);

	//��ȡ�������
	Mat GetCameraMatrix(){return m_cameraMatrix;};
	Mat m_ptCart;
	Mat m_ptImu;
	Mat	m_ptCamera;
	Mat	m_ptImage;
private:
	double rad(double d);
	Mat Gps2Plant(Mat ptLocal, Mat ptTarget);

	//WGS48ת=�����ֱ������ϵ
	Mat WGS48ToCart(Mat ptLocal, Mat ptTarget);

	//���ֱ������=���ߵ�����
	Mat CartToImu(Mat ImuData, Mat Targets);

	//�ߵ�����ϵ=���������ϵ
	Mat ImuToCamera(Mat CalibCam2Imu, Mat Targets);

	//�������ϵ=����������ϵ
	Mat CameraToImage(Mat CamMatrix, Mat targets);

	Mat m_cameraMatrix;
	Mat m_rvec0;
	Mat m_tvec0;
	Mat m_RTImu2Cam;
	Mat m_RefHight2Ground;

	int m_nGridSize;
};		

#endif