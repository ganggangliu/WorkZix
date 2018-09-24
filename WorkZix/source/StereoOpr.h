#ifndef STEREOOPR_H
#define STEREOOPR_H

#include <Windows.h>
#include "OpenCVInc.h"
#include <stdio.h>

typedef struct tag_StereoParam
{
	char szPathIni[MAX_PATH];			//�����ļ�·��
	char szPathIntrinsics[MAX_PATH];	//�ڲ�·��
	char szPathExtrinsics[MAX_PATH];	//���·��
	double dScale;						//ͼ�������ųߴ�
	double dRegUpPer;					//��Чͼ���Ϸ�����
	double dRegDownPer;					//��Чͼ���·�����
	double dRegLeftPer;					//��Чͼ���󷽱���
	double dRegRightPer;				//��Чͼ���ҷ�����
	int nImgHeight;						//ͼ���
	int nImgWidth;						//ͼ���
	int nSGBM_numberOfDisparities;		//SGBM����Ӳ�
	int nSGBM_SADWindowSize;			//SGBM������ߴ�
	int nSGBM_cn;						//SGBMͨ����
	int nSGBM_preFilterCap;				//SGBMԤ�˲����
	int nSGBM_minDisparity;				//SGBM��С�Ӳ�
	int nSGBM_uniquenessRatio;			//SGBMΨһ������ֵ
	int nSGBM_speckleWindowSize;		//SGBM...
	int nSGBM_speckleRange;				//SGBM...
	int nSGBM_disp12MaxDiff;			//SGBM...
	bool bSGBM_fullDP;					//SGBMȫ����
	int nSGBM_m_scale;					//SGBM�ߴ�
	int nSGBM_m_LidarWidth;				//Lidar��ʾ��ȣ�mm��
	int nSGBM_m_LidarDeep;				//Lidar��ʾ��ȣ�mm��
	int nSGBM_m_LidarGridSize;			//Lidar����ߴ磨pixel/m��
	int nSGBM_m_LidarHigh;				//Lidar��ʾ�߶ȣ�mm��
	long tag_StereoParam::LoadParam();
	tag_StereoParam(const char* pszPath=NULL, int nImgH=1200, int nImgW=1600);
}CStereoParam;

class CStereoOpr 
{
public:
	CStereoOpr(void);
	~CStereoOpr(void);
	
public:
	//��ʼ�����ã��������������ļ� StereoParam.ini\intrinsics.yml\extrinsics.yml
	long Init(CStereoParam Param);
	//����sgbm���ͼ��˫Ŀ�ϰ�����
	//ImgL_in��ImgR_in���������˫ĿͼƬ.
	//disp_out: �Ӳ�ͼ
	//Pillar_out:��ֵͼ�ϰ�������
	//PillarImg_out:�ϰ���������ɫ��ͼƬ
	//Ĭ�ϲ����ʺ���kitti���ݼ�
	long BuildPillar(Mat& ImgL_in, Mat& ImgR_in, Mat& disp_out, Mat& Pillar_in, Mat& PillarImg_out);
	Mat CalDispSGBM(Mat& ImgL_in, Mat& ImgR_in);
	//����ǰ��֡�Ӳ�ͼ���ƶ��ϰ�����
	//DispP_in:ǰһ֡�Ӳ�ͼ
	//DispN_in:��һ֡�Ӳ�ͼ
	//rvec,tvec:��֮֡�����ת��ƽ�ƾ���
	//Motion_out:��ֵͼ�ƶ�����
	long MotionDetect(Mat& DispP_in, Mat& DispN_in, Mat& rvec, Mat& tvec, Mat& Motion_out );
	//��matд���ļ�
	void saveXYZ1(const char* filename, const Mat& mat);
	//matlab�е�α��ɫ�任jet colorspace
	//mat_in:8u�Ҷ�ͼ
	//output:α��ɫͼƬ
	Mat Grey2FateColor(Mat& mat_in);
	//��ȡLidarͼƬ
	//Disp:input �Ӳ�ͼ
	//Pillar:input ��ֵ����PillarͼƬ
	//Lidar:output Lidar ͼƬ
	//output:Lidar cloud points
	Mat GetPillarPoints(Mat& Disp, Mat& Pillar, Mat& Lidar);
	//Remap stereo image pair
	//˫Ŀ����
	void Remap(Mat& ImgL_in, Mat& ImgR_in, Mat& ImgL_out, Mat& ImgR_out);
	//Calculate 3D points from stereo points of images
	//������ά�������
	Point3f SteImgPt2XYZ(Point2f ptL, Point2f ptR);
	//Get Camera matrix
	Mat GetCameraMat(){return m_cameraMatrix;};
	//˫Ŀ�����㼰��match������������
	Mat CalcCloudPoints(vector<DMatch>& matches, vector<KeyPoint>& keypoints0, vector<KeyPoint>& keypoints1);

	float m_f;
	float m_TT;
	float m_cx;
	float m_cy;
	StereoSGBM m_sgbm;
//	float m_scale;
private:
	//Get pillar area from disparity image
	long Pillarize(Mat& mat_in, Mat& mat_out);
	//Is it all zero mat?
	long IsAllZero(Mat& mat);
	long NPN(vector<KeyPoint>& ptTest, vector<KeyPoint>& pt0, vector<KeyPoint>& pt1, vector<DMatch>& matches, Mat& mask);
	
	CStereoParam m_StereoParam;
	Size m_imgSize;

	double m_dLidarWidth;
	double m_dLidarHigh;
	double m_dLidarDeep;
	double m_dLidarGridSize;

	Mat m_M1,m_D1,m_M2,m_D2;
	Mat m_R,m_T,m_R1, m_R2, m_P1, m_P2, m_Q;
	
	Mat m_map11;
	Mat m_map12;
	Mat m_map21;
	Mat m_map22;
	Rect m_ValidRect1,m_ValidRect2;
	Mat	m_cameraMatrix;
};

#endif