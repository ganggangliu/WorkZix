#ifndef STEREOOPR_H
#define STEREOOPR_H

#include <Windows.h>
#include "OpenCVInc.h"
#include <stdio.h>

typedef struct tag_StereoParam
{
	char szPathIni[MAX_PATH];			//配置文件路径
	char szPathIntrinsics[MAX_PATH];	//内参路径
	char szPathExtrinsics[MAX_PATH];	//外参路径
	double dScale;						//图像处理缩放尺寸
	double dRegUpPer;					//有效图像上方比例
	double dRegDownPer;					//有效图像下方比例
	double dRegLeftPer;					//有效图像左方比例
	double dRegRightPer;				//有效图像右方比例
	int nImgHeight;						//图像高
	int nImgWidth;						//图像宽
	int nSGBM_numberOfDisparities;		//SGBM最大视差
	int nSGBM_SADWindowSize;			//SGBM遍历框尺寸
	int nSGBM_cn;						//SGBM通道数
	int nSGBM_preFilterCap;				//SGBM预滤波宽度
	int nSGBM_minDisparity;				//SGBM最小视差
	int nSGBM_uniquenessRatio;			//SGBM唯一比例阈值
	int nSGBM_speckleWindowSize;		//SGBM...
	int nSGBM_speckleRange;				//SGBM...
	int nSGBM_disp12MaxDiff;			//SGBM...
	bool bSGBM_fullDP;					//SGBM全方向
	int nSGBM_m_scale;					//SGBM尺寸
	int nSGBM_m_LidarWidth;				//Lidar显示宽度（mm）
	int nSGBM_m_LidarDeep;				//Lidar显示深度（mm）
	int nSGBM_m_LidarGridSize;			//Lidar网格尺寸（pixel/m）
	int nSGBM_m_LidarHigh;				//Lidar显示高度（mm）
	long tag_StereoParam::LoadParam();
	tag_StereoParam(const char* pszPath=NULL, int nImgH=1200, int nImgW=1600);
}CStereoParam;

class CStereoOpr 
{
public:
	CStereoOpr(void);
	~CStereoOpr(void);
	
public:
	//初始化配置，包括以下配置文件 StereoParam.ini\intrinsics.yml\extrinsics.yml
	long Init(CStereoParam Param);
	//基于sgbm深度图的双目障碍物检测
	//ImgL_in、ImgR_in：矫正后的双目图片.
	//disp_out: 视差图
	//Pillar_out:二值图障碍物区域
	//PillarImg_out:障碍物区域着色的图片
	//默认参数适合于kitti数据集
	long BuildPillar(Mat& ImgL_in, Mat& ImgR_in, Mat& disp_out, Mat& Pillar_in, Mat& PillarImg_out);
	Mat CalDispSGBM(Mat& ImgL_in, Mat& ImgR_in);
	//基于前后帧视差图的移动障碍物检测
	//DispP_in:前一帧视差图
	//DispN_in:后一帧视差图
	//rvec,tvec:两帧之间的旋转及平移矩阵
	//Motion_out:二值图移动区域
	long MotionDetect(Mat& DispP_in, Mat& DispN_in, Mat& rvec, Mat& tvec, Mat& Motion_out );
	//将mat写入文件
	void saveXYZ1(const char* filename, const Mat& mat);
	//matlab中的伪彩色变换jet colorspace
	//mat_in:8u灰度图
	//output:伪彩色图片
	Mat Grey2FateColor(Mat& mat_in);
	//提取Lidar图片
	//Disp:input 视差图
	//Pillar:input 二值化的Pillar图片
	//Lidar:output Lidar 图片
	//output:Lidar cloud points
	Mat GetPillarPoints(Mat& Disp, Mat& Pillar, Mat& Lidar);
	//Remap stereo image pair
	//双目矫正
	void Remap(Mat& ImgL_in, Mat& ImgR_in, Mat& ImgL_out, Mat& ImgR_out);
	//Calculate 3D points from stereo points of images
	//计算三维点的坐标
	Point3f SteImgPt2XYZ(Point2f ptL, Point2f ptR);
	//Get Camera matrix
	Mat GetCameraMat(){return m_cameraMatrix;};
	//双目特征点及其match计算特征点云
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