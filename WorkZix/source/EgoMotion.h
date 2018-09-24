#ifndef EGOMOTION_H
#define EGOMOTION_H

#include "FeatureDetectOpr.h"
#include "StereoOpr.h"

typedef struct tag_EgoMotionParam
{
	long nMaxFeatPointDist;	//定位点最大距离
	long LoadParam();
	tag_EgoMotionParam();
}CEgoMotionParam;

typedef struct tagStereoMatchPonint
{
	CvPoint2D32f ptLP;
	CvPoint2D32f ptRP;
	CvPoint2D32f ptLN;
	CvPoint2D32f ptRN;
	CvPoint3D32f	ptP3D;
	CvPoint3D32f	ptN3D;
}StereoMatchPoint;

class CEgoMotion 
{
public:
	CEgoMotion(void);
	~CEgoMotion(void);
	
public:
	//初始化
	long Init(CEgoMotionParam Param, CStereoParam ParamSte, CFeatureDetectParam ParamFeatDet);
	//自运动计算
	//imageL，imageR为校正后的双目图片
	long IterationStep(Mat& imageL, Mat& imageR, Mat& rvec, Mat& tvec, Mat& MatchImage, vector<Point3f>& rVecList, vector<Point3f>& tVecList );
	//画双目光流图
	long DrawOF(Mat& img1, vector<KeyPoint>& keypoints1,Mat& img2, vector<KeyPoint>& keypoints2,
		vector<DMatch>& matches1to2, Mat& outImg, Mat Valid);

private:
	CEgoMotionParam m_Param;
// 	CStereoParam m_ParamStereo;
// 	CFeatureDetectParam m_ParamFeatDet;
	CStereoOpr m_StereoOpr;
	CFeatureDetectOpr m_FeatDetOpr;
	long m_nCont;
	Mat m_imageLP;
	Mat m_imageRP;
	Mat m_imageLN; 
	Mat	m_imageRN;
	vector<KeyPoint> m_kpLP;
	vector<KeyPoint> m_kpRP;
	vector<KeyPoint> m_kpLN;
	vector<KeyPoint> m_kpRN;
	vector<DMatch> m_matchesP;
	vector<DMatch> m_matchesN;
	Mat m_dpsLP;
	Mat m_dpsRP;
	Mat m_dpsLN;
	Mat m_dpsRN;
};

#endif