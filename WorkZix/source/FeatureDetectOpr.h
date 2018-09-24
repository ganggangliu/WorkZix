#ifndef FEATUREDETECTOPR_H
#define FEATUREDETECTOPR_H

#include <Windows.h>
#include "OpenCVInc.h"
using namespace std;

#define FEATURE_TYPE_USE_SIFT		0
#define FEATURE_TYPE_USE_SURF		1
#define FEATURE_TYPE_USE_ORB		2

typedef struct tag_FeatureDetectParam
{
	char szPathIni[MAX_PATH];
	long nImgHeight;			//image height
	long nImgWidth;				//image width
	double dMaxMatchRate;		//max match rate
	long nMaxStereoDisparityX;	//
	long nMaxStereoDisparityY;	//
	long nMaxFeaturePtDist;		//
	long nAlgorithmType;		//Feature point type. 0:sift 1:surf 2:orb
	double contrastThreshold;	//param for sift
	double edgeThreshold;		//param for sift
	double hessianThreshold;	//param for surf
	long nFeatures;				//param for orb
	int nAreaUmin;				//detect area limit u-min
	int nAreaUmax;				//detect area limit u-max
	int nAreaVmin;				//detect area limit v-min
	int nAreaVmax;				//detect area limit v-max
	long LoadParam();
	Mat Param2Mat();
	tag_FeatureDetectParam(const char* pszPath=NULL, int nImgH=1200, int nImgW=1600);
}CFeatureDetectParam;

class CFeatureDetectOpr {
public:
	CFeatureDetectOpr(void);
	~CFeatureDetectOpr(void);

public:
	//初始化参数使用 FeatureDetectParam.ini
	long Init(CFeatureDetectParam Param);
	//获取图片特征点
	//img:input 图片
	//keypoints:output 特征点
	//output:描述子
	long GetFeatures(Mat& img, vector<KeyPoint>& keypoints, Mat& descriptors);
	//双目匹配使用比例限制级UV坐标限制
	//MatchImage:ouput  匹配结果图
	//image0,image1:input 左右目图片
	//else : output
	long StereoFeatureMatch(Mat& MatchImage, Mat& image0, Mat& image1, vector<DMatch>& matches, vector<KeyPoint>& keypoints0, vector<KeyPoint>& keypoints1, Mat& descriptors0, Mat& descriptors1);
	//Stereo Feature match WITH only XY filter
	long StereoFeatureMatchEx(Mat& MatchImage, Mat& image0, Mat& image1, vector<DMatch>& matches, vector<KeyPoint>& keypoints0, vector<KeyPoint>& keypoints1, Mat& descriptors0, Mat& descriptors1);
	//按照行匹配特征点
	long StereoFeatureMatchByLine(Mat& MatchImage, Mat& image0, Mat& image1, vector<DMatch>& matches, vector<KeyPoint>& keypoints0, vector<KeyPoint>& keypoints1, Mat& descriptors0, Mat& descriptors1);
	//Feature point match with ratio filter
	long KNNMatch(Mat& descriptors0, Mat& descriptors1, vector<DMatch>& matches);
	//Feature point match with Cross check()   BFMatcher matcher(NORM_L2,true);
	long KNNMatchCross(Mat& descriptors0, Mat& descriptors1, vector<DMatch>& matches);
	//delete key points that not in matches
	//input:matches
	//input,output: keypoints0, keypoints1, descriptors0, descriptors1
	long FilterKeyPoints(vector<DMatch>& matches, vector<KeyPoint>& keypoints0, vector<KeyPoint>& keypoints1, Mat& descriptors0, Mat& descriptors1);
	//Is nIndex exist in matches[i].queryIdx;
	long IsInVector(vector<DMatch>& matches, int nIndex);

private:
	CFeatureDetectParam m_Param;
	Ptr<Feature2D> m_FeatureOpr;
	Mat m_DetAreaMask;
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

	long m_nAlgorithm;
	double m_contrastThreshold;
	double m_edgeThreshold;
	double m_hessianThreshold;
	double m_nFeatures;
};

#endif