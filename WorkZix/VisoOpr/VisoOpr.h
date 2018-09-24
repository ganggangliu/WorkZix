#include "OpenCVInc.h"

#ifdef VISOOPR_EXPORTS
#define VISOOPR_API __declspec(dllexport)
#else
#define VISOOPR_API __declspec(dllimport)
#endif

typedef struct VISOOPR_API tag_VisoOprParam
{
	double f;
	double cu;
	double cv;
	double base;
	double height;
	double pitch;
	int match_radius;
	int bucket_height;
	int bucket_width;
	int	ransac_iters;
	double	inlier_threshold;
	bool reweighting;
	bool doReconstruction;
	bool doForceHorizon;
	tag_VisoOprParam()
	{
		f = 0;
		cu = 0;
		cv = 0;
		base = 1000;
		height = 1000;
		pitch = 0;
		match_radius = 200;
		bucket_height = 20;
		bucket_width = 20;
		ransac_iters = 200;
		inlier_threshold = 2.0;
		reweighting = false;
		doReconstruction = false;
		doForceHorizon = true;
	};
}CVisoOprParam;

class VISOOPR_API CVisoOpr 
{
public:
	enum VISO_TYPE
	{
		STEREO = 0,
		MONO = 1
	};
	//初始化里程计
	//nType：0-双目里程计   1-单目里程计
	//Param：里程计参数
	CVisoOpr(CVisoOprParam Param, double dScale=1.f, VISO_TYPE type=STEREO);
	~CVisoOpr();

	//双目里程计计算,输入双目灰度图
	bool process(Mat ImgL,Mat ImgR);

	//单目里程计计算,输入双目灰度图
	bool process(Mat ImgL);

	//画光流图
	//nType：0-有效光流  1-所有光流
	long DrawOpticFlow(Mat& img, long nType=0);

	//获取自运动矩阵
	Mat getMotion();

	//获取当前姿态
	Mat getPose();

// 	//获取当前点云
 	Mat GetMonoCloudPoints();

	long DrawStereoMatch(Mat& imgMatch, Mat& imgL, Mat& imgR, vector<DMatch>& matchesT, vector<KeyPoint>& kpt0, vector<KeyPoint>& kpt1);
//	long GetPoint

	Mat GetReconstructionPoints();

private:
	long m_nType;
	
	
	void* m_pVisualOdometry;
	void* m_pReconstruction;
	Mat m_Pose;
	Mat m_Motion;
	double m_Scale;
	CVisoOprParam m_Param;
	long m_nPtsContCur;
};
