#include <Windows.h>
// #include <opencv2/opencv.hpp>
// #include <opencv2/nonfree/nonfree.hpp>
// #include <opencv2/nonfree/features2d.hpp>
// #include "opencv2/legacy/legacy.hpp"
#include "OpenCVInc.h"
#include "LibGuiOpenGL.h"
#include "EgoMotion.h"
#include "ProjectOpr.h"
//using namespace cv;
using namespace std;

#if _DEBUG
#pragma comment(lib,"EgoMotionD.lib")
#else
#pragma comment(lib,"EgoMotion.lib")
#endif

class CVFPLMapWriter 
{
public:
	CVFPLMapWriter();
	~CVFPLMapWriter();

	void Init(const char* pszPath, Mat FeatParam);
	long WriteMap(Mat GpsData, Mat Pose, Mat CloudPts, Mat descriptors, vector<KeyPoint> keypoints);
	void Release();

private:
	char m_szFilePath[MAX_PATH];
	FileStorage m_fsMap;
	long m_nMapIndex;
};

class CVFPLMapReader 
{
public:
	CVFPLMapReader();
	~CVFPLMapReader();

	long ReadMap(const char* pszPath, LIBGUIOPENGL_HANDLE hHandle = NULL);

	Mat m_FeatPointParam;
	vector<Mat> m_VecGpsData;
	vector<Mat> m_VecPose;
	vector<Mat> m_VecCloudPts;
	vector<Mat> m_VecDescriptors;
	vector<vector<KeyPoint>> m_VecKeyPoints;
	vector<double> m_VecMileStone;

private:
	char m_szFilePath[MAX_PATH];
	FileStorage m_fsMap;
	long m_nMapIndex;
};

class CVFPLLocator 
{
public:
	CVFPLLocator();
	~CVFPLLocator();
	
	long LoadMap(const char* pszPath);
	long InitLocator(CStereoParam ParamSte, CFeatureDetectParam ParamFeatDet);
	long Predict(Mat Transition);
	long Predict(double dTime, double dVe, double dVn);
	long Correct(Mat Measurment);
	long Correct(double dLat, double dLon);
	Mat Localization(Mat& img, double dTime, double dVe, double dVn, CProjectOpr& ProjMap);
	Mat GetDist(double lat0, double long0, double lat1, double long1);
	long AddSinglePoint(Mat GpsData);

private:
	long PNP(vector<DMatch>& match, vector<KeyPoint>& kpt, Mat& Cloud, Mat& valid);
	CVFPLMapReader m_VFPLMap;
	LIBGUIOPENGL_HANDLE m_hHandle;
	Mat m_PoseCur;
	vector<Mat> m_VecPose;
	vector<Mat> m_VecTrans;
	vector<Mat> m_VecMeasure;
	double m_dTimeBefor;
	double m_dHeadingBefor;
	CStereoOpr m_StereoOpr;
	CFeatureDetectOpr m_FeatPtOpr;
	vector<double> m_VecMileRecord;
	long m_nCurIndex;
};