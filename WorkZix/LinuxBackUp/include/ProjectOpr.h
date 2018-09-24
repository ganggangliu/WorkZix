#ifndef PROJECT_OPR_H
#define PROJECT_OPR_H

#include "linux.h"
#include "LCM_SENSOR_FUSION_PACKAGE.hpp"
#include "LCM_IBEO_CLOUD_POINTS.hpp"
#include "OpenCVInc.h"

using namespace std;

#define PROJECT_MAX_FILE_LEN 100000

typedef struct tag_ProjectParam
{
    char szModuleFileName[MAX_PATH];
	char szProjPath[MAX_PATH];
	char szImgNameHeadL[MAX_PATH];
	char szImgNameHeadR[MAX_PATH];
	char szImgNameTail[MAX_PATH];
	char szGpsPath[MAX_PATH];
	long LoadParam();
	tag_ProjectParam(const char* pszPath = NULL);
}CProjectParam;

class CProjectOpr 
{
public:
	CProjectOpr();
	~CProjectOpr();

public:
	//
	long Init(CProjectParam Param);
	//
    long GetStereoImgByGpsFile(cv::Mat* pSteMat, cv::Mat& GpsData, int Color);
	//
	//Color: 0: grey  1:color
    long GetStereoImgByDoc(cv::Mat* pSteMat, int Color);
    cv::Mat GetGpsData(){return m_GpsData;};
//private:
	//
	long LoadProject();
	//
    cv::Mat LoadGpsData(char* pszFile);
	CProjectParam m_Param;
    cv::Mat m_GpsData;
	vector<string> m_VecImgPathL;
	vector<string> m_VecImgPathR;
	long m_nReadInd;
};

class CProjectWriter 
{
public:
	CProjectWriter();
	~CProjectWriter();

public:
	//
	long Init(CProjectParam Param);
	//
	long CreateProject();
	long CreateVideo();
	//
    long SaveImageAuto(long nRecInd, cv::Mat* pImg, LCM_GPS_DATA GpsData);
    long SaveImageAutoVideo(long nRecInd, cv::Mat* pImg, LCM_GPS_DATA GpsData);
	long ReleaseVideo();
    long SaveImageManual(long nRecInd, cv::Mat* pImg, LCM_GPS_DATA GpsData);
private:
	void AddTimeTail(char* psz);

	CProjectParam m_Param;
	char m_szManualSavePath[MAX_PATH];
	char m_szAutoSavePath[MAX_PATH];
	char m_szGpsLogAuto[MAX_PATH];
	char m_szGpsLogManual[MAX_PATH];
	bool m_bIsCreated;
	bool m_bIsCreatedVideo;
	long m_nSaveIndManual;
	long m_nSaveIndAuto;
	FILE* m_fpManual;
	FILE* m_fpAuto;

	CvVideoWriter* m_writerL;
	CvVideoWriter* m_writerR;
};

class CLidarDataOpr 
{
public:
    CLidarDataOpr(char* pszModulePath);
	~CLidarDataOpr(void);
	long CreateDir();
	long WriteLog(LCM_IBEO_CLOUD_POINTS* pLidarPoints, LCM_GPS_DATA* pImuInfo);
	long ReadLog(LCM_IBEO_CLOUD_POINTS* pLidarPoints, LCM_GPS_DATA* pImuInfo);
	long LcmSendLog(LCM_IBEO_CLOUD_POINTS* pLidarPoints, LCM_GPS_DATA* pImuInfo);
	long WriteLog(LCM_SENSOR_FUSION_PACKAGE* pPackData);
	long ReadLog(LCM_SENSOR_FUSION_PACKAGE* pPackData);
	long LcmSendLog(LCM_SENSOR_FUSION_PACKAGE* pPackData);

	long GetInd();
	void SetInd(long nInd);

private:
    void AddTimeTail(char* psz);

	bool m_bIsCreateDir;
	long m_nInd;
	char m_szPath[MAX_PATH];
	char m_szPathRead[MAX_PATH];
	FILE* m_fWrite;
	FILE* m_fRead;
	char m_FileBuffer[PROJECT_MAX_FILE_LEN];
};


#endif
