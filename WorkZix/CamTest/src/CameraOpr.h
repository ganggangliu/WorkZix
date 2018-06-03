#ifndef CAMERA_OPR_H
#define CAMERA_OPR_H

#include <string>
#include <fstream>  
#include <iostream>  
#include <boost/function.hpp>
#include <boost/property_tree/ptree.hpp>
//#include <boost/archive/xml_oarchive.hpp>
//#include <boost/archive/xml_iarchive.hpp>
#include <opencv2/opencv.hpp>

typedef boost::function<void(cv::Mat*,void*)> FunctionCameraData;

enum CAMERA_TYPE
{
	POINT_GREY_BFLY_PGE_23S6C_C = 0
};

enum CAMERA_STREAM_TYPE
{
	ORIGINAL_DATA_STREAM,
	COMPRESSED_DATA_STREAM
};

class CCameraParam
{
public:
	int nCameraType;
	std::string szMac;
	std::string szIp;
	unsigned int nOffSetX;
	unsigned int nOffSetY;
	unsigned int nHeight;
	unsigned int nWidth;
	CAMERA_STREAM_TYPE nDataStreamType;
	bool bIsTrigger;
	bool bIsAutoFrameRate;
	unsigned int nFrameRate;
	bool bIsAutoShutter;
	unsigned int nShutterTime;//MicrolliSeconds
	bool bIsAutoGain;
	CCameraParam()
	{
		nCameraType = 0;
		szMac = "MAC";
		szIp = "IP";
		nOffSetX = 0;
		nOffSetY = 0;
		nHeight = 1200;
		nWidth = 1600;
		nDataStreamType = COMPRESSED_DATA_STREAM;
		bIsTrigger = false;
		bIsAutoFrameRate = true;
		nFrameRate = 20;
		bIsAutoShutter = true;
		nShutterTime = 10000;
		bIsAutoGain = true;
	}
	int FromPtree(boost::property_tree::ptree Ptree);
	boost::property_tree::ptree ToPtree();
	int ReadParam(std::string szFileName, std::string szNodeName);
	int WriteParam(std::string szFileName, std::string szNodeName);
// private:  
// 	friend class boost::serialization::access;  
// 	template<class Archive>
// 	void serialize(Archive & ar, const unsigned int version);
};

class CCameraOpr
{
public:
	CCameraOpr(){};
	~CCameraOpr(){};
	virtual int Init(CCameraParam& Param) = 0;
	virtual void SetCallBack(FunctionCameraData hCallBack, void* pUser) = 0;
	virtual int Start() = 0;
	virtual double GetCallBackRate() = 0;
	virtual int GetData(cv::Mat& img) = 0;
	virtual double GetFrameRate() = 0;
	virtual double GetShutter() = 0;
	virtual double GetGain() = 0;
	virtual double GetExposure() = 0;
	virtual int Stop() = 0;
};

typedef CCameraOpr* hCameraHandle;

hCameraHandle CreateCameraHandle(CCameraParam& Param);
void ReleaseCameraHandle(hCameraHandle& hHandle);

#endif