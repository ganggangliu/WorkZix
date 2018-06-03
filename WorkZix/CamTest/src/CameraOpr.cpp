#include <iostream>
#include <sstream>

#include <boost/iostreams/stream.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/typeof/typeof.hpp>

#include "CameraOpr.h"
#include "CameraPointGreyImpl.h"

using namespace std;  
using namespace boost::property_tree;
using namespace boost;

// template<class Archive>
// void CCameraParam::serialize(Archive & ar, const unsigned int version)
// {
// 	ar & BOOST_SERIALIZATION_NVP(nCameraType);
// 	ar & BOOST_SERIALIZATION_NVP(szMac);
// 	ar & BOOST_SERIALIZATION_NVP(szIp);
// 	ar & BOOST_SERIALIZATION_NVP(nOffSetX);
// 	ar & BOOST_SERIALIZATION_NVP(nOffSetY);
// 	ar & BOOST_SERIALIZATION_NVP(nHeight);
// 	ar & BOOST_SERIALIZATION_NVP(nWidth);
// 	ar & BOOST_SERIALIZATION_NVP(nDataStreamType);
// 	ar & BOOST_SERIALIZATION_NVP(bIsTrigger);
// 	ar & BOOST_SERIALIZATION_NVP(bIsAutoFrameRate);
// 	ar & BOOST_SERIALIZATION_NVP(nFrameRate);
// 	ar & BOOST_SERIALIZATION_NVP(bIsAutoShutter);
// 	ar & BOOST_SERIALIZATION_NVP(nShutterTime);//MicrolliSeconds
// 	ar & BOOST_SERIALIZATION_NVP(bIsAutoGain);
// }

int CCameraParam::FromPtree(boost::property_tree::ptree Ptree)
{
	nCameraType = Ptree.get<int>("nCameraType", nCameraType);
	szMac = Ptree.get<string>("szMac", szMac);
	szIp = Ptree.get<string>("szIp", szIp);
	nOffSetX = Ptree.get<unsigned int>("nOffSetX", nOffSetX);
	nOffSetY = Ptree.get<unsigned int>("nOffSetY", nOffSetY);
	nHeight = Ptree.get<unsigned int>("nHeight", nHeight);
	nWidth = Ptree.get<unsigned int>("nWidth", nWidth);
	nDataStreamType = (CAMERA_STREAM_TYPE)Ptree.get<int>("nDataStreamType", nDataStreamType);
	bIsTrigger = Ptree.get<bool>("bIsTrigger", bIsTrigger);
	bIsAutoFrameRate = Ptree.get<bool>("bIsAutoFrameRate", bIsAutoFrameRate);
	nFrameRate = Ptree.get<unsigned int>("nFrameRate", nFrameRate);
	bIsAutoShutter = Ptree.get<bool>("bIsAutoShutter", bIsAutoShutter);
	nShutterTime = Ptree.get<unsigned int>("nShutterTime", nShutterTime);
	bIsAutoGain = Ptree.get<bool>("bIsAutoGain", bIsAutoGain);

	return 1;
}

boost::property_tree::ptree CCameraParam::ToPtree()
{
	ptree pt;
	pt.put<int>("nCameraType", nCameraType);
	pt.put<string>("szMac", szMac);
	pt.put<string>("szIp", szIp);
	pt.put<unsigned int>("nOffSetX", nOffSetX);
	pt.put<unsigned int>("nOffSetY", nOffSetY);
	pt.put<unsigned int>("nHeight", nHeight);
	pt.put<unsigned int>("nWidth", nWidth);
	pt.put<int>("nDataStreamType", nDataStreamType);
	pt.put<bool>("bIsTrigger", bIsTrigger);
	pt.put<bool>("bIsAutoFrameRate", bIsAutoFrameRate);
	pt.put<unsigned int>("nFrameRate", nFrameRate);
	pt.put<bool>("bIsAutoShutter", bIsAutoShutter);
	pt.put<unsigned int>("nShutterTime", nShutterTime);
	pt.put<bool>("bIsAutoGain", bIsAutoGain);

	return pt;

}

int CCameraParam::ReadParam(std::string szFileName, std::string szNodeName)
{
	if (!boost::filesystem::exists(szFileName))
	{
		return -1;
	}
	ptree pt;
	read_xml(szFileName, pt);
	ptree child = pt.get_child(szNodeName);
	FromPtree(child);

	return 1;
}

int CCameraParam::WriteParam(std::string szFileName, std::string szNodeName)
{
// 	std::ofstream fout(szFileName.c_str());
// 	boost::archive::xml_oarchive oa(fout);
// 	oa << boost::serialization::make_nvp(szNodeName.c_str()/*BOOST_PP_STRINGIZE(Param)*/, *this);
// 	fout.close();

	ptree pt = ToPtree();
	ptree pt_out;
	pt_out.put_child(szNodeName, pt);
	boost::property_tree::xml_writer_settings<char> settings('\t', 1);
	write_xml(szFileName, pt_out, std::locale(), settings);

	return 1;
}

hCameraHandle CreateCameraHandle(CCameraParam& Param)
{
	hCameraHandle hCam = 0;
	if (Param.nCameraType == POINT_GREY_BFLY_PGE_23S6C_C)
	{
		hCam = new CCameraPointGreyImpl;
	}

	if (hCam)
	{
		hCam->Init(Param);
	}

	return hCam;
}

void ReleaseCameraHandle(hCameraHandle& hHandle)
{
	if (hHandle)
	{
		delete hHandle;
		hHandle = 0;
	}
}