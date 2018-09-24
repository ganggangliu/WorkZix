//======================================================================
/*! \file IbeoSdkLuxLiveDemo.cpp
 *
 * \copydoc Copyright
 * \author Mario Brumm (mb)
 * \date Jun 1, 2012
 *
 * Demo project for connecting to a LUX and process the received
 * data blocks.
 *///-------------------------------------------------------------------

#include <ibeosdk/lux.hpp>
#include <ibeosdk/IpHelper.hpp>

#include <ibeosdk/datablocks/commands/CommandLuxReset.hpp>
#include <ibeosdk/datablocks/commands/CommandLuxGetStatus.hpp>
#include <ibeosdk/datablocks/commands/CommandLuxGetParameter.hpp>
#include <ibeosdk/datablocks/commands/CommandLuxSetParameter.hpp>
#include <ibeosdk/datablocks/commands/EmptyCommandReply.hpp>
#include <ibeosdk/datablocks/commands/ReplyLuxGetStatus.hpp>
#include <ibeosdk/datablocks/commands/ReplyLuxGetParameter.hpp>
#include <ibeosdk/datablocks/commands/CommandLuxSetNtpTimestampSync.hpp>


#include <iostream>
#include <cstdlib>

//Lcm head files are advised to be placed after ibeo head files
#include "LcmReceiver.h"
#include "LCM_IBEO_OBJECT_LIST.hpp"
#include "LCM_IBEO_CLOUD_POINTS.hpp"


//======================================================================

using namespace ibeosdk;

long g_nDevInd = 0;
std::string g_szChannleNameLidarCloudPoints = "LCM_IBEO_CLOUD_POINTS";
CLcmRevicer<LCM_IBEO_CLOUD_POINTS> g_LcmLidarCloudPoints(g_szChannleNameLidarCloudPoints);
std::string g_szChannleNameLidarObjectsList = "LCM_IBEO_OBJECT_LIST";
CLcmRevicer<LCM_IBEO_OBJECT_LIST> g_LcmLidarObjectsList(g_szChannleNameLidarObjectsList);

//======================================================================

const ibeosdk::Version::MajorVersion majorVersion(4);
const ibeosdk::Version::MinorVersion minorVersion(5);
const ibeosdk::Version::Revision revision(1);
const ibeosdk::Version::PatchLevel patchLevel;
const ibeosdk::Version::Build build;
const std::string info = "Demo";

ibeosdk::Version appVersion(majorVersion, minorVersion, revision, patchLevel, build, info);
IbeoSDK ibeoSDK;

//======================================================================

void live_demo(LogFileManager& logFileManager, std::string ip);

//======================================================================

TimeConversion tc;

//======================================================================

class AllLuxListener : public ibeosdk::DataListener<ScanLux>,
                       public ibeosdk::DataListener<ObjectListLux>,
                       public ibeosdk::DataListener<VehicleStateBasicLux>,
                       public ibeosdk::DataListener<LogMessageError>,
                       public ibeosdk::DataListener<LogMessageDebug>,
                       public ibeosdk::DataListener<LogMessageNote>,
                       public ibeosdk::DataListener<LogMessageWarning> {
public:
	virtual ~AllLuxListener() {}

public:
	//========================================
	NTPTime PointsTimeT;
	void onData(const ScanLux* const scan)
	{
		std::cout << "PointCount:" << scan->getScanPoints().size() << std::endl;
		std::cout << "PointTime:" << scan->getStartTimestamp().getMilliseconds() - PointsTimeT.getMilliseconds() << std::endl << std::endl;
		PointsTimeT = scan->getStartTimestamp();

		std::vector<ScanPointLux> ScanPt =  scan->getScanPoints();
		ibeosdk::INT16 nFlag = scan->getFlags();
		bool bIsMirrorFaceUp = (nFlag & 0x0400);
		int nLayerAdd = 0;
		if (bIsMirrorFaceUp == true)
			nLayerAdd = 4;
		LCM_IBEO_CLOUD_POINTS CloudPt;
		//if (bIsMirrorFaceUp)
		//	CloudPt.nLayerType = 1;
		//else
		//	CloudPt.nLayerType = 0;
		CloudPt.TimeStamp = scan->getStartTimestamp().getMicroseconds();
		CloudPt.FrameInd = scan->getScanNumber();
		CloudPt.DevInd = g_nDevInd;
		CloudPt.ContPoints = ScanPt.size();
		for (int i = 0; i < ScanPt.size(); i++)
		{
			LCM_IBEO_POINT ptT;
			ptT.layer = ScanPt[i].getLayer() + nLayerAdd;
			ptT.flag = ScanPt[i].getFlags();
			ptT.angle = ScanPt[i].getHorizontalAngle();
			ptT.distance = ScanPt[i].getDistance()*10.f;
			ptT.echo = ScanPt[i].getEcho();
			CloudPt.IbeoPoints.push_back(ptT);
		}

		g_LcmLidarCloudPoints.Send(g_szChannleNameLidarCloudPoints,CloudPt);
	}

	//========================================
	NTPTime ObjectsTimeT;
	void onData(const ObjectListLux* const pObj)
	{
		std::cout << "ObjectCount:" << pObj->getObjects().size() << std::endl;
		std::cout << "ObjectTime:" << pObj->getScanStartTimestamp().getMilliseconds() - ObjectsTimeT.getMilliseconds() << std::endl << std::endl;
		ObjectsTimeT = pObj->getScanStartTimestamp();

		const std::vector<ObjectLux>& Objs =  pObj->getObjects();
		LCM_IBEO_OBJECT_LIST ObjList;
		for (int i = 0; i < Objs.size(); i++)
		{
			const std::vector<Point2d>& contPointsRef = Objs[i].getContourPoints();
			LCM_IBEO_OBJECT ObjT;
			for (int j = 0; j < contPointsRef.size(); j++)
			{
				LCM_POINT2D_F ptT;
				ptT.x = -1.f *	contPointsRef[j].getY() * 10.f;
				ptT.y =			contPointsRef[j].getX() * 10.f;
				ObjT.ContourPts.push_back(ptT);
			}
			ObjT.ContContourPt = ObjT.ContourPts.size();
			ObjT.Id = Objs[i].getObjectId();
			ObjT.Age = Objs[i].getObjectAge();
			ObjT.RelativeVelocity.x = -1.f * (float)Objs[i].getRelativeVelocity().getY() * 10.f;
			ObjT.RelativeVelocity.y =		 (float)Objs[i].getRelativeVelocity().getX() * 10.f;
			ObjT.ObjBoxCenter.x = -1.f *	(float)Objs[i].getObjectBoxCenter().getY() * 10.f;
			ObjT.ObjBoxCenter.y =		(float)Objs[i].getObjectBoxCenter().getX() * 10.f;
			ObjT.ObjBoxSize.x = (float)Objs[i].getObjectBoxSizeY() * 10.f;
			ObjT.ObjBoxSize.y = (float)Objs[i].getObjectBoxSizeX() * 10.f;
			ibeosdk::INT16 ObjOrient = Objs[i].getObjectBoxOrientation();
			ObjT.ObjOrientation = Objs[i].angle2rad(ObjOrient) / 3.141592654 * 180.f;
			ObjList.IbeoObjects.push_back(ObjT);
		}
		ObjList.ContObjects = ObjList.IbeoObjects.size();

		g_LcmLidarObjectsList.Send(g_szChannleNameLidarObjectsList,ObjList);
	}

	//========================================
	void onData(const VehicleStateBasicLux* const vsb)
	{
//		logInfo << "VSB for Scan #: " << vsb->getScanNumber() << "  "
//				<< tc.toString(vsb->getTimestamp().toPtime()) << std::endl;
	}

	//========================================
	virtual void onData(const LogMessageError* const logMsg)
	{
		logInfo << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
				<< "LogMessage (Error) received: time: " << logMsg->getTraceLevel() << ": " << logMsg->getMessage() << std::endl;
	}

	//========================================
	virtual void onData(const LogMessageWarning* const logMsg)
	{
		logInfo << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
				<< "LogMessage (Warning) received: time: " << logMsg->getTraceLevel() << ": " << logMsg->getMessage() << std::endl;
	}

	//========================================
	virtual void onData(const LogMessageNote* const logMsg)
	{

		logInfo << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
				<< "LogMessage (Note) received: time: " << logMsg->getTraceLevel() << ": " << logMsg->getMessage() << std::endl;
	}

	//========================================
	virtual void onData(const LogMessageDebug* const logMsg)
	{
		logInfo << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
				<< "LogMessage (Debug) received: time: " << logMsg->getTraceLevel() << ": " << logMsg->getMessage() << std::endl;
	}
}; // AllLuxListener


//======================================================================
//======================================================================
//======================================================================

int checkArguments(const int argc, const char** argv, bool& hasLogFile)
{
	const int minNbOfNeededArguments = 2;
	const int maxNbOfNeededArguments = 3;

	bool wrongNbOfArguments = false;
	if (argc < minNbOfNeededArguments) {
		std::cerr << "Missing argument" << std::endl;
		wrongNbOfArguments = true;
	}
	else if (argc > maxNbOfNeededArguments) {
		std::cerr << "Too many argument" << std::endl;
		wrongNbOfArguments = true;
	}

	if (wrongNbOfArguments) {
		std::cerr << argv[0] << " " << " IP [LOGFILE]" << std::endl;
		std::cerr << "\tIP is the ip address of the LUX sensor, e.g. 192.168.0.1." << std::endl;
		std::cerr << "\tLOGFILE name of the log file. If ommitted, the log output will be performed to stderr." << std::endl;
		return 1;
	}

	hasLogFile = (argc == maxNbOfNeededArguments);
	return 0;
}

//======================================================================

int main(const int argc, const char** argv)
{
// 	long nInd = 0;
// 	while(1)
// 	{
// 		IBEO_OBJECTS_LIST ObjList;
// 		ObjList.DevInd =1;
// 		ObjList.DevType = 2;
// 		ObjList.FrameInd = 3;
// 		ObjList.TimeStamp = 4;
// 		for (int i = 0; i < 100; i++)
// 		{
// 			IBEO_OBJECT ttt;
// 			ObjList.Objects.push_back(ttt);
// 		}
// 		ObjList.ObjectCont = ObjList.Objects.size();
// 		
// 		g_LcmLidarObjectsList.Send(g_szChannleNameLidarObjectsList,ObjList);
// 		Sleep(100);
// 		nInd++;
// 		printf("%d\n",nInd);
// 	}

	std::cerr << argv[0] << " Version " << appVersion.toString();
	std::cerr << "  using IbeoSDK " << ibeoSDK.getVersion().toString() << std::endl;

	bool hasLogFile;
	const int checkResult = checkArguments(argc, argv, hasLogFile);
	if (checkResult != 0)
		exit(checkResult);
	int currArg = 1;

	std::string ip = argv[currArg++];

	const off_t maxLogFileSize = 1000000;

	LogFileManager logFileManager;
	ibeosdk::LogFile::setTargetFileSize(maxLogFileSize);

	if (hasLogFile) {
		ibeosdk::LogFile::setLogFileBaseName(argv[currArg++]);
	}
	const ibeosdk::LogLevel ll = ibeosdk::logLevelFromString("Debug");
	ibeosdk::LogFile::setLogLevel(ll);

	logFileManager.start();

	if (hasLogFile) {
		logInfo << argv[0] << " Version " << appVersion.toString()
		        << "  using IbeoSDK " << ibeoSDK.getVersion().toString() << std::endl;
	}

	live_demo(logFileManager, ip);

	exit(0);
}

//======================================================================

void live_demo(LogFileManager& logFileManager, std::string ip)
{
	AllLuxListener allLuxListener;

	const uint16_t port = getPort(ip, 12002);
	IbeoLux lux(ip, port);
	lux.setLogFileManager(&logFileManager);

	lux.registerListener(&allLuxListener);

	lux.getConnected();

	// Just to keep the program alive
	while (true) {
		if (!lux.isConnected())
			return;
#		ifdef _WIN32
			::Sleep(1);
#		else // _WIN32
			sleep(1);

//#define WITHCOMMANDDEMO
#ifdef WITHCOMMANDDEMO
			//CommandLuxReset resetCmd;

			{
				CommandLuxGetStatus getStatus;
				ReplyLuxGetStatus replyStatus;
				logInfo << "========================" << std::endl;
				logInfo << "Send Command GetStatus" << std::endl;
				logInfo << "------------------------" << std::endl;
				lux.sendCommand(getStatus, replyStatus);
				logError << "### Status: " << replyStatus.toString() << std::endl;
			}

			{
				CommandLuxGetParameter getParam(ParameterIndex(0x1200));
				ReplyLuxGetParameter replyParam;
				logInfo << "========================" << std::endl;
				logInfo << "Send Command getParam MPX" << std::endl;
				logInfo << "------------------------" << std::endl;
				lux.sendCommand(getParam, replyParam);
				logError << "### SENSORMOUNTING_X: " << int16_t(replyParam.getParameterData()) << std::endl;
			}

			{
				logInfo << "========================" << std::endl;
				logInfo << "Send Command setParam MPX" << std::endl;
				logInfo << "------------------------" << std::endl;
				CommandLuxSetParameter setParam(ParameterIndex(0x1200), ParameterData(1500));
				ReplyLuxSetParameter replySetParam;
				lux.sendCommand(setParam, replySetParam);
			}

			{
				logInfo << "========================" << std::endl;
				logInfo << "Send Command getParam MPX" << std::endl;
				logInfo << "------------------------" << std::endl;
				CommandLuxGetParameter getParam(ParameterIndex(0x1200));
				ReplyLuxGetParameter replyParam;				lux.sendCommand(getParam, replyParam);
				logError << "### SENSORMOUNTING_X: " << int16_t(replyParam.getParameterData()) << std::endl;
			}

			{
				logInfo << "========================" << std::endl;
				logInfo << "Send Command timeSync" << std::endl;
				logInfo << "------------------------" << std::endl;
				logError << "### Current time: " << Time::localTime() << std::endl;
				CommandLuxSetNtpTimestampSync timeSync(NTPTime(Time::localTime()));
				ReplyLuxSetNTPTimestampSync syncReply;
				lux.sendCommand(timeSync, syncReply);
			}

			logError << "========================================================" << std::endl << std::endl;
#endif // WITHCOMMANDDEMO

#		endif // _WIN32
	}
}

//======================================================================
