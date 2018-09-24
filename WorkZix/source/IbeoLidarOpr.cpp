#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>

#include "IbeoLidarOpr.h"

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
using namespace ibeosdk;

#if _DEBUG
#pragma comment(lib,"ibeosdkD.lib")
#else
#pragma comment(lib,"ibeosdk.lib")
#endif

class AllLuxListener : public ibeosdk::DataListener<ScanLux>,
                       public ibeosdk::DataListener<ObjectListLux>,
                       public ibeosdk::DataListener<VehicleStateBasicLux>
{
public:
    AllLuxListener(long nType = 4);
    ~AllLuxListener();
    void onData(const ScanLux* const scan);
    void onData(const ObjectListLux* const pObj);
    void onData(const VehicleStateBasicLux* const vsb);

    void SetCallBackPoints(lpIbeoDataRecFunc hCallBack, void* pUser = NULL);
    long GetPoints(LCM_IBEO_CLOUD_POINTS& points);
    void SetCallBackObjects(lpIbeoDataRecFunc hCallBack, void* pUser = NULL);
    long GetObjects(LCM_IBEO_OBJECT_LIST& objects);
	long GetObjectsRepeat(LCM_IBEO_OBJECT_LIST& objects);
	void SetCallBackVehicleState(lpIbeoDataRecFunc hCallBack, void* pUser = NULL);
	long GetVehicleState(LCM_IBEO_VEHICLE_STATE& vehiclestate);

private:
	long m_nType;

	bool m_bIsNewPoints;
    LCM_IBEO_CLOUD_POINTS m_CurPoints;
    lpIbeoDataRecFunc m_CallBackPoints;
    void* m_pUserPoints;
    CRITICAL_SECTION m_mutexPoints;

	bool m_bIsNewObjects;
    LCM_IBEO_OBJECT_LIST m_CurObjects;
    lpIbeoDataRecFunc m_CallBackObjects;
    void* m_pUserObjects;
    CRITICAL_SECTION m_mutexObjects;

	bool m_bIsNewVS;
	LCM_IBEO_VEHICLE_STATE m_VehicleState;
	lpIbeoDataRecFunc m_CallBackVehicleState;
	void* m_pUserVehicleState;
	CRITICAL_SECTION m_mutexVehicleState;

	int m_nObjFrameInd;
};

AllLuxListener::AllLuxListener(long nType)
{
	m_nType = nType;
	m_bIsNewPoints = false;
	m_bIsNewObjects = false;
	m_bIsNewVS = false;
	m_CallBackPoints = NULL;
	m_CallBackObjects = NULL;
	m_CallBackVehicleState = NULL;
	m_pUserPoints = NULL;
	m_pUserObjects = NULL;
	m_pUserVehicleState = NULL;
	InitializeCriticalSection(&m_mutexPoints);
	InitializeCriticalSection(&m_mutexObjects);
	InitializeCriticalSection(&m_mutexVehicleState);
	m_nObjFrameInd = 0;
}

AllLuxListener::~AllLuxListener()
{
	DeleteCriticalSection(&m_mutexPoints);
	DeleteCriticalSection(&m_mutexObjects);
	DeleteCriticalSection(&m_mutexVehicleState);
}

void AllLuxListener::SetCallBackPoints(lpIbeoDataRecFunc hCallBack, void* pUser)
{
    m_CallBackPoints = hCallBack;
    m_pUserPoints = pUser;
}

void AllLuxListener::SetCallBackObjects(lpIbeoDataRecFunc hCallBack, void* pUser)
{
    m_CallBackObjects = hCallBack;
    m_pUserObjects = pUser;
}

void AllLuxListener::SetCallBackVehicleState(lpIbeoDataRecFunc hCallBack, void* pUser)
{
	m_CallBackVehicleState = hCallBack;
	m_pUserVehicleState = pUser;
}

long AllLuxListener::GetPoints(LCM_IBEO_CLOUD_POINTS& points)
{
	EnterCriticalSection(&m_mutexPoints);
	if (!m_bIsNewPoints)
	{
		LeaveCriticalSection(&m_mutexPoints);
		return 0;
	}
    points = m_CurPoints;
	m_bIsNewPoints = false;
	LeaveCriticalSection(&m_mutexPoints);
    return 1;
}

long AllLuxListener::GetObjects(LCM_IBEO_OBJECT_LIST& objects)
{
	EnterCriticalSection(&m_mutexObjects);
	if (!m_bIsNewObjects)
	{
		LeaveCriticalSection(&m_mutexObjects);
		return 0;
	}
    objects = m_CurObjects;
	m_bIsNewObjects = false;
	LeaveCriticalSection(&m_mutexObjects);
    return 1;
}

long AllLuxListener::GetObjectsRepeat(LCM_IBEO_OBJECT_LIST& objects)
{
	EnterCriticalSection(&m_mutexObjects);
	objects = m_CurObjects;
	LeaveCriticalSection(&m_mutexObjects);
	return 1;
}

long AllLuxListener::GetVehicleState(LCM_IBEO_VEHICLE_STATE& VehicleState)
{
	EnterCriticalSection(&m_mutexVehicleState);
	if (!m_bIsNewVS)
	{
		LeaveCriticalSection(&m_mutexVehicleState);
		return 0;
	}
	VehicleState = m_VehicleState;
	m_bIsNewVS = false;
	LeaveCriticalSection(&m_mutexVehicleState);
	return 1;
}

static DWORD WINAPI IbeothreadReadData(LPVOID pParam)
{
	CIbeoLidarOpr* pIbeoOpr = (CIbeoLidarOpr*)pParam;
    pIbeoOpr->runIbeoDataThread();
	return 0;
}

//======================================================================

const ibeosdk::Version::MajorVersion majorVersion(2);
const ibeosdk::Version::MinorVersion minorVersion(11);
const ibeosdk::Version::Revision revision = ibeosdk::Version::Revision(1);
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


    //========================================
    void AllLuxListener::onData(const ScanLux* const scan)
    {
//        logInfo << "Scan received: # " << scan->getScanNumber()
//                <<"  time: " << tc.toString(scan->getStartTimestamp().toPtime(), 3)
//               << std::endl;
        std::vector<ScanPointLux> ScanPt =  scan->getScanPoints();
		ibeosdk::INT16 nFlag = scan->getFlags();
		bool bIsMirrorFaceUp = (nFlag & 0x0400);
		int nLayerAdd = 0;
		if (bIsMirrorFaceUp == true && m_nType == 8)
			nLayerAdd = 4;
        LCM_IBEO_CLOUD_POINTS CloudPt;
// 		if (bIsMirrorFaceUp)
// 			CloudPt.nLayerType = 1;
// 		else
// 			CloudPt.nLayerType = 0;
        CloudPt.TimeStamp = scan->getStartTimestamp().getMilliseconds();
        CloudPt.FrameInd = scan->getScanNumber();
        CloudPt.DevInd = 0;
        CloudPt.ContPoints = ScanPt.size();
        for (int i = 0; i < ScanPt.size(); i++)
        {
            LCM_IBEO_POINT ptT;
            ptT.layer = ScanPt[i].getLayer() + nLayerAdd;
            ptT.flag = ScanPt[i].getFlags();
            ptT.angle = ScanPt[i].getHorizontalAngle();
            ptT.distance = ScanPt[i].getDistance()*10.f;
            ptT.echo = ScanPt[i].getEcho();
			ptT.epw = ScanPt[i].getEchoPulseWidth()*10.f;
            CloudPt.IbeoPoints.push_back(ptT);
        }
		EnterCriticalSection(&m_mutexPoints);
        m_CurPoints = CloudPt;
		m_bIsNewPoints = true;
		LeaveCriticalSection(&m_mutexPoints);
        if(m_CallBackPoints)
        {
            (*m_CallBackPoints)(&CloudPt,m_pUserPoints);
        }
    }

    //========================================
    void AllLuxListener::onData(const ObjectListLux* const pObj)
    {
//        logInfo << "Objects received: # " << pObj->getNumberOfObjects() << std::endl;
		m_nObjFrameInd++;
        const std::vector<ObjectLux>& Objs =  pObj->getObjects();
        LCM_IBEO_OBJECT_LIST ObjList;
		ObjList.TimeStamp = pObj->getScanStartTimestamp().getMilliseconds();
		ObjList.FrameInd = m_nObjFrameInd;
        for (int i = 0; i < Objs.size(); i++)
        {
            const std::vector<Point2d>& contPointsRef = Objs[i].getContourPoints();
            LCM_IBEO_OBJECT ObjT;
            for (int j = 0; j < contPointsRef.size(); j++)
            {
                LCM_POINT2D_F ptT;
                ptT.x = contPointsRef[j].getX() * 10.f;
                ptT.y = contPointsRef[j].getY() * 10.f;
                ObjT.ContourPts.push_back(ptT);
            }
			ObjT.ContContourPt = ObjT.ContourPts.size();

			ObjT.Age = Objs[i].getObjectAge();
			ObjT.RefPoint.x = (float)Objs[i].getReferencePoint().getX() * 10.f;
			ObjT.RefPoint.y = (float)Objs[i].getReferencePoint().getY() * 10.f;
			ObjT.Classification = Objs[i].getClassification();
            ObjT.Id = Objs[i].getObjectId();
            ObjT.ObjBoxCenter.x = (float)Objs[i].getObjectBoxCenter().getX() * 10.f;
            ObjT.ObjBoxCenter.y = (float)Objs[i].getObjectBoxCenter().getY() * 10.f;
            ObjT.ObjBoxSize.x = (float)Objs[i].getObjectBoxSizeX() * 10.f;
            ObjT.ObjBoxSize.y = (float)Objs[i].getObjectBoxSizeY() * 10.f;
			ObjT.RelativeVelocity.x = (float)Objs[i].getRelativeVelocity().getX() * 10.f;
			ObjT.RelativeVelocity.y = (float)Objs[i].getRelativeVelocity().getY() * 10.f;
            ibeosdk::INT16 ObjOrient = Objs[i].getObjectBoxOrientation();
            ObjT.ObjOrientation = Objs[i].angle2rad(ObjOrient) / 3.141592654 * 180.f;
            ObjList.IbeoObjects.push_back(ObjT);
        }
        ObjList.ContObjects = ObjList.IbeoObjects.size();

		EnterCriticalSection(&m_mutexObjects);
        m_CurObjects= ObjList;
		m_bIsNewObjects = true;
		LeaveCriticalSection(&m_mutexObjects);
        if(m_CallBackObjects)
        {
            (*m_CallBackObjects)(&ObjList,m_pUserObjects);
        }
    }

    //========================================
    void AllLuxListener::onData(const VehicleStateBasicLux* const vsb)
    {
//        logInfo << "VSB for Scan #: " << vsb->getScanNumber() << "  "
//               << tc.toString(vsb->getTimestamp().toPtime()) << std::endl;
		double ddd = vsb->getSteeringWheeAngle();
		LCM_IBEO_VEHICLE_STATE VehicleState;
		VehicleState.dCourseAngle = (float)vsb->getCourseAngle()*0.0001/3.141592654*180.0;
		VehicleState.dCurrentYawRate = (float)vsb->getCurrentYawRate()*0.0001/3.141592654*180.0;
		VehicleState.dLongitudinalVelocity = (float)vsb->getLongitudinalVelocity()/10.0;
		VehicleState.dSteeringWheeAngle = (float)vsb->getSteeringWheeAngle()*0.001/3.141592654*180.0;
		VehicleState.dTimeDiff = (float)vsb->getTimeDiff()*0.0001/3.141592654*180.0;
		VehicleState.dWheelAngle = (float)vsb->getWheelAngle()*0.0001/3.141592654*180.0;
		VehicleState.dXDiff = (float)vsb->getXDiff()/1000.0;
		VehicleState.dXPos = (float)vsb->getXPos()/100.0;
		VehicleState.dYaw = (float)vsb->getYaw()*0.0001/3.141592654*180.0;
		VehicleState.dYDiff = (float)vsb->getYDiff()/1000.0;
		VehicleState.dYPos = (float)vsb->getYPos()/100.0;

		EnterCriticalSection(&m_mutexVehicleState);
		m_VehicleState= VehicleState;
		m_bIsNewVS = true;
		LeaveCriticalSection(&m_mutexVehicleState);
		if(m_CallBackVehicleState)
		{
			(*m_CallBackVehicleState)(&VehicleState,m_pUserVehicleState);
		}
	}


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

int main1(const int argc, const char** argv)
{
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
    const ibeosdk::LogLevel ll = ibeosdk::logLevelFromString("Warning");
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

    lux.registerListener(dynamic_cast<DataListener<ScanLux>*>(&allLuxListener));
    lux.registerListener(dynamic_cast<DataListener<ObjectListLux>*>(&allLuxListener));
    lux.registerListener(dynamic_cast<DataListener<VehicleStateBasicLux>*>(&allLuxListener));
//    lux.registerListener(dynamic_cast<DataListener<LogMessageError>*>(&allLuxListener));
//    lux.registerListener(dynamic_cast<DataListener<LogMessageDebug>*>(&allLuxListener));
//    lux.registerListener(dynamic_cast<DataListener<LogMessageNote>*>(&allLuxListener));
//    lux.registerListener(dynamic_cast<DataListener<LogMessageWarning>*>(&allLuxListener));

    lux.getConnected();

    // Just to keep the program alive
    while (true) {
        if (!lux.isConnected())
            return;
#		ifdef _WIN32
            ::Sleep(1);
#		else // _WIN32
            sleep(1);
#		endif // _WIN32
    }
}


CIbeoLidarOpr::CIbeoLidarOpr()
{
    
}

CIbeoLidarOpr::~CIbeoLidarOpr()
{
    if(m_pListener)
    {
        delete m_pListener;
        m_pListener = NULL;
    }
}

void CIbeoLidarOpr::Init(char* pszIp, long nType/* = 4*/)
{
	strcpy(m_szIp,pszIp);
	m_pListener = new AllLuxListener(nType);
	m_id = NULL;
	m_nType = nType;
}

void CIbeoLidarOpr::SetCallBackPoints(lpIbeoDataRecFunc  hCallBack, void* pUser)
{
    ((AllLuxListener*)m_pListener)->SetCallBackPoints(hCallBack,pUser);
}

void CIbeoLidarOpr::SetCallBackObjects(lpIbeoDataRecFunc  hCallBack, void* pUser)
{
    ((AllLuxListener*)m_pListener)->SetCallBackObjects(hCallBack,pUser);
}

void CIbeoLidarOpr::SetCallBackVehicleState(lpIbeoDataRecFunc  hCallBack, void* pUser)
{
	((AllLuxListener*)m_pListener)->SetCallBackVehicleState(hCallBack,pUser);
}

long CIbeoLidarOpr::Start()
{
	if (m_id != 0)
	{
		printf("m_hThread != 0\n");
		return 0;
	}

	m_id = CreateThread(NULL,0,IbeothreadReadData,this,0,NULL);
	if (m_id == 0)
	{
		printf("CreateThread Failed \n");
		return 0;
	}
	return 1;
}

long CIbeoLidarOpr::GetPointsData(LCM_IBEO_CLOUD_POINTS& CloudPoints)
{
    return ((AllLuxListener*)m_pListener)->GetPoints(CloudPoints);
}

long CIbeoLidarOpr::GetObjectsData(LCM_IBEO_OBJECT_LIST& Objects)
{
    return ((AllLuxListener*)m_pListener)->GetObjects(Objects);
}

long CIbeoLidarOpr::GetObjectsDataRepeat(LCM_IBEO_OBJECT_LIST& Objects)
{
	return ((AllLuxListener*)m_pListener)->GetObjectsRepeat(Objects);
}

long CIbeoLidarOpr::GetVehicleState(LCM_IBEO_VEHICLE_STATE& VehicleState)
{
	return ((AllLuxListener*)m_pListener)->GetVehicleState(VehicleState);
}

void CIbeoLidarOpr::runIbeoDataThread()
{
    //////////////////////////////////////////////////////////////////////////
    const off_t maxLogFileSize = 1000000;
    LogFileManager logFileManager;
    ibeosdk::LogFile::setTargetFileSize(maxLogFileSize);
    const ibeosdk::LogLevel ll = ibeosdk::logLevelFromString("Warning");
    ibeosdk::LogFile::setLogLevel(ll);
    logFileManager.start();
    //////////////////////////////////////////////////////////////////////////
    string szIp(m_szIp);
    const uint16_t port = getPort(szIp, 12002);
    IbeoLux lux(szIp, port);
    lux.setLogFileManager(&logFileManager);

//     lux.registerListener(dynamic_cast<DataListener<ScanLux>*>((AllLuxListener*)m_pListener));
//     lux.registerListener(dynamic_cast<DataListener<ObjectListLux>*>((AllLuxListener*)m_pListener));
//     lux.registerListener(dynamic_cast<DataListener<VehicleStateBasicLux>*>((AllLuxListener*)m_pListener));
	lux.registerListener((AllLuxListener*)m_pListener);

    lux.getConnected();

    // Just to keep the program alive
    while (true)
    {
        if (!lux.isConnected())
            return;
#		ifdef _WIN32
        ::Sleep(1);
#		else // _WIN32
        sleep(1);
#		endif // _WIN32
    }
}

//void CIbeoLidarOpr::onData(const ScanLux* const scan)
//{//
//
//}

