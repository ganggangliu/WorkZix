#include <vector>
#include <list>
#include "TcpIpOpr.h"
#include "OpenCVInc.h"
#include "LibGuiOpenGL.h"
#include "DataBuffer.h"

using namespace std;

#if _DEBUG
#pragma comment(lib,"LibGuiOpenGLD.lib")
#else
#pragma comment(lib,"LibGuiOpenGL.lib")
#endif

class VPL16DataBlock
{
public:
	float fAzimuth;
	float fDist[16];
	unsigned char dReflect[16];
	VPL16DataBlock::VPL16DataBlock()
	{
		memset(this,0,sizeof(VPL16DataBlock));
	}
};

class VPL16DataPack
{
public:
	char szHeader[42];
	VPL16DataBlock DataBlock[24];
	unsigned int nTimeStamp;
	unsigned char Factory[2];
	VPL16DataPack::VPL16DataPack()
	{
		memset(this,0,sizeof(VPL16DataPack));
	}
};

class VPL16DataRound
{
public:
	float fStartAngle;
	vector<VPL16DataBlock> VecBlock;
	VPL16DataRound::VPL16DataRound()
	{
		VecBlock.reserve(2000);
	}
};

class VPL16DataPointsCloud
{
public:
	vector<Point3d> CloudPoints;
	vector<unsigned char> Reflect;
	VPL16DataPointsCloud::VPL16DataPointsCloud()
	{
		CloudPoints.reserve(30000);
		Reflect.resize(30000);
	}
	void VPL16DataPointsCloud::Clear()
	{
		CloudPoints.clear();
		Reflect.clear();
	}
};

typedef void(__stdcall *lpVPL16DataCallBack)(Mat*,void*);

class CVPL16Opr
{
public:
	CVPL16Opr();
	~CVPL16Opr();

	void Init(char* pszAddr, int nPort = 2368);

	void SetCallBack(lpVPL16DataCallBack DataCallBack, void* pUser);

	int Start();

	CUDPReciever m_UDPOpr;

public:
	void runPackData2RoundData();
	void runProcessRoundData();
	void OnRoundData(VPL16DataRound& DataRound);
	int ParsePackData(char* pszData, VPL16DataPack& Data);
	double Bytes2Float(char* pBytes, int nLen, double dScale);
	friend void __stdcall OnBinData(void* pData, void* pUser);
	VPL16DataPack m_CurData;
	char* m_pszHead;
	int m_nHeadLen;
	int m_nDataLen;
	int m_nRecLen;
	char* m_pszLocalAddr;
	int m_nPort;
	lpVPL16DataCallBack m_DataCallBack;
	void* m_pUser;
	double m_dStartAngle;
	VPL16DataRound m_CurDataRound;
	double m_dAngleBefor;
	vector<double> m_Ticks;
	VPL16DataPointsCloud m_PointsCloud;
	LIBGUIOPENGL_HANDLE m_OpenGL;
	CDataBuffer<VPL16DataPack> m_PackBuf;
	CDataBuffer<VPL16DataRound> m_RoundBuf;
	void* m_idPack;
	void* m_idRound;
};
