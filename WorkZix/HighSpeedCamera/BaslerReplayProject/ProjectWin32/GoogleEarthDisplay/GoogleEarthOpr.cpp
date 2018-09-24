#include "stdafx.h"
#include "GoogleEarthOpr.h"
#include "ATDispatch.h"
#include <fstream>

using namespace std;

// Google Earth CLSID  
static const CLSID CLSID_GEAPP_ =   
{ 0x8097D7E9, 0xDB9E, 0x4AEF,   
{ 0x9B, 0x28, 0x61, 0xD8, 0x2A, 0x1D, 0xF7, 0x84 } };  

CGoogleEarthOpr::CGoogleEarthOpr()
{
	m_GEApp = 0;
	m_GEHMainWnd = 0;
	m_GEHRenderWnd = 0;
	m_nWindWidth = 0;
	m_nWindHeight = 0;
	m_nInitRange = 150;
	m_bIsReady = false;

	char szExePath[MAX_PATH] = {0};
	GetModuleFileName(NULL, szExePath, MAX_PATH);
	char* pszTerminal = strrchr(szExePath, '\\');
	if (pszTerminal != 0)
		*pszTerminal = 0;
	m_szModuleFilePath = CString(szExePath);
	m_szVehicleIconPath = szExePath + CString("\\Vehicle.png");
	m_szTempPath = szExePath + CString("\\TrackTemp.kml");
}

CGoogleEarthOpr::~CGoogleEarthOpr()
{
	m_GEApp.ReleaseDispatch();
}

bool CGoogleEarthOpr::Create()
{
	system("taskkill /im googleearth.exe /f");
	Sleep(500);
//	system("cd C:\\Program Files\\Google\\Google Earth\\client\\");
// 	system("googleearth.exe /UnregServer");
// 	system("googleearth.exe /RegServer");

	if (m_GEApp != 0)
	{  
		return false;
	}  

	bool nRt = m_GEApp.CreateDispatch(CLSID_GEAPP_);
	if (nRt == false)
	{
		return false;
	}

// 	while(!m_GEApp.IsInitialized())
// 	{
// 		Sleep(1000);
// 	}
	m_bIsReady = true;

	m_GEHMainWnd = (HWND)m_GEApp.GetMainHwnd();  
	m_GEHRenderWnd = (HWND)m_GEApp.GetRenderHwnd();
	::ShowWindow(m_GEHRenderWnd, SW_SHOW);
	CRect Renser;
	::GetWindowRect(m_GEHRenderWnd,Renser);
	m_nWindWidth = Renser.Width();
	m_nWindHeight = Renser.Height();

	return true;
}

HWND CGoogleEarthOpr::GetMainWndHandle()
{
	return m_GEHMainWnd;
}

HWND CGoogleEarthOpr::GetRenderWndHandle()
{
	return m_GEHRenderWnd;
}

int CGoogleEarthOpr::GetGpsPointFromMouse(const int nMouseX, const int nMouseY, 
	double& dLat, double& dLon, double& dAlt)
{
	double RatX = (double)nMouseX/m_nWindWidth*2.0-1.0;
	double RatY = (double)(m_nWindHeight-nMouseY)/m_nWindHeight*2.0-1.0;
	LPDISPATCH res = m_GEApp.GetPointOnTerrainFromScreenCoords(RatX,RatY);

	CATDispatch Pt(res);
	CString szName("Latitude");
	LPOLESTR pwName = szName.AllocSysString();
	VARIANT var = Pt.get(pwName);
	dLat = var.dblVal;

	szName = "Longitude";
	pwName = szName.AllocSysString();
	var = Pt.get(pwName);
	dLon = var.dblVal;

	szName = "Altitude";
	pwName = szName.AllocSysString();
	var = Pt.get(pwName);
	dAlt = var.dblVal;

	return 1;
}

void CGoogleEarthOpr::FocusToPoint(double dLat, double dLon)
{

	LPDISPATCH pCamParam = m_GEApp.GetCamera(0);
	CATDispatch Pt(pCamParam);
	CString szName("FocusPointLatitude");
	LPOLESTR pwName = szName.AllocSysString();
	VARIANT var;
	var.vt = VT_R8;
	var.dblVal = dLat;
	Pt.put(pwName, var);
	szName = "FocusPointLongitude";
	pwName = szName.AllocSysString();
	var.vt = VT_R8;
	var.dblVal = dLon;
	Pt.put(pwName, var);
	m_GEApp.SetCamera(pCamParam, 5);
}

int CGoogleEarthOpr::AddGpsPoint(double dLat, double dLon, double dHead)
{
	if(m_GEApp == 0)
	{
		return -1;
	}
	m_VecLat.push_back(dLat);
	m_VecLon.push_back(dLon);
	m_VecHead.push_back(dHead);

	WriteKmlFile();

	DispTrack();

	return m_VecLat.size();
}

void CGoogleEarthOpr::ClearPoints()
{
	m_VecLat.clear();
	m_VecLon.clear();
	m_VecHead.clear();

	LPDISPATCH res = m_GEApp.GetTemporaryPlaces();

	CATDispatch Pt(res);
	CString szName("GetChildren");
	LPOLESTR pwName = szName.AllocSysString();
	VARIANT ParamCont;
	ParamCont.vt = VT_I2;
	ParamCont.iVal = 0;
	VARIANT var = Pt.Function(pwName, ParamCont);
	LPDISPATCH MyCollection = var.pdispVal;
	CATDispatch MyCollection_(MyCollection);
	szName = ("Count");
	pwName = szName.AllocSysString();
	var = MyCollection_.get(pwName);
	int nItemCont = var.intVal;
	for (unsigned int i = 1; i <= nItemCont; i++)
	{
		szName = ("Item");
		pwName = szName.AllocSysString();
		var = MyCollection_.get(pwName, i);
		LPDISPATCH MyCollection_feature = var.pdispVal;
		CATDispatch MyCollection_feature_(MyCollection_feature);
		szName = ("Visibility");
		pwName = szName.AllocSysString();
		VARIANT in;
		in.vt = VT_BOOL;
		in.boolVal = VARIANT_FALSE;
		MyCollection_feature_.put(pwName, in);
	}
}

void CGoogleEarthOpr::Destroy()
{
	m_GEApp.ReleaseDispatch();
	system("taskkill /im googleearth.exe /f");
}

void CGoogleEarthOpr::WriteKmlFile()
{
	std::ofstream fsKlm(m_szTempPath);
	fsKlm.setf(ios::fixed, ios::floatfield);
	fsKlm << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << endl;
	fsKlm << "<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\" xmlns:kml=\"http://www.opengis.net/kml/2.2\" xmlns:atom=\"http://www.w3.org/2005/Atom\">" << endl;
	fsKlm << "<Document>" << endl;
	//	fsKlm << "<name>ublox.kml</name>" << endl;
	fsKlm << "<name>" << "GpsTrack" << "</name>" << endl;;

	fsKlm << "<StyleMap id=\"msn_ylw-pushpin\">" << endl;
	fsKlm << "<Pair>" << endl;
	fsKlm << "<key>normal</key>" << endl;
	fsKlm << "<styleUrl>#sn_ylw-pushpin</styleUrl>" << endl;
	fsKlm << "</Pair>" << endl;
	fsKlm << "</StyleMap>" << endl;
	fsKlm << "<Style id=\"sn_ylw-pushpin\">" << endl;
	fsKlm << "<LineStyle>" << endl;
	fsKlm << "<color>ff0000ff</color>" << endl;
	// 	fsKlm << "<color>";
	// 	fsKlm << szColor;
	// 	fsKlm << "</color>" << endl;
	fsKlm << "<width>2</width>" << endl;
	fsKlm << "</LineStyle>" << endl;
	fsKlm << "</Style>" << endl;

	fsKlm << "<Placemark>" << endl;
	fsKlm << "<name>GpsTrack</name>" << endl;
	fsKlm << "<styleUrl>#msn_ylw-pushpin</styleUrl>" << endl;
	fsKlm << "<LineString>" << endl;
	fsKlm << "<tessellate>1</tessellate>" << endl;
	fsKlm << "<coordinates>" << endl;

	for (unsigned int i = 0; i < m_VecLat.size(); i++)
	{
		char szLine[1024] = {0};
		sprintf(szLine, "%.7f,%.7f,2", m_VecLon[i],m_VecLat[i]);
		if (i != 0)
		{
			fsKlm << ",";
		}
		fsKlm << szLine;
	}
	fsKlm << endl;

	fsKlm << "</coordinates>" << endl;
	fsKlm << "</LineString>" << endl;
	fsKlm << "</Placemark>" << endl;

	//////////////////////////////////////////////////////////////////////////
	if (m_VecLat.size() > 1)
	{
		fsKlm << "<GroundOverlay>" << endl;
		fsKlm << "<name>Vehicle</name>" << endl;
		fsKlm << "<Icon>" << endl;
		fsKlm << "<href>" << m_szVehicleIconPath << "</href>" << endl;
		fsKlm << "<viewBoundScale>0.75</viewBoundScale>" << endl;
		fsKlm << "</Icon>" << endl;

		double dNorth = 0.0;
		double dSouth = 0.0;
		double dEast = 0.0;
		double dWest = 0.0;
		double dAlt = 0.0;
		double dHead = 0.0;
		GetCurrentVehicleState(dNorth, dSouth, dEast, dWest, dAlt, dHead);

		fsKlm.precision(7);
		fsKlm << "<altitude>" << dAlt + 0.5 << "</altitude>" << endl;
		fsKlm << "<altitudeMode>absolute</altitudeMode>" << endl;
		fsKlm << "<LatLonBox>" << endl;

		fsKlm.precision(7);
		fsKlm << "<north>" << dNorth << "</north>" << endl;
		fsKlm << "<south>" << dSouth << "</south>" << endl;
		fsKlm << "<east>" << dEast << "</east>" << endl;
		fsKlm << "<west>" << dWest << "</west>" << endl;
		fsKlm << "<rotation>" << dHead << "</rotation>" << endl;
		fsKlm << "</LatLonBox>" << endl;
		fsKlm << "</GroundOverlay>" << endl;
	}

	//////////////////////////////////////////////////////////////////////////
	fsKlm << "</Document>" << endl;
	fsKlm << "</kml>" << endl;

	fsKlm.close();
}
int CGoogleEarthOpr::DispTrack()
{
	CString szPath(m_szTempPath);
	m_GEApp.OpenKmlFile(szPath, 0);

// 	CString szPath("GpsTrack.kml");
// 	CFile fs;
//  fs.Open(szPath, CFile::modeRead);
//  char szContent[500000] = {0};
//  memset(szContent, 0 , 500000);
//  fs.Read(szContent,500000);
//  CString str_(szContent);
//  BSTR bstrText = str_.AllocSysString();
//  GEApp.LoadKmlData(&bstrText);
// 	fs.Close();

	if (m_VecLat.size() <= 3)
	{
		LPDISPATCH pCamParam = m_GEApp.GetCamera(1);
		CATDispatch Pt(pCamParam);
		CString szName("FocusPointLatitude");
		LPOLESTR pwName = szName.AllocSysString();
		VARIANT var;
		var.vt = VT_R8;
		var.dblVal = m_VecLat.back();
		Pt.put(pwName, var);
		szName = "FocusPointLongitude";
		pwName = szName.AllocSysString();
		var.vt = VT_R8;
		var.dblVal = m_VecLon.back();
		Pt.put(pwName, var);
		szName = "FocusPointAltitude";
		pwName = szName.AllocSysString();
		var.vt = VT_R8;
		var.dblVal = 0.0;
		Pt.put(pwName, var);
		szName = "FocusPointAltitudeMode";
		pwName = szName.AllocSysString();
		var.vt = VT_I4;
		var.iVal = 1;
		Pt.put(pwName, var);
		szName = "Range";
		pwName = szName.AllocSysString();
		var.vt = VT_R8;
		var.dblVal = m_nInitRange;
		Pt.put(pwName, var);
		m_GEApp.SetCamera(pCamParam, 5);
//		GEApp.SetCameraParams(m_VecLat.back(), m_VecLon.back(), 0, 1, m_dViewRange, 0/*45*/, 0, 5);
	}
	if (m_VecLat.size() > 1)
	{
		LPDISPATCH pCamParam = m_GEApp.GetCamera(0);
		CATDispatch Pt(pCamParam);
		CString szName("FocusPointLatitude");
		LPOLESTR pwName = szName.AllocSysString();
		VARIANT var;
		var.vt = VT_R8;
		var.dblVal = m_VecLat.back();
		Pt.put(pwName, var);
		szName = "FocusPointLongitude";
		pwName = szName.AllocSysString();
		var.vt = VT_R8;
		var.dblVal = m_VecLon.back();
		Pt.put(pwName, var);
		m_GEApp.SetCamera(pCamParam, 5);
	}

	return 1;
}

int CGoogleEarthOpr::GetCurrentVehicleState(
	double& dNorth, double& dSouth, 
	double& dEast, double& dWest, 
	double& dAlt, double& dHead)
{
	double dCenter2Left = 1.0;
	double dCenter2Right = 1.0;
	double dCenter2Front = 2.5;
	double dCenter2Back = 2.5;

	dAlt = 0.0;
	dHead = 0.0;

	double dLat = m_VecLat.back();
	double dLon = m_VecLon.back();

	if (m_VecLon.size() >= 2)
	{
		double dDeltaY = -1.0*(m_VecLon.back() - m_VecLon[m_VecLon.size()-2])*
			cos(m_VecLat.back()/180*3.141592654);
		double dDeltaX = m_VecLat.back() - m_VecLat[m_VecLat.size()-2];
		double dAng = atan2(dDeltaY, dDeltaX)/3.141592654*180.0;
		if (dAng < 0)
		{
			dAng += 360;
		}
		dHead = dAng;
	}

	LPDISPATCH res = m_GEApp.GetPointOnTerrainFromScreenCoords(0.0,0.0);
	CATDispatch Pt(res);
	CString szName("Altitude");
	LPOLESTR pwName = szName.AllocSysString();
	VARIANT var = Pt.get(pwName);
	dAlt = var.dblVal;

	std::vector<double> corners_vehicle_x(4,0.0);
	std::vector<double> corners_vehicle_y(4,0.0);
	corners_vehicle_x[0] = dCenter2Front;
	corners_vehicle_y[0] = dCenter2Left;
	corners_vehicle_x[1] = dCenter2Front;
	corners_vehicle_y[1] = -dCenter2Right;
	corners_vehicle_x[2] = -dCenter2Back;
	corners_vehicle_y[2] = -dCenter2Right;
	corners_vehicle_x[3] = -dCenter2Back;
	corners_vehicle_y[3] = -dCenter2Left;
	std::vector<double> corners_gps_lat(4,0.0);
	std::vector<double> corners_gps_lon(4,0.0);
	for (unsigned int i = 0; i < 4; i++)
	{
		VehicleCorner2Gps(dLat, dLon, 0, corners_vehicle_x[i], corners_vehicle_y[i],
			corners_gps_lat[i], corners_gps_lon[i]);
	}

	dNorth = corners_gps_lat[0];
	dWest = corners_gps_lon[0];
	dSouth = corners_gps_lat[2];
	dEast = corners_gps_lon[2];

	return 1;
}

int CGoogleEarthOpr::VehicleCorner2Gps(
	const double& dLat, const double& dLon, 
	const double& dHead, 
	const double& dX, const double& dY, 
	double& dLat_out, double& dLon_out)
{
	double PI = 3.141592653;
	double dHead_r = dHead/180.f*PI;
	//坐标系反方向旋转								
	double x_ = dX*cos(dHead_r) - dY*sin(dHead_r);
	double y_ = dX*sin(dHead_r) + dY*cos(dHead_r);
	dLat_out = x_/111319.55 + dLat;
	dLon_out = (-y_)/111319.55/cos(dLat/180.0*PI) + dLon;

	return 1;
}