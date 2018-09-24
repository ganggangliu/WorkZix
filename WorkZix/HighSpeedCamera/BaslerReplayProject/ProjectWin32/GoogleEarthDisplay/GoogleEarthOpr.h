#ifndef GOOGLE_EARTH_OPR
#define GOOGLE_EARTH_OPR

#include <vector>
#include "GoogleEarth.h"

class CGoogleEarthOpr
{
public:
	CGoogleEarthOpr();
	~CGoogleEarthOpr();
	bool Create();
	HWND GetMainWndHandle();
	HWND GetRenderWndHandle();
	int GetGpsPointFromMouse(const int nMouseX, const int nMouseY, 
		double& dLat, double& dLon, double& dAlt);
	void FocusToPoint(double dLat, double dLon);
	int AddGpsPoint(double dLat, double dLon, double dHead);
	void ClearPoints();
	void Destroy();

	HWND m_GEHMainWnd;
	HWND m_GEHRenderWnd;
	int m_nWindWidth;
	int m_nWindHeight;
	bool m_bIsReady;

private:
	void WriteKmlFile();
	int DispTrack();
	int GetCurrentVehicleState(
		double& dNorth, double& dSouth,
		double& dEast, double& dWest,
		double& dAlt, double& dHead);
	int VehicleCorner2Gps(
		const double& dLat, const double& dLon, 
		const double& dHead, 
		const double& dX, const double& dY, 
		double& dLat_out, double& dLon_out);

	IApplicationGE m_GEApp;
	int m_nInitRange;

	CString m_szModuleFilePath;
	CString m_szVehicleIconPath;
	CString m_szTempPath;

	std::vector<double> m_VecLat;
	std::vector<double> m_VecLon;
	std::vector<double> m_VecHead;
};


#endif