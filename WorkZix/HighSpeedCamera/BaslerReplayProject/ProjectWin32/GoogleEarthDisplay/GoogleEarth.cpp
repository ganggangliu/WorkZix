
#include "stdafx.h"
#include "GoogleEarth.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


LPDISPATCH IApplicationGE::GetCamera(long considerTerrain)
{
	LPDISPATCH result;
	static BYTE parms[] = VTS_I4;
	InvokeHelper(0x1, DISPATCH_METHOD, VT_DISPATCH, (void*)&result, parms, considerTerrain);
	return result;
}

void IApplicationGE::SetCamera(LPDISPATCH camera, double speed)
{
	static BYTE parms[] = VTS_DISPATCH VTS_R8;
	InvokeHelper(0x2, DISPATCH_METHOD, VT_EMPTY, NULL, parms, camera, speed);
}

void IApplicationGE::SetCameraParams(double lat, double lon, double alt, 
	long altMode, double Range, double Tilt, double Azimuth, double speed)
{
	static BYTE parms[] =
		VTS_R8 VTS_R8 VTS_R8 VTS_I4 VTS_R8 VTS_R8 VTS_R8 VTS_R8;
	InvokeHelper(0x3, DISPATCH_METHOD, VT_EMPTY, NULL, parms,
		 lat, lon, alt, altMode, Range, Tilt, Azimuth, speed);
}

long IApplicationGE::GetStreamingProgressPercentage()
{
	long result;
	InvokeHelper(0x4, DISPATCH_PROPERTYGET, VT_I4, (void*)&result, NULL);
	return result;
}

void IApplicationGE::SaveScreenShot(LPCTSTR fileName, long quality)
{
	static BYTE parms[] = VTS_BSTR VTS_I4;
	InvokeHelper(0x5, DISPATCH_METHOD, VT_EMPTY, NULL, parms, fileName, quality);
}

void IApplicationGE::OpenKmlFile(LPCTSTR fileName, long suppressMessages)
{
	static BYTE parms[] = VTS_BSTR VTS_I4;
	InvokeHelper(0x6, DISPATCH_METHOD, VT_EMPTY, NULL, parms,
		 fileName, suppressMessages);
}

void IApplicationGE::LoadKmlData(BSTR* kmlData)
{
	static BYTE parms[] = VTS_PBSTR;
	InvokeHelper(0x7, DISPATCH_METHOD, VT_EMPTY, NULL, parms, kmlData);
}

double IApplicationGE::GetAutoPilotSpeed()
{
	double result;
	InvokeHelper(0x8, DISPATCH_PROPERTYGET, VT_R8, (void*)&result, NULL);
	return result;
}

void IApplicationGE::SetAutoPilotSpeed(double newValue)
{
	static BYTE parms[] = VTS_R8;
	InvokeHelper(0x8, DISPATCH_PROPERTYPUT, VT_EMPTY, NULL, parms, newValue);
}

LPDISPATCH IApplicationGE::GetViewExtents()
{
	LPDISPATCH result;
	InvokeHelper(0x9, DISPATCH_PROPERTYGET, VT_DISPATCH, (void*)&result, NULL);
	return result;
}

LPDISPATCH IApplicationGE::GetFeatureByName(LPCTSTR Name)
{
	LPDISPATCH result;
	static BYTE parms[] = VTS_BSTR;
	InvokeHelper(0xa, DISPATCH_METHOD, VT_DISPATCH, (void*)&result, parms, Name);
	return result;
}

LPDISPATCH IApplicationGE::GetFeatureByHref(LPCTSTR href)
{
	LPDISPATCH result;
	static BYTE parms[] = VTS_BSTR;
	InvokeHelper(0xb, DISPATCH_METHOD, VT_DISPATCH, (void*)&result, parms, href);
	return result;
}

void IApplicationGE::SetFeatureView(LPDISPATCH feature, double speed)
{
	static BYTE parms[] = VTS_DISPATCH VTS_R8;
	InvokeHelper(0xc, DISPATCH_METHOD, VT_EMPTY, NULL, parms, feature, speed);
}

LPDISPATCH IApplicationGE::GetPointOnTerrainFromScreenCoords(double screen_x, double screen_y)
{
	LPDISPATCH result;
	static BYTE parms[] = VTS_R8 VTS_R8;
	InvokeHelper(0xd, DISPATCH_METHOD, VT_DISPATCH, (void*)&result, parms, screen_x, screen_y);
	return result;
}

long IApplicationGE::GetVersionMajor()
{
	long result;
	InvokeHelper(0xe, DISPATCH_PROPERTYGET, VT_I4, (void*)&result, NULL);
	return result;
}

long IApplicationGE::GetVersionMinor()
{
	long result;
	InvokeHelper(0xf, DISPATCH_PROPERTYGET, VT_I4, (void*)&result, NULL);
	return result;
}

long IApplicationGE::GetVersionBuild()
{
	long result;
	InvokeHelper(0x10, DISPATCH_PROPERTYGET, VT_I4, (void*)&result, NULL);
	return result;
}

long IApplicationGE::GetVersionAppType()
{
	long result;
	InvokeHelper(0x11, DISPATCH_PROPERTYGET, VT_I4, (void*)&result, NULL);
	return result;
}

long IApplicationGE::IsInitialized()
{
	long result;
	InvokeHelper(0x12, DISPATCH_METHOD, VT_I4, (void*)&result, NULL);
	return result;
}

long IApplicationGE::IsOnline()
{
	long result;
	InvokeHelper(0x13, DISPATCH_METHOD, VT_I4, (void*)&result, NULL);
	return result;
}

void IApplicationGE::Login()
{
	InvokeHelper(0x14, DISPATCH_METHOD, VT_EMPTY, NULL, NULL);
}

void IApplicationGE::Logout()
{
	InvokeHelper(0x15, DISPATCH_METHOD, VT_EMPTY, NULL, NULL);
}

void IApplicationGE::ShowDescriptionBalloon(LPDISPATCH feature)
{
	static BYTE parms[] = VTS_DISPATCH;
	InvokeHelper(0x16, DISPATCH_METHOD, VT_EMPTY, NULL, parms,  feature);
}

void IApplicationGE::HideDescriptionBalloons()
{
	InvokeHelper(0x17, DISPATCH_METHOD, VT_EMPTY, NULL, NULL);
}

LPDISPATCH IApplicationGE::GetHighlightedFeature()
{
	LPDISPATCH result;
	InvokeHelper(0x18, DISPATCH_METHOD, VT_DISPATCH, (void*)&result, NULL);
	return result;
}

LPDISPATCH IApplicationGE::GetMyPlaces()
{
	LPDISPATCH result;
	InvokeHelper(0x19, DISPATCH_METHOD, VT_DISPATCH, (void*)&result, NULL);
	return result;
}

LPDISPATCH IApplicationGE::GetTemporaryPlaces()
{
	LPDISPATCH result;
	InvokeHelper(0x1a, DISPATCH_METHOD, VT_DISPATCH, (void*)&result, NULL);
	return result;
}

LPDISPATCH IApplicationGE::GetLayersDatabases()
{
	LPDISPATCH result;
	InvokeHelper(0x1b, DISPATCH_METHOD, VT_DISPATCH, (void*)&result, NULL);
	return result;
}

double IApplicationGE::GetElevationExaggeration()
{
	double result;
	InvokeHelper(0x1c, DISPATCH_PROPERTYGET, VT_R8, (void*)&result, NULL);
	return result;
}

void IApplicationGE::SetElevationExaggeration(double newValue)
{
	static BYTE parms[] = VTS_R8;
	InvokeHelper(0x1c, DISPATCH_PROPERTYPUT, VT_EMPTY, NULL, parms, newValue);
}

long IApplicationGE::GetMainHwnd()
{
	long result;
	InvokeHelper(0x1d, DISPATCH_METHOD, VT_I4, (void*)&result, NULL);
	return result;
}

LPDISPATCH IApplicationGE::GetTourController()
{
	LPDISPATCH result;
	InvokeHelper(0x1e, DISPATCH_PROPERTYGET, VT_DISPATCH, (void*)&result, NULL);
	return result;
}

LPDISPATCH IApplicationGE::GetSearchController()
{
	LPDISPATCH result;
	InvokeHelper(0x1f, DISPATCH_PROPERTYGET, VT_DISPATCH, (void*)&result, NULL);
	return result;
}

LPDISPATCH IApplicationGE::GetAnimationController()
{
	LPDISPATCH result;
	InvokeHelper(0x20, DISPATCH_PROPERTYGET, VT_DISPATCH, (void*)&result, NULL);
	return result;
}

long IApplicationGE::GetRenderHwnd()
{
	long result;
	InvokeHelper(0x21, DISPATCH_METHOD, VT_I4, (void*)&result, NULL);
	return result;
}
