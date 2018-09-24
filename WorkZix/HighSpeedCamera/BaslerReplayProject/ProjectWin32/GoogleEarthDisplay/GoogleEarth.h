/////////////////////////////////////////////////////////////////////////////
// IApplicationGE wrapper class

class IApplicationGE : public COleDispatchDriver
{
public:
	IApplicationGE() {}		// Calls COleDispatchDriver default constructor
	IApplicationGE(LPDISPATCH pDispatch) : COleDispatchDriver(pDispatch) {}
	IApplicationGE(const IApplicationGE& dispatchSrc) : COleDispatchDriver(dispatchSrc) {}

// Operations
public:
	LPDISPATCH GetCamera(long considerTerrain);
	void SetCamera(LPDISPATCH camera, double speed);
	void SetCameraParams(double lat, double lon, double alt, long altMode, double Range, double Tilt, double Azimuth, double speed);
	long GetStreamingProgressPercentage();
	void SaveScreenShot(LPCTSTR fileName, long quality);
	void OpenKmlFile(LPCTSTR fileName, long suppressMessages);
	void LoadKmlData(BSTR* kmlData);
	double GetAutoPilotSpeed();
	void SetAutoPilotSpeed(double newValue);
	LPDISPATCH GetViewExtents();
	LPDISPATCH GetFeatureByName(LPCTSTR Name);
	LPDISPATCH GetFeatureByHref(LPCTSTR href);
	void SetFeatureView(LPDISPATCH feature, double speed);
	LPDISPATCH GetPointOnTerrainFromScreenCoords(double screen_x, double screen_y);
	long GetVersionMajor();
	long GetVersionMinor();
	long GetVersionBuild();
	long GetVersionAppType();
	long IsInitialized();
	long IsOnline();
	void Login();
	void Logout();
	void ShowDescriptionBalloon(LPDISPATCH feature);
	void HideDescriptionBalloons();
	LPDISPATCH GetHighlightedFeature();
	LPDISPATCH GetMyPlaces();
	LPDISPATCH GetTemporaryPlaces();
	LPDISPATCH GetLayersDatabases();
	double GetElevationExaggeration();
	void SetElevationExaggeration(double newValue);
	long GetMainHwnd();
	LPDISPATCH GetTourController();
	LPDISPATCH GetSearchController();
	LPDISPATCH GetAnimationController();
	long GetRenderHwnd();
};
