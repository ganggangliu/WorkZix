// LibGuiOpenGL.cpp : 定义 DLL 应用程序的导出函数。
//

#include "stdafx.h"
#include "LibGuiOpenGL.h"

#include "OpenGl.h"
#include "tinyxml.h"

class CLibGuiOpenGL : public ILibGuiOpenGL
{
public:
	CLibGuiOpenGL(const string &LibXml);

	void AddRefPoints(Mat RefPoints, Mat Color = Mat());
	void AddRefPoints(double x, double y, double heading, Mat Color = Mat());
	void AddPointsRelative(Mat Points, Mat Pose = Mat(), Mat Color = Mat());
	void AddPointsRelative(double x, double y, double heading, Mat Points, Mat Color = Mat());
	void AddLinesRelative(Mat Lines, Mat Pose = Mat(), Mat Color = Mat());
	void AddBreakLinesRelative(Mat Lines, Mat Pose = Mat(), Mat Color = Mat());
	void AddSphereRelative(Point3d pt,float sz, Mat Pose = Mat(), Mat Color = Mat());
	void Clear();
	void ClearRefPoints();
	void ClearPoints();
	void ClearLines();
	void ClearBreakLines();
	void ClearSphere();
	void SetCameraMode(int nMode);
	void SetScale(double dScale);
	void SetAxisLenth(float dLenth);

	void UnInit();
	bool Init();	// inner defined

private:
	string m_LibXmlName;
	string m_LibXmlPath;

	bool LoadParam();

	int m_iMaxPointsLimit;
	double m_gluPerspective_zfar;
	stVisualRange *m_pVisualRange;
	cv::Size m_guiWinSz;
	COpenGl *m_pGL;
};

CLibGuiOpenGL::CLibGuiOpenGL( const string &LibXml )
{
#ifdef _DEBUG
	HMODULE hMod = GetModuleHandle(_T("LibGuiOpenGLD.dll"));
#else
	HMODULE hMod = GetModuleHandle(_T("LibGuiOpenGL.dll"));
#endif
	TCHAR szBuffer[MAX_PATH] = {0};
	GetModuleFileName(hMod, szBuffer, sizeof(szBuffer) / sizeof(TCHAR) - 1);

	// 不支持中文路径
	const size_t size = wcslen(szBuffer) * 2 + 2;
	std::vector<char> buffer(size);
	wcstombs(&buffer[0], szBuffer, size);
	std::string str(&buffer[0]);

	int pos = str.find_last_of('\\', str.length());

	m_LibXmlPath = str.substr(0, pos) + "\\" + LibXml;
	m_LibXmlName = LibXml;
}

bool CLibGuiOpenGL::Init()
{
	m_pVisualRange = new stVisualRange;

// 	if (!LoadParam())
// 	{
// 		//delete m_pVisualRange;
// 		//m_pVisualRange = NULL;
// 
// 		return false;
// 	}

	m_pGL = COpenGl::GetInstance();
	m_pGL->Set_gluPerspective_zfar(DBL_MAX/*m_gluPerspective_zfar*/);
	m_pGL->Set_VisualRange(m_pVisualRange);
	m_pGL->Show(600/*m_guiWinSz.width*/, 600/*m_guiWinSz.height*/);

	return true;
}

void CLibGuiOpenGL::UnInit()
{
	if (m_pGL)
	{
		COpenGl::Destory();
		m_pGL = NULL;
	}

	if (m_pVisualRange)
	{
		delete m_pVisualRange;
		m_pVisualRange = NULL;
	}

	printf("LibGuiOpenGL UnInit Success.\n");
	delete this;
	
	return;
}

bool CLibGuiOpenGL::LoadParam()
{
	TiXmlDocument tDoc;

	bool bRet = tDoc.LoadFile(m_LibXmlPath.c_str());
	if (!bRet)
	{
		return false;
	}

	string strReadTemp;
	TiXmlElement *tRootElement = tDoc.RootElement();	// LibGuiOpenGL
	strReadTemp = tRootElement->Value();
	if (strReadTemp != "LibGuiOpenGL")
	{
		cout << tRootElement->Value() << endl;
		return false;
	}

	enum emSubItemType{ em_gluPerspective_zfar, em_visualRange, em_MaxPointsLimit, em_guiSize, em_Unsupport};
	emSubItemType tCurrSubItemType;

	TiXmlElement *tSubItem = tRootElement->FirstChildElement();
	while (tSubItem != NULL)
	{
		strReadTemp = tSubItem->Value();
		if (strReadTemp == "gluPerspective_zfar")
		{
			tCurrSubItemType = em_gluPerspective_zfar;
		}
		else if (strReadTemp == "visualRange")
		{
			tCurrSubItemType = em_visualRange;
		}
		else if (strReadTemp == "MaxPointsLimit")
		{
			tCurrSubItemType = em_MaxPointsLimit;
		}
		else if (strReadTemp == "guiSize")
		{
			tCurrSubItemType = em_guiSize;
		}
		else
		{
			tCurrSubItemType = em_Unsupport;
		}

		string strElementTitleTemp, strElementValueTemp;
		TiXmlElement *tElement;
		switch (tCurrSubItemType)
		{
		case em_gluPerspective_zfar:
			strElementValueTemp = tSubItem->FirstChild()->Value();
			m_gluPerspective_zfar = atof(strElementValueTemp.c_str());
			break;
		case em_visualRange:
			tElement = tSubItem->FirstChildElement();
			while (tElement != NULL)
			{
				strElementTitleTemp = tElement->Value();
				strElementValueTemp = tElement->FirstChild()->Value();

				if (strElementTitleTemp == "x_min" && strElementValueTemp.length() > 0)
				{
					if (strElementValueTemp != "nil")
					{
						m_pVisualRange->x_min = atoi(strElementValueTemp.c_str());
						m_pVisualRange->b_x_min_nil = false;
					}
				}
				else if (strElementTitleTemp == "x_max" && strElementValueTemp.length() > 0)
				{
					if (strElementValueTemp != "nil")
					{
						m_pVisualRange->x_max = atoi(strElementValueTemp.c_str());
						m_pVisualRange->b_x_max_nil = false;
					}
				}
				else if (strElementTitleTemp == "y_min" && strElementValueTemp.length() > 0)
				{
					if (strElementValueTemp != "nil")
					{
						m_pVisualRange->y_min = atoi(strElementValueTemp.c_str());
						m_pVisualRange->b_y_min_nil = false;
					}
				}
				else if (strElementTitleTemp == "y_max" && strElementValueTemp.length() > 0)
				{
					if (strElementValueTemp != "nil")
					{
						m_pVisualRange->y_max = atoi(strElementValueTemp.c_str());
						m_pVisualRange->b_y_max_nil = false;
					}
				}
				else if (strElementTitleTemp == "z_min" && strElementValueTemp.length() > 0)
				{
					if (strElementValueTemp != "nil")
					{
						m_pVisualRange->z_min = atoi(strElementValueTemp.c_str());
						m_pVisualRange->b_z_min_nil = false;
					}
				}
				else if (strElementTitleTemp == "z_max" && strElementValueTemp.length() > 0)
				{
					if (strElementValueTemp != "nil")
					{
						m_pVisualRange->z_max = atoi(strElementValueTemp.c_str());
						m_pVisualRange->b_z_max_nil = false;
					}
				}

				tElement = tElement->NextSiblingElement();
			}
			break;
		case em_MaxPointsLimit:
			strElementValueTemp = tSubItem->FirstChild()->Value();
			m_iMaxPointsLimit = atoi(strElementValueTemp.c_str());
			break;
		case em_guiSize:
			tElement = tSubItem->FirstChildElement();
			while (tElement != NULL)
			{
				strElementTitleTemp = tElement->Value();
				strElementValueTemp = tElement->FirstChild()->Value();

				if (strElementTitleTemp == "width" && strElementValueTemp.length() > 0)
				{
					m_guiWinSz.width = atoi(strElementValueTemp.c_str());
				}
				else if (strElementTitleTemp == "height" && strElementValueTemp.length() > 0)
				{
					m_guiWinSz.height = atoi(strElementValueTemp.c_str());
				}

				tElement = tElement->NextSiblingElement();
			}
			break;
		case em_Unsupport:
		default:
			continue;
		}

		tSubItem = tSubItem->NextSiblingElement();
	}

	return true;
}

void CLibGuiOpenGL::AddRefPoints(Mat RefPoints, Mat Color)
{
	m_pGL->AddRefPoints(RefPoints,Color);
}
void CLibGuiOpenGL::AddRefPoints(double x, double y, double heading, Mat Color/* = Mat()*/)
{
	m_pGL->AddRefPoints(x,y,heading,Color);
}
void CLibGuiOpenGL::AddPointsRelative(Mat Points, Mat Pose, Mat Color)
{
	m_pGL->AddPointsRelative(Points,Pose,Color);
}
void CLibGuiOpenGL::AddPointsRelative(double x, double y, double heading, Mat Points, Mat Color/* = Mat()*/)
{
	m_pGL->AddPointsRelative(x, y, heading, Points, Color);
}
void CLibGuiOpenGL::AddLinesRelative(Mat Lines, Mat Pose, Mat Color)
{
	m_pGL->AddLinesRelative(Lines,Pose,Color);
}
void CLibGuiOpenGL::AddBreakLinesRelative(Mat Lines, Mat Pose, Mat Color)
{
	m_pGL->AddBreakLinesRelative(Lines,Pose,Color);
}
void CLibGuiOpenGL::AddSphereRelative(Point3d pt,float sz, Mat Pose, Mat Color)
{
	m_pGL->AddSphereRelative(pt, sz, Pose, Color);
}
void CLibGuiOpenGL::Clear()
{
	m_pGL->Clear();
}
void CLibGuiOpenGL::ClearRefPoints()
{
	m_pGL->ClearRefPoints();
}
void CLibGuiOpenGL::ClearPoints()
{
	m_pGL->ClearPoints();
}
void CLibGuiOpenGL::ClearLines()
{
	m_pGL->ClearLines();
}
void CLibGuiOpenGL::ClearBreakLines()
{
	m_pGL->ClearBreakLines();
}
void CLibGuiOpenGL::ClearSphere()
{
	m_pGL->ClearSphere();
}
void CLibGuiOpenGL::SetCameraMode(int nMode)
{
	m_pGL->SetCameraMode(nMode);
}
void CLibGuiOpenGL::SetScale(double dScale)
{
	m_pGL->SetScale(dScale);
}

void CLibGuiOpenGL::SetAxisLenth(float dLenth)
{
	m_pGL->SetAxisLenth(dLenth);
}

extern "C" LIBGUIOPENGL_API LIBGUIOPENGL_HANDLE GetLibGuiOpenGL(const std::string &LibXml)
{
	ILibGuiOpenGL *p = new CLibGuiOpenGL(LibXml);
	if (!((CLibGuiOpenGL *)p)->Init())
	{
		//((CLibGuiOpenGL *)p)->UnInit();
		p->UnInit();
		return NULL;
	}

	return p;
}
