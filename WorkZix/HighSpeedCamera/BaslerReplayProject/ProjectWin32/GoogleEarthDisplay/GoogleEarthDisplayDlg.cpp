
// GoogleEarthDisplayDlg.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "GoogleEarthDisplay.h"
#include "GoogleEarthDisplayDlg.h"
#include "afxdialogex.h"
#include <string>
#include <vector>
#include <cstdint>
#include <fstream> 
#include "ATDispatch.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// ����Ӧ�ó��򡰹��ڡ��˵���� CAboutDlg �Ի���

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// �Ի�������
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

// ʵ��
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CGoogleEarthDisplayDlg �Ի���




CGoogleEarthDisplayDlg::CGoogleEarthDisplayDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CGoogleEarthDisplayDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	m_nDispCont = 0;
}

void CGoogleEarthDisplayDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_EDIT_LAT, m_CEdit_lat);
	DDX_Control(pDX, IDC_EDIT_LONG, m_CEdit_Long);
	DDX_Control(pDX, ID_StartGE, m_CBut_Google_Start);
	DDX_Control(pDX, IDC_BUT_FOCUS, m_CBut_Focus);
	DDX_Control(pDX, IDC_STATIC_LAT, m_CStatic_Lat);
	DDX_Control(pDX, IDC_STATIC_LON, m_CStatic_Lon);
	DDX_Control(pDX, IDC_BUT_READ_LOG, m_CBut_ReadLog);
	DDX_Control(pDX, IDC_BUT_CLEAR, m_CBut_Clear);
	DDX_Control(pDX, ID_StopGE, m_CBut_Stop);
	DDX_Control(pDX, IDC_STATIC1, m_CStetic1);
	DDX_Control(pDX, IDC_STATIC2, m_CStatic2);
	DDX_Control(pDX, IDC_STATIC_LOG_ID, m_CStatic_LogId);
	DDX_Control(pDX, IDC_STATIC_TIP, m_CStatic_TIP);
}

BEGIN_MESSAGE_MAP(CGoogleEarthDisplayDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(ID_StartGE, &CGoogleEarthDisplayDlg::OnBnClickedStartge)
	ON_BN_CLICKED(ID_StopGE, &CGoogleEarthDisplayDlg::OnBnClickedStopge)
	ON_WM_CLOSE()
	ON_WM_SIZE()
	ON_BN_CLICKED(IDC_BUT_FOCUS, &CGoogleEarthDisplayDlg::OnBnClickedButFocus)
	ON_BN_CLICKED(IDC_BUT_READ_LOG, &CGoogleEarthDisplayDlg::OnBnClickedButReadLog)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_BUT_CLEAR, &CGoogleEarthDisplayDlg::OnBnClickedButClear)
END_MESSAGE_MAP()


// CGoogleEarthDisplayDlg ��Ϣ�������

BOOL CGoogleEarthDisplayDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

//	ShowWindow(SW_MAXIMIZE);

	// ��������...���˵�����ӵ�ϵͳ�˵��С�

	// IDM_ABOUTBOX ������ϵͳ���Χ�ڡ�
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// ���ô˶Ի����ͼ�ꡣ��Ӧ�ó��������ڲ��ǶԻ���ʱ����ܽ��Զ�
	//  ִ�д˲���
	SetIcon(m_hIcon, TRUE);			// ���ô�ͼ��
	SetIcon(m_hIcon, FALSE);		// ����Сͼ��

	// TODO: �ڴ���Ӷ���ĳ�ʼ������
	CString szTip("Before All, you need to configurate some options in Google Earth MANUALLY below:\n\
1:\"Tool\",\"Option\",\"3D View\",Latitude and Longitude display = decimal degree\n\
2:\"Tool\",\"Option\",\"Navigation\",Forward speed = Slowest\n\
Run following commands in shell\n\
1:cd C:\\Program Files\\Google\\Google Earth\\client\n\
2:googleearth.exe /RegServer\n\
All above need configurated only once.");
	m_CStatic_TIP.SetWindowText(szTip);

	m_VecCtl.push_back(m_CEdit_lat.m_hWnd);
	m_VecCtl.push_back(m_CEdit_Long.m_hWnd);
	m_VecCtl.push_back(m_CBut_Google_Start.m_hWnd);
	m_VecCtl.push_back(m_CBut_Focus);
	m_VecCtl.push_back(m_CStatic_Lat);
	m_VecCtl.push_back(m_CStatic_Lon);
	m_VecCtl.push_back(m_CBut_ReadLog);
	m_VecCtl.push_back(m_CBut_Clear);
	m_VecCtl.push_back(m_CBut_Stop);
	m_VecCtl.push_back(m_CStetic1);
	m_VecCtl.push_back(m_CStatic2);
	m_VecCtl.push_back(m_CStatic_LogId);

	::SetWindowPos(this->m_hWnd, HWND_TOP, 0, 0, 1300, 750, SWP_SHOWWINDOW);

	char szModuleFileName[MAX_PATH] = {0};
	GetModuleFileName(0, szModuleFileName, MAX_PATH);
	char* pT = strrchr(szModuleFileName, '\\');
	if (pT != 0)
	{
		*pT = 0;
	}
	m_szModuleFilePath = CString(szModuleFileName);
	m_HistoryFilePath = m_szModuleFilePath + CString("\\GoogleEarthDisplay.hist");

	return TRUE;  // ���ǽ��������õ��ؼ������򷵻� TRUE
}

void CGoogleEarthDisplayDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// �����Ի��������С����ť������Ҫ����Ĵ���
//  �����Ƹ�ͼ�ꡣ����ʹ���ĵ�/��ͼģ�͵� MFC Ӧ�ó���
//  �⽫�ɿ���Զ���ɡ�

void CGoogleEarthDisplayDlg::OnPaint()
{
//	SetGEWindow();
	if (IsIconic())
	{
		CPaintDC dc(this); // ���ڻ��Ƶ��豸������

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// ʹͼ���ڹ����������о���
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// ����ͼ��
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
	
}

//���û��϶���С������ʱϵͳ���ô˺���ȡ�ù��
//��ʾ��
HCURSOR CGoogleEarthDisplayDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CGoogleEarthDisplayDlg::OnBnClickedStartge()
{
	bool bRt = m_GoogleEarthOpr.Create();
	if (bRt == false)
	{
		MessageBox("Start Google Earth failed!");
	}
	


	::SetParent(m_GoogleEarthOpr.m_GEHRenderWnd, this->m_hWnd);  
	SetGEWindow();

	
}


void CGoogleEarthDisplayDlg::OnBnClickedStopge()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������

}


void CGoogleEarthDisplayDlg::OnClose()
{
	// TODO: �ڴ������Ϣ�����������/�����Ĭ��ֵ


// 	::SendMessage(m_GoogleEarthOpr.m_GEHMainWnd, WM_CLOSE, 0, 0);  
// 	::SendMessage(m_GoogleEarthOpr.m_GEHRenderWnd, WM_CLOSE, 0, 0);  

	m_GoogleEarthOpr.Destroy();

	CDialogEx::OnClose();
}

void CGoogleEarthDisplayDlg::SetGEWindow()
{
	::SetWindowPos(m_GoogleEarthOpr.m_GEHMainWnd, HWND_BOTTOM, 0, 0, 0, 0,
		SWP_NOSIZE|SWP_HIDEWINDOW);
	CRect rect;
	GetClientRect(rect);
	::MoveWindow(m_GoogleEarthOpr.m_GEHRenderWnd, rect.left, rect.top,
		m_GoogleEarthOpr.m_nWindWidth, m_GoogleEarthOpr.m_nWindHeight,TRUE);

	int nCtlWidth = 100;
	for (unsigned int i = 0; i < m_VecCtl.size(); i++)
	{
		CRect CtlRect;
		::GetWindowRect(m_VecCtl[i], CtlRect);
		ScreenToClient(CtlRect);
		int nOriWidth = CtlRect.Width();
		CRect CtlRect_ = CtlRect;
		CtlRect_.left = CtlRect.left + m_GoogleEarthOpr.m_nWindWidth + 2;
		CtlRect_.right = CtlRect_.left + nOriWidth;
		::MoveWindow(m_VecCtl[i], CtlRect_.left, CtlRect_.top, CtlRect_.Width(), CtlRect_.Height(), TRUE);
	}
	
	CWnd::MoveWindow(0, 0,
		m_GoogleEarthOpr.m_nWindWidth + nCtlWidth + 20, m_GoogleEarthOpr.m_nWindHeight + 40,TRUE);

//	OnPaint();
}

void CGoogleEarthDisplayDlg::OnSize(UINT nType, int cx, int cy)
{
	CDialogEx::OnSize(nType, cx, cy);

//	SetGEWindow();

	// TODO: �ڴ˴������Ϣ����������
}


void CGoogleEarthDisplayDlg::OnBnClickedButFocus()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CString szLat;
	m_CEdit_lat.GetWindowText(szLat);
	CString szLong;
	m_CEdit_Long.GetWindowText(szLong);
	double dLat = atof(szLat);
	double dLong = atof(szLong);
	m_GoogleEarthOpr.FocusToPoint(dLat, dLong);
}

BOOL CGoogleEarthDisplayDlg::PreTranslateMessage(MSG* pMsg)  
{  
    // TODO: Add your specialized code here and/or call the base class  
    //�ж��Ƿ��ǰ�����Ϣ  
	if (pMsg->message == WM_KEYDOWN)
	{
		if (pMsg->wParam == 'A' || pMsg->wParam == 'a')
		{
			int aa = 0;
			CPoint pt;
			GetCursorPos(&pt);
			CRect ClientRect;
			ScreenToClient(ClientRect);
			CRect Rect11;
			::GetWindowRect(m_GoogleEarthOpr.m_GEHRenderWnd, Rect11);
			CPoint ptClient;
			ptClient.x = pt.x - Rect11.left;
			ptClient.y = pt.y - Rect11.top;
			double dLat = 0.0;
			double dLon = 0.0;
			double dAlt = 0.0;
			m_GoogleEarthOpr.GetGpsPointFromMouse(ptClient.x, ptClient.y,
				dLat, dLon, dAlt);

			CString szName;
			szName.Format("%.6f", dLat);
			m_CEdit_lat.SetWindowText(szName);
			szName.Format("%.6f", dLon);
			m_CEdit_Long.SetWindowText(szName);

			char szInfo[1024] = {0};
			sprintf(szInfo, "Latitude:%.7f\nLongitude:%.7f\nAltitude:%.3f", dLat, dLon, dAlt);
			MessageBox(szInfo);
//			m_GoogleEarthOpr.FocusToPoint(dLat, dLon);
		}
	}

    return CDialogEx::PreTranslateMessage(pMsg);  
}  

int CGoogleEarthDisplayDlg::ReadTrackLog(std::string szPath)
{
	std::ifstream fsKlm(szPath.c_str());
	if (!fsKlm.is_open())
	{
		return -1;
	}

	std::string szLine;
	bool bIsFindHead = false;
	std::string szLineT;
	while(fsKlm >> szLineT)
	{
		if (szLineT == "</coordinates>")
		{
			bIsFindHead = false;
			break;
		}
		if (bIsFindHead)
		{
			szLine += (szLineT + ",");
		}
		if (szLineT == "<coordinates>")
		{
			bIsFindHead = true;
		}
	}

	int nPos = 0;
	while(1)
	{
		int nNewPos = szLine.find(',', nPos);
		if (nNewPos < 0)
		{
			break;
		}
		std::string szFirst = szLine.substr(nPos,nNewPos-nPos);
		nPos = nNewPos+1;
		double dLon = atof(szFirst.c_str());

		nNewPos = szLine.find(',', nPos);
		if (nNewPos < 0)
		{
			break;
		}
		szFirst = szLine.substr(nPos,nNewPos-nPos);
		nPos = nNewPos+1;
		double dLat = atof(szFirst.c_str());

		nNewPos = szLine.find(',', nPos);
		if (nNewPos < 0)
		{
			break;
		}
		szFirst = szLine.substr(nPos,nNewPos-nPos);
		nPos = nNewPos+1;
		double dAlt = atof(szFirst.c_str());

		m_VecLatLog.push_back(dLat);
		m_VecLonLog.push_back(dLon);
		m_VecHeadLog.push_back(0.0);
	}

	return 1;
}

void CGoogleEarthDisplayDlg::OnBnClickedButReadLog()
{
	CFile f;
	char HistoryPath[MAX_PATH] = {0};
	if (f.Open(m_HistoryFilePath, CFile::modeRead))
	{
 		f.Read(HistoryPath, MAX_PATH);
 		f.Close();
	}

	CString path;
	CFileDialog fd(TRUE, "Select kml file", HistoryPath, OFN_HIDEREADONLY|OFN_OVERWRITEPROMPT,
		"(*.kml)|*.kml||");
	if (fd.DoModal() == IDOK)
	{
		path = fd.GetPathName();
	}

	if (path.GetLength() > 0)
	{
		if (f.Open(m_HistoryFilePath, CFile::modeWrite|CFile::modeCreate))
		{
			f.Write(path, path.GetLength());
			f.Close();
		}
	}

	ReadTrackLog(std::string(path));

	m_nDispCont = 0;
	SetTimer(0, 500, 0);
}


void CGoogleEarthDisplayDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: �ڴ������Ϣ�����������/�����Ĭ��ֵ
	if (nIDEvent == 0)
	{
		CString szDisp;
		szDisp.Format("Ind:%d", m_nDispCont);
		m_CStatic_LogId.SetWindowText(szDisp);
		if (m_VecLatLog.size() <= 0)
		{
			m_nDispCont = 0;
			KillTimer(0);
			MessageBox("No track points!");
		}
		if (m_nDispCont < m_VecLatLog.size())
		{
			if (m_GoogleEarthOpr.AddGpsPoint(m_VecLatLog[m_nDispCont], 
				m_VecLonLog[m_nDispCont],0) < 0)
			{
				m_nDispCont = 0;
				KillTimer(0);
				MessageBox("Add Gps Point error!");
			}
			else
			{
				m_nDispCont++;
			}
		}
		if (m_nDispCont >= m_VecLatLog.size())
		{
			m_nDispCont = 0;
			KillTimer(0);
			MessageBox("Finish!");
		}
	}

	CDialogEx::OnTimer(nIDEvent);
}

void CGoogleEarthDisplayDlg::OnBnClickedButClear()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	KillTimer(0);
	Sleep(500);
	m_nDispCont = 0;
	m_GoogleEarthOpr.ClearPoints();
	m_VecLatLog.clear();
	m_VecLonLog.clear();
	m_VecHeadLog.clear();
}
