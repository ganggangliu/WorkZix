
// ProjectorMappingDlg.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "ProjectorMapping.h"
#include "ProjectorMappingDlg.h"
#include "afxdialogex.h"


using namespace cv;
using namespace std;

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


// CProjectorMappingDlg �Ի���




CProjectorMappingDlg::CProjectorMappingDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CProjectorMappingDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
// 	AllocConsole();
// 	freopen("CON", "r", stdin );
// 	freopen("CON", "w", stdout);
// 	freopen("CON", "w", stderr);
}

void CProjectorMappingDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CProjectorMappingDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON_DISP_SET, &CProjectorMappingDlg::OnBnClickedButtonDispSet)
	ON_BN_CLICKED(IDC_BUTTON_START, &CProjectorMappingDlg::OnBnClickedButtonStart)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_BUTTON_START_CALIB, &CProjectorMappingDlg::OnBnClickedButtonStartCalib)
	ON_BN_CLICKED(IDC_BUTTON_PROJECT, &CProjectorMappingDlg::OnBnClickedButtonProject)
	ON_BN_CLICKED(IDC_BUTTON_PROJECT_VIDEO, &CProjectorMappingDlg::OnBnClickedButtonProjectVideo)
	ON_BN_CLICKED(IDC_BUTTON_CAMERA_VIEW, &CProjectorMappingDlg::OnBnClickedButtonCameraView)
	ON_WM_DESTROY()
	ON_WM_CLOSE()
END_MESSAGE_MAP()


// CProjectorMappingDlg ��Ϣ�������
bool WINAPI ProjectImageDisplayCallBack(Mat& image, void* pUser);
BOOL CProjectorMappingDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

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

	m_pDlgMonitorSet = new CDialogMonitorSet;

	m_pDialogProj = new CDialogProjectWin;
	m_pDialogProj->Init(m_pDlgMonitorSet->m_DispRect);
	m_pDialogProj->Create(IDD_DIALOG_PROJECT_WIN, this);
	m_pDialogProj->SetBackgroundColor(RGB(0,0,0));
	m_pDialogProj->ShowWindow(SW_SHOWNORMAL);
	
	
	MONITORINFO monitorinfo;
	if (!m_pDlgMonitorSet->GetProjectorInfo(monitorinfo))
	{
		MessageBox(CString("No projector!!!"), CString("Error"));
		return TRUE;
	}
	CProjectorParam Param;
	Param.nCameraInd = 1;
	Param.ProjectorRect.x = 0;
	Param.ProjectorRect.y = 0;
	Param.ProjectorRect.width = monitorinfo.rcMonitor.right - monitorinfo.rcMonitor.left;
	Param.ProjectorRect.height = monitorinfo.rcMonitor.bottom - monitorinfo.rcMonitor.top;
	m_CalibOpr.Init(Param);
	m_CalibOpr.SetProjectorDispCallBack(ProjectImageDisplayCallBack, this);
	

	return TRUE;  // ���ǽ��������õ��ؼ������򷵻� TRUE
}

void CProjectorMappingDlg::OnSysCommand(UINT nID, LPARAM lParam)
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

void CProjectorMappingDlg::OnPaint()
{
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
HCURSOR CProjectorMappingDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



BOOL CProjectorMappingDlg::PreTranslateMessage(MSG* pMsg)
{
	// TODO: �ڴ����ר�ô����/����û���
	if (pMsg->message == WM_KEYDOWN)
	{
		if (VK_RETURN == pMsg->wParam)
		{
			return FALSE;
		}
	}

	return CDialogEx::PreTranslateMessage(pMsg);
}

void CProjectorMappingDlg::OnBnClickedButtonDispSet()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	m_pDlgMonitorSet->DoModal();
}


void CProjectorMappingDlg::OnBnClickedButtonStart()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	return;
	if (!m_cap.open(m_pDlgMonitorSet->m_nCamId))
	{
		MessageBox(CString("Open camera failed!"));
		return;
	}

	SetTimer(0, 100, 0);

	
}


void CProjectorMappingDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: �ڴ������Ϣ�����������/�����Ĭ��ֵ
//	Mat mat = imread("C:\\Users\\Sense\\Desktop\\123.png");
	Mat mat;
	m_cap >> mat; 
	m_pDialogProj->ProjectImage(mat);

	CDialogEx::OnTimer(nIDEvent);
}

bool WINAPI ProjectImageDisplayCallBack(Mat& image, void* pUser)
{
	CProjectorMappingDlg* pDlg = (CProjectorMappingDlg*)pUser;
	return pDlg->ProjectImage(image);
}

void CProjectorMappingDlg::OnBnClickedButtonStartCalib()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	m_CalibOpr.m_Param.nCameraInd = m_pDlgMonitorSet->m_nCamId;
	if (m_CalibOpr.RunCalibEx())
	{
		MessageBox(CString("Calibration finished!"));
	}
	else
	{
		MessageBox(CString("Calibration failed!"));
	}
}

bool CProjectorMappingDlg::ProjectImage(cv::Mat image)
{
	return m_pDialogProj->ProjectImage(image);
}

void CProjectorMappingDlg::OnBnClickedButtonProject()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CString filter;  
	filter="";  
	CFileDialog dlg(TRUE,NULL,NULL,OFN_HIDEREADONLY,filter);  
	if(dlg.DoModal()==IDOK)  
	{  
		CString str;  
		str=dlg.GetPathName();  
		std::string s = (LPCSTR)(CStringA)(str);
		Mat image = imread(s);
		Mat out = m_CalibOpr.Rectify(image);
		if (out.data)
		{
			m_pDialogProj->ProjectImage(out);
			namedWindow("RectifiedImage", 0);
			imshow("RectifiedImage", out);
			waitKey(1);
		}
		else
		{
			MessageBox(CString("Project failed!"));
		}
	}  
}

void CProjectorMappingDlg::OnBnClickedButtonProjectVideo()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������

	CString filter;  
	filter="";  
	CFileDialog dlg(TRUE,NULL,NULL,OFN_HIDEREADONLY,filter);  
//	namedWindow("Video", 0);
	if(dlg.DoModal()==IDOK)  
	{  
		CString str;  
		str=dlg.GetPathName();  
		std::string s = (LPCSTR)(CStringA)(str);
		if (m_cap.isOpened())
		{
			m_cap.release();
		}
		m_cap.open(s);
		if (!m_cap.isOpened())
		{
			MessageBox(CString("Open video faild!"));
			return;
		}
		Mat frame;
		while(1)
		{
			bool bRt = m_cap.read(frame);
			if (!bRt || frame.data == 0)
			{
				MessageBox(CString("Read faild!"));
				m_cap.release();
				return;
			}
			Mat out = m_CalibOpr.Rectify(frame);
			if (out.data == 0)
			{
				MessageBox(CString("Rectify faild!"));
				m_cap.release();
				return;
			}
			m_pDialogProj->ProjectImage(out);
			namedWindow("RectifiedImage", 0);
			imshow("RectifiedImage", out);
 			waitKey(1);
		}
	}

}


void CProjectorMappingDlg::OnBnClickedButtonCameraView()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	Mat mat = Mat::zeros(m_CalibOpr.m_Param.ProjectorRect.height, 
		m_CalibOpr.m_Param.ProjectorRect.width, CV_8UC3);
	mat = Scalar(255,255,255);
	m_pDialogProj->ProjectImage(mat);
	namedWindow("Camera" ,0);
	imshow("Camera", mat);
	waitKey(100);
	if (m_cap.isOpened())
	{
		m_cap.release();
	}
	m_cap.open(m_pDlgMonitorSet->m_nCamId);
	m_cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
	m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
	if (!m_cap.isOpened())
	{
		destroyWindow("Camera");
		MessageBox(CString("Open camera failed!"));
		return;
	}
	while(1)
	{
		Mat frame;
		m_cap >> frame;
		if (frame.data == 0)
		{
			m_cap.release();
			return;
		}
		namedWindow("Camera" ,0);
		imshow("Camera", frame);
		int nRt = waitKey(1);
		if (nRt >= 0)
		{
			m_cap.release();
			destroyWindow("Camera");
			return;
		}
	}
}


void CProjectorMappingDlg::OnDestroy()
{
	CDialogEx::OnDestroy();

	// TODO: �ڴ˴������Ϣ����������
}


BOOL CProjectorMappingDlg::DestroyWindow()
{
	// TODO: �ڴ����ר�ô����/����û���

	return CDialogEx::DestroyWindow();
}


void CProjectorMappingDlg::OnClose()
{
	// TODO: �ڴ������Ϣ�����������/�����Ĭ��ֵ
	if (m_pDlgMonitorSet)
	{
		delete m_pDlgMonitorSet;
		m_pDlgMonitorSet = 0;
	}
	if (m_cap.isOpened())
	{
		m_cap.release();
	}
	if (m_pDialogProj)
	{
		delete m_pDialogProj;
		m_pDialogProj = 0;
	}

	CDialogEx::OnClose();
}
