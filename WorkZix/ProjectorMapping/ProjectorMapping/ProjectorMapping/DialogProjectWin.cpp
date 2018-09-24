// DialogProjectWin.cpp : 实现文件
//

#include "stdafx.h"
#include "ProjectorMapping.h"
#include "DialogProjectWin.h"
#include "afxdialogex.h"

using namespace cv;


// CDialogProjectWin 对话框

IMPLEMENT_DYNAMIC(CDialogProjectWin, CDialogEx)

CDialogProjectWin::CDialogProjectWin(CWnd* pParent /*=NULL*/)
	: CDialogEx(CDialogProjectWin::IDD, pParent)
{
	m_hOpenCvWnd = 0;
	m_bIsFull = true;
	m_pGlobalBuf = 0;
	InitializeCriticalSection(&m_cs);
	m_DispRect = CRect(0, 0, 0, 0);
	m_nDispCont = 0;
}

CDialogProjectWin::~CDialogProjectWin()
{
}

void CDialogProjectWin::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_STATIC_PROJECT, m_CStatic_Project);
}


BEGIN_MESSAGE_MAP(CDialogProjectWin, CDialogEx)
	ON_WM_PAINT()
	ON_WM_TIMER()
	ON_WM_ERASEBKGND()
END_MESSAGE_MAP()


// CDialogProjectWin 消息处理程序


BOOL CDialogProjectWin::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	
	// TODO:  在此添加额外的初始化

	m_CStatic_Project.SetWindowPos(0, 0, 0, 2000, 2000, SWP_SHOWWINDOW);

	SetTimer(0, 20, 0);
	
	ModifyStyle(WS_CAPTION,0,0); 
	MoveWindow(m_DispRect, TRUE);

	return TRUE;  // return TRUE unless you set the focus to a control
	// 异常: OCX 属性页应返回 FALSE
}

int CDialogProjectWin::Init(CRect DispRect)
{
	m_DispRect = DispRect;
	Mat mat = Mat::zeros(m_DispRect.Height(), m_DispRect.Width(), CV_8UC3);
	vector<uchar> buf;
	imencode(".bmp", mat, buf);
// 	m_hGlobal = GlobalAlloc(GMEM_FIXED, buf.size());
// 	m_pGlobalBuf = GlobalLock(m_hGlobal);
// 	CreateStreamOnHGlobal(m_hGlobal, TRUE, & m_pStream);

	return 1;
}

void CDialogProjectWin::OnPaint()
{
	CPaintDC dc(this); // device context for painting

	CDialogEx::OnPaint();
}

BOOL CDialogProjectWin::PreTranslateMessage(MSG* pMsg)
{
	// TODO: 在此添加专用代码和/或调用基类

 	if (pMsg->message == WM_KEYDOWN)
 	{
 		m_bIsFull = !m_bIsFull;
 		if (m_bIsFull)
 		{
 			ModifyStyle(WS_CAPTION,0,0); 
// 			ShowWindow(SW_MINIMIZE);
			MoveWindow(m_DispRect, TRUE);
// 			::SetWindowPos(GetSafeHwnd(),HWND_TOPMOST, 
// 				m_DispRect.left, m_DispRect.top, m_DispRect.Width(), m_DispRect.Height(),SWP_SHOWWINDOW);
 			ShowWindow(SW_MAXIMIZE);
 		}
 		else
 		{
 			ModifyStyle(0,WS_CAPTION,0); 
 			MoveWindow(0,0, 300, 300, TRUE);
 		}
 		if (VK_RETURN == pMsg->wParam || VK_ESCAPE == pMsg->wParam)
 		{
 			return FALSE;
 		}
 	}
	
	OnPaint();
	
 	return CDialogEx::PreTranslateMessage(pMsg);
}

bool CDialogProjectWin::ProjectImage(cv::Mat& image)
{
	EnterCriticalSection(&m_cs);
	image.copyTo(m_matBuf);
	LeaveCriticalSection(&m_cs);
	
	return true;


 	imencode(".bmp", image, m_Buf);
  	memcpy(m_pGlobalBuf, m_Buf.data(), m_Buf.size());
	CImage img;
	img.Load(m_pStream);
	m_CStatic_Project.SetBitmap(img.Detach());


	return true;
}

void CDialogProjectWin::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	if (nIDEvent == 0)
	{
		if (m_matBuf.data == 0)
		{
			return;
		}
		m_nDispCont++;
		char szDisp[256] = {0};
		sprintf(szDisp, "%d", m_nDispCont);
		Mat matDisp;
		EnterCriticalSection(&m_cs);
		m_matBuf.copyTo(matDisp);
		LeaveCriticalSection(&m_cs);
//		putText(matDisp, szDisp, Point(100,100), 0, 1.0, CV_RGB(255,255,255));
		imencode(".bmp", matDisp, m_Buf);

		m_hGlobal = GlobalAlloc(GMEM_MOVEABLE, m_Buf.size());
		m_pGlobalBuf = GlobalLock(m_hGlobal);
		memcpy(m_pGlobalBuf, m_Buf.data(), m_Buf.size());
		CreateStreamOnHGlobal(m_hGlobal, TRUE, & m_pStream);

		CImage img;
		img.Load(m_pStream);

//		m_imageBuf.Load(m_pStream);

//		m_CStatic_Project.SetBitmap(m_imageBuf.Detach());
		CRect rt;
		rt.top = 0; rt.left = 0; rt.right = matDisp.cols; rt.bottom = matDisp.rows;
		CDC* pDC = m_CStatic_Project.GetWindowDC();
		img.Draw(pDC->m_hDC, rt);
		ReleaseDC(pDC);
//		OnPaint();

		GlobalUnlock(m_hGlobal);
		m_pStream->Release();
		FreeResource(m_pGlobalBuf);
	}


	CDialogEx::OnTimer(nIDEvent);
}


BOOL CDialogProjectWin::OnEraseBkgnd(CDC* pDC)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
//	return TRUE;

	return CDialogEx::OnEraseBkgnd(pDC);
}
