
// TestDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "Test.h"
#include "TestDlg.h"
#include "afxdialogex.h"


using namespace cv;

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
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


// CTestDlg 对话框




CTestDlg::CTestDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CTestDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	m_nCont = 0;
}

void CTestDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_STATIC_image, m_CStaticImage);
}

BEGIN_MESSAGE_MAP(CTestDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_WM_TIMER()
	ON_BN_CLICKED(IDOK, &CTestDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDC_BUTTON1, &CTestDlg::OnBnClickedButton1)
END_MESSAGE_MAP()


// CTestDlg 消息处理程序

BOOL CTestDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
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

	// 设置此对话框的图标。当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO: 在此添加额外的初始化代码
	m_CStaticImage.MoveWindow(0,0,2000,2000,1);
//	SetTimer(0, 10, 0);

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void CTestDlg::OnSysCommand(UINT nID, LPARAM lParam)
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

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CTestDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CTestDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CTestDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	m_nCont++;
	Mat mat = Mat::zeros(1000,1000, CV_8UC3);
	char szDisp[256] = {0};
	sprintf(szDisp, "%d", m_nCont);
	putText(mat, szDisp, Point(100,100), 0, 1, CV_RGB(255,255,255),2);

	vector<uchar> buf;
	imencode(".bmp", mat, buf);

	
	HGLOBAL m_hGlobal = GlobalAlloc(GMEM_MOVEABLE, buf.size());
	void* m_pGlobalBuf = GlobalLock(m_hGlobal);
	memcpy(m_pGlobalBuf, buf.data(), buf.size());
	IStream* m_pStream = 0;
	CreateStreamOnHGlobal(m_hGlobal, TRUE, & m_pStream);

	
//	MatToCImage(mat, img);
	img.Destroy();
	img.Load(m_pStream);

	CRect rt;
	rt.top = 0; rt.left = 0; rt.right = mat.cols; rt.bottom = mat.rows;
	img.Draw(m_CStaticImage.GetWindowDC()->m_hDC, rt);
	//		OnPaint();

	GlobalUnlock(m_hGlobal);
	m_pStream->Release();
	FreeResource(m_pGlobalBuf);

	CDialogEx::OnTimer(nIDEvent);
}

void CTestDlg::MatToCImage(cv::Mat& mat, CImage& cimage)
{
	if (0 == mat.total())  
	{  
		return;  
	}  


	int nChannels = mat.channels();  
	if ((1 != nChannels) && (3 != nChannels))  
	{  
		return;  
	}  
	int nWidth    = mat.cols;  
	int nHeight   = mat.rows;  


	//重建cimage  
	cimage.Destroy();  
	cimage.Create(nWidth, nHeight, 8 * nChannels);  


	//拷贝数据  


	uchar* pucRow;                                  //指向数据区的行指针  
	uchar* pucImage = (uchar*)cimage.GetBits();     //指向数据区的指针  
	int nStep = cimage.GetPitch();                  //每行的字节数,注意这个返回值有正有负  


	if (1 == nChannels)                             //对于单通道的图像需要初始化调色板  
	{  
		RGBQUAD* rgbquadColorTable;  
		int nMaxColors = 256;  
		rgbquadColorTable = new RGBQUAD[nMaxColors];  
		cimage.GetColorTable(0, nMaxColors, rgbquadColorTable);  
		for (int nColor = 0; nColor < nMaxColors; nColor++)  
		{  
			rgbquadColorTable[nColor].rgbBlue = (uchar)nColor;  
			rgbquadColorTable[nColor].rgbGreen = (uchar)nColor;  
			rgbquadColorTable[nColor].rgbRed = (uchar)nColor;  
		}  
		cimage.SetColorTable(0, nMaxColors, rgbquadColorTable);  
		delete []rgbquadColorTable;  
	}  


	for (int nRow = 0; nRow < nHeight; nRow++)  
	{  
		pucRow = (mat.ptr<uchar>(nRow));  
		for (int nCol = 0; nCol < nWidth; nCol++)  
		{  
			if (1 == nChannels)  
			{  
				*(pucImage + nRow * nStep + nCol) = pucRow[nCol];  
			}  
			else if (3 == nChannels)  
			{  
				for (int nCha = 0 ; nCha < 3; nCha++)  
				{  
					*(pucImage + nRow * nStep + nCol * 3 + nCha) = pucRow[nCol * 3 + nCha];  
				}             
			}  
		}     
	}  
}

void CTestDlg::OnBnClickedOk()
{
	// TODO: 在此添加控件通知处理程序代码
	CDialogEx::OnOK();
}


void CTestDlg::OnBnClickedButton1()
{
	// TODO: 在此添加控件通知处理程序代码
	m_nCont++;
	Mat mat = Mat::zeros(1000,1000, CV_8UC3);
	char szDisp[256] = {0};
	sprintf(szDisp, "%d", m_nCont);
	putText(mat, szDisp, Point(100,100), 0, 1, CV_RGB(255,255,255),2);

	vector<uchar> buf;
	imencode(".bmp", mat, buf);


	HGLOBAL m_hGlobal = GlobalAlloc(GMEM_MOVEABLE, buf.size());
	void* m_pGlobalBuf = GlobalLock(m_hGlobal);
	memcpy(m_pGlobalBuf, buf.data(), buf.size());
	IStream* m_pStream = 0;
	CreateStreamOnHGlobal(m_hGlobal, TRUE, & m_pStream);


	//	MatToCImage(mat, img);
	img.Destroy();
	img.Load(m_pStream);

	CRect rt;
	rt.top = 0; rt.left = 0; rt.right = mat.cols; rt.bottom = mat.rows;
	img.Draw(m_CStaticImage.GetWindowDC()->m_hDC, rt);
	//		OnPaint();

	GlobalUnlock(m_hGlobal);
	m_pStream->Release();
	FreeResource(m_pGlobalBuf);
}
