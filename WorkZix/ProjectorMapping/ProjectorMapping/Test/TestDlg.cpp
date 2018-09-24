
// TestDlg.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "Test.h"
#include "TestDlg.h"
#include "afxdialogex.h"


using namespace cv;

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


// CTestDlg �Ի���




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


// CTestDlg ��Ϣ�������

BOOL CTestDlg::OnInitDialog()
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
	m_CStaticImage.MoveWindow(0,0,2000,2000,1);
//	SetTimer(0, 10, 0);

	return TRUE;  // ���ǽ��������õ��ؼ������򷵻� TRUE
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

// �����Ի��������С����ť������Ҫ����Ĵ���
//  �����Ƹ�ͼ�ꡣ����ʹ���ĵ�/��ͼģ�͵� MFC Ӧ�ó���
//  �⽫�ɿ���Զ���ɡ�

void CTestDlg::OnPaint()
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
HCURSOR CTestDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CTestDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: �ڴ������Ϣ�����������/�����Ĭ��ֵ
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


	//�ؽ�cimage  
	cimage.Destroy();  
	cimage.Create(nWidth, nHeight, 8 * nChannels);  


	//��������  


	uchar* pucRow;                                  //ָ������������ָ��  
	uchar* pucImage = (uchar*)cimage.GetBits();     //ָ����������ָ��  
	int nStep = cimage.GetPitch();                  //ÿ�е��ֽ���,ע���������ֵ�����и�  


	if (1 == nChannels)                             //���ڵ�ͨ����ͼ����Ҫ��ʼ����ɫ��  
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
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CDialogEx::OnOK();
}


void CTestDlg::OnBnClickedButton1()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
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
