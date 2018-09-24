
// BaslerReplayStartUpDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "BaslerReplayStartUp.h"
#include "BaslerReplayStartUpDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

int TcharToChar (const TCHAR * tchar, char * _char)  
{  
	int iLength ;  
	//获取字节长度   
	iLength = WideCharToMultiByte(CP_ACP, 0, tchar, -1, NULL, 0, NULL, NULL);  
	//将tchar值赋给_char    
	WideCharToMultiByte(CP_ACP, 0, tchar, -1, _char, iLength, NULL, NULL);  

	return iLength;
}  

int CharToTchar (const char * _char, TCHAR * tchar)  
{  
	int iLength ;  

	iLength = MultiByteToWideChar (CP_ACP, 0, _char, strlen (_char) + 1, NULL, 0) ;  
	MultiByteToWideChar (CP_ACP, 0, _char, strlen (_char) + 1, tchar, iLength) ; 

	return iLength;
} 
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

// CBaslerReplayStartUpDlg 对话框


CBaslerReplayStartUpDlg::CBaslerReplayStartUpDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CBaslerReplayStartUpDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	memset(m_ModulePath, 0, MAX_PATH);
	memset(m_HistoryFilePath, 0, MAX_PATH);
	memset(m_ExePath, 0, MAX_PATH);
}

void CBaslerReplayStartUpDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_EDIT_PATH, m_CEditPathInput);
}

BEGIN_MESSAGE_MAP(CBaslerReplayStartUpDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON_SELECT, &CBaslerReplayStartUpDlg::OnBnClickedButtonSelect)
	ON_BN_CLICKED(IDOK, &CBaslerReplayStartUpDlg::OnBnClickedOk)
END_MESSAGE_MAP()


// CBaslerReplayStartUpDlg 消息处理程序

BOOL CBaslerReplayStartUpDlg::OnInitDialog()
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
	GetModuleFileName(0, m_ModulePath, MAX_PATH);
	for (unsigned int i = wcslen(m_ModulePath); i>=0; i--)
	{
		if (m_ModulePath[i] == '/' || m_ModulePath[i] == '\\')
		{
			m_ModulePath[i] = 0;
			break;
		}
	}

	StrCpy(m_HistoryFilePath, m_ModulePath);
	StrCat(m_HistoryFilePath,_T("/BaslerReplay.his"));

	StrCpy(m_ExePath, m_ModulePath);
	StrCat(m_ExePath,_T("/BaslerReplay.exe"));

	TCHAR HistoryPath[MAX_PATH] = {0};
	CFile f;
	if (f.Open(m_HistoryFilePath, CFile::modeRead))
	{
		char HistoryPath_[MAX_PATH] = {0};
		f.Read(HistoryPath_, MAX_PATH);
		CharToTchar(HistoryPath_, HistoryPath);
		f.Close();
	}

	m_CEditPathInput.SetWindowText(HistoryPath);

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void CBaslerReplayStartUpDlg::OnSysCommand(UINT nID, LPARAM lParam)
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

void CBaslerReplayStartUpDlg::OnPaint()
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
HCURSOR CBaslerReplayStartUpDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

static int CALLBACK BrowseCallbackProc(HWND hWnd, UINT uMsg, LPARAM , LPARAM lpData)     
{  
	if(uMsg == BFFM_INITIALIZED)  
	{  
		CTreeCtrl   treePath;  
		HTREEITEM   hItemSel;  
		::SendMessage(hWnd, BFFM_SETSELECTION, TRUE, lpData);  
		treePath.SubclassWindow(::GetDlgItem(hWnd, 0x3741));  
		hItemSel    = treePath.GetSelectedItem();  
		treePath.Expand(hItemSel, TVE_COLLAPSE);  
		treePath.UnsubclassWindow();  
	}   
	return 0;    
}  

void CBaslerReplayStartUpDlg::OnBnClickedButtonSelect()
{
	// TODO: 在此添加控件通知处理程序代码
	TCHAR HistoryPath[MAX_PATH] = {0};
	CFile f;
	if (f.Open(m_HistoryFilePath, CFile::modeRead))
	{
		char HistoryPath_[MAX_PATH] = {0};
		f.Read(HistoryPath_, MAX_PATH);
		CharToTchar(HistoryPath_, HistoryPath);
		f.Close();
	}
	
	BROWSEINFO bi;
	ZeroMemory(&bi,sizeof(BROWSEINFO));
	bi.lpfn   =   BrowseCallbackProc;     
	bi.lParam   =   (LPARAM)HistoryPath;
	LPMALLOC pMalloc;
	LPITEMIDLIST pidl = SHBrowseForFolder(&bi);

	if (pidl==NULL)
		return;

	TCHAR path[MAX_PATH] = {0}; 
	SHGetPathFromIDList(pidl,path);

	m_CEditPathInput.SetWindowText(path);
}


void CBaslerReplayStartUpDlg::OnBnClickedOk()
{
	// TODO: 在此添加控件通知处理程序代码
	CString szProPath;
	m_CEditPathInput.GetWindowText(szProPath);

	ShellExecute(NULL, _T("open"), m_ExePath, szProPath,NULL,SW_SHOWNORMAL); 

	TCHAR HistoryPath[MAX_PATH] = {0};
	StrCpy(HistoryPath, szProPath);
	CFile f;
	if (f.Open(m_HistoryFilePath, CFile::modeWrite|CFile::modeCreate))
	{
		char HistoryPath_[MAX_PATH] = {0};
		TcharToChar(HistoryPath, HistoryPath_);
		f.Write(HistoryPath_, MAX_PATH);
		f.Close();
	}


	CDialogEx::OnOK();
}
