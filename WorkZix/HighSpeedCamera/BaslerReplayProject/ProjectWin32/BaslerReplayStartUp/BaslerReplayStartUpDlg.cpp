
// BaslerReplayStartUpDlg.cpp : ʵ���ļ�
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
	//��ȡ�ֽڳ���   
	iLength = WideCharToMultiByte(CP_ACP, 0, tchar, -1, NULL, 0, NULL, NULL);  
	//��tcharֵ����_char    
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

// CBaslerReplayStartUpDlg �Ի���


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


// CBaslerReplayStartUpDlg ��Ϣ�������

BOOL CBaslerReplayStartUpDlg::OnInitDialog()
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

	return TRUE;  // ���ǽ��������õ��ؼ������򷵻� TRUE
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

// �����Ի��������С����ť������Ҫ����Ĵ���
//  �����Ƹ�ͼ�ꡣ����ʹ���ĵ�/��ͼģ�͵� MFC Ӧ�ó���
//  �⽫�ɿ���Զ���ɡ�

void CBaslerReplayStartUpDlg::OnPaint()
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
	// TODO: �ڴ���ӿؼ�֪ͨ����������
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
	// TODO: �ڴ���ӿؼ�֪ͨ����������
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
