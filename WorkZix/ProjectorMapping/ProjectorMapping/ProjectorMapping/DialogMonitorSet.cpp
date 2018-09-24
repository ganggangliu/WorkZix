// DialogMonitorSet.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "ProjectorMapping.h"
#include "DialogMonitorSet.h"
#include "afxdialogex.h"


// CDialogMonitorSet �Ի���

IMPLEMENT_DYNAMIC(CDialogMonitorSet, CDialogEx)

CDialogMonitorSet::CDialogMonitorSet(CWnd* pParent /*=NULL*/)
	: CDialogEx(CDialogMonitorSet::IDD, pParent)
{
	m_nMonitorSel = 0;
	UpdateDisplayInfo();
	if (m_MonitorList.size() <= 1)
	{
		MessageBox(CString("No projector found!"),
			CString("Warm"));
	}
	m_nCamId = 0;
}

CDialogMonitorSet::~CDialogMonitorSet()
{
}

void CDialogMonitorSet::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_LIST_DISP_DEV, m_CList_DispDev);
	DDX_Control(pDX, IDC_LIST_MONITOR, m_CList_Monitor);
	DDX_Control(pDX, IDC_EDIT_CAMERA_ID, m_CEdit_Camera_Id);
}


BEGIN_MESSAGE_MAP(CDialogMonitorSet, CDialogEx)
	ON_NOTIFY(NM_CLICK, IDC_LIST_MONITOR, &CDialogMonitorSet::OnNMClickListMonitor)
	ON_BN_CLICKED(IDOK, &CDialogMonitorSet::OnBnClickedOk)
	ON_EN_CHANGE(IDC_EDIT_CAMERA_ID, &CDialogMonitorSet::OnEnChangeEditCameraId)
END_MESSAGE_MAP()


// CDialogMonitorSet ��Ϣ�������


BOOL CDialogMonitorSet::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// TODO:  �ڴ���Ӷ���ĳ�ʼ��
	LONG lStyle;
	lStyle = GetWindowLong(m_CList_DispDev.m_hWnd, GWL_STYLE);//��ȡ��ǰ����style
	lStyle &= ~LVS_TYPEMASK; //�����ʾ��ʽλ
	lStyle |= LVS_REPORT; //����style
	SetWindowLong(m_CList_DispDev.m_hWnd, GWL_STYLE, lStyle);//����style

	DWORD dwStyle = m_CList_DispDev.GetExtendedStyle();
	dwStyle |= LVS_EX_FULLROWSELECT;//ѡ��ĳ��ʹ���и�����ֻ������report����listctrl��
	dwStyle |= LVS_EX_GRIDLINES;//�����ߣ�ֻ������report����listctrl��
	dwStyle |= LVS_SHOWSELALWAYS;
//	dwStyle |= LVS_EX_CHECKBOXES;//itemǰ����checkbox�ؼ�
	m_CList_DispDev.SetExtendedStyle(dwStyle); //������չ���

	m_CList_DispDev.InsertColumn(0, CString("Id"), LVCFMT_CENTER, 30);
	m_CList_DispDev.InsertColumn(1, CString("String"), LVCFMT_CENTER, 100);
	m_CList_DispDev.InsertColumn(2, CString("State"), LVCFMT_CENTER,50);
	m_CList_DispDev.InsertColumn(3, CString("Primary"), LVCFMT_CENTER, 50);
	m_CList_DispDev.InsertColumn(4, CString("cb"), LVCFMT_CENTER,30);
	m_CList_DispDev.InsertColumn(5, CString("Name"), LVCFMT_CENTER, 100);


	lStyle;
	lStyle = GetWindowLong(m_CList_Monitor.m_hWnd, GWL_STYLE);//��ȡ��ǰ����style
	lStyle &= ~LVS_TYPEMASK; //�����ʾ��ʽλ
	lStyle |= LVS_REPORT; //����style
	SetWindowLong(m_CList_Monitor.m_hWnd, GWL_STYLE, lStyle);//����style

	dwStyle = m_CList_Monitor.GetExtendedStyle();
	dwStyle |= LVS_EX_FULLROWSELECT;//ѡ��ĳ��ʹ���и�����ֻ������report����listctrl��
	dwStyle |= LVS_EX_GRIDLINES;//�����ߣ�ֻ������report����listctrl��
 	dwStyle |= LVS_SHOWSELALWAYS;
//	dwStyle |= LVS_EX_CHECKBOXES;//itemǰ����checkbox�ؼ�
	m_CList_Monitor.SetExtendedStyle(dwStyle); //������չ���

	m_CList_Monitor.InsertColumn(0, CString("Id"), LVCFMT_CENTER, 30);
	m_CList_Monitor.InsertColumn(1, CString("Primary"), LVCFMT_CENTER, 30);
	m_CList_Monitor.InsertColumn(2, CString("Top"), LVCFMT_CENTER, 50);
	m_CList_Monitor.InsertColumn(3, CString("Bottom"), LVCFMT_CENTER,50);
	m_CList_Monitor.InsertColumn(4, CString("Left"), LVCFMT_CENTER, 50);
	m_CList_Monitor.InsertColumn(5, CString("Right"), LVCFMT_CENTER, 50);
	m_CList_Monitor.InsertColumn(6, CString("Monitor"), LVCFMT_CENTER, 100);

	UpdateDisplayInfo();
	FillDisplayInfo();

	m_CList_Monitor.SetItemState(0, LVIS_SELECTED|LVIS_FOCUSED, LVIS_SELECTED|LVIS_FOCUSED);
	m_CList_Monitor.SetSelectionMark(0);

	return TRUE;  // return TRUE unless you set the focus to a control
	// �쳣: OCX ����ҳӦ���� FALSE
}

BOOL CALLBACK MonitorEnumProc(HMONITOR hMonitor,
	HDC hdcMonitor, LPRECT lprcMonitor, LPARAM dwData)
{
	CDialogMonitorSet* pDlg = (CDialogMonitorSet*) dwData;

	MONITORINFO monitorinfo;         
	monitorinfo.cbSize = sizeof(MONITORINFO);

	GetMonitorInfo(hMonitor, &monitorinfo);//MONITORINFOF_PRIMARY
	pDlg->m_MonitorList.push_back(monitorinfo);

	if (monitorinfo.dwFlags != 1)
	{
		pDlg->m_DispRect = monitorinfo.rcWork;
		int nn = 0;
	}

	return TRUE;
}

void CDialogMonitorSet::FillDisplayInfo()
{
	DISPLAY_DEVICE DisplayDevice;
	int nInd = 0;
	for (std::vector<DISPLAY_DEVICE>::reverse_iterator it = m_DevList.rbegin();
		it != m_DevList.rend(); it++)
	{
		DisplayDevice = *it;
		CString szItem;
		szItem.Format(_T("%d"), nInd);
		int nRow = m_CList_DispDev.InsertItem(0, szItem);
		m_CList_DispDev.SetItemText(nRow, 1, DisplayDevice.DeviceString);
		szItem.Format(_T("%d"), DisplayDevice.StateFlags);
		m_CList_DispDev.SetItemText(nRow, 2, szItem);
		if (DisplayDevice.StateFlags & 0x1)
			szItem = "1";
		else
			szItem = "0";
		m_CList_DispDev.SetItemText(nRow, 3, szItem);
		szItem.Format(_T("%d"), DisplayDevice.cb);
		m_CList_DispDev.SetItemText(nRow, 4, szItem);
		m_CList_DispDev.SetItemText(nRow, 5, DisplayDevice.DeviceName);

		nInd++;
	}

	nInd = 0;
	for (std::vector<MONITORINFO>::reverse_iterator it = m_MonitorList.rbegin();
		it != m_MonitorList.rend(); it++)
	{
		MONITORINFO MonitorInfo;
		MonitorInfo = *it;
		CString szItem;
		szItem.Format(_T("%d"), nInd);
		int nRow = m_CList_Monitor.InsertItem(0, szItem);
		if (MonitorInfo.dwFlags & 0x1)
			szItem = "1";
		else
			szItem = "0";
		m_CList_Monitor.SetItemText(nRow, 1, szItem);
		szItem.Format(_T("%d"), MonitorInfo.rcWork.top);
		m_CList_Monitor.SetItemText(nRow, 2, szItem);
		szItem.Format(_T("%d"), MonitorInfo.rcWork.bottom);
		m_CList_Monitor.SetItemText(nRow, 3, szItem);
		szItem.Format(_T("%d"), MonitorInfo.rcWork.left);
		m_CList_Monitor.SetItemText(nRow, 4, szItem);
		szItem.Format(_T("%d"), MonitorInfo.rcWork.right);
		m_CList_Monitor.SetItemText(nRow, 5, szItem);
		szItem.Format(_T("%d,%d,%d,%d"), MonitorInfo.rcWork.top,
			MonitorInfo.rcWork.bottom, MonitorInfo.rcWork.left ,MonitorInfo.rcWork.right);
		m_CList_Monitor.SetItemText(nRow, 6, szItem);

		nInd++;
	}

	CString szCameraId;
	szCameraId.Format(_T("%d"), m_nCamId);
	m_CEdit_Camera_Id.SetWindowText(szCameraId);
}

void CDialogMonitorSet::UpdateDisplayInfo()
{
	m_DevList.clear();
	DISPLAY_DEVICE DisplayDevice;
	DWORD i=0;
	ZeroMemory(&DisplayDevice, sizeof(DisplayDevice));
	DisplayDevice.cb = sizeof(DisplayDevice);

	for(i=0; EnumDisplayDevices(NULL, i, &DisplayDevice, 0); i++)
	{
		m_DevList.push_back(DisplayDevice);/*DISPLAY_DEVICE_ATTACHED_TO_DESKTOP*/
	}

	m_MonitorList.clear();
	BOOL NN = EnumDisplayMonitors(0, 0, MonitorEnumProc, (LPARAM)this);

	int nnn = 0;
}


void CDialogMonitorSet::OnNMClickListMonitor(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMITEMACTIVATE pNMItemActivate = reinterpret_cast<LPNMITEMACTIVATE>(pNMHDR);
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	if (pNMItemActivate->iItem >= 0)
		m_nMonitorSel = pNMItemActivate->iItem;
	m_CList_Monitor.SetItemState(m_nMonitorSel, LVIS_SELECTED|LVIS_FOCUSED, LVIS_SELECTED|LVIS_FOCUSED);
	m_CList_Monitor.SetSelectionMark(m_nMonitorSel);

	*pResult = 0;
}


void CDialogMonitorSet::OnBnClickedOk()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������

	CDialogEx::OnOK();
}

bool CDialogMonitorSet::GetProjectorInfo(MONITORINFO& monitorinfo)
{
	monitorinfo.cbSize = sizeof(MONITORINFO);
	for (unsigned int i = 0; i < m_MonitorList.size(); i++)
	{
		if (m_MonitorList[i].dwFlags != 1)
		{
			monitorinfo = m_MonitorList[i];
			return true;
		}
	}

	return false;
}

void CDialogMonitorSet::OnEnChangeEditCameraId()
{
	// TODO:  ����ÿؼ��� RICHEDIT �ؼ���������
	// ���ʹ�֪ͨ��������д CDialogEx::OnInitDialog()
	// ���������� CRichEditCtrl().SetEventMask()��
	// ͬʱ�� ENM_CHANGE ��־�������㵽�����С�

	// TODO:  �ڴ���ӿؼ�֪ͨ����������
	CString szId;
	m_CEdit_Camera_Id.GetWindowText(szId);
	m_nCamId = _ttoi(szId);
// 	szId.Format(_T("%d"), nId);
// 	m_CEdit_Camera_Id.SetWindowText(szId);

	return;
}
