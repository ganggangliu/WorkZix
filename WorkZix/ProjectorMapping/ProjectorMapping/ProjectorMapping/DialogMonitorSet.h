#pragma once
#include "stdafx.h"
#include "afxwin.h"
#include <vector>
#include "afxcmn.h"


// CDialogMonitorSet �Ի���

class CDialogMonitorSet : public CDialogEx
{
	DECLARE_DYNAMIC(CDialogMonitorSet)

public:
	CDialogMonitorSet(CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~CDialogMonitorSet();

// �Ի�������
	enum { IDD = IDD_DIALOG_MONITOR_SET };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
public:
	virtual BOOL OnInitDialog();
	std::vector<DISPLAY_DEVICE> m_DevList;
	std::vector<MONITORINFO> m_MonitorList;
	void UpdateDisplayInfo();
	void FillDisplayInfo();
	CListCtrl m_CList_DispDev;
	CListCtrl m_CList_Monitor;
	int m_nMonitorSel;
	afx_msg void OnNMClickListMonitor(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnBnClickedOk();
	bool GetProjectorInfo(MONITORINFO& monitorinfo);
	CRect m_DispRect;
	int m_nCamId;
	CEdit m_CEdit_Camera_Id;
	afx_msg void OnEnChangeEditCameraId();
};
