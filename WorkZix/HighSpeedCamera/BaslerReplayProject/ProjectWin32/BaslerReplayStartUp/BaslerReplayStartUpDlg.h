
// BaslerReplayStartUpDlg.h : 头文件
//

#pragma once
#include "afxwin.h"


// CBaslerReplayStartUpDlg 对话框
class CBaslerReplayStartUpDlg : public CDialogEx
{
// 构造
public:
	CBaslerReplayStartUpDlg(CWnd* pParent = NULL);	// 标准构造函数

// 对话框数据
	enum { IDD = IDD_BASLERREPLAYSTARTUP_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持

// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButtonSelect();
	CEdit m_CEditPathInput;
	afx_msg void OnBnClickedOk();
	TCHAR m_ModulePath[MAX_PATH];
	TCHAR m_HistoryFilePath[MAX_PATH];
	TCHAR m_ExePath[MAX_PATH];
};
